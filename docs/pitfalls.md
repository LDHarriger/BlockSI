# BlockSI Dashboard — Recurring Pitfalls & Framework Gotchas

> This document records NiceGUI and asyncio patterns that have caused bugs
> more than once, so that future agents (and humans) avoid repeating them.

---

## 1. `ui.notify()` (and all NiceGUI UI calls) crash from background tasks

### Symptom
A background coroutine launched with `asyncio.create_task()` runs silently for
a while, then the sequence halts with a traceback like:

```
RuntimeError: The current slot cannot be determined. Make sure you are in a
NiceGUI context when you call this function.
```

The crash happens at the first `ui.*` call inside the background task — the
code before it ran fine.

### Root cause
NiceGUI binds every UI call to the **slot** (client/tab context) that was
active when the coroutine started.  `asyncio.create_task()` **detaches** the
new coroutine from the current slot.  Any subsequent `ui.notify()`,
`ui.update()`, `element.set_text()`, etc. inside that task has no slot context
and raises `RuntimeError`.

This is **not** a logic bug in the sequence code — purely a NiceGUI threading
constraint.

### Fix (already in place — do not remove)
`blocksi_dashboard.py` uses a **notification queue** pattern:

```python
# Module-level queue
_notify_queue: deque[tuple[str, str]] = deque()

def _notify(msg: str, level: str = "positive") -> None:
    ...
    try:
        ui.notify(msg, type=level, ...)
    except RuntimeError:
        _notify_queue.append((msg, level))  # defer to tick

# In _tick_inner() — runs every 1 s with correct slot context:
while _notify_queue:
    msg, lvl = _notify_queue.popleft()
    _notify(msg, lvl)
```

### General rule for new background tasks
- **Never call any `ui.*` function directly** inside a coroutine launched with
  `asyncio.create_task()`.
- Status/progress updates from background tasks must go through:
  - `_notify_queue` (notifications), or
  - Mutations to `SystemState` fields that are read by `_tick_inner()` (labels,
    progress bars, etc.) — `_tick_inner()` always has the correct slot context.
- If you need the background task to trigger a UI dialog or modal, set a flag on
  `SystemState` and let `_tick_inner()` open it.

---

## 2. NiceGUI element `on_change` fires twice — use the `_updating` guard

### Symptom
User moves the power slider; `cmd_set_power(pct)` is called correctly, but the
ESP32 immediately receives a second `CMD,set_power` command reverting power to
the previous value (or to 0), making every manual power adjustment visually
"snap back".

### Root cause
When a slider or number input fires `on_change`, the handler typically copies
the new value into a second linked element (e.g. the number box next to the
slider, or vice versa).  Setting `element.value = x` **in Python re-fires
`on_change`** on that element.  Without a guard, each handler triggers the
other, causing a cascade or a double send of the old value.

### Fix (already in place — do not remove)
All paired UI elements share a closure-local `_updating` flag:

```python
_updating = False

async def _on_power_slide(e) -> None:
    nonlocal _updating
    if _updating or S.sequence_active:
        return
    _updating = True
    pct = int(e.value)
    await cmd_set_power(pct)
    if inp_pwr is not None:
        inp_pwr.value = pct   # would re-fire _on_power_input without guard
    _updating = False

async def _on_power_input(e) -> None:
    nonlocal _updating
    if _updating or S.sequence_active:
        return
    _updating = True
    pct = int(e.value)
    await cmd_set_power(pct)
    if slider is not None:
        slider.value = pct    # would re-fire _on_power_slide without guard
    _updating = False
```

### General rule for new paired inputs
Any time two or more UI elements are linked so that changing one updates the
other, **all** their `on_change` handlers must share and check `_updating`.
Failing to do so causes double-sends, oscillation, or phantom revert behavior.

---

## 3. `_sequence_cleanup()` must be unconditional — do not add `seq_confirmed` guards

### Symptom
Sequence exits (via COMPLETE or user abort) but the generator keeps running at
full power with relays energised.  Power is stuck at, e.g., 86% and never
resets to 0.

### Root cause
An earlier version guarded cleanup with:
```python
if not S.seq_confirmed and source != 'stop':
    return   # ← dangerous: cleanup silently skipped
```
Any code path that set `seq_confirmed = False` before cleanup ran would leave
the system in a live state.

### Fix (already in place — do not remove)
`_sequence_cleanup()` always calls `_safe_standby()` unconditionally:

```python
async def _safe_standby() -> None:
    """Unconditionally: power=0, all relays off. No guards."""
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)
    await cmd_set_relay("o2_conc", False)
    await cmd_set_relay("air_comp", False)

async def _sequence_cleanup(source: str) -> None:
    S.seq_confirmed = False
    await _safe_standby()
    ...
```

### General rule
Never gate `_safe_standby()` or `_sequence_cleanup()` on sequence state flags.
The cleanup must be a **fallback that always runs** regardless of how the
sequence ended.

---

## 4. `asyncio.Event` inside a pre-flight dialog — do not `await` from background

### Symptom
A validation pre-flight dialog (e.g. "residual O3 detected — proceed anyway?")
works fine when called from a button click handler, but hangs forever or crashes
when called from inside an `asyncio.create_task()` coroutine.

### Root cause
`asyncio.Event.wait()` suspends the coroutine.  In a background task (no slot
context), the NiceGUI event loop cannot deliver the button-click event back to
the waiting coroutine — it deadlocks.

### Fix
Pre-flight checks that require user interaction must run in the **button click
handler** (which has a slot context), not inside the background task.  The
click handler performs the check, shows the dialog if needed, and only calls
`asyncio.create_task(background_coro())` after the user confirms.

Example (in `blocksi_dashboard.py`):
```python
async def _start_fill_seq():         # ← button click, has slot context
    lpm = float(fill_lpm_input.value)
    cert = _find_valid_cert(100, lpm)
    if cert is None:
        with ui.dialog() as dlg, ui.card():
            ...
            async def _proceed():
                dlg.close()
                await cmd_sequence_start("cstr_cal", flow=lpm)
            ui.button("Run Validation", on_click=_proceed)
        dlg.open()
        return                       # ← do NOT proceed further
    await cmd_sequence_start("cstr_cal", flow=lpm)
```

---

## 5. Power-O3 chart live markers — two-tier update strategy

### Symptom
The live markers (green dot = actual O3, black ring = power target) on the
Power-O3 curve never move — stuck at (0, 0) after page load.

### Root cause & history
- `power_plot.figure = fig; power_plot.update()` DOES work for full figure
  redraws (NiceGUI routes this through `Plotly.react()` internally) — but
  calling it on every 1s tick adds unnecessary full-chart serialization work.
- `power_plot.run_method('react', fd['data'], fd['layout'])` was tried and
  **FAILS** — raises `"Method 'react' not found"`. NiceGUI's Vue component does
  not expose a `react` method directly. Do not attempt this.
- `power_plot.run_method('restyle', ...)` is similarly **unverified** — NiceGUI's
  plotly Vue component does not document a 'restyle' method. Do not attempt this.
  Use `ui.run_javascript()` to call `Plotly.restyle()` directly on the DOM element.

### Fix (already in place — two-tier approach)
**Full redraws** (LPM change, model reload, model fit): use `_update_power_curve()`
which calls `power_plot.figure = fig; power_plot.update()`.

**Per-tick marker updates** (every 1s): use `_restyle_markers()` which calls:
```python
ui.run_javascript(f'''
  const el = getElement("{power_plot.id}");
  if (el && el.$el) {{
    Plotly.restyle(el.$el,
      {{x: [[{target_pwr}], [{actual_pwr}]], y: [[{target_o3}], [{actual_o3}]]}},
      [2, 3]);
  }}
''')
```
`Plotly.restyle()` is O(1) — it patches specific trace data without
re-rendering the full chart. Fixed trace indices (0=curve, 1=CI band,
2=target ring, 3=actual dot) make this safe.

> **⚠ CRITICAL — DOM access for `ui.plotly`**: `getElement(id)` returns a NiceGUI
> custom Vue component (NOT a Quasar component). Use `el.$el` — the root DOM
> element (the Plotly div). **Do NOT use `el.$refs.qRef`** — it is `undefined`
> on `ui.plotly`. The guard `if(el && el.$refs.qRef)` silently skips every
> restyle call. This rule applies to all NiceGUI `ui.*` components; only
> Quasar-wrapped components expose `$refs.qRef`.

### `ui.run_javascript()` also crashes on dead client — must try/except
`_restyle_markers()` calls `ui.run_javascript()`. When the browser tab disconnects
(closes, refreshes, or the connection is lost), the NiceGUI slot is cleaned up and
`context.client` raises `RuntimeError: The parent element this slot belongs to has
been deleted.` — identical in class to the `ui.notify()` crash from Pitfall #1.

If this exception is not caught, it propagates through `_tick_inner()` → `_tick()`,
and then **NiceGUI's own exception handler also crashes** (it tries to access the
same dead client to log the error). This kills the timer task permanently, stopping
all tick-based updates including the `_notify_queue` drain, sequence cleanup, and
relay sync — which is why the CSTR sequence appeared to halt.

**Fix (already in place)**:
```python
try:
    ui.run_javascript(js)
except RuntimeError:
    pass  # client disconnected — skip silently
```
**General rule**: every call to `ui.run_javascript()` or any `ui.*` function inside
`_tick_inner()` (or any timer callback) must be wrapped in `try/except RuntimeError`.

### CI band visibility
If the CI band appears invisible despite the model having `ci_sigma` data: the
fill may be physically too narrow (e.g. ~2px on a 300px chart for a well-fitted
model with `ci_sigma ≈ 0.007` on a 2.1 %vol y-range). Fix: add a visible border
line so the ±1σ boundary is drawn regardless of fill width:
```python
line=dict(color="rgba(65,105,225,0.5)", width=1)  # was: line=dict(width=0)
```
Do not inflate σ or switch to ±2σ/3σ — a well-fitted model (R²≈0.998) *should*
have narrow CI. The boundary just needs to be visible as a line.

---

## 7. Unused Functions Indicate Missing Capabilities (motor_pot brake-mode)

| Detail | Value |
|--------|-------|
| First hit | 2026-03-26 |
| Symptom | Motorized potentiometer drifts 5–12% below target after positioning; validator catches and corrects but cannot hold — produces sawtooth cycling (134 mismatch events in one session) |
| Root cause | `motor_pot_stop()` used coast mode (`MOTOR_DIR_STOP` → both GPIOs LOW → DRV8833 Hi-Z), allowing wiper drift. `motor_pot_brake()` and `MOTOR_DIR_BRAKE` already existed in the codebase but were never called |
| Fix | One-line change: `motor_pot_stop()` now uses `MOTOR_DIR_BRAKE` instead of `MOTOR_DIR_STOP`. Brake mode shorts motor windings through low-side FETs, providing electromagnetic holding torque |
| Case file | `docs/cases/power_mismatch_audit.md` |

### The Lesson: Unused Functions as Red Flags

When auditing code, **unused functions that implement a related capability are a red flag**.
The existence of `motor_pot_brake()` meant the original developer intended braking to be
available — but it was never wired into the stop path. Multiple code audits missed this
because they focused on what the code *does*, not what it *could do but doesn't*.

**Audit rule**: If a module defines a function that is never called, ask:
1. Was this function meant to be used somewhere?
2. Does the absence of this call explain a known issue?
3. Should it be integrated, or is it dead code to remove?

### Debugging Methodology That Worked

This issue was resolved through a structured hypothesis-driven process:

1. **Instrument first** — Added DIAG messages with settle times, ADC values, and
   directional data to the firmware. This provided the evidence needed to distinguish
   between hypotheses.

2. **Form specific testable hypotheses** — Three hypotheses were defined:
   - H1: Coast-mode wiper drift (mechanical/electrical)
   - H2: ADC noise / measurement error
   - H3: Timing race in the control loop
   Each made different predictions about drift direction, timing, and magnitude.

3. **Collect data that discriminates** — The DIAG log showed 96% downward drift,
   settle times of 3–66 seconds, and a clear sawtooth pattern. This ruled out H2
   (noise would be bidirectional) and H3 (timing races would be instantaneous).

4. **Targeted fix** — Once H1 was confirmed, the fix was a single line change to an
   already-implemented capability. No shotgun debugging, no speculative changes.

**Anti-pattern**: The issue initially "self-resolved" in one session (transient
improvement), tempting the conclusion that it was environmental. It recurred the same
evening. Never close an issue as resolved without understanding the mechanism.
