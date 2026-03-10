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

## 5. Plotly live markers — two-tier update strategy

### Symptom A — markers never move after page load
The `power_plot.figure = fig; power_plot.update()` pattern stays stuck at
(0, 0) after page load regardless of the actual power level.

**Fix**: Use `run_method('react', data, layout)` which calls `Plotly.react()`
directly on the DOM element, bypassing NiceGUI's element-diff path.

### Symptom B — markers only update on manual interactions, not in real-time
`run_method('react', ...)` called every second from `_tick_inner` rebuilds the
**entire figure** (model curve + CI band polygon + markers) every tick.  This
is a large WebSocket payload (~5–10 KB/s) that the browser's Plotly renderer
may batch or drop, making the dots appear frozen during sequences.

### Fix (already in place — do not revert)
The marker update is split into two functions:

```python
def _update_power_curve():
    """Full react — call only when model/LPM/CI band changes."""
    ...
    power_plot.run_method('react', fd['data'], fd['layout'])

def _update_power_markers():
    """Lightweight restyle — safe to call every tick."""
    tgt_o3 = predict_o3_from_power(S.power_target_pct, S.flow_lpm)
    power_plot.run_method(
        'restyle',
        {'x': [[S.power_target_pct], [S.power_actual_pct]],
         'y': [[tgt_o3], [S.vessel_o3_pct]]},
        [2, 3],
    )
```

`_tick_inner` calls `_update_power_markers()` (tiny restyle, <100 bytes).
`_on_lpm_change`, model fitting, and preset clicks call `_update_power_curve()`
(full react, rare).

### Critical constraint: trace indices must stay fixed
`restyle` addresses traces by index.  `_make_power_fig` **always** adds the CI
band trace at index 1 (with empty `x`/`y` lists when no CI data is available),
so the target marker is always index 2 and the actual marker is always index 3.
Never reorder the traces or add traces before index 2.
