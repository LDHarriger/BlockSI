# Dashboard Modularization Plan

> Status: APPROVED — implementation in progress
> Target: `Interfaces/PC/blocksi_dashboard.py` (3800 lines → ~6 focused modules)
> Date: 2026-03-15

---

## Structural Audit Summary

The dashboard has 6 distinct functional domains currently mixed into one file:

| Domain | Lines | Line Range | State Access | UI Coupling |
|--------|-------|------------|-------------|-------------|
| **Config & constants** | ~100 | 1–100 | None | None |
| **SystemState + helpers** | ~230 | 100–390 | Owns `S` | None |
| **Data I/O** (CSV logger, cal files, validation CSV) | ~165 | 436–600 | Reads `S` | None |
| **Validation analysis** | ~130 | 600–730 | Reads `S`, samples | None |
| **TCP Server** (connect, dispatch, SEQ/RSP/STATE handlers) | ~510 | 730–1242 | Mutates `S` | Calls `log()`, `_notify()` |
| **Command helpers** (power, relay, sequence start/abort) | ~285 | 1242–1528 | Mutates `S`, uses `tcp` | Calls `_notify()` |
| **CSTR sequence** (fill/evac coroutine, all constants) | ~500 | 1528–2197 | Heavy `S` mutation, uses `tcp` | Calls `_notify()`, `log()` |
| **UI builder** (`index()` function) | ~1580 | 2197–3780 | Reads/binds `S`, captures UI elements | 100% NiceGUI |
| **Startup** | ~20 | 3780–3800 | Creates `tcp`, loads models | None |

### Key observations

1. **The UI builder (`index()`) is 1580 lines** — by far the largest section and the hardest to work in. It contains nested closures (`_tick_inner`, `_on_power_slide`, chart updaters) that capture local UI element references via closure scope.

2. **Domains 1–7 have zero UI coupling** — they never call `ui.*` to create elements. They only call `log()` and `_notify()` for logging/notifications. These are trivially extractable.

3. **The CSTR sequence (500 lines) is the biggest non-UI block** — it's a self-contained async coroutine with its own constants, CSV writer, debug logger, and state machine. Already well-isolated logically.

4. **Global singletons**: `S` (SystemState), `tcp` (TCPServer), `csv_logger`, `data_buf`, `_notify_queue`, `_log_entries`. All module-level. Any extracted module needs access to these.

5. **The `_notify()` function** is the main coupling point between non-UI code and the UI — it queues notifications that `_tick_inner()` drains. This is already a clean interface.

---

## NiceGUI-Specific Constraints

From researching NiceGUI's architecture:

1. **UI elements must be created inside `@ui.page` or event handlers** — never at module import time. Builder functions called from `index()` are fine.

2. **NiceGUI's context is stack-based** — `ui.label()` inside a `with ui.card():` block works identically whether the call is in `main.py` or an imported module. The Python call stack determines parentage.

3. **Closures capturing UI elements are per-client** — each browser session gets its own `index()` call. Element references (`slider`, `inp_pwr`, etc.) are local to that call. Extracting these into class instances or builder functions that return references is safe.

4. **`ui.run_javascript()` and `ui.notify()` require client context** — but the existing `_notify_queue` pattern already handles this correctly. No change needed.

5. **Circular imports are the main structural risk** — if module A imports state from module B and vice versa. Solution: extract shared state (`S`, `tcp`, constants) into a dedicated module that everything else imports.

---

## Proposed Module Structure

```
Interfaces/PC/
    blocksi_dashboard.py          # Slim entry point: imports, ui.run()
    dashboard/
        __init__.py               # Package init (can be empty or re-export)
        state.py                  # SystemState class, S singleton, constants, config
        tcp_server.py             # TCPServer class, dispatch, SEQ/RSP/STATE handlers
        commands.py               # cmd_set_power, cmd_set_relay, cmd_sequence_start, etc.
        cstr_sequence.py          # _start_fill_evac, CSTR constants, CSV writer, debug logger
        data_io.py                # _CSVLogger, cal file helpers, validation CSV, _save_val_csv
        validation.py             # _analyze_validation (already pure logic)
        ui_main.py                # index() — the @ui.page builder (still large but isolated)
    analysis/                     # Already extracted (no changes needed)
        __init__.py
        power_o3_model.py
        fill_model.py
```

### Why this split

| Module | Lines | Rationale |
|--------|-------|-----------|
| `state.py` | ~250 | Single source of truth for `S`, constants, prediction helpers. Everything imports from here. Eliminates circular dependency risk. |
| `tcp_server.py` | ~510 | Self-contained networking. Depends on `state.S` and `log()`. No UI coupling. |
| `commands.py` | ~285 | All `cmd_*` functions. Depends on `state.S`, `tcp`, `_notify()`. No UI coupling. |
| `cstr_sequence.py` | ~500 | Largest non-UI block. Self-contained coroutine with own constants and CSV logic. Depends on `state.S`, `commands.*`, `_notify()`. |
| `data_io.py` | ~165 | CSV logging, calibration file listing, validation file I/O. Pure I/O, no UI. |
| `validation.py` | ~130 | `_analyze_validation()` — pure statistical computation on sample lists. Zero dependencies on UI or TCP. |
| `ui_main.py` | ~1580 | The `index()` page builder. Remains large but is now the ONLY file with `ui.*` calls. Can be further split later (see Phase 2). |
| `blocksi_dashboard.py` | ~30 | Slim entry: `from dashboard.state import S`, `from dashboard.ui_main import index`, `app.on_startup(...)`, `ui.run(...)` |

---

## Dependency Graph

```
state.py  ←── everything imports from here
   ↑
   ├── tcp_server.py    (imports state, log, _notify)
   ├── data_io.py       (imports state, log)
   ├── validation.py    (imports state for predict helpers)
   ├── commands.py      (imports state, tcp_server, _notify)
   ├── cstr_sequence.py (imports state, commands, data_io, _notify, log)
   └── ui_main.py       (imports everything above)
```

**No circular dependencies.** `state.py` is a leaf — it imports only from `analysis/` (external). Everything else imports downward toward `state.py`.

---

## The `log()` / `_notify()` Problem

These two functions are called by almost every module but are currently defined in `blocksi_dashboard.py` alongside UI state (`_log_entries`, `_notify_queue`). Solution:

**Move `log()` and `_notify()` into `state.py`** alongside their backing data structures (`_log_entries: deque`, `_notify_queue: deque`). They don't create UI elements — they just append to queues that `_tick_inner()` drains. This is clean.

---

## Implementation Strategy — Phase 1 (Safe Extraction)

Phase 1 extracts only non-UI code. The UI builder stays intact. This is the safest approach — no UI behavior changes, no NiceGUI context issues.

### Step-by-step (each step is one commit, tested before proceeding)

1. **Create `dashboard/` package** with `__init__.py`

2. **Extract `state.py`**: Move `SystemState`, `S`, all constants (lines 63–320), prediction helpers (lines 151–188), `log()`, `_notify()`, `_notify_queue`, `_log_entries`, `compute_effective_o2_pct()`. Update imports in `blocksi_dashboard.py`.

3. **Extract `data_io.py`**: Move `_CSVLogger`, `csv_logger`, `list_calibration_files()`, `_save_cal_csv()`, `_save_val_csv()`, `_find_valid_cert()`. Imports `state` for constants and `log()`.

4. **Extract `validation.py`**: Move `_analyze_validation()`. Imports `state` for prediction helpers.

5. **Extract `tcp_server.py`**: Move entire `TCPServer` class. Imports `state` for `S`, `log()`, `_notify()`, `parse_data_line()`, `apply_telemetry()`, `csv_logger`.

6. **Extract `commands.py`**: Move all `cmd_*` functions and `_start_calibration()`, `_start_validation()`, `_safe_standby()`, `_sequence_cleanup()`. Imports `state`, `tcp_server`.

7. **Extract `cstr_sequence.py`**: Move CSTR constants, `_start_fill_evac()`, `_fit_and_save_cstr_model()`, `_write_cstr_csv()`, `_make_cstr_csv_path()`, `_cstr_flog()`, etc. Imports `state`, `commands`, `data_io`.

8. **Slim down `blocksi_dashboard.py`**: Now contains only `index()`, `_startup()`, and `ui.run()`. Rename or keep as entry point.

### Testing after each step

After each extraction:
- Run the dashboard: `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py`
- Verify: UI loads at localhost:8080, sidebar shows sensor readings, power slider works
- If connected to ESP32: verify TCP connection, telemetry streaming, command responses
- No automated tests exist — manual verification required

---

## Phase 2 (Future — UI Split)

After Phase 1, `ui_main.py` will still be ~1580 lines. It can be further split using **NiceGUI builder functions**:

```python
# dashboard/ui/sidebar.py
def build_sidebar(state, commands):
    with ui.left_drawer().classes("q-pa-md").props("width=240"):
        # ... sidebar content

# dashboard/ui/power_tab.py
def build_power_tab(state, commands, chart_refs):
    with ui.tab_panel(tab_power):
        # ... power tab content
```

Each builder function:
- Takes `state` + `commands` as parameters (dependency injection)
- Returns references to UI elements that `_tick_inner()` needs to update
- Is called from `index()` during page construction

This is a larger refactor because it requires changing how `_tick_inner()` accesses element references (currently via closure, would need to be passed as a dict or dataclass). Defer to a separate session.

---

## What NOT to Change

- **Do not use `ui.element` subclassing** — overkill for internal modularization
- **Do not use `app.storage`** — the existing `S` singleton is simpler and faster for a single-user localhost dashboard
- **Do not create elements at module import time** — all UI construction stays inside `index()`
- **Do not change the tick architecture** — `_tick_inner()` draining `_notify_queue` is the correct pattern
- **Do not touch `analysis/`** — already well-modularized

---

## Risk Assessment

| Risk | Mitigation |
|------|-----------|
| Circular imports | `state.py` is a leaf node — everything imports from it, it imports from nobody (except `analysis/`) |
| Broken imports after move | Each step is one module, tested immediately. `from dashboard.state import S` pattern. |
| NiceGUI context issues | Phase 1 only moves non-UI code. No `ui.*` calls are moved. |
| `tcp` global reference | `tcp` singleton created in `_startup()`, stored in `state.py`. All modules import it from there. |
| Performance regression | No new abstractions, no new layers, no serialization. Same function calls, different file locations. |
| Merge conflicts with in-progress work | Use file locking protocol. Do this when no other agent is editing the dashboard. |

---

## Verification Criteria

Each extraction step must pass **all** of the following checks before committing. These are standard GUI refactoring safety criteria.

### 1. Import Integrity
- [ ] `python -c "from dashboard import state"` (and each new module) succeeds with no `ImportError`
- [ ] No circular imports — verified by importing every module in isolation
- [ ] All symbols previously accessible in `blocksi_dashboard.py` are still reachable (via re-export or direct import)

### 2. Behavioral Equivalence
- [ ] Dashboard starts without errors: `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py`
- [ ] UI loads at `localhost:8080` — all 4 tabs render, sidebar populates
- [ ] No Python tracebacks in console during startup or idle tick
- [ ] `_tick_inner()` runs without error (visible as live "last update" timestamp in sidebar)

### 3. State & Singleton Integrity
- [ ] `S` (SystemState) is the **same object** across all modules — verified by `id(S)` or by setting a field in one module and reading it from another
- [ ] `tcp`, `csv_logger`, `data_buf`, `_notify_queue`, `_log_entries` are shared singletons, not copies
- [ ] Module-level initialization order does not cause `None` references (lazy init or deferred assignment where needed)

### 4. Event & Async Safety
- [ ] `_notify()` calls from extracted modules (tcp_server, commands, cstr_sequence) still enqueue to the same `_notify_queue` that `_tick_inner()` drains
- [ ] `log()` calls from extracted modules append to the same `_log_entries` deque
- [ ] Async coroutines (`_start_fill_evac`, command helpers) still resolve in the same event loop — no accidental new-loop creation

### 5. No Performance Regression
- [ ] No new abstraction layers, wrapper classes, or indirection added
- [ ] No serialization/deserialization between modules (everything is in-process Python references)
- [ ] No new `import` at call-time (all imports at module top-level)
- [ ] Tick interval unchanged — UI responsiveness identical

### 6. File & Path Safety
- [ ] All `os.path` references (`DATA_DIR`, `MODEL_DIR`, etc.) resolve to the same absolute paths as before
- [ ] CSV writers, model loaders, and calibration file listers still find/create files in correct directories
- [ ] `__file__`-relative paths (if any) are updated to account for new file locations

### 7. No UI Regressions (Phase 1 specific)
- [ ] Zero `ui.*` calls are moved out of the UI builder — Phase 1 only moves non-UI code
- [ ] NiceGUI element context stack is unaffected (no elements created at import time)
- [ ] Closures in `index()` still capture the correct local references
