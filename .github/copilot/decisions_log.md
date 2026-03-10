# BlockSI Architectural Decisions Log

> Decisions are **immutable** once logged.  To reverse or modify a decision,
> add a new entry that supersedes the original.

---

### 2026-03-09: Backfill-active stuck — reset on connect/disconnect + safety timeout

**Context**: `S.backfill_active` could get stuck `True` permanently, causing `_dispatch()` to skip `apply_telemetry()` for ALL subsequent DATA messages. Two scenarios: (1) TCP drops mid-backfill — `BACKFILL_END` never arrives; (2) ESP32 reconnects with empty data cache — skips `BACKFILL_START`/`BACKFILL_END` entirely, but the flag from the previous connection persists. Symptoms: sidebar cards frozen at zero, green dot stuck at origin, while ECharts continued working (they read from `data_buf`, not from `S` state).

**Decision**:
1. Reset `S.backfill_active = False` (+ `expected`/`received`) in `_on_connect()` immediately after `S.connected = True` — ensures a clean slate for every new connection.
2. Reset same fields in the disconnect `finally` block — clears stale state from interrupted connections.
3. Added `S.backfill_start_time: float` field, set in `_handle_backfill_start()`. `_tick_inner()` checks: if `backfill_active` and elapsed > 30s, force-clears the flag with a warning log — safety net for any future edge cases.

**Rationale**: The backfill state is per-connection and must not survive across connections. The 30s timeout provides defense-in-depth — backfill typically completes in seconds (limited by `DATA_CACHE_SIZE` of ~100 samples × ~2.5s transmission rate).

**Status**: `[IMPLEMENTED]`

---

### 2026-03-09: Power-O3 chart — static curve with dynamic marker restyle

**Context**: `_update_power_curve()` rebuilt the entire Plotly figure (curve, CI band, ring, dot) on every 1s tick, even though only the markers (ring = target, dot = actual) change with incoming telemetry. The model curve and CI band only change on LPM adjustment or model re-fit. Additionally, the CI band was conditionally added (only when model has CI data), making trace indices variable — trace 2 could be the ring or the CI band depending on model state.

**Decision**:
1. `_make_power_fig()` now ALWAYS creates exactly 4 traces in fixed order: [0] model curve, [1] CI band (invisible placeholder `Scatter(x=[], y=[], visible=False)` if no CI data), [2] target ring, [3] actual dot.
2. New `_restyle_markers()` function uses `ui.run_javascript()` → `Plotly.restyle(gd, {x:[[val]], y:[[val]]}, [traceIdx])` to update only traces 2 and 3. Accesses DOM via `getElement(power_plot.id).$refs.qRef`.
3. `_tick_inner()` calls `_restyle_markers()` instead of `_update_power_curve()`. Full rebuild retained for LPM changes, model re-fit, and model load.

**Rationale**: `Plotly.restyle()` is O(1) in trace count — it patches specific trace data without re-rendering the entire chart. The fixed trace indices eliminate the conditional-CI-band index ambiguity. This reduces per-tick work from full figure serialization + websocket push to a single `run_javascript()` call with two restyle operations.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-09: Plotly `react` for live power curve updates

**Context**: The green dot (actual O3 vs actual power) and black ring (target power prediction) on the Power-O3 curve never moved from (0, 0) after page load. `_update_power_curve()` rebuilt a full `go.Figure` and pushed it via `power_plot.figure = fig; power_plot.update()`. NiceGUI's element-diff mechanism compares the Python-side figure dict and sends only diffs — but figure replacement does not reliably push trace-data changes through this path.

**Decision**: Replace `power_plot.figure = fig; power_plot.update()` with:
```python
fd = fig.to_dict()
power_plot.run_method('react', fd['data'], fd['layout'])
```
`run_method('react', ...)` calls Plotly.js's `Plotly.react()` directly on the DOM element, bypassing NiceGUI's diff and guaranteeing a full re-render on every call.

**Rationale**: `Plotly.react()` is designed for efficient full-data updates and is the documented way to update Plotly charts from JavaScript. The Python figure dict is serialized directly — no element diff, no stale state. This also makes sequence live-tracking (green dot moving during validation) work correctly at no additional cost.

**Status**: `[SUPERSEDED by 2026-03-09 "Revert Plotly react" below]`

---

### 2026-03-09: Revert Plotly `react` — use `figure`/`update()` API

**Context**: The previous decision introduced `power_plot.run_method('react', fd['data'], fd['layout'])` which caused `"Method 'react' not found"` errors. NiceGUI's `ui.plotly` Vue component does NOT expose a `react` method — instead, its internal JS `update()` method calls `Plotly.react(this.$el, this.options, ...)`. The `run_method()` API invokes Vue component methods, not arbitrary Plotly.js functions.

**Decision**: Revert to `power_plot.figure = fig; power_plot.update()`. NiceGUI's `Element.update()` enqueues in the client outbox → websocket → client-side JS `update()` → `Plotly.react()` — so the full re-render still happens, just through the proper channel.

**Rationale**: Confirmed from NiceGUI 3.8.0 source: `plotly.py` defines `update()` which sets `_props['options']` then calls `super().update()`. The client-side `plotly.js` defines `update()` which calls `Plotly.react(this.$el, this.options, options.config)`. The figure assignment + `update()` is the documented API.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-09: Motor-to-0 root cause — PC sole authority for power target

**Context**: After manually setting power via the dashboard slider, the motor potentiometer would return to 0%. Three separate root causes were identified across multiple debugging sessions:

1. **Root cause 1 (fixed commit ec0d884)**: `seq_executor.c` called `o3_power_set()` directly instead of `blocksi_state_set_power()`, so the state validator's `target_pct` was stale and the validator "corrected" the motor back to the old target.

2. **Root cause 2 (this fix)**: `_handle_state()` overwrote `S.power_target_pct` from the ESP32's STATE push. The STATE `power` field contains `o3_power_get()` — the ADC-read motor position, NOT the commanded target. On TCP connect/reconnect, the motor reads 0% → STATE sends `power=0` → `S.power_target_pct = 0` → tick syncs slider to 0 → callback sends `cmd_set_power(0)`.

3. **Root cause 3 (this fix)**: NiceGUI's `handle_event()` schedules async handlers via `background_tasks.create()` (asyncio.create_task), so they run in a FUTURE event loop iteration. The `_updating` boolean guard is set True before `slider.value = X` and cleared immediately after — but the async `_on_power_slide()` callback runs AFTER the guard is cleared. The guard cannot protect against this timing.

**Decision**:
1. `_handle_state()`: Store STATE `power` as `S.power_actual_pct` (informational) instead of `S.power_target_pct`. Remove `S.update_derived()` call (derived values depend on target, not actual).
2. `_on_power_slide()` and `_on_power_input()`: Add redundancy check `if pct == S.power_target_pct: return` after extracting the value. This prevents sending a `cmd_set_power(X)` when X already equals the target — catching any tick-echo scenario regardless of guard timing.
3. Document the PC-sole-authority principle with a comment block above `_handle_state()`.

**Rationale**: The PC is sole authority for `power_target_pct` (documented at dashboard line 11 and line 383). The ESP32's STATE message is a "current status report", not a "command echo". Any code path that modifies `power_target_pct` without the user's intent, combined with the tick's slider sync, creates a `cmd_set_power(wrong_value)` vector.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-09: Validation baseline is advisory only — pre-flight warning dialog

**Context**: The 106-H sensor reports exactly 0.0200 %vol as its floor when fully zeroed. The original baseline gate (`baseline_ok = bl_mean < 0.02`) failed any run where the sensor read its own noise floor. Changing `<` to `<=` would fix this case but not the real issue: a warm sensor drifting back toward zero may read > 0.02 legitimately, and hard-failing the run prevents collecting data on whether the warm-up affects results.

**Decision**:
1. `_analyze_validation()`: `baseline_ok` no longer appears in `result["passed"]`. It remains in the result dict as `baseline_mean` / `baseline_ok` for informational display.
2. `_start_validation()`: pre-flight check — if `S.vessel_o3_pct > 0.01` at the moment the user clicks Validate, a persistent `ui.dialog` shows the current reading and offers "Cancel" / "Proceed anyway". Uses `asyncio.Event` to suspend the coroutine until user responds.

**Rationale**: The sensor floor ambiguity is a hardware characteristic (106-H), not a system fault. Treating it as a soft warning gives the operator context while allowing data collection in ambiguous cases. Pass/fail is fully determined by the target-phase accuracy and stability — quantities the operator cannot meaningfully affect by re-zeroing.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-09: ±1σ Confidence bands on Power-O3 curve via Jacobian propagation

**Context**: The Power-O3 curve showed the fitted sigmoid with no uncertainty information. Users had no way to assess model quality visually — only the scalar R² and RMSE in the model summary. The fitting already used `scipy.optimize.curve_fit` which returns `pcov` (parameter covariance matrix), but `pcov` was discarded.

**Decision**:
1. `PowerO3Model`: added `ci_power: list[float]` and `ci_sigma: list[float]` fields (default empty list — backward compatible with existing JSONs).
2. `fit_sigmoid_model()`: after goodness-of-fit, compute a 101-point ±1σ band using Jacobian propagation:
   - `J[n,i]` = partial derivative of sigmoid w.r.t. parameter `i` at power level `n`
   - `var_curve[n] = J @ pcov @ J.T` (via `np.einsum('ni,ij,nj->n', J, pcov, J)`)
   - `ci_sigma = sqrt(max(var_curve, 0))`
3. Stored in the model JSON as `ci_power` / `ci_sigma` (101 floats each).
4. `load_model()`: filter unknown keys against `__dataclass_fields__` — prevents future schema additions from crashing old-model loads.
5. `_make_power_fig()`: if `S.active_model.ci_sigma` is non-empty, adds a `fill="toself"` Plotly scatter trace as the closed polygon `(pwr, o3+σ) + (pwr_rev, o3_rev−σ)` in `rgba(65,105,225,0.15)`. No band rendered for the piecewise fallback model (no `pcov`).

**Rationale**: Jacobian propagation is the correct first-order uncertainty treatment for nonlinear least squares — it accounts for parameter correlations (off-diagonal `pcov` terms) and produces a band that naturally widens in the steep transition region where k and P0 uncertainty dominates. This is more informative than symmetric ±RMSE which is homoscedastic and ignores parameter covariance.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-04: Single-agent model for full-stack development

**Context**: BlockSI features now routinely span the ESP32↔PC boundary
(calibration phases, command argument formats, relay control, buffer sizes).
The dual-agent model required updating `interface_contract.md`, then waiting
for the other agent to read it in a new context window, leading to coordination
lag, context drift, and subtle regressions (e.g. the `CMD,calibrate,random=`
feature touched 5 files across both domains simultaneously).

**Decision**:
1. A **single GitHub Copilot agent** handles both ESP32 firmware and PC
   dashboard work within the same chat session.
2. `collaboration_protocol.md` updated to reflect the single-agent model.
3. Both domain summaries (`dashboard_agent_summary.md` and
   `esp32_agent_summary.md`) are still maintained as persistent memory
   across context windows — they remain the new-chat startup documents.
4. The agent is responsible for updating `interface_contract.md` and both
   summaries at the end of each session (previously each agent updated only
   its own summary).

**Rationale**: With a single agent, both sides of a cross-boundary feature
are implemented in one pass with consistent state.  The interface contract
stays up to date because the same agent writes both sides.  Documentation
handoffs (the primary source of coordination bugs) are eliminated.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-04: Random phase re-added to calibration with PC-generated levels

**Context**: Decision 2026-02-28 dropped the random phase from calibration
because the sweep up + down was sufficient for initial model fitting.  After
collecting calibration data at multiple flow rates (3.0, 3.5, 4.0, 4.5,
5.0 LPM), users wanted more samples at each power level with adequate
stabilization time (~20 samples ≈ 50s), especially at lower flow rates where
the sensor stabilizes more slowly.

**Decision**: (Supersedes random-phase-dropped note in 2026-02-28 entry)
1. **Optional random phase** appended after sweep_down.  PC generates `N`
   unique stratified power levels (one per equal-width window across 0-100%),
   arranged ascending then descending (mountain shape) to minimize pot travel.
   Each level holds for 20 samples.
2. **PC generates levels** (not ESP32) — upholds "ESP32 = Arms, PC = Brains".
   Levels sent as comma-separated integers in `random=` arg.
3. **N=0 (default)** produces identical 203-step behavior to prior decision.
4. **Air compressor toggle** for entire calibration run (all phases).  Previously
   always OFF.  `air_comp=1` enables for baseline, sweeps, and random phase.
5. **O2 concentrator gating**: Only activated when `flow > 0`.  Air-only mode
   (LPM=0, air_comp=1) is valid.  If both LPM=0 and air_comp=0, PC GUI
   rejects start with an error notification.
6. **GUI inputs added**: `# Rnd Lvls` (0-50, default 15) and `Air ON` toggle
   adjacent to the existing `O2 LPM` input in the Calibration expansion.
7. **Step total**: `203 + 2*N` (each unique level visited twice).
8. **LAN command format**: `CMD,calibrate,flow=4.0,air_comp=1,random=3,22,...\n`
   ESP32 parses `air_comp=` and `random=` from args.  `RX_LINE_SIZE` raised
   512 bytes (was 128) to accommodate long random lists.
9. **Max steps**: `SEQ_EXEC_MAX_STEPS` raised 512 (was 256) to accommodate
   203 + up to 100 random steps (50 levels × 2 visits).

**Rationale**: Motorized potentiometer takes ~15s to travel full range.
Sorting random levels ascending then descending minimizes travel while
providing symmetric data at each chosen power level.  Stratified windows
ensure coverage across the full 0-100% range rather than clustering.
20 samples per step ≈ 50s of stabilized readings — adequate for the
sensor time constant at all tested flow rates.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-04: `_safe_standby()` — unconditional safe state function

**Context**: `_sequence_cleanup()` had a `seq_confirmed` guard that skipped
power/relay shutdown if the ESP32 had not confirmed sequence start.  This
caused the dangerous behavior where calibration exiting set power to 86%
(the pot's last position) with all relays remaining on, because the guard
prevented the cleanup commands from being sent.

**Decision**:
1. New `async def _safe_standby()` function — unconditionally sends
   `cmd_set_power(0)`, `relay ozone_gen OFF`, `relay o2_conc OFF`,
   `relay air_comp OFF` with no guards.
2. `_sequence_cleanup()` now always calls `_safe_standby()`, removing the
   `seq_confirmed` guard that bypassed shutdown.
3. `_safe_standby()` is the single reusable "all off" primitive — future
   sequences call it instead of duplicating the relay/power shutdown logic.

**Rationale**: The `seq_confirmed` guard's original intent was to avoid
sending commands to a disconnected ESP32.  But `_safe_standby()` already
handles TCP errors gracefully (commands fail silently if disconnected).
The guard's side-effect — leaving equipment in a live state — is far more
dangerous than the spurious error it was preventing.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-04: DFRobot SEN0321 I2C configuration fix

**Context**: The room O3 sensor (DFRobot SEN0321 Gravity, address 0x73)
was detected by the I2C bus scan but all register read/write operations
failed.  Boot log showed alternating `ESP_ERR_TIMEOUT` on reads and
`ESP_FAIL` on writes.  Room O3 read as -1.000 ppm.

**Root causes identified**:
1. I2C bus speed 400 kHz — sensor's internal STM8S MCU tested at 100 kHz only
2. `i2c_master_write_read_device()` uses I2C repeated-start (Sr) — sensor
   expects separate STOP/START transactions with 100ms gap between write and read
3. `dfrobot_o3_is_present()` used a bare read (could corrupt register pointer)
4. Two tasks (driver auto-sample + `sensor_aggregator`) hitting I2C without mutex
5. Internal pull-ups (~45kΩ) are marginal, though less critical at 100kHz

**Decision**:
1. **`blocksi_pins.h`**: `I2C_MASTER_FREQ_HZ` 400000 → 100000
2. **`dfrobot_ozone.c` `sensor_read_reg()`**: Rewritten — separate
   `i2c_master_write_to_device()`, 100ms `vTaskDelay`, then
   `i2c_master_read_from_device()`.  Matches DFRobot Arduino `i2cReadOzoneData()`.
3. **`dfrobot_ozone.c` `dfrobot_o3_is_present()`**: Changed to write-only
   probe (START → ADDR_W → STOP), matching DFRobot `begin()`.
4. **`peripherals.c`**: `.sample_interval_ms = 0` disables internal auto-sample
   task; `sensor_aggregator` is sole reader (500ms interval).
5. External 4.7kΩ pull-ups: recommended hardware change, not yet added.

**Rationale**: The scan succeeds at 400kHz because it's a short 1-byte probe.
Longer multi-byte register transactions exceed the sensor MCU's timing budget
at fast mode.  The repeated-start mismatch is the secondary cause — the sensor
MCU needs the STOP condition and 100ms to prepare data.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-28: 4-parameter sigmoid model for Power→O3 prediction

**Context**: The dashboard used a hardcoded piecewise model
(`POWER_MODEL_A=1.78`, `POWER_MODEL_B=1.40`) for predicting O3 from
power %. This model had three segments (threshold/ramp/saturation) with
arbitrary breakpoints at 20% and 75% power. Calibration data showed
the real relationship is a smooth sigmoid, and the piecewise model could
not be updated from calibration results — it required manual editing of
constants in the source code.

**Decision**:
1. **4-parameter sigmoid**: `O3 = L / (1 + exp(-k*(P - P0))) + b`
   - `L` = asymptote height (max O3 above baseline)
   - `k` = steepness
   - `P0` = inflection point (power % at half-max)
   - `b` = baseline offset
2. **One model per (flow_lpm, o2_pct)**: Each operating condition gets
   its own model file in `Models/O3Power/` as JSON.
3. **Analysis module**: `Interfaces/PC/analysis/power_o3_model.py` — standalone
   module with fitting, persistence, and prediction functions.
4. **Auto-aggregation**: All calibration CSVs for a (LPM, O2%) condition
   are automatically aggregated for fitting. User can exclude files.
5. **Manual fit only**: User clicks "Fit Model" button — no auto-fit on
   calibration completion.
6. **Graceful fallback**: If no fitted model exists for the current
   condition, the original piecewise model is used automatically.
7. **Model auto-loading**: Active model reloads on startup, LPM change,
   and air compressor toggle (which changes effective O2%).

**Rationale**: A sigmoid is the natural shape for an ozone generator's
power-concentration curve (electrochemical process with saturation).
The 4 parameters are interpretable and bounded. `scipy.optimize.curve_fit`
with bounds produces robust fits — initial test with 3 calibration files
yielded R²=0.9982 vs the piecewise model which had no formal goodness
measure. Per-condition models ensure accuracy across different flow rates
and O2 concentrations.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-28: Single-command calibration — ESP32 owns sweep pattern

**Context**: The recipe-based calibration protocol required 221 commands
(1 `sequence_start` + 218 `seq_step` + 1 `seq_prompt` + 1 `seq_run`)
sent from the PC to the ESP32.  This caused calibration to immediately
abort because:
1. ESP32's LAN client has 50ms polling delay + 63-byte recv buffer,
   processing only ~20-40 commands/second
2. 218 fire-and-forget `seq_step` commands take ~5-7s to process
3. Dashboard's `send_command("seq_run")` picks up stale
   `RSP,OK,seq_step,...` response instead of the real seq_run RSP
4. Periodic relay sync (`_tick()` every 10s) races on the same queue

Additionally, the protocol was overly complex — the sweep pattern
(0→100→0 in 1% steps) is physics-dictated and won't change, making it
essentially motor control rather than "brains" work.  The design was
also CLI-unfriendly: manually typing 200+ commands is impractical.

**Decision**: (Supersedes recipe-based calibration)
1. **Single command**: `CMD,calibrate,flow=<lpm>` loads + runs the
   entire 203-step sweep on ESP32.  1 command in, 1 response out.
2. **ESP32 owns the sweep**: `seq_executor_load_calibration()` builds
   baseline (0%, 15 samples) + sweep up (0→100%, 2 samples each) +
   sweep down (100→0%, 2 samples each) internally.
3. **Recipe protocol preserved**: `sequence_start/seq_step/seq_run` still
   works for future custom sequences (validate, sterilize) where the PC
   genuinely needs to define the step list.
4. **Legacy aliases**: `calibrate_start/stop/status` route to seq_executor.
5. **Random phase dropped** for now — sweep up + down provides sufficient
   hysteresis data for model fitting.

**Rationale**: Standard practice in process control (SCPI, Modbus, etc.)
uses parameterized commands, not individual step-by-step scripting.  A
calibration sweep is a fixed routine — the ESP32 should own it.  This
eliminates ALL response queue issues (1 command instead of 221) and
makes calibration CLI-testable with a single line.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-28: Executor-owned relay prerequisites with hardware interlock

**Context**: Previous decision (2026-02-27) had `seq_run` reject if relays
were OFF, requiring the PC to send separate `relay_set` commands before
`seq_run`.  User feedback: the ESP32 should **activate** relays itself,
not just reject.  Additionally, the air compressor is physically internal
to the MP-8000 generator — its relay circuit has NO power unless the
`ozone_gen` SSR is ON.  This hardware dependency was undocumented and
unenforced.

**Decision**: (Supersedes 2026-02-27)
1. **Relay prereqs via params**: `sequence_start` accepts `relay_o2=<0|1>`,
   `relay_o3=<0|1>`, `relay_air=<0|1>` params.  Omit = don't change.
   The executor saves original states, applies prereqs in safe order
   (O2 → O3 → Air), waits 3s for stabilization, sends `SEQ,*,RELAY`
   notification, then begins step execution.
2. **Cleanup**: On completion: power=0, air_comp=OFF, O2/O3 unchanged.
   On abort: power=0, air_comp=OFF, ozone_gen=OFF (safety).
3. **Hardware interlock in relay_control.c**: `relay_set_with_source()`
   enforces: (a) `air_comp ON` rejected when `ozone_gen OFF`, (b)
   `ozone_gen OFF` auto-sets `air_comp OFF`.  Catches ALL callers.

**Rationale**: Single-command startup (`seq_run`) is simpler and less
error-prone than requiring 2-3 relay commands + delays + retry logic on
the PC side.  The hardware interlock prevents silent failures where the
air compressor relay is toggled but no current flows.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-27: seq_run pre-flight relay validation  `[SUPERSEDED by 2026-02-28]`

**Context**: User attempted a calibration but relays (O2 concentrator and
ozone generator) were not toggled ON.  Power control worked (motor pot
moved) but no O3 was produced because the generator and O2 source were
off.  The old `seq_power_cal.c` checked these as prerequisites (lines 501
and 507) but the generic recipe executor had no such checks.  The PC
dashboard's `cmd_sequence_start` only verified `air_comp=0` — it never
activated `ozone_gen` or `o2_conc`.

**Decision**: Add pre-flight relay checks in the `seq_run` LAN command
handler in `main.c`.  If `ozone_gen` or `o2_conc` are OFF, return
`RSP,ERR,seq_run,preflight_fail:ozone_gen_off` (or `o2_conc_off`).
The ESP32 does NOT auto-activate relays (consistent with "ESP32 = Arms"
principle).  The PC must send `relay_set` commands before `seq_run`.

**Rationale**: Safety — running a recipe without the generator or gas
source ON wastes time and produces meaningless data.  Making the ESP32
reject the recipe forces the PC to handle relay lifecycle explicitly.
The error message is specific enough for the PC to auto-diagnose and
potentially auto-fix (send the relay commands and retry).

**Status**: `[SUPERSEDED]` — See 2026-02-28 entry above.

---

### 2026-02-24: PC is sole authority for power_target_pct

**Context**: Stale ESP32 DATA telemetry lines carried old `power_target_pct`
values.  When `apply_telemetry()` accepted these, the UI slider was overwritten,
triggering a new command to ESP32 -- causing an infinite oscillation loop.

**Decision**: `apply_telemetry()` on the PC dashboard **never** accepts
`power_target_pct` from ESP32 telemetry (DATA field index 15).  The PC is the
sole authority for what power level is commanded.  The ESP32 reports
`power_actual_pct` and `wiper_voltage` as sensor readings, which the PC does
accept.

**Rationale**: The PC originates all manual power commands and runs the
calibration state machine.  There is no scenario where the ESP32 should
unilaterally change the PC's power target.  ESP32-side `power_target_pct` in
telemetry is informational only (useful for debug/logging).

---

### 2026-02-24: ESP32 should own automated sequences (future)

**Context**: The CalibrationRunner on the PC sends individual
`CMD,power_set,N` commands over TCP every 2 seconds.  A network hiccup could
corrupt the sequence.  The ESP32 already has `power_calibration_v2.c` which
runs sweep logic locally with direct hardware access.

**Decision**: For future development, ESP32 will own automated sequences
(calibration sweeps, sterilization cycles, etc.).  Design:
- PC sends `CMD,sequence_start,cal,<params>` to initiate
- ESP32 runs autonomously, streams `SEQ,phase,progress,power,...` status lines
- PC dashboard enters observer mode (`sequence_active=True`), showing progress
- PC sends `CMD,sequence_stop` to abort
- Normal `DATA,...` telemetry continues alongside `SEQ,...` status

**Rationale**: ESP32 has deterministic timing (FreeRTOS), direct hardware
access, and no network dependency during execution.  PC is better suited as
a monitoring/visualization layer.

**Status**: `[DECIDED]` -- not yet implemented.  CalibrationRunner currently
runs PC-side.

---

### 2026-02-24: Deleted deprecated power_calibration.c (DS3502 version)

**Context**: `power_calibration.c` was a standalone program written for the
old DS3502 digital potentiometer.  It had its own `app_main()` and serial
command interface, and was never compiled into the current firmware (not in
`CMakeLists.txt`).  The replacement `power_calibration_v2.c` uses the motor
pot and is fully integrated.

**Decision**: Deleted `power_calibration.c` from the repository.

**Rationale**: Dead code creates confusion.  The file used DS3502 APIs that
no longer exist in the project.

---

### 2026-02-25: Adopted multi-agent collaboration protocol

**Context**: BlockSI development spans two domains (ESP32 firmware and PC
dashboard) with separate context requirements.  Long chat sessions risk
context truncation and stale references.

**Decision**: Established a formal collaboration protocol in
`.github/copilot/` with:
- Domain-separated agent chats (Dashboard Agent, ESP32 Agent)
- Agent-written summaries (`dashboard_agent_summary.md`, `esp32_agent_summary.md`)
- Shared interface contract (`interface_contract.md`)
- This decisions log
- Status taxonomy: `[IMPLEMENTED]`, `[DECIDED]`, `[PROPOSED]`
- Interface Change Rule: update contract first, then summary

**Rationale**: Prevents context loss across sessions, ensures cross-domain
changes are coordinated, and gives each agent focused deep context in its
domain.

---

### 2026-02-25: Relay source-tracking and audit logging

**Context**: Relay dropouts were reported but hard to diagnose because
`relay_set()` logged state changes without identifying the caller (LAN, RPC,
internal, boot, etc.).  Additionally, `blocksi_state_set_relay()` only tracked
2 of 3 relays, and the LAN command handler bypassed state tracking entirely.

**Decision**: 
1. Added `relay_source_t` enum (`BOOT`, `LAN`, `RPC`, `INTERNAL`, `SEQUENCE`,
   `NVS_RESTORE`, `EMERGENCY`, `UNKNOWN`) to all relay state changes.
2. `relay_set_with_source()` logs every change with source tag.
3. All relay commands (LAN handler, Golioth RPC) now route through
   `blocksi_state_set_relay()` for unified tracking.
4. `blocksi_relay_state_t` expanded to track all 3 relays with per-relay
   `last_source` and `last_change_ms`.

**Rationale**: First-line diagnostic for relay dropout investigation.  Source
tracking makes it immediately visible whether a dropout is from a watchdog
reset, network event, RPC call, or firmware bug.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: NVS relay persistence with reset-reason awareness

**Context**: Relay states lived only in RAM.  Any reset (watchdog, brownout,
crash) caused all relays to drop to OFF, interrupting active operations.
However, restoring relays after an intentional power-off (e.g., emergency
unplug) would be a safety hazard — ozone could start flowing unexpectedly.

**Decision**: Use `esp_reset_reason()` to distinguish reset types:
- **Power-on / external reset**: All relays OFF (safety)
- **Watchdog / brownout / panic / SW reset**: Restore from NVS

Relay states are persisted to NVS (`relay_ctrl/states`) on every state change.
On boot, `relay_restore_from_nvs()` is called only for transient resets.
On power-on, the all-OFF state is saved to NVS to clear stale data.

**Rationale**: Balances operational continuity (survive glitches) with safety
(never auto-start ozone after intentional power-off).

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: LAN connect/disconnect event callbacks and STATE push

**Context**: When TCP connection dropped and re-established, the PC dashboard
had no way to know the ESP32's current relay/power state without manually
querying.  Relay controls could appear out of sync.

**Decision**:
1. Added `lan_event_callback_t` to `lan_client_config_t` — called on
   connect and disconnect events.
2. On connect, ESP32 sends unsolicited `STATE,...` message with current
   relay states, power level, and flow rate.
3. Added `STATE` message type to `interface_contract.md`.

Format: `STATE,ozone_gen=<0|1>,o2_conc=<0|1>,air_comp=<0|1>,power=<pct>,flow=<lpm>\n`

**Rationale**: Ensures PC dashboard is always synchronized on reconnect.
Relay/power state survives disconnects on the ESP32 side, and the PC picks
up the actual state immediately without needing to poll.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Generic sequence runner framework

**Context**: The ESP32 needs to run at least 5 autonomous sequence types
(power-O3 calibration, airflow validation, fill model calibration, O3 decay
testing, sterilization).  Each shares common patterns: multi-phase execution,
SEQ progress streaming, graceful abort, safe cleanup, mutual exclusion.

**Decision**: Created a generic `sequence_runner` framework:
- `sequence_runner.h/.c`: Lifecycle management, task spawning, SEQ message
  streaming, LAN command integration (`sequence_start`, `sequence_stop`,
  `sequence_status`)
- `sequence_impl_t` interface: `prepare()`, `execute()`, `request_stop()`,
  `cleanup()` — each sequence type implements this
- Helper functions: `seq_report_progress()`, `seq_check_stop()`, `seq_send_data()`
- Registration pattern: `sequence_runner_register()` per type at boot

First implementation: `seq_power_cal.c` (type name `"cal"`), 4-phase calibration:
1. baseline (30s at 0%, air OFF)
2. sweep_up (0→100% in 1% steps, air OFF)
3. sweep_down (100→0% in 1% steps, air OFF)
4. random_pair (15 random levels × 20s air OFF + 20s air ON)

Commands: `CMD,sequence_start,cal,4.0` / `CMD,sequence_stop` / `CMD,sequence_status`

The old `calibrate_start`/`calibrate_stop`/`calibrate_status` commands remain
functional for backward compatibility but the dashboard should migrate to
the new `sequence_*` commands.

**Rationale**: A single framework avoids duplicating task management, SEQ
streaming, abort handling, and safety cleanup across 5+ sequence types.
Each new sequence is just a ~200-line implementation file.

**Status**: `[IMPLEMENTED]` — framework + power calibration sequence

---

### 2026-02-25: Interactive prompt support in sequence runner

**Context**: The airflow/concentration validation sequence requires operator
interaction at two points: routing the L-valve to the vessel, then routing
direct to the 106-H sensor and matching flow.  The sequence must pause and
wait for the operator to physically adjust hardware and confirm readiness.

**Decision**: Added interactive prompt/confirm mechanism to the sequence runner:
1. `seq_prompt_user(prompt_id, message, ...)` — sends `SEQ,prompt,...` to PC,
   blocks the sequence task on a FreeRTOS queue until confirmation arrives
2. `CMD,sequence_confirm,<prompt_id>[,<value>]` — LAN command from PC, 
   calls `sequence_runner_provide_confirmation()` which unblocks the queue
3. Queue depth 1, prompt ID matching, 5-minute timeout, stop-check polling

The mechanism is generic — any future sequence can use `seq_prompt_user()` for
operator interaction without changes to the framework.

**Rationale**: Physical operations (valve routing, flow adjustment) can never
be automated with current hardware.  Rather than splitting the sequence into
multiple commands ("start part 1", "start part 2"), a single sequence with
interactive pauses gives the operator a guided workflow.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Airflow/concentration validation sequence

**Context**: Before running sterilization or decay testing, the operator needs
to verify that the system produces the expected O3 concentration.  The 106-H
measures O3 at the vessel outlet, but vessel transit delay, surface decay, and
substrate absorption can cause discrepancies.  A direct-to-sensor measurement
eliminates these variables.

**Decision**: Implemented `seq_airflow_val.c` (type: `"validate"`), 5-phase
interactive sequence:
1. `prompt_vessel` — operator routes L-valve to vessel, confirms rotameter LPM
2. `prompt_direct` — operator routes L-valve direct to 106-H, matches flow
3. `stabilize` — power on at target level, 60s stabilization
4. `measure` — 60s measurement window, sample every 2s, compute mean/std
5. `complete` — report expected vs actual O3, deviation percentage

Parameters: `CMD,sequence_start,validate,<power_pct>,<o2_lpm>` (defaults 75%, 4.0)
New messages: `VAL_START`, `VAL_DATA`, `VAL_RESULT`

This sequence is intended as a sub-sequence / pre-flight check for sterilization
and decay testing workflows.

**Rationale**: The direct-to-sensor route gives a clean baseline measurement.
Future sterilization/decay sequences can call this as a prerequisite check.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Sequence Integration Guide — ESP32 executes, Dashboard observes

**Context**: The airflow validation sequence (`seq_airflow_val.c`) is heavily
interactive — operator must physically manipulate valves and confirm via the UI.
This raised the question of whether the ESP32 was overstepping into Dashboard
territory, and whether the Dashboard agent has clear guidance on what to build.

**Decision**: Added a comprehensive "Sequence Integration Guide" section to
`interface_contract.md`.  Key principles:

1. **ESP32 owns all timing-critical execution** — phase transitions, hardware
   control, data streaming, statistics computation.
2. **Dashboard is a thin observer** — shows progress, renders prompts as rich
   UI (not limited to ESP32's plain-text fallback), can abort.
3. **Prompt IDs are semantic** — Dashboard maps `prompt_vessel`, `prompt_direct`
   to rich dialogs with step-by-step instructions, diagrams, and optional
   numeric input.  The ESP32's `<message_text>` is a minimal fallback.
4. **Dashboard never sends `power_set` or `relay_set` during active sequence** —
   except via E-STOP which sends `sequence_stop` first.
5. **Responsibility matrix** and **message handling table** give the Dashboard
   agent a concrete checklist of what to parse and what UI to build.
6. **CalibrationRunner refactoring plan** documented: current PC-driven approach
   must become observer mode using `CMD,sequence_start,cal,...` and `SEQ,...`/`CAL_DATA`
   parsing.

**Rationale**: Clear documentation prevents both agents from accidentally
duplicating work or leaving gaps.  The Dashboard agent needs to build substantial
UI (prompt dialogs, observer banner, live charts, result displays) and should
not have to reverse-engineer the protocol from ESP32 source code.

**Status**: `[DECIDED]` → `[IMPLEMENTED]` (Dashboard agent, session 4)

---

### 2026-02-25: Dashboard observer-mode rewrite (CalibrationRunner deleted)

**Context**: The PC dashboard had a `CalibrationRunner` class (~175 lines) that
drove calibration sequences PC-side by sending individual `CMD,power_set,N`
commands every 2 seconds.  The ESP32 agent implemented a generic sequence runner
framework with autonomous execution, interactive prompts, and data streaming.
The dashboard needed to transition from "driver" to "observer."

**Decision**: Complete rewrite of `blocksi_dashboard.py` to observer mode:
1. **Deleted CalibrationRunner** entirely — no PC-driven sequence logic remains
2. Added 12 TCP message handlers in `_dispatch()`: DATA, RSP, STATE, SEQ,
   SEQ_DONE, CAL_START, CAL_DATA, CAL_COMPLETE, VAL_START, VAL_DATA, VAL_RESULT,
   plus SEQ,prompt special case
3. Added 3 new command helpers: `cmd_sequence_start()`, `cmd_sequence_stop()`,
   `cmd_sequence_confirm()`
4. SystemState rewritten: removed PC-driven cal fields, added observer fields
   (`seq_type`, `seq_phase`, `seq_progress`, `cal_samples`, `val_result`,
   `pending_prompt_id`, etc.)
5. `_tick()` rewritten: handles control lockout, prompt dialog triggering,
   cal/val observer UI updates
6. Pre-rewrite backup saved to `Interfaces/PC/Old/blocksi_dashboard_pre_observer.py`

**Rationale**: ESP32 has deterministic timing, direct hardware access, and no
network dependency during execution.  PC is better suited as monitoring/
visualization layer.  User explicitly chose to do the full rewrite in one pass
to avoid loose ends ("not addressing it all together would leave loose ends").

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Calibration and Validation in Power tab (not separate tabs)

**Context**: The old dashboard had a standalone "Calibration" tab.  With the
addition of a Validation sequence, the question arose of where to place UI
for both sequences.

**Decision**: Both Calibration and Validation are placed in the **Power tab**
as `ui.expansion()` sections, alongside the Power Control expansion.  The
standalone Calibration tab was removed.  Tab count reduced from 5 to 4:
Power, Telemetry, Debug, Settings.

**Rationale**: User preference — "Perhaps we could have BOTH the calibration
sequence and validation accessed as in the power tab since that is what it is
being calibrated or validated."  Future sequence types (fill, decay, sterilize)
will get their own tabs when implemented.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: CalibrationRunner deletion (recoverable from git)

**Context**: The CalibrationRunner was ~175 lines of PC-driven calibration
state machine.  With the ESP32 owning sequence execution via the sequence
runner framework, this code became dead.

**Decision**: Deleted CalibrationRunner entirely rather than keeping it as
fallback.  Pre-rewrite file saved to `Old/blocksi_dashboard_pre_observer.py`.

**Rationale**: User explicitly said "Delete it. If we need to fall back we
can recover it from github."  Keeping dead code creates confusion and
maintenance burden.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-26: Generic recipe executor replaces sequence-specific firmware

**Context**: The ESP32 had sequence-specific firmware (`seq_power_cal.c`,
`seq_airflow_val.c`) that embedded calibration/validation logic — random
value generation, specific phase structures, statistics computation.  This
violated the "ESP32 = Arms, PC = Brains" principle established in session
discussions.  The ESP32 was overstepping into the PC's data analysis domain.

**Decision**: Replace all sequence-specific firmware with a single generic
recipe-based executor (`seq_executor.c`):
1. PC generates complete recipes (step lists, prompts, hold durations)
2. PC sends recipes via `sequence_start` → `seq_step` × N → `seq_prompt` × M → `seq_run`
3. ESP32 executes blindly: set power, count 106-H samples, stream `SEQ,<type>,SAMPLE,...`
4. ESP32 does NO analysis — no mean/std, no model queries, no pass/fail
5. Sample-counted holds (via `seq_sensor_adapter.c`) instead of time-based waits
6. Integration with existing `sequence_runner.c` for UI lockout via bridge functions

Files removed from build (kept on disk): `seq_power_cal.c`, `seq_airflow_val.c`
Files added: `seq_executor.c`, `seq_executor.h`, `seq_sensor_adapter.c`, `seq_sensor_adapter.h`

Superseded messages: `CAL_DATA`, `CAL_START`, `CAL_COMPLETE`, `VAL_DATA`,
`VAL_START`, `VAL_RESULT`, `SEQ_DONE`.  All replaced by generic
`SEQ,<type>,SAMPLE,...` / `STEP` / `COMPLETE` / `ABORTED`.

**Rationale**: The PC has Python + pandas + numpy for data analysis, model
fitting, and certificate generation.  The ESP32 should focus exclusively on
precise hardware control and deterministic timing.  A generic executor means
the PC can run ANY recipe type without firmware changes — only the PC-side
recipe generator needs updating for new sequence types.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-26: Air compressor OFF during calibration and validation

**Context**: The old `seq_power_cal.c` had a Phase 4 ("random_pair") that
toggled the air compressor ON/OFF to characterize air-blend conditions.
Discussion established that O2-only conditions (~93% O2) are the baseline
for calibration and validation.  Air-blend conditions (~21% O2 at ~10 LPM
additional flow) fundamentally change the ozone chemistry and require a
completely separate calibration.

**Decision**: Air compressor must be OFF during all standard calibration
and validation sequences.  The PC should verify `air_comp=0` before sending
`seq_run`.  Air-blend characterization, if needed, is a separate recipe
type generated by the PC.

**Rationale**: Mixing O2-only and air-blend data in one calibration produces
unreliable models.  The variables (O2 concentration, total flow) change
simultaneously.  Clean O2-only data first, then optionally characterize
air-blend as a distinct operating condition.

**Status**: `[DECIDED]`

---

### 2026-02-26: Dashboard migrated to recipe-based sequence protocol

**Context**: The ESP32 agent replaced sequence-specific firmware
(`seq_power_cal.c`, `seq_airflow_val.c`) with a generic recipe-based
executor (`seq_executor.c`).  The old protocol used type-specific messages
(`CAL_START`, `CAL_DATA`, `CAL_COMPLETE`, `VAL_START`, `VAL_DATA`,
`VAL_RESULT`, `SEQ_DONE`).  The new protocol uses generic
`SEQ,<type>,STARTED/STEP/SAMPLE/PROMPT/COMPLETE/ABORTED` messages.

**Decision**: Updated the PC dashboard to:
1. **Generate recipes**: `generate_cal_recipe()` (~218 steps) and
   `generate_val_recipe()` (5 steps + 2 prompts)
2. **Send recipes**: `cmd_sequence_start()` sends the full recipe
   (sequence_start → seq_step × N → seq_prompt × M → seq_run)
3. **Handle generic SEQ messages**: Single `_handle_seq()` routes by
   action (STARTED, STEP, SAMPLE, PROMPT, COMPLETE, ABORTED)
4. **PC-side analysis**: `_analyze_validation()` computes mean/std/CV,
   baseline check, spot correlation, target accuracy, pass/fail
5. **Removed**: All old type-specific handlers and dispatch routes
6. **Air compressor check**: Pre-flight validation before sending `seq_run`
7. **Command updates**: `sequence_abort` (primary), `sequence_confirm`
   (no prompt_id arg)

**Rationale**: Aligns dashboard with the "ESP32 = Arms, PC = Brains"
architecture.  The ESP32 executes recipes blindly; the PC owns all recipe
design, data analysis, and decision-making.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-26: Data management folder restructure and O2% in filenames

**Context**: All data files lived in flat directories (`Data/Stream/`,
`Data/O3PowerCalibration/`) or an ad-hoc `Model/O3Power/` folder.  Calibration
filenames lacked O2 concentration info, making it impossible to distinguish
runs at different gas blends.

**Decision**:
1. **Renamed folders**:
   - `Data/Stream/` → `Data/Telemetry/`
   - `Data/O3PowerCalibration/` → `Data/Calibration/`
   - `Model/O3Power/` → `Models/O3Power/` (top-level `Models/`)
2. **Added folders**: `Data/Validation/`, `Data/Fill/`, `Data/Decay/`,
   `Data/Sterilization/`, `Models/Fill/`, `Models/Decay/`
3. **Calibration filename**: `{YYYY-MM-DD}_{HHMMSS}_PowerO3Cal_{LPM}Lpm_{O2}O2.csv`
   - Timestamp replaces dedup suffix (_2, _3)
   - O2% is weighted-average: `(F_conc × 95 + F_air × 21) / (F_conc + F_air)`, rounded to int
4. **Git tracking**: `Data/` remains gitignored.  `Models/` is git-tracked
   (small JSON files, diffable, important fitted parameters).
5. **New function**: `compute_effective_o2_pct(flow_lpm, air_comp_on) → int`
6. **New constants**: `O2_CONC_PCT = 95`, `AIR_COMP_O2_PCT = 21`

**Rationale**: Separating data by type supports future sequence types.
O2% in filenames enables condition-specific model fitting.  Models in git
ensures fitted parameters are versioned and recoverable.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-27: Dashboard relay prereqs, SEQ RELAY/STATUS handling, air_comp in steps

**Context**: The ESP32 agent implemented executor-owned relay prerequisites
(relay params in `sequence_start`, `SEQ,*,RELAY` and `SEQ,*,STATUS`
notifications, per-step `air_comp` field).  The dashboard was not handling
these new messages — relays weren't toggled during sequences, the banner
showed no feedback during the relay stabilization phase, and step tuples
lacked the `air_comp` field.

**Decision**:
1. **Relay params in `sequence_start`**: Added `relay_o2=1,relay_o3=1,relay_air=0`
   to the command params for calibration and validation sequences.
2. **Belt-and-suspenders**: Dashboard pre-enables relays via `cmd_set_relay()`
   AND passes relay params (executor applies them authoritatively at `seq_run`).
3. **Handle `SEQ,*,RELAY`**: New handler updates `S.relay_*` state and sets
   `seq_phase="relay_setup"` for banner display.
4. **Handle `SEQ,*,STATUS`**: Handles `relay_stabilizing` → sets
   `seq_phase="stabilizing"` for banner display.
5. **Phase-aware banner**: Loading → Enabling relays → Stabilizing → Starting →
   Step X/N — Phase (replaces generic "Step X/N" during pre-run phases).
6. **Air_comp in step tuples**: 5th element added to all recipe steps (currently
   always 0 for pure-O2 calibration), sent via `CMD,seq_step,...,air_comp`.
7. **Air_comp in STEP/SAMPLE parsing**: Parses optional `parts[6]`/`parts[8]`
   fields; SAMPLE dict includes `"air_comp"` key for CSV output.

**Rationale**: Aligns dashboard with ESP32 executor-owned relay lifecycle.
Phase-aware banner gives immediate visual feedback during all sequence phases.
Belt-and-suspenders ensures relays are enabled even if one path fails.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-07: K constant reconciliation to 20°C reference

**Context**: Dashboard used `O3_MASS_FLOW_K = 0.357` (derived with
V_m = 22.4 L/mol at 0°C STP).  ESP32 dosimetry used `3.27e-5`
(derived with V_m = 24.5 at 25°C).  The rotameter is likely calibrated
at 20°C (1 atm), so V_m = 24.04 L/mol is the correct reference.

**Decision**:
1. Dashboard: `O3_MASS_FLOW_K = 0.3327` (= 48000 / (24.04 × 60 × 100))
2. ESP32: `PPM_TO_MGS_FACTOR = 3.327e-5` (= 48 / (24.04 × 60 × 1e6) × 1e3)
3. Both use `V_m = 24.04` with comments noting 20°C, 1 atm reference
4. Updated shared constant in `interface_contract.md`

**Rationale**: Consistent temperature reference eliminates ~7% systematic
error in mass flow calculations.  20°C matches rotameter calibration.

**Status**: `[IMPLEMENTED]`

---

### 2026-03-07: Fill/evacuation CSTR calibration — PC-driven

**Context**: Dosimetry requires knowing the system gas volume to model
fill/evacuation transients.  The vessel is a modified 11.3L pressure
tank with ~60% fill (substrate), leaving ~4L residual gas volume —
but the actual effective volume includes tubing and dead space.

**Decision**:
1. **PC-driven sequence** (not ESP32-driven like calibration/validation):
   PC sends `power_set` and `relay_set` commands directly, monitors
   DATA telemetry for steady-state detection.
2. **Fill phase**: 100% power, monitor until 5 consecutive samples ≥ 95%
   of target O3 (from validated power model)
3. **Evac phase**: 0% power, monitor until 5 consecutive samples < 0.01% vol
4. **CSTR model**: Fit exponential fill and evac curves separately,
   average τ values to compute system volume = mean(τ) × flow
5. **Separate CSVs**: `Data/Fill/` and `Data/Evac/` directories
6. **Model persistence**: JSON in `Models/Fill/`, keyed by (LPM, O2%)
7. **Air compressor toggleable** per phase (fill default off, evac default on)

**Rationale**: PC-driven is simpler than adding another ESP32 sequence
type — the PC already has direct command access and full DATA stream.
Separate fill/evac curves give two independent τ estimates (cross-validation).

**Status**: `[IMPLEMENTED]`

---

### 2026-03-07: Decay — use literature Arrhenius, skip empirical calibration

**Context**: O3 decay in the vessel (thermal/catalytic decomposition)
affects dosimetry accuracy.  Options: (a) empirical fill-hold-measure
decay calibration, (b) literature Arrhenius model.

**Decision**: Use Arrhenius model from ESP32 `dosimetry.c`
(k0=0.0003, α=0.02, Ea/R=2500) as minor correction term.  Skip
empirical decay calibration.

**Rationale**: At 4 LPM through ~8-12L vessel, decay is < 1% of O3
throughput.  Empirical calibration would add complexity for negligible
improvement.  The Arrhenius coefficients can be refined later if needed.

**Status**: `[DECIDED]`

---

### 2026-03-07: 106-H sensor on outlet; dosimetry scope on PC

**Context**: The 106-H ozone monitor is physically connected to the
**outlet** of the sterilization vessel, not the inlet.  A manual L-valve
allows switching between vessel-through and direct-to-sensor paths.

**Decision**:
1. Document 106-H outlet placement in `interface_contract.md` gas path
2. Mass balance: `absorbed = produced(model) - evacuated(106-H) - decayed`
3. All batch-level dosimetry computation on PC only
4. ESP32 `dosimetry.c` provides per-sample helpers (decay rate, sample
   accumulation) but is not the authority for batch dose

**Rationale**: PC has the validated model, the full data stream, and
the computing power for model-based mass balance.  ESP32 provides
real-time per-sample estimates for monitoring/safety.

**Status**: `[DECIDED]`

---

### 2026-03-07: ESP32 dosimetry defaults updated to match hardware

**Context**: Default vessel volume was 2.0L (placeholder), flow was 5.0 LPM.
Actual hardware: 11.3L modified tank, ~60% fill, 4.0 LPM standard flow.

**Decision**: Updated `dosimetry.c` defaults:
- `DEFAULT_VESSEL_VOL_L` = 11.3 (total tank)
- `DEFAULT_MATERIAL_VOL_L` = 7.3 (~60% fill → ~4.0L residual gas)
- `DEFAULT_FLOW_LPM` = 4.0
- `DEFAULT_HUMIDITY_PCT` = 60.0 (unchanged, typical indoor)

**Rationale**: Defaults should reflect actual hardware for meaningful
fallback calculations when PC is not connected.

**Status**: `[IMPLEMENTED]`