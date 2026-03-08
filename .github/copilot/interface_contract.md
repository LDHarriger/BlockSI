# BlockSI Interface Contract

> Last updated: 2026-03-07
>
> This is the **single source of truth** for everything shared between
> the ESP32 firmware and the PC dashboard.  Both agents MUST read this
> file and MUST update it before making cross-boundary changes.

## TCP Connection

| Property | Value |
|----------|-------|
| Transport | TCP over LAN |
| Port | 5000 (configurable via `--port` on PC) |
| Direction | ESP32 **connects to** PC (PC runs `asyncio.start_server`) |
| Encoding | UTF-8, newline-delimited (`\n`) |

## Message Types

### PC -> ESP32:  Commands
```
CMD,<command_name>[,arg1[,arg2]]\n
```

### ESP32 -> PC:  Responses
```
RSP,OK|ERR,<command_name>,<response_data>\n
```

### ESP32 -> PC:  Telemetry (unsolicited, periodic)
```
DATA,<17 fields>\n
```

### ESP32 -> PC:  Calibration data (during sweep)  `[SUPERSEDED]`
```
CAL_DATA,power_pct,actual_pct,o3_pct,temp_c,direction,elapsed_s\n
```
> **SUPERSEDED** by generic `SEQ,<type>,SAMPLE,...` format.  See Recipe Protocol below.

### ESP32 -> PC:  State push (on connect/reconnect)  `[IMPLEMENTED]`
```
STATE,ozone_gen=<0|1>,o2_conc=<0|1>,air_comp=<0|1>,power=<pct>,flow=<lpm>\n
```
Sent automatically by ESP32 immediately after TCP connection is established.
PC dashboard should parse this to synchronize its relay/power display with
the ESP32's actual state.  This ensures the PC is never out of sync after
a disconnect/reconnect cycle.

### ESP32 -> PC:  Backfill (cached data on reconnect)  `[IMPLEMENTED]`

When the PC was disconnected, the ESP32 caches DATA lines in a RAM ring
buffer (500 slots).  On reconnect the ESP32 waits 500 ms (so the PC can
process `time_sync`), then sends:

```
BACKFILL_START,<count>\n          -- Number of cached DATA lines to follow
DATA,...\n                         -- Oldest cached sample (same DATA format)
DATA,...\n                         -- ...
BACKFILL_END\n                     -- All cached lines sent
```

**Timing**: STATE is sent first, then `time_sync` RSP is processed, then
backfill begins after 500 ms.  This guarantees `esp_time_offset_ms` is
valid before backfill DATA is parsed.

**Reboot**: RAM cache is cleared on reboot, so all backfill data is from
the current boot epoch.  `esp_timestamp_ms` values are valid with the
current `time_sync` offset.

**PC handling**: During backfill, DATA samples are written to the CSV log
and appended to the graph buffer but do NOT update live sensor displays
(to avoid displaying stale values in the sidebar).

### ESP32 -> PC:  Recipe sequence messages  `[IMPLEMENTED]`

The generic recipe executor (`seq_executor`) emits these messages during
any sequence type.  The `<type>` field matches what was sent in `sequence_start`.

```
SEQ,<type>,STARTED,steps=<N>,flow=<X>\n           -- Recipe execution begins
SEQ,<type>,RELAY,<relay_name>,<0|1>\n             -- Relay state change (prereqs)
SEQ,<type>,STATUS,relay_stabilizing\n             -- Waiting for equipment (3s)
SEQ,<type>,STEP,<index>,<power_pct>,<phase>,<air_comp>\n -- Step transition
SEQ,<type>,SAMPLE,<step_idx>,<sample_num>,<o3_pct>,<temp_c>,<power_actual>,<air_comp>\n  -- Per-sample data
SEQ,<type>,PROMPT,<prompt_id>,<prompt_text>\n       -- Operator confirmation needed
SEQ,<type>,COMPLETE,<elapsed_s>\n                   -- Finished successfully
SEQ,<type>,ABORTED,<reason>\n                       -- Aborted
```

**SAMPLE fields**:
| Field | Type | Description |
|-------|------|-------------|
| `step_idx` | int | Step index from recipe |
| `sample_num` | int | 0-based sample within this step's hold |
| `o3_pct` | float | Vessel ozone %vol from 106-H |
| `temp_c` | float | 106-H cell temperature °C |
| `power_actual` | float | Actual power % from ADC feedback |
| `air_comp` | int | Air compressor state (0=OFF, 1=ON) |

**Key change from previous protocol**: All sequence types (calibration,
validation, sterilization) use the same `SEQ,<type>,SAMPLE,...` format.
The PC determines what `<type>` means and how to process the samples.
The old `CAL_DATA`, `VAL_DATA`, `CAL_START`, `CAL_COMPLETE`, `VAL_START`,
`VAL_RESULT` messages are **superseded** and no longer emitted.

### Old sequence-specific messages  `[SUPERSEDED]`

The following messages were used by `seq_power_cal.c` and `seq_airflow_val.c`
(now removed from the build).  Listed for reference only:
- `CAL_DATA`, `CAL_START`, `CAL_COMPLETE`
- `VAL_DATA`, `VAL_START`, `VAL_RESULT`
- `SEQ_DONE,<type>,<result>,<elapsed_s>,pts=<count>`
- `SEQ,<phase_name>,<progress_pct>,<power_pct>,<air_comp>,<elapsed_s>` (old format)

---

## CRITICAL:  Separator Rule

Commands use **COMMAS** as separators.  **NEVER** use colons.

- Correct: `CMD,power_set,50\n`
- WRONG:   `CMD,power_set:50\n`  (ESP32 parses `power_set:50` as the command name -- silently fails)

---

## DATA Telemetry Format

18 comma-separated tokens (prefix + 17 fields):

| Index | Field | Type | Unit |
|-------|-------|------|------|
| 0 | `DATA` | prefix | -- |
| 1 | `esp_timestamp_ms` | int | ms since boot |
| 2 | `vessel_o3_pct` | float | %vol |
| 3 | `temp_c` (cell) | float | C |
| 4 | `pressure_mbar` | float | mbar |
| 5 | `sample_v` | float | V |
| 6 | `ref_v` | float | V |
| 7 | `day` | int | -- |
| 8 | `month` | int | -- |
| 9 | `year` | int | -- |
| 10 | `hour` | int | -- |
| 11 | `minute` | int | -- |
| 12 | `second` | int | -- |
| 13 | `room_o3_ppm` | float | ppm |
| 14 | `vessel_temp_c` | float | C (-999 = N/A) |
| 15 | `power_target_pct` | int | % (0-100) |
| 16 | `power_actual_pct` | float | % |
| 17 | `wiper_voltage` | float | V |

**Authority note**: PC dashboard ignores `power_target_pct` (index 15)
from telemetry.  PC is sole authority for commanded power.
See `decisions_log.md` entry 2026-02-24.

---

## LAN Commands  (32 total, as of 2026-02-25)

### Relay Control
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `relay_set` | `<name>,<0\|1>` | `<name>=on\|off` | Names: `ozone_gen`, `o2_conc`, `air_comp` |
| `relay_get` | none | `ozone_gen=<0\|1>,o2_conc=<0\|1>,air_comp=<0\|1>` | |

### Power Control
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `power_set` | `<pct>` (0-100) | `power=<pct>,predicted_o3=<val>` | Integer percentage |
| `power_get` | none | `power=<pct>,actual=<val>` | |
| `flow_set` | `<lpm>` (0.1-10.0) | `flow=<lpm>` | O2 flow for prediction model |
| `flow_get` | none | `flow=<lpm>` | |
| `predict_o3` | `<flow>,<power>` | `predicted_o3=<val>` | Stateless prediction |

### Motor Pot
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `motor_pot_status` | none | Status string | ADC, wiper position |
| `motor_pot_calibrate` | none | `adc_min=<val>,adc_max=<val>,range=<val>` | Full sweep calibration |

### Single-Command Calibration  `[IMPLEMENTED]`

The ESP32 has the calibration sweep pattern on-board.  A single command
loads the recipe and starts execution immediately — no recipe protocol
needed.  Emits standard `SEQ,calibrate,...` messages during execution.

| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `calibrate` | `flow=<lpm>,air_comp=<0\|1>,random=<p1,p2,...>` (all optional) | `running,steps=<N>,flow=<lpm>` | Loads + runs sweep.  Relay prereqs: O2 ON if flow>0, O3 ON, Air per `air_comp`.  Random levels appended if provided. |

**Sweep pattern** (built internally by `seq_executor_load_calibration()`):
- Step 0: 0% power, 15 samples (~37s baseline)
- Steps 1-101: Sweep up 0→100% in 1% increments, 2 samples each
- Steps 102-202: Sweep down 100→0% in 1% increments, 2 samples each
- Steps 203+: Optional random hold steps at PC-generated power levels, 20 samples each
- Total: 203 + len(random_levels) steps

**Random phase**: PC generates stratified levels (N unique values across 0-100%), arranged
ascending then descending (mountain shape) to minimise pot travel.  Levels sent as
comma-separated integers in the `random=` arg: e.g. `random=5,22,38,55,71,84,98,98,84,71,55,38,22,5`.
With 0 random levels (default), calibration behaves identically to prior behaviour.

**Air compressor**: `air_comp=1` enables the air compressor relay for **all phases** (including
baseline and sweeps).  `air_comp=0` (default) keeps it off.  If O2 LPM = 0 and air_comp = 0,
calibration is rejected by the PC GUI before the command is sent.  If flow = 0 but air_comp = 1,
the O2 concentrator relay is left OFF (air-only mode).

**Stop/status** via standard sequence commands:

| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sequence_abort` | `[reason]` | `aborting` | Abort calibration (power->0, O3 OFF, Air OFF) |
| `sequence_status` | none | `state=<s>,type=calibrate,step=<n>/<total>` | |

**Legacy aliases** (route to seq_executor internally):
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `calibrate_start` | `[lpm]` | `running,steps=203,...` | Legacy alias for `calibrate` |
| `calibrate_stop` | none | `aborting,reason=calibrate_stop` | Legacy alias for `sequence_abort` |
| `calibrate_status` | none | `state=<s>,step=<n>/<total>` | Legacy alias for `sequence_status` |

### Recipe-Based Sequences  `[IMPLEMENTED]`

**Architecture**: "ESP32 = Arms, PC = Brains".  The PC generates complete
recipes and sends them step-by-step.  The ESP32 executes blindly with
precise sample-counted holds, streaming per-sample data back.

**Loading protocol** (PC → ESP32, in order):
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sequence_start` | `<type>,<key=value params>` | `type=<type>,status=loading` | Begins recipe loading.  Params: `flow=4.0,relay_o2=1,relay_o3=1,relay_air=0`.  Relay params: omit=don't change, 0=OFF, 1=ON |
| `seq_step` | `<idx>,<pwr>,<hold>,<phase>[,<air_comp>]` | `step=<idx>,pwr=<pwr>,hold=<hold>,air=<0\|1>` | Add step.  E.g. `0,0,15,baseline` or `5,50,5,random,1`.  Max 512 steps.  `air_comp` optional (default 0) |
| `seq_prompt` | `<before>,<id>,<text>` | `prompt=<id>,before=<before>` | Add prompt.  E.g. `0,check_flow,Verify O2 flow`.  Max 16 |
| `seq_run` | none | `running` | Sort steps, apply relay prereqs (3s stabilization), start execution task |

**Runtime control** (PC → ESP32):
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sequence_confirm` | none | `confirmed` | Unblock pending prompt |
| `sequence_abort` | `[reason]` | `aborting` | Abort with optional reason |
| `sequence_stop` | `[reason]` | `stopping` | Legacy alias for abort |
| `sequence_status` | none | `state=<s>,type=<t>,step=<n>/<total>` | States: idle, loading, running, waiting_confirm, complete, aborted |

**Step fields**:
| Field | Range | Description |
|-------|-------|-------------|
| `idx` | 0-65535 | Step ordering index (sorted ascending before execution) |
| `pwr` | 0-100 | Power level percent |
| `hold` | 1-65535 | Number of 106-H samples to collect at this power (~2.5s each) |
| `phase` | string | Phase label for display (max 23 chars, e.g. `baseline`, `sweep_up`) |
| `air_comp` | 0\|1 | Air compressor relay state for this step (optional, default 0) |

**Sequence types** (defined by PC, not by ESP32 firmware):
| Type | PC Generates | Description |
|------|-------------|-------------|
| `calibrate` | **On-board** (single cmd) | 0→100→0 in 1% steps, 203 total — ESP32 owns sweep pattern |
| `validate` | Spot-check steps + prompts | Baseline, 2 spot-checks, target power hold |
| `sterilize` | Multi-phase recipe | Validate, fill, hold, vent (future) |

### Sensors
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sensors_get` | none | `pot=<ok\|err>,lab_o3=<ok\|err>,thermo=<ok\|err>,room_o3=<val>,vessel_temp=<val>` | |
| `room_o3_alarm` | none | Alarm status | DFRobot sensor |

### Recording / Backup (SPIFFS)
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `recording_start` | `[seq_name]` | Recording started | Optional sequence name |
| `recording_stop` | none | Recording stopped | |
| `recording_status` | none | Status string | |
| `backup_list` | none | File list | |
| `backup_delete` | `<filename>` | Deleted | |

### 106-H Monitor
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `106h_status` | none | Connection/config status | |
| `106h_avg_set` | `<option>` (1-5) | Averaging mode set | |
| `106h_avg_get` | none | Current averaging mode | |
| `106h_log_start` | none | Raw logging started | |
| `106h_log_stop` | none | Raw logging stopped | |

### System
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `version` | none | Firmware version | |
| `status` | none | Full system status | |
| `i2c_scan` | none | Detected I2C addresses | |
| `time_sync` | `<pc_timestamp_ms>` | `synced,esp=<ms>,pc=<ms>` | Sent on connect |

---

## Relay Names

| Name | Controls | Notes |
|------|----------|-------|
| `ozone_gen` | MP-8000 generator SSR | Main O3 production |
| `o2_conc` | O2 concentrator SSR | Feed gas source |
| `air_comp` | Air compressor SSR | Adds ~10 LPM @ ~21% O2.  **Internal to MP-8000** — requires `ozone_gen` ON |

SSRs are **active-high** (Kerwinn KG1-1DA25).

**Hardware Interlock** `[IMPLEMENTED]`: The air compressor is physically
inside the MP-8000 ozone generator.  Its relay circuit only has power when
`ozone_gen` is ON.  Firmware enforces this in `relay_set_with_source()`:
- `air_comp ON` with `ozone_gen OFF` → rejected (`ESP_ERR_INVALID_STATE`)
- `ozone_gen OFF` → auto-sets `air_comp OFF` (circuit would lose power anyway)

---

## Shared Constants

These constants must match between ESP32 firmware and PC dashboard:

| Constant | Value | Used by |
|----------|-------|---------|
| `O3_MASS_FLOW_K` | 0.3327 | mg/s = %vol × LPM × K.  K = M_O3 / (V_m × 60 × 100), V_m=24.04 L/mol at 20°C |
| `AIR_COMP_LPM` | 10.0 | LPM added when air compressor on |
| `POWER_MODEL_A` | 1.78 | O3_max = A/F + B (to be replaced by fitted model) |
| `POWER_MODEL_B` | 1.40 | O3_max = A/F + B |
| `DEFAULT_FLOW_LPM` | 4.0 | Default O2 flow rate |
| TCP Port | 5000 | Default LAN connection port |

---

## Sequence Integration Guide  `[IMPLEMENTED]`

> Updated: 2026-02-26.  Reflects the recipe-based executor architecture.
> ESP32: generic `seq_executor` + `sequence_runner` lockout `[IMPLEMENTED]`.
> Dashboard: generates recipes, parses `SEQ,<type>,SAMPLE,...` data `[IMPLEMENTED]`.

### Architecture: "ESP32 = Arms, PC = Brains"

**Calibration** uses single-command protocol (ESP32 has sweep on-board):
```
Dashboard (PC)                     ESP32
──────────────                     ─────
CMD,calibrate,flow=4.0             →   seq_executor_load_calibration(4.0)
                                       seq_executor_run()
                                    ← RSP,OK,calibrate,running,steps=203,flow=4.0
                                    ← SEQ,calibrate,RELAY,...
                                    ← SEQ,calibrate,STARTED,steps=203,flow=4.0
                                    ← SEQ,calibrate,STEP,0,0,baseline,0
                                    ← SEQ,calibrate,SAMPLE,...  (per 106-H sample)
                                    ...
                                    ← SEQ,calibrate,COMPLETE,<elapsed_s>

CMD,sequence_abort[,reason]        →   seq_executor_abort()  (at any time)
                                    ← SEQ,calibrate,ABORTED,<reason>
```

**Other sequences** (validate, sterilize) use recipe protocol:
```
Dashboard (PC)                     ESP32
──────────────                     ─────
CMD,sequence_start,validate,power=75,flow=4.0,relay_o2=1,relay_o3=1,relay_air=0
                                   →   seq_executor_begin()
                                    ← RSP,OK,sequence_start,type=validate,status=loading
CMD,seq_step,0,0,15,baseline       →   seq_executor_add_step()
  ... (repeat for all steps)
CMD,seq_prompt,0,check_flow,...    →   seq_executor_add_prompt()
CMD,seq_run                        →   seq_executor_run()
                                    ← RSP,OK,seq_run,running
                                    ← SEQ,validate,STARTED,...
                                    ← SEQ,validate,SAMPLE,...
                                    ...
                                    ← SEQ,validate,COMPLETE,<elapsed_s>
```

**Key principles**:
1. **Fixed sequences live on ESP32** — calibration sweep is physics-dictated, doesn't change
2. **Custom sequences use recipe protocol** — validation, sterilization (PC defines steps)
3. **ESP32 does NO analysis** — no mean/std, no model queries, no pass/fail
4. **PC does ALL analysis** — collects `SAMPLE` data, fits models, generates certificates
5. **Sample-counted holds** — each step holds for N 106-H samples (~2.5s each)
6. **Control lockout** — `sequence_runner_is_active()` prevents manual power/relay changes

### Responsibility Matrix

| Responsibility | Owner | Notes |
|---|---|---|
| Calibration sweep pattern | **ESP32** | Built-in 203-step sweep via `seq_executor_load_calibration()` |
| Validation/custom recipes | **PC** | Baseline + spots + target (recipe protocol) |
| Hardware control during sequence | **ESP32** | `o3_power_set()` + `relay_set_with_source(RELAY_AIR_COMP,...)` per step |
| Sample-counted hold timing | **ESP32** | Via `seq_sensor_get_sample_count()` from 106-H |
| Per-sample data streaming | **ESP32** | `SEQ,<type>,SAMPLE,...` per 106-H reading |
| Prompt mechanism (send/block/unblock) | **ESP32** | `SEQ,<type>,PROMPT,...` → blocks until `sequence_confirm` |
| Recipe design (what steps, what power, what hold) | **PC** | CalibrationRunner, ValidationRunner |
| Data storage (CSV, JSON) | **PC** | Collects SAMPLE lines into pandas DataFrame |
| Statistical analysis (mean, std, CV) | **PC** | Computes from collected samples |
| Model fitting & prediction | **PC** | Uses calibration data to fit Power→O3 models |
| Pass/fail determination | **PC** | Applies criteria to validation results |
| Certificate generation & expiry | **PC** | JSON certificates with 24h validity |
| Control lockout during sequence | **PC** | Disable power slider, relay toggles when executor active |
| E-STOP (always active) | **PC** | Sends `sequence_abort` + `power_set,0` + `relay_set,ozone_gen,0` |

### Dashboard: Required Message Handling

| Message prefix | When | Dashboard action |
|---|---|---|
| `SEQ,<type>,STARTED,...` | Recipe execution begins | Enter observer mode, show progress banner |
| `SEQ,<type>,STEP,...` | Step transition | Update phase label, power display |
| `SEQ,<type>,SAMPLE,...` | Each 106-H sample (~2.5s) | Append to data buffer, update live chart |
| `SEQ,<type>,PROMPT,...` | Prompt before step | Show modal dialog, wait for user, send `sequence_confirm` |
| `SEQ,<type>,COMPLETE,...` | Sequence finished OK | Run analysis, show results, save data |
| `SEQ,<type>,ABORTED,...` | Sequence aborted | Clear banner, re-enable controls |
| `STATE,...` | TCP connect/reconnect | Sync relay states + power display |

### Dashboard: Recipe Generation Examples

**Calibration** — single command, no recipe needed:
```python
resp = await tcp.send_command(f"calibrate,flow={flow}", timeout=5.0)
# ESP32 builds the 203-step sweep internally and starts immediately.
# Dashboard just observes SEQ,calibrate,SAMPLE,... messages.
```

**Validation recipe** (PC generates ~5 steps + 2 prompts, air OFF):
```python
steps = [
    (0, 0,       15, "baseline",  0),   # 0% for ~37s
    (1, spot1,    5, "spot_low",  0),   # ~33% for ~12s
    (2, spot2,    5, "spot_high", 0),   # ~66% for ~12s
    (3, target,  15, "target",    0),   # Target power for ~37s
    (4, 0,        5, "cooldown",  0),   # 0% for ~12s
]
prompts = [
    (0, "check_flow", "Verify O2 flow matches rotameter"),
    (1, "check_route", "Confirm gas route to 106-H sensor"),
]
```

### Validation Pass/Fail Criteria (PC-side analysis)

| Check | Criterion | Action on fail |
|---|---|---|
| Airflow equality | Within 5% of target LPM | Fail — "Check rotameter" |
| Model exists | Valid un-expired calibration model | Fail — "Run calibration first" |
| Baseline | Mean O3 < 0.02 %vol | Fail — "Residual O3 detected" |
| Spot correlation | Each spot within 0.15 %vol or 15% relative of model | Fail — "Model inaccurate" |
| Target accuracy | Mean O3 within 10% relative of model prediction | Fail — "Concentration off" |
| Target stability | CV < 5% | Fail — "Unstable output" |

Certificate valid for **24 hours**.  Filename:
`YYYY-MM-DD_HHMMSS_{flow}LPM_{power}pwr_Validation.json`

### Relay Prerequisites for Sequences  `[IMPLEMENTED]`

The executor **owns relay lifecycle** during sequences.  The PC specifies
desired relay states as params in `sequence_start`, and the executor
applies them automatically when `seq_run` is received — before any steps
execute.

**Relay params** (in `sequence_start`):
- `relay_o2=<0|1>` — O2 concentrator.  Omit = don't change.
- `relay_o3=<0|1>` — Ozone generator.  Omit = don't change.
- `relay_air=<0|1>` — Air compressor.  Omit = don't change.

**Application order** (safe): O2 → O3 → Air (air requires O3 power).
If any relay changed, the executor waits **3 seconds** for stabilization.

**Typical prereqs by sequence type**:

| Sequence | relay_o2 | relay_o3 | relay_air | Rationale |
|----------|----------|----------|-----------|-----------|
| calibrate | 1 | 1 | 0 | Need O2+O3 for production; air per-step |
| validate | 1 | 1 | 0 | Same as calibrate |
| fill | 1 | 1 | 0 | Fill vessel with O3 |
| sterilize | 0 | 0 | 0 | All off — sealed hold |

**Cleanup behaviour**:
- **On completion**: Power set to 0%, air_comp OFF.  O2/O3 left in current
  state (operator decides when to turn off).
- **On abort**: Power set to 0%, air_comp OFF, ozone_gen OFF (safety —
  stops O3 production immediately).  O2 left on.
- Relay source reverts from `RELAY_SRC_SEQUENCE` to `RELAY_SRC_NONE`.

**Hardware interlock**: The air compressor is internal to the MP-8000
generator.  Its relay circuit has no power unless `ozone_gen` is ON.
See "Hardware Interlock" note in Relay Names section.

### Air Compressor During Sequences

The air compressor relay can be controlled **per-step** via the optional
`air_comp` field in `seq_step`.  This enables calibration recipes that
include air-blend conditions (e.g., random power levels alternating
between air ON and air OFF).

- **Pure O2 calibration**: All steps with `air_comp=0` (or omitted)
- **Air-blend calibration**: Steps with `air_comp=1` toggle the relay ON
- **Validation**: Typically all `air_comp=0` for O2-only conditions

**Hardware dependency**: The air compressor is internal to the MP-8000
ozone generator.  Its relay circuit has **no power** unless `ozone_gen`
is ON.  The firmware interlock in `relay_set_with_source()` rejects
`air_comp=ON` when `ozone_gen` is OFF (returns `ESP_ERR_INVALID_STATE`).
Per-step `air_comp` toggling only works when the executor's relay prereqs
have already set `ozone_gen=ON`.

The executor toggles `RELAY_AIR_COMP` at step transitions when the
`air_comp` value changes, with a 1-second stabilization delay after
each toggle.  On sequence completion or abort, the air compressor is
always set to OFF.

The `STEP` and `SAMPLE` messages include the `air_comp` state so the
PC can track which condition each data point belongs to.

### Dashboard: Sequence Observer Mode

When executor state is `running` or `waiting_confirm`:

```
┌──────────────────────────────────────────────────────────────┐
│  ⚠ CALIBRATE — Step 45/203 — sweep_up 22%  ████░░░░  ABORT │
└──────────────────────────────────────────────────────────────┘
```

- **Amber banner** at top of page
- **Power controls disabled**: slider, preset buttons, linked inputs `.props("disable")`
- **Relay controls disabled**: toggle buttons `.props("disable")`
- **E-STOP remains active**: sends `sequence_abort` then `power_set,0`
- **Step/phase info** from `SEQ,<type>,STEP,...` messages
- **Live chart** from `SEQ,<type>,SAMPLE,...` data

### Dashboard: Prompt UI Design

When the Dashboard receives `SEQ,<type>,PROMPT,<prompt_id>,<text>`:

1. **Show a modal dialog** (blocks other interaction except E-STOP and abort)
2. **Map prompt ID to rich content** if available; fall back to `<text>` from ESP32
3. **Provide Confirm button** + optional numeric input
4. **Send** `CMD,sequence_confirm` when operator confirms
5. **Show "Executing..." spinner** after confirming

---

## Gas Path Topology  `[IMPLEMENTED]`

```
O2 Concentrator (95% O2) ──┐
                            ├──→ MP-8000 Generator ──→ Vessel ──→ 106-H Sensor (outlet)
Air Compressor (~21% O2) ──┘                                         │
                                                                     ↓
                                                              Exhaust / charcoal filter
```

- **106-H is on the outlet**: Measures O3 leaving the vessel, not entering.
- **L-valve (manual)**: Switchable between vessel-through and direct-to-sensor paths.
  Direct path bypasses the vessel for validation (inlet ≈ outlet when no vessel).
- **Mass balance**: `mg_absorbed = mg_produced(model) - mg_measured(106-H) - mg_decayed(Arrhenius)`
- Vessel: ~11.3L modified pressure tank, ~60% fill with substrate → ~4L residual gas volume.
  Actual gas volume determined by fill/evac CSTR calibration.

---

## Fill / Evacuation Calibration  `[IMPLEMENTED]`

PC-driven sequence for determining system gas volume via CSTR model fitting.
Unlike calibration/validation (ESP32-driven), the PC orchestrates this
directly using `power_set` and `relay_set` commands, monitoring DATA telemetry.

### Protocol

```
PC                                  ESP32
──                                  ─────
cmd_set_power(0)                 →  RSP,OK,power_set,...
cmd_set_relay("ozone_gen", 1)    →  RSP,OK,relay_set,...
cmd_set_relay("o2_conc", 1)      →  RSP,OK,relay_set,...
cmd_set_relay("air_comp", 0|1)   →  RSP,OK,relay_set,...
  (15 baseline samples at 0%)
cmd_set_power(100)               →  RSP,OK,power_set,...
  (monitor DATA until 5 consecutive samples ≥ 95% of target O3)
  → Save fill CSV
  (optional: toggle air_comp for evac)
cmd_set_power(0)                 →  RSP,OK,power_set,...
  (monitor DATA until 5 consecutive samples < 0.01% vol)
  → Save evac CSV
  → Safe standby (power=0, all relays off)
```

### CSTR Model

- **Fill**: `C_out(t) = C_in × (1 - exp(-(t - t_d)/τ))`
- **Evac**: `C_out(t) = C_ss × exp(-(t - t_d)/τ)`
- `τ` = time constant (seconds), `t_d` = transport delay
- `V_system = mean(τ_fill, τ_evac) × F` where F = flow in L/s
- Fitted via `scipy.optimize.curve_fit`, stored in `Models/Fill/` as JSON

### File Naming

- Fill: `Data/Fill/{YYYY-MM-DD}_{HHMMSS}_Fill_{LPM}Lpm[_air].csv`
- Evac: `Data/Evac/{YYYY-MM-DD}_{HHMMSS}_Evac_{LPM}Lpm[_air].csv`
- Model: `Models/Fill/fill_evac_{LPM}lpm_{O2}o2.json`

---

## Dosimetry Strategy  `[DECIDED]`

### Mass Balance (steady-state)

```
mg_produced   = C_model(power) × F × K × Δt
mg_evacuated  = C_106H(outlet) × F × K × Δt
mg_decayed    = Arrhenius(T, RH) × V_gas × C_vessel × Δt   (< 1% of throughput)
mg_absorbed   = mg_produced - mg_evacuated - mg_decayed
```

Where K = 0.3327 mg·min / (%vol·L) at 20°C.

### Scope

- **Dosimetry computation**: PC-only (dashboard aggregates batch data)
- **ESP32 dosimetry.c**: Provides per-sample decay estimates and accumulator,
  but batch-level calculations done on PC with validated model
- **Transients**: CSTR model handles fill/evac phases; steady-state mass
  balance for production phase

### Sterilization Batch (future)

Three-phase batch: Fill → Hold (sterilize) → Evac.
- Fill: 100% power, monitor until steady-state
- Hold: timed at set power, accumulate dose
- Evac: 0% power, purge until safe
- Total dose = Σ(mg_absorbed) across all phases
- Implementation deferred to future session
