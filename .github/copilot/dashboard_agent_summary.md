# Dashboard Agent Summary

> Last updated: 2026-02-27 (Session 6 — Relay prereqs + banner + air_comp)

## Current State

The PC dashboard has been updated to use the **recipe-based sequence protocol**.
The PC now generates complete recipes (step lists, prompts, hold durations) and
sends them to the ESP32, which executes blindly.  The PC does ALL analysis.

| Property | Value |
|----------|-------|
| File | `Interfaces/PC/blocksi_dashboard.py` (~2060 lines) |
| Framework | NiceGUI 3.8.0 (Quasar UI components, WebSocket push) |
| Python | 3.14 in `.venv` |
| Dependencies | nicegui, numpy, pandas, plotly |
| Run command | `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py [--port 5000]` |
| UI port | http://localhost:8080 |
| Backup | `Interfaces/PC/Old/blocksi_dashboard_pre_observer.py` (pre-observer rewrite) |

## Architecture: "ESP32 = Arms, PC = Brains"  `[IMPLEMENTED]`

```
Recipe protocol:
  PC → CMD,sequence_start,<type>,<params>
  PC → CMD,seq_step,<idx>,<pwr>,<hold>,<phase>  (× N steps)
  PC → CMD,seq_prompt,<before>,<id>,<text>       (× M prompts)
  PC → CMD,seq_run
  ESP32 → SEQ,<type>,STARTED,steps=N,flow=X
  ESP32 → SEQ,<type>,STEP,<idx>,<power>,<phase>      (per step)
  ESP32 → SEQ,<type>,SAMPLE,<step>,<num>,<o3>,<temp>,<pwr>  (per 106-H sample ~2.5s)
  ESP32 → SEQ,<type>,PROMPT,<id>,<text>               (operator interaction)
  ESP32 → SEQ,<type>,COMPLETE,<elapsed_s>              (or ABORTED,<reason>)
```

**Key responsibilities**:
- **PC generates all recipes** — step lists, random values, hold durations
- **PC does ALL analysis** — statistics, pass/fail, model fitting, CSV saving
- **ESP32 executes blindly** — set power, count 106-H samples, stream data
- **ESP32 does NO analysis** — no mean/std, no model queries, no pass/fail

## Recipe Generators  `[IMPLEMENTED]`

### Calibration (`generate_cal_recipe`)
- ~218 steps: 1 baseline (15 samples) + 101 sweep up (2 each) + 101 sweep down (2 each) + 15 random (5 each)
- 1 prompt: `check_flow` (before step 0)
- Step tuples now include `air_comp` field (5th element, always 0 for pure-O2 cal)
- Air compressor must be OFF

### Validation (`generate_val_recipe`)
- 5 steps: baseline, spot_low (~33%), spot_high (~66%), target, cooldown
- 2 prompts: `check_flow` (before step 0), `check_route` (before step 1)
- PC-side analysis: `_analyze_validation()` computes mean/std/CV, baseline check, spot correlation, target accuracy, pass/fail

### Validation Pass/Fail Criteria (PC-computed)
| Check | Criterion |
|-------|-----------|
| Baseline | Mean O3 < 0.02 %vol |
| Spot correlation | Within 0.15 %vol or 15% relative of model |
| Target accuracy | Mean within 10% relative of prediction |
| Target stability | CV < 5% |

## TCP Message Handlers (4 dispatch routes)  `[IMPLEMENTED]`

| Message | Handler | Dashboard Action |
|---------|---------|------------------|
| `DATA,...` | `_dispatch` | Parse 17-field telemetry → state → data_buf → CSV |
| `RSP,...` | `_handle_rsp` | Time sync, relay get, response queue |
| `STATE,...` | `_handle_state` | Reconnect sync — relay/power/flow state |
| `SEQ,...` | `_handle_seq` | Routes by action: STARTED, RELAY, STATUS, STEP, SAMPLE, PROMPT, COMPLETE, ABORTED |

### SEQ action handling:
| Action | Dashboard response |
|--------|-------------------|
| `STARTED` | Enter sequence mode, set step total, notify |
| `RELAY` | Update relay state (`S.relay_*`), set phase to `relay_setup` |
| `STATUS` | Handle `relay_stabilizing` → set phase to `stabilizing` |
| `STEP` | Update phase, power, progress, parse optional `air_comp` field |
| `SAMPLE` | Append to cal_samples or val_samples (includes `air_comp`) |
| `PROMPT` | Set pending_prompt_id/text → tick opens modal dialog |
| `COMPLETE` | Run post-analysis (cal: save CSV, val: compute stats), clear sequence state |
| `ABORTED` | Clear sequence state, notify |

## SystemState Key Fields  `[IMPLEMENTED]`

```python
# Sequence observer (recipe protocol)
sequence_active: bool     # True during any sequence
seq_type: str             # "calibrate", "validate", etc.
seq_phase: str            # Current step's phase label
seq_progress: float       # 0-100 (step_idx / step_total * 100)
seq_elapsed: float        # Seconds
seq_power: float          # Power target at current step
seq_step_idx: int         # Current step index
seq_step_total: int       # Total steps in recipe

# Prompt (interactive sequences)
pending_prompt_id: str    # e.g., "check_flow", "check_route"
pending_prompt_text: str  # ESP32's fallback text

# Calibration observer
cal_samples: list[dict]   # From SEQ,calibrate,SAMPLE,...
cal_lpm: float            # Flow from recipe params
cal_file: str             # Saved filename after COMPLETE

# Validation observer
val_power: float          # Target power from recipe
val_lpm: float            # Flow from recipe params
val_samples: list[dict]   # From SEQ,validate,SAMPLE,...
val_result: dict          # PC-computed: mean_o3, std_o3, deviation_pct, cv_pct, passed, etc.
```

## Command Helpers  `[IMPLEMENTED]`

| Function | Sends | Notes |
|----------|-------|-------|
| `cmd_sequence_start(type, **kwargs)` | Full recipe: start (with relay prereqs) → steps → prompts → run | Belt-and-suspenders: pre-enables relays + passes `relay_o2=1,relay_o3=1,relay_air=0` in params |
| `cmd_sequence_abort([reason])` | `CMD,sequence_abort[,reason]` | Primary abort command |
| `cmd_sequence_stop()` | `CMD,sequence_abort` | Legacy alias |
| `cmd_sequence_confirm()` | `CMD,sequence_confirm` | No prompt_id arg (ESP32 unblocks pending) |
| `cmd_emergency_stop()` | abort + power_set,0 + relay_set,ozone_gen,0 | Always active |

## UI Layout (4 tabs)  `[IMPLEMENTED]`

### Sidebar (Left Drawer, 240px)
- Connection badge (green steady / red blink)
- Relay toggle buttons (Air / O2 / O3) — locked during sequences
- O2 LPM input — locked during sequences
- 5 sensor cards: Flow LPM, Vessel O3, Room O3, Vessel Temp, Cell Temp

### Sequence Banner (top of page, hidden when no sequence)
- Amber bar with phase-aware text:
  - `loading`: "CALIBRATE — Loading recipe... (218 steps)"
  - `relay_setup`: "CALIBRATE — Enabling relays..."
  - `stabilizing`: "CALIBRATE — Equipment stabilizing... (~3s warm-up)"
  - `started`: "CALIBRATE — Starting..."
  - Normal phases: "CALIBRATE — Step 45/218 — Sweep Up"
- Progress bar, elapsed time, ABORT button

### Prompt Dialog (modal)
- Maps prompt IDs to rich content: `check_flow`, `check_route`, plus legacy `prompt_vessel`, `prompt_direct`
- Confirm → `CMD,sequence_confirm`, Abort → `CMD,sequence_abort`

### Power Tab (3 expansions)
1. **Power Control**: Slider, presets, curve (Plotly), linked settings, E-STOP
2. **Calibration**: LPM input, Start/Stop, 4 phase cards (Baseline, Sweep Up, Sweep Down, Random), progress, ECharts scatter (by phase: Sweep Up/Down/Random), file browser
3. **Validation**: Power%/LPM inputs, Validate button, pass/fail result card (green passed / amber marginal / red failed with deviation + CV), ECharts O3 line chart

### Telemetry Tab
- ECharts: O3+Room, Power+Temp (with dataZoom), raw data table, CSV export

### Debug Tab
- Manual command input, system state JSON dump, colored log (9 categories)

### Settings Tab
- Notification level selector

## What Changed This Session

### Removed (superseded by recipe protocol)
- **Old message handlers**: `_handle_seq_done`, `_handle_cal_start`, `_handle_cal_data`, `_handle_cal_complete`, `_handle_val_start`, `_handle_val_data`, `_handle_val_result`
- **Old dispatch routes**: `SEQ_DONE`, `CAL_START`, `CAL_DATA`, `CAL_COMPLETE`, `VAL_START`, `VAL_DATA`, `VAL_RESULT`
- **Old SystemState field**: `seq_air` (air compressor OFF during all sequences)
- **Old sequence type**: `"cal"` → renamed to `"calibrate"`

### Added
- `import random` for recipe generation
- `generate_cal_recipe()`: Creates ~218 calibration steps
- `generate_val_recipe()`: Creates 5 validation steps + 2 prompts
- `_analyze_validation()`: Full PC-side pass/fail analysis with 4 criteria
- `cmd_sequence_start()`: Sends complete recipe (start → steps × N → prompts × M → run)
- `cmd_sequence_abort()`: Uses `sequence_abort` command instead of `sequence_stop`
- `cmd_sequence_confirm()`: No prompt_id parameter (ESP32 unblocks pending)
- New PROMPT_CONTENT entries: `check_flow`, `check_route`
- SystemState: `seq_step_idx`, `seq_step_total` for step-based progress tracking
- Cal chart: Phase-based series (Sweep Up / Sweep Down / Random) instead of Air ON/OFF
- Val result: Shows pass/fail status + CV%

## Pending Work

- `[PROPOSED]` **Power model fitting UI**: After calibration CSV generated, fit piecewise model, store in `Models/O3Power/`, update constants.
- `[PROPOSED]` **Historical data viewer**: Load and plot old CSV files from `Data/Telemetry/`.
- `[PROPOSED]` **Future sequence types**: `fill`, `decay`, `sterilize` — recipe generators to be added.
- `[PROPOSED]` **Validation certificate**: Generate JSON certificate with 24h validity on pass.
- `[PROPOSED]` **Migrate power curve to ECharts**: Last remaining Plotly chart.

## Data Management  `[IMPLEMENTED]`

### Folder Structure
```
Data/
  Telemetry/       — Daily stream CSVs (was Stream/)
  Calibration/     — Power-O3 calibration CSVs (was O3PowerCalibration/)
  Validation/      — Validation run CSVs (new)
  Fill/            — Future
  Decay/           — Future
  Sterilization/   — Future
Models/            — Git-tracked fitted models
  O3Power/         — Power→O3 models (was Model/O3Power/)
  Fill/            — Future
  Decay/           — Future
```

### File Naming Convention
- Calibration: `{YYYY-MM-DD}_{HHMMSS}_PowerO3Cal_{LPM}Lpm_{O2}O2.csv`
- Telemetry: `{YYYY-MM-DD}_Stream.csv` (daily, appended)
- O2% = weighted average: `(F_conc × 95 + F_air × 21) / (F_conc + F_air)`, rounded to int

### O2% Calculation
```python
O2_CONC_PCT = 95       # O2 concentrator purity
AIR_COMP_O2_PCT = 21   # Atmospheric O2
AIR_COMP_LPM = 10.0    # Air compressor flow
compute_effective_o2_pct(flow_lpm, air_comp_on) → int
```

## Known Caveats

1. **Venv required**: Must use `.venv\Scripts\python.exe`, not system Python
2. **NiceGUI**: `element.visible` toggles sequence banner, skeleton, result card
3. **Quasar disable pattern**: `.props("disable")` / `.props(remove="disable")`
4. **Recipe sending speed**: Steps are sent via raw TCP (fire-and-forget) with 5ms yields to avoid buffer overflow. `seq_run` response validates all steps loaded.
5. **ECharts dark mode**: All ECharts have `darkMode: True`
6. **Plotly retained**: Power curve still uses `ui.plotly` (single Plotly chart remaining)
7. **Air compressor check**: `cmd_sequence_start` validates `air_comp=0` before sending recipe
