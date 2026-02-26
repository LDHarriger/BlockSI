# Dashboard Agent Summary

> Last updated: 2026-02-25 (Session 4 — Observer-mode rewrite)

## Current State

The PC dashboard has been **fully rewritten to observer mode**.  The ESP32 now
owns all sequence execution (calibration, validation).  The dashboard sends
start/stop/confirm commands and observes SEQ/CAL/VAL data streams.

| Property | Value |
|----------|-------|
| File | `Interfaces/PC/blocksi_dashboard.py` (~1530 lines) |
| Framework | NiceGUI 3.8.0 (Quasar UI components, WebSocket push) |
| Python | 3.14 in `.venv` |
| Dependencies | nicegui, numpy, pandas, plotly |
| Run command | `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py [--port 5000]` |
| UI port | http://localhost:8080 |
| Backup | `Interfaces/PC/Old/blocksi_dashboard_pre_observer.py` (pre-rewrite) |
| Old versions | v4-v7 deleted, v8 in `Old/`, v9 deleted |

## Code Structure

| Section | Lines (approx) | Purpose |
|---------|--------|---------|
| Constants + conversions | 1-140 | Power model, O3 math, paths, `PROMPT_CONTENT` mapping |
| `SystemState` class | 140-220 | Singleton: power, relays, sensors, **sequence observer fields**, validation, calibration |
| `parse_data_line()` / `apply_telemetry()` | ~220-280 | Parse 17-field DATA CSV, update state (never touches `power_target`) |
| Logging + buffers | ~280-320 | `debug_log` deque (ts, cat, msg), `_notify()` toast helper, 9 log categories |
| `_CSVLogger` | ~320-360 | Auto-logging to `Data/Stream/YYYY-MM-DD_Stream.csv` |
| Cal file helpers | ~360-400 | `list_calibration_files()`, `_save_cal_csv()` |
| `TCPServer` class | ~400-600 | asyncio server with **12 message handlers** in `_dispatch()` |
| Command helpers | ~600-670 | `cmd_set_power`, `cmd_set_relay`, `cmd_sync_relays`, `cmd_emergency_stop`, **`cmd_sequence_start`**, **`cmd_sequence_stop`**, **`cmd_sequence_confirm`** |
| `index()` page | ~670-1400 | Full UI: drawer, header, **sequence banner**, **prompt dialog**, **4 tab panels**, CSS |
| `_tick()` timer | ~1400-1530 | 1s periodic refresh: sidebar, banner, **control lockout**, **prompt dialog opening**, ECharts, **cal/val observer UI** |

## Architecture: Observer Mode  `[IMPLEMENTED]`

```
Manual mode (sequence_active=False):
  PC owns power_target_pct — telemetry NEVER overwrites it
  User controls slider/inputs/relays freely
  _tick() syncs all UI widgets to SystemState

Sequence mode (sequence_active=True):
  ESP32 owns all execution — PC is read-only observer
  PC sent CMD,sequence_start,<type>,<params> to initiate
  ESP32 streams SEQ,<phase>,<progress>,... every ~1s
  ESP32 streams type-specific data (CAL_DATA, VAL_DATA)
  ESP32 may send SEQ,prompt,<id>,<msg> for operator interaction
  PC shows prompt dialog, operator clicks Confirm
  PC sends CMD,sequence_confirm,<prompt_id>
  ALL power/relay/preset controls locked (disabled via Quasar props)
  E-STOP always active: sends sequence_stop THEN power_set,0
  Auto-abort if ESP32 disconnects mid-sequence
  SEQ_DONE clears sequence_active, re-enables controls
```

## TCP Message Handlers (12 total)  `[IMPLEMENTED]`

| Message | Handler | Dashboard Action |
|---------|---------|------------------|
| `DATA,...` | `_dispatch` | Parse telemetry → state → data_buf → CSV |
| `RSP,...` | `_handle_rsp` | Time sync, relay get, response queue |
| `STATE,...` | `_handle_state` | Reconnect sync — relay/power/flow state |
| `SEQ,...` | `_handle_seq` | Update phase/progress/power/air/elapsed |
| `SEQ,prompt,...` | `_handle_seq` | Set pending_prompt_id/text |
| `SEQ_DONE,...` | `_handle_seq_done` | Clear sequence state, toast, re-enable controls |
| `CAL_START,...` | `_handle_cal_start` | Clear samples, record LPM |
| `CAL_DATA,...` | `_handle_cal_data` | Append to cal_samples (scatter chart) |
| `CAL_COMPLETE,...` | `_handle_cal_complete` | Save CSV, toast summary |
| `VAL_START,...` | `_handle_val_start` | Clear samples, record power/LPM |
| `VAL_DATA,...` | `_handle_val_data` | Append to val_samples (line chart) |
| `VAL_RESULT,...` | `_handle_val_result` | Compute deviation, show pass/fail card, toast |

## SystemState Key Fields  `[IMPLEMENTED]`

```python
# Power — PC is sole authority for target
power_target_pct: int
power_actual_pct: float
wiper_voltage: float
power_error: bool         # |target - actual| > 5%

# Sequence observer (ESP32 owns execution)
sequence_active: bool     # True during any ESP32-driven sequence
seq_type: str             # "cal", "validate", etc.
seq_phase: str            # Current phase name
seq_progress: float       # 0-100
seq_elapsed: float        # Seconds
seq_power: float          # Power ESP32 is commanding
seq_air: bool             # Air compressor state during sequence

# Prompt (interactive sequences)
pending_prompt_id: str    # e.g., "prompt_vessel", "prompt_direct"
pending_prompt_text: str  # ESP32's fallback text

# Calibration observer
cal_samples: list[dict]   # Streamed from CAL_DATA
cal_lpm: float
cal_file: str             # Saved filename after CAL_COMPLETE

# Validation observer
val_power: float
val_lpm: float
val_samples: list[dict]   # Streamed from VAL_DATA
val_result: dict          # From VAL_RESULT (mean_o3, std_o3, deviation_pct, ...)

# Settings
notify_level: str         # "all" | "errors" | "none"

# Relays, sensors, connection, timing — see code for full list
```

## UI Layout (4 tabs)  `[IMPLEMENTED]`

### Sidebar (Left Drawer, 240px)
- Connection badge (green steady / red blink)
- Relay toggle buttons (Air / O2 / O3) — locked during sequences
- O2 LPM input — locked during sequences
- 5 sensor cards: Flow LPM, Vessel O3, Room O3, Vessel Temp, Cell Temp

### Header
- Menu toggle + title + dark mode toggle

### Sequence Banner (top of page, hidden when no sequence)
- Amber bar: sequence name, phase label, progress bar, elapsed time, ABORT button

### Prompt Dialog (modal, shown when ESP32 sends SEQ,prompt,...)
- Persistent dialog with icon, title, rich HTML body
- Maps `prompt_vessel` → "Step 1 — Route to Vessel" with instructions
- Maps `prompt_direct` → "Step 2 — Route Direct to Sensor" with instructions
- Unknown prompt IDs fall back to ESP32's text
- Confirm button → `CMD,sequence_confirm,<id>`
- Abort button → `CMD,sequence_stop`

### Power Tab (3 expansions)
1. **Power Control**: Slider 0-100%, 11 preset buttons, Power-O3 curve (Plotly),
   linked settings boxes (LPM, %Power, %vol O3, mg/s, g@30min), E-STOP
2. **Calibration**: LPM input, Start/Stop buttons, 4-phase stepper cards
   (Baseline, Sweep Up, Sweep Down, Random Pairs) with active highlighting,
   progress bar, info text, live ECharts scatter (Air OFF vs Air ON), file browser
3. **Validation**: Power % + LPM inputs, Validate button, pass/fail result card
   (green <10%, amber 10-20%, red >20%), live ECharts O3 line chart

### Telemetry Tab
- Metric labels (O3, Room O3, Vessel Temp, Cell Temp)
- Skeleton placeholders until first data
- 2 ECharts time-series: O3+Room, Power+Temp (with dataZoom)
- CSV export + raw data table

### Debug Tab
- Manual command input/send
- System state JSON dump (includes all sequence fields)
- Colored HTML log (9 categories: send, recv, error, warn, info, seq, cal, val, state)

### Settings Tab
- Notification level selector (all / errors / none)

## What Was Deleted  `[IMPLEMENTED]`

- **CalibrationRunner class** (~175 lines): PC-driven calibration state machine
  that sent individual `CMD,power_set,N` commands. Replaced by observer mode
  parsing CAL_START/CAL_DATA/CAL_COMPLETE from ESP32 sequence runner.
- **Standalone Calibration tab**: Merged into Power tab as an expansion section.
- **PC-side calibration state fields**: `cal_active`, `cal_phase`, `cal_current_power`,
  `cal_step_start`, `cal_phase_progress`, `cal_air_state`, `cal_data[]`, etc.

## Pending Work (Dashboard-Specific)

- `[PROPOSED]` **Power model fitting UI**: After calibration CSV generated, fit
  piecewise model, store in `Model/O3Power/`, update constants.
- `[PROPOSED]` **Historical data viewer**: Load and plot old CSV files from `Data/`.
- `[PROPOSED]` **Future sequence types**: `fill`, `decay`, `sterilize` — UI shells
  to be added when ESP32 implements them.
- `[PROPOSED]` **Prompt value input**: Add optional numeric input (LPM reading)
  to prompt dialog for `sequence_confirm` value parameter.

## Known Caveats

1. **Venv required**: Must use `.venv\Scripts\python.exe`, not system Python
2. **NiceGUI visibility**: `element.visible` toggles sequence banner, skeleton, result card
3. **Quasar disable pattern**: `.props("disable")` / `.props(remove="disable")`
4. **Connection badge CSS**: Uses `._classes` list manipulation for animation toggling
5. **ECharts dark mode**: All ECharts have `darkMode: True`
6. **Plotly retained**: Power curve still uses `ui.plotly` (single Plotly chart remaining)
7. **Relay lockout**: During sequences, relay buttons + O2 LPM input are disabled alongside power controls
