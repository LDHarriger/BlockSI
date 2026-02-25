# Dashboard Agent Summary

> Last updated: 2026-02-25 (Session 3 — GUI aesthetic overhaul)

## Current State

The PC dashboard has been **fully migrated from Streamlit to NiceGUI** and undergone a comprehensive **GUI aesthetic overhaul**.

| Property | Value |
|----------|-------|
| File | `Interfaces/PC/blocksi_dashboard.py` (~1750 lines) |
| Framework | NiceGUI 3.8.0 (Quasar UI components, WebSocket push) |
| Python | 3.14 in `.venv` |
| Dependencies | nicegui, numpy, pandas, plotly |
| Run command | `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py [--port 5000]` |
| UI port | http://localhost:8080 |
| Old versions | Deleted (v4-v7, v9). v8 archived in `Interfaces/PC/Old/` |

## Code Structure

| Section | Lines (approx) | Purpose |
|---------|--------|---------|
| Constants + conversions | 1-130 | Power model, O3 math, flow constants, `STREAM_DIR`, `CAL_DATA_DIR` |
| `SystemState` class | 130-200 | Singleton: power, relays, sensors, sequence, calibration, `notify_level` |
| `parse_data_line()` | ~200-240 | Parse 17-field DATA CSV from ESP32 |
| `apply_telemetry()` | ~243-270 | Update SystemState from DATA (never touches `power_target`) |
| `TCPServer` class | ~280-420 | asyncio server, handles connect/DATA/RSP/**STATE** |
| `_handle_state()` | ~420 | Parses `STATE,...` push from ESP32 on reconnect |
| Command helpers | ~440-490 | `cmd_set_power()`, `cmd_set_relay()`, `cmd_sync_relays()`, `cmd_emergency_stop()` |
| `_CSVLogger` class | ~493-530 | Auto-logging to `Data/Stream/YYYY-MM-DD_Stream.csv` |
| `log()` + `_notify()` | ~530-570 | Categorized logging (tuple: ts, cat, msg) + toast helper |
| `CalibrationRunner` class | ~575-750 | PC-driven calibration state machine (4 phases) |
| `index()` page | ~800-1490 | Full UI: drawer, header, sequence banner, **5** tab panels, CSS |
| `_tick()` timer | ~1500-1750 | 1s periodic refresh: sidebar, banner, **ECharts**, stepper, colored log |

## Recent Changes (Session 3) `[IMPLEMENTED]`

### Infrastructure
- **`STREAM_DIR`**: Telemetry CSVs now saved to `Data/Stream/` (auto-created)
- **STATE parser**: `_handle_state()` processes `STATE,ozone_gen=...,o2_conc=...,air_comp=...,power=...,flow=...` from ESP32 reconnect push
- **Categorized logging**: `debug_log` stores `(timestamp, category, message)` tuples. Categories: `send`, `recv`, `info`, `error`, `warn`
- **Toast notifications**: `_notify()` helper respects `S.notify_level` (`all`/`errors`/`none`)

### UI Overhaul
- **Connection badge**: `ui.badge` with CSS glow (`conn-steady`) when connected, rapid blink animation (`blink-disconnect`) when disconnected — replaces `conn_icon` + `conn_label`
- **Sensor cards**: 5 bordered cards (Flow LPM, Vessel O3, Room O3, Vessel Temp, Cell Temp) with large value + unit labels — replaces `lbl_*` plain text
- **Dark mode toggle**: Button in header bar, uses `dark.toggle()`
- **Enhanced slider**: Thicker track + larger thumb via `power-slider` CSS class
- **ECharts telemetry**: Replaced Plotly `make_subplots` with 2 `ui.echart()` charts (O3+Room, Power+Temp). Features: `dataZoom` (inside + slider), dark mode, streaming updates, area fill
- **Skeleton placeholders**: Two large rect skeletons shown until first data arrives, then hidden
- **CSV export**: Download button above raw data table, exports to `Data/Stream/` 
- **Calibration stepper**: 4 phase cards (Baseline, Sweep Up, Sweep Down, Random Pairs) — completed phases dim with green border, active phase highlighted with blue glow
- **Colored debug log**: HTML-based log with CSS classes per category (`log-send`, `log-recv`, `log-error`, `log-warn`, `log-info`)
- **Settings tab**: New 5th tab with notification level selector (`all`/`errors`/`none`)
- **CSS block**: Custom `<style>` with `blink-disconnect` keyframe, `conn-steady`, `sensor-card*`, `log-*`, `power-slider`, `cal-step-card` styles

## Authority Model  `[IMPLEMENTED]`

```
Manual mode (sequence_active=False):
  PC owns power_target_pct -- telemetry NEVER overwrites it
  User controls slider/inputs freely
  _tick() syncs slider to S.power_target_pct (always in agreement)

Sequence mode (sequence_active=True):
  CalibrationRunner sets S.power_target_pct directly + sends CMD,power_set,N
  Power controls disabled in UI (slider, presets, linked inputs)
  Amber banner shows: sequence name, phase, progress bar, ABORT button
  E-STOP always active (also aborts running sequence)
  Auto-abort if ESP32 disconnects mid-sequence
```

## SystemState Key Fields

```python
# Power
power_target_pct: int       # PC-authoritative, never from telemetry
power_actual_pct: float     # From telemetry
wiper_voltage: float        # From telemetry
power_error: bool           # |target - actual| > 5%

# Sequence tracking
sequence_active: bool       # True when any automated sequence running
sequence_name: str          # e.g., "Power-O3 Calibration"
sequence_description: str

# Calibration (subset of sequence state)
cal_active: bool
cal_phase: str              # baseline | sweep_up | sweep_down | random_pairs
cal_phase_progress: float   # 0-100 within current phase
cal_current_power: int
cal_air_state: bool

# Settings
notify_level: str           # "all" | "errors" | "none"

# Relays, sensors, connection, timing -- see code for full list
```

## CalibrationRunner Phases (~17 min total)

1. **baseline**: 30s at 0% power, air OFF
2. **sweep_up**: 0->100% in 1% steps, 2s each, air OFF
3. **sweep_down**: 100->0% in 1% steps, 2s each, air OFF
4. **random_pairs**: 15 random power levels x (20s air OFF + 20s air ON)

Output: `Data/O3PowerCalibration/YYYY-MM-DD_PowerO3Cal_{LPM}Lpm.csv`

## UI Layout

- **Left Drawer (240px sidebar)**: Connection badge, relay toggle buttons (Air/O₂/O₃), O₂ LPM input, 5 sensor cards
- **Header**: Menu toggle + title + dark mode toggle button
- **Sequence Banner**: Amber bar (hidden when no sequence running) — sequence name, phase label, progress bar, ABORT button
- **Power Tab**: Slider 0-100% (enhanced CSS), 11 preset buttons, Power-O₃ curve (Plotly), linked settings boxes (LPM, %Power, %vol O₃, mg/s, g@30min), E-STOP
- **Telemetry Tab**: Metric labels, skeleton placeholder (until data), 2 ECharts time-series (O₃+Room, Power+Temp) with dataZoom, CSV export button, raw data table
- **Calibration Tab**: 4-phase stepper cards, Start/Stop buttons, phase label, progress bar, info text, live scatter plot (Plotly), file browser
- **Debug Tab**: Manual command input, response label, system state dump, colored HTML log
- **Settings Tab**: Notification level selector

## Pending Work (Dashboard-Specific)

- `[DECIDED]` **ESP32-owned sequences**: Refactor CalibrationRunner to observer mode — PC sends `CMD,sequence_start,cal,...`, ESP32 runs autonomously, PC shows progress from `SEQ,...` lines. Currently CalibrationRunner does it all PC-side.
- `[PROPOSED]` **Power model fitting UI**: After calibration CSV generated, fit piecewise model, store in `Model/O3Power/`, update constants.
- `[PROPOSED]` **Historical data viewer**: Load and plot old CSV files from `Data/`.

## Known Caveats

1. **Venv required**: Must use `.venv\Scripts\python.exe`, not system Python
2. **NiceGUI visibility**: `element.visible = True/False` toggles sequence banner, skeleton
3. **Quasar disable pattern**: `.props("disable")` / `.props(remove="disable")`
4. **CalibrationRunner bypasses `cmd_set_power()`**: Sends `tcp.send_command("power_set,N")` directly and manages `S.power_target_pct` itself in `step()`
5. **ECharts dark mode**: Charts have `darkMode: True` — works with NiceGUI dark mode toggle
6. **Connection badge CSS**: Uses `._classes` list manipulation (not ideal but functional for NiceGUI badge animation toggling)
