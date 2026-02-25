# Dashboard Agent Summary

> Last updated: 2026-02-25

## Current State

The PC dashboard has been **fully migrated from Streamlit to NiceGUI**.

| Property | Value |
|----------|-------|
| File | `Interfaces/PC/blocksi_dashboard.py` (~1570 lines) |
| Framework | NiceGUI 3.8.0 (Quasar UI components, WebSocket push) |
| Python | 3.14 in `.venv` |
| Dependencies | nicegui, numpy, pandas, plotly |
| Run command | `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py [--port 5000]` |
| UI port | http://localhost:8080 |
| Reference | `blocksi_dashboard_v9.py` (Streamlit, kept as-is) |

## Code Structure

| Section | Lines (approx) | Purpose |
|---------|--------|---------|
| Constants + conversions | 1-130 | Power model, O3 math, flow constants |
| `SystemState` class | 130-190 | Singleton: power, relays, sensors, sequence, calibration |
| `parse_data_line()` | 196-240 | Parse 17-field DATA CSV from ESP32 |
| `apply_telemetry()` | 243-268 | Update SystemState from DATA (never touches `power_target`) |
| `TCPServer` class | 270-400 | asyncio server on port 5000, handles connect/DATA/RSP |
| Command helpers | 406-450 | `cmd_set_power()`, `cmd_set_relay()`, `cmd_sync_relays()`, `cmd_emergency_stop()` |
| `_CSVLogger` class | 453-487 | Auto-logging to `Data/YYYYMMDD_HHMMSS_Stream.csv` |
| `CalibrationRunner` class | 489-660 | PC-driven calibration state machine (4 phases) |
| `index()` page | 696-1550 | Full UI: drawer, header, sequence banner, 4 tab panels |
| `_tick()` timer | ~1290-1550 | 1s periodic refresh: sidebar, banner, charts, debug |

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

# Relays, sensors, connection, timing -- see code for full list
```

## CalibrationRunner Phases (~17 min total)

1. **baseline**: 30s at 0% power, air OFF
2. **sweep_up**: 0->100% in 1% steps, 2s each, air OFF
3. **sweep_down**: 100->0% in 1% steps, 2s each, air OFF
4. **random_pairs**: 15 random power levels x (20s air OFF + 20s air ON)

Output: `Data/O3PowerCalibration/YYYY-MM-DD_PowerO3Cal_{LPM}Lpm.csv`

## UI Layout

- **Left Drawer (sidebar)**: Connection icon, relay toggle buttons (Air/O2/O3), O2 LPM input, sensor readings
- **Header**: Menu toggle + title
- **Sequence Banner**: Amber bar (hidden when no sequence running) -- sequence name, phase label, progress bar, ABORT button
- **Power Tab**: Slider 0-100%, 11 preset buttons (0-100 step 10), Power-O3 curve (Plotly), linked settings boxes (LPM, %Power, %vol O3, mg/s, g@30min), E-STOP
- **Telemetry Tab**: Dual-axis time-series (O3+Room, Power+Temp), metrics row, raw data table
- **Calibration Tab**: Start/Stop buttons, phase label, progress bar, info text, live scatter plot, file browser
- **Debug Tab**: Manual command input, response label, system state dump, scrolling log

## Pending Work (Dashboard-Specific)

- `[DECIDED]` **ESP32-owned sequences**: Refactor CalibrationRunner to observer mode -- PC sends `CMD,sequence_start,cal,...`, ESP32 runs autonomously, PC shows progress from `SEQ,...` lines. Currently CalibrationRunner does it all PC-side.
- `[PROPOSED]` **Power model fitting UI**: After calibration CSV generated, fit piecewise model, store in `Model/O3Power/`, update constants.
- `[PROPOSED]` **Telemetry tab enhancements**: CSV export button, date-range filtering, relay state annotations on charts.
- `[PROPOSED]` **Historical data viewer**: Load and plot old CSV files from `Data/`.

## Known Caveats

1. **Venv required**: Must use `.venv\Scripts\python.exe`, not system Python
2. **NiceGUI visibility**: `element.visible = True/False` toggles sequence banner
3. **Quasar disable pattern**: `.props("disable")` / `.props(remove="disable")`
4. **CalibrationRunner bypasses `cmd_set_power()`**: Sends `tcp.send_command("power_set,N")` directly and manages `S.power_target_pct` itself in `step()`
5. **Streamlit pitfalls (items 2-5 in copilot-instructions.md)**: No longer apply to NiceGUI but kept in instructions for historical reference
