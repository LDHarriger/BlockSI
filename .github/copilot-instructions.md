# BlockSI Project - AI Coding Agent Instructions

## Project Overview

BlockSI is an ozone sterilization monitoring and control system for Shroom-E Co. It comprises:
- **ESP32 Firmware** (`Interfaces/ControlSystem/`): ESP-IDF v5.4+ project controlling ozone generation, monitoring sensors, and streaming data
- **PC Dashboard** (`Interfaces/PC/`): Being migrated from Streamlit (v9) to **NiceGUI**. The Streamlit version (`blocksi_dashboard_v9.py`) is the last working version but has fundamental limitations.
- **Data** (`Data/`): CSV telemetry and sequence logs, organised by type:
  - `Data/Telemetry/` — Per-connection stream CSVs
  - `Data/Calibration/` — Power-O3 characterisation CSVs
  - `Data/Validation/` — Validation run CSVs (filename suffix `_PASS` or `_FAIL`)
  - `Data/CSTR/` — Combined fill+evac calibration CSVs (single CSV per run, `phase` column)
  - `Data/Sterilization/` — Future sequence type
- **Models** (`Models/`): Fitted prediction models (git-tracked):
  - `Models/O3Power/` — Power→O3 sigmoid models (JSON)
  - `Models/CSTR/` — Decay-aware CSTR model (`cstr_model.json`, flow-rate independent)

## Collaboration Protocol

This project uses **domain-separated AI coding agents** with shared
documentation.  All collaboration docs live in `.github/copilot/`:

| File | Purpose |
|------|---------|
| `collaboration_protocol.md` | Rules: agent responsibilities, file ownership, update conventions |
| `interface_contract.md` | **Single source of truth** for LAN commands, DATA format, shared constants |
| `dashboard_agent_summary.md` | PC dashboard agent's current state and pending work |
| `esp32_agent_summary.md` | ESP32 firmware agent's current state and pending work |
| `decisions_log.md` | Architectural decisions with date, context, rationale |

**Key rules**:
- Status tags: `[IMPLEMENTED]`, `[DECIDED]`, `[PROPOSED]` on all items
- **Interface changes**: update `interface_contract.md` FIRST, then your summary
- **Summary updates**: rewrite your summary when prompted (context window ~85%)
- **File ownership**: Dashboard agent writes to `Interfaces/PC/`, ESP32 agent writes to `Interfaces/ControlSystem/`
- **New chat startup**: read `copilot-instructions.md` -> `collaboration_protocol.md` -> `interface_contract.md` -> your agent summary
- **End of session**: update your agent summary, then `git add -A && git commit && git push` (see `collaboration_protocol.md` for full procedure)

## Architecture & Data Flow

```
106-H Ozone Monitor (RS232, on vessel OUTLET) → ESP32 (UART2) → Golioth Cloud + LAN TCP → PC Dashboard
        ↓                                          ↓
   O3 wt% readings                          Control: Relays, Motor Pot, Dosimetry
```

Key integration pattern: The ESP32 acts as a bridge between industrial equipment (106-H monitor, MP-8000 generator) and cloud/PC interfaces. The LAN client (`lan_client.c`) uses a comma-separated text command protocol over TCP.

**Gas path**: O2 concentrator (+ optional air compressor) → MP-8000 generator → Vessel → 106-H sensor (outlet). L-valve switchable for direct-to-sensor bypass (validation). Vessel: ~11.3L modified tank, ~60% fill with substrate → ~4L residual gas volume.

## LAN Command Protocol (CRITICAL)

ESP32 ↔ PC uses **comma-separated** commands over TCP port 5000:
```
Format:  CMD,command_name,arg1,arg2\n  →  RSP,OK|ERR,command_name,response_data\n

Examples:
  CMD,relay_set,ozone_gen,1\n     → RSP,OK,relay_set,ozone_gen=on\n
  CMD,power_set,50\n              → RSP,OK,power_set,power=50,predicted_o3=1.23\n
  CMD,relay_get\n                 → RSP,OK,relay_get,ozone_gen=1,o2_conc=0,air_comp=0\n
  CMD,time_sync,1708888888000\n   → RSP,OK,time_sync,synced,esp=12345,pc=1708888888000\n
```

**WARNING**: Commands use COMMAS not colons as separators. Previous versions had bugs using `power_set:50` which silently fails because the ESP32 parses `power_set:50` as the command name.

See command handlers in `main.c` `lan_command_handler()` (~line 350).

### DATA telemetry format (ESP32 → PC):
```
DATA,esp_timestamp_ms,vessel_o3_pct,temp_c,pressure_mbar,sample_v,ref_v,
     day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c,
     power_target_pct,power_actual_pct,wiper_voltage
```

### Relay names (used in relay_set/relay_get):
- `ozone_gen` — O3 generator SSR
- `o2_conc` — O2 concentrator SSR  
- `air_comp` — Air compressor SSR (+~10 LPM air flow)

## ESP32 Firmware Conventions

### Pin Assignments
All GPIO pins are centralized in `blocksi_pins.h`. **Always check here before adding hardware**:
- I2C: GPIO 21/22 (DFRobot O3 sensor)
- SPI: GPIO 18/19/5 (MAX31855 thermocouple)
- UART2: GPIO 16/17 (106-H RS232 via level shifter)
- Relays: GPIO 12/13/27 (SSR control — 3 relays)
- Motor Pot: GPIO 25/26/34 (DRV8833 + ADC feedback)

### Module Pattern
Each peripheral follows: `module_init()` → `module_deinit()` → `module_is_initialized()`

### Build & Flash
**DO NOT attempt to build or flash firmware from an AI agent.** The ESP-IDF toolchain requires a specific environment (idf_cmd_init.bat, Python venv, toolchain paths) that cannot be reliably set up from an agent terminal. Instead, describe the changes made and the user will build and flash manually.

Reference commands (for human use only):
```powershell
idf.py build
idf.py -p COM3 flash monitor    # exit: Ctrl+]
idf.py menuconfig               # WiFi, Golioth PSK, pins
```

## PC Dashboard -- Status

### Migration Complete: NiceGUI (`blocksi_dashboard.py`)
The dashboard has been fully migrated from Streamlit to NiceGUI.
See `.github/copilot/dashboard_agent_summary.md` for current state.

- **File**: `Interfaces/PC/blocksi_dashboard.py` (~3450 lines, NiceGUI)
- **Reference**: `blocksi_dashboard_v9.py` (Streamlit, kept for reference only)
- **Run**: `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py`
- **Dependencies**: nicegui, numpy, pandas, plotly, scipy (in `.venv`)

### Key constants:
```python
O3_MASS_FLOW_K = 0.3327   # mg/s per (%vol * LPM), V_m=24.04 at 20°C
AIR_COMP_LPM = 10.0      # Additional LPM when air compressor on
POWER_MODEL_A = 1.78      # O3_max = A/F + B
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0
O2_CONC_PCT = 95          # O2 concentrator purity
AIR_COMP_O2_PCT = 21      # Atmospheric O2
```

### UI layout (from hand-drawn mockup):
- **Sidebar**: Connection status, Air/O2/O3 relay toggles, O2 LPM input, sensor readings
- **Power tab**: Slider (0-100%), graduated preset buttons, Power-O3 curve graph, settings boxes (LPM, %Power, %vol O3, mg O3/s, g O3 @ 30min) — all linked via conversions. Expansion sections: Power Control, Calibration, Validation, Fill/Evac Calibration
- **Telemetry tab**: Dual-axis time-series plots (O3 + Room O3, Power + Temp)
- **Debug tab**: Manual command entry, system state dump
- **Settings tab**: Notification preferences

### Calibration sequence design (single-command, `CMD,calibrate`):
1. **Baseline**: 0% power, 15 samples (~37s) — air state per toggle
2. **Sweep Up**: 0→100% in 1% steps, 2 samples each — air state per toggle
3. **Sweep Down**: 100→0% in 1% steps, 2 samples each — air state per toggle
4. **Random Phase** (optional): N unique stratified levels × 2 visits (ascending + descending), 20 samples each — air state per toggle
- GUI inputs: `O2 LPM` | `# Rnd Lvls` (0-50, default 15) | `Air ON` toggle
- O2 concentrator only activated when LPM > 0; air-only mode (LPM=0, Air ON) valid
- Total steps: 203 + 2×N
- Files: `Data/Calibration/{YYYY-MM-DD}_{HHMMSS}_PowerO3Cal_{LPM}Lpm_{O2}O2.csv`
- O2% = weighted-average feed O2: `(F_conc × 95 + F_air × 21) / (F_conc + F_air)` rounded to int
- CSV columns: timestamp, power_pct, o3_pct, o2_lpm, air_comp_on, total_lpm, o2_concentration_pct, cell_temp_c, phase

## Hardware-Specific Notes

- **Motor Pot replaced DS3502**: Ground isolation issues. PRM162 + DRV8833 driver.
- **106-H quirks**: Fixed 19200 baud, non-standard female D9 pinout, RS232 level shifter required.
- **SSRs are active-high** (Kerwinn KG1-1DA25)
- **Air compressor** adds ~10 LPM at ~21% O2 to the O2 concentrator's output (~93% O2)

## Code Style

- ESP-IDF: `esp_err_t` returns, `ESP_LOG*` macros, FreeRTOS tasks
- Module-level static state: `static struct { ... } s_module = {0};`
- Tag convention: `static const char *TAG = "MODULE_NAME";`

## Known Pitfalls (avoid these)

1. **Command separator**: Use COMMAS (`CMD,power_set,50`) not colons (`CMD,power_set:50`)
2. **Power authority**: PC is sole authority for `power_target_pct` -- never accept it from ESP32 telemetry (see decisions_log.md)
3. **Sequence lockout**: When `sequence_active=True`, power controls must be disabled in the UI
4. **Venv required**: PC dashboard must run from `.venv\Scripts\python.exe`, not system Python
5. **NiceGUI Quasar pattern**: Disable controls with `.props("disable")`, re-enable with `.props(remove="disable")`
6. **Never build/flash firmware from agent**: ESP-IDF toolchain env cannot be set up in agent terminals. Make firmware edits and describe changes — user will build and flash manually.

## Common Tasks

| Task | Key Files |
|------|-----------|
| Add new sensor | Create `sensor.c/.h`, add to `sensor_aggregator.c`, register in `peripherals.c` |
| Add LAN command | Add handler in `main.c:lan_command_handler()` |
| Modify power control | `o3_power_control.c` (API), `motor_pot.c` (hardware) |
| Change cloud telemetry | `publish_to_golioth()` in `main.c` (CBOR encoding) |
