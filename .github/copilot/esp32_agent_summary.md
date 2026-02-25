# ESP32 Agent Summary

> Last updated: 2026-02-25 (relay robustness + LAN reconnect work)

## Current State

The ESP32 firmware is an ESP-IDF v5.4+ project controlling ozone generation
hardware, monitoring sensors, and bridging to both Golioth Cloud and a
PC dashboard over LAN TCP.

| Property | Value |
|----------|-------|
| Location | `Interfaces/ControlSystem/` |
| Framework | ESP-IDF v5.4+ with FreeRTOS |
| Build | `idf.py build` or VS Code ESP-IDF extension |
| Flash | `idf.py -p COM3 flash monitor` (Ctrl+] to exit) |
| Config | `idf.py menuconfig` (WiFi, Golioth PSK, pins) |

## Firmware Modules

From `main/CMakeLists.txt` SRCS:

| File | Purpose |
|------|---------|
| `main.c` | Entry point, WiFi, Golioth, LAN command handler (32 commands) |
| `lan_client.c` / `.h` | TCP client -- connects to PC on port 5000.  Sends DATA telemetry, receives CMD, sends RSP |
| `relay_control.c` / `.h` | 3x SSR control (ozone_gen, o2_conc, air_comp) -- active-high |
| `motor_pot.c` / `.h` | PRM162 motorized potentiometer driver via DRV8833 H-bridge + ADC feedback |
| `o3_power_control.c` / `.h` | High-level power control API (uses motor_pot) |
| `model_106h_interface.c` / `.h` | 106-H ozone monitor RS232 interface (UART2, 19200 baud) |
| `dfrobot_ozone.c` / `.h` | DFRobot room-O3 sensor (I2C) |
| `max31855_thermocouple.c` / `.h` | MAX31855 thermocouple interface (SPI) |
| `sensor_aggregator.c` / `.h` | Aggregates all sensor readings into one struct |
| `dosimetry.c` / `.h` | O3 dose calculation |
| `backup_storage.c` / `.h` | SPIFFS-based recording and backup |
| `blocksi_state.c` / `.h` | Unified system state management |
| `power_calibration_v2.c` | Motor pot version of power calibration sweep |
| `power_calibration.h` | Calibration API: `_init()`, `_start()`, `_stop()`, `_get_status()` |
| `sequence_runner.c` / `.h` | Generic sequence framework: lifecycle, SEQ streaming, interactive prompts |
| `seq_power_cal.c` / `.h` | Sequence #1: Power-O3 calibration (4-phase, type: `cal`) |
| `seq_airflow_val.c` / `.h` | Sequence #2: Airflow/concentration validation (5-phase, type: `validate`) |
| `blocksi_pins.h` | **All GPIO pin assignments** -- check before adding hardware |

## Pin Assignments (from `blocksi_pins.h`)

| Bus/Function | GPIOs | Connected to |
|-------------|-------|-------------|
| I2C | 21, 22 | DFRobot O3 sensor |
| SPI | 18, 19, 5 | MAX31855 thermocouple |
| UART2 | 16, 17 | 106-H RS232 (via level shifter) |
| Relays | 12, 13, 27 | SSR control (3 relays) |
| Motor Pot | 25, 26, 34 | DRV8833 H-bridge + ADC feedback |

## Module Pattern

Every peripheral follows: `module_init()` -> `module_deinit()` -> `module_is_initialized()`

Module-level static state: `static struct { ... } s_module = {0};`

Tag convention: `static const char *TAG = "MODULE_NAME";`

## LAN Command Handler

Located in `main.c` `lan_command_handler()` starting at ~line 380.
32 commands across 8 categories.  Full specification in
`.github/copilot/interface_contract.md`.

## Power Calibration (`power_calibration_v2.c`)  `[IMPLEMENTED]`

ESP32-side sweep program using motor pot.  API in `power_calibration.h`:

```c
typedef struct {
    bool active;
    int8_t direction;      // 1=ascending, -1=descending
    uint8_t current_pct;
    uint16_t points_up;
    uint16_t points_down;
    float elapsed_s;
} power_cal_status_t;

esp_err_t power_calibration_init(void);
esp_err_t power_calibration_start(void);
esp_err_t power_calibration_stop(void);
bool power_calibration_is_active(void);
esp_err_t power_calibration_get_status(power_cal_status_t *status);
```

Sweeps 0->100% then 100->0% in 1% steps.  Streams `CAL_DATA,...` over LAN.
Triggered by `CMD,calibrate_start` / `CMD,calibrate_stop`.

**Note**: The old `power_calibration.c` (DS3502 digipot version) was deleted
on 2026-02-24.  Only the motor pot version (`_v2.c`) remains.

## Pending Work (ESP32-Specific)

- `[IMPLEMENTED]` **ESP32-owned sequences**: Generic `sequence_runner` framework
  with `seq_power_cal.c` (4-phase calibration) and `seq_airflow_val.c` (5-phase
  interactive validation).  LAN commands: `sequence_start`, `sequence_stop`,
  `sequence_status`, `sequence_confirm`.  SEQ + interactive prompt streaming.
- `[DECIDED]` **Additional sequence types**: 3 more sequences planned:
  1. Fill model calibration (mean transit time, infer vessel volume)
  2. O3 decay testing (fill, seal, wait, measure decay)
  3. Sterilize batch (validate, configure, 15min sterilization, dose-based)
- `[PROPOSED]` **Relay dropout investigation**: Root causes identified (see
  Recent Changes below). Source-tracking now implemented for ongoing monitoring.
  Monitor logs for `[src=...]` tags to catch remaining unexplained dropouts.
- `[IMPLEMENTED]` **LAN reconnect robustness**: Relay/power states survive
  TCP disconnects. ESP32 pushes `STATE,...` on reconnect. NVS persists relay
  state across watchdog/brownout resets.

## Recent Changes (2026-02-25)

### Sequence Runner Framework  `[IMPLEMENTED]`
- Generic `sequence_runner.c/.h`: task lifecycle, SEQ streaming, mutual exclusion
- `sequence_impl_t` interface: `prepare()`, `execute()`, `request_stop()`, `cleanup()`
- Helper API: `seq_report_progress()`, `seq_check_stop()`, `seq_send_data()`
- Interactive prompt support: `seq_prompt_user()` + `sequence_runner_provide_confirmation()`
- 4 LAN commands: `sequence_start`, `sequence_stop`, `sequence_status`, `sequence_confirm`
- Registration pattern: new sequence types add ~200 LOC each

### Power-O3 Calibration Sequence (`seq_power_cal.c`)  `[IMPLEMENTED]`
- Type name: `"cal"`, param: O2 flow LPM (default 4.0)
- 4 phases: baseline (30s), sweep_up (0-100%), sweep_down (100-0%), random_pair (15 levels × air OFF/ON)
- Enhanced CAL_DATA format: timestamp, power, actual, o3, o2_lpm, air_comp, total_lpm, temp, phase
- CAL_START/CAL_COMPLETE metadata messages
- Prerequisites: O3 gen relay ON, motor pot initialized
- Safe cleanup: power→0%, air comp OFF on abort/completion

### Airflow/Concentration Validation (`seq_airflow_val.c`)  `[IMPLEMENTED]`
- Type name: `"validate"`, params: power_pct, o2_lpm (defaults 75%, 4.0)
- 5 phases: prompt_vessel, prompt_direct, stabilize (60s), measure (60s), complete
- Interactive: operator confirms L-valve routing at two points
- New messages: `VAL_START`, `VAL_DATA`, `VAL_RESULT`
- Reports: mean/std O3, expected O3, deviation percentage
- Intended as pre-flight check before sterilization/decay testing
- **Boundary note**: ESP32 sends `SEQ,prompt,<id>,<message>` with plain-text fallback.
  Dashboard should map prompt IDs (`prompt_vessel`, `prompt_direct`) to rich UI
  dialogs with step-by-step instructions.  See `interface_contract.md` §Sequence
  Integration Guide for the full Dashboard responsibility matrix.

### Relay Source-Tracking  `[IMPLEMENTED]`
- Added `relay_source_t` enum: `BOOT`, `LAN`, `RPC`, `INTERNAL`, `SEQUENCE`,
  `NVS_RESTORE`, `EMERGENCY`, `UNKNOWN`
- `relay_set_with_source()` logs every change with `[src=XX]` tag
- `relay_set()` still works (delegates with `RELAY_SRC_UNKNOWN`)
- LAN command handler and Golioth RPC now route through
  `blocksi_state_set_relay()` for unified tracking
- `blocksi_relay_state_t` tracks all 3 relays with `last_source` and
  `last_change_ms` per relay

### NVS Relay Persistence  `[IMPLEMENTED]`
- Relay states saved to NVS (`relay_ctrl/states`) on every change
- On boot: `esp_reset_reason()` determines behavior:
  - Power-on / external reset → all OFF (safety)
  - Watchdog / brownout / panic / SW reset → restore from NVS
- `relay_save_to_nvs()` / `relay_restore_from_nvs()` in `relay_control.c`

### LAN Reconnect State Push  `[IMPLEMENTED]`
- `lan_event_callback_t` added to `lan_client_config_t`
- On connect: ESP32 sends `STATE,ozone_gen=...,o2_conc=...,air_comp=...,power=...,flow=...`
- On disconnect: logged but no relay state change (states preserved)
- New `STATE` message type added to `interface_contract.md`

## Hardware Notes

- **Motor Pot**: PRM162 dual-gang, Section 1 floats with MP-8000 (~10V above ground), Section 2 gives ADC feedback
- **106-H quirks**: Fixed 19200 baud, non-standard female D9 pinout, RS232 level shifter required
- **SSRs**: Active-high (Kerwinn KG1-1DA25)
- **Air compressor**: Adds ~10 LPM at ~21% O2 to O2 concentrator output (~93% O2)
