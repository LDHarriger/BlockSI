# BlockSI Project - AI Coding Agent Instructions

## Project Overview

BlockSI is an ozone sterilization monitoring and control system for Shroom-E Co. It comprises:
- **ESP32 Firmware** (`Interfaces/ControlSystem/`): ESP-IDF v5.4+ project controlling ozone generation, monitoring sensors, and streaming data
- **PC Dashboard** (`Interfaces/PC/`): Streamlit-based Python app for real-time visualization and device control
- **Data** (`Data/`): CSV telemetry logs from sterilization runs

## Architecture & Data Flow

```
106-H Ozone Monitor (RS232) → ESP32 (UART2) → Golioth Cloud + LAN TCP → PC Dashboard
        ↓                         ↓
   O3 wt% readings         Control: Relays, Motor Pot, Dosimetry
```

Key integration pattern: The ESP32 acts as a bridge between industrial equipment (106-H monitor, MP-8000 generator) and cloud/PC interfaces. The LAN client (`lan_client.c`) uses a simple text command protocol (`cmd:args\n → response\n`).

## ESP32 Firmware Conventions

### Pin Assignments
All GPIO pins are centralized in [blocksi_pins.h](Interfaces/ControlSystem/main/blocksi_pins.h). **Always check here before adding hardware**:
- I2C: GPIO 21/22 (DFRobot O3 sensor)
- SPI: GPIO 18/19/5 (MAX31855 thermocouple)
- UART2: GPIO 16/17 (106-H RS232 via level shifter)
- Relays: GPIO 12/13 (SSR control)
- Motor Pot: GPIO 25/26/34 (DRV8833 + ADC feedback)

### Module Pattern
Each peripheral follows the pattern in [sensor_aggregator.h](Interfaces/ControlSystem/main/sensor_aggregator.h):
```c
esp_err_t module_init(config_t *config);  // Returns ESP_OK/ESP_FAIL
void module_deinit(void);
bool module_is_initialized(void);         // Safe to call anytime
```

### Configuration
Runtime config uses Kconfig (`Kconfig.projbuild`). Access via `CONFIG_*` macros (e.g., `CONFIG_WIFI_SSID`).

### Build & Flash Commands
```powershell
idf.py build                    # Build firmware
idf.py -p COM3 flash monitor    # Flash and open serial monitor (exit: Ctrl+])
idf.py menuconfig               # Configure WiFi, Golioth PSK, pin assignments
```

## Hardware-Specific Notes

- **Motor Pot replaced DS3502**: Ground isolation issues required switching from I2C digital pot to motorized potentiometer (PRM162 + DRV8833). See comment block in `main/CMakeLists.txt`.
- **106-H quirks**: Baud rate is fixed 19200. D9 connector has non-standard female pinout. RS232 level shifter required.
- **SSRs are active-high** (Kerwinn KG1-1DA25)

## LAN Command Protocol

ESP32 ↔ PC uses text commands over TCP port 5000:
```
relay_set:ozone_gen,1    # Turn on ozone generator
power_set:50             # Set power to 50%
sensors_get              # Query all sensor status
recording_start:MyRun    # Start backup recording
```
See command handlers in [main.c](Interfaces/ControlSystem/main/main.c#L350) `lan_command_handler()`.

## PC Dashboard

Run with: `streamlit run blocksi_dashboard.py -- --port 5000`

The dashboard (`blocksi_dashboard.py`) is the current version. Versioned files (`_v4` through `_v8`) are development history.

## Code Style

- ESP-IDF conventions: `esp_err_t` returns, `ESP_LOG*` macros, FreeRTOS tasks
- Module-level static state structs: `static struct { ... } s_module = {0};`
- Tag convention: `static const char *TAG = "MODULE_NAME";`

## Common Tasks

| Task | Key Files |
|------|-----------|
| Add new sensor | Create `sensor.c/.h`, add to `sensor_aggregator.c`, register in `peripherals.c` |
| Add LAN command | Add handler in `main.c:lan_command_handler()` |
| Modify power control | `o3_power_control.c` (API), `motor_pot.c` (hardware) |
| Change cloud telemetry | `publish_to_golioth()` in `main.c` (CBOR encoding) |
