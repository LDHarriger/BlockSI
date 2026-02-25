# BlockSI Interface Contract

> Last updated: 2026-02-25
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

### ESP32 -> PC:  Calibration data (during sweep)  `[IMPLEMENTED]`
```
CAL_DATA,power_pct,actual_pct,o3_pct,temp_c,direction,elapsed_s\n
```

### ESP32 -> PC:  State push (on connect/reconnect)  `[IMPLEMENTED]`
```
STATE,ozone_gen=<0|1>,o2_conc=<0|1>,air_comp=<0|1>,power=<pct>,flow=<lpm>\n
```
Sent automatically by ESP32 immediately after TCP connection is established.
PC dashboard should parse this to synchronize its relay/power display with
the ESP32's actual state.  This ensures the PC is never out of sync after
a disconnect/reconnect cycle.

### Future:  Sequence status  `[DECIDED]`
```
SEQ,<phase>,<progress_pct>,<power_pct>,<extra...>\n
```
Not yet implemented.  Design: ESP32 streams these during autonomous sequences,
PC dashboard enters observer mode (`sequence_active=True`).

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

## LAN Commands  (28 total, as of 2026-02-25)

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

### Calibration Sweep
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `calibrate_start` | none | `calibration=started` | Requires O3 gen relay ON, motor pot init |
| `calibrate_stop` | none | `calibration=stopping` | |
| `calibrate_status` | none | `active=<0\|1>,pct=<val>,dir=<val>,pts=<val>` | |

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
| `air_comp` | Air compressor SSR | Adds ~10 LPM @ ~21% O2 |

SSRs are **active-high** (Kerwinn KG1-1DA25).

---

## Shared Constants

These constants must match between ESP32 firmware and PC dashboard:

| Constant | Value | Used by |
|----------|-------|---------|
| `O3_MASS_FLOW_K` | 0.357 | mg/s = %vol * LPM * K |
| `AIR_COMP_LPM` | 10.0 | LPM added when air compressor on |
| `POWER_MODEL_A` | 1.78 | O3_max = A/F + B (to be replaced by fitted model) |
| `POWER_MODEL_B` | 1.40 | O3_max = A/F + B |
| `DEFAULT_FLOW_LPM` | 4.0 | Default O2 flow rate |
| TCP Port | 5000 | Default LAN connection port |
