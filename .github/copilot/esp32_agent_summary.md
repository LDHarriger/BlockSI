# ESP32 Agent Summary

> Last updated: 2026-02-28 (executor-owned relay prereqs + hardware interlock)

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
| `main.c` | Entry point, WiFi, Golioth, LAN command handler |
| `lan_client.c` / `.h` | TCP client — connects to PC on port 5000 |
| `relay_control.c` / `.h` | 3x SSR control (ozone_gen, o2_conc, air_comp) — active-high, NVS persistence, hardware interlock |
| `motor_pot.c` / `.h` | PRM162 motorized potentiometer driver via DRV8833 H-bridge + ADC feedback |
| `o3_power_control.c` / `.h` | High-level power control API (uses motor_pot) |
| `model_106h_interface.c` / `.h` | 106-H ozone monitor RS232 interface (UART2, 19200 baud) |
| `dfrobot_ozone.c` / `.h` | DFRobot room-O3 sensor (I2C) |
| `max31855_thermocouple.c` / `.h` | MAX31855 thermocouple interface (SPI) |
| `sensor_aggregator.c` / `.h` | Aggregates all sensor readings into one struct |
| `dosimetry.c` / `.h` | O3 dose calculation |
| `backup_storage.c` / `.h` | SPIFFS-based recording and backup |
| `blocksi_state.c` / `.h` | Unified system state management |
| `power_calibration_v2.c` | Legacy motor pot calibration sweep |
| `sequence_runner.c` / `.h` | Sequence framework: lockout, SEQ streaming, prompt support |
| `seq_executor.c` / `.h` | **Generic recipe-based step executor** (replaces seq_power_cal + seq_airflow_val) |
| `seq_sensor_adapter.c` / `.h` | 106-H sample counter for executor's sample-counted holds |
| `blocksi_pins.h` | **All GPIO pin assignments** — check before adding hardware |

**Removed from build** (kept on disk for reference):
| `seq_power_cal.c` / `.h` | Old: Power-O3 calibration (4-phase, type: `cal`) |
| `seq_airflow_val.c` / `.h` | Old: Airflow validation (5-phase, type: `validate`) |

## Recipe-Based Executor  `[IMPLEMENTED]`

Architecture: **"ESP32 = Arms, PC = Brains"**

The PC generates complete recipes and sends them step-by-step via LAN.
The ESP32 executes blindly with precise sample-counted holds:

```
PC → CMD,sequence_start,<type>,flow=4.0,relay_o2=1,relay_o3=1,relay_air=0
PC → CMD,seq_step,<idx>,<pwr>,<hold_samples>,<phase>[,<air_comp>]  (× N)
PC → CMD,seq_prompt,<before>,<id>,<text>               (× M)
PC → CMD,seq_run
ESP32 →                          (applies relay prereqs, waits 3s)
ESP32 → SEQ,<type>,RELAY,o2_conc=ON,ozone_gen=ON,air_comp=OFF
ESP32 → SEQ,<type>,STARTED,...
ESP32 → SEQ,<type>,STEP,...,<air_comp>  (per step transition)
ESP32 → SEQ,<type>,SAMPLE,...,<air_comp>  (per 106-H sample, ~2.5s)
ESP32 → SEQ,<type>,COMPLETE,...    (or ABORTED)
```

Key properties:
- Max 256 steps, 16 prompts per recipe
- Steps sorted by index before execution
- Sample-counted holds via `seq_sensor_get_sample_count()` (monotonic 106-H counter)
- **Relay prereqs**: `sequence_start` params (`relay_o2`, `relay_o3`, `relay_air`) specify
  desired relay states.  Executor saves originals, applies in safe order (O2 → O3 → Air),
  waits 3s if any changed, sends `SEQ,*,RELAY` notification.
- **Cleanup**: completion → power=0, air=OFF, O2/O3 unchanged.  Abort → same + ozone_gen=OFF.
- Per-step air compressor relay control via `air_comp` field (optional, default OFF)
- 1-second stabilization delay after air compressor toggle
- Prompts block execution via binary semaphore until `CMD,sequence_confirm`
- FreeRTOS task (8KB stack, priority 5)
- Integrates with `sequence_runner` for `sequence_runner_is_active()` lockout
  via `sequence_runner_force_active()` / `sequence_runner_force_idle()` bridge

ESP32 does NOT: generate random values, compute statistics, query models,
determine pass/fail, store calibration data, or generate certificates.

## LAN Commands — Sequence Protocol

| Command | Args | Purpose |
|---------|------|---------|
| `sequence_start` | `<type>,<key=val params>` | Begin recipe loading.  Relay params: `relay_o2`, `relay_o3`, `relay_air` |
| `seq_step` | `<idx>,<pwr>,<hold>,<phase>[,<air_comp>]` | Add step (air_comp optional, default 0) |
| `seq_prompt` | `<before>,<id>,<text>` | Add prompt |
| `seq_run` | none | Apply relay prereqs, sort steps, start execution task |
| `sequence_confirm` | none | Unblock pending prompt |
| `sequence_abort` | `[reason]` | Abort sequence |
| `sequence_stop` | `[reason]` | Legacy alias for abort |
| `sequence_status` | none | Query executor state |

Full specification in `interface_contract.md` §Recipe-Based Sequences.

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

## Pending Work

- `[IMPLEMENTED]` **Executor-owned relay prerequisites**: `sequence_start` params
  specify relay states (`relay_o2`, `relay_o3`, `relay_air`).  Executor saves originals,
  applies in safe order, waits 3s, sends `SEQ,*,RELAY`.  Supersedes old preflight reject.
- `[IMPLEMENTED]` **Hardware interlock**: Air compressor is internal to MP-8000.
  `relay_set_with_source()` rejects `air_comp ON` when `ozone_gen OFF`, and
  auto-sets `air_comp OFF` when `ozone_gen` is turned OFF.
- `[IMPLEMENTED]` **Air compressor per-step control**: `seq_exec_step_t` has `air_comp`
  bool.  Executor toggles `RELAY_AIR_COMP` at step transitions.  STEP and SAMPLE
  messages include air_comp state.
- `[IMPLEMENTED]` **Recipe executor**: Generic `seq_executor` with sample-counted
  holds, prompt blocking, and LAN streaming.  Replaces all sequence-specific firmware.
- `[IMPLEMENTED]` **Relay robustness**: Source tracking, NVS persistence, reconnect
  state push, reset-reason-aware restore.
- `[PROPOSED]` **Relay dropout investigation**: Source-tracking implemented for
  ongoing monitoring. Monitor logs for `[src=...]` tags.
- `[PROPOSED]` **Legacy cleanup**: `power_calibration_v2.c` and its
  `calibrate_start`/`calibrate_stop` commands could be removed once the Dashboard
  fully adopts the recipe protocol.

## Recent Changes (2026-02-28)

### Executor-owned relay prerequisites + hardware interlock  `[IMPLEMENTED]`
- `relay_control.c`: Hardware interlock in `relay_set_with_source()` —
  (a) `air_comp ON` rejected when `ozone_gen OFF` (`ESP_ERR_INVALID_STATE`),
  (b) `ozone_gen OFF` auto-sets `air_comp OFF`.
- `seq_executor.c`: Added `seq_relay_prereqs_t`, `apply_relay_prereqs()`,
  `restore_relays()`.  Parses `relay_o2/relay_o3/relay_air` from params.
  Applies in safe order (O2 → O3 → Air), saves originals, waits 3s,
  sends `SEQ,*,RELAY` notification.  Cleanup: abort kills O3+Air, completion
  kills Air only.
- `main.c`: Removed old preflight reject from `seq_run` handler.  Executor
  now owns relay lifecycle.
- Supersedes 2026-02-27 preflight reject approach.

### Previous Changes (2026-02-26)
- `seq_executor.c/.h`: Generic step executor — receives recipe from PC, executes
  with sample-counted holds, streams `SEQ,<type>,SAMPLE,...` per 106-H reading
- `seq_sensor_adapter.c/.h`: Monotonic 106-H sample counter, called from
  `on_106h_sample()` in main.c
- `sequence_runner.c/.h`: Added `sequence_runner_force_active()` and
  `sequence_runner_force_idle()` bridge functions for executor lockout
- `main.c`: New LAN command handlers for `seq_step`, `seq_prompt`, `seq_run`,
  `sequence_abort`.  Modified `sequence_start` to call `seq_executor_begin()`.
  Added `seq_sensor_notify_new_sample()` in `on_106h_sample()`.
  Replaced old `seq_power_cal`/`seq_airflow_val` registration with `seq_executor_init()`.
- `CMakeLists.txt`: Swapped old sequence files for new executor files.
- Superseded messages: `CAL_DATA`, `CAL_START`, `CAL_COMPLETE`, `VAL_DATA`,
  `VAL_START`, `VAL_RESULT`, `SEQ_DONE` — all replaced by generic `SEQ,<type>,*` format.

### Previous Session Changes (2026-02-25)
- Sequence runner framework + interactive prompt support
- `seq_power_cal.c` + `seq_airflow_val.c` (now removed from build)
- Relay source-tracking (`relay_source_t`, `relay_set_with_source()`)
- NVS relay persistence with reset-reason awareness
- LAN reconnect state push (`STATE,...`)

## Hardware Notes

- **Motor Pot**: PRM162 dual-gang, Section 1 floats with MP-8000 (~10V above ground), Section 2 gives ADC feedback
- **106-H quirks**: Fixed 19200 baud, non-standard female D9 pinout, RS232 level shifter required
- **SSRs**: Active-high (Kerwinn KG1-1DA25)
- **Air compressor**: Internal to MP-8000.  Adds ~10 LPM at ~21% O2 to O2 concentrator
  output (~93% O2).  Relay circuit has NO power unless `ozone_gen` SSR is ON.
  Firmware interlock enforces this dependency.
