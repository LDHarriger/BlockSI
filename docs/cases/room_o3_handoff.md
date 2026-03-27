# Room O3 Sensor — Copilot Handoff (2026-03-26)

## Problem
DFRobot SEN0321 room O3 sensor shows "quantized mode" — values snap to fixed ppb levels instead of smooth readings. See full audit: `docs/cases/room_o3_sensor_audit.md`.

## What was built
Three diagnostic firmware commands added to `main.c` (line ~1132), backed by helper functions in `dfrobot_ozone.c/.h`:
- `diag_room_o3_raw,<count>` — raw byte-level I2C reads
- `diag_room_o3_delay,<ms>,<count>` — reads with configurable conversion delay
- `diag_room_o3_regs` — dumps registers 0x03-0x0A

Dashboard side: `tcp_server.py` `_DIAG_SUBTYPES` updated to log these to `Data/Diagnostics/`.

Send commands from dashboard **Debug tab** (no `CMD,` prefix needed — `send_command()` adds it).

## First diagnostic results (`Data/Diagnostics/2026-03-26_223852_diag.log`)

**Critical finding: I2C bus contention between sensor_aggregator and diagnostic commands.**

- First 9 raw samples: `ESP_ERR_TIMEOUT` (bus busy — aggregator holding it)
- Spike values are **register contents from wrong addresses**, not real O3:
  - `0x70,0xDF` (28895 ppb) = contents of register 0x04
  - `0xFF,0x86` (65414 ppb) = contents of register 0x05
  - `0x04,0x00` (1024 ppb) = register *address* 0x04 leaking into byte0
- Register dumps between runs showed scrambled register map — pointer corruption confirmed

**Root cause of contaminated diagnostics**: `sensor_aggregator.c:sample_room_o3()` calls `dfrobot_o3_read()` every 500ms on the same I2C bus. Its mutex protects the data accumulator, NOT the I2C bus. The multi-step I2C sequence (trigger write → 100ms delay → pointer write → read) is non-atomic and interleaves with diagnostic reads.

## Aggregator pause — IMPLEMENTED

`sensor_aggregator_pause_room_o3()` / `_resume_room_o3()` added to `sensor_aggregator.c/.h`. All three diag commands auto-pause/resume around their I2C reads. Explicit `diag_room_o3_pause` / `diag_room_o3_resume` commands also available for extended sessions.

## What needs to happen next

1. **Flash updated firmware and re-run diagnostics with clean bus** — Re-run `diag_room_o3_raw,20` and `diag_room_o3_regs` from the Debug tab. The aggregator now auto-pauses, so results should be clean. This will reveal whether quantized mode is:
   - **(a)** Inherent to the sensor hardware/firmware (SEN0321 internal register pointer bug)
   - **(b)** Only caused by I2C contention (which would be unexpected in normal operation since DFRobot is the only I2C device on the bus now — MCP4725/DS3502 removed, motor pot uses GPIO, thermocouple uses SPI)

3. **If clean reads still show quantized values** → sensor's onboard MCU has a register-latch bug. May need: different read timing, periodic re-init, or hardware replacement.

4. **If clean reads are smooth** → the aggregator is somehow conflicting with itself (check for re-entrant timer callbacks) or there's an intermittent electrical issue on the I2C bus.

## Key files
- `Interfaces/ControlSystem/main/dfrobot_ozone.c` — driver + 3 new diag helpers (after line 401)
- `Interfaces/ControlSystem/main/main.c` — command handlers (line ~1132)
- `Interfaces/ControlSystem/main/sensor_aggregator.c` — aggregator task, `sample_room_o3()` at line 52
- `Interfaces/PC/dashboard/tcp_server.py` — DIAG handler + subtype whitelist (line 241)
- `docs/cases/room_o3_sensor_audit.md` — full audit with hypotheses and data analysis
