# BlockSI Architectural Decisions Log

> Decisions are **immutable** once logged.  To reverse or modify a decision,
> add a new entry that supersedes the original.

---

### 2026-02-24: PC is sole authority for power_target_pct

**Context**: Stale ESP32 DATA telemetry lines carried old `power_target_pct`
values.  When `apply_telemetry()` accepted these, the UI slider was overwritten,
triggering a new command to ESP32 -- causing an infinite oscillation loop.

**Decision**: `apply_telemetry()` on the PC dashboard **never** accepts
`power_target_pct` from ESP32 telemetry (DATA field index 15).  The PC is the
sole authority for what power level is commanded.  The ESP32 reports
`power_actual_pct` and `wiper_voltage` as sensor readings, which the PC does
accept.

**Rationale**: The PC originates all manual power commands and runs the
calibration state machine.  There is no scenario where the ESP32 should
unilaterally change the PC's power target.  ESP32-side `power_target_pct` in
telemetry is informational only (useful for debug/logging).

---

### 2026-02-24: ESP32 should own automated sequences (future)

**Context**: The CalibrationRunner on the PC sends individual
`CMD,power_set,N` commands over TCP every 2 seconds.  A network hiccup could
corrupt the sequence.  The ESP32 already has `power_calibration_v2.c` which
runs sweep logic locally with direct hardware access.

**Decision**: For future development, ESP32 will own automated sequences
(calibration sweeps, sterilization cycles, etc.).  Design:
- PC sends `CMD,sequence_start,cal,<params>` to initiate
- ESP32 runs autonomously, streams `SEQ,phase,progress,power,...` status lines
- PC dashboard enters observer mode (`sequence_active=True`), showing progress
- PC sends `CMD,sequence_stop` to abort
- Normal `DATA,...` telemetry continues alongside `SEQ,...` status

**Rationale**: ESP32 has deterministic timing (FreeRTOS), direct hardware
access, and no network dependency during execution.  PC is better suited as
a monitoring/visualization layer.

**Status**: `[DECIDED]` -- not yet implemented.  CalibrationRunner currently
runs PC-side.

---

### 2026-02-24: Deleted deprecated power_calibration.c (DS3502 version)

**Context**: `power_calibration.c` was a standalone program written for the
old DS3502 digital potentiometer.  It had its own `app_main()` and serial
command interface, and was never compiled into the current firmware (not in
`CMakeLists.txt`).  The replacement `power_calibration_v2.c` uses the motor
pot and is fully integrated.

**Decision**: Deleted `power_calibration.c` from the repository.

**Rationale**: Dead code creates confusion.  The file used DS3502 APIs that
no longer exist in the project.

---

### 2026-02-25: Adopted multi-agent collaboration protocol

**Context**: BlockSI development spans two domains (ESP32 firmware and PC
dashboard) with separate context requirements.  Long chat sessions risk
context truncation and stale references.

**Decision**: Established a formal collaboration protocol in
`.github/copilot/` with:
- Domain-separated agent chats (Dashboard Agent, ESP32 Agent)
- Agent-written summaries (`dashboard_agent_summary.md`, `esp32_agent_summary.md`)
- Shared interface contract (`interface_contract.md`)
- This decisions log
- Status taxonomy: `[IMPLEMENTED]`, `[DECIDED]`, `[PROPOSED]`
- Interface Change Rule: update contract first, then summary

**Rationale**: Prevents context loss across sessions, ensures cross-domain
changes are coordinated, and gives each agent focused deep context in its
domain.

---

### 2026-02-25: Relay source-tracking and audit logging

**Context**: Relay dropouts were reported but hard to diagnose because
`relay_set()` logged state changes without identifying the caller (LAN, RPC,
internal, boot, etc.).  Additionally, `blocksi_state_set_relay()` only tracked
2 of 3 relays, and the LAN command handler bypassed state tracking entirely.

**Decision**: 
1. Added `relay_source_t` enum (`BOOT`, `LAN`, `RPC`, `INTERNAL`, `SEQUENCE`,
   `NVS_RESTORE`, `EMERGENCY`, `UNKNOWN`) to all relay state changes.
2. `relay_set_with_source()` logs every change with source tag.
3. All relay commands (LAN handler, Golioth RPC) now route through
   `blocksi_state_set_relay()` for unified tracking.
4. `blocksi_relay_state_t` expanded to track all 3 relays with per-relay
   `last_source` and `last_change_ms`.

**Rationale**: First-line diagnostic for relay dropout investigation.  Source
tracking makes it immediately visible whether a dropout is from a watchdog
reset, network event, RPC call, or firmware bug.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: NVS relay persistence with reset-reason awareness

**Context**: Relay states lived only in RAM.  Any reset (watchdog, brownout,
crash) caused all relays to drop to OFF, interrupting active operations.
However, restoring relays after an intentional power-off (e.g., emergency
unplug) would be a safety hazard — ozone could start flowing unexpectedly.

**Decision**: Use `esp_reset_reason()` to distinguish reset types:
- **Power-on / external reset**: All relays OFF (safety)
- **Watchdog / brownout / panic / SW reset**: Restore from NVS

Relay states are persisted to NVS (`relay_ctrl/states`) on every state change.
On boot, `relay_restore_from_nvs()` is called only for transient resets.
On power-on, the all-OFF state is saved to NVS to clear stale data.

**Rationale**: Balances operational continuity (survive glitches) with safety
(never auto-start ozone after intentional power-off).

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: LAN connect/disconnect event callbacks and STATE push

**Context**: When TCP connection dropped and re-established, the PC dashboard
had no way to know the ESP32's current relay/power state without manually
querying.  Relay controls could appear out of sync.

**Decision**:
1. Added `lan_event_callback_t` to `lan_client_config_t` — called on
   connect and disconnect events.
2. On connect, ESP32 sends unsolicited `STATE,...` message with current
   relay states, power level, and flow rate.
3. Added `STATE` message type to `interface_contract.md`.

Format: `STATE,ozone_gen=<0|1>,o2_conc=<0|1>,air_comp=<0|1>,power=<pct>,flow=<lpm>\n`

**Rationale**: Ensures PC dashboard is always synchronized on reconnect.
Relay/power state survives disconnects on the ESP32 side, and the PC picks
up the actual state immediately without needing to poll.

**Status**: `[IMPLEMENTED]`
