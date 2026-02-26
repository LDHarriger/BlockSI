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

---

### 2026-02-25: Generic sequence runner framework

**Context**: The ESP32 needs to run at least 5 autonomous sequence types
(power-O3 calibration, airflow validation, fill model calibration, O3 decay
testing, sterilization).  Each shares common patterns: multi-phase execution,
SEQ progress streaming, graceful abort, safe cleanup, mutual exclusion.

**Decision**: Created a generic `sequence_runner` framework:
- `sequence_runner.h/.c`: Lifecycle management, task spawning, SEQ message
  streaming, LAN command integration (`sequence_start`, `sequence_stop`,
  `sequence_status`)
- `sequence_impl_t` interface: `prepare()`, `execute()`, `request_stop()`,
  `cleanup()` — each sequence type implements this
- Helper functions: `seq_report_progress()`, `seq_check_stop()`, `seq_send_data()`
- Registration pattern: `sequence_runner_register()` per type at boot

First implementation: `seq_power_cal.c` (type name `"cal"`), 4-phase calibration:
1. baseline (30s at 0%, air OFF)
2. sweep_up (0→100% in 1% steps, air OFF)
3. sweep_down (100→0% in 1% steps, air OFF)
4. random_pair (15 random levels × 20s air OFF + 20s air ON)

Commands: `CMD,sequence_start,cal,4.0` / `CMD,sequence_stop` / `CMD,sequence_status`

The old `calibrate_start`/`calibrate_stop`/`calibrate_status` commands remain
functional for backward compatibility but the dashboard should migrate to
the new `sequence_*` commands.

**Rationale**: A single framework avoids duplicating task management, SEQ
streaming, abort handling, and safety cleanup across 5+ sequence types.
Each new sequence is just a ~200-line implementation file.

**Status**: `[IMPLEMENTED]` — framework + power calibration sequence

---

### 2026-02-25: Interactive prompt support in sequence runner

**Context**: The airflow/concentration validation sequence requires operator
interaction at two points: routing the L-valve to the vessel, then routing
direct to the 106-H sensor and matching flow.  The sequence must pause and
wait for the operator to physically adjust hardware and confirm readiness.

**Decision**: Added interactive prompt/confirm mechanism to the sequence runner:
1. `seq_prompt_user(prompt_id, message, ...)` — sends `SEQ,prompt,...` to PC,
   blocks the sequence task on a FreeRTOS queue until confirmation arrives
2. `CMD,sequence_confirm,<prompt_id>[,<value>]` — LAN command from PC, 
   calls `sequence_runner_provide_confirmation()` which unblocks the queue
3. Queue depth 1, prompt ID matching, 5-minute timeout, stop-check polling

The mechanism is generic — any future sequence can use `seq_prompt_user()` for
operator interaction without changes to the framework.

**Rationale**: Physical operations (valve routing, flow adjustment) can never
be automated with current hardware.  Rather than splitting the sequence into
multiple commands ("start part 1", "start part 2"), a single sequence with
interactive pauses gives the operator a guided workflow.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Airflow/concentration validation sequence

**Context**: Before running sterilization or decay testing, the operator needs
to verify that the system produces the expected O3 concentration.  The 106-H
measures O3 at the vessel outlet, but vessel transit delay, surface decay, and
substrate absorption can cause discrepancies.  A direct-to-sensor measurement
eliminates these variables.

**Decision**: Implemented `seq_airflow_val.c` (type: `"validate"`), 5-phase
interactive sequence:
1. `prompt_vessel` — operator routes L-valve to vessel, confirms rotameter LPM
2. `prompt_direct` — operator routes L-valve direct to 106-H, matches flow
3. `stabilize` — power on at target level, 60s stabilization
4. `measure` — 60s measurement window, sample every 2s, compute mean/std
5. `complete` — report expected vs actual O3, deviation percentage

Parameters: `CMD,sequence_start,validate,<power_pct>,<o2_lpm>` (defaults 75%, 4.0)
New messages: `VAL_START`, `VAL_DATA`, `VAL_RESULT`

This sequence is intended as a sub-sequence / pre-flight check for sterilization
and decay testing workflows.

**Rationale**: The direct-to-sensor route gives a clean baseline measurement.
Future sterilization/decay sequences can call this as a prerequisite check.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Sequence Integration Guide — ESP32 executes, Dashboard observes

**Context**: The airflow validation sequence (`seq_airflow_val.c`) is heavily
interactive — operator must physically manipulate valves and confirm via the UI.
This raised the question of whether the ESP32 was overstepping into Dashboard
territory, and whether the Dashboard agent has clear guidance on what to build.

**Decision**: Added a comprehensive "Sequence Integration Guide" section to
`interface_contract.md`.  Key principles:

1. **ESP32 owns all timing-critical execution** — phase transitions, hardware
   control, data streaming, statistics computation.
2. **Dashboard is a thin observer** — shows progress, renders prompts as rich
   UI (not limited to ESP32's plain-text fallback), can abort.
3. **Prompt IDs are semantic** — Dashboard maps `prompt_vessel`, `prompt_direct`
   to rich dialogs with step-by-step instructions, diagrams, and optional
   numeric input.  The ESP32's `<message_text>` is a minimal fallback.
4. **Dashboard never sends `power_set` or `relay_set` during active sequence** —
   except via E-STOP which sends `sequence_stop` first.
5. **Responsibility matrix** and **message handling table** give the Dashboard
   agent a concrete checklist of what to parse and what UI to build.
6. **CalibrationRunner refactoring plan** documented: current PC-driven approach
   must become observer mode using `CMD,sequence_start,cal,...` and `SEQ,...`/`CAL_DATA`
   parsing.

**Rationale**: Clear documentation prevents both agents from accidentally
duplicating work or leaving gaps.  The Dashboard agent needs to build substantial
UI (prompt dialogs, observer banner, live charts, result displays) and should
not have to reverse-engineer the protocol from ESP32 source code.

**Status**: `[DECIDED]` → `[IMPLEMENTED]` (Dashboard agent, session 4)

---

### 2026-02-25: Dashboard observer-mode rewrite (CalibrationRunner deleted)

**Context**: The PC dashboard had a `CalibrationRunner` class (~175 lines) that
drove calibration sequences PC-side by sending individual `CMD,power_set,N`
commands every 2 seconds.  The ESP32 agent implemented a generic sequence runner
framework with autonomous execution, interactive prompts, and data streaming.
The dashboard needed to transition from "driver" to "observer."

**Decision**: Complete rewrite of `blocksi_dashboard.py` to observer mode:
1. **Deleted CalibrationRunner** entirely — no PC-driven sequence logic remains
2. Added 12 TCP message handlers in `_dispatch()`: DATA, RSP, STATE, SEQ,
   SEQ_DONE, CAL_START, CAL_DATA, CAL_COMPLETE, VAL_START, VAL_DATA, VAL_RESULT,
   plus SEQ,prompt special case
3. Added 3 new command helpers: `cmd_sequence_start()`, `cmd_sequence_stop()`,
   `cmd_sequence_confirm()`
4. SystemState rewritten: removed PC-driven cal fields, added observer fields
   (`seq_type`, `seq_phase`, `seq_progress`, `cal_samples`, `val_result`,
   `pending_prompt_id`, etc.)
5. `_tick()` rewritten: handles control lockout, prompt dialog triggering,
   cal/val observer UI updates
6. Pre-rewrite backup saved to `Interfaces/PC/Old/blocksi_dashboard_pre_observer.py`

**Rationale**: ESP32 has deterministic timing, direct hardware access, and no
network dependency during execution.  PC is better suited as monitoring/
visualization layer.  User explicitly chose to do the full rewrite in one pass
to avoid loose ends ("not addressing it all together would leave loose ends").

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: Calibration and Validation in Power tab (not separate tabs)

**Context**: The old dashboard had a standalone "Calibration" tab.  With the
addition of a Validation sequence, the question arose of where to place UI
for both sequences.

**Decision**: Both Calibration and Validation are placed in the **Power tab**
as `ui.expansion()` sections, alongside the Power Control expansion.  The
standalone Calibration tab was removed.  Tab count reduced from 5 to 4:
Power, Telemetry, Debug, Settings.

**Rationale**: User preference — "Perhaps we could have BOTH the calibration
sequence and validation accessed as in the power tab since that is what it is
being calibrated or validated."  Future sequence types (fill, decay, sterilize)
will get their own tabs when implemented.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-25: CalibrationRunner deletion (recoverable from git)

**Context**: The CalibrationRunner was ~175 lines of PC-driven calibration
state machine.  With the ESP32 owning sequence execution via the sequence
runner framework, this code became dead.

**Decision**: Deleted CalibrationRunner entirely rather than keeping it as
fallback.  Pre-rewrite file saved to `Old/blocksi_dashboard_pre_observer.py`.

**Rationale**: User explicitly said "Delete it. If we need to fall back we
can recover it from github."  Keeping dead code creates confusion and
maintenance burden.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-26: Generic recipe executor replaces sequence-specific firmware

**Context**: The ESP32 had sequence-specific firmware (`seq_power_cal.c`,
`seq_airflow_val.c`) that embedded calibration/validation logic — random
value generation, specific phase structures, statistics computation.  This
violated the "ESP32 = Arms, PC = Brains" principle established in session
discussions.  The ESP32 was overstepping into the PC's data analysis domain.

**Decision**: Replace all sequence-specific firmware with a single generic
recipe-based executor (`seq_executor.c`):
1. PC generates complete recipes (step lists, prompts, hold durations)
2. PC sends recipes via `sequence_start` → `seq_step` × N → `seq_prompt` × M → `seq_run`
3. ESP32 executes blindly: set power, count 106-H samples, stream `SEQ,<type>,SAMPLE,...`
4. ESP32 does NO analysis — no mean/std, no model queries, no pass/fail
5. Sample-counted holds (via `seq_sensor_adapter.c`) instead of time-based waits
6. Integration with existing `sequence_runner.c` for UI lockout via bridge functions

Files removed from build (kept on disk): `seq_power_cal.c`, `seq_airflow_val.c`
Files added: `seq_executor.c`, `seq_executor.h`, `seq_sensor_adapter.c`, `seq_sensor_adapter.h`

Superseded messages: `CAL_DATA`, `CAL_START`, `CAL_COMPLETE`, `VAL_DATA`,
`VAL_START`, `VAL_RESULT`, `SEQ_DONE`.  All replaced by generic
`SEQ,<type>,SAMPLE,...` / `STEP` / `COMPLETE` / `ABORTED`.

**Rationale**: The PC has Python + pandas + numpy for data analysis, model
fitting, and certificate generation.  The ESP32 should focus exclusively on
precise hardware control and deterministic timing.  A generic executor means
the PC can run ANY recipe type without firmware changes — only the PC-side
recipe generator needs updating for new sequence types.

**Status**: `[IMPLEMENTED]`

---

### 2026-02-26: Air compressor OFF during calibration and validation

**Context**: The old `seq_power_cal.c` had a Phase 4 ("random_pair") that
toggled the air compressor ON/OFF to characterize air-blend conditions.
Discussion established that O2-only conditions (~93% O2) are the baseline
for calibration and validation.  Air-blend conditions (~21% O2 at ~10 LPM
additional flow) fundamentally change the ozone chemistry and require a
completely separate calibration.

**Decision**: Air compressor must be OFF during all standard calibration
and validation sequences.  The PC should verify `air_comp=0` before sending
`seq_run`.  Air-blend characterization, if needed, is a separate recipe
type generated by the PC.

**Rationale**: Mixing O2-only and air-blend data in one calibration produces
unreliable models.  The variables (O2 concentration, total flow) change
simultaneously.  Clean O2-only data first, then optionally characterize
air-blend as a distinct operating condition.

**Status**: `[DECIDED]`