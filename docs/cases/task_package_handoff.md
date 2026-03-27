# Handoff: Testing Feedback & Validation Redesign Task Package

**From**: Claude_VSCode
**To**: Copilot_VSCode
**Date**: 2026-03-24
**Context**: User's task package dated 2026-03-25 with 5 prioritized tasks

---

## Bug Fix Applied (Pre-Task Work)

### TCP Singleton Fix — DONE, UNCOMMITTED

The modular dashboard (from commit `0d5f7d9`) had a broken TCP singleton that
prevented ESP32↔Dashboard connection. Dashboard showed blinking red
"Disconnected", console spammed `AttributeError: 'NoneType' object has no
attribute 'send_command'`.

**Root cause**: `tcp_server.py` line 486 had `tcp: TCPServer = None`. Other
modules imported with `from dashboard.tcp_server import tcp`, which copied
the `None` reference at import time. The later assignment in `_startup()`
didn't propagate.

**Files changed**:
- `Interfaces/PC/dashboard/tcp_server.py` line 486: Changed from
  `tcp: TCPServer = None` to `tcp = TCPServer()` (eager instantiation)
- `Interfaces/PC/blocksi_dashboard.py`: Removed `from dashboard.tcp_server
  import TCPServer`, changed `_startup()` to set `.port` and call `.start()`
  on existing singleton instead of creating new instance

---

## Task 2: Power Mismatch Audit — PARTIALLY COMPLETE

### What's Done

1. **Firmware code review**: Read and analyzed all relevant source files:
   - `motor_pot.c` / `.h` — full driver, coast stop, ADC averaging
   - `o3_power_control.c` / `.h` — power API wrapper
   - `blocksi_state.c` / `.h` — validator (hysteresis, DIAG, retry)
   - `main.c` — command handler, DATA telemetry format, `on_106h_sample()`
   - `blocksi_pins.h` — pin assignments, ADC range, deadband

2. **Data characterization**: Analyzed 3 calibration CSVs (Mar 22–24):
   - Systematic positive error (actual < target), worsens with power level
   - Mar 22 tracked well (error <3.3%); Mar 23–24 showed 5–9% deficit at high power
   - Bimodal ADC noise pattern in Mar 23–24, NOT present in Mar 22
   - Progressive degradation across sessions

3. **Audit report written**: `docs/cases/power_mismatch_audit.md`
   - Complete findings with data tables
   - Three hypotheses:
     - **H1**: Coast-mode wiper drift (no holding torque after positioning)
     - **H2**: Insufficient ADC averaging (3 samples, ESP32 ADC1 noise)
     - **H3**: Progressive hardware degradation of PRM-162/motor assembly
   - Test plan for next lab session
   - Diagnostic command specifications

### What Remains

4. **Firmware diagnostic commands — NOT IMPLEMENTED**

   Add two new LAN commands to `main.c` command handler (insert after the
   `motor_pot_calibrate` block, around line 569):

   **`CMD,diag_power_drift,<target_pct>`**:
   - Move motor to `<target_pct>` via `motor_pot_set_power()`
   - Read ADC at t=0, 100ms, 500ms, 1000ms, 2000ms, 5000ms
   - Send each reading as `DIAG,drift_sample,target=<T>,t_ms=<t>,adc=<raw>,pct=<pct>\n`
   - Send summary: `DIAG,drift_summary,target=<T>,initial_pct=<p0>,final_pct=<p5s>,drift=<delta>\n`
   - RSP: `drift_test=started,target=<T>`

   **`CMD,diag_power_noise,<num_samples>`**:
   - Take `<num_samples>` ADC readings at current position (no motor movement)
   - Use `motor_pot_read_adc()` for each sample with 10ms delay
   - Compute min, max, mean, std
   - Send: `DIAG,noise_stats,n=<N>,mean=<mean>,std=<std>,min=<min>,max=<max>,range_pct=<range>\n`
   - RSP: `noise_test=complete,n=<N>`

   Helper functions can go in `motor_pot.c` or directly in the command handler.
   The commands should be blocking (OK for diagnostic use, not for production).

   **Key code references**:
   - Command handler: `main.c:412` (`lan_command_handler`)
   - Insert point: after line 569 (`// End of pot handlers`)
   - ADC read: `motor_pot_read_adc()` (3-sample average) or direct
     `read_adc_averaged()` (static, would need a new public wrapper for N samples)
   - Motor goto: `motor_pot_set_power(float)`
   - DIAG send: `lan_client_send_message(char *)`

5. **Enhance existing `DIAG,power_mismatch` in `blocksi_state.c:276`**:
   - Add `adc_raw` and `settle_ms` to the DIAG message format
   - Current: `DIAG,power_mismatch,target=%u,actual=%.1f,retry=%u\n`
   - New: `DIAG,power_mismatch,target=%u,actual=%.1f,retry=%u,adc=%u,settle=%lld\n`
   - Read `motor_pot_read_adc()` for raw ADC and compute settle time from
     `now - state->power.last_command_ms`

6. **Dashboard-side DIAG logging — NOT IMPLEMENTED**

   The dashboard already has a `_handle_diag()` dispatcher (added in Session 14).
   Need to verify it logs all DIAG subtypes to file. If needed, add a dedicated
   diagnostic log file writer in `tcp_server.py` or `commands.py` that captures
   `drift_sample`, `drift_summary`, `noise_stats` DIAG messages to a timestamped
   file in `Data/Diagnostics/`.

---

## Task 1: Measurement Verification — NOT STARTED

Replace legacy validation framework with new measurement verification sequence.

**Key requirements from task package**:
- Archive `validation.py` (move to `archive/`)
- Remove validation UI tab and commands
- Create `Interfaces/PC/dashboard/verification.py` with:
  - Measure C_in at 100% power (stability criteria: 45-sample window, slope < 0.0003)
  - Measure C_in at power_hold (same stability criteria)
  - Save verification CSV with measured values
  - Update dose schedule recomputation with measured C_in values
- Add `decisions_log.md` entry for the architectural change
- Process time is FIXED (~30 min) — never adjusted

**Safety constraint**: Process time must never be adjusted to compensate for
concentration discrepancies (Chick-Watson kinetics — log-linear, not time-linear).

---

## Task 3: Rename Data/Calibration/ → Data/Power-O3_cal/ — NOT STARTED

- `git mv Interfaces/Data/Calibration Interfaces/Data/Power-O3_cal`
- Update all code references (grep for `Data/Calibration` and `Data\\Calibration`)
- Files that likely reference this path:
  - `dashboard/data_io.py`
  - `dashboard/commands.py`
  - `dashboard/state.py`
  - `analysis/power_o3_model.py`
  - Various sequence files

---

## Task 4: GUI Improvements — NOT STARTED (Blocked by Task 1)

- Sequence redirection: after calibration, prompt to run verification
- Collapsible calibration sections
- Telemetry plot reorganization
- Raw data table

---

## Task 5: Resource Documentation Audit — DONE

Report written: `docs/documentation_gaps.md`

Findings: 17 gaps identified across 3 severity levels:
- **Critical (4)**: GPIO mismatch in hardware.md, missing O3 alarm thresholds,
  PRM162 floating-ground wiring quirk, vessel volume inconsistency
- **Important (6)**: 106-H D9 pinout, RS232 level shifter, mixing screw motor,
  O2 concentrator model, gas path plumbing, exhaust filter
- **Informational (7)**: Orphaned resources, unlinked pinout image, motor pot
  constants, dosimetry dead volume, stale interface contract constants, etc.

Two Resources/ files are orphaned (evaluated alternatives): `PortaSense_D16.pdf`,
`DRI0044_TB6612FNG_MotorDriver.pdf`. None of the Resource files are cited by
filename in any documentation.

---

## Uncommitted Changes Summary

All changes are uncommitted on `main` branch:

| File | Change |
|------|--------|
| `Interfaces/PC/dashboard/tcp_server.py` | TCP singleton fix (line 486: `None` → `TCPServer()`) |
| `Interfaces/PC/blocksi_dashboard.py` | Startup refactor to use existing singleton |
| `docs/cases/power_mismatch_audit.md` | **NEW** — full audit report with hypotheses |
| `Interfaces/ControlSystem/main/main.c` | Existing uncommitted changes (pre-existing) |
| `Interfaces/Models/O3Power/*.json` | Existing uncommitted changes (pre-existing) |

---

## Key Architecture Notes

- TCP protocol: comma-separated, NEVER colons — `CMD,name,arg1,arg2\n`
- PC is sole authority for `power_target_pct`
- Motor pot uses **coast stop** (no holding torque) — key finding for mismatch
- Validator: 1000ms interval, 1500ms settle, 5% trigger / 2.5% clear hysteresis
- ADC averaging: 3 samples for readback, 5 samples during motor goto loop
- `POSITION_DEADBAND = 20` ADC counts (~0.5% of 3896 range)
- Dashboard runs NiceGUI at :8080, TCP server at :5000
- Venv at project root: `.venv\Scripts\python.exe`
