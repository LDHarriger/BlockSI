# Case File: CSTR Power Stability

**Status**: ACTIVE — fixes implemented Session 14, awaiting verification  
**Filed**: 2026-03-10  
**Last updated**: 2026-03-10

## Summary

During CSTR fill+evac calibration runs the motor pot power drops to 0%
mid-fill while relays stay ON, producing invalid calibration data.

## Observations (4 runs on 2026-03-10)

### Runs 1–3 (before dashboard fix)
- **Symptom**: Power and relays both cut to 0/OFF mid-fill
- **Debug log marker**: `seq_cleanup_pending=True`
- **Root cause**: Deferred-cleanup race — ESP32 `SEQ,cleanup` from a prior
  calibration sequence was still pending.  Dashboard `_safe_standby()` fired
  mid-fill, killing relays and power.
- **Fix** (commit `a27c224`): Clear `seq_cleanup_pending` at startup, guard
  `_safe_standby()` with `not S.fill_active`.

### Run 4 (after Runs 1–3 dashboard fix applied)
- **Symptom**: Target stayed 100%, actual fell to 0%, relays stayed ON
- **Debug log marker**: `seq_cleanup_pending=False`, `relay(o3=True …)`,
  `pwr_tgt=100%`, `pwr_act=0.0%`
- **Root cause**: DFRobot room-O3 sensor noise spike ≥0.30 ppm triggered
  `lab_o3_alarm_handler()` → `o3_power_emergency_stop()` which called
  `motor_pot_set_power(0)` directly, bypassing the state manager.
  `target_pct` stayed at 100%.  The validator detected the mismatch and
  retried, but each retry was killed by the next noise spike.  After 3
  retries the validator gave up permanently → motor stuck at 0%.

## Root Cause Chain (Run 4, detailed)

1. `CMD,power_set,100` → motor reaches 100% (RSP OK)
2. DFRobot noise spike > 0.30 ppm → `lab_o3_alarm_handler()`
3. `o3_power_emergency_stop()` → `motor_pot_set_power(0)` — target NOT updated
4. Validator detects 100% target vs 0% actual → retry #1
5. Next noise spike → emergency stop again
6. Retries 2, 3 follow same cycle → validator gives up after `POWER_MISMATCH_RETRY_MAX=3`
7. Motor stuck at 0%, target 100%, relays ON
8. O3 peaks at 0.10% then declines (no generator power)

## Firmware Fixes Applied (Session 14)

### 1. Remove DFRobot automated emergency stop  `[IMPLEMENTED]`
- **File**: `peripherals.c` `lab_o3_alarm_handler()`
- **Change**: Removed `o3_power_emergency_stop()` call.  Alarm callback now
  logs only.  DFRobot sensor is too noisy for automated safety actions.
- **Rationale**: Sensor reads >1 ppm noise transients regularly.  A reliable
  room O3 sensor (e.g., UV-based) would be needed before re-enabling.

### 2. Emergency stop routes through state manager  `[IMPLEMENTED]`
- **File**: `o3_power_control.c` `o3_power_emergency_stop()`
- **Change**: Calls `blocksi_state_set_power(0)` instead of
  `motor_pot_set_power(0)`.  This updates `target_pct` so the validator
  won't fight the stop.

### 3. Fix DATA telemetry column 15  `[IMPLEMENTED]`
- **File**: `main.c` `on_106h_sample()`
- **Change**: Column 15 (`power_target_pct`) now sends
  `blocksi_state_get()->power.target_pct` instead of `o3_power_get()`
  (which returns ADC actual, same as column 16).

### 4. Validator: remove retry limit, add hysteresis  `[IMPLEMENTED]`
- **File**: `blocksi_state.c` `blocksi_state_validate()`,
  `blocksi_state.h` constants
- **Changes**:
  - Removed `POWER_MISMATCH_RETRY_MAX`.  Validator retries indefinitely.
  - Added `POWER_MISMATCH_CLEAR = 2.5%` hysteresis — error triggers at
    5% but only clears at 2.5%.
  - Reads fresh ADC via `motor_pot_get_position_percent()` instead of
    stale `state.power.actual_pct` (updated only every ~2.5s).
  - Sends `DIAG,power_mismatch,...` and `DIAG,power_resolved,...` to PC
    via `lan_client_send_message()`.

## Dashboard Fixes Applied (Session 14)

### 5. DIAG handler in `_dispatch()`  `[IMPLEMENTED]`
- **File**: `blocksi_dashboard.py`
- **Change**: New `_handle_diag()` routes DIAG lines to:
  - In-memory log (as `warn`)
  - Toast notification
  - CSTR debug log file (if sequence active)

### 6. Fill-loop power watchdog  `[IMPLEMENTED]` (earlier in session)
- **File**: `blocksi_dashboard.py` fill loop
- **Change**: After `FILL_POWER_GRACE_SAMPLES=5`, checks every sample.
  If `power_actual_pct < 80%`, re-sends `cmd_set_power(100)` up to
  `FILL_POWER_RESEND_MAX=3` times with ring-buffer clear.

### 7. `power_actual_pct` in CSTR CSV  `[IMPLEMENTED]` (earlier in session)
- Added column to `_snap()` and `_write_cstr_csv()` for post-mortem
  analysis of power stability.

## Future Considerations  `[DEFERRED]`

### Validator interval tuning
- Current 1000 ms interval may be too frequent or too infrequent.
  After verifying the fix works, consider whether the interval should
  be adjusted.  Not blocking — the hysteresis already prevents flapping.

### DFRobot sensor replacement
- The DFRobot electrochemical sensor is unsuitable for safety-critical
  decisions.  If automated room-O3 shutdown is needed in future, install
  a UV-absorption sensor with stable baseline.

### Motor pot brake-then-coast
- `motor_pot_brake()` exists but is never called.  `motor_pot_stop()`
  uses coast (no holding torque).  Horizontal mounting makes drift
  unlikely, and manual tests show stability.  Monitor during testing —
  if drift reappears without DFRobot interference, consider using brake
  for the first 200 ms after positioning.

## Test Plan

1. Flash updated firmware (`idf.py build && idf.py -p COM3 flash`)
2. Connect USB monitor to observe ESP32 logs
3. Run CSTR fill+evac with standard parameters
4. Verify:
   - Power stays at target throughout fill
   - No DIAG messages during normal operation
   - DATA column 15 shows correct target (not ADC)
   - If manually triggered, emergency_stop updates target to 0
5. Optionally: inject artificial mismatch (manually turn pot knob) and
   confirm validator corrects it indefinitely + sends DIAG

## Related Files

| File | Relevance |
|------|-----------|
| `peripherals.c` | DFRobot alarm handler (emergency stop removed) |
| `o3_power_control.c` | `emergency_stop()` now uses state manager |
| `blocksi_state.c` / `.h` | Validator rewrite (hysteresis, fresh ADC, no limit) |
| `main.c` | DATA column 15 fix |
| `blocksi_dashboard.py` | DIAG handler, fill watchdog, CSV column |
| `motor_pot.c` | Motor driver (not modified, audited) |
| `Data/CSTR/2026-03-10_*` | Debug logs and checkpoint CSVs from 4 runs |
