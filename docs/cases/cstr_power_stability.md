# Case File: CSTR Power Stability

**Status**: RESOLVED — power stability fixed (Session 14), fill stopping criteria  
improved (Session 15)  
**Filed**: 2026-03-10  
**Last updated**: 2026-03-11

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

### Self-calibrating probe evac for stopping criteria  `[PROPOSED]`
- Add an optional phase at the start of the CSTR sequence: inject a
  small O3 bolus (e.g. ramp to ~50% power briefly to reach ~0.5% vol
  in the vessel), then evacuate back to 0% with flow-only.
- Measure the asymptotic slope as O3 approaches zero during this
  mini-evac.  Use this measured slope as the fill stopping threshold
  for the subsequent full fill phase.
- Advantage: the stopping criterion is self-calibrated to the current
  hardware configuration (flow rate, plumbing, sensor response) each
  run — no hardcoded constants needed.
- Deferred: hardcode from existing evac data first, revisit if
  hardware or plumbing changes.

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
| `blocksi_dashboard.py` | DIAG handler, fill watchdog, CSV column, stopping criteria |
| `motor_pot.c` | Motor driver (not modified, audited) |
| `Data/CSTR/2026-03-10_*` | Debug logs and checkpoint CSVs from 4 runs |

## Session 15: Fill Stopping Criteria Fix

### Problem
After power stability was confirmed working (Session 14), the first
successful CSTR run (`2026-03-10_221012_CSTR_4Lpm.csv`) revealed the
fill phase stops too early.  O3 was still rising at the end of fill,
and even continued rising after evac began:

- Fill: 186 samples, final O3 = 1.5500%
- First evac samples: O3 rose to 1.5600% (proving fill stopped prematurely)
- Rolling avg absolute delta at fill end: 0.00111 (still changing)
- Progress bar target: 1.8404% (sigmoid model at 100% power) — too high

### Root Cause
Two issues:

1. **Range-based stopping too lenient for quantized signals**:
   The 106-H sensor has 0.01% resolution.  A slowly rising signal
   (e.g., +0.01% every 5-7 samples) can have a 30-sample window range
   of 0.04% — below the 0.05% threshold — while still clearly trending
   upward.  Range catches variance but misses monotonic trends.

2. **Wrong fill target value**:
   `target_o3 = predict_o3_from_power(100, flow)` returns the sigmoid
   model's inlet concentration (1.8404%), not the vessel steady-state
   asymptote.  With O3 decay in the vessel, the actual asymptote is
   `C_ss = C_in / (1 + k_d × τ)` ≈ 1.58% (from CSTR model fit).

### Fixes Applied  `[IMPLEMENTED]`

#### 8. Evac-derived slope-based fill stopping criterion
- **File**: `blocksi_dashboard.py`
- **Principle**: The evac's asymptotic behaviour as O3 approaches zero
  is the same physical process (CSTR washout) as the fill's approach
  to C_ss, just in reverse.  The evac data from the 2026-03-10 CSTR run
  directly measures what "converged" looks like in this system.
- **Evac analysis** (45-sample sliding windows):
  - O3 0.02→0.01%: |slope| ≈ 0.000325 (barely still decaying)
  - O3 0.01→0.01%: |slope| = 0.000000 (converged)
  - Fill at premature stop: slope = 0.00184 — **6× above** convergence
- **Changes**:
  - `FILL_STEADY_COUNT` 30 → 45 (longer window catches slow trends)
  - `FILL_STEADY_RANGE` 0.05 → 0.08 (range is sanity bound only)
  - `FILL_STEADY_SLOPE = 0.0003` — derived from evac convergence zone
  - Removed `FILL_MIN_SAMPLES` — redundant; the 45-sample window
    naturally prevents triggering before ~112 s
  - Log lines include slope value for post-mortem analysis.

#### 9. Validation-measured max O3 for fill target
- **File**: `blocksi_dashboard.py`
- **Change**: `target_o3` now comes from the most recent 100% validation
  PASS CSV (`_find_valid_cert(100, flow, max_age_h=720)`).  Reads the
  `target` phase stable tail (skip `VAL_TRANSIENT_SKIP`) and uses
  `mean(o3_pct)` — a direct-to-sensor measurement held at steady-state.
- **Fallback**: sigmoid prediction if no validation cert exists.
- **Rationale**: Avoids circular reasoning (model predicts → model
  validates → model predicts).  Validation is a measured value.

### Validation
With the previous run's data (45-sample window ending at sample 186):
- Fill slope = 0.00184 — 6× above threshold 0.0003 → would NOT have stopped
- Fill would have continued until true steady-state
- Validation mean O3 at 100%: 1.9262% (vs old sigmoid prediction 1.8404%)
