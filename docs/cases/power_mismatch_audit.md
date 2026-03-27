# Case File: Power Mismatch Audit

**Status**: RESOLVED — brake-mode fix confirmed, power tracking within ±1%
**Filed**: 2026-03-24
**Updated**: 2026-03-27 (fix verified in lab)
**Resolved**: 2026-03-27
**Related**: `docs/cases/cstr_power_stability.md` (Session 14 fixes)

## Objective

Characterize the `DIAG,power_mismatch` warnings from the ESP32 state validator.
Determine root cause(s) and formulate actionable hypotheses for lab verification.

## Data Sources

Three post-fix calibration runs (all collected after Session 14 firmware fixes):

| File | Date | Flow | Temp (C) | Notes |
|------|------|------|----------|-------|
| `Calibration/2026-03-22_213345_PowerO3Cal_3.5Lpm_95O2.csv` | Mar 22 | 3.5 Lpm | 32.7–35.1 | **Best tracking** |
| `Calibration/2026-03-23_002734_PowerO3Cal_4Lpm_95O2.csv` | Mar 23 | 4.0 Lpm | 38.0 | Moderate deficit |
| `Calibration/2026-03-24_233518_PowerO3Cal_3.5Lpm_95O2.csv` | Mar 24 | 3.5 Lpm | 30.7–31.7 | **Worst tracking** |

CSV columns: `step_idx,sample_num,o3_pct,temp_c,power_actual,air_comp,phase,power_target`

## Firmware Architecture (Relevant Code)

| Component | File | Key Detail |
|-----------|------|------------|
| Motor driver | `motor_pot.c` | Closed-loop goto with 25ms iterations, **coast stop** (no brake/holding torque) |
| ADC reading | `motor_pot.c:read_adc_averaged()` | 3-sample average, 1ms between samples |
| Position mapping | `motor_pot.c:motor_pot_get_position_percent()` | Linear: `(adc - 100) / 3896 * 100` |
| Validator | `blocksi_state.c:blocksi_state_validate()` | Every 1000ms; waits 1500ms settle; 5% trigger / 2.5% clear hysteresis |
| Power command | `blocksi_state.c:blocksi_state_set_power()` | Sets target, calls `o3_power_set()` (blocking), clears `motor_moving` |
| Telemetry | `main.c:on_106h_sample()` | `power_target_pct` from state manager, `power_actual` from `motor_pot_get_state()` |

### Key Architectural Observations

1. **Coast stop, not brake**: `motor_pot_stop()` sets both H-bridge pins LOW (coast).
   No holding torque after positioning. `motor_pot_brake()` exists but is never called.
2. **3-sample ADC averaging**: `motor_pot_get_position_percent()` takes only 3 ADC1 samples.
   ESP32 ADC1 is documented to have ±10–30 count noise on 12-bit (0–4095) range.
3. **Linear pot assumption**: ADC-to-percent mapping is linear. If the PRM-162 has
   nonlinear taper, mapping error would be position-dependent. However, both the
   motor control loop and the readback use the same mapping, so this should cancel
   during `goto_position()`.
4. **Settle time = 1500ms**: Validator waits 1.5s after `last_command_ms` before
   checking. During this window the wiper is undriven (coast).

## Data Characterization

### Finding 1: Systematic Under-Delivery at Mid-to-High Power

Error = `power_target - power_actual` is **overwhelmingly positive** (actual < target):

| Target % | Mar 22 Error | Mar 23 Error | Mar 24 Error |
|----------|-------------|-------------|-------------|
| 20 | +0.2 | +3.0 | -0.2 |
| 40 | +1.5 | +4.0 | +4.5 |
| 60 | +3.3 | +6.4 | +6.3 |
| 80 | +0.8 | +6.6 | +6.5 |
| 100 | +0.2 | +5.2 | +8.9 |

The error is **directional** (always toward lower actual) and **power-level-dependent**
(grows with target). Mar 22 tracked nearly perfectly; Mar 23–24 show large deficits.

### Finding 2: Bimodal ADC Distribution (Mar 23–24 Only)

In the random phase (20 samples/step), Mar 23 and Mar 24 files show readings
toggling between two discrete values separated by ~5–8%:

- Target 39%: readings alternate ~34% and ~39% (File 1/Mar 24)
- Target 63%: readings alternate ~57% and ~64%
- Target 82%: readings alternate ~74% and ~83%

**This pattern is NOT present in Mar 22** (File 3), which shows tight unimodal
distributions (spread ≤ 3.6%). Something changed between Mar 22 and Mar 23.

### Finding 3: Progressive Degradation Across Sessions

Target = 100% actual readings:
- Mar 22: **99.8%** (nearly perfect)
- Mar 23: **94.8%** (5% deficit)
- Mar 24: **91.1%** (9% deficit)

This is a 3-day trend of increasing under-delivery at high power.

### Finding 4: Baseline ADC Noise (Mar 24 Only)

At target = 0%, Mar 24 shows `power_actual` readings of 0.0–1.7%.
Mar 22 and Mar 23 read 0.0%. This suggests a DC offset or noise floor change.

### Finding 5: Threshold Violations

Using `POWER_MISMATCH_TOLERANCE = 5%`:
- **Mar 22**: Essentially zero violations. All readings within tolerance.
- **Mar 23**: Many violations starting at target ≥ 40%. Most high-power readings fail.
- **Mar 24**: Violations from target ≥ 35%. Nearly every reading at target ≥ 80% fails.

### Finding 6: Overall MAE by Session

| Session | Flow | Temp (C) | Overall MAE (%) | Bias (%) |
|---------|------|----------|-----------------|----------|
| Mar 22 | 3.5 Lpm | 32.7–34.1 | ~0.9 | −0.1 |
| Mar 23 | 4.0 Lpm | 38.0 | ~3.2 | −2.8 |
| Mar 24 | 3.5 Lpm | 30.7 | ~4.9 | −4.3 |
| k_d_cal (Mar 10, fill at 100%) | 4.0 Lpm | 39.5 | ~0.35 | −0.35 |

### Finding 7: Transition vs Steady-State Error

First sample after a power step change carries ~4.5× the error of settled
samples. Quantified from Mar 22 random phase (20 samples/step):
- **Transition (sample 0)**: mean |error| ≈ 1.8%
- **Settled (samples 5–19)**: mean |error| ≈ 0.4%

This is directly relevant to DIAG,power_mismatch: the 1500ms
`POWER_SETTLE_TIME_MS` may be insufficient if the motor physically reaches
target but ADC readings take additional time to stabilize.

### Finding 8: Sweep Direction Asymmetry

In Mar 22 (File A, best tracking):
- **Sweep up**: bias = −0.6% (undershoot — actual below target)
- **Sweep down**: bias = +0.5% (overshoot — actual above target)

This hysteresis reversal is consistent with motor backlash or pot wiper
friction. In Mar 23–24 the systematic undershoot was so large it overrode
the sweep-down overshoot.

### Finding 9: Worst Power Bands

Across all files, mismatch peaks in two bands:
- **50–70%**: 5–8% error (possible PWM regime change near 50%)
- **85–100%**: 5–9% error (saturation effects, motor near end-stop)

## Hypotheses

### H1: Coast-Mode Wiper Drift (Most Likely)

**Claim**: After `motor_pot_goto_position()` positions the wiper, coast stop
(no holding torque) allows the wiper to drift toward lower resistance under
mechanical restoring forces (spring return, friction asymmetry, gravity).

**Supports**:
- Error is directional (always toward lower actual)
- Error increases with position (more potential energy to drift back)
- The 1500ms settle window allows drift to accumulate before validation
- Coast mode is explicitly chosen over brake mode in the driver

**Falsification test**: Run `CMD,diag_power_drift,<target>` to log ADC at
0ms, 100ms, 500ms, 1s, 2s, 5s after motor stop. If readings decrease over
time, drift is confirmed. Compare with brake mode.

### H2: Insufficient ADC Averaging Amplified by Noise

**Claim**: 3-sample ADC averaging is insufficient for ESP32 ADC1 noise
characteristics. At higher pot positions, source impedance increases, making
ADC readings noisier. The bimodal pattern in Mar 23–24 is aliased noise from
a 3-sample window that samples different noise states.

**Supports**:
- ESP32 ADC1 is documented as noisy (±10–30 counts)
- 3 samples is very few — std of a 3-sample mean can still be large
- Bimodal pattern suggests readings flip between two noise clusters
- Noise worsens at higher pot positions (higher impedance)

**Falsification test**: Run `CMD,diag_power_noise,<N>` to take N (e.g., 50)
ADC samples at current position. Report min/max/mean/std/histogram. If std
is >1% (≈39 ADC counts), noise is a significant contributor.

### H3: Progressive Hardware Degradation

**Claim**: The PRM-162 pot–motor assembly is mechanically degrading. Wiper
contact resistance is increasing, motor gearing is loosening, or friction
is changing. This explains the progressive worsening across 3 sessions.

**Supports**:
- Clear performance degradation: Mar 22 (good) → Mar 23 (moderate) → Mar 24 (poor)
- The degradation is systematic, not random
- Temperature alone doesn't explain it (Mar 24 was coolest at 30.7°C but worst)

**Falsification test**: Run the same `CMD,diag_power_drift` at 100% on
consecutive sessions. If the deficit increases linearly over sessions, hardware
degradation is confirmed. If a power cycle or warm-up restores performance,
the issue is thermal.

## Diagnostic Commands Added

### `CMD,diag_power_drift,<target_pct>`

Moves motor to `<target_pct>`, then takes ADC readings at t=0, 100ms, 500ms,
1s, 2s, 5s after stop. Sends each as:
```
DIAG,drift_sample,target=<T>,t_ms=<t>,adc=<raw>,pct=<pct>
```
After all samples, sends:
```
DIAG,drift_summary,target=<T>,initial_pct=<p0>,final_pct=<p5s>,drift=<delta>
```

### `CMD,diag_power_noise,<num_samples>`

Takes `<num_samples>` ADC readings at current position (no motor movement).
Reports:
```
DIAG,noise_stats,n=<N>,mean=<mean>,std=<std>,min=<min>,max=<max>,range_pct=<range>
```

### Enhanced `DIAG,power_mismatch` (validator)

Existing message enhanced with additional fields:
```
DIAG,power_mismatch,target=<T>,actual=<A>,retry=<R>,adc_raw=<raw>,settle_ms=<ms>
```

## Test Plan (Next Lab Session)

1. **Power up, connect dashboard, verify telemetry flowing**
2. **H1 — Drift test**:
   - `CMD,diag_power_drift,50` — mid-range position
   - `CMD,diag_power_drift,75` — high position
   - `CMD,diag_power_drift,100` — max position
   - Look for: ADC decreasing over the 5-second window
3. **H2 — Noise test**:
   - Move to 50%: `CMD,power_set,50`, wait 2s
   - `CMD,diag_power_noise,50` — 50 samples at mid-range
   - Move to 100%: `CMD,power_set,100`, wait 2s
   - `CMD,diag_power_noise,50` — 50 samples at max
   - Look for: std > 1% or bimodal min/max spread
4. **H3 — Degradation baseline**:
   - Run drift test at 100% at start and end of session
   - Compare with Mar 22 baseline (99.8% at target 100)
5. **Brake mode experiment** (if H1 confirmed):
   - Modify `motor_pot_stop()` to use brake instead of coast
   - Repeat drift test — if drift disappears, brake is the fix

## Related Files

| File | Relevance |
|------|-----------|
| `motor_pot.c` / `.h` | Motor driver, coast/brake stop, ADC averaging |
| `blocksi_state.c` / `.h` | Validator (mismatch detection, hysteresis, DIAG send) |
| `o3_power_control.c` | Power API wrapper |
| `main.c` | DATA telemetry (columns 15–17), command handler |
| `blocksi_pins.h` | GPIO34 ADC, deadband=20, ADC range 100–3996 |
| `Calibration/2026-03-2*_*.csv` | Source data for this analysis |
| `docs/cases/cstr_power_stability.md` | Prior power stability case (Session 14) |

---

## Session: 2026-03-26 Afternoon (±1% — Transient Resolution)

The power mismatch issue **self-resolved without corrective action**. A
calibration run (`2026-03-26_162744_PowerO3Cal_3.75Lpm_95O2.csv`, 1,011 samples)
demonstrated ±1% tracking. The diagnostic commands were not exercised. The
improvement from Mar 24 (MAE ~4.9%) to this run occurred without any firmware
or hardware intervention.

---

## Session: 2026-03-26 Evening — ISSUE RECURRED

### Data Sources

| File | Time | Type | Flow | Notes |
|------|------|------|------|-------|
| `Power-O3_cal/2026-03-26_191615_PowerO3Cal_3.50Lpm_95O2.csv` | 19:16 | Cal run | 3.50 Lpm | **134 mismatch alerts** |
| `Validation/2026-03-26_192632_Verification_50hold_3.50Lpm_PASS.csv` | 19:26 | Verification | 3.50 Lpm | Passed but with continuous drift cycling |
| `Diagnostics/2026-03-26_184503_diag.log` | 18:45–19:26 | DIAG log | — | 134 mismatch + 84 resolved events |
| `Telemetry/2026-03-26_184128_Stream.csv` | 18:41+ | Stream | — | 1,559 telemetry samples |

### DIAG Log Analysis

The DIAG log provides **the first real-time evidence of the drift-correction
cycle** thanks to the diagnostic instrumentation added in Session 15.

**Overall statistics:**
- 134 `power_mismatch` events, 84 `power_resolved` events
- **128 of 134 mismatches are downward** (actual below target) — 96% directional
- Only 6 events show actual above target (transients after correction overshoot)
- Mean drift at detection: **−6.1%** (actual below target)
- Max drift at detection: **−12.1%** (target=73, actual=58.1)
- Settle time range: 1,529ms – 65,999ms (mean 6,584ms)

### Finding 10: Repeating Sawtooth Drift-Correction Cycle

The clearest evidence comes from the verification hold phase (target=50%,
held for ~3 minutes starting at 19:22). The DIAG log shows a continuous
cycle:

```
[19:23:03] mismatch target=50, actual=44.5, settle=4999ms
[19:23:05] resolved target=50, actual=48.5
[19:23:26] mismatch target=50, actual=43.6, settle=23000ms   ← drifted again
[19:23:32] resolved target=50, actual=51.8
[19:24:35] mismatch target=50, actual=45.0, settle=65999ms   ← 66 SECONDS of drift
[19:24:38] resolved target=50, actual=51.3
[19:24:39] mismatch target=50, actual=44.6, settle=3000ms    ← drifted within 3s!
[19:24:44] resolved target=50, actual=51.4
[19:25:01] mismatch target=50, actual=44.3, settle=18999ms
...pattern continues for entire hold...
```

**Pattern:** Motor is driven to ~50%, motor stops in coast mode, wiper drifts
down to ~44–45% over 3–66 seconds, validator detects >5% error, re-drives
motor to ~50–52%, motor stops, wiper drifts down again. Endlessly.

### Finding 11: Drift Magnitude Scales with Position

Consistent with the earlier data — the drift is not constant, it increases
with wiper position:

| Target | Typical drift to | Deficit | Events |
|--------|-----------------|---------|--------|
| 37–43% | 31–37% | ~6% | 5 |
| 50–59% | 44–53% | ~6% | 44 |
| 60–73% | 53–66% | ~7% | 38 |
| 86–93% | 78–87% | ~8% | 34 |
| 100% | 91–95% | ~6–9% | 2 |

### Finding 12: Drift is NOT Instantaneous

The settle times in the DIAG messages reveal the drift timeline:
- **57 events** with settle > 5 seconds — motor was at rest for >5s before drift detected
- **20 events** with settle > 10 seconds
- **2 events** with settle > 30 seconds (one at 40s, one at 66s)
- Some events at settle=3000ms — drift happening within 3 seconds of positioning

This confirms the wiper physically moves after the motor stops. It is not
an ADC noise artifact — the drift takes seconds to develop and is always
in the same direction (lower resistance / lower position).

### Finding 13: Correction Works but Cannot Hold

Every `power_resolved` event shows the validator successfully re-positions
the wiper (actual matches target within 2.5%). But the correction is
immediately undermined by the next drift cycle. The validator is fighting
a losing battle against a continuous downward force.

### Finding 14: Calibration Impact

The calibration run at 3.50 LPM shows ~10% variability in power tracking:

| Target % | Mean Actual | Mean Error | Max Error | Range |
|----------|-------------|------------|-----------|-------|
| 37 | 36.8 | +0.2 | **+9.2** | 11.7 |
| 52 | 48.9 | +3.1 | **+8.1** | 14.7 |
| 60 | 55.2 | +4.8 | **+7.7** | 12.0 |
| 73 | 67.6 | +5.4 | **+14.9** | 16.9 |
| 86 | 78.4 | +7.6 | **+13.9** | 16.1 |
| 93 | 89.9 | +3.1 | **+12.0** | 13.6 |

The large ranges (up to 16.9%) directly correspond to samples caught mid-drift
vs. immediately after correction.

### Finding 15: O₃ Concentration Impact

The power instability is measurably affecting ozone production. During the
verification hold phase, O₃ concentration fluctuates in response to the
power sawtooth — the system cannot maintain a stable ozone output.

---

## Root Cause Analysis (Updated)

### H1: Coast-Mode Wiper Drift — **CONFIRMED (Primary Cause)**

The DIAG data now conclusively demonstrates H1:

1. **Motor positions correctly** — every `power_resolved` event shows target reached
2. **Motor stops in coast mode** — `motor_pot_stop()` sets both AIN1=0, AIN2=0
3. **DRV8833 outputs go Hi-Z** — coast mode disconnects the motor from supply
4. **Wiper drifts downward** — consistent, directional, position-dependent
5. **Drift develops over seconds** — settle times of 3s to 66s before detection
6. **Validator corrects, drift recurs** — confirmed sawtooth cycling

**The smoking gun is the verification hold phase at target=50%.** The motor
is positioned once and never commanded again, yet the wiper drifts from 50%
to ~44% repeatedly, with the validator catching and correcting each time.

### Possible Drift Mechanisms

The user raises a valid concern: **the wiper should only move if current
flows through the motor windings.** In true coast mode (Hi-Z outputs), no
current should flow. So what could be driving the motor?

#### Mechanism A: LEDC PWM Glitch at Duty=0 (Firmware Concern)

The motor is stopped by setting LEDC PWM duty to 0 on both channels, NOT by
configuring the GPIOs as plain digital outputs. The LEDC peripheral remains
active. On ESP32, `ledc_set_duty(channel, 0)` should produce a constant-LOW
output, but:

- **ESP32 LEDC errata**: Some ESP32 revisions have known issues where LEDC
  at duty=0 or duty=max can produce occasional glitch pulses at the PWM
  frequency. A single glitch pulse at 1kHz on one channel would apply a
  1ms drive pulse to the motor — enough to move the wiper slightly.
- **Race condition**: If both channels don't update simultaneously, there
  could be a brief moment where one channel transitions while the other is
  still at a non-zero duty.

**Test**: After `motor_pot_stop()`, detach the LEDC channels and set GPIOs
as plain digital outputs LOW. If drift stops, the LEDC peripheral is the
source.

#### Mechanism B: DRV8833 Leakage Current Through Body Diodes

Even in coast mode, the DRV8833's half-bridge FETs have parasitic body diodes.
If there is any back-EMF from the motor (even from vibration, thermal EMF, or
mechanical restoring force on the pot), current can flow through these diodes
and create a small but sustained driving torque.

The PRM-162 has a linear pot with a metal wiper and a worm gear mechanism.
If the worm gear has any backlash and the wiper contact creates a mechanical
restoring force, the body diode leakage path could allow slow movement.

**However**: body diode leakage would typically be in the nA–µA range, which
is unlikely to provide enough torque to move a worm-geared motor. This
mechanism is less likely than A.

#### Mechanism C: DRV8833 SLP Pin Always Enabled

The SLP (sleep) pin is hardwired to 5V. When SLP is active, the DRV8833
internal logic is powered and monitoring the IN1/IN2 pins. If there is
any noise on the ESP32 GPIO25/26 lines (crosstalk from adjacent GPIO,
power supply ripple coupled to the LEDC output driver), the DRV8833
could interpret noise as a brief drive command.

With SLP=LOW, the DRV8833 would fully disable its output stage and ignore
all input signals. But SLP is tied HIGH.

**Test**: The SLP pin cannot easily be tested without hardware modification.

#### Mechanism D: Brake Mode as the Solution

Brake mode (AIN1=HIGH, AIN2=HIGH) shorts both motor terminals through the
DRV8833's low-side FETs. This provides:
1. **Electromagnetic braking** — any motor rotation generates current through
   the short, creating opposing torque
2. **No Hi-Z path** — eliminates body diode leakage concern
3. **Active holding** — motor windings resist rotation

The `motor_pot_brake()` function already exists but is never called.

---

## Proposed Fix

### Immediate: Switch from Coast to Brake Stop

Change `motor_pot_stop()` to use brake mode instead of coast:

```c
// BEFORE (coast — Hi-Z, no holding torque):
void motor_pot_stop(void) {
    motor_pot_set_motor(MOTOR_DIR_STOP, 0, MOTOR_DECAY_FAST);
}

// AFTER (brake — active holding via low-side short):
void motor_pot_stop(void) {
    motor_pot_set_motor(MOTOR_DIR_BRAKE, 0, MOTOR_DECAY_FAST);
}
```

This is a one-line change. The `motor_pot_brake()` function and the
`MOTOR_DIR_BRAKE` case in `motor_pot_set_motor()` already exist. Brake
mode sets both AIN1 and AIN2 to duty=255 (HIGH), which activates both
low-side FETs in the DRV8833, shorting the motor windings and preventing
any rotation.

**Risks**:
- Slightly higher quiescent current draw (~1–5mA through the FETs)
- Motor windings under constant short — not a concern for a 5kΩ pot motor
- No overheating risk at zero rotation

### Secondary: LEDC Detach After Stop (Belt-and-Suspenders)

After confirming brake mode works, optionally add LEDC channel detach
after stop to eliminate any possibility of PWM glitches:

```c
void motor_pot_stop(void) {
    motor_pot_set_motor(MOTOR_DIR_BRAKE, 0, MOTOR_DECAY_FAST);
    // Optionally: detach LEDC and set GPIO directly
    // ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);  // HIGH
    // ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);  // HIGH
}
```

### Verification Protocol

After applying the brake fix:
1. Run `CMD,diag_power_drift,50` — check if drift disappears with brake
2. Run `CMD,diag_power_drift,75` — check mid-range
3. Run `CMD,diag_power_drift,100` — check high-range
4. Run a full `PowerO3Cal` — MAE should return to <1%
5. Run verification — hold phase should show stable power without correction cycles

---

## Resolution Confirmation (2026-03-27)

The brake-mode fix (`MOTOR_DIR_BRAKE` in `motor_pot_stop()`) was verified in lab.
Power tracking is now within ±1% — no sawtooth drift cycling, no mismatch events.

**What fixed it**: One-line change in `motor_pot.c` — `motor_pot_stop()` uses
`MOTOR_DIR_BRAKE` instead of `MOTOR_DIR_STOP`. Brake mode shorts the motor
windings through the DRV8833 low-side FETs, providing electromagnetic holding
torque that prevents wiper drift.

**Key takeaway**: `motor_pot_brake()` already existed in the codebase but was
never called. The function's existence was a signal that braking was an intended
capability. Multiple audits missed this. See `docs/pitfalls.md` §7 for the
generalized lesson.
