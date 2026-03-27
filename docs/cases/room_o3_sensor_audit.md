# Room O3 Sensor (DFRobot SEN0321) Data Audit

**Date**: 2026-03-26
**Primary data**: `Data/Telemetry/2026-03-26_211057_Stream.csv` (865 rows, 21:10-21:39)
**Secondary data**: `Data/Telemetry/2026-03-26_184128_Stream.csv` (2754 rows, 18:41-20:13)
**Firmware**: `dfrobot_ozone.c`, `sensor_aggregator.c`, `peripherals.c`

---

## Executive Summary

The DFRobot SEN0321 room O3 safety sensor exhibits two distinct operating modes:

1. **Continuous mode** (correct): Smooth, monotonically varying values at 1-ppb
   resolution. Observed for ~2 minutes during an active O3 leak event.
2. **Quantized mode** (aberrant): Values snap to a handful of fixed ppb values
   (20, 355, 522, 3347, 3681) with no intermediate readings. This is the
   dominant behavior (~95% of operating time).

The transition from continuous to quantized mode happened mid-session,
corroborating the user's observation that "the sensor briefly showed sensible
data, then went chaotic."

---

## Data Analysis

### The Leak Event (211057_Stream.csv)

The most recent session captured a real O3 plumbing leak:

| Time | Phase | room_o3_ppm | Behavior |
|------|-------|-------------|----------|
| 21:10-21:17 | Baseline | 0.020 | Clean, stable |
| **21:17:33-21:18:35** | **Leak detection** | **0.021 -> 0.137** | **Smooth ramp, 1-ppb resolution** |
| 21:18:35-21:19:35 | Decay | 0.133 -> 0.031 | Smooth decay, still continuous |
| 21:20-21:22 | Return to baseline | 0.020 | Clean |
| **21:22-21:39** | **Quantized mode** | **{0.020, 0.355, 0.522}** | **Jumps between 3 fixed values only** |

Key observations:
- **The sensor CAN produce correct data** (Phase 2 proves the electrochemical
  cell works)
- **The transition to quantized mode is abrupt** (no gradual degradation)
- **Quantized mode persists indefinitely** once entered
- During the sensible phase, vessel O3 was 0.78-0.93% and power was ramping
  down from 100% to 0%

### Quantized Value Analysis (184128_Stream.csv)

The earlier session ran entirely in quantized mode. Value distribution:

| ppb | ppm | Hex | Byte pair | Count | % |
|-----|-----|-----|-----------|-------|---|
| 20 | 0.020 | 0x0014 | [0x00, 0x14] | 1141 | 41.4% |
| 355 | 0.355 | 0x0163 | [0x01, 0x63] | 870 | 31.6% |
| 3347 | 3.347 | 0x0D13 | [0x0D, 0x13] | 192 | 7.0% |
| 13 | 0.013 | 0x000D | [0x00, 0x0D] | 168 | 6.1% |
| 522 | 0.522 | 0x020A | [0x02, 0x0A] | 134 | 4.9% |
| 348 | 0.348 | 0x015C | [0x01, 0x5C] | 94 | 3.4% |
| 3681 | 3.681 | 0x0E61 | [0x0E, 0x61] | 91 | 3.3% |

**Critical observations:**

1. **Values are NOT multiples of 10 ppb** (the sensor's stated resolution).
   The continuous-mode data showed 1-ppb resolution. These quantized values
   represent raw register reads, not properly converted measurements.

2. **Spike high bytes match baseline low bytes**: 3347 ppb = [0x0D, 0x13] has
   high byte 0x0D = 13 decimal, and 13 ppb appears as a standalone reading 168
   times. Similarly, 522 = [0x02, 0x0A] has low byte 0x0A = 10, and 10 ppb
   appears 12 times. This suggests byte-level data corruption where bytes from
   different conversions are mixed.

3. **Spikes do NOT correlate with motor power** (mean power during spikes:
   50.1%, during baseline: 41.2%), ruling out simple EMI.

4. **The pattern is ~1/3 non-baseline per window**: In 30-sample windows, ~10
   samples show 0.355 ppm, ~3-5 show spikes, ~15-17 show baseline. This is
   consistent with only 1 valid read per aggregation window, with the others
   failing silently.

### Temporal Pattern

Consecutive values follow a repeating sequence: **B-M-B** (Baseline-Medium-
Baseline) is the most common triplet (380 occurrences), followed by M-B-B (363)
and B-B-M (323). The "medium" reading at 0.355 ppm appears every 2-3 samples
without any smooth transitions to or from it.

---

## Firmware Architecture

### Read Path

```
sensor_aggregator task (500ms loop)
  -> dfrobot_o3_read()
    1. sensor_write_reg(0x04, 0x01)   -- trigger passive conversion
    2. vTaskDelay(100ms)               -- wait for conversion
    3. sensor_read_reg(0x07, data, 2)  -- read passive data
       a. write [0x07] to set register pointer
       b. vTaskDelay(100ms)            -- unnecessary second delay
       c. read 2 bytes from device     -- gets [high, low] = ppb
    4. raw_ppb = (data[0] << 8) | data[1]
    5. ppm = raw_ppb * 0.001 + offset
  -> accumulate in s_agg (sum, count, min, max)

Every ~2s, main.c calls sensor_aggregator_get_and_reset()
  -> room_o3_ppm = sum / count (average)
  -> reset accumulators
  -> value included in DATA telemetry
```

### Configuration

- I2C bus: Port 0, GPIO 21 (SDA) / GPIO 22 (SCL), 100 kHz
- Address: 0x73
- Mode: Passive (trigger on demand)
- Auto-sample task: **Disabled** (`sample_interval_ms = 0`)
- Sole reader: `sensor_aggregator` task
- No other I2C devices on the bus (thermocouple uses SPI)

### Known Issues in Firmware

1. **Double 100ms delay**: `sensor_read_reg()` adds its own 100ms delay after
   setting the register pointer (line 120). This is unnecessary — the delay
   after the trigger (line 319 in `dfrobot_o3_read`) is what the sensor needs.
   The second delay wastes time but shouldn't cause data corruption.

2. **No raw byte logging**: The firmware only logs the computed ppm value at
   DEBUG level. There is no way to inspect the raw [data[0], data[1]] bytes to
   distinguish between sensor-side and firmware-side corruption.

3. **No I2C transaction atomicity**: The trigger-wait-read sequence spans ~300ms
   and is not protected by a mutex. While no other task reads this sensor, any
   future I2C device addition would create a race condition.

4. **Alarm callback disabled** (peripherals.c:167): Safety kill was disabled due
   to noise. The comment documents the known issue but no fix was applied.

---

## Hypotheses

### H1 (Primary): Passive register stale/corrupted latch

The SEN0321's internal MCU latches the conversion result into registers 0x07-
0x08 after a passive trigger. If the register latch is faulty or has a timing
window where it's inconsistent, an I2C read during that window returns stale or
byte-mixed data.

**Evidence for:**
- The quantized values are exact and repeating (355 ppb appears 870 times
  identically — not random noise)
- Byte-level analysis shows cross-contamination between high and low bytes of
  different conversions (0x0D from baseline appearing as high byte in spike)
- The sensor works correctly in continuous mode but fails after entering
  quantized mode — consistent with a state-dependent latch issue
- No external interference (no EMI correlation, no bus contention)

**Evidence against:**
- We cannot directly observe the register contents without raw byte logging
- The transition trigger is unclear (may be O3 level, temperature, or time)

### H2: Register pointer misdirection

After the trigger write to register 0x04, the sensor's internal register
pointer may not correctly reset to 0x07 when we write the pointer. If the I2C
write of [0x07] is acknowledged but internally ignored, the read returns data
from whichever register the pointer happens to be at (possibly 0x09-0x0A for
auto-mode data, or 0x05-0x06 for undefined registers).

**Evidence for:**
- Different quantized values could correspond to different register pairs
- The auto-mode registers (0x09-0x0A) may contain stale data from initialization
- Explains why values are not multiples of 10 ppb (reading wrong registers)

**Evidence against:**
- The ESP-IDF I2C write returns ESP_OK (ACK received), so the sensor
  acknowledged the register pointer write
- Would expect more random values if reading arbitrary registers

### H3: Electrochemical cell saturation / ADC boundary effect

After exposure to O3 above a threshold (~0.1 ppm), the electrochemical cell's
output voltage shifts to a region where the sensor's internal ADC operates near
a bit boundary. The ADC toggling between adjacent codes produces the bimodal
output (baseline vs. fixed offset).

**Evidence for:**
- Transition to quantized mode correlates with peak O3 (~0.137 ppm)
- The cell recovers eventually (subsequent sessions start in quantized mode
  but this may be because residual O3 keeps the cell in the affected range)

**Evidence against:**
- The continuous-mode data showed 1-ppb resolution at the same concentration
  range, so the ADC quantization is finer than the observed jumps
- The spike values (3347 ppb) are far too large for ADC boundary effects
- Does not explain the byte-level cross-contamination pattern

---

## Recommended Debugging Additions

### 1. Raw Byte Telemetry (Firmware)

Add raw byte logging to the DATA telemetry to distinguish sensor-side from
firmware-side corruption:

In `dfrobot_ozone.c`, add a public function:
```c
esp_err_t dfrobot_o3_read_with_debug(float *ppm, uint8_t *raw_bytes);
```

Store `data[0]` and `data[1]` from the I2C read and include them in the
aggregator output. Add two new columns to the DATA telemetry:
`room_o3_byte0`, `room_o3_byte1`

### 2. Register Dump Diagnostic Command

Add a LAN command `CMD,diag_room_o3_dump` that reads ALL sensor registers
(0x00-0x0F) and sends them as:
```
DIAG,room_o3_regs,0x00=XX,0x01=XX,...,0x0F=XX
```

This will reveal whether auto-mode registers contain stale data and whether the
register pointer is working correctly.

### 3. Conversion Delay Experiment

Add a LAN command `CMD,diag_room_o3_delay,<ms>` that overrides the 100ms
conversion delay for N readings and reports the results. Try 200ms, 500ms,
1000ms to test if insufficient conversion time causes the quantized behavior.

### 4. Continuous Read Mode

Add a diagnostic that takes 100 consecutive reads (trigger + read each time)
with the results logged as DIAG messages. This will show whether the quantized
behavior is consistent within a burst or intermittent.

### 5. I2C Error Rate Counter

Add per-call error counting (currently only tracks `consecutive_errors`). Track
separately:
- Trigger write failures
- Register pointer write failures
- Data read failures
- Successful reads returning identical values

---

## Testing Procedures

### Test 1: Zero-Air Baseline (No O3 generation)

**Purpose**: Establish baseline sensor behavior without O3.
**Procedure**:
1. Ensure no O3 generation for 30+ minutes
2. Stream telemetry for 10 minutes with raw byte logging enabled
3. Analyze: Are values continuous near 0 ppb, or quantized?

**Expected outcome**: If H1/H2, still quantized (state-dependent). If H3, may
show continuous behavior.

### Test 2: Conversion Delay Sweep

**Purpose**: Test if conversion timing causes quantization.
**Procedure**:
1. Use `diag_room_o3_delay` command to set delay to 200ms
2. Stream 50 readings, log raw bytes
3. Repeat with 500ms, 1000ms delays
4. Compare quantization rates across delays

**Expected outcome**: If H2 (timing), longer delays will produce more
continuous values. If H1 (latch), no improvement.

### Test 3: Register Dump During Both Modes

**Purpose**: Compare register contents during continuous vs. quantized operation.
**Procedure**:
1. Start streaming in clean air (should trigger continuous mode if sensor
   recently power-cycled)
2. Run `diag_room_o3_dump` — capture register snapshot
3. Expose sensor to O3 (open vessel briefly)
4. Wait for transition to quantized mode
5. Run `diag_room_o3_dump` again
6. Compare: Did auto-mode registers (0x09-0x0A) change? Did mode register
   (0x03) change?

**Expected outcome**: Will reveal if register pointer or mode state changed.

### Test 4: Power Cycle Recovery

**Purpose**: Test if power cycling the sensor resets it to continuous mode.
**Procedure**:
1. Confirm sensor is in quantized mode
2. Power cycle the sensor (toggle VCC via a GPIO relay, or re-initialize)
3. Immediately start streaming
4. Observe: Does it start in continuous mode?

**Expected outcome**: If the sensor's internal MCU state is the issue, a power
cycle should reset it to continuous mode temporarily.

### Test 5: Alternative Sensor

**Purpose**: Determine if the issue is specific to this sensor unit.
**Procedure**:
1. Replace the SEN0321 with a new unit (same model)
2. Run Tests 1-3
3. Compare results

**Expected outcome**: If defective unit, new sensor works correctly. If design
flaw, same behavior on new unit.

---

## Immediate Mitigation

Until the root cause is resolved, the telemetry data from this sensor should be
treated as unreliable. The current firmware correctly disables automated safety
actions based on this sensor (peripherals.c:167). The handheld O3 sniffer
remains the primary safety tool for leak detection.

If the sensor shows continuous-mode behavior (values like 0.033, 0.047, 0.092),
those readings can be trusted as they match the expected electrochemical cell
response curve.

If the sensor shows quantized-mode behavior (values snapping between exact
0.020 / 0.355 / 0.522 / 3.347 ppm), discard all readings as unreliable.

A simple firmware filter could detect quantized mode by checking: if the last N
readings are all within {20, 355, 522, 3347, 3681} ppb (exact match), flag the
sensor as `DEGRADED` in telemetry.
