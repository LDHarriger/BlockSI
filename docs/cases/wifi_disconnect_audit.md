# WiFi Disconnect Resilience Audit (WP-0)

> Date: 2026-03-22
> Scope: ESP32 firmware behavior during TCP disconnection for 30+ minute batch sterilization

---

## 1. Recipe Executor During Disconnect

**Finding: Recipe continues running unaffected by TCP state.**

The `seq_executor.c` executor task (`executor_task()`, line 296) runs as an
independent FreeRTOS task.  It iterates through steps, calls
`blocksi_state_set_power()` (line 355) and counts samples via
`seq_sensor_get_sample_count()` (line 368).  None of these depend on TCP
connectivity.

The only TCP interaction is `send_msg()` (line 472), which calls
`lan_client_send_message()`.  That function checks `s_lan.connected` (line 436
of `lan_client.c`) and returns `ESP_ERR_INVALID_STATE` if disconnected.
**The return value is ignored by `send_msg()`** -- messages are fire-and-forget.

**Conclusion**: A running recipe survives TCP disconnect completely.  Power
levels continue stepping, sample-counted holds continue counting, and the
recipe completes normally.  However:

- All `SEQ,<type>,SAMPLE,...` messages sent during disconnect are **lost forever**
  -- they are NOT cached in the data_cache ring buffer.
- `SEQ,<type>,COMPLETE` or `SEQ,<type>,ABORTED` may also be lost if TCP is
  still down when the recipe finishes.

**Risk**: HIGH.  For a 30-min sterilization batch, the PC loses all per-sample
dosimetry data for the disconnected period.  This makes the batch un-certifiable.

---

## 2. Mid-Recipe Power Override

**Finding: `CMD,power_set` is ACCEPTED during a running recipe -- no guard exists.**

The `lan_command_handler()` in `main.c` (line 464) handles `power_set` by
calling `blocksi_state_set_power()` (line 477).  There is **no check** for
`sequence_runner_is_active()` or `seq_executor_get_state()`.

The `sequence_runner_is_active()` check is only used on the **PC dashboard
side** (UI lockout) -- it is never enforced in firmware command dispatch.

Meanwhile, the executor task also calls `blocksi_state_set_power()` (line 355
of `seq_executor.c`) at each step transition.  If the PC sends `power_set`
mid-step, both the LAN handler and the executor will race on
`s_state_mgr.state.power.target_pct`.  The executor's next step transition
will overwrite the PC's value, but during the current step's hold period, the
**PC's override will persist** -- the executor only sets power once per step.

**Behavior**: Accept (silently). No crash, no rejection.  The recipe continues
with the overridden power level until the next step transition.

**Proposed firmware modification**:
Add a guard in `lan_command_handler()` for `power_set`:
```
if (seq_executor_get_state() == SEQ_EXEC_RUNNING ||
    seq_executor_get_state() == SEQ_EXEC_WAITING_CONFIRM) {
    snprintf(response, response_size, "rejected_sequence_active");
    return false;
}
```
Similarly guard `relay_set` to prevent relay toggling during recipes.
Exception: `sequence_abort` and `power_set,0` (E-STOP) should always be
accepted.

---

## 3. Backfill Completeness

**Finding: Ring buffer is INSUFFICIENT for a 30-minute disconnect.**

- Ring buffer: `DATA_CACHE_SLOTS = 500` (`data_cache.h`, line 28)
- 106-H sample rate: ~2.5 seconds per sample
- 30-minute disconnect: 1800s / 2.5s = **720 samples**
- Buffer capacity: 500 samples = ~20.8 minutes at 2.5s/sample

**At 720 samples needed vs 500 slots, the oldest 220 samples are silently
overwritten** (ring buffer behavior, `data_cache.h` line 43: "oldest entry is
silently overwritten").

Additionally, backfill only covers `DATA` telemetry lines.  It does **NOT**
cache:
- `SEQ,<type>,SAMPLE,...` messages (recipe per-sample data)
- `SEQ,<type>,STEP,...` messages (step transitions)
- `SEQ,<type>,COMPLETE` / `ABORTED` messages

**The regular DATA telemetry (18-field format) IS cached** and backfilled on
reconnect.  The backfill protocol (`main.c` lines 375-404, `tcp_server.py`
lines 130-180) works correctly: STATE push first, time_sync, then
BACKFILL_START/DATA.../BACKFILL_END.

**Risk assessment for batch sterilization**:
- DATA backfill covers ~20.8 min max (loses earliest data for longer outages)
- SEQ SAMPLE data is completely lost during disconnect (not cached at all)
- This means a batch sterilization recipe's dosimetry data has a gap

**Recommendations**:
1. Increase `DATA_CACHE_SLOTS` to 1000 (covers ~41 min, cost: ~160 KB RAM)
2. Add a second ring buffer for SEQ messages, OR have seq_executor cache its
   own samples (critical for process_batch)
3. Alternative: have ESP32 dosimetry accumulate dose during disconnect
   (see section 4)

---

## 4. ESP32 Dosimetry Role

**Finding: dosimetry.c CAN maintain a running dose during disconnect, but
has limitations.**

`dosimetry.c` provides:
- Per-sample processing via `dosimetry_process_sample()` (line 207) -- accepts
  `sample_interval_ms`, computes `o3_mg_s` and `o3_mg_sample`
- Cycle tracking via `dosimetry_cycle_start/stop/get()` (lines 286-358) --
  accumulates `total_o3_measured_mg` and `sample_count`
- Decay estimation via `dosimetry_estimate_decay_rate()` (line 171)

**Current state**: dosimetry.c exists but is **not called** by seq_executor.
The executor reads sensor values (line 376-378 of `seq_executor.c`) only to
format `SEQ,SAMPLE` messages -- it does not feed them to dosimetry.

**Utility for process_batch**:
- **YES**: If the executor calls `dosimetry_process_sample()` for each 106-H
  reading during a recipe, the cycle accumulator will maintain total measured
  O3 mass regardless of TCP state.
- On reconnect, the PC can query accumulated dose via a new command
  (e.g., `CMD,dosimetry_get`) to recover the running total.
- **Limitation**: dosimetry.c only tracks measured (outlet) O3, not absorbed
  dose.  Absorbed dose requires the power model (PC-side).  However, since
  the executor controls power level deterministically per recipe step, the
  PC can reconstruct `mg_produced` from the recipe definition + elapsed time.

**Decision**: ESP32 dosimetry should be activated during batch recipes as a
**safety net accumulator**.  It provides:
1. Running `total_o3_measured_mg` surviving disconnect
2. `sample_count` and `duration_ms` for gap detection
3. On reconnect, PC can compare its own partial data against ESP32 totals
   to validate completeness

**Required firmware changes**:
- In `executor_task()`, call `dosimetry_cycle_start()` at recipe start
- In the sample loop, call `dosimetry_process_sample()` for each reading
- At recipe end, call `dosimetry_cycle_stop()`
- Add `CMD,dosimetry_get` LAN command to query cycle state
- Pass actual `sample_interval_ms` (computed from timestamps, not hardcoded)

---

## 5. Safe Default Recipe for PC-Offline Sterilization

A minimum viable "fire-and-forget" recipe must be safe even if the PC never
reconnects during the entire batch.

**Proposed MVP sterilization recipe** (sent by PC before batch start):

| Step | Power | Hold | Phase | Air | Rationale |
|------|-------|------|-------|-----|-----------|
| 0 | 100% | 360 samples (~15 min) | fill | 0 | Fill vessel to steady-state O3 |
| 1 | 50% | 720 samples (~30 min) | hold | 0 | Maintain target concentration |
| 2 | 0% | 120 samples (~5 min) | purge | 0 | Evacuate O3 before opening |

Relay prereqs: `relay_o2=1,relay_o3=1,relay_air=0`

**Safety properties**:
- Recipe is self-contained: all steps, power levels, and durations are
  pre-loaded before execution starts
- If TCP drops, executor continues and completes all three phases
- On completion, `restore_relays()` sets power=0 and air_comp=OFF
- On abort (E-STOP at ESP32 side), power=0, ozone_gen=OFF
- No prompts in the recipe (prompts would stall forever if PC is offline)
- Hold durations are generous (sample-counted, not time-counted, so they
  adapt to actual 106-H sample rate)

**What the PC loses during disconnect**:
- Per-sample O3 concentration data (SEQ SAMPLE messages)
- Ability to adjust power mid-batch
- Ability to extend/shorten phases based on live readings
- Dosimetry data (unless ESP32 accumulator is activated per section 4)

**What remains safe**:
- Ozone generation follows recipe power levels exactly
- Recipe completes and shuts down autonomously
- ESP32 dosimetry accumulator (if implemented) preserves total dose
- DATA telemetry backfill recovers up to 500 samples of raw sensor data

---

## 6. Hardcoded Sample Interval Audit

All instances of hardcoded sample timing values that should be replaced with
dynamic `dt` computation:

### Critical -- used in calculations or sleep loops:

| File | Line | Value | Context |
|------|------|-------|---------|
| `Interfaces/PC/dashboard/k_d_cal.py` | 34 | `FILL_SAMPLE_INTERVAL = 2.5` | Used as sleep interval in fill/evac monitoring loops (lines 295, 328, 453) — **now uses dynamic dt from ESP32 timestamps** |
| `Interfaces/ControlSystem/main/dosimetry.c` | 208, 219 | `sample_interval_ms` parameter | Accepts interval as argument -- correct design, but callers must pass actual dt |
| `Interfaces/ControlSystem/main/main.c` | 83 | `SENSOR_SAMPLE_INTERVAL_MS 500` | Secondary sensor polling (DFRobot, thermocouple) -- not 106-H related, acceptable |
| `Interfaces/ControlSystem/main/blocksi_pins.h` | 66 | `LAB_O3_SAMPLE_INTERVAL_MS 1000` | DFRobot lab O3 sensor polling -- not 106-H related, acceptable |

### Informational -- documentation references to ~2.5s:

| File | Line | Value | Context |
|------|------|-------|---------|
| `docs/interface_contract.md` | 249 | `~2.5s each` | Step hold documentation |
| `docs/interface_contract.md` | 376 | `~2.5s each` | Sample-counted holds note |
| `docs/interface_contract.md` | 404 | `~2.5s` | SAMPLE message frequency |
| `docs/reference/hardware.md` | 49 | `~2.5s sample interval` | 106-H description |
| `docs/esp32_agent_summary.md` | 65 | `~2.5s` | Calibration sample timing |
| `docs/decisions_log.md` | 92 | `~2.5s` | Backfill transmission estimate |
| `docs/cases/cstr_power_stability.md` | 75 | `~2.5s` | Power actual_pct update frequency |

### Action items for dynamic dt:

1. **`k_d_cal.py` (formerly `cstr_sequence.py`) FILL_SAMPLE_INTERVAL**: The hardcoded 2.5s
   sleep has been replaced with dynamic `dt` computation from consecutive
   `esp_timestamp_ms` differences (completed in WP-1 rewrite).

2. **dosimetry.c callers**: Ensure any future caller of
   `dosimetry_process_sample()` passes the actual measured interval between
   106-H samples, not a hardcoded value.

3. **Documentation**: Update `~2.5s` references to say "~2-3s (depends on
   106-H averaging mode)" to avoid implying a fixed rate.

---

## 7. Summary: Risk Assessment for 30-Minute Batch

| Risk | Severity | Likelihood | Mitigation |
|------|----------|------------|------------|
| Recipe stalls on disconnect | None | N/A | Executor is TCP-independent (verified) |
| SEQ SAMPLE data lost during disconnect | HIGH | Moderate | Activate ESP32 dosimetry accumulator; cache SEQ messages |
| DATA backfill overflow (>20 min outage) | MEDIUM | Low-Moderate | Increase DATA_CACHE_SLOTS to 1000 |
| PC sends power_set during recipe | MEDIUM | Low | Add firmware guard (reject if sequence active) |
| Prompt stalls if PC offline | HIGH | Moderate | No prompts in batch recipes; or add timeout |
| Hardcoded 2.5s interval in CSTR | LOW | Always | Replace with dynamic dt from timestamps |

## 8. Recommended Resilience Strategy: Hybrid

**Recipe-based execution** (ESP32 autonomy) for the mechanical sequence:
- PC pre-loads complete fill/hold/purge recipe before batch starts
- ESP32 executes autonomously; no prompts in batch recipes
- ESP32 dosimetry accumulates total measured dose throughout

**PC-driven analysis** when connected:
- PC monitors SEQ SAMPLE data in real-time for live dosimetry
- PC can send `sequence_abort` if readings are unsafe
- On reconnect: PC queries `dosimetry_get` for ESP32 accumulated totals,
  receives DATA backfill, and reconciles any gap

**Required firmware changes** (prioritized):
1. **Guard power_set/relay_set during active sequence** (safety, ~10 lines)
2. **Integrate dosimetry into seq_executor** (disconnect resilience, ~20 lines)
3. **Add `CMD,dosimetry_get` command** (reconnect recovery, ~15 lines)
4. **Increase DATA_CACHE_SLOTS to 1000** (backfill coverage, 1-line change)
5. **Cache SEQ SAMPLE messages** in ring buffer (optional, significant effort)
6. **Add prompt timeout** for batch recipes (prevent infinite stall, ~5 lines)
