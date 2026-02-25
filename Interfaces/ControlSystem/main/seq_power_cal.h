/**
 * @file seq_power_cal.h
 * @brief Power-O3 calibration sequence for the sequence runner framework
 * 
 * Characterizes O3 output vs. power percentage for the MP-8000 ozone generator.
 * 
 * Sequence phases:
 *   1. baseline    — 30s at 0% power, air compressor OFF
 *   2. sweep_up    — 0→100% in 1% steps, ~3s each, air OFF
 *   3. sweep_down  — 100→0% in 1% steps, ~3s each, air OFF
 *   4. random_pair — 15 random power levels × (20s air OFF + 20s air ON)
 * 
 * Streams CAL_DATA points over LAN for each measurement.
 * Total estimated time: ~17–22 minutes depending on motor settle times.
 * 
 * LAN Usage:
 *   CMD,sequence_start,cal,<o2_lpm>   (e.g., CMD,sequence_start,cal,4.0)
 *   CMD,sequence_stop                  (abort at any time)
 * 
 * CAL_DATA format:
 *   CAL_DATA,<timestamp_ms>,<power_pct>,<actual_pct>,<o3_pct>,<o2_lpm>,
 *            <air_comp_on>,<total_lpm>,<cell_temp_c>,<phase>
 */

#ifndef SEQ_POWER_CAL_H
#define SEQ_POWER_CAL_H

#include "sequence_runner.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the power calibration sequence implementation
 * 
 * Returns a persistent pointer to the static implementation struct.
 * Register with sequence_runner_register() during initialization.
 * 
 * @return Pointer to the sequence implementation
 */
const sequence_impl_t* seq_power_cal_get_impl(void);

#ifdef __cplusplus
}
#endif

#endif // SEQ_POWER_CAL_H
