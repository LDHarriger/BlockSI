/**
 * @file seq_airflow_val.h
 * @brief Airflow/concentration validation sequence for the sequence runner framework
 * 
 * Validates that the O3 generation system produces the expected ozone concentration
 * at a given power level and flow configuration.  This is a "pre-flight" check
 * run before sterilization or decay testing.
 * 
 * The sequence is interactive — it requires operator confirmation at two points:
 *   1. Route air to the vessel and confirm O2 flow on the rotameter
 *   2. Route air direct to the 106-H sensor, match flow with needle valve
 * 
 * The actual measurement is taken on the direct-to-sensor route, which gives
 * a clean baseline concentration without vessel transit delay, surface decay,
 * or substrate absorption effects.
 * 
 * Sequence phases:
 *   1. prompt_vessel  — Operator routes air to vessel, confirms LPM
 *   2. prompt_direct  — Operator routes air direct to 106-H, confirms LPM match
 *   3. stabilize      — O3 generator at target power, wait for reading to settle
 *   4. measure        — Record O3 readings, compute mean/std/expected
 *   5. complete       — Report validation result (pass/fail/info)
 * 
 * LAN Usage:
 *   CMD,sequence_start,validate,<power_pct>,<o2_lpm>
 *   CMD,sequence_confirm,prompt_vessel[,<confirmed_lpm>]
 *   CMD,sequence_confirm,prompt_direct[,<confirmed_lpm>]
 *   CMD,sequence_stop                 (abort at any time)
 * 
 * SEQ Messages:
 *   SEQ,prompt,prompt_vessel,<message>     (waiting for operator)
 *   SEQ,prompt,prompt_direct,<message>     (waiting for operator)
 *   SEQ,stabilize,<progress>,<power>,0,<elapsed>
 *   SEQ,measure,<progress>,<power>,0,<elapsed>
 *   SEQ,complete,100,0,0,<elapsed>
 * 
 * VAL_DATA format (measurement points during measure phase):
 *   VAL_DATA,<timestamp_ms>,<power_pct>,<actual_pct>,<o3_pct>,<o2_lpm>,<cell_temp_c>
 * 
 * VAL_RESULT format (at completion):
 *   VAL_RESULT,power=<pct>,o2_lpm=<lpm>,mean_o3=<pct>,std_o3=<pct>,
 *              expected_o3=<pct>,samples=<n>,elapsed=<s>
 */

#ifndef SEQ_AIRFLOW_VAL_H
#define SEQ_AIRFLOW_VAL_H

#include "sequence_runner.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the airflow validation sequence implementation
 * 
 * @return Pointer to static sequence_impl_t (type name: "validate")
 */
const sequence_impl_t* seq_airflow_val_get_impl(void);

#ifdef __cplusplus
}
#endif

#endif // SEQ_AIRFLOW_VAL_H
