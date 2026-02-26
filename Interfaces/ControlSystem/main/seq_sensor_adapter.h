/**
 * @file seq_sensor_adapter.h
 * @brief Monotonic sample counter for the sequence executor
 *
 * Provides a lightweight notification mechanism: the 106-H parser calls
 * seq_sensor_notify_new_sample() each time a valid reading arrives.
 * The executor polls seq_sensor_get_sample_count() to implement
 * sample-counted holds.
 *
 * The actual sensor values are read by the executor from blocksi_state
 * (which is already updated by the 106-H callback in main.c).
 */

#ifndef SEQ_SENSOR_ADAPTER_H
#define SEQ_SENSOR_ADAPTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the monotonic sample count
 *
 * Incremented each time the 106-H parser delivers a valid reading.
 */
uint32_t seq_sensor_get_sample_count(void);

/**
 * @brief Increment the sample counter
 *
 * Called from the 106-H sample callback in main.c.
 * Keep this function trivial — it runs in UART task context.
 */
void seq_sensor_notify_new_sample(void);

#ifdef __cplusplus
}
#endif

#endif /* SEQ_SENSOR_ADAPTER_H */
