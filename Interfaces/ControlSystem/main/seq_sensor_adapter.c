/**
 * @file seq_sensor_adapter.c
 * @brief Monotonic sample counter — trivial implementation
 */

#include "seq_sensor_adapter.h"

static volatile uint32_t s_sample_count = 0;

uint32_t seq_sensor_get_sample_count(void)
{
    return s_sample_count;
}

void seq_sensor_notify_new_sample(void)
{
    s_sample_count++;
}
