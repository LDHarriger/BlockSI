/**
 * @file sensor_aggregator.h
 * @brief Aggregates and averages sensor readings to sync with 106-H sample rate
 * 
 * The 106-H samples at ~2 second intervals. Secondary sensors (lab O3, thermocouple)
 * sample faster and are averaged over each 106-H sample window.
 */

#ifndef SENSOR_AGGREGATOR_H
#define SENSOR_AGGREGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Aggregated sensor reading (averaged over sample window)
 */
typedef struct {
    // Lab O3 sensor (DFRobot 0-10ppm)
    float room_o3_ppm;          // Averaged room O3 in ppm
    float room_o3_min;          // Minimum in window
    float room_o3_max;          // Maximum in window
    uint16_t room_o3_count;     // Number of samples averaged
    bool room_o3_valid;         // True if sensor connected
    
    // Thermocouple (vessel temperature)
    float vessel_temp_c;        // Averaged temperature in °C
    float vessel_temp_min;      // Minimum in window
    float vessel_temp_max;      // Maximum in window
    uint16_t vessel_temp_count; // Number of samples averaged
    bool vessel_temp_valid;     // True if sensor connected
    
    // Cold junction temperature (from MAX31855)
    float ambient_temp_c;       // Internal/ambient temperature
    bool ambient_temp_valid;
} aggregated_sensors_t;

/**
 * @brief Initialize sensor aggregator
 * 
 * Starts background sampling tasks for secondary sensors.
 * 
 * @param sample_interval_ms Interval for secondary sensor sampling (e.g., 500ms)
 * @return ESP_OK on success
 */
esp_err_t sensor_aggregator_init(uint32_t sample_interval_ms);

/**
 * @brief Deinitialize sensor aggregator
 */
void sensor_aggregator_deinit(void);

/**
 * @brief Get aggregated sensor readings and reset accumulators
 * 
 * Call this when a 106-H sample arrives to get averaged secondary sensor data.
 * Resets the accumulators for the next window.
 * 
 * @param sensors Output: aggregated sensor readings
 * @return ESP_OK on success
 */
esp_err_t sensor_aggregator_get_and_reset(aggregated_sensors_t *sensors);

/**
 * @brief Get current aggregated readings without resetting
 * 
 * @param sensors Output: current aggregated readings
 * @return ESP_OK on success
 */
esp_err_t sensor_aggregator_peek(aggregated_sensors_t *sensors);

/**
 * @brief Check if room O3 alarm is active
 * 
 * @return true if room O3 exceeds safety threshold
 */
bool sensor_aggregator_room_o3_alarm(void);

/**
 * @brief Get instantaneous room O3 reading
 * 
 * @return Room O3 in ppm, or -1 if not available
 */
float sensor_aggregator_get_room_o3(void);

/**
 * @brief Get instantaneous vessel temperature
 * 
 * @return Temperature in °C, or NAN if not available
 */
float sensor_aggregator_get_vessel_temp(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_AGGREGATOR_H
