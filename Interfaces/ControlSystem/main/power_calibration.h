/**
 * @file power_calibration.h
 * @brief Power calibration sweep for MP-8000 ozone generator
 * 
 * Characterizes O3 output vs. power percentage for predictive control.
 */

#ifndef POWER_CALIBRATION_H
#define POWER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration status structure
 */
typedef struct {
    bool active;            // Sweep in progress
    int8_t direction;       // 1 = ascending, -1 = descending
    uint8_t current_pct;    // Current power percentage
    uint16_t points_up;     // Points collected in ascending phase
    uint16_t points_down;   // Points collected in descending phase
    float elapsed_s;        // Seconds since sweep started
} power_cal_status_t;

/**
 * @brief Initialize calibration module
 * 
 * @return ESP_OK on success
 */
esp_err_t power_calibration_init(void);

/**
 * @brief Start calibration sweep
 * 
 * Sweeps power 0→100% then 100→0% in 1% steps.
 * One 106-H sample captured per step.
 * Data sent as CAL_DATA messages over LAN.
 * 
 * @return ESP_OK if sweep started
 * @return ESP_ERR_INVALID_STATE if already running or motor pot not initialized
 */
esp_err_t power_calibration_start(void);

/**
 * @brief Stop calibration sweep
 * 
 * @return ESP_OK if stop requested
 * @return ESP_ERR_INVALID_STATE if not running
 */
esp_err_t power_calibration_stop(void);

/**
 * @brief Check if calibration is active
 * 
 * @return true if sweep in progress
 */
bool power_calibration_is_active(void);

/**
 * @brief Get current calibration status
 * 
 * @param status Output status structure
 * @return ESP_OK on success
 */
esp_err_t power_calibration_get_status(power_cal_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // POWER_CALIBRATION_H
