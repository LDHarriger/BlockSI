/**
 * @file o3_power_control.h
 * @brief Ozone generator power control via ESP32 DAC
 * 
 * Controls MP-8000 ozone generator power level by replacing the manual
 * potentiometer with a DAC-driven voltage output.
 * 
 * Hardware:
 * - ESP32 GPIO25 (DAC1) → voltage divider → op-amp buffer → MP-8000
 * - Output voltage range: 0 - 1.22V (matching original pot range)
 * 
 * The effective power range is 20-75% of the pot range, where:
 * - Below 20%: No measurable ozone output
 * - Above 75%: Diminishing returns
 */

#ifndef O3_POWER_CONTROL_H
#define O3_POWER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Power calibration data point
 */
typedef struct {
    uint8_t power_pct;          // Power setting (0-100%)
    float o3_mean_ppm;          // Mean O3 concentration
    float o3_std_ppm;           // Standard deviation
    uint16_t sample_count;      // Number of samples
} o3_power_cal_point_t;

/**
 * @brief Power calibration curve
 */
typedef struct {
    float flow_rate_lpm;        // Flow rate during calibration
    uint8_t point_count;        // Number of calibration points
    o3_power_cal_point_t points[21];  // Up to 21 points (0%, 5%, 10%, ... 100%)
    uint32_t timestamp;         // Unix timestamp of calibration
    bool valid;                 // True if calibration is valid
} o3_power_calibration_t;

/**
 * @brief Callback for power sweep progress
 * 
 * @param power_pct Current power level being tested
 * @param o3_mean Current mean O3 reading
 * @param o3_std Current standard deviation
 * @param progress Sweep progress (0.0 - 1.0)
 */
typedef void (*o3_power_sweep_callback_t)(uint8_t power_pct, float o3_mean, 
                                           float o3_std, float progress);

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize power control
 * 
 * Configures DAC output and sets power to 0%.
 * 
 * @return ESP_OK on success
 */
esp_err_t o3_power_init(void);

/**
 * @brief Deinitialize power control
 */
void o3_power_deinit(void);

// ============================================================================
// Power Control
// ============================================================================

/**
 * @brief Set power level
 * 
 * Maps 0-100% to effective DAC range with proper voltage scaling.
 * 
 * @param percent Power level (0-100%)
 * @return ESP_OK on success
 */
esp_err_t o3_power_set(uint8_t percent);

/**
 * @brief Get current power level
 * 
 * @return Current power level (0-100%)
 */
uint8_t o3_power_get(void);

/**
 * @brief Set raw DAC value (for testing/calibration)
 * 
 * @param dac_value Raw 8-bit DAC value (0-255)
 * @return ESP_OK on success
 */
esp_err_t o3_power_set_raw(uint8_t dac_value);

/**
 * @brief Get raw DAC value
 * 
 * @return Current raw DAC value (0-255)
 */
uint8_t o3_power_get_raw(void);

/**
 * @brief Get expected output voltage for current setting
 * 
 * @return Expected voltage in volts
 */
float o3_power_get_voltage(void);

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Start power sweep calibration
 * 
 * Performs automated sweep to characterize O3 output vs power level.
 * This is a blocking operation that takes several minutes.
 * 
 * Algorithm:
 * 1. Start at 0% power, establish baseline
 * 2. Coarse sweep (5% steps) to find active range
 * 3. Fine sweep (1% steps) within active range
 * 4. Collect statistics at each point (configurable samples)
 * 5. Store calibration curve
 * 
 * @param flow_rate_lpm Current O2 flow rate (for reference)
 * @param samples_per_point Number of samples to collect at each power level
 * @param callback Progress callback (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t o3_power_calibrate(float flow_rate_lpm, uint16_t samples_per_point,
                              o3_power_sweep_callback_t callback);

/**
 * @brief Abort running calibration
 */
void o3_power_calibrate_abort(void);

/**
 * @brief Check if calibration is in progress
 * 
 * @return true if calibration is running
 */
bool o3_power_calibrate_is_running(void);

/**
 * @brief Get calibration data
 * 
 * @param cal Output calibration structure
 * @return ESP_OK if valid calibration exists
 */
esp_err_t o3_power_get_calibration(o3_power_calibration_t *cal);

/**
 * @brief Save calibration to NVS
 * 
 * @return ESP_OK on success
 */
esp_err_t o3_power_save_calibration(void);

/**
 * @brief Load calibration from NVS
 * 
 * @return ESP_OK on success
 */
esp_err_t o3_power_load_calibration(void);

/**
 * @brief Clear stored calibration
 * 
 * @return ESP_OK on success
 */
esp_err_t o3_power_clear_calibration(void);

// ============================================================================
// Estimation (requires valid calibration)
// ============================================================================

/**
 * @brief Estimate O3 output rate for given power level
 * 
 * Uses calibration curve to interpolate expected O3 concentration.
 * 
 * @param power_pct Power level (0-100%)
 * @return Estimated O3 concentration in ppm, or -1 if no calibration
 */
float o3_power_estimate_output(uint8_t power_pct);

/**
 * @brief Find power level needed for target O3 output
 * 
 * Inverse of o3_power_estimate_output().
 * 
 * @param target_ppm Target O3 concentration in ppm
 * @return Required power level (0-100%), or 0xFF if not achievable
 */
uint8_t o3_power_find_for_target(float target_ppm);

#ifdef __cplusplus
}
#endif

#endif // O3_POWER_CONTROL_H
