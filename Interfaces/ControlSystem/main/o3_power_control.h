/**
 * @file o3_power_control.h
 * @brief Ozone generator power control via DS3502 digital potentiometer
 * 
 * Controls MP-8000 ozone generator power level by replacing the manual
 * potentiometer with a DS3502 digital potentiometer in rheostat mode.
 * 
 * Hardware:
 * - DS3502 (10kΩ) replaces original 4.7kΩ rheostat
 * - RH → MP-8000 control circuit (4.9V source)
 * - RW → MP-8000 current sink (held at 0V)
 * - RL → Jumpered to RW (rheostat mode)
 * 
 * Control Behavior:
 * - Original circuit: 0-3.7kΩ range controlled ozone output
 * - DS3502: 0-10kΩ range, with ~60 steps covering original range
 * - Higher resistance = more current to control circuit = more ozone
 * - R=0: Generator OFF (current sink sees short, shuts down)
 * 
 * Power Zones (from calibration):
 * - 0-20%: Sub-threshold, minimal output
 * - 20-75%: Linear active range  
 * - 75-100%: Saturation, diminishing returns
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
 * @brief Power control mode
 */
typedef enum {
    O3_POWER_MODE_ORIGINAL,     // Limit to original 4.7kΩ pot range
    O3_POWER_MODE_EXTENDED      // Use full 10kΩ DS3502 range
} o3_power_mode_t;

/**
 * @brief Power control state
 */
typedef struct {
    uint8_t wiper_position;     // Raw wiper position (0-127)
    uint16_t resistance_ohms;   // Current resistance
    float power_percent;        // Power percentage (0-100%)
    float predicted_o3_ppm;     // Predicted O3 based on calibration
    o3_power_mode_t mode;       // Current operating mode
} o3_power_state_t;

/**
 * @brief Calibration data point
 */
typedef struct {
    uint8_t wiper;              // Wiper setting
    float o3_mean_ppm;          // Mean O3 concentration
    float o3_std_ppm;           // Standard deviation
    uint16_t sample_count;      // Number of samples
    uint32_t hold_time_ms;      // Time held at this setting
} o3_power_cal_point_t;

/**
 * @brief Calibration curve
 */
typedef struct {
    float flow_rate_lpm;        // Flow rate during calibration
    uint8_t point_count;        // Number of calibration points
    o3_power_cal_point_t points[128];  // Up to 128 points (full wiper range)
    uint32_t timestamp;         // Unix timestamp of calibration
    bool valid;                 // True if calibration is valid
} o3_power_calibration_t;

/**
 * @brief Callback for calibration sweep progress
 */
typedef void (*o3_power_sweep_callback_t)(uint8_t wiper, float o3_reading, 
                                           float progress, void *user_data);

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize power control with DS3502
 * 
 * @return ESP_OK on success
 */
esp_err_t o3_power_init(void);

/**
 * @brief Deinitialize power control (sets power to 0)
 */
void o3_power_deinit(void);

/**
 * @brief Check if power control is initialized
 */
bool o3_power_is_initialized(void);

// ============================================================================
// Power Control
// ============================================================================

/**
 * @brief Set power level (0-100%)
 * 
 * @param percent Power percentage
 * @return ESP_OK on success
 */
esp_err_t o3_power_set_percent(float percent);

/**
 * @brief Get current power level
 * 
 * @return Power percentage (0-100%)
 */
float o3_power_get_percent(void);

/**
 * @brief Set power by raw wiper position (0-127)
 * 
 * @param wiper Wiper position
 * @return ESP_OK on success
 */
esp_err_t o3_power_set_wiper(uint8_t wiper);

/**
 * @brief Get current wiper position
 * 
 * @return Wiper position (0-127)
 */
uint8_t o3_power_get_wiper(void);

/**
 * @brief Set target resistance in ohms
 * 
 * @param ohms Target resistance
 * @return ESP_OK on success
 */
esp_err_t o3_power_set_resistance(uint16_t ohms);

/**
 * @brief Get current resistance
 * 
 * @return Resistance in ohms
 */
uint16_t o3_power_get_resistance(void);

/**
 * @brief Set operating mode (original range vs extended)
 * 
 * @param mode Operating mode
 */
void o3_power_set_mode(o3_power_mode_t mode);

/**
 * @brief Get current operating mode
 */
o3_power_mode_t o3_power_get_mode(void);

/**
 * @brief Emergency stop - set power to 0 immediately
 */
void o3_power_emergency_stop(void);

// ============================================================================
// State and Prediction
// ============================================================================

/**
 * @brief Get complete power state
 * 
 * @param state Output state structure
 * @return ESP_OK on success
 */
esp_err_t o3_power_get_state(o3_power_state_t *state);

/**
 * @brief Predict O3 output for given power setting
 * 
 * Uses calibration data if available, otherwise returns estimate.
 * 
 * @param percent Power percentage
 * @param flow_lpm Flow rate in LPM
 * @return Predicted O3 in ppm
 */
float o3_power_predict_o3(float percent, float flow_lpm);

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Start calibration sweep
 * 
 * Sweeps through wiper positions, holding at each for specified time.
 * Calls callback with progress updates.
 * 
 * @param start_wiper Starting wiper position
 * @param end_wiper Ending wiper position  
 * @param step_size Wiper increment between points
 * @param hold_time_ms Time to hold at each position (ms)
 * @param callback Progress callback (can be NULL)
 * @param user_data User data passed to callback
 * @return ESP_OK on success
 */
esp_err_t o3_power_start_calibration(uint8_t start_wiper, uint8_t end_wiper,
                                      uint8_t step_size, uint32_t hold_time_ms,
                                      o3_power_sweep_callback_t callback,
                                      void *user_data);

/**
 * @brief Stop ongoing calibration
 */
void o3_power_stop_calibration(void);

/**
 * @brief Check if calibration is in progress
 */
bool o3_power_calibration_active(void);

/**
 * @brief Get calibration data
 * 
 * @param cal Output calibration structure
 * @return ESP_OK if valid calibration exists
 */
esp_err_t o3_power_get_calibration(o3_power_calibration_t *cal);

/**
 * @brief Clear calibration data
 */
void o3_power_clear_calibration(void);

#ifdef __cplusplus
}
#endif

#endif // O3_POWER_CONTROL_H
