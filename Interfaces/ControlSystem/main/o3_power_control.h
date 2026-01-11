/**
 * @file o3_power_control.h
 * @brief Ozone generator power control via motorized potentiometer
 * 
 * Controls MP-8000 ozone generator power level using a motorized dual-gang
 * potentiometer (PRM162) driven by DRV8833 H-bridge.
 * 
 * Hardware:
 * - PRM162-K415K-502B1: Dual 5kΩ motorized rotary potentiometer
 * - DRV8833: H-bridge motor driver
 * - Section 1 → MP-8000 control circuit (floating, ~10V above earth)
 * - Section 2 → ESP32 ADC for position feedback (0-3.3V)
 * 
 * This replaces the DS3502 digital potentiometer which had ground isolation
 * issues due to the MP-8000's floating control circuit.
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
    O3_POWER_MODE_ORIGINAL,     // Standard operating mode
    O3_POWER_MODE_EXTENDED      // Reserved for future use
} o3_power_mode_t;

/**
 * @brief Power control state
 */
typedef struct {
    uint8_t wiper_position;     // Pseudo wiper position (0-127 for compatibility)
    uint16_t resistance_ohms;   // Current resistance
    float power_percent;        // Power percentage (0-100%)
    float predicted_o3_ppm;     // Predicted O3 based on calibration
    o3_power_mode_t mode;       // Current operating mode
} o3_power_state_t;

/**
 * @brief Calibration data point
 */
typedef struct {
    uint8_t wiper;              // Pseudo wiper setting (0-127)
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
    o3_power_cal_point_t points[128];  // Up to 128 points
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

esp_err_t o3_power_init(void);
void o3_power_deinit(void);
bool o3_power_is_initialized(void);

// ============================================================================
// Power Control
// ============================================================================

esp_err_t o3_power_set_percent(float percent);
float o3_power_get_percent(void);
esp_err_t o3_power_set_wiper(uint8_t wiper);
uint8_t o3_power_get_wiper(void);
esp_err_t o3_power_set_resistance(uint16_t ohms);
uint16_t o3_power_get_resistance(void);
void o3_power_set_mode(o3_power_mode_t mode);
o3_power_mode_t o3_power_get_mode(void);
void o3_power_emergency_stop(void);

// ============================================================================
// State and Prediction
// ============================================================================

esp_err_t o3_power_get_state(o3_power_state_t *state);
float o3_power_predict_o3(float percent, float flow_lpm);

// ============================================================================
// Calibration
// ============================================================================

esp_err_t o3_power_start_calibration(uint8_t start_wiper, uint8_t end_wiper,
                                      uint8_t step_size, uint32_t hold_time_ms,
                                      o3_power_sweep_callback_t callback,
                                      void *user_data);
void o3_power_stop_calibration(void);
bool o3_power_calibration_active(void);
esp_err_t o3_power_get_calibration(o3_power_calibration_t *cal);
void o3_power_clear_calibration(void);

// ============================================================================
// Backward-Compatible API
// ============================================================================

uint8_t o3_power_get(void);
esp_err_t o3_power_set(uint8_t power_pct);
float o3_power_get_voltage(void);

#ifdef __cplusplus
}
#endif

#endif // O3_POWER_CONTROL_H
