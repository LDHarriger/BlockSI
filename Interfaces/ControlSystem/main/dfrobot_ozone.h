/**
 * @file dfrobot_ozone.h
 * @brief DFRobot Gravity I2C Ozone Sensor (0-10ppm) driver
 * 
 * This sensor is used for laboratory/ambient air monitoring to ensure
 * worker safety. It measures O3 in the 0-10 ppm range with 10 ppb resolution.
 * 
 * Specifications:
 * - Measurement range: 0-10 ppm
 * - Resolution: 10 ppb
 * - Response time: <30 seconds
 * - I2C interface (default address 0x73)
 * - Operating voltage: 3.3-5.5V
 * 
 * Safety Thresholds:
 * - OSHA PEL (8-hour TWA): 0.1 ppm
 * - OSHA STEL (15-min): 0.3 ppm
 * - IDLH (Immediately Dangerous): 5 ppm
 */

#ifndef DFROBOT_OZONE_H
#define DFROBOT_OZONE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default I2C address
 */
#define DFROBOT_O3_ADDR_DEFAULT     0x73

/**
 * @brief Safety alarm levels (ppm)
 */
#define O3_ALARM_WARNING            0.1f    // OSHA 8-hour limit
#define O3_ALARM_DANGER             0.3f    // OSHA 15-min limit  
#define O3_ALARM_CRITICAL           1.0f    // Evacuate
#define O3_ALARM_IDLH               5.0f    // Immediately dangerous

/**
 * @brief Alarm callback type
 * 
 * @param level_ppm Current O3 level in ppm
 * @param alarm_level Which threshold was exceeded
 */
typedef void (*dfrobot_o3_alarm_callback_t)(float level_ppm, float alarm_level);

/**
 * @brief Configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;            // I2C port number
    uint8_t i2c_addr;               // I2C address
    uint16_t sample_interval_ms;    // Auto-sample interval (0 = manual only)
    dfrobot_o3_alarm_callback_t alarm_callback;  // Alarm callback (can be NULL)
} dfrobot_o3_config_t;

/**
 * @brief Sensor status
 */
typedef struct {
    float last_reading_ppm;         // Last O3 reading
    uint32_t last_reading_time_ms;  // Timestamp of last reading
    float max_reading_ppm;          // Maximum observed
    float min_reading_ppm;          // Minimum observed
    uint32_t reading_count;         // Total readings taken
    bool alarm_active;              // True if any alarm threshold exceeded
} dfrobot_o3_status_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize DFRobot O3 sensor
 * 
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_init(const dfrobot_o3_config_t *config);

/**
 * @brief Deinitialize sensor
 */
void dfrobot_o3_deinit(void);

/**
 * @brief Check if sensor is present on I2C bus
 * 
 * @return true if device responds
 */
bool dfrobot_o3_is_present(void);

// ============================================================================
// Measurement
// ============================================================================

/**
 * @brief Read current O3 concentration
 * 
 * @param ppm Output: O3 concentration in ppm
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_read(float *ppm);

/**
 * @brief Read raw ADC value (for debugging)
 * 
 * @param raw Output: raw ADC value
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_read_raw(uint16_t *raw);

/**
 * @brief Get last reading without I2C communication
 * 
 * @return Last O3 reading in ppm, or -1 if no reading available
 */
float dfrobot_o3_get_last(void);

/**
 * @brief Get sensor status
 * 
 * @param status Output status structure
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_get_status(dfrobot_o3_status_t *status);

/**
 * @brief Reset min/max statistics
 */
void dfrobot_o3_reset_stats(void);

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Set calibration offset
 * 
 * Adds offset to raw reading: ppm = raw_ppm + offset
 * 
 * @param offset_ppm Offset in ppm (can be negative)
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_set_offset(float offset_ppm);

/**
 * @brief Get current calibration offset
 * 
 * @return Offset in ppm
 */
float dfrobot_o3_get_offset(void);

/**
 * @brief Perform zero calibration
 * 
 * Call while sensor is in clean air. Sets offset to make current reading = 0.
 * 
 * @return ESP_OK on success
 */
esp_err_t dfrobot_o3_calibrate_zero(void);

// ============================================================================
// Alarm Configuration
// ============================================================================

/**
 * @brief Enable/disable alarm checking
 * 
 * @param enable True to enable alarm checking
 */
void dfrobot_o3_alarm_enable(bool enable);

/**
 * @brief Set custom alarm thresholds
 * 
 * @param warning Warning level (ppm)
 * @param danger Danger level (ppm)
 * @param critical Critical level (ppm)
 */
void dfrobot_o3_set_alarm_levels(float warning, float danger, float critical);

/**
 * @brief Check if alarm is currently active
 * 
 * @return true if any alarm threshold is exceeded
 */
bool dfrobot_o3_alarm_is_active(void);

/**
 * @brief Get current alarm level
 * 
 * @return Highest exceeded threshold (0 if none)
 */
float dfrobot_o3_get_alarm_level(void);

#ifdef __cplusplus
}
#endif

#endif // DFROBOT_OZONE_H
