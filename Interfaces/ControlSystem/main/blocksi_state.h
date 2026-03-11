/**
 * @file blocksi_state.h
 * @brief Unified system state management for BlockSI
 * 
 * Provides a single source of truth for system targets and actuals.
 * Supports periodic state validation and automatic correction.
 */

#ifndef BLOCKSI_STATE_H
#define BLOCKSI_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "relay_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// State Structures
// =============================================================================

/**
 * @brief Power control state
 */
typedef struct {
    uint8_t target_pct;         // Commanded power (0-100%)
    float actual_pct;           // Measured power from ADC
    float wiper_voltage;        // Raw wiper voltage (0-3.3V)
    uint16_t adc_raw;           // Raw ADC reading
    bool motor_moving;          // Motor currently in motion
    bool error_mismatch;        // Target != actual beyond tolerance
    uint8_t retry_count;        // Number of correction attempts
    int64_t last_command_ms;    // Time of last power command
} power_state_t;

/**
 * @brief Per-relay tracked state
 */
typedef struct {
    bool target;                // Desired relay state
    bool actual;                // Confirmed relay state
    relay_source_t last_source; // Who last changed this relay
    int64_t last_change_ms;     // When it was last changed (uptime ms)
} tracked_relay_t;

/**
 * @brief Relay control state (all 3 relays tracked)
 */
typedef struct {
    tracked_relay_t relays[RELAY_COUNT];  // Indexed by relay_id_t
} blocksi_relay_state_t;

/**
 * @brief Sensor readings
 */
typedef struct {
    float vessel_o3_pct;        // 106-H ozone in %vol
    float room_o3_ppm;          // DFRobot sensor in ppm
    float vessel_temp_c;        // Thermocouple temperature
    float cell_temp_c;          // 106-H cell temperature
    float pressure_mbar;        // 106-H pressure
    bool vessel_o3_valid;
    bool room_o3_valid;
    bool vessel_temp_valid;
} sensor_state_t;

/**
 * @brief Calibration state
 */
typedef struct {
    bool active;                // Calibration sweep in progress
    bool stop_requested;        // User requested stop
    uint8_t current_step_pct;   // Current power % in sweep
    uint8_t direction;          // 1 = up, -1 = down
    uint16_t points_collected;  // Number of data points
    int64_t start_time_ms;      // When calibration started
} calibration_state_t;

/**
 * @brief Timing and sync state
 */
typedef struct {
    int64_t esp_uptime_ms;      // ESP32 uptime in ms
    int64_t pc_time_offset_ms;  // Offset to convert to PC time (0 if not synced)
    bool time_synced;           // True if time sync received from PC
    int64_t last_sync_ms;       // When last sync occurred
} timing_state_t;

/**
 * @brief Complete system state
 */
typedef struct {
    power_state_t power;
    blocksi_relay_state_t relays;
    sensor_state_t sensors;
    calibration_state_t calibration;
    timing_state_t timing;
    
    // Connection state
    bool lan_connected;
    int64_t last_data_send_ms;
    uint32_t data_send_count;
    
    // Error tracking
    uint32_t power_mismatch_count;
    uint32_t command_timeout_count;
} blocksi_state_t;

// =============================================================================
// API Functions
// =============================================================================

/**
 * @brief Initialize state management
 */
esp_err_t blocksi_state_init(void);

/**
 * @brief Get pointer to current system state (read-only recommended)
 */
const blocksi_state_t* blocksi_state_get(void);

/**
 * @brief Get mutable pointer (for internal updates)
 */
blocksi_state_t* blocksi_state_get_mutable(void);

/**
 * @brief Update power target and initiate movement
 * @param target_pct Target power percentage (0-100)
 * @return ESP_OK on success
 */
esp_err_t blocksi_state_set_power(uint8_t target_pct);

/**
 * @brief Update relay target state (with source tracking)
 * @param relay Relay ID (0=O3 generator, 1=O2 concentrator, 2=Air compressor)
 * @param state true=ON, false=OFF
 * @param source Who/what initiated this change
 * @return ESP_OK on success
 */
esp_err_t blocksi_state_set_relay(uint8_t relay, bool state, relay_source_t source);

/**
 * @brief Update sensor readings (called from sensor callbacks)
 */
void blocksi_state_update_sensors(const sensor_state_t *sensors);

/**
 * @brief Update power actual from ADC reading
 */
void blocksi_state_update_power_actual(float actual_pct, float voltage, uint16_t adc_raw);

/**
 * @brief Sync time with PC
 * @param pc_timestamp_ms PC's current time in milliseconds since epoch
 */
void blocksi_state_sync_time(int64_t pc_timestamp_ms);

/**
 * @brief Get current ESP32 time adjusted for PC sync
 * @return Time in ms (ESP uptime if not synced, PC time if synced)
 */
int64_t blocksi_state_get_time_ms(void);

/**
 * @brief Check state consistency and correct mismatches
 * Called periodically from a background task
 */
void blocksi_state_validate(void);

/**
 * @brief Start calibration sweep
 */
esp_err_t blocksi_state_start_calibration(void);

/**
 * @brief Stop calibration sweep
 */
void blocksi_state_stop_calibration(void);

// =============================================================================
// Configuration
// =============================================================================

#define POWER_MISMATCH_TOLERANCE    5.0f    // % tolerance for target vs actual
#define POWER_MISMATCH_CLEAR        2.5f    // % hysteresis band to clear error
#define POWER_SETTLE_TIME_MS        1500    // Wait after movement before validating
#define STATE_VALIDATE_INTERVAL_MS  1000    // How often to check state

#ifdef __cplusplus
}
#endif

#endif // BLOCKSI_STATE_H
