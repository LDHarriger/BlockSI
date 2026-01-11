/**
 * @file motor_pot.h
 * @brief Motorized Potentiometer Driver for PRM162 + DRV8833
 * 
 * Controls ozone generator power via motorized dual-gang potentiometer.
 * Uses DRV8833 H-bridge for motor control and ADC for position feedback
 * from the servo track.
 * 
 * Hardware:
 *   - PRM162-K415K-502B1: Dual 5kΩ motorized rotary potentiometer
 *   - DRV8833: Dual H-bridge motor driver
 *   - Section 1: Connected to MP-8000 control circuit (floating)
 *   - Section 2: Connected to ESP32 for position feedback (servo track)
 */

#ifndef MOTOR_POT_H
#define MOTOR_POT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// ============================================================================
// Configuration Constants
// ============================================================================

// DRV8833 Pin Assignments
#define MOTOR_POT_AIN1_GPIO     25      // PWM capable pin
#define MOTOR_POT_AIN2_GPIO     26      // PWM capable pin
#define MOTOR_POT_SLP_GPIO      27      // Sleep pin (HIGH = active)

// ADC Configuration for Servo Track
#define MOTOR_POT_ADC_GPIO      34      // ADC1_CH6, input only
#define MOTOR_POT_ADC_CHANNEL   ADC1_CHANNEL_6
#define MOTOR_POT_ADC_ATTEN     ADC_ATTEN_DB_12  // 0-3.3V range

// Motor Characteristics (PRM162)
#define MOTOR_VOLTAGE_NOMINAL   4.5f    // Rated voltage
#define MOTOR_CURRENT_MAX_MA    100     // Max current draw

// Control Parameters
#define MOTOR_PWM_FREQ_HZ       1000    // PWM frequency
#define MOTOR_PWM_RESOLUTION    8       // 8-bit (0-255)
#define MOTOR_PWM_DEFAULT       180     // Default speed (~70%)
#define MOTOR_PWM_SLOW          100     // Slow speed for fine positioning

// Position Control
#define POSITION_ADC_MIN        100     // ADC value at 0% (with margin)
#define POSITION_ADC_MAX        3996    // ADC value at 100% (with margin)
#define POSITION_DEADBAND       20      // ADC counts tolerance
#define POSITION_FINE_THRESHOLD 100     // Switch to slow speed within this range

// Timeout Protection
#define MOTOR_TIMEOUT_MS        5000    // Max time for any movement
#define MOTOR_SETTLE_MS         50      // Time to wait after stopping

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Motor direction enum
 */
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,      // Increase resistance (CW)
    MOTOR_DIR_REVERSE,      // Decrease resistance (CCW)
    MOTOR_DIR_BRAKE
} motor_direction_t;

/**
 * @brief PWM decay mode
 */
typedef enum {
    MOTOR_DECAY_FAST = 0,   // Better for positioning
    MOTOR_DECAY_SLOW        // Smoother but less responsive
} motor_decay_mode_t;

/**
 * @brief Motor pot configuration
 */
typedef struct {
    int ain1_gpio;          // DRV8833 AIN1
    int ain2_gpio;          // DRV8833 AIN2
    int slp_gpio;           // DRV8833 SLP (sleep, -1 to disable)
    int adc_gpio;           // Servo track ADC input
    uint16_t pot_ohms;      // Potentiometer full scale (5000 for PRM162)
    bool invert_direction;  // Swap forward/reverse if needed
} motor_pot_config_t;

/**
 * @brief Motor pot state
 */
typedef struct {
    uint16_t adc_raw;           // Raw ADC reading (0-4095)
    float position_percent;     // Position as percentage (0-100)
    uint16_t resistance_ohms;   // Estimated resistance
    float power_percent;        // Power setting for MP-8000
    motor_direction_t direction; // Current motor direction
    bool is_moving;             // Motor currently active
} motor_pot_state_t;

/**
 * @brief Calibration data
 */
typedef struct {
    uint16_t adc_min;           // ADC at mechanical minimum
    uint16_t adc_max;           // ADC at mechanical maximum
    bool calibrated;            // Calibration complete flag
} motor_pot_calibration_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the motorized potentiometer system
 * @param config Configuration parameters (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t motor_pot_init(const motor_pot_config_t *config);

/**
 * @brief Deinitialize and set to safe state
 */
void motor_pot_deinit(void);

/**
 * @brief Check if system is initialized
 */
bool motor_pot_is_initialized(void);

/**
 * @brief Enable/disable the DRV8833 (via SLP pin)
 */
esp_err_t motor_pot_enable(bool enable);

// ----------------------------------------------------------------------------
// Motor Control
// ----------------------------------------------------------------------------

/**
 * @brief Set motor direction and speed
 * @param direction Direction (stop, forward, reverse, brake)
 * @param pwm_duty PWM duty cycle (0-255)
 * @param decay_mode Fast or slow decay
 */
esp_err_t motor_pot_set_motor(motor_direction_t direction, uint8_t pwm_duty, 
                               motor_decay_mode_t decay_mode);

/**
 * @brief Stop motor (coast)
 */
esp_err_t motor_pot_stop(void);

/**
 * @brief Stop motor (active brake)
 */
esp_err_t motor_pot_brake(void);

/**
 * @brief Run motor for specified duration
 * @param direction Direction to move
 * @param duration_ms Duration in milliseconds
 * @param pwm_duty PWM duty cycle
 * @return Actual duration moved (may be less if limit reached)
 */
uint32_t motor_pot_pulse(motor_direction_t direction, uint32_t duration_ms, 
                          uint8_t pwm_duty);

// ----------------------------------------------------------------------------
// Position Control
// ----------------------------------------------------------------------------

/**
 * @brief Read current position from servo track ADC
 * @param adc_value Output: raw ADC value (0-4095)
 * @return ESP_OK on success
 */
esp_err_t motor_pot_read_position(uint16_t *adc_value);

/**
 * @brief Get position as percentage (0-100%)
 */
float motor_pot_get_position_percent(void);

/**
 * @brief Get estimated resistance based on position
 */
uint16_t motor_pot_get_resistance(void);

/**
 * @brief Move to target position (blocking)
 * @param target_percent Target position (0-100%)
 * @param timeout_ms Maximum time to reach position
 * @return ESP_OK if reached, ESP_ERR_TIMEOUT if not reached
 */
esp_err_t motor_pot_goto_position(float target_percent, uint32_t timeout_ms);

/**
 * @brief Set power percentage (main API for ozone control)
 * @param power_percent Power level (0-100%)
 * @return ESP_OK on success
 */
esp_err_t motor_pot_set_power(float power_percent);

/**
 * @brief Get current power setting
 */
float motor_pot_get_power(void);

// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

/**
 * @brief Run automatic calibration (finds min/max positions)
 * @return ESP_OK on success
 */
esp_err_t motor_pot_calibrate(void);

/**
 * @brief Home to minimum position
 */
esp_err_t motor_pot_home(void);

/**
 * @brief Set calibration values manually
 */
esp_err_t motor_pot_set_calibration(uint16_t adc_min, uint16_t adc_max);

/**
 * @brief Get calibration data
 */
esp_err_t motor_pot_get_calibration(motor_pot_calibration_t *cal);

// ----------------------------------------------------------------------------
// State and Diagnostics
// ----------------------------------------------------------------------------

/**
 * @brief Get complete state
 */
esp_err_t motor_pot_get_state(motor_pot_state_t *state);

/**
 * @brief Print diagnostic info
 */
void motor_pot_print_status(void);

#endif // MOTOR_POT_H
