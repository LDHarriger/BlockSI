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
 *   - DRV8833: Dual H-bridge motor driver (SLP tied to 5V, always enabled)
 *   - Section 1: Connected to MP-8000 control circuit (floating)
 *   - Section 2: Connected to ESP32 for position feedback (servo track)
 * 
 * Pin assignments are defined in blocksi_pins.h
 */

#ifndef MOTOR_POT_H
#define MOTOR_POT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// ============================================================================
// Configuration Constants (defaults, can be overridden via config struct)
// ============================================================================

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
// Types
// ============================================================================

/**
 * @brief Motor direction
 */
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,      // Increase resistance (CW looking at shaft)
    MOTOR_DIR_REVERSE,      // Decrease resistance (CCW)
    MOTOR_DIR_BRAKE         // Active brake (both outputs high)
} motor_direction_t;

/**
 * @brief PWM decay mode
 */
typedef enum {
    MOTOR_DECAY_FAST = 0,   // Fast decay - more responsive, more ripple
    MOTOR_DECAY_SLOW        // Slow decay - smoother, more torque at low speed
} motor_decay_mode_t;

/**
 * @brief Motor pot configuration
 */
typedef struct {
    int ain1_gpio;          // DRV8833 AIN1 pin
    int ain2_gpio;          // DRV8833 AIN2 pin
    int adc_gpio;           // ADC input for position feedback
    uint16_t pot_ohms;      // Potentiometer full scale resistance
    bool invert_direction;  // Swap forward/reverse if wired backwards
} motor_pot_config_t;

/**
 * @brief Calibration data
 */
typedef struct {
    uint16_t adc_min;       // ADC reading at 0% position
    uint16_t adc_max;       // ADC reading at 100% position
    bool calibrated;        // True if calibration has been performed
} motor_pot_calibration_t;

/**
 * @brief Current state
 */
typedef struct {
    uint16_t adc_raw;           // Raw ADC reading
    float position_percent;     // Position 0-100%
    uint16_t resistance_ohms;   // Calculated resistance
    float power_percent;        // Same as position_percent (for API compat)
    motor_direction_t direction;// Current motor direction
    uint8_t pwm_duty;           // Current PWM duty cycle
    bool is_moving;             // True if motor is running
} motor_pot_state_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize motor pot driver
 * 
 * @param config Configuration (NULL for defaults from blocksi_pins.h)
 * @return ESP_OK on success
 */
esp_err_t motor_pot_init(const motor_pot_config_t *config);

/**
 * @brief Deinitialize motor pot driver
 */
void motor_pot_deinit(void);

/**
 * @brief Check if motor pot is initialized
 */
bool motor_pot_is_initialized(void);

// ============================================================================
// Motor Control
// ============================================================================

/**
 * @brief Set motor direction and speed
 * 
 * @param direction Motor direction
 * @param pwm_duty PWM duty cycle (0-255)
 * @param decay_mode Fast or slow decay
 * @return ESP_OK on success
 */
esp_err_t motor_pot_set_motor(motor_direction_t direction, uint8_t pwm_duty, 
                               motor_decay_mode_t decay_mode);

/**
 * @brief Stop motor (coast)
 */
void motor_pot_stop(void);

/**
 * @brief Brake motor (active stop)
 */
void motor_pot_brake(void);

/**
 * @brief Run motor for specified duration
 * 
 * @param direction Direction to move
 * @param duration_ms Time to run in milliseconds
 * @param pwm_duty PWM duty cycle
 * @return ESP_OK on success
 */
esp_err_t motor_pot_pulse(motor_direction_t direction, uint32_t duration_ms, 
                           uint8_t pwm_duty);

// ============================================================================
// Position Control
// ============================================================================

/**
 * @brief Read current ADC position
 * 
 * @return Raw ADC value (0-4095)
 */
uint16_t motor_pot_read_adc(void);

/**
 * @brief Get position as percentage
 * 
 * @return Position 0-100%
 */
float motor_pot_get_position_percent(void);

/**
 * @brief Move to target position (blocking)
 * 
 * Uses closed-loop control to reach target within deadband.
 * 
 * @param target_percent Target position (0-100%)
 * @param timeout_ms Maximum time to reach target
 * @return ESP_OK if reached, ESP_ERR_TIMEOUT if not
 */
esp_err_t motor_pot_goto_position(float target_percent, uint32_t timeout_ms);

/**
 * @brief Set power level (alias for goto_position)
 * 
 * @param power_percent Power level 0-100%
 * @return ESP_OK on success
 */
esp_err_t motor_pot_set_power(float power_percent);

/**
 * @brief Get current power level
 * 
 * @return Power level 0-100%
 */
float motor_pot_get_power(void);

/**
 * @brief Get current resistance
 * 
 * @return Resistance in ohms
 */
uint16_t motor_pot_get_resistance(void);

/**
 * @brief Move to home position (0%)
 * 
 * @return ESP_OK on success
 */
esp_err_t motor_pot_home(void);

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Run automatic calibration
 * 
 * Finds end stops by driving until stall detected.
 * Updates internal calibration data.
 * 
 * @return ESP_OK on success
 */
esp_err_t motor_pot_calibrate(void);

/**
 * @brief Set calibration values manually
 * 
 * @param adc_min ADC value at 0%
 * @param adc_max ADC value at 100%
 */
void motor_pot_set_calibration(uint16_t adc_min, uint16_t adc_max);

/**
 * @brief Get current calibration
 * 
 * @param cal Output calibration data
 * @return ESP_OK if calibrated
 */
esp_err_t motor_pot_get_calibration(motor_pot_calibration_t *cal);

// ============================================================================
// State
// ============================================================================

/**
 * @brief Get complete motor pot state
 * 
 * @param state Output state structure
 * @return ESP_OK on success
 */
esp_err_t motor_pot_get_state(motor_pot_state_t *state);

#endif // MOTOR_POT_H
