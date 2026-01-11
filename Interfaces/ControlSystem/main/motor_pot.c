/**
 * @file motor_pot.c
 * @brief Motorized Potentiometer Driver Implementation
 * 
 * Controls PRM162 motorized potentiometer via DRV8833 H-bridge.
 * Uses PWM for motor speed control and ADC for position feedback.
 */

#include "motor_pot.h"
#include "blocksi_pins.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MOTOR_POT";

// ADC attenuation for 0-3.3V range
#define MOTOR_POT_ADC_ATTEN     ADC_ATTEN_DB_12

// ============================================================================
// Module State
// ============================================================================

static struct {
    bool initialized;
    bool enabled;
    
    // Pin assignments
    int ain1_gpio;
    int ain2_gpio;
    int slp_gpio;
    int adc_gpio;
    
    // PWM channels
    ledc_channel_t pwm_ch_ain1;
    ledc_channel_t pwm_ch_ain2;
    
    // ADC handle
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t adc_channel;
    
    // Calibration
    motor_pot_calibration_t calibration;
    
    // Configuration
    uint16_t pot_ohms;
    bool invert_direction;
    
    // Current state
    motor_direction_t current_direction;
    uint8_t current_pwm;
    uint16_t last_adc_reading;
    
} s_motor = {
    .initialized = false,
    .enabled = false,
    .ain1_gpio = MOTOR_POT_AIN1_GPIO,
    .ain2_gpio = MOTOR_POT_AIN2_GPIO,
    .slp_gpio = MOTOR_POT_SLP_GPIO,
    .adc_gpio = MOTOR_POT_ADC_GPIO,
    .pwm_ch_ain1 = LEDC_CHANNEL_0,
    .pwm_ch_ain2 = LEDC_CHANNEL_1,
    .pot_ohms = MOTOR_POT_RESISTANCE,
    .invert_direction = false,
    .calibration = {
        .adc_min = POSITION_ADC_MIN,
        .adc_max = POSITION_ADC_MAX,
        .calibrated = false
    }
};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Initialize PWM for motor control
 */
static esp_err_t init_pwm(void)
{
    esp_err_t ret;
    
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure AIN1 channel
    ledc_channel_config_t ch1_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = s_motor.pwm_ch_ain1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_motor.ain1_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ch1_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure AIN1 PWM: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure AIN2 channel
    ledc_channel_config_t ch2_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = s_motor.pwm_ch_ain2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_motor.ain2_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ch2_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure AIN2 PWM: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PWM initialized: AIN1=GPIO%d, AIN2=GPIO%d, freq=%dHz",
             s_motor.ain1_gpio, s_motor.ain2_gpio, MOTOR_PWM_FREQ_HZ);
    
    return ESP_OK;
}

/**
 * @brief Initialize ADC for position feedback
 */
static esp_err_t init_adc(void)
{
    esp_err_t ret;
    
    // Determine ADC channel from GPIO
    adc_unit_t unit;
    ret = adc_oneshot_io_to_channel(s_motor.adc_gpio, &unit, &s_motor.adc_channel);
    if (ret != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d is not a valid ADC1 pin", s_motor.adc_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ret = adc_oneshot_new_unit(&unit_cfg, &s_motor.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = MOTOR_POT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12
    };
    ret = adc_oneshot_config_channel(s_motor.adc_handle, s_motor.adc_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_motor.adc_handle);
        return ret;
    }
    
    ESP_LOGI(TAG, "ADC initialized: GPIO%d, channel=%d", 
             s_motor.adc_gpio, s_motor.adc_channel);
    
    return ESP_OK;
}

/**
 * @brief Initialize sleep/enable pin
 */
static esp_err_t init_sleep_pin(void)
{
    if (s_motor.slp_gpio < 0) {
        return ESP_OK;  // Not used
    }
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_motor.slp_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SLP pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start with driver disabled
    gpio_set_level(s_motor.slp_gpio, 0);
    
    ESP_LOGI(TAG, "Sleep pin initialized: GPIO%d", s_motor.slp_gpio);
    return ESP_OK;
}

/**
 * @brief Read ADC with averaging
 */
static uint16_t read_adc_averaged(int num_samples)
{
    if (!s_motor.initialized) return 0;
    
    int32_t sum = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < num_samples; i++) {
        int raw;
        if (adc_oneshot_read(s_motor.adc_handle, s_motor.adc_channel, &raw) == ESP_OK) {
            sum += raw;
            valid_samples++;
        }
        if (num_samples > 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    if (valid_samples == 0) return s_motor.last_adc_reading;
    
    uint16_t avg = (uint16_t)(sum / valid_samples);
    s_motor.last_adc_reading = avg;
    return avg;
}

/**
 * @brief Convert ADC value to position percentage
 */
static float adc_to_percent(uint16_t adc_value)
{
    int32_t range = s_motor.calibration.adc_max - s_motor.calibration.adc_min;
    if (range <= 0) range = POSITION_ADC_MAX - POSITION_ADC_MIN;
    
    int32_t adjusted = adc_value - s_motor.calibration.adc_min;
    float percent = (adjusted * 100.0f) / range;
    
    // Clamp to 0-100
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    return percent;
}

/**
 * @brief Convert position percentage to target ADC value
 */
static uint16_t percent_to_adc(float percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    int32_t range = s_motor.calibration.adc_max - s_motor.calibration.adc_min;
    if (range <= 0) range = POSITION_ADC_MAX - POSITION_ADC_MIN;
    
    return (uint16_t)(s_motor.calibration.adc_min + (percent / 100.0f) * range);
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t motor_pot_init(const motor_pot_config_t *config)
{
    esp_err_t ret;
    
    if (s_motor.initialized) {
        ESP_LOGW(TAG, "Already initialized, deinitializing first");
        motor_pot_deinit();
    }
    
    // Apply configuration
    if (config != NULL) {
        s_motor.ain1_gpio = config->ain1_gpio;
        s_motor.ain2_gpio = config->ain2_gpio;
        s_motor.slp_gpio = config->slp_gpio;
        s_motor.adc_gpio = config->adc_gpio;
        s_motor.pot_ohms = config->pot_ohms > 0 ? config->pot_ohms : MOTOR_POT_RESISTANCE;
        s_motor.invert_direction = config->invert_direction;
    }
    
    ESP_LOGI(TAG, "Initializing motor pot driver");
    ESP_LOGI(TAG, "  AIN1: GPIO%d, AIN2: GPIO%d, SLP: GPIO%d",
             s_motor.ain1_gpio, s_motor.ain2_gpio, s_motor.slp_gpio);
    ESP_LOGI(TAG, "  ADC: GPIO%d, Pot: %u ohms",
             s_motor.adc_gpio, s_motor.pot_ohms);
    
    // Initialize PWM
    ret = init_pwm();
    if (ret != ESP_OK) return ret;
    
    // Initialize ADC
    ret = init_adc();
    if (ret != ESP_OK) return ret;
    
    // Initialize sleep pin
    ret = init_sleep_pin();
    if (ret != ESP_OK) return ret;
    
    s_motor.initialized = true;
    s_motor.current_direction = MOTOR_DIR_STOP;
    s_motor.current_pwm = 0;
    
    // Take initial reading
    s_motor.last_adc_reading = read_adc_averaged(5);
    
    ESP_LOGI(TAG, "Motor pot initialized, initial ADC: %u (%.1f%%)",
             s_motor.last_adc_reading, 
             adc_to_percent(s_motor.last_adc_reading));
    
    return ESP_OK;
}

void motor_pot_deinit(void)
{
    if (!s_motor.initialized) return;
    
    // Stop motor first
    motor_pot_stop();
    motor_pot_enable(false);
    
    // Clean up ADC
    if (s_motor.adc_handle) {
        adc_oneshot_del_unit(s_motor.adc_handle);
        s_motor.adc_handle = NULL;
    }
    
    s_motor.initialized = false;
    ESP_LOGI(TAG, "Motor pot deinitialized");
}

bool motor_pot_is_initialized(void)
{
    return s_motor.initialized;
}

esp_err_t motor_pot_enable(bool enable)
{
    if (s_motor.slp_gpio < 0) {
        s_motor.enabled = enable;
        return ESP_OK;
    }
    
    gpio_set_level(s_motor.slp_gpio, enable ? 1 : 0);
    s_motor.enabled = enable;
    
    ESP_LOGD(TAG, "Motor driver %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Motor Control
// ----------------------------------------------------------------------------

esp_err_t motor_pot_set_motor(motor_direction_t direction, uint8_t pwm_duty,
                               motor_decay_mode_t decay_mode)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Handle direction inversion
    motor_direction_t actual_dir = direction;
    if (s_motor.invert_direction) {
        if (direction == MOTOR_DIR_FORWARD) actual_dir = MOTOR_DIR_REVERSE;
        else if (direction == MOTOR_DIR_REVERSE) actual_dir = MOTOR_DIR_FORWARD;
    }
    
    uint32_t duty1 = 0, duty2 = 0;
    
    switch (actual_dir) {
        case MOTOR_DIR_STOP:
            // Coast: both LOW
            duty1 = 0;
            duty2 = 0;
            break;
            
        case MOTOR_DIR_FORWARD:
            if (decay_mode == MOTOR_DECAY_FAST) {
                // Fast decay forward: PWM on AIN1, LOW on AIN2
                duty1 = pwm_duty;
                duty2 = 0;
            } else {
                // Slow decay forward: HIGH on AIN1, PWM on AIN2
                duty1 = 255;
                duty2 = 255 - pwm_duty;
            }
            break;
            
        case MOTOR_DIR_REVERSE:
            if (decay_mode == MOTOR_DECAY_FAST) {
                // Fast decay reverse: LOW on AIN1, PWM on AIN2
                duty1 = 0;
                duty2 = pwm_duty;
            } else {
                // Slow decay reverse: PWM on AIN1, HIGH on AIN2
                duty1 = 255 - pwm_duty;
                duty2 = 255;
            }
            break;
            
        case MOTOR_DIR_BRAKE:
            // Active brake: both HIGH
            duty1 = 255;
            duty2 = 255;
            break;
    }
    
    // Apply PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain1, duty1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain2, duty2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain2);
    
    s_motor.current_direction = direction;
    s_motor.current_pwm = pwm_duty;
    
    ESP_LOGD(TAG, "Motor: dir=%d, duty=%u, decay=%s",
             direction, pwm_duty, decay_mode == MOTOR_DECAY_FAST ? "fast" : "slow");
    
    return ESP_OK;
}

esp_err_t motor_pot_stop(void)
{
    return motor_pot_set_motor(MOTOR_DIR_STOP, 0, MOTOR_DECAY_FAST);
}

esp_err_t motor_pot_brake(void)
{
    return motor_pot_set_motor(MOTOR_DIR_BRAKE, 255, MOTOR_DECAY_FAST);
}

uint32_t motor_pot_pulse(motor_direction_t direction, uint32_t duration_ms,
                          uint8_t pwm_duty)
{
    if (!s_motor.initialized || direction == MOTOR_DIR_STOP) {
        return 0;
    }
    
    // Enable driver
    motor_pot_enable(true);
    
    // Read starting position
    uint16_t start_adc = read_adc_averaged(3);
    
    // Start motor
    motor_pot_set_motor(direction, pwm_duty, MOTOR_DECAY_FAST);
    
    // Run for duration
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t end_tick = start_tick + pdMS_TO_TICKS(duration_ms);
    
    while (xTaskGetTickCount() < end_tick) {
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Check if we've hit a limit (ADC stopped changing)
        uint16_t current_adc = read_adc_averaged(1);
        
        // End-stop detection: if at extreme and trying to go further
        if (direction == MOTOR_DIR_FORWARD && current_adc >= s_motor.calibration.adc_max - 50) {
            break;
        }
        if (direction == MOTOR_DIR_REVERSE && current_adc <= s_motor.calibration.adc_min + 50) {
            break;
        }
    }
    
    // Stop motor
    motor_pot_stop();
    
    // Calculate actual duration
    uint32_t actual_ms = pdTICKS_TO_MS(xTaskGetTickCount() - start_tick);
    
    // Settle time
    vTaskDelay(pdMS_TO_TICKS(MOTOR_SETTLE_MS));
    
    // Read final position
    uint16_t end_adc = read_adc_averaged(3);
    
    ESP_LOGD(TAG, "Pulse: dir=%d, requested=%ums, actual=%ums, ADC: %u->%u",
             direction, (unsigned)duration_ms, (unsigned)actual_ms, start_adc, end_adc);
    
    return actual_ms;
}

// ----------------------------------------------------------------------------
// Position Control
// ----------------------------------------------------------------------------

esp_err_t motor_pot_read_position(uint16_t *adc_value)
{
    if (!s_motor.initialized || adc_value == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    *adc_value = read_adc_averaged(3);
    return ESP_OK;
}

float motor_pot_get_position_percent(void)
{
    if (!s_motor.initialized) return 0;
    
    uint16_t adc = read_adc_averaged(3);
    return adc_to_percent(adc);
}

uint16_t motor_pot_get_resistance(void)
{
    float percent = motor_pot_get_position_percent();
    return (uint16_t)((percent / 100.0f) * s_motor.pot_ohms);
}

esp_err_t motor_pot_goto_position(float target_percent, uint32_t timeout_ms)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Moving to %.1f%%, timeout=%ums", target_percent, (unsigned)timeout_ms);
    
    // Enable driver
    motor_pot_enable(true);
    
    uint16_t target_adc = percent_to_adc(target_percent);
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_tick = start_tick + pdMS_TO_TICKS(timeout_ms);
    
    int stall_count = 0;
    uint16_t last_adc = 0;
    
    while (xTaskGetTickCount() < timeout_tick) {
        uint16_t current_adc = read_adc_averaged(3);
        int32_t error = (int32_t)target_adc - (int32_t)current_adc;
        
        // Check if within deadband
        if (abs(error) <= POSITION_DEADBAND) {
            ESP_LOGI(TAG, "Position reached: ADC=%u (target=%u)", current_adc, target_adc);
            motor_pot_stop();
            motor_pot_enable(false);
            return ESP_OK;
        }
        
        // Determine direction and speed
        motor_direction_t dir = (error > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
        uint8_t pwm = (abs(error) > POSITION_FINE_THRESHOLD) ? 
                       MOTOR_PWM_DEFAULT : MOTOR_PWM_SLOW;
        
        // Run motor
        motor_pot_set_motor(dir, pwm, MOTOR_DECAY_FAST);
        
        // Brief movement
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Stall detection
        if (abs(current_adc - last_adc) < 5) {
            stall_count++;
            if (stall_count > 20) {
                ESP_LOGW(TAG, "Motor stalled at ADC=%u", current_adc);
                motor_pot_stop();
                motor_pot_enable(false);
                return ESP_ERR_TIMEOUT;
            }
        } else {
            stall_count = 0;
        }
        last_adc = current_adc;
    }
    
    motor_pot_stop();
    motor_pot_enable(false);
    
    ESP_LOGW(TAG, "Timeout reaching position. Current: %.1f%%, Target: %.1f%%",
             motor_pot_get_position_percent(), target_percent);
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t motor_pot_set_power(float power_percent)
{
    if (power_percent < 0) power_percent = 0;
    if (power_percent > 100) power_percent = 100;
    
    ESP_LOGI(TAG, "Setting power to %.1f%%", power_percent);
    
    return motor_pot_goto_position(power_percent, MOTOR_TIMEOUT_MS);
}

float motor_pot_get_power(void)
{
    return motor_pot_get_position_percent();
}

// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

esp_err_t motor_pot_calibrate(void)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting calibration...");
    
    motor_pot_enable(true);
    
    // Move to minimum (reverse until stall)
    ESP_LOGI(TAG, "Finding minimum position...");
    uint16_t last_adc = read_adc_averaged(5);
    int stall_count = 0;
    
    motor_pot_set_motor(MOTOR_DIR_REVERSE, MOTOR_PWM_DEFAULT, MOTOR_DECAY_FAST);
    
    for (int i = 0; i < 500; i++) {  // Max 5 seconds
        vTaskDelay(pdMS_TO_TICKS(10));
        uint16_t current_adc = read_adc_averaged(1);
        
        if (abs(current_adc - last_adc) < 3) {
            stall_count++;
            if (stall_count > 30) {
                break;  // Stalled at minimum
            }
        } else {
            stall_count = 0;
        }
        last_adc = current_adc;
    }
    motor_pot_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    s_motor.calibration.adc_min = read_adc_averaged(10);
    ESP_LOGI(TAG, "Minimum ADC: %u", s_motor.calibration.adc_min);
    
    // Move to maximum (forward until stall)
    ESP_LOGI(TAG, "Finding maximum position...");
    stall_count = 0;
    last_adc = s_motor.calibration.adc_min;
    
    motor_pot_set_motor(MOTOR_DIR_FORWARD, MOTOR_PWM_DEFAULT, MOTOR_DECAY_FAST);
    
    for (int i = 0; i < 500; i++) {  // Max 5 seconds
        vTaskDelay(pdMS_TO_TICKS(10));
        uint16_t current_adc = read_adc_averaged(1);
        
        if (abs(current_adc - last_adc) < 3) {
            stall_count++;
            if (stall_count > 30) {
                break;  // Stalled at maximum
            }
        } else {
            stall_count = 0;
        }
        last_adc = current_adc;
    }
    motor_pot_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    s_motor.calibration.adc_max = read_adc_averaged(10);
    ESP_LOGI(TAG, "Maximum ADC: %u", s_motor.calibration.adc_max);
    
    // Validate calibration
    int range = s_motor.calibration.adc_max - s_motor.calibration.adc_min;
    if (range < 1000) {
        ESP_LOGE(TAG, "Calibration failed: range too small (%d)", range);
        s_motor.calibration.calibrated = false;
        motor_pot_enable(false);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    s_motor.calibration.calibrated = true;
    ESP_LOGI(TAG, "Calibration complete: min=%u, max=%u, range=%d",
             s_motor.calibration.adc_min, s_motor.calibration.adc_max, range);
    
    motor_pot_enable(false);
    return ESP_OK;
}

esp_err_t motor_pot_home(void)
{
    ESP_LOGI(TAG, "Homing to minimum position...");
    return motor_pot_goto_position(0, MOTOR_TIMEOUT_MS);
}

esp_err_t motor_pot_set_calibration(uint16_t adc_min, uint16_t adc_max)
{
    if (adc_max <= adc_min || adc_max - adc_min < 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_motor.calibration.adc_min = adc_min;
    s_motor.calibration.adc_max = adc_max;
    s_motor.calibration.calibrated = true;
    
    ESP_LOGI(TAG, "Calibration set: min=%u, max=%u", adc_min, adc_max);
    return ESP_OK;
}

esp_err_t motor_pot_get_calibration(motor_pot_calibration_t *cal)
{
    if (cal == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(cal, &s_motor.calibration, sizeof(motor_pot_calibration_t));
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// State and Diagnostics
// ----------------------------------------------------------------------------

esp_err_t motor_pot_get_state(motor_pot_state_t *state)
{
    if (!s_motor.initialized || state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    state->adc_raw = read_adc_averaged(3);
    state->position_percent = adc_to_percent(state->adc_raw);
    state->resistance_ohms = (uint16_t)((state->position_percent / 100.0f) * s_motor.pot_ohms);
    state->power_percent = state->position_percent;
    state->direction = s_motor.current_direction;
    state->is_moving = (s_motor.current_direction != MOTOR_DIR_STOP && 
                        s_motor.current_direction != MOTOR_DIR_BRAKE);
    
    return ESP_OK;
}

void motor_pot_print_status(void)
{
    if (!s_motor.initialized) {
        ESP_LOGI(TAG, "Motor pot not initialized");
        return;
    }
    
    motor_pot_state_t state;
    motor_pot_get_state(&state);
    
    ESP_LOGI(TAG, "=== Motor Pot Status ===");
    ESP_LOGI(TAG, "  ADC Raw:    %u", state.adc_raw);
    ESP_LOGI(TAG, "  Position:   %.1f%%", state.position_percent);
    ESP_LOGI(TAG, "  Resistance: %u ohms", state.resistance_ohms);
    ESP_LOGI(TAG, "  Power:      %.1f%%", state.power_percent);
    ESP_LOGI(TAG, "  Moving:     %s", state.is_moving ? "Yes" : "No");
    ESP_LOGI(TAG, "  Calibrated: %s", s_motor.calibration.calibrated ? "Yes" : "No");
    ESP_LOGI(TAG, "  Cal Range:  %u - %u", 
             s_motor.calibration.adc_min, s_motor.calibration.adc_max);
}
