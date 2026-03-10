/**
 * @file motor_pot.c
 * @brief Motorized Potentiometer Driver Implementation
 * 
 * Controls PRM162 motorized potentiometer via DRV8833 H-bridge.
 * Uses PWM for motor speed control and ADC for position feedback.
 * 
 * Note: DRV8833 SLP pin is tied to 5V on PCB (always enabled).
 */

#include "motor_pot.h"
#include "blocksi_pins.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MOTOR_POT";

// ADC attenuation for 0-3.3V range
#define MOTOR_POT_ADC_ATTEN     ADC_ATTEN_DB_12

// Stall detection parameters
#define STALL_ADC_THRESHOLD     8       // Minimum ADC change to consider movement
#define STALL_COUNT_LIMIT       30      // Iterations before declaring stall (30 * 25ms = 750ms)

// ============================================================================
// Module State
// ============================================================================

// Mutex for thread-safe motor operations
static SemaphoreHandle_t s_motor_mutex = NULL;

static struct {
    bool initialized;
    
    // Pin assignments
    int ain1_gpio;
    int ain2_gpio;
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
    .ain1_gpio = MOTOR_POT_AIN1_GPIO,
    .ain2_gpio = MOTOR_POT_AIN2_GPIO,
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
// Internal Helpers
// ============================================================================

/**
 * @brief Convert GPIO number to ADC channel
 */
static adc_channel_t gpio_to_adc_channel(int gpio)
{
    // ADC1 channels on ESP32
    switch (gpio) {
        case 36: return ADC_CHANNEL_0;
        case 37: return ADC_CHANNEL_1;
        case 38: return ADC_CHANNEL_2;
        case 39: return ADC_CHANNEL_3;
        case 32: return ADC_CHANNEL_4;
        case 33: return ADC_CHANNEL_5;
        case 34: return ADC_CHANNEL_6;
        case 35: return ADC_CHANNEL_7;
        default: return ADC_CHANNEL_6;  // Default to GPIO34
    }
}

/**
 * @brief Read ADC with averaging
 */
static uint16_t read_adc_averaged(int samples)
{
    if (!s_motor.initialized || s_motor.adc_handle == NULL) {
        return 0;
    }
    
    int32_t sum = 0;
    int valid = 0;
    
    for (int i = 0; i < samples; i++) {
        int raw;
        if (adc_oneshot_read(s_motor.adc_handle, s_motor.adc_channel, &raw) == ESP_OK) {
            sum += raw;
            valid++;
        }
        if (i < samples - 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    if (valid == 0) return s_motor.last_adc_reading;
    
    s_motor.last_adc_reading = (uint16_t)(sum / valid);
    return s_motor.last_adc_reading;
}

/**
 * @brief Convert ADC to percentage using calibration
 */
static float adc_to_percent(uint16_t adc)
{
    int32_t range = s_motor.calibration.adc_max - s_motor.calibration.adc_min;
    if (range <= 0) return 0;
    
    int32_t offset = adc - s_motor.calibration.adc_min;
    float pct = (offset * 100.0f) / range;
    
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    
    return pct;
}

/**
 * @brief Convert percentage to target ADC
 */
static uint16_t percent_to_adc(float percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    int32_t range = s_motor.calibration.adc_max - s_motor.calibration.adc_min;
    return s_motor.calibration.adc_min + (uint16_t)((percent / 100.0f) * range);
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t motor_pot_init(const motor_pot_config_t *config)
{
    if (s_motor.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing motor pot driver");
    
    // Create mutex for thread-safe operations
    if (s_motor_mutex == NULL) {
        s_motor_mutex = xSemaphoreCreateMutex();
        if (s_motor_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Apply configuration
    if (config != NULL) {
        s_motor.ain1_gpio = config->ain1_gpio;
        s_motor.ain2_gpio = config->ain2_gpio;
        s_motor.adc_gpio = config->adc_gpio;
        s_motor.pot_ohms = config->pot_ohms > 0 ? config->pot_ohms : MOTOR_POT_RESISTANCE;
        s_motor.invert_direction = config->invert_direction;
    }
    
    ESP_LOGI(TAG, "  AIN1: GPIO%d, AIN2: GPIO%d", s_motor.ain1_gpio, s_motor.ain2_gpio);
    ESP_LOGI(TAG, "  ADC: GPIO%d", s_motor.adc_gpio);
    ESP_LOGI(TAG, "  Note: SLP tied to 5V (always enabled)");
    
    // Initialize LEDC timer (shared by both channels)
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure AIN1 PWM channel
    ledc_channel_config_t ch1_cfg = {
        .gpio_num = s_motor.ain1_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = s_motor.pwm_ch_ain1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ch1_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel 1 config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure AIN2 PWM channel
    ledc_channel_config_t ch2_cfg = {
        .gpio_num = s_motor.ain2_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = s_motor.pwm_ch_ain2,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ch2_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel 2 config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ADC
    s_motor.adc_channel = gpio_to_adc_channel(s_motor.adc_gpio);
    
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ret = adc_oneshot_new_unit(&adc_cfg, &s_motor.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = MOTOR_POT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12
    };
    ret = adc_oneshot_config_channel(s_motor.adc_handle, s_motor.adc_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_motor.adc_handle);
        return ret;
    }
    
    // Motor starts stopped
    motor_pot_stop();
    
    s_motor.initialized = true;
    
    // Read initial position
    uint16_t adc = read_adc_averaged(5);
    ESP_LOGI(TAG, "Motor pot initialized, ADC=%u (%.1f%%)", 
             adc, adc_to_percent(adc));
    
    return ESP_OK;
}

void motor_pot_deinit(void)
{
    if (!s_motor.initialized) return;
    
    // Stop motor
    motor_pot_stop();
    
    // Release ADC
    if (s_motor.adc_handle) {
        adc_oneshot_del_unit(s_motor.adc_handle);
        s_motor.adc_handle = NULL;
    }
    
    // Stop PWM channels
    ledc_stop(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain1, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain2, 0);
    
    s_motor.initialized = false;
    ESP_LOGI(TAG, "Motor pot deinitialized");
}

bool motor_pot_is_initialized(void)
{
    return s_motor.initialized;
}

// ============================================================================
// Motor Control
// ============================================================================

esp_err_t motor_pot_set_motor(motor_direction_t direction, uint8_t pwm_duty,
                               motor_decay_mode_t decay_mode)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Apply direction inversion if needed
    if (s_motor.invert_direction) {
        if (direction == MOTOR_DIR_FORWARD) direction = MOTOR_DIR_REVERSE;
        else if (direction == MOTOR_DIR_REVERSE) direction = MOTOR_DIR_FORWARD;
    }
    
    uint32_t ain1_duty = 0;
    uint32_t ain2_duty = 0;
    
    switch (direction) {
        case MOTOR_DIR_STOP:
            // Coast - both low
            ain1_duty = 0;
            ain2_duty = 0;
            break;
            
        case MOTOR_DIR_FORWARD:
            if (decay_mode == MOTOR_DECAY_FAST) {
                // Fast decay: PWM on AIN1, LOW on AIN2
                ain1_duty = pwm_duty;
                ain2_duty = 0;
            } else {
                // Slow decay: HIGH on AIN1, PWM complement on AIN2
                ain1_duty = 255;
                ain2_duty = 255 - pwm_duty;
            }
            break;
            
        case MOTOR_DIR_REVERSE:
            if (decay_mode == MOTOR_DECAY_FAST) {
                // Fast decay: LOW on AIN1, PWM on AIN2
                ain1_duty = 0;
                ain2_duty = pwm_duty;
            } else {
                // Slow decay: PWM complement on AIN1, HIGH on AIN2
                ain1_duty = 255 - pwm_duty;
                ain2_duty = 255;
            }
            break;
            
        case MOTOR_DIR_BRAKE:
            // Active brake - both high
            ain1_duty = 255;
            ain2_duty = 255;
            break;
    }
    
    // Update PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain1, ain1_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain2, ain2_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor.pwm_ch_ain2);
    
    s_motor.current_direction = direction;
    s_motor.current_pwm = pwm_duty;
    
    return ESP_OK;
}

void motor_pot_stop(void)
{
    motor_pot_set_motor(MOTOR_DIR_STOP, 0, MOTOR_DECAY_FAST);
}

void motor_pot_brake(void)
{
    motor_pot_set_motor(MOTOR_DIR_BRAKE, 0, MOTOR_DECAY_FAST);
}

esp_err_t motor_pot_pulse(motor_direction_t direction, uint32_t duration_ms,
                           uint8_t pwm_duty)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = motor_pot_set_motor(direction, pwm_duty, MOTOR_DECAY_FAST);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    motor_pot_stop();
    vTaskDelay(pdMS_TO_TICKS(MOTOR_SETTLE_MS));
    
    return ESP_OK;
}

// ============================================================================
// Position Control
// ============================================================================

uint16_t motor_pot_read_adc(void)
{
    return read_adc_averaged(3);
}

float motor_pot_get_position_percent(void)
{
    uint16_t adc = read_adc_averaged(3);
    return adc_to_percent(adc);
}

esp_err_t motor_pot_goto_position(float target_percent, uint32_t timeout_ms)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Acquire mutex to prevent concurrent motor operations
    if (xSemaphoreTake(s_motor_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Motor busy, cannot acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Clamp target
    if (target_percent < 0) target_percent = 0;
    if (target_percent > 100) target_percent = 100;
    
    uint16_t target_adc = percent_to_adc(target_percent);
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    ESP_LOGI(TAG, "Going to %.1f%% (ADC %u), timeout %ums",
             target_percent, target_adc, (unsigned)timeout_ms);
    
    // Control loop
    int stall_count = 0;
    uint16_t last_adc = 0;
    esp_err_t result = ESP_OK;
    
    while (1) {
        // Check timeout
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
        if (elapsed > timeout_ms) {
            motor_pot_stop();
            ESP_LOGW(TAG, "Position timeout at %.1f%%", motor_pot_get_position_percent());
            result = ESP_ERR_TIMEOUT;
            break;
        }
        
        // Read current position with more averaging for noise reduction
        uint16_t current_adc = read_adc_averaged(5);
        int32_t error = (int32_t)target_adc - (int32_t)current_adc;
        
        // Check if within deadband
        if (abs(error) <= POSITION_DEADBAND) {
            motor_pot_stop();
            vTaskDelay(pdMS_TO_TICKS(MOTOR_SETTLE_MS));
            ESP_LOGI(TAG, "Position reached: ADC=%u (%.1f%%)", 
                     current_adc, adc_to_percent(current_adc));
            result = ESP_OK;
            break;
        }
        
        // Stall detection — only active during full-speed approach.
        //
        // In the fine-approach zone (abs(error) < POSITION_FINE_THRESHOLD) the
        // motor runs at MOTOR_PWM_SLOW (~39% duty) which produces only ~4-5 ADC
        // counts per 25 ms loop iteration — below STALL_ADC_THRESHOLD=8.  Running
        // stall detection at slow speed causes a guaranteed false-stall after
        // STALL_COUNT_LIMIT iterations and stops the motor before it reaches the
        // deadband.  In the fine zone we simply let the position loop run until the
        // deadband or the overall timeout is reached.
        if (abs(error) >= POSITION_FINE_THRESHOLD) {
            if (abs((int32_t)current_adc - (int32_t)last_adc) < STALL_ADC_THRESHOLD) {
                stall_count++;
                if (stall_count > STALL_COUNT_LIMIT) {
                    motor_pot_stop();
                    ESP_LOGW(TAG, "Stall detected at ADC=%u (target=%u, error=%ld)",
                             current_adc, target_adc, (long)error);
                    // Accept if close enough (within 2x deadband)
                    result = (abs(error) <= POSITION_DEADBAND * 2) ? ESP_OK : ESP_ERR_TIMEOUT;
                    break;
                }
            } else {
                stall_count = 0;
            }
        } else {
            // Entered fine-approach zone — reset stall counter so any earlier
            // near-miss counts don't carry over when we re-enter at full speed.
            stall_count = 0;
        }
        last_adc = current_adc;
        
        // Determine direction and speed
        motor_direction_t dir;
        uint8_t speed;
        
        if (error > 0) {
            dir = MOTOR_DIR_FORWARD;
        } else {
            dir = MOTOR_DIR_REVERSE;
        }
        
        // Use slow speed when close to target for better precision
        if (abs(error) < POSITION_FINE_THRESHOLD) {
            speed = MOTOR_PWM_SLOW;
        } else {
            speed = MOTOR_PWM_DEFAULT;
        }
        
        // Apply motor control
        motor_pot_set_motor(dir, speed, MOTOR_DECAY_FAST);
        
        // Brief movement then re-check (25ms for smoother control)
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    
    // Release mutex
    xSemaphoreGive(s_motor_mutex);
    return result;
}

esp_err_t motor_pot_set_power(float power_percent)
{
    return motor_pot_goto_position(power_percent, MOTOR_TIMEOUT_MS);
}

float motor_pot_get_power(void)
{
    return motor_pot_get_position_percent();
}

uint16_t motor_pot_get_resistance(void)
{
    float pct = motor_pot_get_position_percent();
    return (uint16_t)((pct / 100.0f) * s_motor.pot_ohms);
}

esp_err_t motor_pot_home(void)
{
    ESP_LOGI(TAG, "Homing to 0%%");
    return motor_pot_goto_position(0, MOTOR_TIMEOUT_MS);
}

// ============================================================================
// Calibration
// ============================================================================

esp_err_t motor_pot_calibrate(void)
{
    if (!s_motor.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting calibration...");
    
    // Drive to minimum (reverse until stall)
    ESP_LOGI(TAG, "Finding minimum...");
    motor_pot_set_motor(MOTOR_DIR_REVERSE, MOTOR_PWM_DEFAULT, MOTOR_DECAY_FAST);
    
    uint16_t last_adc = 0;
    int stall_count = 0;
    
    for (int i = 0; i < 500; i++) {  // Max 10 seconds
        vTaskDelay(pdMS_TO_TICKS(20));
        uint16_t adc = read_adc_averaged(3);
        
        if (abs((int32_t)adc - (int32_t)last_adc) < 5) {
            stall_count++;
            if (stall_count > 10) {
                s_motor.calibration.adc_min = adc;
                ESP_LOGI(TAG, "Minimum found: ADC=%u", adc);
                break;
            }
        } else {
            stall_count = 0;
        }
        last_adc = adc;
    }
    
    motor_pot_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Drive to maximum (forward until stall)
    ESP_LOGI(TAG, "Finding maximum...");
    motor_pot_set_motor(MOTOR_DIR_FORWARD, MOTOR_PWM_DEFAULT, MOTOR_DECAY_FAST);
    
    last_adc = 0;
    stall_count = 0;
    
    for (int i = 0; i < 500; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        uint16_t adc = read_adc_averaged(3);
        
        if (abs((int32_t)adc - (int32_t)last_adc) < 5) {
            stall_count++;
            if (stall_count > 10) {
                s_motor.calibration.adc_max = adc;
                ESP_LOGI(TAG, "Maximum found: ADC=%u", adc);
                break;
            }
        } else {
            stall_count = 0;
        }
        last_adc = adc;
    }
    
    motor_pot_stop();
    
    // Validate calibration
    if (s_motor.calibration.adc_max <= s_motor.calibration.adc_min + 100) {
        ESP_LOGE(TAG, "Calibration failed: range too small");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    s_motor.calibration.calibrated = true;
    ESP_LOGI(TAG, "Calibration complete: min=%u, max=%u, range=%d",
             s_motor.calibration.adc_min, s_motor.calibration.adc_max,
             s_motor.calibration.adc_max - s_motor.calibration.adc_min);
    
    // Return to home
    return motor_pot_home();
}

void motor_pot_set_calibration(uint16_t adc_min, uint16_t adc_max)
{
    s_motor.calibration.adc_min = adc_min;
    s_motor.calibration.adc_max = adc_max;
    s_motor.calibration.calibrated = true;
    ESP_LOGI(TAG, "Manual calibration set: min=%u, max=%u", adc_min, adc_max);
}

esp_err_t motor_pot_get_calibration(motor_pot_calibration_t *cal)
{
    if (cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *cal = s_motor.calibration;
    return s_motor.calibration.calibrated ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// ============================================================================
// State
// ============================================================================

esp_err_t motor_pot_get_state(motor_pot_state_t *state)
{
    if (!s_motor.initialized || state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    state->adc_raw = read_adc_averaged(3);
    state->position_percent = adc_to_percent(state->adc_raw);
    state->power_percent = state->position_percent;
    state->resistance_ohms = (uint16_t)((state->position_percent / 100.0f) * s_motor.pot_ohms);
    state->direction = s_motor.current_direction;
    state->pwm_duty = s_motor.current_pwm;
    state->is_moving = (s_motor.current_direction != MOTOR_DIR_STOP && 
                        s_motor.current_direction != MOTOR_DIR_BRAKE);
    
    return ESP_OK;
}
