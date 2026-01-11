/**
 * @file o3_power_control.c
 * @brief Ozone generator power control implementation
 * 
 * Updated to use motorized potentiometer (PRM162 + DRV8833) instead of
 * digital potentiometer (DS3502) which had ground isolation issues.
 */

#include "o3_power_control.h"
#include "motor_pot.h"
#include "blocksi_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "O3_POWER";

// Original pot specifications
#define ORIGINAL_POT_OHMS       5000    // PRM162 is 5kΩ (close to original 4.7kΩ)

// Calibration model coefficients (from previous characterization)
// O3_max = 1.78/F + 1.40 where F = flow rate in LPM
#define CAL_COEFF_A     1.78f
#define CAL_COEFF_B     1.40f

// Power zones
#define POWER_ZONE_THRESHOLD    20.0f   // Below this, minimal output
#define POWER_ZONE_SATURATION   75.0f   // Above this, diminishing returns

// Module state
static struct {
    bool initialized;
    o3_power_mode_t mode;
    o3_power_calibration_t calibration;
    bool calibration_active;
    bool calibration_stop_requested;
} s_power = {
    .initialized = false,
    .mode = O3_POWER_MODE_ORIGINAL,
    .calibration_active = false,
    .calibration_stop_requested = false
};

// ============================================================================
// Initialization
// ============================================================================

esp_err_t o3_power_init(void)
{
    ESP_LOGI(TAG, "Initializing O3 power control with motorized potentiometer");
    
    // Check if motor pot is already initialized (by peripherals_init_dac)
    if (!motor_pot_is_initialized()) {
        // Initialize motor pot with pin assignments from blocksi_pins.h
        motor_pot_config_t config = {
            .ain1_gpio = MOTOR_POT_AIN1_GPIO,
            .ain2_gpio = MOTOR_POT_AIN2_GPIO,
            .slp_gpio = MOTOR_POT_SLP_GPIO,
            .adc_gpio = MOTOR_POT_ADC_GPIO,
            .pot_ohms = MOTOR_POT_RESISTANCE,
            .invert_direction = false   // Adjust if needed after testing
        };
        
        esp_err_t ret = motor_pot_init(&config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize motor pot: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Home to minimum position (safe state - generator off)
        ESP_LOGI(TAG, "Homing to minimum position...");
        ret = motor_pot_home();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Homing failed, continuing anyway");
        }
    } else {
        ESP_LOGI(TAG, "Motor pot already initialized");
    }
    
    // Initialize calibration data
    memset(&s_power.calibration, 0, sizeof(s_power.calibration));
    s_power.calibration.valid = false;
    
    s_power.initialized = true;
    ESP_LOGI(TAG, "O3 power control initialized");
    
    return ESP_OK;
}

void o3_power_deinit(void)
{
    if (s_power.initialized) {
        // Stop any ongoing calibration
        o3_power_stop_calibration();
        
        // Set to safe state
        o3_power_emergency_stop();
        
        motor_pot_deinit();
        s_power.initialized = false;
    }
    ESP_LOGI(TAG, "O3 power control deinitialized");
}

bool o3_power_is_initialized(void)
{
    return s_power.initialized;
}

// ============================================================================
// Power Control
// ============================================================================

esp_err_t o3_power_set_percent(float percent)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    ESP_LOGI(TAG, "Setting power to %.1f%%", percent);
    
    return motor_pot_set_power(percent);
}

float o3_power_get_percent(void)
{
    if (!s_power.initialized) {
        return 0;
    }
    
    return motor_pot_get_power();
}

esp_err_t o3_power_set_wiper(uint8_t wiper)
{
    // For motor pot, convert wiper (0-127) to percentage
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    float percent = (wiper / 127.0f) * 100.0f;
    ESP_LOGD(TAG, "Setting wiper %u -> %.1f%%", wiper, percent);
    return motor_pot_set_power(percent);
}

uint8_t o3_power_get_wiper(void)
{
    // For motor pot, convert percentage back to pseudo-wiper (0-127)
    if (!s_power.initialized) {
        return 0;
    }
    
    float percent = motor_pot_get_power();
    return (uint8_t)((percent / 100.0f) * 127.0f + 0.5f);
}

esp_err_t o3_power_set_resistance(uint16_t ohms)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert resistance to percentage (0-5000 ohms -> 0-100%)
    float percent = (ohms / 5000.0f) * 100.0f;
    if (percent > 100) percent = 100;
    
    ESP_LOGI(TAG, "Setting resistance %u ohms -> %.1f%%", ohms, percent);
    return motor_pot_set_power(percent);
}

uint16_t o3_power_get_resistance(void)
{
    if (!s_power.initialized) {
        return 0;
    }
    return motor_pot_get_resistance();
}

void o3_power_set_mode(o3_power_mode_t mode)
{
    s_power.mode = mode;
    ESP_LOGI(TAG, "Power mode set to %s",
             mode == O3_POWER_MODE_ORIGINAL ? "original" : "extended");
}

o3_power_mode_t o3_power_get_mode(void)
{
    return s_power.mode;
}

void o3_power_emergency_stop(void)
{
    ESP_LOGW(TAG, "EMERGENCY STOP - Setting power to 0");
    if (s_power.initialized) {
        motor_pot_set_power(0);
    }
}

// ============================================================================
// State and Prediction
// ============================================================================

esp_err_t o3_power_get_state(o3_power_state_t *state)
{
    if (!s_power.initialized || state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    motor_pot_state_t mp_state;
    esp_err_t ret = motor_pot_get_state(&mp_state);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Map motor pot state to o3 power state
    state->wiper_position = (uint8_t)((mp_state.position_percent / 100.0f) * 127.0f);
    state->resistance_ohms = mp_state.resistance_ohms;
    state->power_percent = mp_state.power_percent;
    state->mode = s_power.mode;
    
    // Predict O3 output (assuming 5 LPM default flow)
    state->predicted_o3_ppm = o3_power_predict_o3(state->power_percent, 5.0f);
    
    return ESP_OK;
}

float o3_power_predict_o3(float percent, float flow_lpm)
{
    // If we have valid calibration data, use interpolation
    if (s_power.calibration.valid && s_power.calibration.point_count > 1) {
        // Find bracketing points and interpolate
        // For now, use simple model
    }
    
    // Default model: O3_max = 1.78/F + 1.40 (from previous characterization)
    float o3_max = CAL_COEFF_A / flow_lpm + CAL_COEFF_B;
    
    // Apply power zone scaling
    float scaling;
    if (percent < POWER_ZONE_THRESHOLD) {
        // Below threshold - minimal output
        scaling = (percent / POWER_ZONE_THRESHOLD) * 0.1f;  // Max 10% output
    } else if (percent > POWER_ZONE_SATURATION) {
        // Saturation zone - diminishing returns
        float excess = percent - POWER_ZONE_SATURATION;
        float base = (POWER_ZONE_SATURATION - POWER_ZONE_THRESHOLD) / 
                     (100.0f - POWER_ZONE_THRESHOLD);
        scaling = base + (excess / (100.0f - POWER_ZONE_SATURATION)) * (1.0f - base);
    } else {
        // Linear zone
        scaling = (percent - POWER_ZONE_THRESHOLD) / 
                  (100.0f - POWER_ZONE_THRESHOLD);
    }
    
    return o3_max * scaling;
}

// ============================================================================
// Calibration
// ============================================================================

esp_err_t o3_power_start_calibration(uint8_t start_wiper, uint8_t end_wiper,
                                      uint8_t step_size, uint32_t hold_time_ms,
                                      o3_power_sweep_callback_t callback,
                                      void *user_data)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_power.calibration_active) {
        ESP_LOGW(TAG, "Calibration already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (step_size == 0) step_size = 1;
    if (hold_time_ms < 1000) hold_time_ms = 1000;  // Minimum 1 second hold
    
    // Convert wiper (0-127) to percentage
    float start_pct = (start_wiper / 127.0f) * 100.0f;
    float end_pct = (end_wiper / 127.0f) * 100.0f;
    float step_pct = (step_size / 127.0f) * 100.0f;
    
    ESP_LOGI(TAG, "Starting calibration: %.1f%% -> %.1f%%, step=%.1f%%, hold=%ums",
             start_pct, end_pct, step_pct, (unsigned)hold_time_ms);
    
    s_power.calibration_active = true;
    s_power.calibration_stop_requested = false;
    
    // Clear previous calibration
    memset(&s_power.calibration, 0, sizeof(s_power.calibration));
    s_power.calibration.valid = false;
    
    // Calculate total steps for progress reporting
    int total_steps = (int)(fabsf(end_pct - start_pct) / step_pct) + 1;
    int current_step = 0;
    
    // Direction
    float direction = (end_pct >= start_pct) ? 1.0f : -1.0f;
    
    // Sweep through positions
    for (float pct = start_pct; 
         (direction > 0) ? (pct <= end_pct) : (pct >= end_pct);
         pct += direction * step_pct) {
        
        // Check for stop request
        if (s_power.calibration_stop_requested) {
            ESP_LOGI(TAG, "Calibration stopped by request");
            break;
        }
        
        // Move to position
        esp_err_t ret = motor_pot_set_power(pct);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set position to %.1f%%", pct);
            continue;
        }
        
        // Wait for hold time
        vTaskDelay(pdMS_TO_TICKS(hold_time_ms));
        
        // Call callback with progress
        float progress = (float)current_step / (float)total_steps;
        uint8_t wiper = (uint8_t)((pct / 100.0f) * 127.0f);
        if (callback) {
            callback(wiper, 0, progress, user_data);  // O3 reading filled by caller
        }
        
        // Store calibration point (O3 reading will be filled externally)
        if (s_power.calibration.point_count < 128) {
            o3_power_cal_point_t *pt = &s_power.calibration.points[s_power.calibration.point_count];
            pt->wiper = wiper;
            pt->hold_time_ms = hold_time_ms;
            pt->sample_count = 0;
            pt->o3_mean_ppm = 0;
            pt->o3_std_ppm = 0;
            s_power.calibration.point_count++;
        }
        
        current_step++;
    }
    
    // Return to safe state
    motor_pot_set_power(0);
    
    s_power.calibration_active = false;
    s_power.calibration.valid = (s_power.calibration.point_count > 0);
    
    ESP_LOGI(TAG, "Calibration complete: %u points collected",
             s_power.calibration.point_count);
    
    return ESP_OK;
}

void o3_power_stop_calibration(void)
{
    if (s_power.calibration_active) {
        ESP_LOGI(TAG, "Requesting calibration stop");
        s_power.calibration_stop_requested = true;
    }
}

bool o3_power_calibration_active(void)
{
    return s_power.calibration_active;
}

esp_err_t o3_power_get_calibration(o3_power_calibration_t *cal)
{
    if (cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_power.calibration.valid) {
        return ESP_ERR_NOT_FOUND;
    }
    
    memcpy(cal, &s_power.calibration, sizeof(o3_power_calibration_t));
    return ESP_OK;
}

void o3_power_clear_calibration(void)
{
    memset(&s_power.calibration, 0, sizeof(s_power.calibration));
    s_power.calibration.valid = false;
    ESP_LOGI(TAG, "Calibration data cleared");
}

// ============================================================================
// Backward-Compatible API (matches old MCP4725 DAC interface)
// ============================================================================

uint8_t o3_power_get(void)
{
    return (uint8_t)o3_power_get_percent();
}

esp_err_t o3_power_set(uint8_t power_pct)
{
    return o3_power_set_percent((float)power_pct);
}

float o3_power_get_voltage(void)
{
    // Legacy compatibility: return a pseudo-voltage for display purposes
    // Maps 0-100% to 0-3.3V (as the old DAC would have)
    return (o3_power_get_percent() / 100.0f) * 3.3f;
}
