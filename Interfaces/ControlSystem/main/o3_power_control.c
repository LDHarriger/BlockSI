/**
 * @file o3_power_control.c
 * @brief Ozone generator power control implementation
 */

#include "o3_power_control.h"
#include "ds3502_digipot.h"
#include "blocksi_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "O3_POWER";

// Original pot specifications
#define ORIGINAL_POT_OHMS       4700
#define ORIGINAL_POT_MAX_WIPER  60      // Approximate wiper for 4.7kΩ on 10kΩ pot

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
    ESP_LOGI(TAG, "Initializing O3 power control with DS3502");
    
    // Configure DS3502
    ds3502_config_t config = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = DS3502_I2C_ADDR,  // From blocksi_pins.h
        .full_scale_ohms = 10000,
        .original_pot_ohms = ORIGINAL_POT_OHMS
    };
    
    esp_err_t ret = ds3502_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS3502: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start at 0% power (safe state)
    ret = ds3502_set_wiper(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial power level");
        return ret;
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
        
        ds3502_deinit();
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
    
    // Calculate wiper position based on mode
    uint8_t wiper;
    if (s_power.mode == O3_POWER_MODE_ORIGINAL) {
        // Limit to original pot range (~60 steps for 4.7kΩ)
        wiper = (uint8_t)((percent / 100.0f) * ORIGINAL_POT_MAX_WIPER + 0.5f);
    } else {
        // Full range (0-127)
        wiper = (uint8_t)((percent / 100.0f) * DS3502_WIPER_MAX + 0.5f);
    }
    
    ESP_LOGI(TAG, "Setting power to %.1f%% (wiper=%u, mode=%s)",
             percent, wiper,
             s_power.mode == O3_POWER_MODE_ORIGINAL ? "original" : "extended");
    
    return ds3502_set_wiper(wiper);
}

float o3_power_get_percent(void)
{
    if (!s_power.initialized) {
        return 0;
    }
    
    uint8_t wiper;
    if (ds3502_get_wiper(&wiper) != ESP_OK) {
        return 0;
    }
    
    // Calculate percentage based on mode
    float max_wiper = (s_power.mode == O3_POWER_MODE_ORIGINAL) ? 
                       ORIGINAL_POT_MAX_WIPER : DS3502_WIPER_MAX;
    
    float percent = (wiper / max_wiper) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    return percent;
}

esp_err_t o3_power_set_wiper(uint8_t wiper)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Setting wiper to %u", wiper);
    return ds3502_set_wiper(wiper);
}

uint8_t o3_power_get_wiper(void)
{
    if (!s_power.initialized) {
        return 0;
    }
    
    uint8_t wiper;
    if (ds3502_get_wiper(&wiper) != ESP_OK) {
        return 0;
    }
    return wiper;
}

esp_err_t o3_power_set_resistance(uint16_t ohms)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Setting resistance to %u ohms", ohms);
    return ds3502_set_resistance(ohms);
}

uint16_t o3_power_get_resistance(void)
{
    if (!s_power.initialized) {
        return 0;
    }
    return ds3502_get_resistance();
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
        ds3502_set_wiper(0);
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
    
    ds3502_state_t ds_state;
    esp_err_t ret = ds3502_get_state(&ds_state);
    if (ret != ESP_OK) {
        return ret;
    }
    
    state->wiper_position = ds_state.wiper_position;
    state->resistance_ohms = ds_state.resistance_ohms;
    state->power_percent = o3_power_get_percent();
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
    
    ESP_LOGI(TAG, "Starting calibration: wiper %u→%u, step=%u, hold=%ums",
             start_wiper, end_wiper, step_size, (unsigned)hold_time_ms);
    
    s_power.calibration_active = true;
    s_power.calibration_stop_requested = false;
    
    // Clear previous calibration
    memset(&s_power.calibration, 0, sizeof(s_power.calibration));
    s_power.calibration.valid = false;
    
    // Calculate total steps for progress reporting
    int total_steps = (abs((int)end_wiper - (int)start_wiper) / step_size) + 1;
    int current_step = 0;
    
    // Direction
    int8_t direction = (end_wiper >= start_wiper) ? 1 : -1;
    
    // Sweep through positions
    for (uint8_t wiper = start_wiper; 
         (direction > 0) ? (wiper <= end_wiper) : (wiper >= end_wiper);
         wiper += direction * step_size) {
        
        // Check for stop request
        if (s_power.calibration_stop_requested) {
            ESP_LOGI(TAG, "Calibration stopped by request");
            break;
        }
        
        // Set wiper position
        esp_err_t ret = ds3502_set_wiper(wiper);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set wiper to %u", wiper);
            continue;
        }
        
        // Wait for hold time
        vTaskDelay(pdMS_TO_TICKS(hold_time_ms));
        
        // Call callback with progress
        float progress = (float)current_step / (float)total_steps;
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
        
        // Prevent underflow
        if (wiper == 0 && direction < 0) break;
        if (wiper == 127 && direction > 0) break;
    }
    
    // Return to safe state
    ds3502_set_wiper(0);
    
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
