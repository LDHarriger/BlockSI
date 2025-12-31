/**
 * @file o3_power_control.c
 * @brief Ozone generator power control implementation
 * 
 * Uses MCP4725 12-bit I2C DAC for precise power control.
 * The 12-bit resolution (4096 steps) provides much finer control
 * than the ESP32's internal 8-bit DAC (256 steps).
 */

#include "o3_power_control.h"
#include "mcp4725_dac.h"
#include "blocksi_pins.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "O3_POWER";

// Effective power range (from empirical measurements)
#define EFFECTIVE_MIN_PCT   O3_POWER_EFFECTIVE_MIN_PCT
#define EFFECTIVE_MAX_PCT   O3_POWER_EFFECTIVE_MAX_PCT

// NVS storage
#define NVS_NAMESPACE       "o3_power"
#define NVS_KEY_CAL         "calibration"

// Calibration parameters
#define CAL_STABILIZE_MS    10000   // 10 seconds to stabilize after power change
#define CAL_WARMUP_MS       300000  // 5 minutes warm-up
#define CAL_COARSE_STEP     5       // Coarse sweep step size (%)
#define CAL_FINE_STEP       1       // Fine sweep step size (%)
#define CAL_CHANGE_THRESHOLD 0.5f   // ppm change to detect active range

// External function to get O3 reading (defined in dosimetry.c)
extern float dosimetry_get_current_o3_ppm(void);

// Module state
static struct {
    bool initialized;
    uint8_t current_power_pct;
    uint16_t current_dac_value;     // 12-bit for MCP4725
    o3_power_calibration_t calibration;
    
    // Calibration state
    bool cal_running;
    bool cal_abort;
    o3_power_sweep_callback_t cal_callback;
    
    // Statistics accumulator
    float cal_sum;
    float cal_sum_sq;
    uint16_t cal_count;
} s_power = {0};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Convert power percentage to 12-bit DAC value
 */
static uint16_t power_to_dac(uint8_t power_pct)
{
    if (power_pct == 0) {
        return 0;
    }
    
    // Map 1-100% to effective range, then to voltage, then to DAC
    float effective_pct = EFFECTIVE_MIN_PCT + 
                          (power_pct / 100.0f) * (EFFECTIVE_MAX_PCT - EFFECTIVE_MIN_PCT);
    
    // Convert to percentage of full DAC range
    // Note: MCP4725 is 12-bit (0-4095)
    uint16_t dac_value = (uint16_t)((effective_pct / 100.0f) * 4095.0f);
    
    return dac_value;
}

/**
 * @brief Calculate expected output voltage for DAC value
 */
static float dac_to_voltage(uint16_t dac_value)
{
    float dac_voltage = (dac_value / 4095.0f) * DAC_VDD_VOLTAGE;
    return dac_voltage * DAC_DIVIDER_RATIO;
}

/**
 * @brief Reset statistics accumulator
 */
static void reset_stats(void)
{
    s_power.cal_sum = 0;
    s_power.cal_sum_sq = 0;
    s_power.cal_count = 0;
}

/**
 * @brief Add sample to statistics
 */
static void add_sample(float o3_ppm)
{
    s_power.cal_sum += o3_ppm;
    s_power.cal_sum_sq += o3_ppm * o3_ppm;
    s_power.cal_count++;
}

/**
 * @brief Get current statistics
 */
static void get_stats(float *mean, float *std)
{
    if (s_power.cal_count == 0) {
        *mean = 0;
        *std = 0;
        return;
    }
    
    *mean = s_power.cal_sum / s_power.cal_count;
    
    if (s_power.cal_count > 1) {
        float variance = (s_power.cal_sum_sq - 
                         (s_power.cal_sum * s_power.cal_sum) / s_power.cal_count) /
                         (s_power.cal_count - 1);
        *std = (variance > 0) ? sqrtf(variance) : 0;
    } else {
        *std = 0;
    }
}

// ============================================================================
// Public API - Initialization
// ============================================================================

esp_err_t o3_power_init(void)
{
    if (s_power.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing O3 power control");
    
    // Check if MCP4725 DAC is available
    if (!mcp4725_is_present()) {
        ESP_LOGE(TAG, "MCP4725 DAC not found - power control unavailable");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Set to 0% power initially
    esp_err_t ret = mcp4725_set_value(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial power level");
        return ret;
    }
    
    s_power.current_power_pct = 0;
    s_power.current_dac_value = 0;
    s_power.initialized = true;
    
    // Try to load saved calibration
    o3_power_load_calibration();
    
    ESP_LOGI(TAG, "O3 power control initialized");
    ESP_LOGI(TAG, "  Effective range: %d%% - %d%%", EFFECTIVE_MIN_PCT, EFFECTIVE_MAX_PCT);
    ESP_LOGI(TAG, "  Output voltage range: 0 - %.2fV", DAC_MAX_OUTPUT_V);
    
    return ESP_OK;
}

void o3_power_deinit(void)
{
    if (!s_power.initialized) {
        return;
    }
    
    // Set power to 0 before deinitializing
    o3_power_set(0);
    
    s_power.initialized = false;
    ESP_LOGI(TAG, "O3 power control deinitialized");
}

// ============================================================================
// Public API - Power Control
// ============================================================================

esp_err_t o3_power_set(uint8_t percent)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (percent > 100) {
        percent = 100;
    }
    
    uint16_t dac_value = power_to_dac(percent);
    
    esp_err_t ret = mcp4725_set_value(dac_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DAC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    s_power.current_power_pct = percent;
    s_power.current_dac_value = dac_value;
    
    ESP_LOGI(TAG, "Power set to %d%% (DAC=%d, V=%.3fV)", 
             percent, dac_value, dac_to_voltage(dac_value));
    
    return ESP_OK;
}

uint8_t o3_power_get(void)
{
    return s_power.current_power_pct;
}

esp_err_t o3_power_set_raw(uint8_t dac_value_8bit)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Scale 8-bit to 12-bit for compatibility
    uint16_t dac_value = (uint16_t)dac_value_8bit << 4;
    
    esp_err_t ret = mcp4725_set_value(dac_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    s_power.current_dac_value = dac_value;
    s_power.current_power_pct = (uint8_t)((dac_value / 4095.0f) * 100);
    
    ESP_LOGD(TAG, "Raw DAC set to %d (V=%.3fV)", dac_value, dac_to_voltage(dac_value));
    
    return ESP_OK;
}

uint8_t o3_power_get_raw(void)
{
    // Return as 8-bit for compatibility
    return (uint8_t)(s_power.current_dac_value >> 4);
}

float o3_power_get_voltage(void)
{
    return dac_to_voltage(s_power.current_dac_value);
}

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Collect samples at current power level
 */
static esp_err_t collect_samples(uint16_t num_samples, float *mean, float *std)
{
    reset_stats();
    
    ESP_LOGI(TAG, "Collecting %d samples...", num_samples);
    
    for (uint16_t i = 0; i < num_samples; i++) {
        if (s_power.cal_abort) {
            ESP_LOGW(TAG, "Sample collection aborted");
            return ESP_ERR_TIMEOUT;
        }
        
        float o3 = dosimetry_get_current_o3_ppm();
        add_sample(o3);
        
        ESP_LOGD(TAG, "  Sample %d: %.2f ppm", i + 1, o3);
        
        // Wait for next sample (assuming ~2 second sample rate)
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    
    get_stats(mean, std);
    ESP_LOGI(TAG, "Statistics: mean=%.2f, std=%.3f ppm", *mean, *std);
    return ESP_OK;
}

/**
 * @brief Store calibration point
 */
static void store_cal_point(uint8_t power_pct, float mean, float std, uint16_t count)
{
    if (s_power.calibration.point_count >= 21) {
        ESP_LOGW(TAG, "Calibration point buffer full");
        return;
    }
    
    o3_power_cal_point_t *p = &s_power.calibration.points[s_power.calibration.point_count];
    p->power_pct = power_pct;
    p->o3_mean_ppm = mean;
    p->o3_std_ppm = std;
    p->sample_count = count;
    
    s_power.calibration.point_count++;
    
    ESP_LOGI(TAG, "Cal point #%d: %d%% → %.1f ± %.2f ppm (%d samples)", 
             s_power.calibration.point_count, power_pct, mean, std, count);
}

/**
 * @brief Check if change is statistically significant
 */
static bool is_significant_change(float mean1, float std1, float mean2, float std2)
{
    // Change is significant if |mean2 - mean1| > 2 * sqrt(std1² + std2²)
    float combined_std = sqrtf(std1 * std1 + std2 * std2);
    float threshold = 2.0f * combined_std;
    if (threshold < CAL_CHANGE_THRESHOLD) {
        threshold = CAL_CHANGE_THRESHOLD;
    }
    
    return fabsf(mean2 - mean1) > threshold;
}

esp_err_t o3_power_calibrate(float flow_rate_lpm, uint16_t samples_per_point,
                              o3_power_sweep_callback_t callback)
{
    if (!s_power.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_power.cal_running) {
        ESP_LOGW(TAG, "Calibration already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting Power Calibration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Flow rate: %.1f LPM", flow_rate_lpm);
    ESP_LOGI(TAG, "Samples per point: %d", samples_per_point);
    
    s_power.cal_running = true;
    s_power.cal_abort = false;
    s_power.cal_callback = callback;
    
    // Clear previous calibration
    memset(&s_power.calibration, 0, sizeof(s_power.calibration));
    s_power.calibration.flow_rate_lpm = flow_rate_lpm;
    
    esp_err_t ret = ESP_OK;
    float mean, std;
    float prev_mean = 0, prev_std = 0;
    uint8_t active_start = 0;
    uint8_t active_end = 100;
    bool found_start = false;
    bool found_end = false;
    uint8_t plateau_count = 0;
    
    // -------------------------------------------------------------------------
    // Phase 0: Warm-up (optional but recommended)
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 0: Warming up generator at 50%% power...");
    o3_power_set(50);
    
    for (int i = 0; i < 30 && !s_power.cal_abort; i++) {  // 5 minutes
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "  Warm-up: %d/30", i + 1);
        if (callback) {
            callback(50, 0, 0, i / 300.0f);  // 0-10% progress
        }
    }
    
    if (s_power.cal_abort) goto cleanup;
    
    // -------------------------------------------------------------------------
    // Phase 1: Establish baseline at 0%
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 1: Establishing baseline at 0%% power...");
    o3_power_set(0);
    vTaskDelay(pdMS_TO_TICKS(CAL_STABILIZE_MS));
    
    ret = collect_samples(samples_per_point, &mean, &std);
    if (ret != ESP_OK) goto cleanup;
    
    store_cal_point(0, mean, std, samples_per_point);
    prev_mean = mean;
    prev_std = std;
    
    if (callback) {
        callback(0, mean, std, 0.15f);
    }
    
    // -------------------------------------------------------------------------
    // Phase 2: Coarse sweep to find active range
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 2: Coarse sweep (0%% to 100%% in %d%% steps)...", CAL_COARSE_STEP);
    
    for (uint8_t pct = CAL_COARSE_STEP; pct <= 100 && !s_power.cal_abort; pct += CAL_COARSE_STEP) {
        ESP_LOGI(TAG, "Testing %d%%...", pct);
        o3_power_set(pct);
        vTaskDelay(pdMS_TO_TICKS(CAL_STABILIZE_MS));
        
        ret = collect_samples(samples_per_point, &mean, &std);
        if (ret != ESP_OK) goto cleanup;
        
        store_cal_point(pct, mean, std, samples_per_point);
        
        // Detect start of active range
        if (!found_start && is_significant_change(prev_mean, prev_std, mean, std)) {
            active_start = pct - CAL_COARSE_STEP;
            found_start = true;
            plateau_count = 0;
            ESP_LOGI(TAG, ">>> Active range starts at ~%d%%", active_start);
        }
        
        // Detect end of active range (plateau)
        if (found_start && !found_end) {
            if (!is_significant_change(prev_mean, prev_std, mean, std)) {
                plateau_count++;
                ESP_LOGD(TAG, "Plateau count: %d", plateau_count);
                if (plateau_count >= 3) {
                    active_end = pct;
                    found_end = true;
                    ESP_LOGI(TAG, ">>> Active range ends at ~%d%%", active_end);
                }
            } else {
                plateau_count = 0;
            }
        }
        
        prev_mean = mean;
        prev_std = std;
        
        if (callback) {
            float progress = 0.15f + 0.5f * (pct / 100.0f);
            callback(pct, mean, std, progress);
        }
    }
    
    // -------------------------------------------------------------------------
    // Phase 3: Fine sweep in active range
    // -------------------------------------------------------------------------
    if (found_start && !s_power.cal_abort) {
        ESP_LOGI(TAG, "Phase 3: Fine sweep (%d%% to %d%% in %d%% steps)...", 
                 active_start, active_end, CAL_FINE_STEP);
        
        for (uint8_t pct = active_start + 1; 
             pct < active_end && !s_power.cal_abort; 
             pct += CAL_FINE_STEP) {
            
            // Skip if already covered in coarse sweep
            if ((pct % CAL_COARSE_STEP) == 0) continue;
            
            ESP_LOGI(TAG, "Fine testing %d%%...", pct);
            o3_power_set(pct);
            vTaskDelay(pdMS_TO_TICKS(CAL_STABILIZE_MS));
            
            ret = collect_samples(samples_per_point, &mean, &std);
            if (ret != ESP_OK) goto cleanup;
            
            store_cal_point(pct, mean, std, samples_per_point);
            
            if (callback) {
                float progress = 0.65f + 0.3f * ((pct - active_start) / 
                                                  (float)(active_end - active_start));
                callback(pct, mean, std, progress);
            }
        }
    }
    
    // -------------------------------------------------------------------------
    // Phase 4: Finalize
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 4: Finalizing calibration...");
    
    // Sort calibration points by power percentage
    for (int i = 0; i < s_power.calibration.point_count - 1; i++) {
        for (int j = 0; j < s_power.calibration.point_count - i - 1; j++) {
            if (s_power.calibration.points[j].power_pct > 
                s_power.calibration.points[j+1].power_pct) {
                o3_power_cal_point_t temp = s_power.calibration.points[j];
                s_power.calibration.points[j] = s_power.calibration.points[j+1];
                s_power.calibration.points[j+1] = temp;
            }
        }
    }
    
    s_power.calibration.timestamp = 0;  // TODO: Add timestamp
    s_power.calibration.valid = true;
    
    // Return to 0% power
    o3_power_set(0);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Calibration Complete");
    ESP_LOGI(TAG, "  Points collected: %d", s_power.calibration.point_count);
    ESP_LOGI(TAG, "  Active range: %d%% - %d%%", active_start, active_end);
    ESP_LOGI(TAG, "========================================");
    
    if (callback) {
        callback(0, 0, 0, 1.0f);
    }
    
    // Save to NVS
    o3_power_save_calibration();
    
cleanup:
    s_power.cal_running = false;
    s_power.cal_callback = NULL;
    
    if (s_power.cal_abort) {
        ESP_LOGW(TAG, "Calibration aborted");
        o3_power_set(0);
        return ESP_ERR_TIMEOUT;
    }
    
    return ret;
}

void o3_power_calibrate_abort(void)
{
    if (s_power.cal_running) {
        s_power.cal_abort = true;
        ESP_LOGW(TAG, "Calibration abort requested");
    }
}

bool o3_power_calibrate_is_running(void)
{
    return s_power.cal_running;
}

esp_err_t o3_power_get_calibration(o3_power_calibration_t *cal)
{
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_power.calibration.valid) {
        return ESP_ERR_NOT_FOUND;
    }
    
    *cal = s_power.calibration;
    return ESP_OK;
}

esp_err_t o3_power_save_calibration(void)
{
    if (!s_power.calibration.valid) {
        return ESP_ERR_INVALID_STATE;
    }
    
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_blob(nvs, NVS_KEY_CAL, &s_power.calibration, sizeof(s_power.calibration));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    
    nvs_close(nvs);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS (%d points)", 
                 s_power.calibration.point_count);
    } else {
        ESP_LOGE(TAG, "Failed to save calibration: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t o3_power_load_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No saved calibration found");
        return ret;
    }
    
    size_t size = sizeof(s_power.calibration);
    ret = nvs_get_blob(nvs, NVS_KEY_CAL, &s_power.calibration, &size);
    
    nvs_close(nvs);
    
    if (ret == ESP_OK && s_power.calibration.valid) {
        ESP_LOGI(TAG, "Loaded calibration from NVS (%d points, flow=%.1f LPM)",
                 s_power.calibration.point_count, s_power.calibration.flow_rate_lpm);
    } else {
        s_power.calibration.valid = false;
        ESP_LOGD(TAG, "No valid calibration in NVS");
    }
    
    return ret;
}

esp_err_t o3_power_clear_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = nvs_erase_key(nvs, NVS_KEY_CAL);
    if (ret == ESP_OK || ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_commit(nvs);
        memset(&s_power.calibration, 0, sizeof(s_power.calibration));
        ESP_LOGI(TAG, "Calibration cleared");
        ret = ESP_OK;
    }
    
    nvs_close(nvs);
    return ret;
}

// ============================================================================
// Estimation
// ============================================================================

float o3_power_estimate_output(uint8_t power_pct)
{
    if (!s_power.calibration.valid || s_power.calibration.point_count < 2) {
        return -1.0f;
    }
    
    // Find surrounding calibration points and interpolate
    o3_power_cal_point_t *lower = NULL;
    o3_power_cal_point_t *upper = NULL;
    
    for (int i = 0; i < s_power.calibration.point_count; i++) {
        o3_power_cal_point_t *p = &s_power.calibration.points[i];
        
        if (p->power_pct <= power_pct) {
            lower = p;
        }
        if (p->power_pct >= power_pct && upper == NULL) {
            upper = p;
        }
    }
    
    if (lower == NULL) {
        return s_power.calibration.points[0].o3_mean_ppm;
    }
    if (upper == NULL) {
        return s_power.calibration.points[s_power.calibration.point_count - 1].o3_mean_ppm;
    }
    if (lower == upper) {
        return lower->o3_mean_ppm;
    }
    
    // Linear interpolation
    float t = (power_pct - lower->power_pct) / 
              (float)(upper->power_pct - lower->power_pct);
    return lower->o3_mean_ppm + t * (upper->o3_mean_ppm - lower->o3_mean_ppm);
}

uint8_t o3_power_find_for_target(float target_ppm)
{
    if (!s_power.calibration.valid || s_power.calibration.point_count < 2) {
        return 0xFF;
    }
    
    // Find calibration points that bracket the target
    for (int i = 0; i < s_power.calibration.point_count - 1; i++) {
        o3_power_cal_point_t *p1 = &s_power.calibration.points[i];
        o3_power_cal_point_t *p2 = &s_power.calibration.points[i + 1];
        
        if (p1->o3_mean_ppm <= target_ppm && p2->o3_mean_ppm >= target_ppm) {
            // Linear interpolation
            float t = (target_ppm - p1->o3_mean_ppm) / 
                      (p2->o3_mean_ppm - p1->o3_mean_ppm);
            return (uint8_t)(p1->power_pct + t * (p2->power_pct - p1->power_pct));
        }
    }
    
    // Target not achievable
    return 0xFF;
}
