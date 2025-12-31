/**
 * @file dosimetry.c
 * @brief Ozone dosimetry calculations implementation
 */

#include "dosimetry.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "DOSIMETRY";

// Physical constants
#define MOLAR_MASS_O3       48.0f       // g/mol
#define MOLAR_VOLUME_STP    24.5f       // L/mol at 25°C, 1 atm
#define GAS_CONSTANT_R      8.314f      // J/(mol·K)

// Default parameters
#define DEFAULT_FLOW_LPM        5.0f
#define DEFAULT_VESSEL_VOL_L    2.0f
#define DEFAULT_MATERIAL_VOL_L  1.0f
#define DEFAULT_TEMP_C          25.0f
#define DEFAULT_HUMIDITY_PCT    60.0f
#define DEFAULT_SENSOR_PATH_L   0.1f

// Decay model parameters (empirical - adjust based on literature/experiments)
#define DECAY_K0            0.0003f     // Base decay rate at 25°C, dry (1/s)
#define DECAY_HUMIDITY_ALPHA 0.02f      // Humidity factor
#define DECAY_EA_OVER_R     2500.0f     // Activation energy / R (K)
#define DECAY_T_REF_K       298.15f     // Reference temperature (25°C in K)

// Conversion factor: ppm to mg/s at 1 LPM
// = (48 g/mol) / (24.5 L/mol) × (1 L/min) × (1 min/60 s) × (1 mg/1000 g) × (1/1e6 for ppm)
// = 48 / 24.5 / 60 / 1000 / 1e6 × 1e6 = 48 / 24.5 / 60 / 1000
// = 3.27e-5 mg/s per ppm per LPM
#define PPM_TO_MGS_FACTOR   3.27e-5f

// Module state
static struct {
    bool initialized;
    dosimetry_params_t params;
    dosimetry_sample_t last_sample;
    dosimetry_cycle_t cycle;
    bool has_sample;
} s_dosi = {0};

// ============================================================================
// Initialization
// ============================================================================

esp_err_t dosimetry_init(const dosimetry_params_t *params)
{
    ESP_LOGI(TAG, "Initializing dosimetry module");
    
    // Set default parameters
    s_dosi.params.flow_rate_lpm = DEFAULT_FLOW_LPM;
    s_dosi.params.vessel_volume_L = DEFAULT_VESSEL_VOL_L;
    s_dosi.params.material_volume_L = DEFAULT_MATERIAL_VOL_L;
    s_dosi.params.temperature_C = DEFAULT_TEMP_C;
    s_dosi.params.humidity_pct = DEFAULT_HUMIDITY_PCT;
    s_dosi.params.sensor_path_volume_L = DEFAULT_SENSOR_PATH_L;
    
    // Override with provided parameters
    if (params) {
        if (params->flow_rate_lpm > 0) {
            s_dosi.params.flow_rate_lpm = params->flow_rate_lpm;
        }
        if (params->vessel_volume_L > 0) {
            s_dosi.params.vessel_volume_L = params->vessel_volume_L;
        }
        if (params->material_volume_L >= 0) {
            s_dosi.params.material_volume_L = params->material_volume_L;
        }
        s_dosi.params.temperature_C = params->temperature_C;
        if (params->humidity_pct >= 0 && params->humidity_pct <= 100) {
            s_dosi.params.humidity_pct = params->humidity_pct;
        }
        if (params->sensor_path_volume_L > 0) {
            s_dosi.params.sensor_path_volume_L = params->sensor_path_volume_L;
        }
    }
    
    ESP_LOGI(TAG, "Flow rate: %.1f LPM", s_dosi.params.flow_rate_lpm);
    ESP_LOGI(TAG, "Vessel volume: %.2f L", s_dosi.params.vessel_volume_L);
    ESP_LOGI(TAG, "Material volume: %.2f L", s_dosi.params.material_volume_L);
    ESP_LOGI(TAG, "Temperature: %.1f °C", s_dosi.params.temperature_C);
    ESP_LOGI(TAG, "Humidity: %.0f %%", s_dosi.params.humidity_pct);
    
    memset(&s_dosi.last_sample, 0, sizeof(s_dosi.last_sample));
    memset(&s_dosi.cycle, 0, sizeof(s_dosi.cycle));
    s_dosi.has_sample = false;
    s_dosi.initialized = true;
    
    return ESP_OK;
}

esp_err_t dosimetry_set_params(const dosimetry_params_t *params)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_dosi.params = *params;
    ESP_LOGI(TAG, "Parameters updated - Flow: %.1f LPM, Vessel: %.2f L", 
             params->flow_rate_lpm, params->vessel_volume_L);
    
    return ESP_OK;
}

esp_err_t dosimetry_get_params(dosimetry_params_t *params)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *params = s_dosi.params;
    return ESP_OK;
}

// ============================================================================
// Flow Rate Control
// ============================================================================

esp_err_t dosimetry_set_flow_rate(float flow_lpm)
{
    if (flow_lpm <= 0 || flow_lpm > 20.0f) {
        ESP_LOGW(TAG, "Invalid flow rate: %.2f LPM", flow_lpm);
        return ESP_ERR_INVALID_ARG;
    }
    
    s_dosi.params.flow_rate_lpm = flow_lpm;
    ESP_LOGI(TAG, "Flow rate set to %.1f LPM", flow_lpm);
    
    return ESP_OK;
}

float dosimetry_get_flow_rate(void)
{
    return s_dosi.params.flow_rate_lpm;
}

// ============================================================================
// Conversion Utilities
// ============================================================================

float dosimetry_ppm_to_mg_s(float o3_ppm, float flow_lpm)
{
    // ṁ [mg/s] = C [ppm] × F [L/min] × conversion_factor
    return o3_ppm * flow_lpm * PPM_TO_MGS_FACTOR;
}

float dosimetry_wtpct_to_ppm(float wt_pct)
{
    // wt% to ppm: 1 wt% = 10,000 ppm
    return wt_pct * 10000.0f;
}

float dosimetry_ppm_to_wtpct(float ppm)
{
    return ppm / 10000.0f;
}

// ============================================================================
// Decay Model
// ============================================================================

float dosimetry_estimate_decay_rate(float temperature_C, float humidity_pct)
{
    float T_K = temperature_C + 273.15f;
    
    // Arrhenius with humidity correction
    // k = k0 × (1 + α×RH) × exp(-Ea/R × (1/T - 1/T_ref))
    float humidity_factor = 1.0f + DECAY_HUMIDITY_ALPHA * (humidity_pct / 100.0f);
    float temp_factor = expf(-DECAY_EA_OVER_R * (1.0f/T_K - 1.0f/DECAY_T_REF_K));
    
    return DECAY_K0 * humidity_factor * temp_factor;
}

float dosimetry_get_residence_time(void)
{
    // τ = V / F, convert F from L/min to L/s
    float F_Ls = s_dosi.params.flow_rate_lpm / 60.0f;
    float V = s_dosi.params.vessel_volume_L + s_dosi.params.sensor_path_volume_L;
    
    return V / F_Ls;
}

float dosimetry_estimate_decay_loss(float o3_input_mg)
{
    float k = dosimetry_estimate_decay_rate(s_dosi.params.temperature_C, 
                                             s_dosi.params.humidity_pct);
    float tau = dosimetry_get_residence_time();
    
    // First-order decay: m_out = m_in × exp(-k×τ)
    // Loss = m_in × (1 - exp(-k×τ))
    return o3_input_mg * (1.0f - expf(-k * tau));
}

// ============================================================================
// Sample Processing
// ============================================================================

esp_err_t dosimetry_process_sample(float o3_ppm, float temperature_C, 
                                    float pressure_mbar, uint32_t sample_interval_ms,
                                    dosimetry_sample_t *out_sample)
{
    if (!s_dosi.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Calculate mass flow rate
    float o3_mg_s = dosimetry_ppm_to_mg_s(o3_ppm, s_dosi.params.flow_rate_lpm);
    
    // Calculate mass evacuated during this sample window
    float dt_s = sample_interval_ms / 1000.0f;
    float o3_mg_sample = o3_mg_s * dt_s;
    
    // Pressure correction (optional refinement)
    // Actual molar volume = V_m × (P_std / P_actual) × (T_actual / T_std)
    // For now, we ignore this as 106-H already compensates internally
    
    // Store sample
    s_dosi.last_sample.o3_ppm = o3_ppm;
    s_dosi.last_sample.temperature_C = temperature_C;
    s_dosi.last_sample.pressure_mbar = pressure_mbar;
    s_dosi.last_sample.o3_mg_s = o3_mg_s;
    s_dosi.last_sample.o3_mg_sample = o3_mg_sample;
    s_dosi.last_sample.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_dosi.has_sample = true;
    
    // Update cycle if active
    if (s_dosi.cycle.active) {
        s_dosi.cycle.total_o3_measured_mg += o3_mg_sample;
        s_dosi.cycle.sample_count++;
        s_dosi.cycle.duration_ms = s_dosi.last_sample.timestamp_ms - s_dosi.cycle.start_time_ms;
    }
    
    // Output sample if requested
    if (out_sample) {
        *out_sample = s_dosi.last_sample;
    }
    
    ESP_LOGD(TAG, "Sample: %.1f ppm → %.4f mg/s, %.4f mg", 
             o3_ppm, o3_mg_s, o3_mg_sample);
    
    return ESP_OK;
}

esp_err_t dosimetry_get_last_sample(dosimetry_sample_t *sample)
{
    if (!sample) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_dosi.has_sample) {
        return ESP_ERR_NOT_FOUND;
    }
    
    *sample = s_dosi.last_sample;
    return ESP_OK;
}

float dosimetry_get_current_o3_ppm(void)
{
    if (!s_dosi.has_sample) {
        return 0.0f;
    }
    
    // Check if sample is recent (within 30 seconds)
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms - s_dosi.last_sample.timestamp_ms > 30000) {
        return 0.0f;  // Stale data
    }
    
    return s_dosi.last_sample.o3_ppm;
}

// ============================================================================
// Treatment Cycle Tracking
// ============================================================================

esp_err_t dosimetry_cycle_start(void)
{
    if (!s_dosi.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_dosi.cycle.active) {
        ESP_LOGW(TAG, "Cycle already active, stopping previous");
        dosimetry_cycle_stop(NULL);
    }
    
    memset(&s_dosi.cycle, 0, sizeof(s_dosi.cycle));
    s_dosi.cycle.start_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_dosi.cycle.active = true;
    
    ESP_LOGI(TAG, "Treatment cycle started");
    
    return ESP_OK;
}

esp_err_t dosimetry_cycle_stop(dosimetry_cycle_t *cycle)
{
    if (!s_dosi.cycle.active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_dosi.cycle.active = false;
    s_dosi.cycle.duration_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_dosi.cycle.start_time_ms;
    
    // Calculate estimated decay loss
    float avg_rate = s_dosi.cycle.total_o3_measured_mg / (s_dosi.cycle.duration_ms / 1000.0f);
    float k = dosimetry_estimate_decay_rate(s_dosi.params.temperature_C, 
                                             s_dosi.params.humidity_pct);
    float tau = dosimetry_get_residence_time();
    s_dosi.cycle.total_o3_decayed_mg = s_dosi.cycle.total_o3_measured_mg * (1.0f - expf(-k * tau));
    
    // Note: total_o3_delivered and total_o3_absorbed require generator-side measurements
    // For now, these remain at 0 and should be calculated separately when that data is available
    
    ESP_LOGI(TAG, "Treatment cycle stopped");
    ESP_LOGI(TAG, "  Duration: %.1f s", s_dosi.cycle.duration_ms / 1000.0f);
    ESP_LOGI(TAG, "  Samples: %lu", (unsigned long)s_dosi.cycle.sample_count);
    ESP_LOGI(TAG, "  Total O3 measured: %.2f mg", s_dosi.cycle.total_o3_measured_mg);
    ESP_LOGI(TAG, "  Estimated decay: %.2f mg", s_dosi.cycle.total_o3_decayed_mg);
    
    if (cycle) {
        *cycle = s_dosi.cycle;
    }
    
    return ESP_OK;
}

esp_err_t dosimetry_cycle_get(dosimetry_cycle_t *cycle)
{
    if (!cycle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_dosi.cycle.active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    *cycle = s_dosi.cycle;
    // Update duration to current time
    cycle->duration_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_dosi.cycle.start_time_ms;
    
    return ESP_OK;
}

bool dosimetry_cycle_is_active(void)
{
    return s_dosi.cycle.active;
}
