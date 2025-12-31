/**
 * @file dosimetry.h
 * @brief Ozone dosimetry calculations
 * 
 * Converts O3 concentration measurements to mass flow rates and tracks
 * total ozone delivered, absorbed, and decayed over a treatment cycle.
 * 
 * Key calculations:
 * - Mass flow rate: ṁ = C × F × (M_O3 / V_m) where M_O3=48g/mol, V_m=24.5L/mol
 * - O3 decay: modeled using Arrhenius-type relationship with humidity factor
 * - CSTR model: for fill/evacuation transient corrections
 */

#ifndef DOSIMETRY_H
#define DOSIMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Process parameters
 */
typedef struct {
    float flow_rate_lpm;        // O2/O3 flow rate in L/min (default: 5.0)
    float vessel_volume_L;      // Total vessel + plumbing volume in L
    float material_volume_L;    // Volume of material being treated in L
    float temperature_C;        // Process temperature in °C (default: 25)
    float humidity_pct;         // Estimated RH in vessel (default: 60)
    float sensor_path_volume_L; // Volume from vessel to 106-H in L
} dosimetry_params_t;

/**
 * @brief Single sample data with mass calculation
 */
typedef struct {
    // Raw sensor data
    float o3_ppm;               // O3 concentration in ppm
    float temperature_C;        // Cell temperature
    float pressure_mbar;        // Cell pressure
    
    // Calculated values
    float o3_mg_s;              // Mass flow rate in mg/s
    float o3_mg_sample;         // Mass evacuated during this sample window in mg
    
    // Timestamp
    uint32_t timestamp_ms;      // Milliseconds since boot
} dosimetry_sample_t;

/**
 * @brief Accumulated dosimetry for a treatment cycle
 */
typedef struct {
    // Totals
    float total_o3_delivered_mg;    // Total O3 mass from generator
    float total_o3_measured_mg;     // Total O3 mass measured at outlet
    float total_o3_absorbed_mg;     // Estimated absorbed by material
    float total_o3_decayed_mg;      // Estimated natural decay
    
    // Timing
    uint32_t start_time_ms;
    uint32_t duration_ms;
    uint32_t sample_count;
    
    // State
    bool active;
} dosimetry_cycle_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize dosimetry module
 * 
 * @param params Process parameters (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t dosimetry_init(const dosimetry_params_t *params);

/**
 * @brief Update process parameters
 * 
 * @param params New parameters
 * @return ESP_OK on success
 */
esp_err_t dosimetry_set_params(const dosimetry_params_t *params);

/**
 * @brief Get current process parameters
 * 
 * @param params Output parameter structure
 * @return ESP_OK on success
 */
esp_err_t dosimetry_get_params(dosimetry_params_t *params);

// ============================================================================
// Flow Rate Control
// ============================================================================

/**
 * @brief Set O2/O3 flow rate
 * 
 * This is a manual setting - must match physical flow meter reading.
 * 
 * @param flow_lpm Flow rate in liters per minute
 * @return ESP_OK on success
 */
esp_err_t dosimetry_set_flow_rate(float flow_lpm);

/**
 * @brief Get current flow rate setting
 * 
 * @return Flow rate in LPM
 */
float dosimetry_get_flow_rate(void);

// ============================================================================
// Sample Processing
// ============================================================================

/**
 * @brief Process a new 106-H sample
 * 
 * Calculates mass flow rate and per-sample mass from concentration.
 * If a cycle is active, updates cycle totals.
 * 
 * @param o3_ppm O3 concentration in ppm (wt% × 10000)
 * @param temperature_C Cell temperature
 * @param pressure_mbar Cell pressure
 * @param sample_interval_ms Time since last sample in ms
 * @param out_sample Output sample with calculated values (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t dosimetry_process_sample(float o3_ppm, float temperature_C, 
                                    float pressure_mbar, uint32_t sample_interval_ms,
                                    dosimetry_sample_t *out_sample);

/**
 * @brief Get the most recent sample
 * 
 * @param sample Output sample structure
 * @return ESP_OK if sample available
 */
esp_err_t dosimetry_get_last_sample(dosimetry_sample_t *sample);

/**
 * @brief Get current O3 concentration in ppm
 * 
 * Convenience function for power calibration module.
 * 
 * @return O3 concentration in ppm, or 0 if no recent sample
 */
float dosimetry_get_current_o3_ppm(void);

// ============================================================================
// Treatment Cycle Tracking
// ============================================================================

/**
 * @brief Start a new treatment cycle
 * 
 * Resets accumulators and begins tracking O3 dosimetry.
 * 
 * @return ESP_OK on success
 */
esp_err_t dosimetry_cycle_start(void);

/**
 * @brief Stop current treatment cycle
 * 
 * Finalizes calculations and returns summary.
 * 
 * @param cycle Output cycle summary (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t dosimetry_cycle_stop(dosimetry_cycle_t *cycle);

/**
 * @brief Get current cycle status
 * 
 * @param cycle Output cycle data
 * @return ESP_OK if cycle is active
 */
esp_err_t dosimetry_cycle_get(dosimetry_cycle_t *cycle);

/**
 * @brief Check if a cycle is active
 * 
 * @return true if cycle is running
 */
bool dosimetry_cycle_is_active(void);

// ============================================================================
// Decay Model
// ============================================================================

/**
 * @brief Estimate O3 decay rate constant
 * 
 * Based on Arrhenius model with humidity correction:
 * k = k0 × (1 + α×RH) × exp(-Ea/R × (1/T - 1/T_ref))
 * 
 * @param temperature_C Temperature in Celsius
 * @param humidity_pct Relative humidity percentage
 * @return Decay rate constant in 1/s
 */
float dosimetry_estimate_decay_rate(float temperature_C, float humidity_pct);

/**
 * @brief Calculate residence time in vessel
 * 
 * τ = V / F
 * 
 * @return Residence time in seconds
 */
float dosimetry_get_residence_time(void);

/**
 * @brief Estimate O3 mass lost to decay during residence
 * 
 * @param o3_input_mg Input O3 mass
 * @return Estimated mass lost to decay
 */
float dosimetry_estimate_decay_loss(float o3_input_mg);

// ============================================================================
// Conversion Utilities
// ============================================================================

/**
 * @brief Convert O3 concentration to mass flow rate
 * 
 * ṁ [mg/s] = C [ppm] × F [L/min] × (48 / 24.5) × (1/60) × (1/1000)
 *          = C × F × 3.27e-5
 * 
 * @param o3_ppm O3 concentration in ppm
 * @param flow_lpm Flow rate in L/min
 * @return Mass flow rate in mg/s
 */
float dosimetry_ppm_to_mg_s(float o3_ppm, float flow_lpm);

/**
 * @brief Convert wt% to ppm
 * 
 * @param wt_pct Concentration in wt%
 * @return Concentration in ppm
 */
float dosimetry_wtpct_to_ppm(float wt_pct);

/**
 * @brief Convert ppm to wt%
 * 
 * @param ppm Concentration in ppm
 * @return Concentration in wt%
 */
float dosimetry_ppm_to_wtpct(float ppm);

#ifdef __cplusplus
}
#endif

#endif // DOSIMETRY_H
