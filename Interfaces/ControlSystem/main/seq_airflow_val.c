/**
 * @file seq_airflow_val.c
 * @brief Airflow/concentration validation sequence implementation
 * 
 * Interactive pre-flight check: verifies ozone concentration at a given power
 * level by measuring on the direct-to-sensor route (bypassing the vessel).
 * 
 * Phases:
 *   1. prompt_vessel  — Operator routes air to vessel, confirms flow
 *   2. prompt_direct  — Operator routes air direct to 106-H, matches flow
 *   3. stabilize      — Power on, wait for 106-H reading to settle
 *   4. measure        — Record O3 for measurement window, compute stats
 *   5. complete       — Report expected vs. actual concentration
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "seq_airflow_val.h"
#include "sequence_runner.h"
#include "motor_pot.h"
#include "o3_power_control.h"
#include "relay_control.h"
#include "blocksi_state.h"
#include "lan_client.h"

static const char *TAG = "SEQ_VAL";

// =============================================================================
// Configuration
// =============================================================================

#define VAL_STABILIZE_TIME_MS       60000   // 60s stabilization after power set
#define VAL_MEASURE_TIME_MS         60000   // 60s measurement window
#define VAL_SAMPLE_INTERVAL_MS      2000    // Sample every 2s
#define VAL_MAX_SAMPLES             64      // Max measurement samples
#define VAL_PROMPT_TIMEOUT_MS       300000  // 5 minute timeout for operator prompts
#define VAL_MOTOR_SETTLE_MS         500     // Motor settle time

// Phase names
static const char *PHASE_PROMPT_VESSEL  = "prompt_vessel";
static const char *PHASE_PROMPT_DIRECT  = "prompt_direct";
static const char *PHASE_STABILIZE      = "stabilize";
static const char *PHASE_MEASURE        = "measure";
static const char *PHASE_COMPLETE       = "complete";

// Progress weight allocation (approximate time ratios)
// Prompts are variable time so we allocate small fixed amounts
#define WEIGHT_PROMPT_VESSEL    2
#define WEIGHT_PROMPT_DIRECT    2
#define WEIGHT_STABILIZE        40
#define WEIGHT_MEASURE          56
#define WEIGHT_TOTAL            (WEIGHT_PROMPT_VESSEL + WEIGHT_PROMPT_DIRECT + \
                                 WEIGHT_STABILIZE + WEIGHT_MEASURE)

// =============================================================================
// Model constants (must match dashboard — see interface_contract.md)
// =============================================================================

#define POWER_MODEL_A           1.78f
#define POWER_MODEL_B           1.40f

// =============================================================================
// Internal State
// =============================================================================

typedef struct {
    // Parameters
    uint8_t power_pct;              // Target power level
    float o2_lpm;                   // O2 flow rate
    float confirmed_vessel_lpm;     // Operator-confirmed vessel route LPM
    float confirmed_direct_lpm;     // Operator-confirmed direct route LPM

    // Stop flag
    volatile bool stop_requested;

    // Measurement storage
    float samples_o3[VAL_MAX_SAMPLES];
    float samples_temp[VAL_MAX_SAMPLES];
    uint16_t sample_count;

    // Timing
    int64_t start_time_us;
} val_state_t;

static val_state_t s_val = {0};

// =============================================================================
// Helpers
// =============================================================================

static int64_t get_time_us(void)
{
    return esp_timer_get_time();
}

static float get_elapsed_s(void)
{
    return (float)(get_time_us() - s_val.start_time_us) / 1000000.0f;
}

/**
 * @brief Get current 106-H reading from blocksi_state
 */
static bool get_106h_sample(float *o3_pct, float *temp_c)
{
    const blocksi_state_t *state = blocksi_state_get();
    if (state == NULL || !state->sensors.vessel_o3_valid) {
        return false;
    }
    *o3_pct = state->sensors.vessel_o3_pct;
    *temp_c = state->sensors.cell_temp_c;
    return true;
}

/**
 * @brief Get motor pot actual position
 */
static float get_actual_pct(void)
{
    motor_pot_state_t state;
    motor_pot_get_state(&state);
    return state.position_percent;
}

/**
 * @brief Predict expected O3 concentration from power model
 * 
 * Uses O3_max = A/F + B where F is flow rate in LPM.
 * Predicted O3 at a given power: O3_pct = (power/100) * O3_max
 */
static float predict_o3_pct(uint8_t power_pct, float flow_lpm)
{
    if (flow_lpm < 0.1f) flow_lpm = 0.1f;
    float o3_max = POWER_MODEL_A / flow_lpm + POWER_MODEL_B;
    return ((float)power_pct / 100.0f) * o3_max;
}

/**
 * @brief Calculate overall progress 
 */
static float calc_progress(int phase_index, float phase_fraction)
{
    float weights[] = {WEIGHT_PROMPT_VESSEL, WEIGHT_PROMPT_DIRECT, 
                       WEIGHT_STABILIZE, WEIGHT_MEASURE};
    float base = 0;
    for (int i = 0; i < phase_index && i < 4; i++) {
        base += weights[i];
    }
    float result = (base + weights[phase_index] * phase_fraction) / WEIGHT_TOTAL * 100.0f;
    if (result > 100.0f) result = 100.0f;
    return result;
}

/**
 * @brief Send one VAL_DATA measurement point over LAN
 */
static void send_val_point(uint8_t power_pct, float actual_pct,
                           float o3_pct, float cell_temp_c)
{
    int64_t timestamp_ms = blocksi_state_get_time_ms();

    char msg[160];
    snprintf(msg, sizeof(msg),
             "VAL_DATA,%lld,%u,%.1f,%.4f,%.1f,%.1f",
             (long long)timestamp_ms,
             power_pct,
             actual_pct,
             o3_pct,
             s_val.o2_lpm,
             cell_temp_c);

    seq_send_data(msg);

    ESP_LOGI(TAG, "VAL: P=%u%% A=%.1f%% O3=%.4f%% T=%.1f",
             power_pct, actual_pct, o3_pct, cell_temp_c);
}

/**
 * @brief Compute mean of float array
 */
static float compute_mean(const float *data, uint16_t count)
{
    if (count == 0) return NAN;
    double sum = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (!isnan(data[i])) sum += data[i];
    }
    return (float)(sum / count);
}

/**
 * @brief Compute standard deviation of float array
 */
static float compute_std(const float *data, uint16_t count, float mean)
{
    if (count < 2) return 0.0f;
    double sum_sq = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (!isnan(data[i])) {
            double diff = data[i] - mean;
            sum_sq += diff * diff;
        }
    }
    return (float)sqrt(sum_sq / (count - 1));
}

// =============================================================================
// Phase Implementations
// =============================================================================

/**
 * @brief Phase 1: Prompt operator to route air to vessel
 * 
 * The operator should toggle the L-valve to route air through the vessel
 * at the configured O2 LPM.  They confirm the rotameter reading.
 */
static esp_err_t run_prompt_vessel(void)
{
    ESP_LOGI(TAG, "--- Phase 1: Prompt — Route to vessel ---");

    char prompt_msg[128];
    snprintf(prompt_msg, sizeof(prompt_msg),
             "Route L-valve to VESSEL at %.1f LPM. Confirm rotameter reading.",
             s_val.o2_lpm);

    char response[SEQ_PROMPT_VALUE_MAX] = {0};
    esp_err_t ret = seq_prompt_user(PHASE_PROMPT_VESSEL, prompt_msg,
                                     response, sizeof(response),
                                     VAL_PROMPT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Vessel prompt failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Parse confirmed LPM if provided, otherwise use configured value
    if (strlen(response) > 0) {
        float confirmed = strtof(response, NULL);
        if (confirmed >= 0.1f && confirmed <= 15.0f) {
            s_val.confirmed_vessel_lpm = confirmed;
            ESP_LOGI(TAG, "Operator confirmed vessel route: %.1f LPM", confirmed);
        } else {
            s_val.confirmed_vessel_lpm = s_val.o2_lpm;
            ESP_LOGW(TAG, "Invalid confirmed LPM '%.1f', using %.1f",
                     confirmed, s_val.o2_lpm);
        }
    } else {
        s_val.confirmed_vessel_lpm = s_val.o2_lpm;
        ESP_LOGI(TAG, "Operator confirmed (no value), using %.1f LPM", s_val.o2_lpm);
    }

    seq_report_progress(PHASE_PROMPT_VESSEL, calc_progress(0, 1.0f), 0, false);
    return ESP_OK;
}

/**
 * @brief Phase 2: Prompt operator to route air direct to 106-H
 * 
 * The operator should route air directly to the 106-H sensor (bypassing
 * the vessel) and adjust the needle valve so the flow matches the vessel
 * route flow.  They confirm both LPM values are equal.
 */
static esp_err_t run_prompt_direct(void)
{
    ESP_LOGI(TAG, "--- Phase 2: Prompt — Route direct to 106-H ---");

    char prompt_msg[192];
    snprintf(prompt_msg, sizeof(prompt_msg),
             "Route L-valve DIRECT to 106-H. Adjust needle valve to match "
             "%.1f LPM. Confirm flow is equal on both routes.",
             s_val.confirmed_vessel_lpm);

    char response[SEQ_PROMPT_VALUE_MAX] = {0};
    esp_err_t ret = seq_prompt_user(PHASE_PROMPT_DIRECT, prompt_msg,
                                     response, sizeof(response),
                                     VAL_PROMPT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Direct prompt failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Parse confirmed LPM if provided
    if (strlen(response) > 0) {
        float confirmed = strtof(response, NULL);
        if (confirmed >= 0.1f && confirmed <= 15.0f) {
            s_val.confirmed_direct_lpm = confirmed;
            ESP_LOGI(TAG, "Operator confirmed direct route: %.1f LPM", confirmed);
        } else {
            s_val.confirmed_direct_lpm = s_val.confirmed_vessel_lpm;
        }
    } else {
        s_val.confirmed_direct_lpm = s_val.confirmed_vessel_lpm;
        ESP_LOGI(TAG, "Operator confirmed (no value), using %.1f LPM",
                 s_val.confirmed_vessel_lpm);
    }

    seq_report_progress(PHASE_PROMPT_DIRECT, calc_progress(1, 1.0f), 0, false);
    return ESP_OK;
}

/**
 * @brief Phase 3: Stabilize — set power and wait for 106-H to settle
 * 
 * Turns on the generator at the target power and waits for the ozone
 * reading to reach steady state.  Uses the direct-to-sensor route so
 * there's no vessel transit delay.
 */
static esp_err_t run_stabilize(void)
{
    ESP_LOGI(TAG, "--- Phase 3: Stabilize at %u%% power (%.0fs) ---",
             s_val.power_pct, VAL_STABILIZE_TIME_MS / 1000.0f);

    // Set power
    esp_err_t ret = o3_power_set(s_val.power_pct);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power to %u%%: %s",
                 s_val.power_pct, esp_err_to_name(ret));
        return ret;
    }

    // Wait for motor to reach position
    vTaskDelay(pdMS_TO_TICKS(VAL_MOTOR_SETTLE_MS));

    // Wait for stabilization, reporting progress
    int64_t phase_start = get_time_us();
    int64_t phase_end = phase_start + (int64_t)VAL_STABILIZE_TIME_MS * 1000;

    while (get_time_us() < phase_end) {
        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        float o3_pct = NAN, temp_c = NAN;
        get_106h_sample(&o3_pct, &temp_c);
        float actual = get_actual_pct();

        // Send data during stabilization (useful for monitoring)
        send_val_point(s_val.power_pct, actual, o3_pct, temp_c);

        float elapsed_in_phase = (float)(get_time_us() - phase_start) / 1000000.0f;
        float phase_frac = elapsed_in_phase / (VAL_STABILIZE_TIME_MS / 1000.0f);
        if (phase_frac > 1.0f) phase_frac = 1.0f;

        seq_report_progress(PHASE_STABILIZE, calc_progress(2, phase_frac),
                           s_val.power_pct, false);

        vTaskDelay(pdMS_TO_TICKS(VAL_SAMPLE_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "Stabilization complete");
    return ESP_OK;
}

/**
 * @brief Phase 4: Measure — collect O3 readings and compute statistics
 * 
 * Samples the 106-H every 2 seconds for 60 seconds, computes mean and
 * standard deviation.  All samples are stored for the validation result.
 */
static esp_err_t run_measure(void)
{
    ESP_LOGI(TAG, "--- Phase 4: Measure O3 concentration (%.0fs) ---",
             VAL_MEASURE_TIME_MS / 1000.0f);

    s_val.sample_count = 0;

    int64_t phase_start = get_time_us();
    int64_t phase_end = phase_start + (int64_t)VAL_MEASURE_TIME_MS * 1000;

    while (get_time_us() < phase_end && s_val.sample_count < VAL_MAX_SAMPLES) {
        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        float o3_pct = NAN, temp_c = NAN;
        bool valid = get_106h_sample(&o3_pct, &temp_c);
        float actual = get_actual_pct();

        if (valid && !isnan(o3_pct)) {
            s_val.samples_o3[s_val.sample_count] = o3_pct;
            s_val.samples_temp[s_val.sample_count] = temp_c;
            s_val.sample_count++;
        }

        send_val_point(s_val.power_pct, actual, o3_pct, temp_c);

        float elapsed_in_phase = (float)(get_time_us() - phase_start) / 1000000.0f;
        float phase_frac = elapsed_in_phase / (VAL_MEASURE_TIME_MS / 1000.0f);
        if (phase_frac > 1.0f) phase_frac = 1.0f;

        seq_report_progress(PHASE_MEASURE, calc_progress(3, phase_frac),
                           s_val.power_pct, false);

        vTaskDelay(pdMS_TO_TICKS(VAL_SAMPLE_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "Measurement complete: %u samples", s_val.sample_count);
    return ESP_OK;
}

/**
 * @brief Phase 5: Complete — compute results and send validation summary
 */
static esp_err_t run_complete(void)
{
    ESP_LOGI(TAG, "--- Phase 5: Validation report ---");

    float mean_o3 = compute_mean(s_val.samples_o3, s_val.sample_count);
    float std_o3 = compute_std(s_val.samples_o3, s_val.sample_count, mean_o3);
    float expected_o3 = predict_o3_pct(s_val.power_pct, s_val.confirmed_direct_lpm);
    float mean_temp = compute_mean(s_val.samples_temp, s_val.sample_count);

    // Send VAL_RESULT summary
    char result_msg[256];
    snprintf(result_msg, sizeof(result_msg),
             "VAL_RESULT,power=%u,o2_lpm=%.1f,mean_o3=%.4f,std_o3=%.4f,"
             "expected_o3=%.4f,mean_temp=%.1f,samples=%u,elapsed=%.1f",
             s_val.power_pct,
             s_val.confirmed_direct_lpm,
             mean_o3,
             std_o3,
             expected_o3,
             mean_temp,
             s_val.sample_count,
             get_elapsed_s());
    seq_send_data(result_msg);

    // Log result
    float deviation = (expected_o3 > 0.001f) ? 
                      fabsf(mean_o3 - expected_o3) / expected_o3 * 100.0f : 0.0f;

    ESP_LOGI(TAG, "=== VALIDATION RESULT ===");
    ESP_LOGI(TAG, "  Power:      %u%%", s_val.power_pct);
    ESP_LOGI(TAG, "  Flow:       %.1f LPM (vessel=%.1f, direct=%.1f)",
             s_val.o2_lpm, s_val.confirmed_vessel_lpm, s_val.confirmed_direct_lpm);
    ESP_LOGI(TAG, "  Mean O3:    %.4f %%vol  (std: %.4f)", mean_o3, std_o3);
    ESP_LOGI(TAG, "  Expected:   %.4f %%vol", expected_o3);
    ESP_LOGI(TAG, "  Deviation:  %.1f%%", deviation);
    ESP_LOGI(TAG, "  Mean Temp:  %.1f C", mean_temp);
    ESP_LOGI(TAG, "  Samples:    %u in %.1fs", s_val.sample_count, get_elapsed_s());

    if (deviation > 20.0f) {
        ESP_LOGW(TAG, "  RESULT:     >20%% deviation — check system");
    } else if (deviation > 10.0f) {
        ESP_LOGI(TAG, "  RESULT:     10-20%% deviation — acceptable, model may need update");
    } else {
        ESP_LOGI(TAG, "  RESULT:     <10%% deviation — PASS");
    }

    seq_report_progress(PHASE_COMPLETE, 100.0f, 0, false);
    return ESP_OK;
}

// =============================================================================
// Sequence Interface Implementation
// =============================================================================

/**
 * @brief Parse parameters: "<power_pct>,<o2_lpm>"
 * 
 * Both are optional:
 *   - Default power: 75%  
 *   - Default LPM: 4.0
 * 
 * Examples:
 *   CMD,sequence_start,validate           → 75%, 4.0 LPM
 *   CMD,sequence_start,validate,50        → 50%, 4.0 LPM
 *   CMD,sequence_start,validate,50,5.0    → 50%, 5.0 LPM
 */
static esp_err_t val_prepare(const char *params)
{
    // Defaults
    s_val.power_pct = 75;
    s_val.o2_lpm = 4.0f;
    s_val.confirmed_vessel_lpm = 0.0f;
    s_val.confirmed_direct_lpm = 0.0f;
    s_val.stop_requested = false;
    s_val.sample_count = 0;
    s_val.start_time_us = 0;

    // Parse params
    if (params != NULL && strlen(params) > 0) {
        // Try to parse "power_pct,o2_lpm"
        char param_copy[SEQ_PARAMS_MAX];
        strncpy(param_copy, params, sizeof(param_copy) - 1);
        param_copy[sizeof(param_copy) - 1] = '\0';

        char *saveptr = NULL;
        char *tok = strtok_r(param_copy, ",", &saveptr);

        if (tok) {
            int power = atoi(tok);
            if (power >= 1 && power <= 100) {
                s_val.power_pct = (uint8_t)power;
            } else {
                ESP_LOGE(TAG, "Invalid power: %d (must be 1-100)", power);
                return ESP_ERR_INVALID_ARG;
            }
        }

        tok = strtok_r(NULL, ",", &saveptr);
        if (tok) {
            float lpm = strtof(tok, NULL);
            if (lpm >= 0.1f && lpm <= 15.0f) {
                s_val.o2_lpm = lpm;
            } else {
                ESP_LOGE(TAG, "Invalid LPM: %.1f (must be 0.1-15.0)", lpm);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    // Check prerequisites
    if (!motor_pot_is_initialized()) {
        ESP_LOGE(TAG, "Motor pot not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (relay_get_state(RELAY_OZONE_GEN) != RELAY_ON) {
        ESP_LOGE(TAG, "O3 generator relay must be ON");
        return ESP_ERR_INVALID_STATE;
    }

    if (relay_get_state(RELAY_O2_CONC) != RELAY_ON) {
        ESP_LOGW(TAG, "O2 concentrator is OFF — readings may be zero");
    }

    ESP_LOGI(TAG, "Validation prepared: power=%u%%, O2=%.1f LPM",
             s_val.power_pct, s_val.o2_lpm);
    return ESP_OK;
}

static esp_err_t val_execute(void)
{
    s_val.start_time_us = get_time_us();
    esp_err_t ret;

    // Send metadata
    char start_msg[128];
    snprintf(start_msg, sizeof(start_msg),
             "VAL_START,power=%u,o2_lpm=%.1f",
             s_val.power_pct, s_val.o2_lpm);
    seq_send_data(start_msg);

    // Phase 1: Prompt — route to vessel
    ret = run_prompt_vessel();
    if (ret != ESP_OK) return ret;

    // Phase 2: Prompt — route direct to 106-H
    ret = run_prompt_direct();
    if (ret != ESP_OK) return ret;

    // Phase 3: Stabilize at target power
    ret = run_stabilize();
    if (ret != ESP_OK) return ret;

    // Phase 4: Measure concentration
    ret = run_measure();
    if (ret != ESP_OK) return ret;

    // Phase 5: Compute and report results
    ret = run_complete();
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

static void val_request_stop(void)
{
    s_val.stop_requested = true;
    ESP_LOGW(TAG, "Validation stop requested");
}

static void val_cleanup(void)
{
    // Safe state: power to 0%
    // We do NOT turn off relays — the operator may want them on for the next step
    ESP_LOGI(TAG, "Cleanup: setting power to 0%%");
    o3_power_set(0);
}

// =============================================================================
// Public API
// =============================================================================

static const sequence_impl_t s_airflow_val_impl = {
    .type_name    = "validate",
    .description  = "Airflow/concentration validation",
    .prepare      = val_prepare,
    .execute      = val_execute,
    .request_stop = val_request_stop,
    .cleanup      = val_cleanup,
};

const sequence_impl_t* seq_airflow_val_get_impl(void)
{
    return &s_airflow_val_impl;
}
