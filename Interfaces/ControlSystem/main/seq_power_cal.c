/**
 * @file seq_power_cal.c
 * @brief Power-O3 calibration sequence implementation
 * 
 * Four-phase calibration:
 *   Phase 1 — Baseline:     30s at 0% power, air OFF (establish zero)
 *   Phase 2 — Sweep Up:     0→100% in 1% steps, ~3s per step, air OFF
 *   Phase 3 — Sweep Down:   100→0% in 1% steps, ~3s per step, air OFF
 *   Phase 4 — Random Pairs: 15 random power levels × (20s air OFF + 20s air ON)
 * 
 * Each measurement point is streamed as a CAL_DATA message.
 * SEQ progress messages are sent by the framework (~1/s).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

#include "seq_power_cal.h"
#include "sequence_runner.h"
#include "motor_pot.h"
#include "o3_power_control.h"
#include "relay_control.h"
#include "blocksi_state.h"
#include "lan_client.h"

static const char *TAG = "SEQ_CAL";

// =============================================================================
// Configuration
// =============================================================================

#define CAL_STEP_SIZE_PCT           1       // 1% steps for fine resolution
#define CAL_SETTLE_TIME_MS          2500    // Wait for 106-H sample (~2s) + margin
#define CAL_MOTOR_SETTLE_MS         500     // Wait for motor to stop
#define CAL_BASELINE_DURATION_MS    30000   // 30 second baseline at 0%
#define CAL_BASELINE_SAMPLE_MS      2000    // Sample every 2s during baseline
#define CAL_RANDOM_AIR_OFF_MS       20000   // 20s with air OFF per random level
#define CAL_RANDOM_AIR_ON_MS        20000   // 20s with air ON per random level
#define CAL_RANDOM_SAMPLE_MS        2000    // Sample every 2s during random phase
#define CAL_RANDOM_COUNT            15      // Number of random power levels
#define CAL_MAX_TOTAL_TIME_MS       1500000 // 25 minute absolute max

// Phase weights for progress calculation (approximate time ratios)
#define WEIGHT_BASELINE             2       // ~30s
#define WEIGHT_SWEEP_UP             30      // ~5 min
#define WEIGHT_SWEEP_DOWN           30      // ~5 min
#define WEIGHT_RANDOM               38      // ~10 min
#define WEIGHT_TOTAL                (WEIGHT_BASELINE + WEIGHT_SWEEP_UP + WEIGHT_SWEEP_DOWN + WEIGHT_RANDOM)

// =============================================================================
// Phase Names (used in SEQ messages and CAL_DATA)
// =============================================================================

static const char *PHASE_BASELINE   = "baseline";
static const char *PHASE_SWEEP_UP   = "sweep_up";
static const char *PHASE_SWEEP_DOWN = "sweep_down";
static const char *PHASE_RANDOM     = "random_pair";

// =============================================================================
// Internal State
// =============================================================================

typedef struct {
    // Parameters
    float o2_lpm;                   // O2 flow rate (from command params)

    // Stop flag
    volatile bool stop_requested;

    // Counters
    uint16_t total_points;
    uint16_t points_baseline;
    uint16_t points_sweep_up;
    uint16_t points_sweep_down;
    uint16_t points_random;

    // Random levels (generated on prepare)
    uint8_t random_levels[CAL_RANDOM_COUNT];

    // Start time for elapsed calculation
    int64_t start_time_us;
} cal_state_t;

static cal_state_t s_cal = {0};

// =============================================================================
// Helpers
// =============================================================================

static int64_t get_time_us(void)
{
    return esp_timer_get_time();
}

static float get_elapsed_s(void)
{
    return (float)(get_time_us() - s_cal.start_time_us) / 1000000.0f;
}

/**
 * @brief Wait for motor pot to stop moving
 */
static esp_err_t wait_for_motor(uint32_t timeout_ms)
{
    int64_t deadline = get_time_us() + (int64_t)timeout_ms * 1000;

    while (get_time_us() < deadline) {
        motor_pot_state_t state;
        motor_pot_get_state(&state);
        if (!state.is_moving) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Get current 106-H readings from blocksi_state
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
 * @brief Get current motor pot actual position
 */
static float get_actual_pct(void)
{
    motor_pot_state_t state;
    motor_pot_get_state(&state);
    return state.position_percent;
}

/**
 * @brief Calculate total LPM based on O2 flow and air compressor state
 */
static float calc_total_lpm(float o2_lpm, bool air_on)
{
    return o2_lpm + (air_on ? 10.0f : 0.0f);
}

/**
 * @brief Send one CAL_DATA point over LAN
 * 
 * Format: CAL_DATA,timestamp_ms,power_pct,actual_pct,o3_pct,o2_lpm,
 *         air_comp_on,total_lpm,cell_temp_c,phase
 */
static void send_cal_point(uint8_t power_pct, float actual_pct,
                           float o3_pct, float cell_temp_c,
                           bool air_comp_on, const char *phase)
{
    float total_lpm = calc_total_lpm(s_cal.o2_lpm, air_comp_on);

    // Use blocksi_state time (PC-synced if available)
    int64_t timestamp_ms = blocksi_state_get_time_ms();

    char msg[192];
    snprintf(msg, sizeof(msg),
             "CAL_DATA,%lld,%u,%.1f,%.4f,%.1f,%d,%.1f,%.1f,%s",
             (long long)timestamp_ms,
             power_pct,
             actual_pct,
             o3_pct,
             s_cal.o2_lpm,
             air_comp_on ? 1 : 0,
             total_lpm,
             cell_temp_c,
             phase);

    seq_send_data(msg);

    // Local log at info level for serial monitor
    ESP_LOGI(TAG, "CAL: P=%u%% A=%.1f%% O3=%.4f%% T=%.1f air=%d [%s]",
             power_pct, actual_pct, o3_pct, cell_temp_c, air_comp_on, phase);

    s_cal.total_points++;
}

/**
 * @brief Set power and wait for settle
 * @return ESP_OK on success, ESP_FAIL if stop requested
 */
static esp_err_t set_power_and_settle(uint8_t pct)
{
    esp_err_t ret = o3_power_set(pct);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power to %u%%: %s", pct, esp_err_to_name(ret));
        return ret;
    }

    // Wait for motor to reach position
    vTaskDelay(pdMS_TO_TICKS(CAL_MOTOR_SETTLE_MS));
    wait_for_motor(2000);

    // Wait for 106-H measurement cycle
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_TIME_MS));

    return ESP_OK;
}

/**
 * @brief Calculate overall progress percentage
 */
static float calc_progress(int phase_index, float phase_fraction)
{
    float base = 0;
    float weights[] = {WEIGHT_BASELINE, WEIGHT_SWEEP_UP, WEIGHT_SWEEP_DOWN, WEIGHT_RANDOM};
    for (int i = 0; i < phase_index; i++) {
        base += weights[i];
    }
    return (base + weights[phase_index] * phase_fraction) / WEIGHT_TOTAL * 100.0f;
}

/**
 * @brief Generate 15 random power levels in 1–100 range using Fisher-Yates partial shuffle
 */
static void generate_random_levels(void)
{
    // Create array of possible levels (1–100)
    uint8_t pool[100];
    for (int i = 0; i < 100; i++) {
        pool[i] = (uint8_t)(i + 1);
    }

    // Fisher-Yates partial shuffle: pick CAL_RANDOM_COUNT from the pool
    for (int i = 0; i < CAL_RANDOM_COUNT; i++) {
        uint32_t rand_val = esp_random();
        int j = i + (rand_val % (100 - i));

        // Swap
        uint8_t tmp = pool[i];
        pool[i] = pool[j];
        pool[j] = tmp;

        s_cal.random_levels[i] = pool[i];
    }

    // Sort for logging (simple insertion sort on 15 elements)
    for (int i = 1; i < CAL_RANDOM_COUNT; i++) {
        uint8_t key = s_cal.random_levels[i];
        int j = i - 1;
        while (j >= 0 && s_cal.random_levels[j] > key) {
            s_cal.random_levels[j + 1] = s_cal.random_levels[j];
            j--;
        }
        s_cal.random_levels[j + 1] = key;
    }

    ESP_LOGI(TAG, "Random levels: ");
    for (int i = 0; i < CAL_RANDOM_COUNT; i++) {
        ESP_LOGI(TAG, "  [%d] = %u%%", i, s_cal.random_levels[i]);
    }
}

// =============================================================================
// Phase Implementations
// =============================================================================

/**
 * @brief Phase 1: Baseline — 30s at 0% power, air OFF
 * Establishes the zero-O3 baseline for the sensor.
 */
static esp_err_t run_baseline(void)
{
    ESP_LOGI(TAG, "--- Phase 1: Baseline (30s at 0%%, air OFF) ---");

    // Ensure air compressor is OFF
    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);

    // Set power to 0%
    esp_err_t ret = o3_power_set(0);
    if (ret != ESP_OK) return ret;

    int64_t phase_start = get_time_us();
    int64_t phase_end = phase_start + (int64_t)CAL_BASELINE_DURATION_MS * 1000;

    while (get_time_us() < phase_end) {
        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        // Sample
        float o3_pct = NAN, temp_c = NAN;
        get_106h_sample(&o3_pct, &temp_c);
        float actual = get_actual_pct();

        send_cal_point(0, actual, o3_pct, temp_c, false, PHASE_BASELINE);
        s_cal.points_baseline++;

        // Progress within baseline phase
        float elapsed_in_phase = (float)(get_time_us() - phase_start) / 1000000.0f;
        float phase_frac = elapsed_in_phase / (CAL_BASELINE_DURATION_MS / 1000.0f);
        if (phase_frac > 1.0f) phase_frac = 1.0f;
        seq_report_progress(PHASE_BASELINE, calc_progress(0, phase_frac), 0, false);

        vTaskDelay(pdMS_TO_TICKS(CAL_BASELINE_SAMPLE_MS));
    }

    ESP_LOGI(TAG, "Baseline complete: %u points", s_cal.points_baseline);
    return ESP_OK;
}

/**
 * @brief Phase 2: Sweep Up — 0% → 100% in 1% steps, air OFF
 */
static esp_err_t run_sweep_up(void)
{
    ESP_LOGI(TAG, "--- Phase 2: Sweep Up 0%% → 100%% ---");

    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);

    uint8_t total_steps = 100 / CAL_STEP_SIZE_PCT + 1;  // 0,1,2,...,100

    for (uint8_t pct = 0; pct <= 100; pct += CAL_STEP_SIZE_PCT) {
        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        esp_err_t ret = set_power_and_settle(pct);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power set failed at %u%%, skipping", pct);
            continue;
        }

        float o3_pct = NAN, temp_c = NAN;
        get_106h_sample(&o3_pct, &temp_c);
        float actual = get_actual_pct();

        send_cal_point(pct, actual, o3_pct, temp_c, false, PHASE_SWEEP_UP);
        s_cal.points_sweep_up++;

        float phase_frac = (float)pct / 100.0f;
        seq_report_progress(PHASE_SWEEP_UP, calc_progress(1, phase_frac), pct, false);
    }

    ESP_LOGI(TAG, "Sweep up complete: %u points", s_cal.points_sweep_up);
    return ESP_OK;
}

/**
 * @brief Phase 3: Sweep Down — 100% → 0% in 1% steps, air OFF
 * Starts from 99% to avoid duplicate 100% point.
 */
static esp_err_t run_sweep_down(void)
{
    ESP_LOGI(TAG, "--- Phase 3: Sweep Down 100%% → 0%% ---");

    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);

    for (int pct = 100 - CAL_STEP_SIZE_PCT; pct >= 0; pct -= CAL_STEP_SIZE_PCT) {
        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        esp_err_t ret = set_power_and_settle((uint8_t)pct);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power set failed at %d%%, skipping", pct);
            continue;
        }

        float o3_pct = NAN, temp_c = NAN;
        get_106h_sample(&o3_pct, &temp_c);
        float actual = get_actual_pct();

        send_cal_point((uint8_t)pct, actual, o3_pct, temp_c, false, PHASE_SWEEP_DOWN);
        s_cal.points_sweep_down++;

        float phase_frac = (float)(100 - pct) / 100.0f;
        seq_report_progress(PHASE_SWEEP_DOWN, calc_progress(2, phase_frac), (uint8_t)pct, false);
    }

    ESP_LOGI(TAG, "Sweep down complete: %u points", s_cal.points_sweep_down);
    return ESP_OK;
}

/**
 * @brief Phase 4: Random Pairs — 15 random levels × (20s air OFF + 20s air ON)
 * 
 * For each random power level:
 *   1. Set power, wait for settle
 *   2. Air OFF for 20s — sample every 2s
 *   3. Air ON for 20s — sample every 2s
 * 
 * This measures the effect of air dilution at various power levels.
 */
static esp_err_t run_random_pairs(void)
{
    ESP_LOGI(TAG, "--- Phase 4: Random Pairs (%d levels × air OFF/ON) ---",
             CAL_RANDOM_COUNT);

    for (int level = 0; level < CAL_RANDOM_COUNT; level++) {
        uint8_t pct = s_cal.random_levels[level];

        ESP_LOGI(TAG, "Random level %d/%d: %u%%", level + 1, CAL_RANDOM_COUNT, pct);

        if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

        // Set power
        esp_err_t ret = set_power_and_settle(pct);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power set failed at %u%%, skipping", pct);
            continue;
        }

        // --- Air OFF phase (20s) ---
        relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Extra settle after air change

        int64_t phase_start = get_time_us();
        int64_t phase_end = phase_start + (int64_t)CAL_RANDOM_AIR_OFF_MS * 1000;

        while (get_time_us() < phase_end) {
            if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

            float o3_pct = NAN, temp_c = NAN;
            get_106h_sample(&o3_pct, &temp_c);
            float actual = get_actual_pct();

            send_cal_point(pct, actual, o3_pct, temp_c, false, PHASE_RANDOM);
            s_cal.points_random++;

            // Progress: each level is 1/15 of the random phase,
            // air-off is first half, air-on is second half
            float level_frac = ((float)level + 0.0f + 
                                (float)(get_time_us() - phase_start) / 
                                (float)((CAL_RANDOM_AIR_OFF_MS + CAL_RANDOM_AIR_ON_MS) * 1000))
                               / (float)CAL_RANDOM_COUNT;
            seq_report_progress(PHASE_RANDOM, calc_progress(3, level_frac), pct, false);

            vTaskDelay(pdMS_TO_TICKS(CAL_RANDOM_SAMPLE_MS));
        }

        // --- Air ON phase (20s) ---
        relay_set_with_source(RELAY_AIR_COMP, RELAY_ON, RELAY_SRC_SEQUENCE);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Settle after air compressor start

        phase_start = get_time_us();
        phase_end = phase_start + (int64_t)CAL_RANDOM_AIR_ON_MS * 1000;

        while (get_time_us() < phase_end) {
            if (seq_check_stop()) return ESP_ERR_INVALID_STATE;

            float o3_pct = NAN, temp_c = NAN;
            get_106h_sample(&o3_pct, &temp_c);
            float actual = get_actual_pct();

            send_cal_point(pct, actual, o3_pct, temp_c, true, PHASE_RANDOM);
            s_cal.points_random++;

            float level_frac = ((float)level + 0.5f + 
                                (float)(get_time_us() - phase_start) / 
                                (float)((CAL_RANDOM_AIR_ON_MS) * 1000) * 0.5f)
                               / (float)CAL_RANDOM_COUNT;
            seq_report_progress(PHASE_RANDOM, calc_progress(3, level_frac), pct, true);

            vTaskDelay(pdMS_TO_TICKS(CAL_RANDOM_SAMPLE_MS));
        }
    }

    // Turn air compressor OFF after random phase
    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);

    ESP_LOGI(TAG, "Random pairs complete: %u points", s_cal.points_random);
    return ESP_OK;
}

// =============================================================================
// Sequence Interface Implementation
// =============================================================================

static esp_err_t cal_prepare(const char *params)
{
    // Parse O2 LPM from params (default to 4.0 if not provided)
    s_cal.o2_lpm = 4.0f;
    if (params != NULL && strlen(params) > 0) {
        float lpm = strtof(params, NULL);
        if (lpm >= 0.1f && lpm <= 15.0f) {
            s_cal.o2_lpm = lpm;
        } else {
            ESP_LOGE(TAG, "Invalid O2 LPM: %.1f (must be 0.1–15.0)", lpm);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Check prerequisites
    if (!motor_pot_is_initialized()) {
        ESP_LOGE(TAG, "Motor pot not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // O3 generator must be ON to produce ozone
    if (relay_get_state(RELAY_OZONE_GEN) != RELAY_ON) {
        ESP_LOGE(TAG, "O3 generator relay must be ON before calibration");
        return ESP_ERR_INVALID_STATE;
    }

    // O2 concentrator should be ON for gas flow
    if (relay_get_state(RELAY_O2_CONC) != RELAY_ON) {
        ESP_LOGW(TAG, "O2 concentrator is OFF — calibration will proceed but readings may be zero");
    }

    // Generate random levels
    generate_random_levels();

    // Reset counters
    s_cal.stop_requested = false;
    s_cal.total_points = 0;
    s_cal.points_baseline = 0;
    s_cal.points_sweep_up = 0;
    s_cal.points_sweep_down = 0;
    s_cal.points_random = 0;
    s_cal.start_time_us = 0;  // Will be set in execute

    ESP_LOGI(TAG, "Calibration prepared: O2=%.1f LPM, %d random levels",
             s_cal.o2_lpm, CAL_RANDOM_COUNT);
    return ESP_OK;
}

static esp_err_t cal_execute(void)
{
    s_cal.start_time_us = get_time_us();
    esp_err_t ret;

    // Send initial metadata as a special CAL_START message
    char start_msg[128];
    snprintf(start_msg, sizeof(start_msg),
             "CAL_START,o2_lpm=%.1f,random_count=%d,step_pct=%d",
             s_cal.o2_lpm, CAL_RANDOM_COUNT, CAL_STEP_SIZE_PCT);
    seq_send_data(start_msg);

    // Phase 1: Baseline
    ret = run_baseline();
    if (ret != ESP_OK) return ret;

    // Phase 2: Sweep Up
    ret = run_sweep_up();
    if (ret != ESP_OK) return ret;

    // Phase 3: Sweep Down
    ret = run_sweep_down();
    if (ret != ESP_OK) return ret;

    // Phase 4: Random Pairs
    ret = run_random_pairs();
    if (ret != ESP_OK) return ret;

    // All phases complete — send summary
    char done_msg[192];
    snprintf(done_msg, sizeof(done_msg),
             "CAL_COMPLETE,total=%u,baseline=%u,sweep_up=%u,sweep_down=%u,random=%u,elapsed=%.1f",
             s_cal.total_points,
             s_cal.points_baseline,
             s_cal.points_sweep_up,
             s_cal.points_sweep_down,
             s_cal.points_random,
             get_elapsed_s());
    seq_send_data(done_msg);

    seq_report_progress("complete", 100.0f, 0, false);

    ESP_LOGI(TAG, "Calibration complete! Total: %u pts (B=%u, U=%u, D=%u, R=%u) in %.1fs",
             s_cal.total_points,
             s_cal.points_baseline,
             s_cal.points_sweep_up,
             s_cal.points_sweep_down,
             s_cal.points_random,
             get_elapsed_s());

    return ESP_OK;
}

static void cal_request_stop(void)
{
    s_cal.stop_requested = true;
    ESP_LOGW(TAG, "Calibration stop requested");
}

static void cal_cleanup(void)
{
    // Safe state: power to 0%, air compressor OFF
    ESP_LOGI(TAG, "Cleanup: setting power to 0%%, air comp OFF");
    o3_power_set(0);
    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);

    // Note: we do NOT turn off O3 generator or O2 concentrator — 
    // those were preconditions and the user may want them to stay on.
}

// =============================================================================
// Public API
// =============================================================================

static const sequence_impl_t s_power_cal_impl = {
    .type_name    = "cal",
    .description  = "Power-O3 calibration (4-phase)",
    .prepare      = cal_prepare,
    .execute      = cal_execute,
    .request_stop = cal_request_stop,
    .cleanup      = cal_cleanup,
};

const sequence_impl_t* seq_power_cal_get_impl(void)
{
    return &s_power_cal_impl;
}
