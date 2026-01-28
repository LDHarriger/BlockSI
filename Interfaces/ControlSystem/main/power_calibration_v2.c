/**
 * @file power_calibration.c
 * @brief Power calibration sweep for MP-8000 ozone generator (motor pot version)
 * 
 * Updated for PRM162 motorized potentiometer with ADC feedback.
 * 
 * Purpose:
 * - Characterize O3 output vs. power percentage relationship
 * - Identify power zones (threshold, linear, saturation)
 * - Generate piecewise calibration model
 * 
 * Protocol:
 * 1. Start at power=0%
 * 2. Sweep up in 1% steps, capturing one 106-H sample per step
 * 3. Sweep back down in 1% steps
 * 4. The up+down sweep allows hysteresis detection
 * 5. Data streamed to LAN client as CAL_DATA messages
 * 
 * Since generator is directly connected to 106-H with minimal path volume,
 * each step only needs one 106-H sample (~2 seconds to settle).
 * 
 * LAN Commands:
 *   calibrate_start - Start calibration sweep (if O3 gen relay is ON)
 *   calibrate_stop  - Abort calibration sweep
 *   calibrate_status - Get current calibration status
 * 
 * Output Format (over LAN):
 *   CAL_DATA,power_pct,actual_pct,o3_pct,temp_c,direction,elapsed_s
 *   direction: 1=ascending, -1=descending
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "power_calibration.h"
#include "motor_pot.h"
#include "o3_power_control.h"
#include "blocksi_state.h"
#include "lan_client.h"

static const char *TAG = "CAL";

// ============================================================================
// Configuration
// ============================================================================

// Sweep parameters - designed for fast characterization
#define STEP_SIZE_PCT           1       // 1% steps for fine resolution
#define SETTLE_TIME_MS          2500    // Wait for 106-H sample (2s avg) + margin
#define CAL_MOTOR_SETTLE_MS     500     // Additional time for motor to stop (longer than motor_pot.h)
#define MAX_SWEEP_TIME_MS       600000  // 10 minute max sweep time

// ============================================================================
// State
// ============================================================================

typedef struct {
    bool active;
    bool stop_requested;
    int8_t direction;           // 1 = ascending, -1 = descending
    uint8_t current_pct;
    uint8_t target_pct;
    uint16_t points_up;
    uint16_t points_down;
    int64_t start_time_ms;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;
} calibration_ctx_t;

static calibration_ctx_t s_cal = {0};

// ============================================================================
// Calibration Data Point
// ============================================================================

typedef struct {
    uint8_t power_pct;          // Commanded power
    float actual_pct;           // Actual power from ADC
    float o3_pct;               // Ozone %vol from 106-H
    float temp_c;               // Cell temperature
    int8_t direction;           // 1 = up, -1 = down
    float elapsed_s;            // Seconds since sweep start
} cal_data_point_t;

// ============================================================================
// Helper Functions
// ============================================================================

static int64_t get_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t wait_for_motor_settle(uint32_t timeout_ms)
{
    int64_t deadline = get_time_ms() + timeout_ms;
    
    while (get_time_ms() < deadline) {
        motor_pot_state_t state;
        motor_pot_get_state(&state);
        
        if (!state.is_moving) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    return ESP_ERR_TIMEOUT;
}

static bool get_106h_sample(float *o3_pct, float *temp_c)
{
    // Get readings from blocksi_state (updated by on_106h_sample callback)
    const blocksi_state_t *state = blocksi_state_get();
    if (state == NULL || !state->sensors.vessel_o3_valid) {
        return false;
    }
    
    *o3_pct = state->sensors.vessel_o3_pct;
    *temp_c = state->sensors.cell_temp_c;
    return true;
}

static void send_cal_data(const cal_data_point_t *point)
{
    char msg[128];
    snprintf(msg, sizeof(msg), 
             "CAL_DATA,%u,%.1f,%.4f,%.1f,%d,%.1f",
             point->power_pct,
             point->actual_pct,
             point->o3_pct,
             point->temp_c,
             point->direction,
             point->elapsed_s);
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(msg);
#endif
    
    // Also log for serial monitoring
    ESP_LOGI(TAG, "CAL: P=%u%%, A=%.1f%%, O3=%.4f%%, T=%.1f, dir=%d",
             point->power_pct, point->actual_pct, point->o3_pct,
             point->temp_c, point->direction);
}

// ============================================================================
// Calibration Sweep Task
// ============================================================================

static void calibration_sweep_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Calibration sweep starting...");
    
    s_cal.active = true;
    s_cal.stop_requested = false;
    s_cal.points_up = 0;
    s_cal.points_down = 0;
    s_cal.start_time_ms = get_time_ms();
    
    // Update blocksi_state (use mutable getter for calibration updates)
    blocksi_state_t *state = blocksi_state_get_mutable();
    if (state) {
        state->calibration.active = true;
        state->calibration.stop_requested = false;
        state->calibration.current_step_pct = 0;
        state->calibration.direction = 1;
        state->calibration.points_collected = 0;
        state->calibration.start_time_ms = s_cal.start_time_ms;
    }
    
    // -------------------------------------------------------------------------
    // Phase 1: Ascending sweep (0% → 100%)
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 1: Ascending 0%% → 100%%");
    s_cal.direction = 1;
    
    for (s_cal.current_pct = 0; s_cal.current_pct <= 100; s_cal.current_pct += STEP_SIZE_PCT) {
        
        // Check for stop request or timeout
        if (s_cal.stop_requested) {
            ESP_LOGW(TAG, "Sweep stopped by user");
            goto cleanup;
        }
        
        int64_t elapsed = get_time_ms() - s_cal.start_time_ms;
        if (elapsed > MAX_SWEEP_TIME_MS) {
            ESP_LOGE(TAG, "Sweep timeout exceeded");
            goto cleanup;
        }
        
        // Set power and wait for motor
        esp_err_t ret = o3_power_set(s_cal.current_pct);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set power to %u%%", s_cal.current_pct);
            continue;
        }
        
        // Wait for motor to reach position
        vTaskDelay(pdMS_TO_TICKS(CAL_MOTOR_SETTLE_MS));
        wait_for_motor_settle(2000);
        
        // Wait for 106-H measurement
        vTaskDelay(pdMS_TO_TICKS(SETTLE_TIME_MS));
        
        // Get readings
        motor_pot_state_t pot_state;
        motor_pot_get_state(&pot_state);
        
        float o3_pct, temp_c;
        if (!get_106h_sample(&o3_pct, &temp_c)) {
            ESP_LOGW(TAG, "No 106-H data at %u%%", s_cal.current_pct);
            o3_pct = NAN;
            temp_c = NAN;
        }
        
        // Create and send data point
        cal_data_point_t point = {
            .power_pct = s_cal.current_pct,
            .actual_pct = pot_state.position_percent,
            .o3_pct = o3_pct,
            .temp_c = temp_c,
            .direction = 1,
            .elapsed_s = (get_time_ms() - s_cal.start_time_ms) / 1000.0f
        };
        
        send_cal_data(&point);
        s_cal.points_up++;
        
        // Update state
        if (state) {
            state->calibration.current_step_pct = s_cal.current_pct;
            state->calibration.points_collected = s_cal.points_up;
        }
    }
    
    // -------------------------------------------------------------------------
    // Phase 2: Descending sweep (100% → 0%)
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 2: Descending 100%% → 0%%");
    s_cal.direction = -1;
    
    // Start from 99% to avoid duplicate 100% point
    for (s_cal.current_pct = 99; s_cal.current_pct > 0; s_cal.current_pct -= STEP_SIZE_PCT) {
        
        if (s_cal.stop_requested) {
            ESP_LOGW(TAG, "Sweep stopped by user");
            goto cleanup;
        }
        
        int64_t elapsed = get_time_ms() - s_cal.start_time_ms;
        if (elapsed > MAX_SWEEP_TIME_MS) {
            ESP_LOGE(TAG, "Sweep timeout exceeded");
            goto cleanup;
        }
        
        esp_err_t ret = o3_power_set(s_cal.current_pct);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set power to %u%%", s_cal.current_pct);
            continue;
        }
        
        vTaskDelay(pdMS_TO_TICKS(CAL_MOTOR_SETTLE_MS));
        wait_for_motor_settle(2000);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_TIME_MS));
        
        motor_pot_state_t pot_state;
        motor_pot_get_state(&pot_state);
        
        float o3_pct, temp_c;
        if (!get_106h_sample(&o3_pct, &temp_c)) {
            o3_pct = NAN;
            temp_c = NAN;
        }
        
        cal_data_point_t point = {
            .power_pct = s_cal.current_pct,
            .actual_pct = pot_state.position_percent,
            .o3_pct = o3_pct,
            .temp_c = temp_c,
            .direction = -1,
            .elapsed_s = (get_time_ms() - s_cal.start_time_ms) / 1000.0f
        };
        
        send_cal_data(&point);
        s_cal.points_down++;
        
        if (state) {
            state->calibration.current_step_pct = s_cal.current_pct;
            state->calibration.direction = -1;
            state->calibration.points_collected = s_cal.points_up + s_cal.points_down;
        }
    }
    
    // Final 0% point
    o3_power_set(0);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_TIME_MS));
    
    motor_pot_state_t pot_state;
    motor_pot_get_state(&pot_state);
    
    float o3_pct, temp_c;
    get_106h_sample(&o3_pct, &temp_c);
    
    cal_data_point_t point = {
        .power_pct = 0,
        .actual_pct = pot_state.position_percent,
        .o3_pct = o3_pct,
        .temp_c = temp_c,
        .direction = -1,
        .elapsed_s = (get_time_ms() - s_cal.start_time_ms) / 1000.0f
    };
    send_cal_data(&point);
    s_cal.points_down++;
    
    ESP_LOGI(TAG, "Calibration complete! Up: %u points, Down: %u points",
             s_cal.points_up, s_cal.points_down);
    
cleanup:
    // Ensure safe state
    o3_power_set(0);
    
    s_cal.active = false;
    s_cal.task_handle = NULL;
    
    if (state) {
        state->calibration.active = false;
    }
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    char complete_msg[64];
    snprintf(complete_msg, sizeof(complete_msg), 
             "CAL_COMPLETE,up=%u,down=%u", s_cal.points_up, s_cal.points_down);
    lan_client_send_message(complete_msg);
#endif
    
    vTaskDelete(NULL);
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t power_calibration_init(void)
{
    if (s_cal.mutex == NULL) {
        s_cal.mutex = xSemaphoreCreateMutex();
        if (s_cal.mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    s_cal.active = false;
    s_cal.stop_requested = false;
    s_cal.task_handle = NULL;
    
    ESP_LOGI(TAG, "Power calibration initialized");
    return ESP_OK;
}

esp_err_t power_calibration_start(void)
{
    if (s_cal.mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_cal.mutex, portMAX_DELAY);
    
    if (s_cal.active) {
        xSemaphoreGive(s_cal.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if motor pot is initialized
    if (!motor_pot_is_initialized()) {
        ESP_LOGE(TAG, "Motor pot not initialized");
        xSemaphoreGive(s_cal.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create calibration task
    BaseType_t ret = xTaskCreate(
        calibration_sweep_task,
        "cal_sweep",
        4096,
        NULL,
        5,
        &s_cal.task_handle
    );
    
    xSemaphoreGive(s_cal.mutex);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create calibration task");
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t power_calibration_stop(void)
{
    if (!s_cal.active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_cal.stop_requested = true;
    ESP_LOGI(TAG, "Calibration stop requested");
    
    return ESP_OK;
}

bool power_calibration_is_active(void)
{
    return s_cal.active;
}

esp_err_t power_calibration_get_status(power_cal_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    status->active = s_cal.active;
    status->direction = s_cal.direction;
    status->current_pct = s_cal.current_pct;
    status->points_up = s_cal.points_up;
    status->points_down = s_cal.points_down;
    
    if (s_cal.active) {
        status->elapsed_s = (get_time_ms() - s_cal.start_time_ms) / 1000.0f;
    } else {
        status->elapsed_s = 0;
    }
    
    return ESP_OK;
}
