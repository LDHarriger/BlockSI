/**
 * @file blocksi_state.c
 * @brief Unified system state management implementation
 */

#include "blocksi_state.h"
#include "motor_pot.h"
#include "o3_power_control.h"
#include "relay_control.h"
#include "lan_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "STATE";

// =============================================================================
// Module State
// =============================================================================

static struct {
    blocksi_state_t state;
    SemaphoreHandle_t mutex;
    TaskHandle_t validator_task;
    bool initialized;
} s_state_mgr = {0};

// =============================================================================
// Internal Helpers
// =============================================================================

static void lock(void)
{
    if (s_state_mgr.mutex) {
        xSemaphoreTake(s_state_mgr.mutex, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_state_mgr.mutex) {
        xSemaphoreGive(s_state_mgr.mutex);
    }
}

static int64_t get_uptime_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// =============================================================================
// State Validator Task
// =============================================================================

static void state_validator_task(void *arg)
{
    ESP_LOGI(TAG, "State validator task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(STATE_VALIDATE_INTERVAL_MS));
        blocksi_state_validate();
    }
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t blocksi_state_init(void)
{
    if (s_state_mgr.initialized) {
        return ESP_OK;
    }
    
    // Create mutex
    s_state_mgr.mutex = xSemaphoreCreateMutex();
    if (s_state_mgr.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize state to safe defaults
    memset(&s_state_mgr.state, 0, sizeof(blocksi_state_t));
    s_state_mgr.state.sensors.vessel_temp_c = -999.0f;  // Invalid marker
    
    // Start validator task
    BaseType_t ret = xTaskCreate(
        state_validator_task,
        "state_val",
        4096,
        NULL,
        3,  // Lower priority than main tasks
        &s_state_mgr.validator_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create validator task");
        // Non-fatal, continue without automatic validation
    }
    
    s_state_mgr.initialized = true;
    ESP_LOGI(TAG, "State manager initialized");
    
    return ESP_OK;
}

const blocksi_state_t* blocksi_state_get(void)
{
    return &s_state_mgr.state;
}

blocksi_state_t* blocksi_state_get_mutable(void)
{
    return &s_state_mgr.state;
}

esp_err_t blocksi_state_set_power(uint8_t target_pct)
{
    if (target_pct > 100) target_pct = 100;
    
    lock();
    s_state_mgr.state.power.target_pct = target_pct;
    s_state_mgr.state.power.last_command_ms = get_uptime_ms();
    s_state_mgr.state.power.retry_count = 0;
    s_state_mgr.state.power.error_mismatch = false;
    s_state_mgr.state.power.motor_moving = true;  // Set before movement starts
    unlock();
    
    ESP_LOGI(TAG, "Power target set to %u%%", target_pct);
    
    // Initiate motor movement (blocking call)
    esp_err_t ret = o3_power_set(target_pct);
    
    // Motor movement complete (or failed)
    lock();
    s_state_mgr.state.power.motor_moving = false;
    s_state_mgr.state.power.last_command_ms = get_uptime_ms();  // Update time after movement
    unlock();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t blocksi_state_set_relay(uint8_t relay, bool state, relay_source_t source)
{
    if (relay >= RELAY_COUNT) {
        ESP_LOGE(TAG, "Invalid relay ID: %d", relay);
        return ESP_ERR_INVALID_ARG;
    }
    
    relay_id_t relay_id = (relay_id_t)relay;
    
    lock();
    s_state_mgr.state.relays.relays[relay].target = state;
    unlock();
    
    esp_err_t ret = relay_set_with_source(relay_id, state ? RELAY_ON : RELAY_OFF, source);
    
    if (ret == ESP_OK) {
        lock();
        s_state_mgr.state.relays.relays[relay].actual = state;
        s_state_mgr.state.relays.relays[relay].last_source = source;
        s_state_mgr.state.relays.relays[relay].last_change_ms = get_uptime_ms();
        unlock();
    }
    
    return ret;
}

void blocksi_state_update_sensors(const sensor_state_t *sensors)
{
    if (sensors == NULL) return;
    
    lock();
    memcpy(&s_state_mgr.state.sensors, sensors, sizeof(sensor_state_t));
    unlock();
}

void blocksi_state_update_power_actual(float actual_pct, float voltage, uint16_t adc_raw)
{
    lock();
    s_state_mgr.state.power.actual_pct = actual_pct;
    s_state_mgr.state.power.wiper_voltage = voltage;
    s_state_mgr.state.power.adc_raw = adc_raw;
    unlock();
}

void blocksi_state_sync_time(int64_t pc_timestamp_ms)
{
    int64_t esp_time = get_uptime_ms();
    
    lock();
    s_state_mgr.state.timing.pc_time_offset_ms = pc_timestamp_ms - esp_time;
    s_state_mgr.state.timing.time_synced = true;
    s_state_mgr.state.timing.last_sync_ms = esp_time;
    unlock();
    
    ESP_LOGI(TAG, "Time synced with PC, offset=%lldms", 
             (long long)s_state_mgr.state.timing.pc_time_offset_ms);
}

int64_t blocksi_state_get_time_ms(void)
{
    int64_t now = get_uptime_ms();
    
    lock();
    s_state_mgr.state.timing.esp_uptime_ms = now;
    int64_t result = now;
    if (s_state_mgr.state.timing.time_synced) {
        result += s_state_mgr.state.timing.pc_time_offset_ms;
    }
    unlock();
    
    return result;
}

void blocksi_state_validate(void)
{
    lock();
    blocksi_state_t *state = &s_state_mgr.state;
    
    // Skip if motor is currently moving
    if (state->power.motor_moving) {
        unlock();
        return;
    }
    
    // Wait a bit after command completes before checking (allow ADC to settle)
    int64_t now = get_uptime_ms();
    if (now - state->power.last_command_ms < POWER_SETTLE_TIME_MS) {
        unlock();
        return;
    }
    
    // Read fresh ADC position (not the stale cached value)
    float fresh_pct = motor_pot_get_position_percent();
    
    // Check power mismatch with hysteresis:
    //   Enter error state when error > POWER_MISMATCH_TOLERANCE (5%)
    //   Clear error state when error < POWER_MISMATCH_CLEAR (2.5%)
    float error = state->power.target_pct - fresh_pct;
    if (error < 0) error = -error;
    
    float threshold = state->power.error_mismatch
                    ? POWER_MISMATCH_CLEAR
                    : POWER_MISMATCH_TOLERANCE;
    
    if (error > threshold) {
        if (!state->power.error_mismatch) {
            state->power.error_mismatch = true;
            state->power_mismatch_count++;
            ESP_LOGW(TAG, "Power mismatch detected: target=%u%%, actual=%.1f%%, error=%.1f%%",
                     state->power.target_pct, fresh_pct, error);
        }
        
        // Always retry — no retry limit.  If something keeps knocking the
        // motor off position, the validator will keep correcting it.
        state->power.retry_count++;
        uint8_t target = state->power.target_pct;
        uint8_t attempt = state->power.retry_count;
        state->power.motor_moving = true;
        unlock();
        
        ESP_LOGI(TAG, "Retrying power command (attempt %u) to %u%%",
                 attempt, target);
        
        // Send DIAG to PC so dashboard can log it
        char diag[128];
        snprintf(diag, sizeof(diag),
                 "DIAG,power_mismatch,target=%u,actual=%.1f,retry=%u\n",
                 target, fresh_pct, attempt);
        lan_client_send_message(diag);
        
        esp_err_t ret = o3_power_set(target);
        
        lock();
        state->power.motor_moving = false;
        state->power.last_command_ms = get_uptime_ms();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Retry failed: %s", esp_err_to_name(ret));
        }
        unlock();
        return;
    } else {
        // Within tolerance — clear error state
        if (state->power.error_mismatch) {
            ESP_LOGI(TAG, "Power mismatch resolved: actual=%.1f%% (target=%u%%)",
                     fresh_pct, state->power.target_pct);
            
            char diag[128];
            snprintf(diag, sizeof(diag),
                     "DIAG,power_resolved,target=%u,actual=%.1f\n",
                     state->power.target_pct, fresh_pct);
            unlock();
            lan_client_send_message(diag);
            lock();
        }
        state->power.error_mismatch = false;
        state->power.retry_count = 0;
    }
    
    unlock();
}

esp_err_t blocksi_state_start_calibration(void)
{
    lock();
    if (s_state_mgr.state.calibration.active) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    
    s_state_mgr.state.calibration.active = true;
    s_state_mgr.state.calibration.stop_requested = false;
    s_state_mgr.state.calibration.current_step_pct = 0;
    s_state_mgr.state.calibration.direction = 1;
    s_state_mgr.state.calibration.points_collected = 0;
    s_state_mgr.state.calibration.start_time_ms = get_uptime_ms();
    unlock();
    
    ESP_LOGI(TAG, "Calibration started");
    return ESP_OK;
}

void blocksi_state_stop_calibration(void)
{
    lock();
    s_state_mgr.state.calibration.stop_requested = true;
    unlock();
    ESP_LOGI(TAG, "Calibration stop requested");
}
