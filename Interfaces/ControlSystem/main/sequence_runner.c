/**
 * @file sequence_runner.c
 * @brief Generic autonomous sequence framework implementation
 * 
 * Manages lifecycle of multi-phase sequences: registration, start/stop,
 * FreeRTOS task management, SEQ message streaming, and safe cleanup.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sequence_runner.h"
#include "lan_client.h"

static const char *TAG = "SEQ_RUN";

// =============================================================================
// Internal State
// =============================================================================

typedef struct {
    // Registry
    const sequence_impl_t *implementations[SEQ_MAX_IMPLEMENTATIONS];
    uint8_t impl_count;

    // Active sequence
    const sequence_impl_t *active_impl;
    sequence_state_t state;
    bool stop_requested;

    // Progress tracking
    char phase_name[SEQ_PHASE_NAME_MAX];
    float progress_pct;
    uint8_t power_pct;
    bool air_comp;
    int64_t start_time_us;          // esp_timer_get_time() at start
    int64_t last_seq_msg_us;        // Rate limiting for SEQ messages

    // Task management
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;

    // Parameters for current run
    char type_name[SEQ_TYPE_NAME_MAX];
    char params[SEQ_PARAMS_MAX];

    // Interactive prompt support
    QueueHandle_t confirm_queue;            // Receives confirmation values
    char pending_prompt_id[SEQ_PROMPT_ID_MAX]; // Currently waiting prompt ID
    bool prompt_pending;                    // True when waiting for confirmation
} seq_runner_state_t;

static seq_runner_state_t s_seq = {0};

// =============================================================================
// Internal Helpers
// =============================================================================

static int64_t get_time_us(void)
{
    return esp_timer_get_time();
}

static float get_elapsed_s(void)
{
    if (s_seq.start_time_us == 0) return 0.0f;
    return (float)(get_time_us() - s_seq.start_time_us) / 1000000.0f;
}

/**
 * @brief Send a SEQ status message over LAN
 */
static void send_seq_message(void)
{
    char msg[128];
    snprintf(msg, sizeof(msg), "SEQ,%s,%.1f,%u,%d,%.1f",
             s_seq.phase_name,
             s_seq.progress_pct,
             s_seq.power_pct,
             s_seq.air_comp ? 1 : 0,
             get_elapsed_s());

#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(msg);
#endif

    ESP_LOGD(TAG, "%s", msg);
}

/**
 * @brief Send a SEQ completion/error notification
 */
static void send_seq_complete(const char *result)
{
    char msg[128];
    snprintf(msg, sizeof(msg), "SEQ_DONE,%s,%s,%.1f,pts=%u",
             s_seq.type_name,
             result,
             get_elapsed_s(),
             0);  // Sequence implementations can override with their own message

#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(msg);
#endif
}

/**
 * @brief Find a registered implementation by type name
 */
static const sequence_impl_t* find_impl(const char *type_name)
{
    if (type_name == NULL) return NULL;

    for (uint8_t i = 0; i < s_seq.impl_count; i++) {
        if (strcmp(s_seq.implementations[i]->type_name, type_name) == 0) {
            return s_seq.implementations[i];
        }
    }
    return NULL;
}

// =============================================================================
// Sequence Execution Task
// =============================================================================

static void sequence_task(void *pvParameters)
{
    const sequence_impl_t *impl = s_seq.active_impl;

    ESP_LOGI(TAG, "=== SEQUENCE START: %s (%s) ===", impl->type_name, impl->description);
    ESP_LOGI(TAG, "Params: \"%s\"", s_seq.params);

    // Initialize progress
    s_seq.start_time_us = get_time_us();
    s_seq.last_seq_msg_us = 0;
    s_seq.progress_pct = 0.0f;
    s_seq.power_pct = 0;
    s_seq.air_comp = false;
    strncpy(s_seq.phase_name, "starting", SEQ_PHASE_NAME_MAX - 1);

    // Send initial SEQ message
    send_seq_message();

    // Execute the sequence
    esp_err_t result = impl->execute();

    // Determine outcome
    const char *result_str;
    if (s_seq.stop_requested) {
        s_seq.state = SEQ_STATE_COMPLETE;  // Clean stop counts as complete
        result_str = "aborted";
        ESP_LOGW(TAG, "=== SEQUENCE ABORTED: %s (%.1fs) ===",
                 impl->type_name, get_elapsed_s());
    } else if (result == ESP_OK) {
        s_seq.state = SEQ_STATE_COMPLETE;
        result_str = "ok";
        ESP_LOGI(TAG, "=== SEQUENCE COMPLETE: %s (%.1fs) ===",
                 impl->type_name, get_elapsed_s());
    } else {
        s_seq.state = SEQ_STATE_ERROR;
        result_str = "error";
        ESP_LOGE(TAG, "=== SEQUENCE ERROR: %s (%.1fs, err=%d) ===",
                 impl->type_name, get_elapsed_s(), result);
    }

    // Cleanup — always called
    if (impl->cleanup) {
        ESP_LOGI(TAG, "Running cleanup for %s", impl->type_name);
        impl->cleanup();
    }

    // Update final progress
    strncpy(s_seq.phase_name, result_str, SEQ_PHASE_NAME_MAX - 1);
    send_seq_message();
    send_seq_complete(result_str);

    // Reset runner state
    xSemaphoreTake(s_seq.mutex, portMAX_DELAY);
    s_seq.active_impl = NULL;
    s_seq.stop_requested = false;
    s_seq.task_handle = NULL;
    // Keep state as COMPLETE or ERROR until next start
    xSemaphoreGive(s_seq.mutex);

    vTaskDelete(NULL);
}

// =============================================================================
// Public API — Framework Management
// =============================================================================

esp_err_t sequence_runner_init(void)
{
    if (s_seq.mutex == NULL) {
        s_seq.mutex = xSemaphoreCreateMutex();
        if (s_seq.mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_seq.confirm_queue == NULL) {
        // Queue depth 1: only one prompt outstanding at a time
        s_seq.confirm_queue = xQueueCreate(1, SEQ_PROMPT_VALUE_MAX);
        if (s_seq.confirm_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create confirm queue");
            return ESP_ERR_NO_MEM;
        }
    }

    s_seq.state = SEQ_STATE_IDLE;
    s_seq.impl_count = 0;
    s_seq.active_impl = NULL;
    s_seq.task_handle = NULL;
    s_seq.stop_requested = false;
    s_seq.prompt_pending = false;
    s_seq.pending_prompt_id[0] = '\0';

    ESP_LOGI(TAG, "Sequence runner initialized");
    return ESP_OK;
}

esp_err_t sequence_runner_register(const sequence_impl_t *impl)
{
    if (impl == NULL || impl->type_name == NULL || impl->execute == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_seq.impl_count >= SEQ_MAX_IMPLEMENTATIONS) {
        ESP_LOGE(TAG, "Registry full (%d max)", SEQ_MAX_IMPLEMENTATIONS);
        return ESP_ERR_NO_MEM;
    }

    // Check for duplicate
    if (find_impl(impl->type_name) != NULL) {
        ESP_LOGW(TAG, "Duplicate type '%s', replacing", impl->type_name);
        for (uint8_t i = 0; i < s_seq.impl_count; i++) {
            if (strcmp(s_seq.implementations[i]->type_name, impl->type_name) == 0) {
                s_seq.implementations[i] = impl;
                return ESP_OK;
            }
        }
    }

    s_seq.implementations[s_seq.impl_count++] = impl;
    ESP_LOGI(TAG, "Registered sequence: %s (%s)", impl->type_name, impl->description);
    return ESP_OK;
}

esp_err_t sequence_runner_start(const char *type_name, const char *params)
{
    if (s_seq.mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Find implementation
    const sequence_impl_t *impl = find_impl(type_name);
    if (impl == NULL) {
        ESP_LOGE(TAG, "Unknown sequence type: '%s'", type_name ? type_name : "(null)");
        return ESP_ERR_NOT_FOUND;
    }

    xSemaphoreTake(s_seq.mutex, portMAX_DELAY);

    // Check if already running
    if (s_seq.state == SEQ_STATE_RUNNING || s_seq.state == SEQ_STATE_STOPPING) {
        ESP_LOGE(TAG, "Cannot start '%s': another sequence is active (%s)",
                 type_name, s_seq.type_name);
        xSemaphoreGive(s_seq.mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare — validate params and set up internal state
    if (impl->prepare) {
        esp_err_t ret = impl->prepare(params);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Preparation failed for '%s': %s",
                     type_name, esp_err_to_name(ret));
            xSemaphoreGive(s_seq.mutex);
            return ret;
        }
    }

    // Store context
    strncpy(s_seq.type_name, type_name, SEQ_TYPE_NAME_MAX - 1);
    s_seq.type_name[SEQ_TYPE_NAME_MAX - 1] = '\0';
    if (params) {
        strncpy(s_seq.params, params, SEQ_PARAMS_MAX - 1);
        s_seq.params[SEQ_PARAMS_MAX - 1] = '\0';
    } else {
        s_seq.params[0] = '\0';
    }

    s_seq.active_impl = impl;
    s_seq.state = SEQ_STATE_RUNNING;
    s_seq.stop_requested = false;

    // Create execution task (6KB stack — sequences may use substantial local vars)
    BaseType_t ret = xTaskCreate(
        sequence_task,
        "seq_exec",
        6144,
        NULL,
        5,          // Same priority as old calibration task
        &s_seq.task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sequence task");
        s_seq.state = SEQ_STATE_ERROR;
        s_seq.active_impl = NULL;
        xSemaphoreGive(s_seq.mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_seq.mutex);

    ESP_LOGI(TAG, "Started sequence '%s' with params '%s'",
             type_name, params ? params : "");
    return ESP_OK;
}

esp_err_t sequence_runner_stop(void)
{
    xSemaphoreTake(s_seq.mutex, portMAX_DELAY);

    if (s_seq.state != SEQ_STATE_RUNNING) {
        xSemaphoreGive(s_seq.mutex);
        return ESP_ERR_INVALID_STATE;
    }

    s_seq.state = SEQ_STATE_STOPPING;
    s_seq.stop_requested = true;

    // Notify the sequence implementation
    if (s_seq.active_impl && s_seq.active_impl->request_stop) {
        s_seq.active_impl->request_stop();
    }

    xSemaphoreGive(s_seq.mutex);

    ESP_LOGW(TAG, "Stop requested for sequence '%s'", s_seq.type_name);
    return ESP_OK;
}

esp_err_t sequence_runner_get_status(sequence_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    status->state = s_seq.state;
    strncpy(status->type_name, s_seq.type_name, SEQ_TYPE_NAME_MAX - 1);
    status->type_name[SEQ_TYPE_NAME_MAX - 1] = '\0';
    strncpy(status->phase_name, s_seq.phase_name, SEQ_PHASE_NAME_MAX - 1);
    status->phase_name[SEQ_PHASE_NAME_MAX - 1] = '\0';
    status->progress_pct = s_seq.progress_pct;
    status->power_pct = s_seq.power_pct;
    status->air_comp = s_seq.air_comp;
    status->elapsed_s = get_elapsed_s();

    return ESP_OK;
}

bool sequence_runner_is_active(void)
{
    return (s_seq.state == SEQ_STATE_RUNNING || s_seq.state == SEQ_STATE_STOPPING);
}

// =============================================================================
// Helpers for Sequence Implementations
// =============================================================================

void seq_report_progress(const char *phase_name, float progress_pct,
                         uint8_t power_pct, bool air_comp)
{
    // Update stored state
    if (phase_name) {
        strncpy(s_seq.phase_name, phase_name, SEQ_PHASE_NAME_MAX - 1);
        s_seq.phase_name[SEQ_PHASE_NAME_MAX - 1] = '\0';
    }
    s_seq.progress_pct = progress_pct;
    s_seq.power_pct = power_pct;
    s_seq.air_comp = air_comp;

    // Rate-limit SEQ messages
    int64_t now = get_time_us();
    if ((now - s_seq.last_seq_msg_us) >= (SEQ_UPDATE_INTERVAL_MS * 1000LL)) {
        send_seq_message();
        s_seq.last_seq_msg_us = now;
    }
}

bool seq_check_stop(void)
{
    return s_seq.stop_requested;
}

void seq_send_data(const char *msg)
{
    if (msg == NULL) return;

#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(msg);
#endif

    // Also log at debug level
    ESP_LOGD(TAG, "DATA: %s", msg);
}

// =============================================================================
// Interactive Prompt Support
// =============================================================================

esp_err_t seq_prompt_user(const char *prompt_id, const char *message,
                          char *value_out, size_t max_len, uint32_t timeout_ms)
{
    if (prompt_id == NULL || message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check for stop before sending prompt
    if (seq_check_stop()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clear any stale confirmations from the queue
    char discard[SEQ_PROMPT_VALUE_MAX];
    while (xQueueReceive(s_seq.confirm_queue, discard, 0) == pdTRUE) {
        ESP_LOGW(TAG, "Discarded stale confirmation");
    }

    // Store the pending prompt ID and mark as waiting
    strncpy(s_seq.pending_prompt_id, prompt_id, SEQ_PROMPT_ID_MAX - 1);
    s_seq.pending_prompt_id[SEQ_PROMPT_ID_MAX - 1] = '\0';
    s_seq.prompt_pending = true;

    // Send prompt to PC: SEQ,prompt,<id>,<message>
    char msg_buf[256];
    snprintf(msg_buf, sizeof(msg_buf), "SEQ,prompt,%s,%s", prompt_id, message);
#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(msg_buf);
#endif
    ESP_LOGI(TAG, "Prompt [%s]: %s", prompt_id, message);

    // Also update the phase name to show waiting state
    seq_report_progress(prompt_id, s_seq.progress_pct, s_seq.power_pct, s_seq.air_comp);

    // Block waiting for confirmation
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    char received_value[SEQ_PROMPT_VALUE_MAX] = {0};

    // Poll in 1s increments so we can check stop_requested
    int64_t deadline_us = (timeout_ms == 0) ? INT64_MAX : 
                          esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (true) {
        // Check for stop request
        if (seq_check_stop()) {
            s_seq.prompt_pending = false;
            return ESP_ERR_INVALID_STATE;
        }

        // Try to receive with 1s timeout
        TickType_t wait = pdMS_TO_TICKS(1000);
        if (timeout_ms > 0) {
            int64_t remaining_us = deadline_us - esp_timer_get_time();
            if (remaining_us <= 0) {
                s_seq.prompt_pending = false;
                ESP_LOGW(TAG, "Prompt [%s] timed out", prompt_id);
                return ESP_ERR_TIMEOUT;
            }
            uint32_t remaining_ms = (uint32_t)(remaining_us / 1000);
            if (remaining_ms < 1000) {
                wait = pdMS_TO_TICKS(remaining_ms);
            }
        }

        if (xQueueReceive(s_seq.confirm_queue, received_value, wait) == pdTRUE) {
            // Got confirmation
            s_seq.prompt_pending = false;
            if (value_out && max_len > 0) {
                strncpy(value_out, received_value, max_len - 1);
                value_out[max_len - 1] = '\0';
            }
            ESP_LOGI(TAG, "Prompt [%s] confirmed: '%s'", prompt_id, received_value);
            return ESP_OK;
        }
    }
}

esp_err_t sequence_runner_provide_confirmation(const char *prompt_id, const char *value)
{
    if (!s_seq.prompt_pending) {
        ESP_LOGW(TAG, "Confirmation received but no prompt pending");
        return ESP_ERR_INVALID_STATE;
    }

    // Verify prompt ID matches
    if (prompt_id != NULL && s_seq.pending_prompt_id[0] != '\0') {
        if (strcmp(prompt_id, s_seq.pending_prompt_id) != 0) {
            ESP_LOGW(TAG, "Confirmation ID mismatch: expected '%s', got '%s'",
                     s_seq.pending_prompt_id, prompt_id);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Send value through queue
    char value_buf[SEQ_PROMPT_VALUE_MAX] = {0};
    if (value && strlen(value) > 0) {
        strncpy(value_buf, value, SEQ_PROMPT_VALUE_MAX - 1);
    }

    if (xQueueSend(s_seq.confirm_queue, value_buf, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue confirmation");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Confirmation provided for [%s]: '%s'", 
             prompt_id ? prompt_id : "?", value ? value : "");
    return ESP_OK;
}
