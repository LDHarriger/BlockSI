/**
 * @file seq_executor.c
 * @brief Generic recipe-based sequence step executor
 *
 * Architecture: "ESP32 = Arms, PC = Brains"
 *
 * The PC sends a complete recipe (steps + prompts + params) and the ESP32
 * executes it with precise sample-counted holds, streaming per-sample
 * data for the PC to record and analyze.
 *
 * The ESP32 does NOT:
 *   - Generate random values or step lists
 *   - Analyze data or compute statistics
 *   - Store or query models
 *   - Determine pass/fail
 */

#include "seq_executor.h"
#include "seq_sensor_adapter.h"
#include "sequence_runner.h"
#include "o3_power_control.h"
#include "relay_control.h"
#include "blocksi_state.h"
#include "lan_client.h"
#include "motor_pot.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "SEQ_EXEC";

/* ── Internal state ─────────────────────────────────────────────── */

static struct {
    /* Recipe metadata */
    char type[SEQ_EXEC_TYPE_LEN];
    char params[SEQ_EXEC_PARAMS_LEN];
    float flow_lpm;                         /* Parsed from params */

    /* Steps */
    seq_exec_step_t steps[SEQ_EXEC_MAX_STEPS];
    uint16_t        step_count;

    /* Prompts */
    seq_exec_prompt_t prompts[SEQ_EXEC_MAX_PROMPTS];
    uint16_t          prompt_count;

    /* Execution state */
    seq_exec_state_t  state;
    int               current_step;         /* -1 if not running */
    int64_t           start_time_us;

    /* Task + synchronization */
    TaskHandle_t      task_handle;
    SemaphoreHandle_t confirm_sem;          /* Posted on sequence_confirm */
    SemaphoreHandle_t mutex;                /* Protects state transitions */

    /* Abort */
    volatile bool     abort_requested;
    char              abort_reason[64];
} s_exec;

/* ── Forward declarations ───────────────────────────────────────── */

static void  executor_task(void *arg);
static int   step_compare(const void *a, const void *b);
static void  issue_prompts_before_step(uint16_t step_index);
static void  send_msg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static float parse_param_float(const char *params, const char *key, float def);
static void  safe_power_off(void);
static float get_elapsed_s(void);

/* ── Public API ─────────────────────────────────────────────────── */

esp_err_t seq_executor_init(void)
{
    if (!s_exec.confirm_sem) {
        s_exec.confirm_sem = xSemaphoreCreateBinary();
    }
    if (!s_exec.mutex) {
        s_exec.mutex = xSemaphoreCreateMutex();
    }
    if (!s_exec.confirm_sem || !s_exec.mutex) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return ESP_ERR_NO_MEM;
    }
    s_exec.state = SEQ_EXEC_IDLE;
    s_exec.current_step = -1;
    s_exec.task_handle = NULL;
    ESP_LOGI(TAG, "Executor initialized");
    return ESP_OK;
}

esp_err_t seq_executor_begin(const char *type, const char *params)
{
    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    if (s_exec.state == SEQ_EXEC_RUNNING ||
        s_exec.state == SEQ_EXEC_WAITING_CONFIRM) {
        xSemaphoreGive(s_exec.mutex);
        ESP_LOGE(TAG, "Cannot begin: sequence active");
        return ESP_ERR_INVALID_STATE;
    }

    /* Clear previous recipe */
    memset(s_exec.steps, 0, sizeof(s_exec.steps));
    memset(s_exec.prompts, 0, sizeof(s_exec.prompts));
    s_exec.step_count = 0;
    s_exec.prompt_count = 0;
    s_exec.current_step = -1;
    s_exec.abort_requested = false;
    s_exec.abort_reason[0] = '\0';
    s_exec.start_time_us = 0;

    /* Store metadata */
    strncpy(s_exec.type, type ? type : "unknown", SEQ_EXEC_TYPE_LEN - 1);
    s_exec.type[SEQ_EXEC_TYPE_LEN - 1] = '\0';

    if (params) {
        strncpy(s_exec.params, params, SEQ_EXEC_PARAMS_LEN - 1);
        s_exec.params[SEQ_EXEC_PARAMS_LEN - 1] = '\0';
    } else {
        s_exec.params[0] = '\0';
    }

    s_exec.flow_lpm = parse_param_float(s_exec.params, "flow", 4.0f);
    s_exec.state = SEQ_EXEC_LOADING;

    xSemaphoreGive(s_exec.mutex);
    ESP_LOGI(TAG, "Recipe begin: type=%s params=%s", s_exec.type, s_exec.params);
    return ESP_OK;
}

esp_err_t seq_executor_add_step(uint16_t index, uint8_t power_pct,
                                uint16_t hold_samples, const char *phase,
                                bool air_comp)
{
    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    if (s_exec.state != SEQ_EXEC_LOADING) {
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_exec.step_count >= SEQ_EXEC_MAX_STEPS) {
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_NO_MEM;
    }

    seq_exec_step_t *s = &s_exec.steps[s_exec.step_count++];
    s->index = index;
    s->power_pct = power_pct;
    s->hold_samples = hold_samples;
    s->air_comp = air_comp;
    strncpy(s->phase, phase ? phase : "", SEQ_EXEC_PHASE_LEN - 1);
    s->phase[SEQ_EXEC_PHASE_LEN - 1] = '\0';

    xSemaphoreGive(s_exec.mutex);
    return ESP_OK;
}

esp_err_t seq_executor_add_prompt(uint16_t before_step, const char *id,
                                  const char *text)
{
    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    if (s_exec.state != SEQ_EXEC_LOADING) {
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_exec.prompt_count >= SEQ_EXEC_MAX_PROMPTS) {
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_NO_MEM;
    }

    seq_exec_prompt_t *p = &s_exec.prompts[s_exec.prompt_count++];
    p->before_step = before_step;
    p->consumed = false;
    strncpy(p->id, id ? id : "", SEQ_EXEC_PROMPT_ID_LEN - 1);
    p->id[SEQ_EXEC_PROMPT_ID_LEN - 1] = '\0';
    strncpy(p->text, text ? text : "", SEQ_EXEC_PROMPT_TXT_LEN - 1);
    p->text[SEQ_EXEC_PROMPT_TXT_LEN - 1] = '\0';

    xSemaphoreGive(s_exec.mutex);
    return ESP_OK;
}

esp_err_t seq_executor_run(void)
{
    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    if (s_exec.state != SEQ_EXEC_LOADING) {
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_exec.step_count == 0) {
        s_exec.state = SEQ_EXEC_IDLE;
        xSemaphoreGive(s_exec.mutex);
        return ESP_ERR_INVALID_ARG;
    }

    /* Sort steps by index */
    qsort(s_exec.steps, s_exec.step_count, sizeof(seq_exec_step_t), step_compare);

    /* Clear stale confirm */
    xSemaphoreTake(s_exec.confirm_sem, 0);

    ESP_LOGI(TAG, "Recipe ready: %d steps, %d prompts, flow=%.1f LPM",
             s_exec.step_count, s_exec.prompt_count, s_exec.flow_lpm);

    /* Create executor task (8KB — generous for snprintf buffers) */
    BaseType_t ret = xTaskCreate(executor_task, "seq_exec", 8192, NULL, 5,
                                 &s_exec.task_handle);
    if (ret != pdPASS) {
        s_exec.state = SEQ_EXEC_IDLE;
        xSemaphoreGive(s_exec.mutex);
        ESP_LOGE(TAG, "Failed to create executor task");
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_exec.mutex);
    return ESP_OK;
}

esp_err_t seq_executor_confirm(void)
{
    if (s_exec.state != SEQ_EXEC_WAITING_CONFIRM) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_exec.confirm_sem);
    return ESP_OK;
}

esp_err_t seq_executor_abort(const char *reason)
{
    if (s_exec.state == SEQ_EXEC_LOADING) {
        s_exec.state = SEQ_EXEC_IDLE;
        return ESP_OK;
    }
    if (s_exec.state != SEQ_EXEC_RUNNING &&
        s_exec.state != SEQ_EXEC_WAITING_CONFIRM) {
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(s_exec.abort_reason, reason ? reason : "user",
            sizeof(s_exec.abort_reason) - 1);
    s_exec.abort_reason[sizeof(s_exec.abort_reason) - 1] = '\0';
    s_exec.abort_requested = true;

    /* Unblock prompt wait */
    xSemaphoreGive(s_exec.confirm_sem);

    ESP_LOGW(TAG, "Abort requested: %s", s_exec.abort_reason);
    return ESP_OK;
}

seq_exec_state_t seq_executor_get_state(void) { return s_exec.state; }
const char *seq_executor_get_type(void)       { return s_exec.type; }
int seq_executor_get_current_step(void)       { return s_exec.current_step; }
int seq_executor_get_step_count(void)         { return (int)s_exec.step_count; }

/* ── Executor task ──────────────────────────────────────────────── */

static void executor_task(void *arg)
{
    (void)arg;

    s_exec.state = SEQ_EXEC_RUNNING;
    s_exec.start_time_us = esp_timer_get_time();

    /* Activate sequence_runner lockout (for UI power-control disable) */
    sequence_runner_force_active(s_exec.type);
    seq_report_progress("starting", 0.0f, 0, false);

    send_msg("SEQ,%s,STARTED,steps=%d,flow=%.1f",
             s_exec.type, s_exec.step_count, s_exec.flow_lpm);

    ESP_LOGI(TAG, "=== EXECUTOR START: %s, %d steps ===",
             s_exec.type, s_exec.step_count);

    /* ── Execute each step ────────────────────────────────────── */

    bool air_comp_active = false;   /* Track air compressor state for cleanup */

    for (uint16_t i = 0; i < s_exec.step_count; i++) {
        if (s_exec.abort_requested) break;

        seq_exec_step_t *step = &s_exec.steps[i];
        s_exec.current_step = (int)i;

        /* Issue any prompts scheduled before this step */
        issue_prompts_before_step(step->index);
        if (s_exec.abort_requested) break;

        /* ── Air compressor relay control ─────────────────────── */
        if (step->air_comp != air_comp_active) {
            relay_set_with_source(RELAY_AIR_COMP,
                                  step->air_comp ? RELAY_ON : RELAY_OFF,
                                  RELAY_SRC_SEQUENCE);
            air_comp_active = step->air_comp;
            ESP_LOGI(TAG, "Air compressor → %s", air_comp_active ? "ON" : "OFF");
            /* Allow flow to stabilize after air compressor toggle */
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* Notify step transition */
        float progress = (float)i / (float)s_exec.step_count * 100.0f;
        seq_report_progress(step->phase, progress, step->power_pct, false);
        send_msg("SEQ,%s,STEP,%d,%d,%s,%d",
                 s_exec.type, step->index, step->power_pct, step->phase,
                 step->air_comp ? 1 : 0);

        ESP_LOGI(TAG, "Step %d/%d: pwr=%d%% hold=%d phase=%s air=%d",
                 i + 1, s_exec.step_count, step->power_pct,
                 step->hold_samples, step->phase, step->air_comp ? 1 : 0);

        /* Set power level */
        o3_power_set(step->power_pct);

        /* Wait for motor to settle (~500ms typical) */
        vTaskDelay(pdMS_TO_TICKS(500));

        /* ── Sample-counted hold ──────────────────────────────── */

        uint16_t collected = 0;
        uint32_t last_count = seq_sensor_get_sample_count();

        while (collected < step->hold_samples) {
            if (s_exec.abort_requested) break;

            uint32_t now_count = seq_sensor_get_sample_count();
            if (now_count > last_count) {
                /* New sample(s) arrived from 106-H */
                uint32_t new_samples = now_count - last_count;
                last_count = now_count;

                for (uint32_t n = 0; n < new_samples && collected < step->hold_samples; n++) {
                    /* Read current sensor values from unified state */
                    const blocksi_state_t *state = blocksi_state_get();
                    float o3_pct = state ? state->sensors.vessel_o3_pct : 0.0f;
                    float temp_c = state ? state->sensors.cell_temp_c : 0.0f;
                    float actual_pct = o3_power_get_percent();

                    /* Stream sample to PC (with air_comp state) */
                    send_msg("SEQ,%s,SAMPLE,%d,%d,%.4f,%.1f,%.1f,%d",
                             s_exec.type, step->index, collected,
                             o3_pct, temp_c, actual_pct,
                             step->air_comp ? 1 : 0);

                    collected++;
                }

                /* Update progress within step */
                float step_progress = (float)i / (float)s_exec.step_count +
                                      ((float)collected / (float)step->hold_samples)
                                      / (float)s_exec.step_count;
                seq_report_progress(step->phase, step_progress * 100.0f,
                                    step->power_pct, false);
            }

            vTaskDelay(pdMS_TO_TICKS(100));  /* Poll at 10 Hz */
        }
    }

    /* ── Cleanup ─────────────────────────────────────────────── */

    safe_power_off();

    float elapsed = get_elapsed_s();

    if (s_exec.abort_requested) {
        s_exec.state = SEQ_EXEC_ABORTED;
        send_msg("SEQ,%s,ABORTED,%s", s_exec.type, s_exec.abort_reason);
        seq_report_progress("aborted", 100.0f, 0, false);
        ESP_LOGW(TAG, "=== EXECUTOR ABORTED: %s (%.1fs) ===",
                 s_exec.type, elapsed);
    } else {
        s_exec.state = SEQ_EXEC_COMPLETE;
        send_msg("SEQ,%s,COMPLETE,%.1f", s_exec.type, elapsed);
        seq_report_progress("complete", 100.0f, 0, false);
        ESP_LOGI(TAG, "=== EXECUTOR COMPLETE: %s (%.1fs) ===",
                 s_exec.type, elapsed);
    }

    s_exec.current_step = -1;
    s_exec.task_handle = NULL;

    /* Release sequence_runner lockout */
    sequence_runner_force_idle();

    vTaskDelete(NULL);
}

/* ── Prompt handling ────────────────────────────────────────────── */

static void issue_prompts_before_step(uint16_t step_index)
{
    for (uint16_t p = 0; p < s_exec.prompt_count; p++) {
        seq_exec_prompt_t *prompt = &s_exec.prompts[p];

        if (prompt->consumed || prompt->before_step != step_index) continue;

        /* Send prompt to PC */
        send_msg("SEQ,%s,PROMPT,%s,%s", s_exec.type, prompt->id, prompt->text);
        ESP_LOGI(TAG, "Prompt [%s]: %s", prompt->id, prompt->text);

        s_exec.state = SEQ_EXEC_WAITING_CONFIRM;

        /* Block until confirm or abort */
        while (true) {
            if (xSemaphoreTake(s_exec.confirm_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
                break;  /* Confirmed (or abort unblocked us) */
            }
            /* Still waiting — keep the connection alive */
        }

        prompt->consumed = true;

        if (s_exec.abort_requested) return;

        s_exec.state = SEQ_EXEC_RUNNING;
        ESP_LOGI(TAG, "Prompt [%s] confirmed", prompt->id);
    }
}

/* ── Helpers ────────────────────────────────────────────────────── */

static int step_compare(const void *a, const void *b)
{
    return (int)((const seq_exec_step_t *)a)->index -
           (int)((const seq_exec_step_t *)b)->index;
}

static void send_msg(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

#ifdef CONFIG_LAN_CLIENT_ENABLED
    lan_client_send_message(buf);
#endif
}

static float parse_param_float(const char *params, const char *key, float def)
{
    if (!params || !key) return def;
    char search[32];
    snprintf(search, sizeof(search), "%s=", key);
    const char *found = strstr(params, search);
    if (!found) return def;
    return strtof(found + strlen(search), NULL);
}

static void safe_power_off(void)
{
    o3_power_set(0);
    /* Ensure air compressor is OFF after sequence */
    relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);
    ESP_LOGI(TAG, "Safe shutdown: power → 0%%, air_comp → OFF");
}

static float get_elapsed_s(void)
{
    if (s_exec.start_time_us == 0) return 0.0f;
    return (float)(esp_timer_get_time() - s_exec.start_time_us) / 1000000.0f;
}
