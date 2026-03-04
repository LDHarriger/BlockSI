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

/*
 * Relay prerequisite: -1 = don't change, 0 = force OFF, 1 = force ON
 * Parsed from sequence_start params: relay_o2=1, relay_o3=1, relay_air=0
 */
typedef struct {
    int8_t o2_conc;     /**< O2 concentrator */
    int8_t ozone_gen;   /**< O3 generator (MP-8000) */
    int8_t air_comp;    /**< Air compressor (internal to MP-8000, requires ozone_gen ON) */
} seq_relay_prereqs_t;

static struct {
    /* Recipe metadata */
    char type[SEQ_EXEC_TYPE_LEN];
    char params[SEQ_EXEC_PARAMS_LEN];
    float flow_lpm;                         /* Parsed from params */

    /* Relay prerequisites (from sequence_start params) */
    seq_relay_prereqs_t relay_prereqs;
    /* Original relay states saved before sequence (for restore on abort) */
    relay_state_t orig_o2_conc;
    relay_state_t orig_ozone_gen;
    relay_state_t orig_air_comp;

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
static int   parse_param_int(const char *params, const char *key, int def);
static void  apply_relay_prereqs(void);
static void  restore_relays(void);
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

    /* Parse relay prerequisites: -1=don't change, 0=OFF, 1=ON */
    s_exec.relay_prereqs.o2_conc   = (int8_t)parse_param_int(s_exec.params, "relay_o2",  -1);
    s_exec.relay_prereqs.ozone_gen = (int8_t)parse_param_int(s_exec.params, "relay_o3",  -1);
    s_exec.relay_prereqs.air_comp  = (int8_t)parse_param_int(s_exec.params, "relay_air", -1);

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

    /* ── Apply relay prerequisites (O2, O3, air) before power steps ── */
    apply_relay_prereqs();
    if (s_exec.abort_requested) goto cleanup;

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

cleanup:
    restore_relays();

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

static int parse_param_int(const char *params, const char *key, int def)
{
    if (!params || !key) return def;
    char search[32];
    snprintf(search, sizeof(search), "%s=", key);
    const char *found = strstr(params, search);
    if (!found) return def;
    return atoi(found + strlen(search));
}

/**
 * @brief Save current relay states and apply prerequisites before sequence
 *
 * Order matters due to hardware dependency:
 *   1. O2 concentrator (independent)
 *   2. O3 generator (independent, but air_comp requires it ON)
 *   3. Air compressor (internal to MP-8000, needs ozone_gen ON first)
 *
 * Uses relay_set_with_source() which enforces the air_comp→ozone_gen
 * interlock at the driver level.
 */
static void apply_relay_prereqs(void)
{
    /* Save original states for potential restore on abort */
    s_exec.orig_o2_conc   = relay_get_state(RELAY_O2_CONC);
    s_exec.orig_ozone_gen = relay_get_state(RELAY_OZONE_GEN);
    s_exec.orig_air_comp  = relay_get_state(RELAY_AIR_COMP);

    ESP_LOGI(TAG, "Relay state before sequence: o2=%d o3=%d air=%d",
             s_exec.orig_o2_conc, s_exec.orig_ozone_gen, s_exec.orig_air_comp);

    bool changed = false;

    /* Apply in safe order: O2 → O3 → Air (air needs O3 power) */

    if (s_exec.relay_prereqs.o2_conc >= 0) {
        relay_state_t target = s_exec.relay_prereqs.o2_conc ? RELAY_ON : RELAY_OFF;
        if (relay_get_state(RELAY_O2_CONC) != target) {
            relay_set_with_source(RELAY_O2_CONC, target, RELAY_SRC_SEQUENCE);
            send_msg("SEQ,%s,RELAY,o2_conc,%d", s_exec.type, target);
            ESP_LOGI(TAG, "Prereq: o2_conc → %s", target ? "ON" : "OFF");
            changed = true;
        }
    }

    if (s_exec.relay_prereqs.ozone_gen >= 0) {
        relay_state_t target = s_exec.relay_prereqs.ozone_gen ? RELAY_ON : RELAY_OFF;
        if (relay_get_state(RELAY_OZONE_GEN) != target) {
            relay_set_with_source(RELAY_OZONE_GEN, target, RELAY_SRC_SEQUENCE);
            send_msg("SEQ,%s,RELAY,ozone_gen,%d", s_exec.type, target);
            ESP_LOGI(TAG, "Prereq: ozone_gen → %s", target ? "ON" : "OFF");
            changed = true;
        }
    }

    if (s_exec.relay_prereqs.air_comp >= 0) {
        relay_state_t target = s_exec.relay_prereqs.air_comp ? RELAY_ON : RELAY_OFF;
        if (relay_get_state(RELAY_AIR_COMP) != target) {
            esp_err_t ret = relay_set_with_source(RELAY_AIR_COMP, target, RELAY_SRC_SEQUENCE);
            if (ret == ESP_OK) {
                send_msg("SEQ,%s,RELAY,air_comp,%d", s_exec.type, target);
                ESP_LOGI(TAG, "Prereq: air_comp → %s", target ? "ON" : "OFF");
                changed = true;
            } else {
                /* Interlock rejected (ozone_gen not ON) — logged by relay_control */
                ESP_LOGW(TAG, "Prereq: air_comp ON rejected (ozone_gen interlock)");
            }
        }
    }

    /* Allow relays and equipment to warm up / stabilize */
    if (changed) {
        ESP_LOGI(TAG, "Waiting 3s for relay/equipment stabilization...");
        send_msg("SEQ,%s,STATUS,relay_stabilizing", s_exec.type);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    ESP_LOGI(TAG, "Relay prerequisites applied");
}

/**
 * @brief Restore relays to safe state on sequence end
 *
 * Normal completion: power=0, air_comp=OFF, O2/O3 left in current state
 *                    (user may want to keep using the system)
 * Abort:            power=0, air_comp=OFF, ozone_gen=OFF (safety), O2 left on
 *
 * Order: air_comp OFF first (before ozone_gen, though killing ozone_gen
 * also kills air power — keep software state clean).
 */
static void restore_relays(void)
{
    /* Always kill power first */
    o3_power_set(0);

    /* Always turn off air compressor (it's inside the MP-8000) */
    if (relay_get_state(RELAY_AIR_COMP) == RELAY_ON) {
        relay_set_with_source(RELAY_AIR_COMP, RELAY_OFF, RELAY_SRC_SEQUENCE);
        send_msg("SEQ,%s,RELAY,air_comp,0", s_exec.type);
    }

    if (s_exec.abort_requested) {
        /* On abort: turn off O3 generator for safety (this also kills air power) */
        if (relay_get_state(RELAY_OZONE_GEN) == RELAY_ON) {
            relay_set_with_source(RELAY_OZONE_GEN, RELAY_OFF, RELAY_SRC_SEQUENCE);
            send_msg("SEQ,%s,RELAY,ozone_gen,0", s_exec.type);
        }
        ESP_LOGW(TAG, "Abort cleanup: power=0, o3_gen=OFF, air=OFF, o2 unchanged");
    } else {
        /* Normal completion: leave o2 and o3 in their current state */
        ESP_LOGI(TAG, "Completion cleanup: power=0, air=OFF, o2/o3 unchanged");
    }
}

static float get_elapsed_s(void)
{
    if (s_exec.start_time_us == 0) return 0.0f;
    return (float)(esp_timer_get_time() - s_exec.start_time_us) / 1000000.0f;
}

/**
 * @brief Add a step directly to the step buffer (no mutex, no state check)
 *
 * Caller must hold the mutex and ensure state == LOADING.
 */
static void add_step_internal(uint16_t index, uint8_t power_pct,
                              uint16_t hold_samples, const char *phase,
                              bool air_comp)
{
    if (s_exec.step_count >= SEQ_EXEC_MAX_STEPS) return;
    seq_exec_step_t *s = &s_exec.steps[s_exec.step_count++];
    s->index = index;
    s->power_pct = power_pct;
    s->hold_samples = hold_samples;
    s->air_comp = air_comp;
    strncpy(s->phase, phase ? phase : "", SEQ_EXEC_PHASE_LEN - 1);
    s->phase[SEQ_EXEC_PHASE_LEN - 1] = '\0';
}

/* ── Built-in calibration recipe ────────────────────────────────── */

esp_err_t seq_executor_load_calibration(float flow_lpm, bool air_comp_on,
                                        const uint8_t *random_powers,
                                        int num_random)
{
    /*
     * Calibration sweep:
     *   Phase 1 — Baseline: 0% power, 15 samples (~37s)
     *   Phase 2 — Sweep Up: 0→100% in 1% steps, 2 samples each (~505s)
     *   Phase 3 — Sweep Down: 100→0% in 1% steps, 2 samples each (~505s)
     *   Phase 4 — Random: PC-generated levels, 20 samples each (optional)
     *
     * Total base: 203 steps + num_random random steps
     *
     * Relay prereqs: O2=ON if flow>0, O3=ON, Air per air_comp_on.
     * The executor applies these at run time.
     */
    char params[SEQ_EXEC_PARAMS_LEN];
    int relay_o2 = (flow_lpm > 0.1f) ? 1 : 0;
    int relay_air = air_comp_on ? 1 : 0;
    snprintf(params, sizeof(params),
             "flow=%.1f,relay_o2=%d,relay_o3=1,relay_air=%d",
             flow_lpm, relay_o2, relay_air);

    esp_err_t ret = seq_executor_begin("calibrate", params);
    if (ret != ESP_OK) return ret;

    /* Must hold mutex while populating steps (begin left state = LOADING) */
    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    uint16_t idx = 0;

    /* Phase 1 — Baseline */
    add_step_internal(idx++, 0, 15, "baseline", air_comp_on);

    /* Phase 2 — Sweep Up: 0 → 100% */
    for (int pwr = 0; pwr <= 100; pwr++) {
        add_step_internal(idx++, (uint8_t)pwr, 2, "sweep_up", air_comp_on);
    }

    /* Phase 3 — Sweep Down: 100 → 0% */
    for (int pwr = 100; pwr >= 0; pwr--) {
        add_step_internal(idx++, (uint8_t)pwr, 2, "sweep_down", air_comp_on);
    }

    /* Phase 4 — Random hold (PC-generated, 20 samples each) */
    if (random_powers != NULL && num_random > 0) {
        for (int i = 0; i < num_random && idx < SEQ_EXEC_MAX_STEPS; i++) {
            add_step_internal(idx++, random_powers[i], 20, "random", air_comp_on);
        }
    }

    ESP_LOGI(TAG, "Calibration recipe loaded: %d steps, flow=%.1f LPM, air=%d, random=%d",
             s_exec.step_count, flow_lpm, relay_air, num_random);

    xSemaphoreGive(s_exec.mutex);
    return ESP_OK;
}

/* ── Built-in validation recipe ─────────────────────────────────── */

esp_err_t seq_executor_load_validation(uint8_t power_pct, float flow_lpm)
{
    /*
     * Validation sequence:
     *   Phase 1 — Baseline: 0% power, 15 samples (~37s)
     *   Phase 2 — Spot Low: ~33% of target, 5 samples (~12s)
     *   Phase 3 — Spot High: ~66% of target, 5 samples (~12s)
     *   Phase 4 — Target: full power, 15 samples (~37s)
     *   Phase 5 — Cooldown: 0% power, 5 samples (~12s)
     *
     * Total: 5 steps, ~112s (~1.9 min)
     *
     * Relay prereqs: O2=ON, O3=ON, Air=OFF.
     * Prompts: check_flow before baseline, check_route before spot_low.
     */
    char params[SEQ_EXEC_PARAMS_LEN];
    snprintf(params, sizeof(params),
             "power=%u,flow=%.1f,relay_o2=1,relay_o3=1,relay_air=0",
             power_pct, flow_lpm);

    esp_err_t ret = seq_executor_begin("validate", params);
    if (ret != ESP_OK) return ret;

    xSemaphoreTake(s_exec.mutex, portMAX_DELAY);

    /* Spot check power levels */
    uint8_t spot1 = (uint8_t)(power_pct * 33 / 100);
    uint8_t spot2 = (uint8_t)(power_pct * 66 / 100);
    if (spot1 < 10) spot1 = 10;
    if (spot2 < 10) spot2 = 10;

    /* Steps */
    add_step_internal(0, 0,          15, "baseline",  false);
    add_step_internal(1, spot1,       5, "spot_low",  false);
    add_step_internal(2, spot2,       5, "spot_high", false);
    add_step_internal(3, power_pct,  15, "target",    false);
    add_step_internal(4, 0,           5, "cooldown",  false);

    xSemaphoreGive(s_exec.mutex);

    /* Prompts: check flow before baseline, check route before spot_low */
    seq_executor_add_prompt(0, "check_flow", "Verify O2 flow matches rotameter");
    seq_executor_add_prompt(1, "check_route", "Confirm gas route to 106-H sensor");

    ESP_LOGI(TAG, "Validation recipe loaded: 5 steps, power=%u%%, flow=%.1f LPM",
             power_pct, flow_lpm);
    return ESP_OK;
}
