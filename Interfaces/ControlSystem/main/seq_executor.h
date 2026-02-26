/**
 * @file seq_executor.h
 * @brief Generic recipe-based sequence step executor
 *
 * Architecture: "ESP32 = Arms, PC = Brains"
 * - PC sends full recipe (steps, prompts, parameters)
 * - ESP32 executes steps with precise sample-counted holds
 * - ESP32 streams per-sample data back to PC
 * - ESP32 does NOT analyze data, fit models, or decide pass/fail
 *
 * Recipe loading protocol:
 *   CMD,sequence_start,<type>,<key=value params>     Begin recipe
 *   CMD,seq_prompt,<before_step>,<prompt_id>,<text>   Add prompt
 *   CMD,seq_step,<index>,<power_pct>,<hold_samples>,<phase>  Add step
 *   CMD,seq_run                                       Finalize & execute
 *
 * Runtime streaming (ESP32 → PC):
 *   SEQ,<type>,STARTED,steps=<N>,flow=<X>
 *   SEQ,<type>,STEP,<index>,<power>,<phase>
 *   SEQ,<type>,SAMPLE,<step_idx>,<sample_num>,<o3_pct>,<temp_c>,<power_actual>
 *   SEQ,<type>,PROMPT,<prompt_id>,<prompt_text>
 *   SEQ,<type>,COMPLETE,<elapsed_s>
 *   SEQ,<type>,ABORTED,<reason>
 *
 * Runtime control:
 *   CMD,sequence_confirm                              Confirm pending prompt
 *   CMD,sequence_abort[,<reason>]                     Abort sequence
 *   CMD,sequence_status                               Query state
 */

#ifndef SEQ_EXECUTOR_H
#define SEQ_EXECUTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Limits ─────────────────────────────────────────────────────── */

#define SEQ_EXEC_MAX_STEPS      256     /**< Max steps in a recipe */
#define SEQ_EXEC_MAX_PROMPTS    16      /**< Max prompts in a recipe */
#define SEQ_EXEC_PHASE_LEN      24      /**< Max phase label length */
#define SEQ_EXEC_PROMPT_ID_LEN  32      /**< Max prompt ID length */
#define SEQ_EXEC_PROMPT_TXT_LEN 160     /**< Max prompt text length */
#define SEQ_EXEC_TYPE_LEN       24      /**< Max sequence type string */
#define SEQ_EXEC_PARAMS_LEN     128     /**< Max parameter string */

/* ── Step definition ────────────────────────────────────────────── */

typedef struct {
    uint16_t index;                             /**< Step ordering index */
    uint8_t  power_pct;                         /**< Power level 0-100 */
    uint16_t hold_samples;                      /**< Sensor samples to hold */
    char     phase[SEQ_EXEC_PHASE_LEN];         /**< Phase label */
} seq_exec_step_t;

/* ── Prompt definition ──────────────────────────────────────────── */

typedef struct {
    uint16_t before_step;                       /**< Issue before this step index */
    char     id[SEQ_EXEC_PROMPT_ID_LEN];        /**< ID for PC to match */
    char     text[SEQ_EXEC_PROMPT_TXT_LEN];     /**< CLI-friendly fallback text */
    bool     consumed;                          /**< Already issued */
} seq_exec_prompt_t;

/* ── Executor state ─────────────────────────────────────────────── */

typedef enum {
    SEQ_EXEC_IDLE = 0,          /**< No recipe loaded */
    SEQ_EXEC_LOADING,           /**< Receiving steps/prompts */
    SEQ_EXEC_RUNNING,           /**< Executing steps */
    SEQ_EXEC_WAITING_CONFIRM,   /**< Blocked on prompt confirmation */
    SEQ_EXEC_COMPLETE,          /**< Finished successfully */
    SEQ_EXEC_ABORTED,           /**< Aborted by user or error */
} seq_exec_state_t;

/* ── Public API ─────────────────────────────────────────────────── */

/**
 * @brief Initialize the executor (call once at boot)
 */
esp_err_t seq_executor_init(void);

/**
 * @brief Begin loading a new recipe
 *
 * Clears previous recipe. Transitions to LOADING state.
 *
 * @param type   Sequence type (e.g. "calibrate", "validate")
 * @param params Raw key=value parameter string (e.g. "flow=4.0")
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if currently running
 */
esp_err_t seq_executor_begin(const char *type, const char *params);

/**
 * @brief Add a step to the recipe being loaded
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not loading, ESP_ERR_NO_MEM if full
 */
esp_err_t seq_executor_add_step(uint16_t index, uint8_t power_pct,
                                uint16_t hold_samples, const char *phase);

/**
 * @brief Add a prompt to the recipe being loaded
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not loading, ESP_ERR_NO_MEM if full
 */
esp_err_t seq_executor_add_prompt(uint16_t before_step, const char *id,
                                  const char *text);

/**
 * @brief Finalize recipe, sort steps, and start execution task
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not loading, ESP_ERR_INVALID_ARG if no steps
 */
esp_err_t seq_executor_run(void);

/**
 * @brief Provide user confirmation for a pending prompt
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE if not waiting for confirmation
 */
esp_err_t seq_executor_confirm(void);

/**
 * @brief Abort the running sequence
 *
 * @param reason Human-readable reason (may be NULL)
 */
esp_err_t seq_executor_abort(const char *reason);

/**
 * @brief Get current state
 */
seq_exec_state_t seq_executor_get_state(void);

/**
 * @brief Get current type name (empty if idle)
 */
const char *seq_executor_get_type(void);

/**
 * @brief Get current step index (-1 if not running)
 */
int seq_executor_get_current_step(void);

/**
 * @brief Get total step count in loaded recipe
 */
int seq_executor_get_step_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SEQ_EXECUTOR_H */
