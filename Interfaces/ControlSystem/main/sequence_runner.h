/**
 * @file sequence_runner.h
 * @brief Generic autonomous sequence framework for BlockSI
 * 
 * Provides a framework for running multi-phase autonomous sequences on the
 * ESP32, such as power-O3 calibration, airflow validation, fill model
 * calibration, ozone decay testing, and sterilization cycles.
 * 
 * Design principles:
 * - ESP32 owns execution: deterministic timing, direct hardware access
 * - PC is observer: receives SEQ status lines, can abort via sequence_stop
 * - One sequence at a time (mutual exclusion)
 * - Safe cleanup on abort: power → 0%, appropriate relay state
 * - Normal DATA telemetry continues alongside SEQ messages
 * 
 * Architecture:
 *   sequence_runner.c       — Framework: lifecycle, SEQ streaming, LAN integration
 *   seq_power_cal.c         — Sequence #1: Power-O3 calibration
 *   (future) seq_validate.c — Sequence #2: Airflow/concentration validation
 *   (future) seq_fill.c     — Sequence #3: Fill model calibration
 *   (future) seq_decay.c    — Sequence #4: O3 decay testing
 *   (future) seq_sterilize.c — Sequence #5: Sterilize batch
 * 
 * LAN Commands:
 *   CMD,sequence_start,<type>,<params>  → RSP,OK,sequence_start,type=<type>
 *   CMD,sequence_stop                   → RSP,OK,sequence_stop,stopping
 *   CMD,sequence_status                 → RSP,OK,sequence_status,<status_fields>
 * 
 * SEQ Message Format (ESP32 → PC, periodic during sequence):
 *   SEQ,<phase_name>,<progress_pct>,<power_pct>,<air_comp>,<elapsed_s>\n
 */

#ifndef SEQUENCE_RUNNER_H
#define SEQUENCE_RUNNER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Constants
// =============================================================================

#define SEQ_MAX_IMPLEMENTATIONS     8       ///< Maximum registered sequence types
#define SEQ_TYPE_NAME_MAX           16      ///< Max length of type name string
#define SEQ_PHASE_NAME_MAX          24      ///< Max length of phase name string
#define SEQ_PARAMS_MAX              64      ///< Max length of params string
#define SEQ_UPDATE_INTERVAL_MS      1000    ///< How often SEQ messages are sent
#define SEQ_PROMPT_ID_MAX           24      ///< Max length of prompt ID string
#define SEQ_PROMPT_VALUE_MAX        64      ///< Max length of confirmation value

// =============================================================================
// Types
// =============================================================================

/**
 * @brief Sequence execution state
 */
typedef enum {
    SEQ_STATE_IDLE = 0,         ///< No sequence running
    SEQ_STATE_RUNNING,          ///< Sequence in progress
    SEQ_STATE_STOPPING,         ///< Stop requested, waiting for clean exit
    SEQ_STATE_COMPLETE,         ///< Sequence finished successfully
    SEQ_STATE_ERROR             ///< Sequence terminated with error
} sequence_state_t;

/**
 * @brief Status snapshot (returned by sequence_runner_get_status)
 */
typedef struct {
    sequence_state_t state;
    char type_name[SEQ_TYPE_NAME_MAX];      ///< Active sequence type (e.g., "cal")
    char phase_name[SEQ_PHASE_NAME_MAX];    ///< Current phase (e.g., "sweep_up")
    float progress_pct;                     ///< Overall progress 0–100
    uint8_t power_pct;                      ///< Current power setting
    bool air_comp;                          ///< Air compressor state
    float elapsed_s;                        ///< Total elapsed seconds
} sequence_status_t;

/**
 * @brief Sequence implementation interface
 * 
 * Each sequence type (calibration, sterilization, etc.) provides one of these.
 * The sequence_runner calls these methods at the appropriate times.
 */
typedef struct {
    /// Short name used in LAN commands (e.g., "cal", "validate", "sterilize")
    const char *type_name;

    /// Human-readable description for logging
    const char *description;

    /**
     * @brief Prepare sequence from parameter string
     * 
     * Called before task creation to validate params and set up internal state.
     * Params string is everything after the type name in the command, 
     * e.g., for "CMD,sequence_start,cal,4.0" params would be "4.0".
     * 
     * @param params Parameter string (may be NULL or empty)
     * @return ESP_OK if ready to run, error code otherwise
     */
    esp_err_t (*prepare)(const char *params);

    /**
     * @brief Execute the sequence (blocking)
     * 
     * Runs in the sequence runner's FreeRTOS task. Must:
     * - Call seq_check_stop() periodically and return early if true
     * - Call seq_report_progress() to update phase/progress/power
     * - Perform all hardware control directly
     * - Return ESP_OK on success, error on failure
     * 
     * @return ESP_OK on normal completion, ESP_ERR_* on failure
     */
    esp_err_t (*execute)(void);

    /**
     * @brief Request graceful stop
     * 
     * Called from a different task when user requests abort.
     * Should set internal flags so execute() returns at next check point.
     * Must NOT perform lengthy operations (called from LAN handler context).
     */
    void (*request_stop)(void);

    /**
     * @brief Clean up after execution
     * 
     * Called after execute() returns (whether success, error, or abort).
     * Must ensure safe hardware state: power → 0%, relays to appropriate state.
     */
    void (*cleanup)(void);
} sequence_impl_t;

// =============================================================================
// Framework API
// =============================================================================

/**
 * @brief Initialize the sequence runner framework
 * 
 * Creates internal mutex and resets state. Call once during app_main().
 * 
 * @return ESP_OK on success
 */
esp_err_t sequence_runner_init(void);

/**
 * @brief Register a sequence implementation
 * 
 * Call during init for each available sequence type.
 * 
 * @param impl Pointer to sequence implementation (must be static/persistent)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if registry full
 */
esp_err_t sequence_runner_register(const sequence_impl_t *impl);

/**
 * @brief Start a sequence by type name
 * 
 * Looks up the registered implementation, calls prepare(), then spawns
 * the execution task.
 * 
 * @param type_name Sequence type (e.g., "cal")
 * @param params Parameter string (e.g., "4.0" for O2 LPM)
 * @return ESP_OK if started, ESP_ERR_NOT_FOUND if unknown type,
 *         ESP_ERR_INVALID_STATE if another sequence is running
 */
esp_err_t sequence_runner_start(const char *type_name, const char *params);

/**
 * @brief Request graceful stop of the running sequence
 * 
 * @return ESP_OK if stop requested, ESP_ERR_INVALID_STATE if nothing running
 */
esp_err_t sequence_runner_stop(void);

/**
 * @brief Get current sequence status
 * 
 * @param status Output status snapshot
 * @return ESP_OK on success
 */
esp_err_t sequence_runner_get_status(sequence_status_t *status);

/**
 * @brief Check if any sequence is currently active
 * 
 * @return true if a sequence is running or stopping
 */
bool sequence_runner_is_active(void);

// =============================================================================
// Helpers for Sequence Implementations
// =============================================================================
// These are called from within a sequence's execute() function.

/**
 * @brief Report current phase and progress
 * 
 * Updates the status and sends a SEQ message over LAN if enough time
 * has passed since the last one (rate-limited to SEQ_UPDATE_INTERVAL_MS).
 * 
 * @param phase_name Current phase name (e.g., "sweep_up")
 * @param progress_pct Overall progress 0–100
 * @param power_pct Current power setting
 * @param air_comp Current air compressor state
 */
void seq_report_progress(const char *phase_name, float progress_pct,
                         uint8_t power_pct, bool air_comp);

/**
 * @brief Check if stop has been requested
 * 
 * Sequence implementations should call this frequently (e.g., every step)
 * and return early from execute() if true.
 * 
 * @return true if stop was requested
 */
bool seq_check_stop(void);

/**
 * @brief Send a sequence-specific data message over LAN
 * 
 * For sending data points that don't fit in the SEQ format
 * (e.g., CAL_DATA lines). The message is sent as-is with a newline appended.
 * 
 * @param msg Message string (without trailing newline)
 */
void seq_send_data(const char *msg);

// =============================================================================
// Interactive Prompt Support
// =============================================================================
// For sequences that need operator confirmation (e.g., valve routing).
// The sequence task calls seq_prompt_user() which blocks until the PC sends
// CMD,sequence_confirm,<prompt_id>[,<value>].

/**
 * @brief Prompt the user and wait for confirmation
 * 
 * Sends a SEQ,prompt,<prompt_id>,<message> to the PC dashboard, then blocks
 * until the operator responds via CMD,sequence_confirm,<prompt_id>[,<value>].
 * 
 * @param prompt_id   Short identifier (e.g., "route_vessel")
 * @param message     Human-readable prompt text
 * @param value_out   Buffer to receive operator's response value (may be empty)
 * @param max_len     Size of value_out buffer
 * @param timeout_ms  Timeout in ms (0 = infinite; use with caution)
 * @return ESP_OK on confirmation, ESP_ERR_TIMEOUT if timed out,
 *         ESP_ERR_INVALID_STATE if stop was requested
 */
esp_err_t seq_prompt_user(const char *prompt_id, const char *message,
                          char *value_out, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Provide a confirmation value from the LAN handler
 * 
 * Called from the LAN command handler when CMD,sequence_confirm arrives.
 * Unblocks the waiting seq_prompt_user() call.
 * 
 * @param prompt_id  The prompt ID being confirmed
 * @param value      Optional value string (may be NULL or empty)
 * @return ESP_OK if accepted, ESP_ERR_INVALID_STATE if no prompt pending
 */
esp_err_t sequence_runner_provide_confirmation(const char *prompt_id, const char *value);

// =============================================================================
// Bridge Functions for External Executors (seq_executor.c)
// =============================================================================
// These allow the new recipe-based executor to manage the sequence runner's
// lockout state without going through the full register/start/stop lifecycle.

/**
 * @brief Force the runner into active (RUNNING) state
 * 
 * Used by seq_executor so that sequence_runner_is_active() returns true
 * during recipe execution, providing UI lockout for power controls.
 * 
 * @param type  Type name to display in status queries
 * @return ESP_OK, ESP_ERR_INVALID_STATE if already running a registered sequence
 */
esp_err_t sequence_runner_force_active(const char *type);

/**
 * @brief Force the runner back to IDLE state
 * 
 * Called by seq_executor when recipe completes or aborts.
 */
void sequence_runner_force_idle(void);

#ifdef __cplusplus
}
#endif

#endif // SEQUENCE_RUNNER_H
