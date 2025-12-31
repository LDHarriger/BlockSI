/**
 * @file model_106h_interface.h
 * @brief Interface for 2B Technologies Model 106-H Ozone Monitor
 * 
 * Handles bidirectional UART communication:
 * - RX: Parses incoming measurement data
 * - TX: Sends configuration commands
 * - Response waiting: Waits for 106-H acknowledgments
 */

#ifndef MODEL_106H_INTERFACE_H
#define MODEL_106H_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parsed sample data from 106-H
 */
typedef struct {
    float ozone_wt_pct;      // Ozone concentration in wt% O2
    float temperature_c;      // Cell temperature in Celsius
    float pressure_mbar;      // Cell pressure in mbar
    float sample_pdv_v;       // Sample photodiode voltage
    float ref_pdv_v;          // Reference photodiode voltage
    uint8_t day;
    uint8_t month;
    uint8_t year;             // 2-digit year
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} m106h_sample_t;

/**
 * @brief Callback for received samples
 */
typedef void (*m106h_sample_callback_t)(const m106h_sample_t *sample);

/**
 * @brief Callback for log data lines
 * 
 * @param line Log data line (NULL when transmission complete)
 * @param is_end True when transmission is complete
 */
typedef void (*m106h_log_data_callback_t)(const char *line, bool is_end);

/**
 * @brief Configuration structure
 */
typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;            // 2400, 4800, or 19200
    m106h_sample_callback_t callback;
} m106h_config_t;

/**
 * @brief 106-H menu state
 */
typedef enum {
    M106H_STATE_MEASURING,    // Normal measurement mode
    M106H_STATE_IN_MENU,      // Serial menu active (not measuring)
} m106h_state_t;

/**
 * @brief Averaging time options
 */
typedef enum {
    M106H_AVG_2SEC = 1,
    M106H_AVG_10SEC = 2,
    M106H_AVG_1MIN = 3,
    M106H_AVG_5MIN = 4,
    M106H_AVG_1HR = 5
} m106h_avg_time_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the 106-H interface
 * 
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t m106h_init(const m106h_config_t *config);

/**
 * @brief Deinitialize the interface
 */
void m106h_deinit(void);

/**
 * @brief Get statistics
 * 
 * @param total_samples Output: total samples received
 * @param parse_errors Output: number of parse errors
 */
void m106h_get_stats(uint32_t *total_samples, uint32_t *parse_errors);

/**
 * @brief Get current state (measuring vs menu)
 */
m106h_state_t m106h_get_state(void);

// ============================================================================
// Response Waiting
// ============================================================================

/**
 * @brief Wait for a response from the 106-H
 * 
 * Blocks until a response is received or timeout expires.
 * The RX task detects responses including prompts like "menu>" that
 * don't end with \r\n (using timeout-based partial line flushing).
 * 
 * @param timeout_ms Maximum time to wait in milliseconds
 * @param response Output buffer for response (can be NULL)
 * @param response_size Size of response buffer
 * @return ESP_OK if response received, ESP_ERR_TIMEOUT if timed out
 */
esp_err_t m106h_wait_for_response(uint32_t timeout_ms, char *response, size_t response_size);

/**
 * @brief Get last response without waiting
 * 
 * @return Pointer to internal response buffer (do not free)
 */
const char* m106h_get_last_response(void);

// ============================================================================
// TX Command Functions
// ============================================================================

/**
 * @brief Send raw character to 106-H
 * 
 * Use for single-character commands that work during measurement.
 * 
 * @param c Character to send
 * @return ESP_OK on success
 */
esp_err_t m106h_send_char(char c);

/**
 * @brief Send string to 106-H
 * 
 * @param str String to send (null-terminated)
 * @return ESP_OK on success
 */
esp_err_t m106h_send_string(const char *str);

/**
 * @brief Output header line (command: 'h')
 * 
 * Works during measurement. 106-H will output column headers.
 */
esp_err_t m106h_cmd_header(void);

/**
 * @brief Enter serial menu (command: 'm')
 * 
 * WARNING: This stops measurements until m106h_cmd_exit_menu() is called.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_enter_menu(void);

/**
 * @brief Exit serial menu and resume measuring (command: 'x')
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_exit_menu(void);

/**
 * @brief Perform zero calibration (command: 'V')
 * 
 * Must be in menu mode. Takes several seconds to complete.
 * Call m106h_cmd_enter_menu() first.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_zero(void);

/**
 * @brief Get available averaging times (command: 'a')
 * 
 * Must be in menu mode. Returns list of options.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_get_averaging(void);

/**
 * @brief Set averaging time (low-level)
 * 
 * Must be in menu mode. Send the selection number after calling
 * m106h_cmd_get_averaging().
 * 
 * @param selection Averaging selection (1-5 typically)
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_set_averaging(uint8_t selection);

/**
 * @brief Get instrument serial number (command: 'n')
 * 
 * Must be in menu mode.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_serial_number(void);

/**
 * @brief Set clock date (command sequence: 'c', 'd', DDMMYY, CR)
 * 
 * Must be in menu mode.
 * 
 * @param day Day (1-31)
 * @param month Month (1-12)
 * @param year Year (0-99, 2-digit)
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_set_date(uint8_t day, uint8_t month, uint8_t year);

/**
 * @brief Set clock time (command sequence: 'c', 't', HHMMSS, CR)
 * 
 * Must be in menu mode.
 * 
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_set_time(uint8_t hour, uint8_t minute, uint8_t second);

/**
 * @brief Perform lamp test (command: 'p')
 * 
 * Must be in menu mode. Send ESC (0x1B) to exit.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_cmd_lamp_test(void);

/**
 * @brief Cancel lamp test
 * 
 * Sends ESC character to exit lamp test.
 */
esp_err_t m106h_cmd_lamp_test_exit(void);

/**
 * @brief Execute menu command sequence safely
 * 
 * Enters menu, executes command, exits menu automatically.
 * Blocks until complete. Use for one-off commands.
 * 
 * @param cmd_char Command character to send
 * @param arg Optional argument string (NULL if none)
 * @param delay_ms Delay between commands
 * @return ESP_OK on success
 */
esp_err_t m106h_execute_menu_command(char cmd_char, const char *arg, uint32_t delay_ms);

// ============================================================================
// High-Level Functions (with response waiting)
// ============================================================================

/**
 * @brief Set averaging time with full menu navigation
 * 
 * Handles entire sequence: enter menu, navigate to averaging,
 * select option, exit menu. Waits for 106-H responses at each step.
 * 
 * @param avg_time Averaging time to set
 * @return ESP_OK on success
 */
esp_err_t m106h_set_averaging(m106h_avg_time_t avg_time);

/**
 * @brief Get current averaging time setting
 * 
 * Returns the last known setting (may not reflect manual changes).
 * 
 * @return Current averaging time
 */
m106h_avg_time_t m106h_get_averaging(void);

// ============================================================================
// Logging Control
// ============================================================================

/**
 * @brief Start internal data logging (command: 'l')
 * 
 * Works during measurement. Overwrites existing logged data.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_log_start(void);

/**
 * @brief Stop internal data logging (command: 'e')
 * 
 * Works during measurement.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_log_stop(void);

/**
 * @brief Request logged data transmission (command: 't')
 * 
 * Works during measurement. Use callback to receive data.
 * 
 * @return ESP_OK on success
 */
esp_err_t m106h_log_transmit(void);

/**
 * @brief Check if internal logging is active
 * 
 * @return true if logging is active
 */
bool m106h_is_logging(void);

/**
 * @brief Set callback for log data reception
 * 
 * @param callback Function to call for each log line
 */
void m106h_set_log_callback(m106h_log_data_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // MODEL_106H_INTERFACE_H
