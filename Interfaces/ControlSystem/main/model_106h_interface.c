/**
 * @file model_106h_interface.c
 * @brief Implementation of 2B Technologies Model 106-H interface
 * 
 * Features:
 * - Bidirectional UART communication
 * - Response-waiting for menu commands
 * - Timeout-based prompt detection (handles "menu>" without line endings)
 */

#include "model_106h_interface.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "106H";

// RX buffer and parsing
#define RX_BUF_SIZE     256
#define LINE_BUF_SIZE   128
#define RESPONSE_BUF_SIZE 256

// Timeout for partial line flush (to catch prompts like "menu>")
#define PARTIAL_LINE_TIMEOUT_MS  300

// Module state
static struct {
    bool initialized;
    uart_port_t uart_num;
    m106h_sample_callback_t callback;
    m106h_log_data_callback_t log_callback;
    TaskHandle_t rx_task;
    SemaphoreHandle_t tx_mutex;
    
    // State tracking
    m106h_state_t state;
    bool logging_active;
    bool receiving_log_data;
    m106h_avg_time_t current_avg;
    
    // Response handling
    char response_buf[RESPONSE_BUF_SIZE];
    SemaphoreHandle_t response_sem;
    bool response_ready;
    
    // Statistics
    uint32_t total_samples;
    uint32_t parse_errors;
    
    // RX line buffer
    char line_buf[LINE_BUF_SIZE];
    int line_pos;
    TickType_t last_rx_tick;  // For partial line timeout
} s_106h = {0};

/**
 * @brief Parse a data line from the 106-H
 * 
 * Format: ozone,temp,pressure,sample_pdv,ref_pdv,DD/MM/YY,HH:MM:SS
 * Example: 1.03,31.7,835.9,1.28888,0.49086,01/03/17,18:40:55
 */
static bool parse_data_line(const char *line, m106h_sample_t *sample)
{
    int day, month, year, hour, minute, second;
    int parsed = sscanf(line, "%f,%f,%f,%f,%f,%d/%d/%d,%d:%d:%d",
                        &sample->ozone_wt_pct,
                        &sample->temperature_c,
                        &sample->pressure_mbar,
                        &sample->ref_pdv_v,
                        &sample->sample_pdv_v,
                        &day, &month, &year,
                        &hour, &minute, &second);
    
    if (parsed >= 7) {
        sample->day = (uint8_t)day;
        sample->month = (uint8_t)month;
        sample->year = (uint8_t)year;
        sample->hour = (uint8_t)hour;
        sample->minute = (uint8_t)minute;
        sample->second = (uint8_t)second;
        return true;
    }
    
    return false;
}

/**
 * @brief Store response and signal waiting task
 */
static void store_response(const char *response)
{
    strncpy(s_106h.response_buf, response, RESPONSE_BUF_SIZE - 1);
    s_106h.response_buf[RESPONSE_BUF_SIZE - 1] = '\0';
    s_106h.response_ready = true;
    xSemaphoreGive(s_106h.response_sem);
    ESP_LOGI(TAG, "RX response: \"%s\"", s_106h.response_buf);
}

/**
 * @brief Process a complete line (or partial line on timeout)
 */
static void process_line(const char *line)
{
    if (strlen(line) == 0) {
        return;
    }
    
    // Check for log transmission messages
    if (strncmp(line, "Logged Data", 11) == 0) {
        ESP_LOGI(TAG, "Log data transmission starting");
        s_106h.receiving_log_data = true;
        store_response(line);
        return;
    }
    
    if (strncmp(line, "End of Logged Data", 18) == 0) {
        ESP_LOGI(TAG, "Log data transmission complete");
        s_106h.receiving_log_data = false;
        if (s_106h.log_callback) {
            s_106h.log_callback(NULL, true);
        }
        store_response(line);
        return;
    }
    
    // Check for logging started/stopped messages
    if (strncmp(line, "Logging Started", 15) == 0) {
        ESP_LOGI(TAG, "106-H internal logging started");
        s_106h.logging_active = true;
        store_response(line);
        return;
    }
    
    if (strncmp(line, "Logging Ended", 13) == 0 || 
        strncmp(line, "Logging Stopped", 15) == 0) {
        ESP_LOGI(TAG, "106-H internal logging stopped");
        s_106h.logging_active = false;
        store_response(line);
        return;
    }
    
    // Check for menu prompt
    if (strstr(line, "menu>") != NULL || strstr(line, "menu >") != NULL) {
        ESP_LOGI(TAG, "106-H menu prompt detected");
        s_106h.state = M106H_STATE_IN_MENU;
        store_response(line);
        return;
    }
    
    // Check for averaging menu responses (lines containing averaging times)
    if (strstr(line, "2 sec") != NULL || strstr(line, "10 sec") != NULL ||
        strstr(line, "1 min") != NULL || strstr(line, "5 min") != NULL ||
        strstr(line, "1 hr") != NULL || strstr(line, "Averaging") != NULL) {
        store_response(line);
        return;
    }
    
    // If receiving log data, forward to callback
    if (s_106h.receiving_log_data) {
        if (s_106h.log_callback) {
            s_106h.log_callback(line, false);
        }
        return;
    }
    
    // Try to parse as data line
    m106h_sample_t sample = {0};
    if (parse_data_line(line, &sample)) {
        s_106h.total_samples++;
        s_106h.state = M106H_STATE_MEASURING;
        
        if (s_106h.callback) {
            s_106h.callback(&sample);
        }
    } else {
        // Not a data line - treat as command response
        store_response(line);
    }
}

/**
 * @brief UART RX task with timeout-based partial line handling
 */
static void rx_task(void *arg)
{
    uint8_t rx_buf[RX_BUF_SIZE];
    
    ESP_LOGI(TAG, "RX task started");
    s_106h.last_rx_tick = xTaskGetTickCount();
    
    while (1) {
        int len = uart_read_bytes(s_106h.uart_num, rx_buf, RX_BUF_SIZE - 1, 
                                   pdMS_TO_TICKS(50));  // Short timeout for responsiveness
        
        if (len > 0) {
            s_106h.last_rx_tick = xTaskGetTickCount();
            
            // Process received bytes
            for (int i = 0; i < len; i++) {
                char c = rx_buf[i];
                
                if (c == '\n' || c == '\r') {
                    if (s_106h.line_pos > 0) {
                        s_106h.line_buf[s_106h.line_pos] = '\0';
                        process_line(s_106h.line_buf);
                        s_106h.line_pos = 0;
                    }
                } else if (s_106h.line_pos < LINE_BUF_SIZE - 1) {
                    s_106h.line_buf[s_106h.line_pos++] = c;
                }
            }
        } else {
            // No data received - check for partial line timeout
            // This catches prompts like "menu>" that don't end with \r\n
            if (s_106h.line_pos > 0) {
                TickType_t elapsed = xTaskGetTickCount() - s_106h.last_rx_tick;
                if (elapsed >= pdMS_TO_TICKS(PARTIAL_LINE_TIMEOUT_MS)) {
                    // Timeout - flush partial line as response
                    s_106h.line_buf[s_106h.line_pos] = '\0';
                    ESP_LOGD(TAG, "Partial line timeout, flushing: \"%s\"", s_106h.line_buf);
                    process_line(s_106h.line_buf);
                    s_106h.line_pos = 0;
                }
            }
        }
    }
}

// ============================================================================
// Public API - Initialization
// ============================================================================

esp_err_t m106h_init(const m106h_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_106h.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing 106-H interface");
    ESP_LOGI(TAG, "UART%d, TX=%d, RX=%d, %d baud",
             config->uart_num, config->tx_gpio, config->rx_gpio, config->baud_rate);
    
    // Store config
    s_106h.uart_num = config->uart_num;
    s_106h.callback = config->callback;
    s_106h.state = M106H_STATE_MEASURING;
    s_106h.current_avg = M106H_AVG_10SEC;  // Default
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(config->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_num, 
                                  config->tx_gpio, config->rx_gpio,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(config->uart_num, 
                                         RX_BUF_SIZE * 2, RX_BUF_SIZE * 2,
                                         0, NULL, 0));
    
    // Create TX mutex
    s_106h.tx_mutex = xSemaphoreCreateMutex();
    if (!s_106h.tx_mutex) {
        uart_driver_delete(config->uart_num);
        return ESP_ERR_NO_MEM;
    }
    
    // Create response semaphore
    s_106h.response_sem = xSemaphoreCreateBinary();
    if (!s_106h.response_sem) {
        vSemaphoreDelete(s_106h.tx_mutex);
        uart_driver_delete(config->uart_num);
        return ESP_ERR_NO_MEM;
    }
    
    // Start RX task
    BaseType_t ret = xTaskCreate(rx_task, "106h_rx", 4096, NULL, 10, &s_106h.rx_task);
    if (ret != pdPASS) {
        vSemaphoreDelete(s_106h.response_sem);
        vSemaphoreDelete(s_106h.tx_mutex);
        uart_driver_delete(config->uart_num);
        return ESP_FAIL;
    }
    
    s_106h.initialized = true;
    ESP_LOGI(TAG, "106-H interface initialized");
    
    return ESP_OK;
}

void m106h_deinit(void)
{
    if (!s_106h.initialized) {
        return;
    }
    
    if (s_106h.rx_task) {
        vTaskDelete(s_106h.rx_task);
        s_106h.rx_task = NULL;
    }
    
    if (s_106h.response_sem) {
        vSemaphoreDelete(s_106h.response_sem);
        s_106h.response_sem = NULL;
    }
    
    if (s_106h.tx_mutex) {
        vSemaphoreDelete(s_106h.tx_mutex);
        s_106h.tx_mutex = NULL;
    }
    
    uart_driver_delete(s_106h.uart_num);
    s_106h.initialized = false;
    
    ESP_LOGI(TAG, "106-H interface deinitialized");
}

void m106h_get_stats(uint32_t *total_samples, uint32_t *parse_errors)
{
    if (total_samples) *total_samples = s_106h.total_samples;
    if (parse_errors) *parse_errors = s_106h.parse_errors;
}

m106h_state_t m106h_get_state(void)
{
    return s_106h.state;
}

// ============================================================================
// Response Waiting
// ============================================================================

/**
 * @brief Clear any pending response
 */
static void clear_response(void)
{
    s_106h.response_ready = false;
    s_106h.response_buf[0] = '\0';
    // Drain any pending semaphore gives
    xSemaphoreTake(s_106h.response_sem, 0);
}

/**
 * @brief Wait for a response from the 106-H
 * 
 * @param timeout_ms Maximum time to wait
 * @param response Output buffer for response (can be NULL)
 * @param response_size Size of response buffer
 * @return ESP_OK if response received, ESP_ERR_TIMEOUT if timed out
 */
esp_err_t m106h_wait_for_response(uint32_t timeout_ms, char *response, size_t response_size)
{
    if (xSemaphoreTake(s_106h.response_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (response && response_size > 0) {
            strncpy(response, s_106h.response_buf, response_size - 1);
            response[response_size - 1] = '\0';
        }
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "Response timeout after %lu ms", (unsigned long)timeout_ms);
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Get last response without waiting
 */
const char* m106h_get_last_response(void)
{
    return s_106h.response_buf;
}

// ============================================================================
// Public API - TX Commands
// ============================================================================

esp_err_t m106h_send_char(char c)
{
    if (!s_106h.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_106h.tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    int written = uart_write_bytes(s_106h.uart_num, &c, 1);
    xSemaphoreGive(s_106h.tx_mutex);
    
    if (written != 1) {
        ESP_LOGE(TAG, "TX failed");
        return ESP_FAIL;
    }
    
    // Wait for TX to complete
    uart_wait_tx_done(s_106h.uart_num, pdMS_TO_TICKS(100));
    
    if (c >= 0x20 && c < 0x7F) {
        ESP_LOGI(TAG, "TX: '%c' (0x%02X)", c, (unsigned char)c);
    } else {
        ESP_LOGI(TAG, "TX: 0x%02X", (unsigned char)c);
    }
    return ESP_OK;
}

esp_err_t m106h_send_string(const char *str)
{
    if (!s_106h.initialized || !str) {
        return ESP_ERR_INVALID_STATE;
    }
    
    size_t len = strlen(str);
    if (len == 0) {
        return ESP_OK;
    }
    
    if (xSemaphoreTake(s_106h.tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    int written = uart_write_bytes(s_106h.uart_num, str, len);
    xSemaphoreGive(s_106h.tx_mutex);
    
    if (written != (int)len) {
        ESP_LOGE(TAG, "TX failed");
        return ESP_FAIL;
    }
    
    uart_wait_tx_done(s_106h.uart_num, pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "TX string: \"%s\"", str);
    return ESP_OK;
}

esp_err_t m106h_cmd_header(void)
{
    ESP_LOGI(TAG, "Requesting header");
    return m106h_send_char('h');
}

esp_err_t m106h_cmd_enter_menu(void)
{
    ESP_LOGW(TAG, "Entering menu (measurements will stop)");
    s_106h.state = M106H_STATE_IN_MENU;
    return m106h_send_char('m');
}

esp_err_t m106h_cmd_exit_menu(void)
{
    ESP_LOGI(TAG, "Exiting menu (resuming measurements)");
    esp_err_t ret = m106h_send_char('x');
    if (ret == ESP_OK) {
        s_106h.state = M106H_STATE_MEASURING;
    }
    return ret;
}

esp_err_t m106h_cmd_zero(void)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        ESP_LOGW(TAG, "Not in menu mode, enter menu first");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Performing zero calibration");
    return m106h_send_char('V');
}

esp_err_t m106h_cmd_get_averaging(void)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Getting averaging options");
    return m106h_send_char('a');
}

esp_err_t m106h_cmd_set_averaging(uint8_t selection)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "%u\r", (unsigned)selection);
    ESP_LOGI(TAG, "Setting averaging to option %u", (unsigned)selection);
    return m106h_send_string(cmd);
}

esp_err_t m106h_cmd_serial_number(void)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Requesting serial number");
    return m106h_send_char('n');
}

esp_err_t m106h_cmd_set_date(uint8_t day, uint8_t month, uint8_t year)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = m106h_send_char('c');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = m106h_send_char('d');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    char date_str[16];
    snprintf(date_str, sizeof(date_str), "%02u%02u%02u\r", 
             (unsigned)day, (unsigned)month, (unsigned)year);
    ESP_LOGI(TAG, "Setting date: %02u/%02u/%02u", 
             (unsigned)day, (unsigned)month, (unsigned)year);
    
    return m106h_send_string(date_str);
}

esp_err_t m106h_cmd_set_time(uint8_t hour, uint8_t minute, uint8_t second)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = m106h_send_char('c');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = m106h_send_char('t');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02u%02u%02u\r", 
             (unsigned)hour, (unsigned)minute, (unsigned)second);
    ESP_LOGI(TAG, "Setting time: %02u:%02u:%02u", 
             (unsigned)hour, (unsigned)minute, (unsigned)second);
    
    return m106h_send_string(time_str);
}

esp_err_t m106h_cmd_lamp_test(void)
{
    if (s_106h.state != M106H_STATE_IN_MENU) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting lamp test");
    return m106h_send_char('p');
}

esp_err_t m106h_cmd_lamp_test_exit(void)
{
    ESP_LOGI(TAG, "Exiting lamp test");
    return m106h_send_char(0x1B);  // ESC
}

esp_err_t m106h_execute_menu_command(char cmd_char, const char *arg, uint32_t delay_ms)
{
    esp_err_t ret;
    
    // Enter menu
    ret = m106h_cmd_enter_menu();
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Send command
    ret = m106h_send_char(cmd_char);
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Send argument if provided
    if (arg) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        ret = m106h_send_string(arg);
        if (ret != ESP_OK) {
            m106h_cmd_exit_menu();
            return ret;
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Exit menu
    ret = m106h_cmd_exit_menu();
    
    return ret;
}

// ============================================================================
// Logging Control Functions
// ============================================================================

esp_err_t m106h_log_start(void)
{
    ESP_LOGI(TAG, "Starting 106-H internal logging");
    s_106h.logging_active = true;
    return m106h_send_char('l');
}

esp_err_t m106h_log_stop(void)
{
    ESP_LOGI(TAG, "Stopping 106-H internal logging");
    esp_err_t ret = m106h_send_char('e');
    if (ret == ESP_OK) {
        s_106h.logging_active = false;
    }
    return ret;
}

esp_err_t m106h_log_transmit(void)
{
    ESP_LOGI(TAG, "Requesting logged data transmission");
    s_106h.receiving_log_data = false;
    return m106h_send_char('t');
}

bool m106h_is_logging(void)
{
    return s_106h.logging_active;
}

// ============================================================================
// High-Level Averaging Control (with response waiting)
// ============================================================================

esp_err_t m106h_set_averaging(m106h_avg_time_t avg_time)
{
    if (avg_time < M106H_AVG_2SEC || avg_time > M106H_AVG_1HR) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const char *avg_names[] = {"", "2 sec", "10 sec", "1 min", "5 min", "1 hr"};
    ESP_LOGI(TAG, "=== Setting averaging time to %s ===", avg_names[avg_time]);
    
    esp_err_t ret;
    char response[128];
    
    // Clear any pending responses
    clear_response();
    
    // Step 1: Enter menu
    ESP_LOGI(TAG, "Step 1: Entering menu...");
    ret = m106h_cmd_enter_menu();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send menu command");
        return ret;
    }
    
    // Wait for "menu>" prompt (up to 3 seconds)
    ret = m106h_wait_for_response(3000, response, sizeof(response));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No menu prompt received, continuing anyway...");
    } else {
        ESP_LOGI(TAG, "Menu response: %s", response);
    }
    
    // Step 2: Request averaging submenu
    clear_response();
    ESP_LOGI(TAG, "Step 2: Requesting averaging submenu...");
    ret = m106h_send_char('a');
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Wait for averaging options (up to 2 seconds)
    ret = m106h_wait_for_response(2000, response, sizeof(response));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No averaging options received, continuing anyway...");
    } else {
        ESP_LOGI(TAG, "Averaging options response: %s", response);
    }
    
    // Step 3: Send selection
    clear_response();
    ESP_LOGI(TAG, "Step 3: Selecting option %u...", (unsigned)avg_time);
    char sel_str[8];
    snprintf(sel_str, sizeof(sel_str), "%u\r", (unsigned)avg_time);
    ret = m106h_send_string(sel_str);
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Wait for confirmation (up to 2 seconds)
    ret = m106h_wait_for_response(2000, response, sizeof(response));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Selection response: %s", response);
    }
    
    // Step 4: Exit menu
    clear_response();
    ESP_LOGI(TAG, "Step 4: Exiting menu...");
    ret = m106h_cmd_exit_menu();
    if (ret == ESP_OK) {
        s_106h.current_avg = avg_time;
        ESP_LOGI(TAG, "=== Averaging set successfully ===");
    }
    
    return ret;
}

m106h_avg_time_t m106h_get_averaging(void)
{
    return s_106h.current_avg;
}

void m106h_set_log_callback(m106h_log_data_callback_t callback)
{
    s_106h.log_callback = callback;
}
