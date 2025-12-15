/**
 * @file model_106h_interface.c
 * @brief Implementation of 2B Technologies Model 106-H interface
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
    
    // Statistics
    uint32_t total_samples;
    uint32_t parse_errors;
    
    // RX line buffer
    char line_buf[LINE_BUF_SIZE];
    int line_pos;
} s_106h = {0};

/**
 * @brief Parse a data line from the 106-H
 * 
 * Format: ozone,temp,pressure,ref_pdv,sample_pdv,DD/MM/YY,HH:MM:SS
 * Example: 1.03,31.7,835.9,1.28888,0.49086,01/03/17,18:40:55
 */
static bool parse_data_line(const char *line, m106h_sample_t *sample)
{
    // Parse CSV fields
    int day, month, year, hour, minute, second;
    int parsed = sscanf(line, "%f,%f,%f,%f,%f,%d/%d/%d,%d:%d:%d",
                        &sample->ozone_wt_pct,
                        &sample->temperature_c,
                        &sample->pressure_mbar,
                        &sample->ref_pdv_v,
                        &sample->sample_pdv_v,
                        &day, &month, &year,
                        &hour, &minute, &second);
    
    if (parsed != 11) {
        return false;
    }
    
    sample->day = (uint8_t)day;
    sample->month = (uint8_t)month;
    sample->year = (uint8_t)year;
    sample->hour = (uint8_t)hour;
    sample->minute = (uint8_t)minute;
    sample->second = (uint8_t)second;
    
    return true;
}

/**
 * @brief Process a complete line
 */
static void process_line(const char *line)
{
    // Skip empty lines
    if (line[0] == '\0') {
        return;
    }
    
    // Check for menu prompt
    if (strncmp(line, "menu>", 5) == 0) {
        s_106h.state = M106H_STATE_IN_MENU;
        return;
    }
    
    // Check for log transmission messages
    if (strncmp(line, "Logged Data", 11) == 0) {
        ESP_LOGI(TAG, "Log data transmission starting");
        s_106h.receiving_log_data = true;
        return;
    }
    
    if (strncmp(line, "End of Logged Data", 18) == 0) {
        ESP_LOGI(TAG, "Log data transmission complete");
        s_106h.receiving_log_data = false;
        if (s_106h.log_callback) {
            s_106h.log_callback(NULL, true);  // Signal end
        }
        return;
    }
    
    // Check for logging started/stopped messages
    if (strncmp(line, "Logging Started", 15) == 0) {
        ESP_LOGI(TAG, "106-H internal logging started");
        s_106h.logging_active = true;
        return;
    }
    
    if (strncmp(line, "Logging Ended", 13) == 0 || 
        strncmp(line, "Logging Stopped", 15) == 0) {
        ESP_LOGI(TAG, "106-H internal logging stopped");
        s_106h.logging_active = false;
        return;
    }
    
    // If receiving log data, forward to log callback
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
        // Could be a response to a command, log it
        ESP_LOGI(TAG, "RX (response): %s", line);
    }
}

/**
 * @brief UART RX task
 */
static void rx_task(void *arg)
{
    uint8_t rx_buf[RX_BUF_SIZE];
    
    ESP_LOGI(TAG, "RX task started");
    
    while (1) {
        int len = uart_read_bytes(s_106h.uart_num, rx_buf, RX_BUF_SIZE - 1, 
                                   pdMS_TO_TICKS(100));
        
        if (len > 0) {
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
    
    // Start RX task
    BaseType_t ret = xTaskCreate(rx_task, "106h_rx", 4096, NULL, 10, &s_106h.rx_task);
    if (ret != pdPASS) {
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
    
    ESP_LOGI(TAG, "TX: '%c' (0x%02X) -> UART%d", c, (unsigned char)c, s_106h.uart_num);
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
    
    ESP_LOGD(TAG, "TX: \"%s\"", str);
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
    
    // Enter clock menu
    esp_err_t ret = m106h_send_char('c');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Select date
    ret = m106h_send_char('d');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Send date in DDMMYY format
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
    
    // Enter clock menu
    esp_err_t ret = m106h_send_char('c');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Select time
    ret = m106h_send_char('t');
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Send time in HHMMSS format
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

esp_err_t m106h_menu_command(char cmd, uint32_t delay_ms)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Menu command '%c' with %lums delay", cmd, (unsigned long)delay_ms);
    
    // Enter menu
    ret = m106h_cmd_enter_menu();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for menu prompt
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Send command
    ret = m106h_send_char(cmd);
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Wait for command to complete
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
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
    s_106h.receiving_log_data = false;  // Will be set true when "Logged Data" received
    return m106h_send_char('t');
}

bool m106h_is_logging(void)
{
    return s_106h.logging_active;
}

esp_err_t m106h_set_averaging(m106h_avg_time_t avg_time)
{
    if (avg_time < M106H_AVG_2SEC || avg_time > M106H_AVG_1HR) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const char *avg_names[] = {"", "2 sec", "10 sec", "1 min", "5 min", "1 hr"};
    ESP_LOGI(TAG, "Setting averaging time to %s", avg_names[avg_time]);
    
    // Enter menu - the 106-H needs time to stop measuring and show menu
    esp_err_t ret = m106h_cmd_enter_menu();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for 106-H to fully enter menu mode and display "menu>"
    // This is critical - the 106-H is slow to respond
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Request averaging menu
    ESP_LOGI(TAG, "Requesting averaging submenu");
    ret = m106h_send_char('a');
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Wait for averaging options to display
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Send selection digit followed by carriage return
    ESP_LOGI(TAG, "Selecting option %u", (unsigned)avg_time);
    char sel = '0' + avg_time;  // Convert 1-5 to '1'-'5'
    ret = m106h_send_char(sel);
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Send carriage return to confirm selection
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = m106h_send_char('\r');
    if (ret != ESP_OK) {
        m106h_cmd_exit_menu();
        return ret;
    }
    
    // Wait for 106-H to process selection and return to main menu
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Exit menu
    ret = m106h_cmd_exit_menu();
    if (ret == ESP_OK) {
        s_106h.current_avg = avg_time;
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
