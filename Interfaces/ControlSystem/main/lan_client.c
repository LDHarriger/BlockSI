/**
 * @file lan_client.c
 * @brief Implementation of LAN TCP client
 */

#include "lan_client.h"
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LAN";

// Buffer sizes
#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 256
#define RX_LINE_SIZE   128

// Module state
static struct {
    bool initialized;
    bool connected;
    int sock;
    
    // Configuration
    char server_ip[16];
    uint16_t server_port;
    lan_command_handler_t cmd_handler;
    uint32_t reconnect_interval_ms;
    
    // Task handle
    TaskHandle_t task_handle;
    
    // Thread safety for sending
    SemaphoreHandle_t tx_mutex;
    
    // Statistics
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t reconnect_count;
    
    // Receive buffer
    char rx_buffer[RX_LINE_SIZE];
    int rx_pos;
} s_lan = {0};

/**
 * @brief Set socket to non-blocking mode
 */
static esp_err_t set_socket_nonblocking(int sock)
{
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        return ESP_FAIL;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Attempt to connect to PC server
 */
static esp_err_t connect_to_server(void)
{
    struct sockaddr_in server_addr;
    
    // Create socket
    s_lan.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_lan.sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    
    // Set up server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(s_lan.server_port);
    
    if (inet_pton(AF_INET, s_lan.server_ip, &server_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid server IP: %s", s_lan.server_ip);
        close(s_lan.sock);
        s_lan.sock = -1;
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Connecting to %s:%d...", s_lan.server_ip, s_lan.server_port);
    
    // Connect (blocking for simplicity - timeout handled by caller)
    if (connect(s_lan.sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGD(TAG, "Connection failed: errno %d", errno);
        close(s_lan.sock);
        s_lan.sock = -1;
        return ESP_FAIL;
    }
    
    // Set non-blocking for subsequent I/O
    if (set_socket_nonblocking(s_lan.sock) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set non-blocking mode");
    }
    
    // Set TCP keepalive
    int keepalive = 1;
    setsockopt(s_lan.sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    
    s_lan.connected = true;
    s_lan.rx_pos = 0;
    
    ESP_LOGI(TAG, "Connected to PC server");
    
    // Send hello message
    const char *hello = "HELLO,BlockSI,1.2.0\n";
    send(s_lan.sock, hello, strlen(hello), 0);
    
    return ESP_OK;
}

/**
 * @brief Close connection
 */
static void disconnect(void)
{
    if (s_lan.sock >= 0) {
        close(s_lan.sock);
        s_lan.sock = -1;
    }
    s_lan.connected = false;
    s_lan.rx_pos = 0;
}

/**
 * @brief Process a received line (command from PC)
 * 
 * @param line_buf Pointer to line buffer (must be RX_LINE_SIZE)
 */
static void process_rx_line(char *line_buf)
{
    // Trim trailing whitespace
    size_t len = strlen(line_buf);
    while (len > 0 && (line_buf[len-1] == '\r' || line_buf[len-1] == '\n' || line_buf[len-1] == ' ')) {
        line_buf[--len] = '\0';
    }
    
    if (len == 0) {
        return;
    }
    
    ESP_LOGI(TAG, "RX: %s", line_buf);
    s_lan.rx_count++;
    
    // Parse command: CMD,command,args...
    if (len < 5 || strncmp(line_buf, "CMD,", 4) != 0) {
        ESP_LOGW(TAG, "Invalid message format (expected CMD,...)");
        return;
    }
    
    // Extract command and args with explicit bounds
    char cmd_name[32];
    char args_buf[RX_LINE_SIZE];
    const char *args_ptr = NULL;
    
    // Find comma after command name
    const char *cmd_start = line_buf + 4;
    const char *comma = strchr(cmd_start, ',');
    
    if (comma != NULL) {
        // Calculate command length (bounded)
        size_t cmd_len = (size_t)(comma - cmd_start);
        if (cmd_len > sizeof(cmd_name) - 1) {
            cmd_len = sizeof(cmd_name) - 1;
        }
        memcpy(cmd_name, cmd_start, cmd_len);
        cmd_name[cmd_len] = '\0';
        
        // Copy args (bounded)
        const char *args_start = comma + 1;
        size_t args_len = strlen(args_start);
        if (args_len > sizeof(args_buf) - 1) {
            args_len = sizeof(args_buf) - 1;
        }
        memcpy(args_buf, args_start, args_len);
        args_buf[args_len] = '\0';
        args_ptr = args_buf;
    } else {
        // No args - copy command name only (bounded)
        size_t cmd_len = strlen(cmd_start);
        if (cmd_len > sizeof(cmd_name) - 1) {
            cmd_len = sizeof(cmd_name) - 1;
        }
        memcpy(cmd_name, cmd_start, cmd_len);
        cmd_name[cmd_len] = '\0';
    }
    
    // Call command handler
    if (s_lan.cmd_handler) {
        char response[64];
        response[0] = '\0';
        bool ok = s_lan.cmd_handler(cmd_name, args_ptr, response, sizeof(response));
        
        // Format and send response with explicit field width limits
        char rsp_line[128];
        int rsp_len = snprintf(rsp_line, sizeof(rsp_line), "RSP,%s,%.28s,%.56s\n", 
                               ok ? "OK" : "ERR", cmd_name, response);
        
        if (rsp_len > 0 && rsp_len < (int)sizeof(rsp_line)) {
            if (xSemaphoreTake(s_lan.tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                send(s_lan.sock, rsp_line, (size_t)rsp_len, 0);
                xSemaphoreGive(s_lan.tx_mutex);
            }
        }
    }
}

/**
 * @brief Receive and process data from socket
 */
static void process_rx_data(void)
{
    char temp[64];
    
    int len = recv(s_lan.sock, temp, sizeof(temp) - 1, 0);
    
    if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available (non-blocking)
            return;
        }
        ESP_LOGE(TAG, "recv error: errno %d", errno);
        disconnect();
        return;
    }
    
    if (len == 0) {
        // Connection closed by peer
        ESP_LOGI(TAG, "Connection closed by PC");
        disconnect();
        return;
    }
    
    // Process received bytes, looking for complete lines
    for (int i = 0; i < len; i++) {
        char c = temp[i];
        
        if (c == '\n') {
            // Complete line received
            s_lan.rx_buffer[s_lan.rx_pos] = '\0';
            process_rx_line(s_lan.rx_buffer);
            s_lan.rx_pos = 0;
        } else if (c != '\r' && s_lan.rx_pos < RX_LINE_SIZE - 1) {
            s_lan.rx_buffer[s_lan.rx_pos++] = c;
        }
    }
}

/**
 * @brief LAN client task
 */
static void lan_client_task(void *arg)
{
    ESP_LOGI(TAG, "LAN client task started");
    ESP_LOGI(TAG, "Server: %s:%d", s_lan.server_ip, s_lan.server_port);
    
    while (1) {
        if (!s_lan.connected) {
            // Attempt connection
            if (connect_to_server() != ESP_OK) {
                s_lan.reconnect_count++;
                vTaskDelay(pdMS_TO_TICKS(s_lan.reconnect_interval_ms));
                continue;
            }
        }
        
        // Process incoming data
        process_rx_data();
        
        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t lan_client_init(const lan_client_config_t *config)
{
    if (!config || !config->server_ip) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_lan.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing LAN client");
    
    // Store configuration
    strncpy(s_lan.server_ip, config->server_ip, sizeof(s_lan.server_ip) - 1);
    s_lan.server_ip[sizeof(s_lan.server_ip) - 1] = '\0';
    s_lan.server_port = config->server_port;
    s_lan.cmd_handler = config->cmd_handler;
    s_lan.reconnect_interval_ms = config->reconnect_interval_ms > 0 
                                   ? config->reconnect_interval_ms : 5000;
    
    s_lan.sock = -1;
    s_lan.connected = false;
    
    // Create mutex for thread-safe sending
    s_lan.tx_mutex = xSemaphoreCreateMutex();
    if (!s_lan.tx_mutex) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Start client task
    BaseType_t ret = xTaskCreate(lan_client_task, "lan_client", 4096, NULL, 5, &s_lan.task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vSemaphoreDelete(s_lan.tx_mutex);
        return ESP_FAIL;
    }
    
    s_lan.initialized = true;
    ESP_LOGI(TAG, "LAN client initialized");
    
    return ESP_OK;
}

esp_err_t lan_client_deinit(void)
{
    if (!s_lan.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing LAN client");
    
    // Stop task
    if (s_lan.task_handle) {
        vTaskDelete(s_lan.task_handle);
        s_lan.task_handle = NULL;
    }
    
    // Close socket
    disconnect();
    
    // Delete mutex
    if (s_lan.tx_mutex) {
        vSemaphoreDelete(s_lan.tx_mutex);
        s_lan.tx_mutex = NULL;
    }
    
    s_lan.initialized = false;
    
    return ESP_OK;
}

bool lan_client_is_connected(void)
{
    return s_lan.connected;
}

esp_err_t lan_client_send_sample(const m106h_sample_t *sample)
{
    if (!s_lan.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get timestamp (milliseconds since boot)
    int64_t timestamp_ms = esp_timer_get_time() / 1000;
    
    // Format: DATA,timestamp,o3,temp,press,sample_v,ref_v,day,month,year,hour,min,sec
    char buffer[TX_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer),
                       "DATA,%lld,%.4f,%.2f,%.2f,%.5f,%.5f,%u,%u,%u,%u,%u,%u\n",
                       timestamp_ms,
                       sample->ozone_wt_pct,
                       sample->temperature_c,
                       sample->pressure_mbar,
                       sample->sample_pdv_v,
                       sample->ref_pdv_v,
                       (unsigned)sample->day,
                       (unsigned)sample->month,
                       (unsigned)sample->year,
                       (unsigned)sample->hour,
                       (unsigned)sample->minute,
                       (unsigned)sample->second);
    
    if (len < 0 || len >= (int)sizeof(buffer)) {
        return ESP_ERR_NO_MEM;
    }
    
    // Thread-safe send
    if (xSemaphoreTake(s_lan.tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    int sent = send(s_lan.sock, buffer, (size_t)len, 0);
    xSemaphoreGive(s_lan.tx_mutex);
    
    if (sent < 0) {
        ESP_LOGE(TAG, "Send failed: errno %d", errno);
        disconnect();
        return ESP_FAIL;
    }
    
    s_lan.tx_count++;
    ESP_LOGD(TAG, "TX: %d bytes", sent);
    
    return ESP_OK;
}

esp_err_t lan_client_send_message(const char *message)
{
    if (!s_lan.connected || !message) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_lan.tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    size_t len = strlen(message);
    int sent = send(s_lan.sock, message, len, 0);
    
    // Add newline if not present
    if (len > 0 && message[len-1] != '\n') {
        send(s_lan.sock, "\n", 1, 0);
    }
    
    xSemaphoreGive(s_lan.tx_mutex);
    
    if (sent < 0) {
        disconnect();
        return ESP_FAIL;
    }
    
    s_lan.tx_count++;
    return ESP_OK;
}

void lan_client_get_stats(bool *connected, uint32_t *tx_count, 
                          uint32_t *rx_count, uint32_t *reconnects)
{
    if (connected) *connected = s_lan.connected;
    if (tx_count) *tx_count = s_lan.tx_count;
    if (rx_count) *rx_count = s_lan.rx_count;
    if (reconnects) *reconnects = s_lan.reconnect_count;
}
