/**
 * @file lan_client.h
 * @brief LAN TCP Client for local PC communication
 * 
 * Provides bidirectional communication with a PC on the local network:
 * - Pushes telemetry data to PC
 * - Receives and executes commands from PC
 * - Auto-reconnects on connection loss
 */

#ifndef LAN_CLIENT_H
#define LAN_CLIENT_H

#include <stdbool.h>
#include "esp_err.h"
#include "model_106h_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Command callback function type
 * 
 * Called when a command is received from the PC.
 * 
 * @param cmd Command name (e.g., "relay_set", "status")
 * @param args Argument string (everything after command), NULL if no args
 * @param response Buffer to write response message
 * @param response_size Size of response buffer
 * @return true if command handled successfully
 */
typedef bool (*lan_command_handler_t)(const char *cmd, const char *args,
                                       char *response, size_t response_size);

/**
 * @brief LAN client configuration
 */
typedef struct {
    const char *server_ip;           ///< PC server IP address
    uint16_t server_port;            ///< PC server port
    lan_command_handler_t cmd_handler; ///< Command handler callback
    uint32_t reconnect_interval_ms;  ///< Time between reconnection attempts
} lan_client_config_t;

/**
 * @brief Initialize LAN client
 * 
 * Starts a background task that maintains connection to PC server.
 * 
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t lan_client_init(const lan_client_config_t *config);

/**
 * @brief Deinitialize LAN client
 * 
 * Closes connection and stops background task.
 * 
 * @return ESP_OK on success
 */
esp_err_t lan_client_deinit(void);

/**
 * @brief Check if connected to PC
 * 
 * @return true if TCP connection is active
 */
bool lan_client_is_connected(void);

/**
 * @brief Send telemetry sample to PC
 * 
 * Formats sample as CSV and sends over TCP.
 * Non-blocking - returns immediately if not connected.
 * 
 * @param sample Pointer to sample data
 * @return ESP_OK if sent, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t lan_client_send_sample(const m106h_sample_t *sample);

/**
 * @brief Send arbitrary message to PC
 * 
 * @param message Null-terminated string to send
 * @return ESP_OK if sent, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t lan_client_send_message(const char *message);

/**
 * @brief Get connection statistics
 * 
 * @param connected Output: current connection state
 * @param tx_count Output: total messages sent
 * @param rx_count Output: total commands received
 * @param reconnects Output: number of reconnection attempts
 */
void lan_client_get_stats(bool *connected, uint32_t *tx_count, 
                          uint32_t *rx_count, uint32_t *reconnects);

#ifdef __cplusplus
}
#endif

#endif // LAN_CLIENT_H
