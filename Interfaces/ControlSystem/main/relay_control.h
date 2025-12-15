/**
 * @file relay_control.h
 * @brief Solid State Relay Control for BlockSI
 * 
 * Controls two SSRs:
 * - Ozone Generator (MP-8000)
 * - Oxygen Concentrator
 * 
 * Provides local API and Golioth RPC integration for remote control.
 */

#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <stdbool.h>
#include "esp_err.h"
#include "golioth/client.h"
#include "golioth/rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Relay identifiers
 */
typedef enum {
    RELAY_OZONE_GEN = 0,    ///< Ozone generator (MP-8000)
    RELAY_O2_CONC,          ///< Oxygen concentrator
    RELAY_COUNT             ///< Number of relays
} relay_id_t;

/**
 * @brief Relay state
 */
typedef enum {
    RELAY_OFF = 0,
    RELAY_ON = 1
} relay_state_t;

/**
 * @brief Relay configuration
 */
typedef struct {
    int ozone_gen_gpio;     ///< GPIO pin for ozone generator SSR
    int o2_conc_gpio;       ///< GPIO pin for oxygen concentrator SSR
    bool active_high;       ///< true = HIGH activates relay, false = LOW activates
} relay_config_t;

/**
 * @brief Initialize relay control module
 * 
 * Configures GPIO pins as outputs and sets initial state to OFF.
 * 
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t relay_init(const relay_config_t *config);

/**
 * @brief Set relay state
 * 
 * @param relay Relay identifier
 * @param state Desired state (RELAY_ON or RELAY_OFF)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for invalid relay
 */
esp_err_t relay_set(relay_id_t relay, relay_state_t state);

/**
 * @brief Toggle relay state
 * 
 * @param relay Relay identifier
 * @return ESP_OK on success
 */
esp_err_t relay_toggle(relay_id_t relay);

/**
 * @brief Get current relay state
 * 
 * @param relay Relay identifier
 * @return Current state, or RELAY_OFF if invalid relay
 */
relay_state_t relay_get_state(relay_id_t relay);

/**
 * @brief Get relay name string
 * 
 * @param relay Relay identifier
 * @return Human-readable name
 */
const char* relay_get_name(relay_id_t relay);

/**
 * @brief Set all relays to OFF (emergency stop)
 */
void relay_all_off(void);

/**
 * @brief Register Golioth RPC handlers for remote relay control
 * 
 * Registers the following RPC methods:
 * - "relay_set": Set specific relay state
 *   Params: {"relay": "ozone_gen"|"o2_conc", "state": true|false}
 * - "relay_get": Get current relay states
 *   Returns: {"ozone_gen": true|false, "o2_conc": true|false}
 * - "relay_all_off": Emergency stop - turn off all relays
 * 
 * @param rpc Golioth RPC handle from golioth_rpc_init()
 * @return ESP_OK on success
 */
esp_err_t relay_register_rpc(struct golioth_rpc *rpc);

#ifdef __cplusplus
}
#endif

#endif // RELAY_CONTROL_H
