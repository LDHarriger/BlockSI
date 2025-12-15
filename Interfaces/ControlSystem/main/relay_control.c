/**
 * @file relay_control.c
 * @brief Implementation of SSR relay control
 */

#include "relay_control.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "zcbor_decode.h"
#include "zcbor_encode.h"
#include "zcbor_common.h"

static const char *TAG = "RELAY";

// Relay names for logging and RPC
static const char *RELAY_NAMES[RELAY_COUNT] = {
    "ozone_gen",
    "o2_conc"
};

// Human-readable names
static const char *RELAY_DISPLAY_NAMES[RELAY_COUNT] = {
    "Ozone Generator",
    "Oxygen Concentrator"
};

// Module state
static struct {
    bool initialized;
    int gpio_pins[RELAY_COUNT];
    bool active_high;
    relay_state_t states[RELAY_COUNT];
} s_relay = {0};

/**
 * @brief Apply relay state to GPIO
 */
static void apply_gpio_state(relay_id_t relay)
{
    bool gpio_level;
    
    if (s_relay.active_high) {
        gpio_level = (s_relay.states[relay] == RELAY_ON);
    } else {
        gpio_level = (s_relay.states[relay] == RELAY_OFF);
    }
    
    gpio_set_level(s_relay.gpio_pins[relay], gpio_level ? 1 : 0);
}

esp_err_t relay_init(const relay_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing relay control");
    ESP_LOGI(TAG, "  Ozone Generator: GPIO%d", config->ozone_gen_gpio);
    ESP_LOGI(TAG, "  O2 Concentrator: GPIO%d", config->o2_conc_gpio);
    ESP_LOGI(TAG, "  Active: %s", config->active_high ? "HIGH" : "LOW");
    
    // Store configuration
    s_relay.gpio_pins[RELAY_OZONE_GEN] = config->ozone_gen_gpio;
    s_relay.gpio_pins[RELAY_O2_CONC] = config->o2_conc_gpio;
    s_relay.active_high = config->active_high;
    
    // Initialize states to OFF
    s_relay.states[RELAY_OZONE_GEN] = RELAY_OFF;
    s_relay.states[RELAY_O2_CONC] = RELAY_OFF;
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->ozone_gen_gpio) | (1ULL << config->o2_conc_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set initial GPIO states (both OFF)
    apply_gpio_state(RELAY_OZONE_GEN);
    apply_gpio_state(RELAY_O2_CONC);
    
    s_relay.initialized = true;
    ESP_LOGI(TAG, "Relay control initialized - all relays OFF");
    
    return ESP_OK;
}

esp_err_t relay_set(relay_id_t relay, relay_state_t state)
{
    if (!s_relay.initialized) {
        ESP_LOGE(TAG, "Relay module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (relay >= RELAY_COUNT) {
        ESP_LOGE(TAG, "Invalid relay ID: %d", relay);
        return ESP_ERR_INVALID_ARG;
    }
    
    relay_state_t old_state = s_relay.states[relay];
    s_relay.states[relay] = state;
    apply_gpio_state(relay);
    
    if (old_state != state) {
        ESP_LOGI(TAG, "%s: %s -> %s", 
                 RELAY_DISPLAY_NAMES[relay],
                 old_state == RELAY_ON ? "ON" : "OFF",
                 state == RELAY_ON ? "ON" : "OFF");
    }
    
    return ESP_OK;
}

esp_err_t relay_toggle(relay_id_t relay)
{
    if (relay >= RELAY_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    relay_state_t new_state = (s_relay.states[relay] == RELAY_ON) ? RELAY_OFF : RELAY_ON;
    return relay_set(relay, new_state);
}

relay_state_t relay_get_state(relay_id_t relay)
{
    if (relay >= RELAY_COUNT) {
        return RELAY_OFF;
    }
    return s_relay.states[relay];
}

const char* relay_get_name(relay_id_t relay)
{
    if (relay >= RELAY_COUNT) {
        return "unknown";
    }
    return RELAY_DISPLAY_NAMES[relay];
}

void relay_all_off(void)
{
    ESP_LOGW(TAG, "Emergency stop - all relays OFF");
    relay_set(RELAY_OZONE_GEN, RELAY_OFF);
    relay_set(RELAY_O2_CONC, RELAY_OFF);
}

// ============================================================================
// Golioth RPC Handlers
// ============================================================================

/**
 * @brief Parse relay name from CBOR string
 * @return relay_id_t or -1 if not found
 */
static int parse_relay_name(const uint8_t *str, size_t len)
{
    for (int i = 0; i < RELAY_COUNT; i++) {
        if (len == strlen(RELAY_NAMES[i]) && 
            memcmp(str, RELAY_NAMES[i], len) == 0) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief RPC handler for "relay_set"
 * 
 * Expected params: {"relay": "ozone_gen"|"o2_conc", "state": true|false}
 */
static enum golioth_rpc_status on_relay_set(
    zcbor_state_t *request_params_array,
    zcbor_state_t *response_detail_map,
    void *callback_arg)
{
    struct zcbor_string relay_name = {0};
    bool state = false;
    bool ok;
    
    // Parse parameters - expecting a map with "relay" and "state" keys
    // The RPC params come as an array, first element should be our map
    ok = zcbor_tstr_decode(request_params_array, &relay_name);
    if (!ok) {
        ESP_LOGE(TAG, "RPC relay_set: failed to decode relay name");
        return GOLIOTH_RPC_INVALID_ARGUMENT;
    }
    
    ok = zcbor_bool_decode(request_params_array, &state);
    if (!ok) {
        ESP_LOGE(TAG, "RPC relay_set: failed to decode state");
        return GOLIOTH_RPC_INVALID_ARGUMENT;
    }
    
    // Find relay by name
    int relay_id = parse_relay_name(relay_name.value, relay_name.len);
    if (relay_id < 0) {
        ESP_LOGE(TAG, "RPC relay_set: unknown relay name");
        return GOLIOTH_RPC_INVALID_ARGUMENT;
    }
    
    // Set the relay
    esp_err_t ret = relay_set((relay_id_t)relay_id, state ? RELAY_ON : RELAY_OFF);
    if (ret != ESP_OK) {
        return GOLIOTH_RPC_INTERNAL;
    }
    
    ESP_LOGI(TAG, "RPC relay_set: %s = %s", RELAY_NAMES[relay_id], state ? "ON" : "OFF");
    
    // Build response
    ok = zcbor_tstr_put_lit(response_detail_map, "relay")
        && zcbor_tstr_encode(response_detail_map, &relay_name)
        && zcbor_tstr_put_lit(response_detail_map, "state")
        && zcbor_bool_put(response_detail_map, state);
    
    if (!ok) {
        return GOLIOTH_RPC_RESOURCE_EXHAUSTED;
    }
    
    return GOLIOTH_RPC_OK;
}

/**
 * @brief RPC handler for "relay_get"
 * 
 * Returns current state of all relays
 */
static enum golioth_rpc_status on_relay_get(
    zcbor_state_t *request_params_array,
    zcbor_state_t *response_detail_map,
    void *callback_arg)
{
    (void)request_params_array;  // No params needed
    
    bool ozone_state = (relay_get_state(RELAY_OZONE_GEN) == RELAY_ON);
    bool o2_state = (relay_get_state(RELAY_O2_CONC) == RELAY_ON);
    
    ESP_LOGI(TAG, "RPC relay_get: ozone_gen=%s, o2_conc=%s",
             ozone_state ? "ON" : "OFF", o2_state ? "ON" : "OFF");
    
    bool ok = zcbor_tstr_put_lit(response_detail_map, "ozone_gen")
        && zcbor_bool_put(response_detail_map, ozone_state)
        && zcbor_tstr_put_lit(response_detail_map, "o2_conc")
        && zcbor_bool_put(response_detail_map, o2_state);
    
    if (!ok) {
        return GOLIOTH_RPC_RESOURCE_EXHAUSTED;
    }
    
    return GOLIOTH_RPC_OK;
}

/**
 * @brief RPC handler for "relay_all_off"
 * 
 * Emergency stop - turn off all relays
 */
static enum golioth_rpc_status on_relay_all_off(
    zcbor_state_t *request_params_array,
    zcbor_state_t *response_detail_map,
    void *callback_arg)
{
    (void)request_params_array;
    
    relay_all_off();
    
    bool ok = zcbor_tstr_put_lit(response_detail_map, "status")
        && zcbor_tstr_put_lit(response_detail_map, "all_off");
    
    if (!ok) {
        return GOLIOTH_RPC_RESOURCE_EXHAUSTED;
    }
    
    return GOLIOTH_RPC_OK;
}

esp_err_t relay_register_rpc(struct golioth_rpc *rpc)
{
    if (!rpc) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int err;
    
    err = golioth_rpc_register(rpc, "relay_set", on_relay_set, NULL);
    if (err) {
        ESP_LOGE(TAG, "Failed to register relay_set RPC: %d", err);
        return ESP_FAIL;
    }
    
    err = golioth_rpc_register(rpc, "relay_get", on_relay_get, NULL);
    if (err) {
        ESP_LOGE(TAG, "Failed to register relay_get RPC: %d", err);
        return ESP_FAIL;
    }
    
    err = golioth_rpc_register(rpc, "relay_all_off", on_relay_all_off, NULL);
    if (err) {
        ESP_LOGE(TAG, "Failed to register relay_all_off RPC: %d", err);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Registered RPC methods: relay_set, relay_get, relay_all_off");
    return ESP_OK;
}