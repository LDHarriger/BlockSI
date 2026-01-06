/**
 * @file ds3502_digipot.c
 * @brief DS3502 I2C Digital Potentiometer Driver Implementation
 */

#include "ds3502_digipot.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "DS3502";

// Module state
static struct {
    bool initialized;
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    uint16_t full_scale_ohms;
    uint16_t original_pot_ohms;
    uint8_t current_wiper;
} s_ds3502 = {
    .initialized = false,
    .full_scale_ohms = DS3502_RESISTANCE_FULL_SCALE,
    .original_pot_ohms = 4700,  // Default to original MP-8000 pot value
    .current_wiper = 0
};

// ----------------------------------------------------------------------------
// Internal I2C helpers
// ----------------------------------------------------------------------------

static esp_err_t ds3502_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(s_ds3502.i2c_port, s_ds3502.i2c_addr,
                                       data, 2, pdMS_TO_TICKS(100));
}

static esp_err_t ds3502_read_reg(uint8_t reg, uint8_t *value)
{
    esp_err_t ret;
    
    // Write register address
    ret = i2c_master_write_to_device(s_ds3502.i2c_port, s_ds3502.i2c_addr,
                                      &reg, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read value
    return i2c_master_read_from_device(s_ds3502.i2c_port, s_ds3502.i2c_addr,
                                        value, 1, pdMS_TO_TICKS(100));
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

esp_err_t ds3502_init(const ds3502_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_ds3502.i2c_port = config->i2c_port;
    s_ds3502.i2c_addr = config->i2c_addr;
    s_ds3502.full_scale_ohms = config->full_scale_ohms > 0 ? 
                                config->full_scale_ohms : DS3502_RESISTANCE_FULL_SCALE;
    s_ds3502.original_pot_ohms = config->original_pot_ohms > 0 ?
                                  config->original_pot_ohms : 4700;
    
    ESP_LOGI(TAG, "Initializing DS3502 at address 0x%02X", s_ds3502.i2c_addr);
    ESP_LOGI(TAG, "Full scale: %u ohms, Original pot: %u ohms",
             s_ds3502.full_scale_ohms, s_ds3502.original_pot_ohms);
    
    // Check device presence
    if (!ds3502_is_present()) {
        ESP_LOGE(TAG, "DS3502 not found at address 0x%02X", s_ds3502.i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Read current wiper position
    uint8_t wiper;
    esp_err_t ret = ds3502_read_reg(DS3502_REG_WIPER, &wiper);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read wiper position");
        return ret;
    }
    s_ds3502.current_wiper = wiper & 0x7F;  // Mask to 7 bits
    
    // Disable IVR writes by default (protect NV memory)
    ret = ds3502_write_reg(DS3502_REG_MODE, DS3502_MODE_IVR_WRITE_DISABLE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mode register");
    }
    
    s_ds3502.initialized = true;
    
    ESP_LOGI(TAG, "DS3502 initialized, current wiper: %u (R=%u ohms)",
             s_ds3502.current_wiper, ds3502_get_resistance());
    
    return ESP_OK;
}

void ds3502_deinit(void)
{
    // Set to minimum resistance (safest state - generator off)
    if (s_ds3502.initialized) {
        ds3502_set_wiper(0);
    }
    s_ds3502.initialized = false;
    ESP_LOGI(TAG, "DS3502 deinitialized");
}

bool ds3502_is_present(void)
{
    uint8_t dummy;
    esp_err_t ret = i2c_master_write_to_device(s_ds3502.i2c_port, s_ds3502.i2c_addr,
                                                &dummy, 0, pdMS_TO_TICKS(50));
    return (ret == ESP_OK);
}

esp_err_t ds3502_set_wiper(uint8_t position)
{
    if (!s_ds3502.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp to valid range
    if (position > DS3502_WIPER_MAX) {
        position = DS3502_WIPER_MAX;
    }
    
    esp_err_t ret = ds3502_write_reg(DS3502_REG_WIPER, position);
    if (ret == ESP_OK) {
        s_ds3502.current_wiper = position;
        ESP_LOGD(TAG, "Wiper set to %u (R=%u ohms)", position, ds3502_get_resistance());
    } else {
        ESP_LOGE(TAG, "Failed to set wiper: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ds3502_get_wiper(uint8_t *position)
{
    if (!s_ds3502.initialized || position == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t wiper;
    esp_err_t ret = ds3502_read_reg(DS3502_REG_WIPER, &wiper);
    if (ret == ESP_OK) {
        *position = wiper & 0x7F;
        s_ds3502.current_wiper = *position;
    }
    
    return ret;
}

esp_err_t ds3502_set_resistance(uint16_t ohms)
{
    uint8_t wiper = ds3502_resistance_to_wiper(ohms);
    return ds3502_set_wiper(wiper);
}

uint16_t ds3502_get_resistance(void)
{
    return ds3502_wiper_to_resistance(s_ds3502.current_wiper);
}

esp_err_t ds3502_set_power_percent(float percent, bool limit_to_original)
{
    if (!s_ds3502.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp percentage
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    uint8_t wiper;
    
    if (limit_to_original) {
        // Map 0-100% to the original pot range (0 to original_pot_ohms)
        // Calculate max wiper for original pot range
        uint8_t max_wiper = ds3502_resistance_to_wiper(s_ds3502.original_pot_ohms);
        wiper = (uint8_t)((percent / 100.0f) * max_wiper + 0.5f);
    } else {
        // Map 0-100% to full DS3502 range (0-127)
        wiper = (uint8_t)((percent / 100.0f) * DS3502_WIPER_MAX + 0.5f);
    }
    
    ESP_LOGD(TAG, "Power %.1f%% → wiper %u (limit_original=%d)",
             percent, wiper, limit_to_original);
    
    return ds3502_set_wiper(wiper);
}

float ds3502_get_power_percent(void)
{
    // Return percentage of full scale
    return (s_ds3502.current_wiper / (float)DS3502_WIPER_MAX) * 100.0f;
}

esp_err_t ds3502_save_to_nv(void)
{
    if (!s_ds3502.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Saving wiper position %u to NV memory", s_ds3502.current_wiper);
    
    // Enable IVR writes
    esp_err_t ret = ds3502_write_reg(DS3502_REG_MODE, DS3502_MODE_IVR_WRITE_ENABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable IVR writes");
        return ret;
    }
    
    // Write current wiper position (this also writes to IVR when enabled)
    ret = ds3502_write_reg(DS3502_REG_WIPER, s_ds3502.current_wiper);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write wiper to NV");
    }
    
    // Disable IVR writes again
    ds3502_write_reg(DS3502_REG_MODE, DS3502_MODE_IVR_WRITE_DISABLE);
    
    return ret;
}

esp_err_t ds3502_get_state(ds3502_state_t *state)
{
    if (!s_ds3502.initialized || state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    state->wiper_position = s_ds3502.current_wiper;
    state->resistance_ohms = ds3502_get_resistance();
    state->power_percent = ds3502_get_power_percent();
    
    return ESP_OK;
}

uint8_t ds3502_resistance_to_wiper(uint16_t ohms)
{
    // R = (wiper / 127) * full_scale + wiper_resistance
    // Solving for wiper:
    // wiper = ((R - wiper_resistance) / full_scale) * 127
    
    if (ohms <= DS3502_WIPER_RESISTANCE) {
        return 0;
    }
    
    uint32_t adjusted = ohms - DS3502_WIPER_RESISTANCE;
    uint32_t wiper = (adjusted * DS3502_WIPER_MAX) / s_ds3502.full_scale_ohms;
    
    if (wiper > DS3502_WIPER_MAX) {
        wiper = DS3502_WIPER_MAX;
    }
    
    return (uint8_t)wiper;
}

uint16_t ds3502_wiper_to_resistance(uint8_t wiper)
{
    // R = (wiper / 127) * full_scale + wiper_resistance
    uint32_t r = ((uint32_t)wiper * s_ds3502.full_scale_ohms) / DS3502_WIPER_MAX;
    r += DS3502_WIPER_RESISTANCE;
    
    return (uint16_t)r;
}
