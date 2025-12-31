/**
 * @file mcp4725_dac.c
 * @brief MCP4725 12-bit I2C DAC driver implementation
 */

#include "mcp4725_dac.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MCP4725";

// MCP4725 command bytes
#define MCP4725_CMD_FAST_WRITE      0x00    // Fast mode write (no EEPROM)
#define MCP4725_CMD_WRITE_DAC       0x40    // Write DAC register
#define MCP4725_CMD_WRITE_DAC_EEPROM 0x60   // Write DAC and EEPROM

// Module state
static struct {
    bool initialized;
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    float vdd_voltage;
    float divider_ratio;
    uint16_t current_value;
} s_dac = {0};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Write to MCP4725
 */
static esp_err_t mcp4725_write(uint8_t cmd, uint16_t value)
{
    uint8_t data[3];
    
    if (cmd == MCP4725_CMD_FAST_WRITE) {
        // Fast write: 2 bytes
        // Byte 0: [0 0 PD1 PD0 D11 D10 D9 D8]
        // Byte 1: [D7 D6 D5 D4 D3 D2 D1 D0]
        data[0] = (value >> 8) & 0x0F;  // Upper 4 bits of 12-bit value
        data[1] = value & 0xFF;          // Lower 8 bits
        
        ESP_LOGD(TAG, "Fast write: 0x%02X 0x%02X (value=%u)", data[0], data[1], value);
        
        esp_err_t ret = i2c_master_write_to_device(s_dac.i2c_port, s_dac.i2c_addr,
                                                    data, 2, pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fast write failed: %s", esp_err_to_name(ret));
        }
        return ret;
    } else {
        // Standard write: 3 bytes
        // Byte 0: Command byte [C2 C1 C0 x x PD1 PD0 x]
        // Byte 1: [D11 D10 D9 D8 D7 D6 D5 D4]
        // Byte 2: [D3 D2 D1 D0 x x x x]
        data[0] = cmd;
        data[1] = (value >> 4) & 0xFF;   // Upper 8 bits
        data[2] = (value << 4) & 0xF0;   // Lower 4 bits, shifted
        
        ESP_LOGD(TAG, "Write: cmd=0x%02X data=0x%02X 0x%02X (value=%u)", 
                 cmd, data[1], data[2], value);
        
        esp_err_t ret = i2c_master_write_to_device(s_dac.i2c_port, s_dac.i2c_addr,
                                                    data, 3, pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
}

/**
 * @brief Read from MCP4725
 */
static esp_err_t mcp4725_read(uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_read_from_device(s_dac.i2c_port, s_dac.i2c_addr,
                                                 data, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t mcp4725_init(const mcp4725_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_dac.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing MCP4725 DAC");
    ESP_LOGI(TAG, "  I2C port: %d", config->i2c_port);
    ESP_LOGI(TAG, "  I2C address: 0x%02X", config->i2c_addr);
    ESP_LOGI(TAG, "  VDD voltage: %.2fV", config->vdd_voltage);
    ESP_LOGI(TAG, "  Divider ratio: %.3f", config->divider_ratio);
    
    s_dac.i2c_port = config->i2c_port;
    s_dac.i2c_addr = config->i2c_addr;
    s_dac.vdd_voltage = config->vdd_voltage;
    s_dac.divider_ratio = config->divider_ratio;
    
    // Check if device is present
    if (!mcp4725_is_present()) {
        ESP_LOGE(TAG, "MCP4725 not found at address 0x%02X", config->i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "MCP4725 detected at address 0x%02X", config->i2c_addr);
    
    // Read current status
    uint16_t value, eeprom_value;
    mcp4725_power_mode_t power_mode;
    bool eeprom_busy;
    
    esp_err_t ret = mcp4725_read_status(&value, &eeprom_value, &power_mode, &eeprom_busy);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current DAC value: %u (%.2f%%)", value, (value / 4095.0f) * 100);
        ESP_LOGI(TAG, "EEPROM value: %u", eeprom_value);
        ESP_LOGI(TAG, "Power mode: %d", power_mode);
        s_dac.current_value = value;
    }
    
    // Set to 0 initially for safety
    ret = mcp4725_set_value(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial value");
        return ret;
    }
    
    s_dac.initialized = true;
    ESP_LOGI(TAG, "MCP4725 initialized successfully");
    
    return ESP_OK;
}

void mcp4725_deinit(void)
{
    if (!s_dac.initialized) {
        return;
    }
    
    // Set to 0 before deinitializing
    mcp4725_set_value(0);
    
    s_dac.initialized = false;
    ESP_LOGI(TAG, "MCP4725 deinitialized");
}

bool mcp4725_is_present(void)
{
    // Try to read from device
    uint8_t data[5];
    esp_err_t ret = i2c_master_read_from_device(s_dac.i2c_port, s_dac.i2c_addr,
                                                 data, 1, pdMS_TO_TICKS(50));
    return (ret == ESP_OK);
}

esp_err_t mcp4725_set_value(uint16_t value)
{
    if (!s_dac.initialized) {
        // Allow setting during init
        if (s_dac.i2c_port == 0 && s_dac.i2c_addr == 0) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    if (value > 4095) {
        value = 4095;
    }
    
    esp_err_t ret = mcp4725_write(MCP4725_CMD_FAST_WRITE, value);
    if (ret == ESP_OK) {
        s_dac.current_value = value;
        ESP_LOGD(TAG, "Set value: %u (%.2f%%)", value, (value / 4095.0f) * 100);
    }
    
    return ret;
}

esp_err_t mcp4725_set_value_eeprom(uint16_t value)
{
    if (!s_dac.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (value > 4095) {
        value = 4095;
    }
    
    ESP_LOGI(TAG, "Writing to EEPROM: %u", value);
    
    esp_err_t ret = mcp4725_write(MCP4725_CMD_WRITE_DAC_EEPROM, value);
    if (ret == ESP_OK) {
        s_dac.current_value = value;
        
        // Wait for EEPROM write to complete (max 50ms)
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Verify write
        uint16_t eeprom_val;
        mcp4725_power_mode_t mode;
        bool busy;
        if (mcp4725_read_status(NULL, &eeprom_val, &mode, &busy) == ESP_OK) {
            if (eeprom_val == value) {
                ESP_LOGI(TAG, "EEPROM write verified");
            } else {
                ESP_LOGW(TAG, "EEPROM verification mismatch: wrote %u, read %u", 
                         value, eeprom_val);
            }
        }
    }
    
    return ret;
}

esp_err_t mcp4725_get_value(uint16_t *value)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *value = s_dac.current_value;
    return ESP_OK;
}

esp_err_t mcp4725_set_voltage(float voltage)
{
    if (!s_dac.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Calculate DAC voltage needed (before divider)
    float dac_voltage = voltage / s_dac.divider_ratio;
    
    // Clamp to valid range
    if (dac_voltage < 0) dac_voltage = 0;
    if (dac_voltage > s_dac.vdd_voltage) dac_voltage = s_dac.vdd_voltage;
    
    // Convert to 12-bit value
    uint16_t value = (uint16_t)((dac_voltage / s_dac.vdd_voltage) * 4095.0f + 0.5f);
    
    ESP_LOGD(TAG, "Set voltage: %.3fV → DAC=%.3fV → value=%u", 
             voltage, dac_voltage, value);
    
    return mcp4725_set_value(value);
}

float mcp4725_get_voltage(void)
{
    float dac_voltage = (s_dac.current_value / 4095.0f) * s_dac.vdd_voltage;
    return dac_voltage * s_dac.divider_ratio;
}

esp_err_t mcp4725_set_percent(float percent)
{
    if (!s_dac.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    uint16_t value = (uint16_t)((percent / 100.0f) * 4095.0f + 0.5f);
    
    ESP_LOGD(TAG, "Set percent: %.1f%% → value=%u", percent, value);
    
    return mcp4725_set_value(value);
}

float mcp4725_get_percent(void)
{
    return (s_dac.current_value / 4095.0f) * 100.0f;
}

esp_err_t mcp4725_set_power_mode(mcp4725_power_mode_t mode)
{
    if (!s_dac.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Power-down bits are in the fast write command
    // [0 0 PD1 PD0 D11 D10 D9 D8]
    uint8_t data[2];
    data[0] = ((mode & 0x03) << 4) | ((s_dac.current_value >> 8) & 0x0F);
    data[1] = s_dac.current_value & 0xFF;
    
    ESP_LOGI(TAG, "Setting power mode: %d", mode);
    
    return i2c_master_write_to_device(s_dac.i2c_port, s_dac.i2c_addr,
                                       data, 2, pdMS_TO_TICKS(100));
}

esp_err_t mcp4725_read_status(uint16_t *value, uint16_t *eeprom_value,
                               mcp4725_power_mode_t *power_mode, bool *eeprom_busy)
{
    uint8_t data[5];
    
    esp_err_t ret = mcp4725_read(data, 5);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse response
    // Byte 0: [RDY/BSY POR x x x PD1 PD0 x]
    // Byte 1: [D11 D10 D9 D8 D7 D6 D5 D4] - DAC register
    // Byte 2: [D3 D2 D1 D0 x x x x]       - DAC register
    // Byte 3: [x PD1 PD0 x D11 D10 D9 D8] - EEPROM
    // Byte 4: [D7 D6 D5 D4 D3 D2 D1 D0]   - EEPROM
    
    if (eeprom_busy) {
        *eeprom_busy = !(data[0] & 0x80);  // RDY/BSY bit (0 = busy)
    }
    
    if (power_mode) {
        *power_mode = (mcp4725_power_mode_t)((data[0] >> 1) & 0x03);
    }
    
    if (value) {
        *value = ((uint16_t)(data[1]) << 4) | ((data[2] >> 4) & 0x0F);
    }
    
    if (eeprom_value) {
        *eeprom_value = ((uint16_t)(data[3] & 0x0F) << 8) | data[4];
    }
    
    ESP_LOGD(TAG, "Status: value=%u, eeprom=%u, mode=%d, busy=%d",
             value ? *value : 0, eeprom_value ? *eeprom_value : 0,
             power_mode ? *power_mode : 0, eeprom_busy ? *eeprom_busy : 0);
    
    return ESP_OK;
}
