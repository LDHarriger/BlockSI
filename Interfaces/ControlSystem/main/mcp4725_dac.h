/**
 * @file mcp4725_dac.h
 * @brief MCP4725 12-bit I2C DAC driver
 * 
 * Used for controlling MP-8000 ozone generator power level.
 * The Adafruit MCP4725 breakout includes an output buffer op-amp.
 * 
 * Specifications:
 * - 12-bit resolution (0-4095)
 * - Output voltage: 0 to VDD (typically 3.3V or 5V)
 * - I2C interface (up to 400kHz)
 * - EEPROM for power-on default value
 * 
 * Voltage divider on output:
 * - R1 = 18kΩ (to DAC output)
 * - R2 = 10kΩ (to GND)
 * - Output range: 0 - 1.18V (for 3.3V VDD)
 */

#ifndef MCP4725_DAC_H
#define MCP4725_DAC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MCP4725 I2C addresses
 * 
 * Default is 0x60. Can be changed to 0x61 by connecting A0 to VDD.
 */
#define MCP4725_ADDR_DEFAULT    0x60
#define MCP4725_ADDR_A0_HIGH    0x61

/**
 * @brief Power-down modes
 */
typedef enum {
    MCP4725_POWER_NORMAL = 0,       // Normal operation
    MCP4725_POWER_DOWN_1K = 1,      // Power-down with 1kΩ to GND
    MCP4725_POWER_DOWN_100K = 2,    // Power-down with 100kΩ to GND
    MCP4725_POWER_DOWN_500K = 3     // Power-down with 500kΩ to GND
} mcp4725_power_mode_t;

/**
 * @brief Configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;            // I2C port number
    uint8_t i2c_addr;               // I2C address (0x60 or 0x61)
    float vdd_voltage;              // Supply voltage (for voltage calculation)
    float divider_ratio;            // Output voltage divider ratio (R2/(R1+R2))
} mcp4725_config_t;

/**
 * @brief Initialize MCP4725 DAC
 * 
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t mcp4725_init(const mcp4725_config_t *config);

/**
 * @brief Deinitialize MCP4725
 */
void mcp4725_deinit(void);

/**
 * @brief Check if MCP4725 is present on I2C bus
 * 
 * @return true if device responds
 */
bool mcp4725_is_present(void);

/**
 * @brief Set DAC output value (fast write, no EEPROM)
 * 
 * @param value 12-bit value (0-4095)
 * @return ESP_OK on success
 */
esp_err_t mcp4725_set_value(uint16_t value);

/**
 * @brief Set DAC output value and save to EEPROM
 * 
 * EEPROM has limited write cycles (~1 million).
 * Use sparingly - only for setting power-on default.
 * 
 * @param value 12-bit value (0-4095)
 * @return ESP_OK on success
 */
esp_err_t mcp4725_set_value_eeprom(uint16_t value);

/**
 * @brief Get current DAC value
 * 
 * @param value Output: current 12-bit value
 * @return ESP_OK on success
 */
esp_err_t mcp4725_get_value(uint16_t *value);

/**
 * @brief Set output voltage (after divider)
 * 
 * Converts desired output voltage to DAC value.
 * 
 * @param voltage Desired output voltage in volts
 * @return ESP_OK on success
 */
esp_err_t mcp4725_set_voltage(float voltage);

/**
 * @brief Get current output voltage (after divider)
 * 
 * @return Output voltage in volts
 */
float mcp4725_get_voltage(void);

/**
 * @brief Set output as percentage of full scale
 * 
 * @param percent Percentage (0.0 - 100.0)
 * @return ESP_OK on success
 */
esp_err_t mcp4725_set_percent(float percent);

/**
 * @brief Get current output as percentage
 * 
 * @return Percentage (0.0 - 100.0)
 */
float mcp4725_get_percent(void);

/**
 * @brief Set power-down mode
 * 
 * @param mode Power-down mode
 * @return ESP_OK on success
 */
esp_err_t mcp4725_set_power_mode(mcp4725_power_mode_t mode);

/**
 * @brief Read device status
 * 
 * @param value Output: current DAC value
 * @param eeprom_value Output: EEPROM stored value
 * @param power_mode Output: current power mode
 * @param eeprom_busy Output: true if EEPROM write in progress
 * @return ESP_OK on success
 */
esp_err_t mcp4725_read_status(uint16_t *value, uint16_t *eeprom_value,
                               mcp4725_power_mode_t *power_mode, bool *eeprom_busy);

#ifdef __cplusplus
}
#endif

#endif // MCP4725_DAC_H
