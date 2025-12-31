/**
 * @file peripherals.h
 * @brief Peripheral initialization and management
 * 
 * Centralized initialization for all BlockSI peripherals:
 * - I2C bus (shared by MCP4725 DAC and DFRobot O3 sensor)
 * - SPI bus (for MAX31855 thermocouple)
 * - Individual device initialization
 */

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Peripheral status flags
 */
typedef struct {
    bool i2c_initialized;
    bool spi_initialized;
    bool dac_initialized;
    bool lab_o3_initialized;
    bool thermocouple_initialized;
} peripherals_status_t;

/**
 * @brief Initialize I2C bus
 * 
 * Must be called before initializing any I2C devices.
 * Safe to call multiple times (only initializes once).
 * 
 * @return ESP_OK on success
 */
esp_err_t peripherals_init_i2c(void);

/**
 * @brief Initialize SPI bus
 * 
 * Must be called before initializing SPI devices.
 * Safe to call multiple times (only initializes once).
 * 
 * @return ESP_OK on success
 */
esp_err_t peripherals_init_spi(void);

/**
 * @brief Initialize MCP4725 DAC for O3 power control
 * 
 * Requires I2C to be initialized first.
 * 
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not present
 */
esp_err_t peripherals_init_dac(void);

/**
 * @brief Initialize DFRobot lab O3 sensor
 * 
 * Requires I2C to be initialized first.
 * 
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not present
 */
esp_err_t peripherals_init_lab_o3(void);

/**
 * @brief Initialize MAX31855 thermocouple
 * 
 * Requires SPI to be initialized first.
 * 
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not present
 */
esp_err_t peripherals_init_thermocouple(void);

/**
 * @brief Initialize all peripherals
 * 
 * Calls all individual init functions. Continues even if some fail.
 * Check status to see which devices initialized successfully.
 * 
 * @return ESP_OK if all initialized, ESP_ERR_NOT_FOUND if any missing
 */
esp_err_t peripherals_init_all(void);

/**
 * @brief Get peripheral status
 * 
 * @param status Output status structure
 * @return ESP_OK on success
 */
esp_err_t peripherals_get_status(peripherals_status_t *status);

/**
 * @brief Scan I2C bus for devices
 * 
 * Prints addresses of all responding devices.
 * Useful for debugging I2C issues.
 */
void peripherals_scan_i2c(void);

/**
 * @brief Deinitialize all peripherals
 */
void peripherals_deinit_all(void);

#ifdef __cplusplus
}
#endif

#endif // PERIPHERALS_H
