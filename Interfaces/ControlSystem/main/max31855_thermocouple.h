/**
 * @file max31855_thermocouple.h
 * @brief MAX31855 K-type thermocouple interface driver
 * 
 * The MAX31855 converts K-type thermocouple signal to digital via SPI.
 * 
 * Specifications:
 * - Temperature range: -200°C to +1350°C
 * - Resolution: 0.25°C
 * - Cold-junction compensation: built-in
 * - SPI interface (read-only)
 * - 14-bit thermocouple data + 12-bit cold junction
 */

#ifndef MAX31855_THERMOCOUPLE_H
#define MAX31855_THERMOCOUPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Thermocouple fault codes
 */
typedef enum {
    MAX31855_FAULT_NONE = 0,
    MAX31855_FAULT_OPEN = 1,        // Thermocouple open circuit
    MAX31855_FAULT_SHORT_GND = 2,   // Short to GND
    MAX31855_FAULT_SHORT_VCC = 4    // Short to VCC
} max31855_fault_t;

/**
 * @brief Configuration structure
 */
typedef struct {
    spi_host_device_t spi_host;     // SPI host (SPI2_HOST or SPI3_HOST)
    int cs_gpio;                     // Chip select GPIO
    int sck_gpio;                    // Clock GPIO (can be -1 to use default)
    int miso_gpio;                   // MISO GPIO (can be -1 to use default)
} max31855_config_t;

/**
 * @brief Reading result structure
 */
typedef struct {
    float thermocouple_c;            // Thermocouple temperature in °C
    float cold_junction_c;           // Cold junction (internal) temperature in °C
    max31855_fault_t fault;          // Fault status
    bool valid;                      // True if reading is valid
} max31855_reading_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize MAX31855 thermocouple interface
 * 
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t max31855_init(const max31855_config_t *config);

/**
 * @brief Deinitialize MAX31855
 */
void max31855_deinit(void);

/**
 * @brief Check if MAX31855 is responding
 * 
 * @return true if device responds with valid data pattern
 */
bool max31855_is_present(void);

// ============================================================================
// Temperature Reading
// ============================================================================

/**
 * @brief Read thermocouple temperature
 * 
 * @param temp_c Output: temperature in Celsius
 * @return ESP_OK on success, ESP_ERR_INVALID_RESPONSE if fault detected
 */
esp_err_t max31855_read_temp(float *temp_c);

/**
 * @brief Read full data (thermocouple, cold junction, faults)
 * 
 * @param reading Output: complete reading structure
 * @return ESP_OK on success
 */
esp_err_t max31855_read_full(max31855_reading_t *reading);

/**
 * @brief Read raw 32-bit data from device
 * 
 * For debugging purposes.
 * 
 * @param raw Output: raw 32-bit value
 * @return ESP_OK on success
 */
esp_err_t max31855_read_raw(uint32_t *raw);

/**
 * @brief Get last reading without SPI communication
 * 
 * @return Last temperature reading, or NAN if no valid reading
 */
float max31855_get_last(void);

/**
 * @brief Get fault status from last reading
 * 
 * @return Fault code from last reading
 */
max31855_fault_t max31855_get_last_fault(void);

/**
 * @brief Convert fault code to string
 * 
 * @param fault Fault code
 * @return Human-readable fault description
 */
const char* max31855_fault_to_string(max31855_fault_t fault);

// ============================================================================
// Temperature Units
// ============================================================================

/**
 * @brief Convert Celsius to Fahrenheit
 */
static inline float max31855_c_to_f(float celsius)
{
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

/**
 * @brief Convert Celsius to Kelvin
 */
static inline float max31855_c_to_k(float celsius)
{
    return celsius + 273.15f;
}

#ifdef __cplusplus
}
#endif

#endif // MAX31855_THERMOCOUPLE_H
