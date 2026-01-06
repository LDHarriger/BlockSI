/**
 * @file ds3502_digipot.h
 * @brief DS3502 I2C Digital Potentiometer Driver
 * 
 * Controls MP-8000 ozone generator power level by replacing the manual
 * 4.7kΩ potentiometer with a DS3502 10kΩ digital potentiometer.
 * 
 * Hardware Configuration:
 * - DS3502 powered by ESP32 3.3V (VCC)
 * - V+ connected to RH (Adafruit default jumper) for high-voltage analog
 * - RH: Connected to MP-8000 control circuit (4.9V source)
 * - RW: Connected to MP-8000 current sink (held at 0V by control circuit)
 * - RL: Jumpered to RW for rheostat mode
 * 
 * Control Behavior:
 * - Wiper 0x00 (0): Minimum resistance (~0Ω) → Generator OFF
 * - Wiper 0x7F (127): Maximum resistance (10kΩ) → Maximum power
 * - Original pot was 4.7kΩ, so ~60 steps covers original range
 * 
 * I2C Details:
 * - Default address: 0x28
 * - Address range: 0x28-0x2B (via A0/A1 pins)
 * - Speed: Up to 400kHz
 */

#ifndef DS3502_DIGIPOT_H
#define DS3502_DIGIPOT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Addresses (directly from datasheet)
#define DS3502_ADDR_00      0x28    // A1=0, A0=0 (default)
#define DS3502_ADDR_01      0x29    // A1=0, A0=1
#define DS3502_ADDR_10      0x2A    // A1=1, A0=0
#define DS3502_ADDR_11      0x2B    // A1=1, A0=1

// Register addresses
#define DS3502_REG_WIPER    0x00    // Wiper position register
#define DS3502_REG_MODE     0x02    // Mode register (IVR write control)

// Mode register bits
#define DS3502_MODE_IVR_WRITE_ENABLE    0x00    // Allow writes to IVR (NV memory)
#define DS3502_MODE_IVR_WRITE_DISABLE   0x80    // Prevent writes to IVR

// Wiper range
#define DS3502_WIPER_MIN    0
#define DS3502_WIPER_MAX    127

// Resistance specs (from datasheet)
#define DS3502_RESISTANCE_FULL_SCALE    10000   // 10kΩ nominal
#define DS3502_WIPER_RESISTANCE         40      // ~40Ω typical wiper resistance

/**
 * @brief Configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;            // I2C port number (I2C_NUM_0 or I2C_NUM_1)
    uint8_t i2c_addr;               // I2C address (0x28-0x2B)
    uint16_t full_scale_ohms;       // Full scale resistance (default 10000)
    uint16_t original_pot_ohms;     // Original potentiometer value for scaling (e.g., 4700)
} ds3502_config_t;

/**
 * @brief Runtime state
 */
typedef struct {
    uint8_t wiper_position;         // Current wiper position (0-127)
    uint16_t resistance_ohms;       // Current resistance in ohms
    float power_percent;            // Mapped power percentage (0-100%)
} ds3502_state_t;

/**
 * @brief Initialize DS3502 digital potentiometer
 * 
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t ds3502_init(const ds3502_config_t *config);

/**
 * @brief Deinitialize DS3502
 */
void ds3502_deinit(void);

/**
 * @brief Check if DS3502 is present on I2C bus
 * 
 * @return true if device responds to address
 */
bool ds3502_is_present(void);

/**
 * @brief Set wiper position directly (0-127)
 * 
 * @param position Wiper position (0 = min R, 127 = max R)
 * @return ESP_OK on success
 */
esp_err_t ds3502_set_wiper(uint8_t position);

/**
 * @brief Get current wiper position
 * 
 * @param position Output: current wiper position
 * @return ESP_OK on success
 */
esp_err_t ds3502_get_wiper(uint8_t *position);

/**
 * @brief Set resistance in ohms (approximate)
 * 
 * @param ohms Desired resistance (0 to full_scale_ohms)
 * @return ESP_OK on success
 */
esp_err_t ds3502_set_resistance(uint16_t ohms);

/**
 * @brief Get current resistance in ohms (approximate)
 * 
 * @return Resistance in ohms
 */
uint16_t ds3502_get_resistance(void);

/**
 * @brief Set power percentage (0-100%)
 * 
 * Maps percentage to wiper position. Can optionally limit to original
 * potentiometer range for compatibility.
 * 
 * @param percent Power level (0-100%)
 * @param limit_to_original If true, limit to original pot range
 * @return ESP_OK on success
 */
esp_err_t ds3502_set_power_percent(float percent, bool limit_to_original);

/**
 * @brief Get current power percentage
 * 
 * @return Power percentage (0-100%)
 */
float ds3502_get_power_percent(void);

/**
 * @brief Save current wiper position to non-volatile memory
 * 
 * The DS3502 will restore this position on power-up.
 * NV memory has limited write cycles (~50,000), use sparingly.
 * 
 * @return ESP_OK on success
 */
esp_err_t ds3502_save_to_nv(void);

/**
 * @brief Get current state
 * 
 * @param state Output: current device state
 * @return ESP_OK on success
 */
esp_err_t ds3502_get_state(ds3502_state_t *state);

/**
 * @brief Calculate wiper position for target resistance
 * 
 * @param ohms Target resistance
 * @return Wiper position (0-127)
 */
uint8_t ds3502_resistance_to_wiper(uint16_t ohms);

/**
 * @brief Calculate resistance for wiper position
 * 
 * @param wiper Wiper position (0-127)
 * @return Resistance in ohms
 */
uint16_t ds3502_wiper_to_resistance(uint8_t wiper);

#ifdef __cplusplus
}
#endif

#endif // DS3502_DIGIPOT_H
