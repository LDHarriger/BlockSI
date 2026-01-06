/**
 * @file blocksi_pins.h
 * @brief Central pin configuration for BlockSI Control System
 * 
 * All GPIO assignments are defined here to prevent conflicts and
 * provide a single reference for hardware connections.
 * 
 * ESP32-WROOM-32 Pin Usage:
 * ========================
 * 
 * === I2C Bus (Shared) ===
 * GPIO 21: SDA  - DS3502 DigiPot, DFRobot O3 Sensor
 * GPIO 22: SCL  - DS3502 DigiPot, DFRobot O3 Sensor
 * 
 * === SPI Bus (VSPI) ===
 * GPIO 18: SCK  - MAX31855 Thermocouple
 * GPIO 19: MISO - MAX31855 Thermocouple
 * GPIO 23: MOSI - (Reserved, not used by MAX31855)
 * GPIO  5: CS   - MAX31855 Thermocouple
 * 
 * === UART2 (106-H Interface) ===
 * GPIO 16: RX   - 106-H TX (via RS232 level shifter)
 * GPIO 17: TX   - 106-H RX (via RS232 level shifter)
 * 
 * === Relay Control ===
 * GPIO 12: Relay 1 - O3 Generator power
 * GPIO 13: Relay 2 - O2 Concentrator power
 * 
 * === Reserved/Available ===
 * GPIO 25: DAC1 (available)
 * GPIO 26: DAC2/ADC2_CH9 (available)
 * GPIO 27: ADC2_CH7 (available)
 * GPIO 32: ADC1_CH4 (available)
 * GPIO 33: ADC1_CH5 (available)
 * GPIO 34: ADC1_CH6, input only (available)
 * GPIO 35: ADC1_CH7, input only (available)
 * 
 * === Boot/Flash Strapping (Avoid for outputs) ===
 * GPIO  0: Boot button
 * GPIO  2: Must be low/floating for boot
 * GPIO 15: Affects boot log level
 */

#ifndef BLOCKSI_PINS_H
#define BLOCKSI_PINS_H

// ============================================================================
// I2C Bus Configuration
// ============================================================================

#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_GPIO     21
#define I2C_MASTER_SCL_GPIO     22
#define I2C_MASTER_FREQ_HZ      400000      // 400kHz Fast Mode

// I2C Device Addresses
#define DS3502_I2C_ADDR         0x28        // DS3502 DigiPot for O3 power control
#define I2C_ADDR_DFROBOT_O3     0x73        // DFRobot Gravity O3 safety sensor

// Legacy - MCP4725 DAC (replaced by DS3502)
// #define I2C_ADDR_MCP4725     0x62        // Adafruit MCP4725 (A0 floating = VCC)

// ============================================================================
// SPI Bus Configuration (VSPI)
// ============================================================================

#define SPI_HOST_DEVICE         SPI3_HOST   // VSPI
#define SPI_SCK_GPIO            18
#define SPI_MISO_GPIO           19
#define SPI_MOSI_GPIO           23          // Not used by MAX31855, but reserved
#define SPI_CS_THERMOCOUPLE     5           // MAX31855 chip select

// ============================================================================
// UART2 - 106-H Ozone Monitor
// ============================================================================

#define UART_106H_PORT          UART_NUM_2
#define UART_106H_TX_GPIO       17
#define UART_106H_RX_GPIO       16
#define UART_106H_BAUD          19200

// ============================================================================
// Relay Control (Solid State Relays)
// ============================================================================

#define RELAY_O3_GEN_GPIO       12          // Ozone generator SSR
#define RELAY_O2_CONC_GPIO      13          // Oxygen concentrator SSR
#define RELAY_ACTIVE_HIGH       1           // SSRs activate on HIGH

// ============================================================================
// DS3502 Digital Potentiometer Configuration
// ============================================================================

// DS3502 specs
#define DS3502_FULL_SCALE_OHMS  10000       // 10kΩ full scale
#define DS3502_WIPER_STEPS      128         // 7-bit resolution (0-127)
#define DS3502_WIPER_R_OHMS     40          // Typical wiper resistance

// Original MP-8000 potentiometer specs (for compatibility scaling)
#define ORIGINAL_POT_OHMS       4700        // Original 4.7kΩ rheostat
#define ORIGINAL_POT_WIPER_MAX  60          // Wiper position for ~4.7kΩ on DS3502

// ============================================================================
// Power Control Calibration Constants
// ============================================================================

// Power zones (from MP-8000 characterization)
#define POWER_THRESHOLD_PCT     20.0f       // Below this: minimal O3 output
#define POWER_SATURATION_PCT    75.0f       // Above this: diminishing returns

// O3 prediction model: O3_max = A/flow + B (ppm at max power)
#define O3_MODEL_COEFF_A        1.78f       // Inverse flow coefficient
#define O3_MODEL_COEFF_B        1.40f       // Base concentration offset

#endif // BLOCKSI_PINS_H
