/**
 * @file blocksi_pins.h
 * @brief Central pin configuration for BlockSI Control System
 * 
 * All GPIO assignments are defined here to prevent conflicts and
 * provide a single reference for hardware connections.
 * 
 * ESP32-WROOM-32 (DOIT DevKit V1) Pin Usage:
 * ==========================================
 * 
 * BOOT STRAPPING PINS TO AVOID FOR ACTIVE OUTPUTS:
 * - GPIO0:  Boot button (must be HIGH for normal boot)
 * - GPIO2:  Must be LOW/floating during boot
 * - GPIO12: MTDI - MUST be LOW during boot (controls flash voltage!)
 * - GPIO15: MTDO - affects boot log level (less critical)
 * 
 * === I2C Bus (Shared) ===
 * GPIO 21: SDA  - MCP4725 DAC (0x60), DFRobot O3 Sensor (0x73)
 * GPIO 22: SCL  - MCP4725 DAC (0x60), DFRobot O3 Sensor (0x73)
 * 
 * === SPI Bus (VSPI) ===
 * GPIO 18: SCK  - MAX31855 Thermocouple
 * GPIO 19: MISO - MAX31855 Thermocouple  
 * GPIO 23: MOSI - (Reserved, not used by MAX31855)
 * GPIO  5: CS   - MAX31855 Thermocouple (safe GPIO)
 * 
 * === UART2 (106-H Interface) ===
 * GPIO 16: RX   - 106-H TX (via RS232 level shifter)
 * GPIO 17: TX   - 106-H RX (via RS232 level shifter)
 * 
 * === Relay Control (SSRs) ===
 * GPIO 12: Relay 1 - O3 Generator power (strapping pin, but validated)
 * GPIO 13: Relay 2 - O2 Concentrator power (safe GPIO)
 * 
 * === Available for Expansion ===
 * GPIO  4: ADC2_CH0, TOUCH0 - available
 * GPIO 25: Was internal DAC1, now available (external MCP4725 used)
 * GPIO 26: ADC2_CH9 - available
 * GPIO 27: ADC2_CH7 - available  
 * GPIO 32: ADC1_CH4 - available
 * GPIO 33: ADC1_CH5 - available
 * GPIO 34: ADC1_CH6, input only - available for sensors
 * GPIO 35: ADC1_CH7, input only - available for sensors
 * GPIO 14: ADC2_CH6 - available
 * GPIO 15: Now available (was SPI CS, moved to GPIO5)
 * 
 * === Do Not Use ===
 * GPIO  0: Boot button
 * GPIO  1: UART0 TX (USB serial)
 * GPIO  2: Boot strapping
 * GPIO  3: UART0 RX (USB serial)
 * GPIO 6-11: Internal flash (not exposed)
 */

#ifndef BLOCKSI_PINS_H
#define BLOCKSI_PINS_H

#include "driver/i2c.h"
#include "driver/spi_master.h"

// ============================================================================
// I2C Bus Configuration
// ============================================================================

#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_GPIO     21
#define I2C_MASTER_SCL_GPIO     22
#define I2C_MASTER_FREQ_HZ      400000      // 400kHz Fast Mode

// I2C Device Addresses
#define I2C_ADDR_MCP4725        0x62        // DAC for O3 power control (A0 floating/high)
#define I2C_ADDR_DFROBOT_O3     0x73        // Lab O3 safety sensor

// ============================================================================
// SPI Bus Configuration (VSPI)
// ============================================================================

#define SPI_HOST_DEVICE         SPI3_HOST   // VSPI
#define SPI_SCK_GPIO            18
#define SPI_MISO_GPIO           19
#define SPI_MOSI_GPIO           23          // Not used by MAX31855, but reserved
#define SPI_CS_THERMOCOUPLE     5           // MAX31855 chip select (safe GPIO)

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
// Note: GPIO12 is a strapping pin but works if relay driver keeps it LOW at boot
// These pins have been validated in the existing hardware setup

#define RELAY_O3_GEN_GPIO       12          // Ozone generator SSR
#define RELAY_O2_CONC_GPIO      13          // Oxygen concentrator SSR

// ============================================================================
// MCP4725 DAC Configuration (O3 Power Control)
// ============================================================================

#define DAC_VDD_VOLTAGE         3.3f        // Supply voltage
#define DAC_DIVIDER_R1          18000       // 18kΩ to DAC output
#define DAC_DIVIDER_R2          10000       // 10kΩ to GND
#define DAC_DIVIDER_RATIO       ((float)DAC_DIVIDER_R2 / (DAC_DIVIDER_R1 + DAC_DIVIDER_R2))
#define DAC_MAX_OUTPUT_V        (DAC_VDD_VOLTAGE * DAC_DIVIDER_RATIO)  // ~1.18V

// Power control effective range (determined empirically)
#define O3_POWER_EFFECTIVE_MIN_PCT  20      // Below this: no O3 output
#define O3_POWER_EFFECTIVE_MAX_PCT  75      // Above this: diminishing returns

// ============================================================================
// DFRobot Lab O3 Sensor Configuration
// ============================================================================

#define LAB_O3_SAMPLE_INTERVAL_MS   1000    // Sample every 1 second
#define LAB_O3_ALARM_WARNING        0.1f    // ppm - OSHA 8-hr TWA limit
#define LAB_O3_ALARM_DANGER         0.3f    // ppm - OSHA 15-min STEL
#define LAB_O3_ALARM_CRITICAL       1.0f    // ppm - Evacuate immediately

// ============================================================================
// Expansion/Available Pins (for future use)
// ============================================================================

#define GPIO_AVAILABLE_1        4           // ADC2_CH0, TOUCH0
#define GPIO_AVAILABLE_2        25          // Was internal DAC
#define GPIO_AVAILABLE_3        26          // ADC2_CH9
#define GPIO_AVAILABLE_4        27          // ADC2_CH7
#define GPIO_AVAILABLE_5        32          // ADC1_CH4
#define GPIO_AVAILABLE_6        33          // ADC1_CH5
#define GPIO_INPUT_ONLY_1       34          // ADC1_CH6 (input only)
#define GPIO_INPUT_ONLY_2       35          // ADC1_CH7 (input only)
#define GPIO_AVAILABLE_7        14          // ADC2_CH6
#define GPIO_AVAILABLE_8        15          // Was SPI CS, now available

#endif // BLOCKSI_PINS_H
