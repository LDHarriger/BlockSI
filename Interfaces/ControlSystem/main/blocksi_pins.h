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
 * GPIO 21: SDA  - MCP4725 DAC, DFRobot O3 Sensor
 * GPIO 22: SCL  - MCP4725 DAC, DFRobot O3 Sensor
 * 
 * === SPI Bus (VSPI) ===
 * GPIO 18: SCK  - MAX31855 Thermocouple
 * GPIO 19: MISO - MAX31855 Thermocouple
 * GPIO 23: MOSI - (Reserved, not used by MAX31855)
 * GPIO 15: CS   - MAX31855 Thermocouple
 * 
 * === UART2 (106-H Interface) ===
 * GPIO 16: RX   - 106-H TX (via RS232 level shifter)
 * GPIO 17: TX   - 106-H RX (via RS232 level shifter)
 * 
 * === Relay Control ===
 * GPIO  4: Relay 1 - O3 Generator power
 * GPIO  5: Relay 2 - O2 Concentrator power
 * 
 * === Reserved/Available ===
 * GPIO 25: (Was internal DAC, now available)
 * GPIO 26: ADC2_CH9 (available)
 * GPIO 27: ADC2_CH7 (available)
 * GPIO 32: ADC1_CH4 (available)
 * GPIO 33: ADC1_CH5 (available)
 * GPIO 34: ADC1_CH6, input only (available)
 * GPIO 35: ADC1_CH7, input only (available)
 * 
 * === Boot/Flash Strapping (Avoid) ===
 * GPIO  0: Boot button (avoid for outputs)
 * GPIO  2: Must be low for boot
 * GPIO 12: Must be low for boot (MTDI)
 * GPIO 15: OK to use but affects boot log level
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
#define I2C_ADDR_MCP4725        0x60        // DAC for O3 power control
#define I2C_ADDR_DFROBOT_O3     0x73        // Lab O3 safety sensor

// ============================================================================
// SPI Bus Configuration (VSPI)
// ============================================================================

#define SPI_HOST_DEVICE         SPI3_HOST   // VSPI
#define SPI_SCK_GPIO            18
#define SPI_MISO_GPIO           19
#define SPI_MOSI_GPIO           23          // Not used, but reserved
#define SPI_CS_THERMOCOUPLE     15          // MAX31855 chip select

// ============================================================================
// UART2 - 106-H Ozone Monitor
// ============================================================================

#define UART_106H_PORT          UART_NUM_2
#define UART_106H_TX_GPIO       17
#define UART_106H_RX_GPIO       16
#define UART_106H_BAUD          19200

// ============================================================================
// Relay Control
// ============================================================================

#define RELAY_O3_GEN_GPIO       4           // Ozone generator
#define RELAY_O2_CONC_GPIO      5           // Oxygen concentrator

// ============================================================================
// MCP4725 DAC Configuration (O3 Power Control)
// ============================================================================

#define DAC_VDD_VOLTAGE         3.3f        // Supply voltage
#define DAC_DIVIDER_R1          18000       // 18kΩ to DAC output
#define DAC_DIVIDER_R2          10000       // 10kΩ to GND
#define DAC_DIVIDER_RATIO       (DAC_DIVIDER_R2 / (float)(DAC_DIVIDER_R1 + DAC_DIVIDER_R2))
#define DAC_MAX_OUTPUT_V        (DAC_VDD_VOLTAGE * DAC_DIVIDER_RATIO)  // ~1.18V

// Power control effective range (determined empirically)
#define O3_POWER_EFFECTIVE_MIN_PCT  20      // Below this: no O3 output
#define O3_POWER_EFFECTIVE_MAX_PCT  75      // Above this: diminishing returns

// ============================================================================
// Lab O3 Sensor Configuration
// ============================================================================

#define LAB_O3_SAMPLE_INTERVAL_MS   5000    // Sample every 5 seconds

// Safety alarm thresholds (ppm)
#define LAB_O3_ALARM_WARNING        0.1f    // OSHA 8-hour TWA
#define LAB_O3_ALARM_DANGER         0.3f    // OSHA 15-min STEL
#define LAB_O3_ALARM_CRITICAL       1.0f    // Evacuate

// ============================================================================
// Thermocouple Configuration
// ============================================================================

#define THERMO_SAMPLE_INTERVAL_MS   1000    // Sample every second

// ============================================================================
// Default Process Parameters
// ============================================================================

#define DEFAULT_FLOW_RATE_LPM       5.0f    // O2 flow rate
#define DEFAULT_VESSEL_VOLUME_L     2.0f    // Treatment vessel volume
#define DEFAULT_MATERIAL_VOLUME_L   1.0f    // Volume of material

#endif // BLOCKSI_PINS_H
