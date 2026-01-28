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
 * === I2C Bus ===
 * GPIO 21: SDA  - DFRobot O3 Sensor
 * GPIO 22: SCL  - DFRobot O3 Sensor
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
 * === Motor Pot Control (PRM162 + DRV8833) ===
 * GPIO 25: DRV8833 AIN1 - Motor control (PWM)
 * GPIO 26: DRV8833 AIN2 - Motor control (PWM)
 * GPIO 34: ADC input    - Servo track position feedback
 * Note: DRV8833 SLP pin is tied to 5V (always enabled)
 * 
 * === Reserved/Available ===
 * GPIO 27: Available (was DRV8833 SLP, now hardwired)
 * GPIO 32: ADC1_CH4 (available)
 * GPIO 33: ADC1_CH5 (available)
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
#define I2C_ADDR_DFROBOT_O3     0x73        // DFRobot Gravity O3 safety sensor

// ============================================================================
// Lab O3 Safety Sensor Configuration (DFRobot)
// ============================================================================

#define LAB_O3_SAMPLE_INTERVAL_MS   1000    // Sample every 1 second
#define LAB_O3_ALARM_WARNING        0.07f   // 70 ppb - OSHA action level
#define LAB_O3_ALARM_DANGER         0.10f   // 100 ppb - OSHA PEL
#define LAB_O3_ALARM_CRITICAL       0.30f   // 300 ppb - Immediate danger

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
#define RELAY_AIR_COMP_GPIO     14          // Air compressor (MP-8000 internal)
#define RELAY_ACTIVE_HIGH       1           // SSRs activate on HIGH

// ============================================================================
// Motorized Potentiometer Control (PRM162 + DRV8833)
// ============================================================================

// DRV8833 H-Bridge Motor Driver Pins
#define MOTOR_POT_AIN1_GPIO     25          // PWM control pin 1
#define MOTOR_POT_AIN2_GPIO     26          // PWM control pin 2
// Note: SLP is tied to 5V on PCB (always enabled, no GPIO control)

// Servo Track ADC (position feedback from secondary pot section)
#define MOTOR_POT_ADC_GPIO      34          // ADC1_CH6, input only

// PRM162 Specifications
#define MOTOR_POT_RESISTANCE    5000        // 5kΩ per section
#define MOTOR_POT_VOLTAGE       4.5f        // Motor rated voltage
#define MOTOR_POT_CURRENT_MA    100         // Motor max current

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
