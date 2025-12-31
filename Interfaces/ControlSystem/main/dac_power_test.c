/**
 * @file dac_power_test.c
 * @brief Standalone test firmware for O3 generator DAC power control circuit
 * 
 * This is a minimal test program to verify the DAC → voltage divider → op-amp
 * circuit before integrating with the full BlockSI system.
 * 
 * Hardware Setup:
 * - ESP32 GPIO25 (DAC1) → 18kΩ → junction → 10kΩ → GND
 * - Junction → MCP6001 non-inverting input
 * - MCP6001 output → MP-8000 control input (pot wiper replacement)
 * 
 * Test Procedure:
 * 1. Flash this firmware
 * 2. Connect multimeter to op-amp output
 * 3. Use serial commands to set DAC values
 * 4. Verify output voltage matches expected values
 * 
 * Expected Output Voltages:
 *   DAC=0   (0%)   → 0.00V
 *   DAC=64  (25%)  → 0.30V
 *   DAC=128 (50%)  → 0.59V
 *   DAC=192 (75%)  → 0.89V
 *   DAC=255 (100%) → 1.18V
 * 
 * Serial Commands:
 *   "dac <0-255>"  - Set raw DAC value
 *   "pct <0-100>"  - Set percentage
 *   "sweep"        - Sweep 0-100% in 10% steps
 *   "fine"         - Fine sweep 0-255 in steps of 1
 *   "hold <ms>"    - Set hold time between steps (default 1000ms)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "DAC_TEST";

// DAC configuration
#define DAC_CHANNEL     DAC_CHANNEL_1   // GPIO25
#define DAC_GPIO        25

// Voltage scaling constants
// ESP32 DAC: 0-255 → 0-3.3V
// Voltage divider: 10k/(18k+10k) = 0.357
// Output range: 0 - 1.18V
#define DIVIDER_RATIO   0.357f
#define VCC             3.3f

// Test parameters
static uint32_t s_hold_time_ms = 1000;

/**
 * @brief Calculate expected output voltage for a DAC value
 */
static float calc_expected_voltage(uint8_t dac_value)
{
    float dac_voltage = (dac_value / 255.0f) * VCC;
    return dac_voltage * DIVIDER_RATIO;
}

/**
 * @brief Set DAC output and print expected voltage
 */
static void set_dac(uint8_t value)
{
    esp_err_t ret = dac_output_voltage(DAC_CHANNEL, value);
    float expected_v = calc_expected_voltage(value);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DAC=%3d (%5.1f%%) → Expected: %.3fV", 
                 value, (value / 255.0f) * 100.0f, expected_v);
    } else {
        ESP_LOGE(TAG, "Failed to set DAC: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Set DAC output by percentage
 */
static void set_dac_percent(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint8_t dac_value = (uint8_t)((percent / 100.0f) * 255);
    set_dac(dac_value);
}

/**
 * @brief Sweep DAC from 0 to 255 in specified steps
 */
static void sweep_dac(uint8_t step_size)
{
    ESP_LOGI(TAG, "=== Starting DAC sweep (step=%d, hold=%lums) ===", 
             step_size, (unsigned long)s_hold_time_ms);
    ESP_LOGI(TAG, "Measure voltage at op-amp output and compare to expected");
    ESP_LOGI(TAG, "");
    
    for (int dac = 0; dac <= 255; dac += step_size) {
        set_dac((uint8_t)dac);
        vTaskDelay(pdMS_TO_TICKS(s_hold_time_ms));
        
        // Check for abort (any serial input)
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, 0) > 0) {
            ESP_LOGW(TAG, "Sweep aborted by user");
            return;
        }
    }
    
    ESP_LOGI(TAG, "=== Sweep complete ===");
}

/**
 * @brief Process serial command
 */
static void process_command(char *cmd)
{
    char *token = strtok(cmd, " \r\n");
    if (!token) return;
    
    if (strcmp(token, "dac") == 0) {
        token = strtok(NULL, " \r\n");
        if (token) {
            int value = atoi(token);
            if (value >= 0 && value <= 255) {
                set_dac((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "DAC value must be 0-255");
            }
        } else {
            ESP_LOGW(TAG, "Usage: dac <0-255>");
        }
    }
    else if (strcmp(token, "pct") == 0) {
        token = strtok(NULL, " \r\n");
        if (token) {
            int value = atoi(token);
            if (value >= 0 && value <= 100) {
                set_dac_percent((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Percentage must be 0-100");
            }
        } else {
            ESP_LOGW(TAG, "Usage: pct <0-100>");
        }
    }
    else if (strcmp(token, "sweep") == 0) {
        sweep_dac(26);  // ~10% steps (0, 26, 51, 77, 102, 128, 153, 179, 204, 230, 255)
    }
    else if (strcmp(token, "fine") == 0) {
        sweep_dac(1);   // Every DAC value
    }
    else if (strcmp(token, "hold") == 0) {
        token = strtok(NULL, " \r\n");
        if (token) {
            s_hold_time_ms = atoi(token);
            ESP_LOGI(TAG, "Hold time set to %lu ms", (unsigned long)s_hold_time_ms);
        } else {
            ESP_LOGW(TAG, "Usage: hold <milliseconds>");
        }
    }
    else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        printf("\n");
        printf("=== DAC Power Control Test Commands ===\n");
        printf("  dac <0-255>   Set raw DAC value\n");
        printf("  pct <0-100>   Set percentage\n");
        printf("  sweep         Sweep 0-100%% in ~10%% steps\n");
        printf("  fine          Fine sweep every DAC value\n");
        printf("  hold <ms>     Set hold time between steps\n");
        printf("  help          Show this help\n");
        printf("\n");
        printf("Expected voltage range: 0.00V - 1.18V\n");
        printf("Voltage = DAC/255 * 3.3V * 0.357\n");
        printf("\n");
    }
    else {
        ESP_LOGW(TAG, "Unknown command: %s (type 'help' for commands)", token);
    }
}

/**
 * @brief Serial input task
 */
static void serial_task(void *arg)
{
    char cmd_buf[64];
    int cmd_pos = 0;
    
    while (1) {
        uint8_t c;
        int len = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Echo character
            uart_write_bytes(UART_NUM_0, (const char *)&c, 1);
            
            if (c == '\r' || c == '\n') {
                uart_write_bytes(UART_NUM_0, "\n", 1);
                cmd_buf[cmd_pos] = '\0';
                if (cmd_pos > 0) {
                    process_command(cmd_buf);
                }
                cmd_pos = 0;
            } else if (c == 0x7F || c == 0x08) {  // Backspace
                if (cmd_pos > 0) {
                    cmd_pos--;
                    uart_write_bytes(UART_NUM_0, "\b \b", 3);
                }
            } else if (cmd_pos < sizeof(cmd_buf) - 1) {
                cmd_buf[cmd_pos++] = c;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "   DAC Power Control Circuit Test");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Hardware: GPIO%d → R1(18k) → MCP6001 → Output", DAC_GPIO);
    ESP_LOGI(TAG, "Expected output range: 0.00V - 1.18V");
    ESP_LOGI(TAG, "");
    
    // Initialize DAC
    esp_err_t ret = dac_output_enable(DAC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DAC: %s", esp_err_to_name(ret));
        return;
    }
    
    // Set initial output to 0
    set_dac(0);
    
    ESP_LOGI(TAG, "Type 'help' for available commands");
    ESP_LOGI(TAG, "");
    
    // Start serial command task
    xTaskCreate(serial_task, "serial", 4096, NULL, 5, NULL);
    
    // Print voltage reference table
    printf("\n");
    printf("=== Expected Voltage Reference Table ===\n");
    printf("  DAC   Percent   Voltage\n");
    printf("  ---   -------   -------\n");
    for (int pct = 0; pct <= 100; pct += 10) {
        uint8_t dac = (uint8_t)((pct / 100.0f) * 255);
        float v = calc_expected_voltage(dac);
        printf("  %3d    %3d%%     %.3fV\n", dac, pct, v);
    }
    printf("\n");
}
