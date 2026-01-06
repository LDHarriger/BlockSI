/**
 * @file power_calibration.c
 * @brief Power calibration sweep for MP-8000 ozone generator
 * 
 * This program sweeps through DS3502 wiper positions and records
 * corresponding ozone concentrations from the 106-H monitor.
 * 
 * Purpose:
 * - Characterize the O3 output vs. resistance relationship
 * - Determine useful operating range (original 4.7kΩ vs extended 10kΩ)
 * - Identify power zones (threshold, linear, saturation)
 * - Generate calibration data for predictive control
 * 
 * Protocol:
 * 1. Start at wiper=0 (minimum R, generator likely off)
 * 2. Step through wiper positions with configurable step size
 * 3. Hold at each position for settling time
 * 4. Record multiple O3 samples and compute mean/std
 * 5. Output CSV data via serial and optionally to LAN
 * 
 * Serial Commands:
 *   start [start] [end] [step] [hold_ms]  - Start sweep
 *   stop                                   - Abort sweep
 *   set <wiper>                           - Set specific wiper position
 *   read                                  - Read current O3 and wiper
 *   status                                - Show system status
 * 
 * Output Format (CSV):
 *   wiper,resistance_ohms,o3_ppm,o3_std,temp_c,flow_lpm,samples,elapsed_ms
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "peripherals.h"
#include "ds3502_digipot.h"
#include "model_106h_interface.h"
#include "blocksi_pins.h"

static const char *TAG = "CAL";

// ============================================================================
// Configuration
// ============================================================================

// Default sweep parameters
#define DEFAULT_START_WIPER     0
#define DEFAULT_END_WIPER       127
#define DEFAULT_STEP_SIZE       5
#define DEFAULT_HOLD_TIME_MS    30000   // 30 seconds per step
#define DEFAULT_SETTLE_TIME_MS  10000   // Wait 10s before sampling
#define DEFAULT_SAMPLE_COUNT    10      // Samples to average at each step

// Serial command buffer
#define CMD_BUF_SIZE            128
#define UART_CMD_PORT           UART_NUM_0

// ============================================================================
// State
// ============================================================================

typedef struct {
    bool sweep_active;
    bool sweep_stop_requested;
    uint8_t start_wiper;
    uint8_t end_wiper;
    uint8_t step_size;
    uint32_t hold_time_ms;
    uint32_t settle_time_ms;
    uint16_t sample_count;
    uint8_t current_wiper;
    uint32_t sweep_start_time;
} calibration_state_t;

static calibration_state_t s_cal = {
    .sweep_active = false,
    .sweep_stop_requested = false,
    .start_wiper = DEFAULT_START_WIPER,
    .end_wiper = DEFAULT_END_WIPER,
    .step_size = DEFAULT_STEP_SIZE,
    .hold_time_ms = DEFAULT_HOLD_TIME_MS,
    .settle_time_ms = DEFAULT_SETTLE_TIME_MS,
    .sample_count = DEFAULT_SAMPLE_COUNT,
    .current_wiper = 0
};

// ============================================================================
// O3 Sampling
// ============================================================================

typedef struct {
    float mean;
    float std;
    float min;
    float max;
    float temp;
    float pressure;
    float flow;
    uint16_t count;
} o3_stats_t;

static esp_err_t collect_o3_samples(uint16_t num_samples, uint32_t interval_ms, o3_stats_t *stats)
{
    if (stats == NULL || num_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(stats, 0, sizeof(o3_stats_t));
    stats->min = 1e9;
    stats->max = -1e9;
    
    float sum = 0;
    float sum_sq = 0;
    float temp_sum = 0;
    float press_sum = 0;
    float flow_sum = 0;
    uint16_t valid_count = 0;
    
    for (uint16_t i = 0; i < num_samples; i++) {
        // Get reading from 106-H
        model_106h_data_t data;
        if (model_106h_get_latest_data(&data) == ESP_OK) {
            float o3 = data.ozone_ppm;
            
            sum += o3;
            sum_sq += o3 * o3;
            temp_sum += data.cell_temp_c;
            press_sum += data.cell_press_mbar;
            flow_sum += data.flow_rate_ccm / 1000.0f;  // Convert to LPM
            
            if (o3 < stats->min) stats->min = o3;
            if (o3 > stats->max) stats->max = o3;
            
            valid_count++;
        }
        
        if (interval_ms > 0 && i < num_samples - 1) {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
    }
    
    if (valid_count > 0) {
        stats->mean = sum / valid_count;
        stats->temp = temp_sum / valid_count;
        stats->pressure = press_sum / valid_count;
        stats->flow = flow_sum / valid_count;
        stats->count = valid_count;
        
        if (valid_count > 1) {
            float variance = (sum_sq - (sum * sum) / valid_count) / (valid_count - 1);
            stats->std = (variance > 0) ? sqrtf(variance) : 0;
        }
    }
    
    return (valid_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// ============================================================================
// Calibration Sweep
// ============================================================================

static void print_csv_header(void)
{
    printf("\n=== CALIBRATION SWEEP ===\n");
    printf("wiper,resistance_ohms,o3_ppm,o3_std,o3_min,o3_max,temp_c,press_mbar,flow_lpm,samples,elapsed_s\n");
}

static void run_calibration_sweep(void)
{
    s_cal.sweep_active = true;
    s_cal.sweep_stop_requested = false;
    s_cal.sweep_start_time = esp_timer_get_time() / 1000;  // ms
    
    ESP_LOGI(TAG, "Starting calibration sweep: wiper %u→%u, step=%u, hold=%ums",
             s_cal.start_wiper, s_cal.end_wiper, s_cal.step_size, 
             (unsigned)s_cal.hold_time_ms);
    
    print_csv_header();
    
    // Determine direction
    int8_t direction = (s_cal.end_wiper >= s_cal.start_wiper) ? 1 : -1;
    
    for (uint8_t wiper = s_cal.start_wiper;
         (direction > 0) ? (wiper <= s_cal.end_wiper) : (wiper >= s_cal.end_wiper);
         wiper += direction * s_cal.step_size) {
        
        // Check for stop request
        if (s_cal.sweep_stop_requested) {
            ESP_LOGW(TAG, "Sweep stopped by user");
            break;
        }
        
        s_cal.current_wiper = wiper;
        
        // Set wiper position
        esp_err_t ret = ds3502_set_wiper(wiper);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set wiper to %u", wiper);
            continue;
        }
        
        uint16_t resistance = ds3502_get_resistance();
        ESP_LOGI(TAG, "Wiper=%u (R=%u ohms), settling...", wiper, resistance);
        
        // Wait for settling
        vTaskDelay(pdMS_TO_TICKS(s_cal.settle_time_ms));
        
        // Collect samples
        ESP_LOGI(TAG, "Collecting %u samples...", s_cal.sample_count);
        o3_stats_t stats;
        uint32_t sample_interval = (s_cal.hold_time_ms - s_cal.settle_time_ms) / s_cal.sample_count;
        
        ret = collect_o3_samples(s_cal.sample_count, sample_interval, &stats);
        
        // Calculate elapsed time
        uint32_t elapsed_ms = (esp_timer_get_time() / 1000) - s_cal.sweep_start_time;
        float elapsed_s = elapsed_ms / 1000.0f;
        
        // Output CSV row
        if (ret == ESP_OK) {
            printf("%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.2f,%u,%.1f\n",
                   wiper, resistance,
                   stats.mean, stats.std, stats.min, stats.max,
                   stats.temp, stats.pressure, stats.flow,
                   stats.count, elapsed_s);
        } else {
            printf("%u,%u,NaN,NaN,NaN,NaN,NaN,NaN,NaN,0,%.1f\n",
                   wiper, resistance, elapsed_s);
        }
        
        // Prevent underflow/overflow
        if (wiper == 0 && direction < 0) break;
        if (wiper >= 127 && direction > 0) break;
    }
    
    // Return to safe state
    ESP_LOGI(TAG, "Sweep complete, setting power to 0");
    ds3502_set_wiper(0);
    
    s_cal.sweep_active = false;
    printf("=== SWEEP COMPLETE ===\n\n");
}

// ============================================================================
// Command Handler
// ============================================================================

static void handle_command(const char *cmd)
{
    char cmd_copy[CMD_BUF_SIZE];
    strncpy(cmd_copy, cmd, CMD_BUF_SIZE - 1);
    cmd_copy[CMD_BUF_SIZE - 1] = '\0';
    
    // Tokenize
    char *token = strtok(cmd_copy, " \t\r\n");
    if (token == NULL) return;
    
    if (strcmp(token, "start") == 0) {
        if (s_cal.sweep_active) {
            printf("ERROR: Sweep already active\n");
            return;
        }
        
        // Parse optional parameters: start end step hold_ms
        char *arg;
        if ((arg = strtok(NULL, " ")) != NULL) s_cal.start_wiper = atoi(arg);
        if ((arg = strtok(NULL, " ")) != NULL) s_cal.end_wiper = atoi(arg);
        if ((arg = strtok(NULL, " ")) != NULL) s_cal.step_size = atoi(arg);
        if ((arg = strtok(NULL, " ")) != NULL) s_cal.hold_time_ms = atoi(arg);
        
        // Validate
        if (s_cal.step_size == 0) s_cal.step_size = 1;
        if (s_cal.hold_time_ms < 5000) s_cal.hold_time_ms = 5000;
        
        printf("Starting sweep: %u→%u, step=%u, hold=%ums\n",
               s_cal.start_wiper, s_cal.end_wiper, 
               s_cal.step_size, (unsigned)s_cal.hold_time_ms);
        
        run_calibration_sweep();
        
    } else if (strcmp(token, "stop") == 0) {
        if (s_cal.sweep_active) {
            s_cal.sweep_stop_requested = true;
            printf("Stop requested\n");
        } else {
            printf("No sweep active\n");
        }
        
    } else if (strcmp(token, "set") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) {
            printf("Usage: set <wiper 0-127>\n");
            return;
        }
        uint8_t wiper = atoi(arg);
        if (wiper > 127) wiper = 127;
        
        ds3502_set_wiper(wiper);
        printf("Wiper set to %u (R=%u ohms)\n", wiper, ds3502_get_resistance());
        
    } else if (strcmp(token, "read") == 0) {
        uint8_t wiper;
        ds3502_get_wiper(&wiper);
        uint16_t resistance = ds3502_get_resistance();
        
        model_106h_data_t data;
        if (model_106h_get_latest_data(&data) == ESP_OK) {
            printf("Wiper=%u, R=%u ohms, O3=%.4f ppm, T=%.1f C, P=%.1f mbar\n",
                   wiper, resistance, data.ozone_ppm, 
                   data.cell_temp_c, data.cell_press_mbar);
        } else {
            printf("Wiper=%u, R=%u ohms, O3=<no data>\n", wiper, resistance);
        }
        
    } else if (strcmp(token, "status") == 0) {
        printf("\n=== STATUS ===\n");
        printf("Sweep active: %s\n", s_cal.sweep_active ? "YES" : "NO");
        
        uint8_t wiper;
        ds3502_get_wiper(&wiper);
        printf("Current wiper: %u (R=%u ohms)\n", wiper, ds3502_get_resistance());
        
        // 106-H status
        if (model_106h_is_connected()) {
            model_106h_data_t data;
            if (model_106h_get_latest_data(&data) == ESP_OK) {
                printf("106-H: Connected, O3=%.4f ppm\n", data.ozone_ppm);
            } else {
                printf("106-H: Connected, no recent data\n");
            }
        } else {
            printf("106-H: Not connected\n");
        }
        
        printf("\nSweep parameters:\n");
        printf("  Start wiper: %u\n", s_cal.start_wiper);
        printf("  End wiper: %u\n", s_cal.end_wiper);
        printf("  Step size: %u\n", s_cal.step_size);
        printf("  Hold time: %u ms\n", (unsigned)s_cal.hold_time_ms);
        printf("  Settle time: %u ms\n", (unsigned)s_cal.settle_time_ms);
        printf("  Samples: %u\n", s_cal.sample_count);
        printf("\n");
        
    } else if (strcmp(token, "help") == 0) {
        printf("\nCommands:\n");
        printf("  start [start] [end] [step] [hold_ms]  - Start calibration sweep\n");
        printf("  stop                                   - Stop sweep\n");
        printf("  set <wiper>                           - Set wiper position (0-127)\n");
        printf("  read                                  - Read current values\n");
        printf("  status                                - Show system status\n");
        printf("  help                                  - Show this help\n");
        printf("\nExamples:\n");
        printf("  start                    - Full sweep with defaults\n");
        printf("  start 0 60 5 30000      - Sweep 0-60, step 5, hold 30s\n");
        printf("  start 0 127 10 60000    - Extended range, step 10, hold 60s\n");
        printf("\n");
        
    } else {
        printf("Unknown command: %s (type 'help' for commands)\n", token);
    }
}

// ============================================================================
// Command Task
// ============================================================================

static void command_task(void *pvParameters)
{
    char cmd_buf[CMD_BUF_SIZE];
    int cmd_pos = 0;
    
    printf("\n=== BlockSI Power Calibration ===\n");
    printf("Type 'help' for commands\n\n");
    
    while (1) {
        // Read characters from UART
        uint8_t c;
        int len = uart_read_bytes(UART_CMD_PORT, &c, 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            if (c == '\r' || c == '\n') {
                if (cmd_pos > 0) {
                    cmd_buf[cmd_pos] = '\0';
                    handle_command(cmd_buf);
                    cmd_pos = 0;
                }
                printf("> ");
                fflush(stdout);
            } else if (c == '\b' || c == 127) {
                // Backspace
                if (cmd_pos > 0) {
                    cmd_pos--;
                    printf("\b \b");
                    fflush(stdout);
                }
            } else if (cmd_pos < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_pos++] = c;
                printf("%c", c);
                fflush(stdout);
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "BlockSI Power Calibration Starting");
    
    // Initialize UART for commands
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_CMD_PORT, &uart_config);
    uart_driver_install(UART_CMD_PORT, 256, 0, 0, NULL, 0);
    
    // Initialize I2C
    esp_err_t ret = peripherals_init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    
    // Initialize DS3502
    ds3502_config_t ds_config = {
        .i2c_port = I2C_MASTER_PORT,
        .i2c_addr = DS3502_I2C_ADDR,
        .full_scale_ohms = DS3502_FULL_SCALE_OHMS,
        .original_pot_ohms = ORIGINAL_POT_OHMS
    };
    
    ret = ds3502_init(&ds_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS3502");
        return;
    }
    
    // Set to safe state
    ds3502_set_wiper(0);
    
    // Initialize 106-H interface
    ret = model_106h_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "106-H init failed - continuing without O3 monitor");
    }
    
    // Start command task
    xTaskCreate(command_task, "cmd_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Calibration system ready");
}
