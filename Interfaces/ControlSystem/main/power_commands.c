/**
 * @file power_commands.c
 * @brief Command handlers for DS3502 power control
 * 
 * Add these handlers to main.c's command processing section.
 * Replaces the MCP4725 DAC-based power control.
 */

#include "ds3502_digipot.h"
#include "o3_power_control.h"
#include "blocksi_pins.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Response buffer (defined in main.c)
extern char response_buf[64];

/**
 * @brief Handle power_get command
 * 
 * Returns: pct=<percent>,wiper=<0-127>,resistance=<ohms>,pred=<predicted_o3>
 */
void handle_power_get(void)
{
    o3_power_state_t state;
    
    if (o3_power_get_state(&state) == ESP_OK) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,power_get,pct=%.1f,wiper=%u,resistance=%u,pred=%.2f",
                 state.power_percent,
                 state.wiper_position,
                 state.resistance_ohms,
                 state.predicted_o3_ppm);
    } else {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,power_get,not_initialized");
    }
}

/**
 * @brief Handle power_set command
 * 
 * Args: <percent>
 * Sets power level as percentage (0-100)
 */
void handle_power_set(const char *args)
{
    if (args == NULL || strlen(args) == 0) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,power_set,missing_arg");
        return;
    }
    
    float percent = atof(args);
    
    // Clamp to valid range
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    esp_err_t ret = o3_power_set_percent(percent);
    
    if (ret == ESP_OK) {
        o3_power_state_t state;
        o3_power_get_state(&state);
        
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,power_set,pct=%.1f,wiper=%u,resistance=%u,pred=%.2f",
                 state.power_percent,
                 state.wiper_position,
                 state.resistance_ohms,
                 state.predicted_o3_ppm);
    } else {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,power_set,%s", esp_err_to_name(ret));
    }
}

/**
 * @brief Handle wiper_set command (advanced)
 * 
 * Args: <wiper 0-127>
 * Sets raw wiper position directly
 */
void handle_wiper_set(const char *args)
{
    if (args == NULL || strlen(args) == 0) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,wiper_set,missing_arg");
        return;
    }
    
    int wiper = atoi(args);
    
    // Clamp to valid range
    if (wiper < 0) wiper = 0;
    if (wiper > 127) wiper = 127;
    
    esp_err_t ret = o3_power_set_wiper((uint8_t)wiper);
    
    if (ret == ESP_OK) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,wiper_set,wiper=%d,resistance=%u",
                 wiper, o3_power_get_resistance());
    } else {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,wiper_set,%s", esp_err_to_name(ret));
    }
}

/**
 * @brief Handle power_mode command
 * 
 * Args: original|extended
 * Sets operating mode (limit to 4.7kΩ or use full 10kΩ range)
 */
void handle_power_mode(const char *args)
{
    if (args == NULL || strlen(args) == 0) {
        // Return current mode
        o3_power_mode_t mode = o3_power_get_mode();
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,power_mode,mode=%s",
                 mode == O3_POWER_MODE_ORIGINAL ? "original" : "extended");
        return;
    }
    
    if (strcmp(args, "original") == 0) {
        o3_power_set_mode(O3_POWER_MODE_ORIGINAL);
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,power_mode,mode=original");
    } else if (strcmp(args, "extended") == 0) {
        o3_power_set_mode(O3_POWER_MODE_EXTENDED);
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,power_mode,mode=extended");
    } else {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,power_mode,invalid_mode");
    }
}

/**
 * @brief Handle sensors_get command
 * 
 * Returns status of all secondary sensors including DS3502
 */
void handle_sensors_get(void)
{
    // Check DS3502
    const char *digipot_status = ds3502_is_present() ? "ok" : "err";
    
    // Get room O3 (from DFRobot sensor)
    float room_o3 = 0;  // dfrobot_o3_read(&room_o3) == ESP_OK
    
    // Get vessel temp (from thermocouple)
    float vessel_temp = 0;  // max31855_read_temp(&vessel_temp) == ESP_OK
    
    snprintf(response_buf, sizeof(response_buf),
             "RSP,OK,sensors_get,digipot=%s,room_o3=%.3f,vessel_temp=%.1f",
             digipot_status, room_o3, vessel_temp);
}

/**
 * @brief Handle cal_start command
 * 
 * Args: <start_wiper>,<end_wiper>,<step>,<hold_ms>
 * Starts calibration sweep
 */
void handle_cal_start(const char *args)
{
    if (args == NULL) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,cal_start,missing_args");
        return;
    }
    
    // Parse: start,end,step,hold_ms
    int start = 0, end = 127, step = 5;
    uint32_t hold_ms = 30000;
    
    char args_copy[64];
    strncpy(args_copy, args, sizeof(args_copy) - 1);
    
    char *token = strtok(args_copy, ",");
    if (token) start = atoi(token);
    
    token = strtok(NULL, ",");
    if (token) end = atoi(token);
    
    token = strtok(NULL, ",");
    if (token) step = atoi(token);
    
    token = strtok(NULL, ",");
    if (token) hold_ms = atoi(token);
    
    esp_err_t ret = o3_power_start_calibration(start, end, step, hold_ms, NULL, NULL);
    
    if (ret == ESP_OK) {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,OK,cal_start,start=%d,end=%d,step=%d,hold=%u",
                 start, end, step, (unsigned)hold_ms);
    } else {
        snprintf(response_buf, sizeof(response_buf),
                 "RSP,ERR,cal_start,%s", esp_err_to_name(ret));
    }
}

/**
 * @brief Handle cal_stop command
 */
void handle_cal_stop(void)
{
    o3_power_stop_calibration();
    snprintf(response_buf, sizeof(response_buf), "RSP,OK,cal_stop,stopped");
}

/**
 * @brief Process incoming command
 * 
 * Add this to main.c's command processing switch/if chain:
 * 
 *   } else if (strcmp(cmd, "power_get") == 0) {
 *       handle_power_get();
 *   } else if (strcmp(cmd, "power_set") == 0) {
 *       handle_power_set(args);
 *   } else if (strcmp(cmd, "wiper_set") == 0) {
 *       handle_wiper_set(args);
 *   } else if (strcmp(cmd, "power_mode") == 0) {
 *       handle_power_mode(args);
 *   } else if (strcmp(cmd, "cal_start") == 0) {
 *       handle_cal_start(args);
 *   } else if (strcmp(cmd, "cal_stop") == 0) {
 *       handle_cal_stop();
 *   }
 */

// =============================================================================
// Initialization - Add to app_main()
// =============================================================================

/*
 * Replace MCP4725/DAC initialization with:
 *
 *   // Initialize DS3502 digital potentiometer for power control
 *   ESP_LOGI(TAG, "Initializing DS3502 power control");
 *   ret = o3_power_init();
 *   if (ret != ESP_OK) {
 *       ESP_LOGW(TAG, "DS3502 init failed: %s", esp_err_to_name(ret));
 *   } else {
 *       ESP_LOGI(TAG, "DS3502 power control initialized");
 *   }
 *
 * And remove any MCP4725/DAC initialization code.
 */
