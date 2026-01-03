/**
 * @file main.c
 * @brief BlockSI Control System - Main Application v2.0
 * 
 * Integrates:
 * - 106-H ozone monitor interface (RS232/UART)
 * - MCP4725 DAC for O3 generator power control
 * - DFRobot lab O3 sensor for safety monitoring
 * - MAX31855 thermocouple for vessel temperature
 * - WiFi connectivity
 * - Golioth cloud connection (PSK authentication)
 * - LightDB Stream for telemetry data
 * - LAN streaming to PC dashboard
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include <zcbor_encode.h>

#include "golioth/client.h"
#include "golioth/stream.h"
#include "golioth/rpc.h"

#include "model_106h_interface.h"
#include "relay_control.h"
#include "lan_client.h"
#include "backup_storage.h"
#include "peripherals.h"
#include "blocksi_pins.h"
#include "dosimetry.h"
#include "o3_power_control.h"
#include "sensor_aggregator.h"
#include "mcp4725_dac.h"
#include "dfrobot_ozone.h"
#include "max31855_thermocouple.h"

static const char *TAG = "BLOCKSI";

// Firmware version
#define FIRMWARE_VERSION    "2.0.0"

// Configuration from menuconfig
#define WIFI_SSID               CONFIG_WIFI_SSID
#define WIFI_PASSWORD           CONFIG_WIFI_PASSWORD
#define GOLIOTH_PSK_ID          CONFIG_GOLIOTH_PSK_ID
#define GOLIOTH_PSK             CONFIG_GOLIOTH_PSK

#define M106H_UART_NUM          CONFIG_M106H_UART_NUM
#define M106H_TX_GPIO           CONFIG_M106H_TX_GPIO
#define M106H_RX_GPIO           CONFIG_M106H_RX_GPIO
#define M106H_BAUD_RATE         19200  // Fixed for 106-H

// LAN client configuration
#ifdef CONFIG_LAN_CLIENT_ENABLED
#define LAN_SERVER_IP           CONFIG_LAN_SERVER_IP
#define LAN_SERVER_PORT         CONFIG_LAN_SERVER_PORT
#define LAN_RECONNECT_MS        CONFIG_LAN_RECONNECT_INTERVAL_MS
#endif

// Secondary sensor sample interval (faster than 106-H)
#define SENSOR_SAMPLE_INTERVAL_MS   500

// WiFi event bits
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// Golioth client handle and state
static struct golioth_client *s_client = NULL;
static volatile bool s_golioth_connected = false;

// Synchronization primitives
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t s_golioth_connected_sem = NULL;

// Current process settings
static float s_current_flow_lpm = 5.0f;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    static int s_retry_num = 0;
    const int max_retry = 5;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi station started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < max_retry) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)", s_retry_num, max_retry);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d attempts", max_retry);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi station
 */
static esp_err_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization complete, connecting to %s", WIFI_SSID);
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "WiFi connection failed");
        return ESP_FAIL;
    }
}

/**
 * @brief Golioth event callback
 */
static void golioth_on_event(struct golioth_client *client,
                              enum golioth_client_event event,
                              void *arg)
{
    if (event == GOLIOTH_CLIENT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Golioth connected");
        s_golioth_connected = true;
        if (s_golioth_connected_sem) {
            xSemaphoreGive(s_golioth_connected_sem);
        }
    } else if (event == GOLIOTH_CLIENT_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "Golioth disconnected");
        s_golioth_connected = false;
    }
}

/**
 * @brief Initialize Golioth client
 */
static esp_err_t golioth_init(void)
{
    s_golioth_connected_sem = xSemaphoreCreateBinary();
    if (!s_golioth_connected_sem) {
        ESP_LOGE(TAG, "Failed to create Golioth semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    struct golioth_client_config config = {
        .credentials = {
            .auth_type = GOLIOTH_TLS_AUTH_TYPE_PSK,
            .psk = {
                .psk_id = GOLIOTH_PSK_ID,
                .psk_id_len = strlen(GOLIOTH_PSK_ID),
                .psk = GOLIOTH_PSK,
                .psk_len = strlen(GOLIOTH_PSK),
            }
        }
    };
    
    s_client = golioth_client_create(&config);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to create Golioth client");
        vSemaphoreDelete(s_golioth_connected_sem);
        return ESP_FAIL;
    }
    
    golioth_client_register_event_callback(s_client, golioth_on_event, NULL);
    
    ESP_LOGI(TAG, "Golioth client created, waiting for connection...");
    return ESP_OK;
}

/**
 * @brief Stream push callback
 */
static void stream_push_handler(struct golioth_client *client,
                                 enum golioth_status status,
                                 const struct golioth_coap_rsp_code *coap_rsp_code,
                                 const char *path,
                                 void *arg)
{
    if (status != GOLIOTH_OK) {
        ESP_LOGW(TAG, "Stream push failed: status=%d", status);
    }
}

/**
 * @brief Publish extended sample data to Golioth
 */
static void publish_to_golioth(const m106h_sample_t *sample, 
                                const aggregated_sensors_t *sensors)
{
    if (!s_golioth_connected || !s_client) {
        return;
    }
    
    uint8_t cbor_buf[384];
    ZCBOR_STATE_E(zse, 1, cbor_buf, sizeof(cbor_buf), 1);
    
    char timestamp[24];
    snprintf(timestamp, sizeof(timestamp), "%02u/%02u/%02u %02u:%02u:%02u",
             (unsigned)sample->day, (unsigned)sample->month, (unsigned)sample->year,
             (unsigned)sample->hour, (unsigned)sample->minute, (unsigned)sample->second);
    
    // Encode CBOR map
    bool ok = zcbor_map_start_encode(zse, 10);
    
    ok = ok && zcbor_tstr_put_lit(zse, "ozone_wt_pct");
    ok = ok && zcbor_float32_put(zse, sample->ozone_wt_pct);
    
    ok = ok && zcbor_tstr_put_lit(zse, "temperature_c");
    ok = ok && zcbor_float32_put(zse, sample->temperature_c);
    
    ok = ok && zcbor_tstr_put_lit(zse, "pressure_mbar");
    ok = ok && zcbor_float32_put(zse, sample->pressure_mbar);
    
    ok = ok && zcbor_tstr_put_lit(zse, "sample_pdv_v");
    ok = ok && zcbor_float32_put(zse, sample->sample_pdv_v);
    
    ok = ok && zcbor_tstr_put_lit(zse, "ref_pdv_v");
    ok = ok && zcbor_float32_put(zse, sample->ref_pdv_v);
    
    // Add room O3
    ok = ok && zcbor_tstr_put_lit(zse, "room_o3_ppm");
    if (sensors->room_o3_valid) {
        ok = ok && zcbor_float32_put(zse, sensors->room_o3_ppm);
    } else {
        ok = ok && zcbor_nil_put(zse, NULL);
    }
    
    // Add vessel temperature
    ok = ok && zcbor_tstr_put_lit(zse, "vessel_temp_c");
    if (sensors->vessel_temp_valid) {
        ok = ok && zcbor_float32_put(zse, sensors->vessel_temp_c);
    } else {
        ok = ok && zcbor_nil_put(zse, NULL);
    }
    
    // Add power level
    ok = ok && zcbor_tstr_put_lit(zse, "power_pct");
    ok = ok && zcbor_uint32_put(zse, o3_power_get());
    
    // Add flow rate
    ok = ok && zcbor_tstr_put_lit(zse, "flow_lpm");
    ok = ok && zcbor_float32_put(zse, s_current_flow_lpm);
    
    ok = ok && zcbor_tstr_put_lit(zse, "timestamp");
    ok = ok && zcbor_tstr_put_term(zse, timestamp, sizeof(timestamp));
    
    ok = ok && zcbor_map_end_encode(zse, 10);
    
    if (!ok) {
        ESP_LOGE(TAG, "CBOR encoding failed");
        return;
    }
    
    size_t payload_size = (size_t)((uintptr_t)zse->payload - (uintptr_t)cbor_buf);
    
    golioth_stream_set_async(s_client, "", GOLIOTH_CONTENT_TYPE_CBOR,
                              cbor_buf, payload_size, stream_push_handler, NULL);
}

#ifdef CONFIG_LAN_CLIENT_ENABLED
/**
 * @brief Predict O3 output based on flow rate and power level
 * 
 * Empirical model from calibration data:
 * - At max power: O3 = 1.78/F + 1.40 (hyperbolic flow relationship)
 * - Power scaling: approximately linear in active range (20-75%)
 */
static float predict_o3_output(float flow_lpm, uint8_t power_pct)
{
    if (flow_lpm <= 0 || power_pct == 0) {
        return 0.0f;
    }
    
    // Max power O3 at this flow rate (hyperbolic model)
    float o3_max = 1.78f / flow_lpm + 1.40f;
    
    // Power scaling (empirical three-region model simplified)
    float power_factor;
    if (power_pct < 20) {
        power_factor = power_pct / 100.0f;
    } else if (power_pct <= 75) {
        power_factor = 0.30f + (power_pct - 20.0f) / 55.0f * 0.70f;
    } else {
        power_factor = 1.0f;
    }
    
    return o3_max * power_factor;
}

/**
 * @brief Handle commands received from PC over LAN
 */
static bool lan_command_handler(const char *cmd, const char *args,
                                 char *response, size_t response_size)
{
    ESP_LOGI(TAG, "LAN command: %s, args: %s", cmd, args ? args : "(none)");
    
    // =========================================================================
    // Relay commands
    // =========================================================================
    if (strcmp(cmd, "relay_set") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        char relay_name[16];
        int state;
        if (sscanf(args, "%15[^,],%d", relay_name, &state) != 2) {
            snprintf(response, response_size, "parse_error");
            return false;
        }
        
        relay_id_t relay;
        if (strcmp(relay_name, "ozone_gen") == 0) {
            relay = RELAY_OZONE_GEN;
        } else if (strcmp(relay_name, "o2_conc") == 0) {
            relay = RELAY_O2_CONC;
        } else {
            snprintf(response, response_size, "unknown_relay");
            return false;
        }
        
        relay_state_t new_state = state ? RELAY_ON : RELAY_OFF;
        if (relay_set(relay, new_state) == ESP_OK) {
            snprintf(response, response_size, "%s=%s", relay_name, state ? "on" : "off");
            return true;
        }
        snprintf(response, response_size, "failed");
        return false;
        
    } else if (strcmp(cmd, "relay_get") == 0) {
        snprintf(response, response_size, "ozone_gen=%d,o2_conc=%d",
                 relay_get_state(RELAY_OZONE_GEN) == RELAY_ON ? 1 : 0,
                 relay_get_state(RELAY_O2_CONC) == RELAY_ON ? 1 : 0);
        return true;
    
    // =========================================================================
    // Power control commands
    // =========================================================================
    } else if (strcmp(cmd, "power_set") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        int power_pct = atoi(args);
        if (power_pct < 0 || power_pct > 100) {
            snprintf(response, response_size, "invalid_range");
            return false;
        }
        
        esp_err_t ret = o3_power_set((uint8_t)power_pct);
        if (ret == ESP_OK) {
            float predicted = predict_o3_output(s_current_flow_lpm, power_pct);
            snprintf(response, response_size, "power=%d,predicted_o3=%.2f", 
                     power_pct, predicted);
            return true;
        } else if (ret == ESP_ERR_INVALID_STATE) {
            snprintf(response, response_size, "dac_not_initialized");
        } else {
            snprintf(response, response_size, "failed");
        }
        return false;
        
    } else if (strcmp(cmd, "power_get") == 0) {
        uint8_t power = o3_power_get();
        float voltage = o3_power_get_voltage();
        float predicted = predict_o3_output(s_current_flow_lpm, power);
        snprintf(response, response_size, "pct=%d,flow=%.1f,voltage=%.2f,pred=%.2f",
                 power, s_current_flow_lpm, voltage, predicted);
        return true;
        
    } else if (strcmp(cmd, "flow_set") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        float flow = atof(args);
        if (flow < 0.1f || flow > 10.0f) {
            snprintf(response, response_size, "invalid_range");
            return false;
        }
        
        s_current_flow_lpm = flow;
        dosimetry_set_flow_rate(flow);
        
        float predicted = predict_o3_output(flow, o3_power_get());
        snprintf(response, response_size, "flow=%.1f,predicted_o3=%.2f", 
                 flow, predicted);
        return true;
        
    } else if (strcmp(cmd, "flow_get") == 0) {
        snprintf(response, response_size, "flow=%.1f", s_current_flow_lpm);
        return true;
        
    } else if (strcmp(cmd, "predict_o3") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        float flow;
        int power;
        if (sscanf(args, "%f,%d", &flow, &power) != 2) {
            snprintf(response, response_size, "parse_error");
            return false;
        }
        
        float predicted = predict_o3_output(flow, power);
        snprintf(response, response_size, "predicted_o3=%.2f", predicted);
        return true;
        
    // =========================================================================
    // Sensor status commands
    // =========================================================================
    } else if (strcmp(cmd, "sensors_get") == 0) {
        // Use _is_present() to check if each device was detected during init
        const char *dac_s = mcp4725_is_present() ? "ok" : "err";
        const char *o3_s = dfrobot_o3_is_present() ? "ok" : "err";
        const char *tc_s = max31855_is_present() ? "ok" : "err";
        
        // Get current values from aggregator
        aggregated_sensors_t sensors;
        sensor_aggregator_peek(&sensors);
        
        // Response must fit in ~56 chars (lan_client limit)
        snprintf(response, response_size, 
                 "dac=%s,lab_o3=%s,thermo=%s,room_o3=%.3f,vessel_temp=%.1f",
                 dac_s, o3_s, tc_s,
                 sensors.room_o3_valid ? sensors.room_o3_ppm : 0.0f,
                 sensors.vessel_temp_valid ? sensors.vessel_temp_c : -999.0f);
        return true;
        
    } else if (strcmp(cmd, "room_o3_alarm") == 0) {
        bool alarm = sensor_aggregator_room_o3_alarm();
        float level = dfrobot_o3_get_alarm_level();
        snprintf(response, response_size, "alarm=%s,level=%.2f",
                 alarm ? "active" : "inactive", level);
        return true;
    
    // =========================================================================
    // Recording commands (using backup_storage API)
    // =========================================================================
    } else if (strcmp(cmd, "recording_start") == 0) {
        const char *seq_name = args ? args : "Sequence";
        esp_err_t ret = backup_start_sequence(seq_name);
        snprintf(response, response_size, "recording=%s", 
                 ret == ESP_OK ? "started" : "failed");
        return ret == ESP_OK;
        
    } else if (strcmp(cmd, "recording_stop") == 0) {
        esp_err_t ret = backup_stop_sequence();
        snprintf(response, response_size, "recording=%s",
                 ret == ESP_OK ? "stopped" : "failed");
        return ret == ESP_OK;
        
    } else if (strcmp(cmd, "recording_status") == 0) {
        snprintf(response, response_size, "recording=%s",
                 backup_is_recording() ? "active" : "inactive");
        return true;
        
    // =========================================================================
    // Backup file commands
    // =========================================================================
    } else if (strcmp(cmd, "backup_list") == 0) {
        backup_file_info_t files[5];
        uint8_t count = 0;
        backup_list_files(files, 5, &count);
        
        if (count == 0) {
            snprintf(response, response_size, "files=none");
        } else {
            char *p = response;
            int remaining = response_size;
            int n = snprintf(p, remaining, "files=%d", count);
            p += n; remaining -= n;
            
            for (int i = 0; i < count && remaining > 20; i++) {
                n = snprintf(p, remaining, ",%s:%lu", 
                             files[i].filename, (unsigned long)files[i].size_bytes);
                p += n; remaining -= n;
            }
        }
        return true;
        
    } else if (strcmp(cmd, "backup_delete") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        esp_err_t ret = backup_delete_file(args);
        snprintf(response, response_size, "deleted=%s", ret == ESP_OK ? "ok" : "failed");
        return ret == ESP_OK;
    
    // =========================================================================
    // 106-H commands
    // =========================================================================
    } else if (strcmp(cmd, "106h_status") == 0) {
        m106h_state_t state = m106h_get_state();
        m106h_avg_time_t avg = m106h_get_averaging();
        const char *avg_names[] = {"unknown", "2sec", "10sec", "1min", "5min", "1hr"};
        
        snprintf(response, response_size, "state=%s,avg=%s,logging=%s",
                 state == M106H_STATE_MEASURING ? "measuring" : "menu",
                 avg_names[avg <= 5 ? avg : 0],
                 m106h_is_logging() ? "active" : "inactive");
        return true;
        
    } else if (strcmp(cmd, "106h_avg_set") == 0) {
        if (!args) {
            snprintf(response, response_size, "missing_args");
            return false;
        }
        
        int option = atoi(args);
        if (option < 1 || option > 5) {
            snprintf(response, response_size, "invalid_option");
            return false;
        }
        
        esp_err_t ret = m106h_set_averaging((m106h_avg_time_t)option);
        if (ret == ESP_OK) {
            const char *names[] = {"", "2sec", "10sec", "1min", "5min", "1hr"};
            snprintf(response, response_size, "avg=%s", names[option]);
            return true;
        }
        snprintf(response, response_size, "failed");
        return false;
        
    } else if (strcmp(cmd, "106h_avg_get") == 0) {
        m106h_avg_time_t avg = m106h_get_averaging();
        const char *names[] = {"unknown", "2sec", "10sec", "1min", "5min", "1hr"};
        snprintf(response, response_size, "avg=%s,option=%d", 
                 names[avg <= 5 ? avg : 0], (int)avg);
        return true;
        
    } else if (strcmp(cmd, "106h_log_start") == 0) {
        esp_err_t ret = m106h_log_start();
        snprintf(response, response_size, "logging=%s", ret == ESP_OK ? "started" : "failed");
        return ret == ESP_OK;
        
    } else if (strcmp(cmd, "106h_log_stop") == 0) {
        esp_err_t ret = m106h_log_stop();
        snprintf(response, response_size, "logging=%s", ret == ESP_OK ? "stopped" : "failed");
        return ret == ESP_OK;
        
    // =========================================================================
    // System commands
    // =========================================================================
    } else if (strcmp(cmd, "version") == 0) {
        snprintf(response, response_size, "version=%s,idf=%s", 
                 FIRMWARE_VERSION, esp_get_idf_version());
        return true;
        
    } else if (strcmp(cmd, "status") == 0) {
        peripherals_status_t pstatus;
        peripherals_get_status(&pstatus);
        
        snprintf(response, response_size, 
                 "wifi=%s,golioth=%s,dac=%s,lab_o3=%s,thermo=%s,heap=%lu",
                 (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) ? "ok" : "err",
                 s_golioth_connected ? "ok" : "err",
                 pstatus.dac_initialized ? "ok" : "err",
                 pstatus.lab_o3_initialized ? "ok" : "err",
                 pstatus.thermocouple_initialized ? "ok" : "err",
                 (unsigned long)esp_get_free_heap_size());
        return true;
        
    } else if (strcmp(cmd, "i2c_scan") == 0) {
        peripherals_scan_i2c();
        snprintf(response, response_size, "scan_complete");
        return true;
        
    // =========================================================================
    // Unknown command
    // =========================================================================
    } else {
        snprintf(response, response_size, "unknown_command");
        return false;
    }
}
#endif // CONFIG_LAN_CLIENT_ENABLED

/**
 * @brief Callback for new 106-H samples
 * 
 * Called from UART task context. Keep processing minimal.
 */
static void on_106h_sample(const m106h_sample_t *sample)
{
    // Get aggregated secondary sensor data
    aggregated_sensors_t sensors;
    sensor_aggregator_get_and_reset(&sensors);
    
    // Process dosimetry
    dosimetry_sample_t dosi_sample;
    float o3_ppm = sample->ozone_wt_pct * 10000.0f;  // wt% to ppm
    dosimetry_process_sample(o3_ppm, sample->temperature_c, sample->pressure_mbar,
                              2000, &dosi_sample);  // Assuming 2s interval
    
    // Publish to Golioth cloud
    publish_to_golioth(sample, &sensors);
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    // Send extended sample to LAN - use existing API
    lan_client_send_sample(sample);
    
    // TODO: Extend lan_client to support additional sensor data
    // For now, the 106-H data goes through standard channel
#endif

    // Write to backup storage if recording
    if (backup_is_recording()) {
        backup_write_sample(sample);
    }
    
    // Log summary
    ESP_LOGI(TAG, "O3=%.3f%%, Room=%.3fppm, Vessel=%.1f°C, Pwr=%d%%",
             sample->ozone_wt_pct,
             sensors.room_o3_valid ? sensors.room_o3_ppm : -1.0f,
             sensors.vessel_temp_valid ? sensors.vessel_temp_c : -999.0f,
             o3_power_get());
}

/**
 * @brief Status monitoring task
 */
static void status_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));  // Every 60 seconds
        
        uint32_t total_samples, parse_errors;
        m106h_get_stats(&total_samples, &parse_errors);
        
        peripherals_status_t pstatus;
        peripherals_get_status(&pstatus);
        
        ESP_LOGI(TAG, "=== Status ===");
        ESP_LOGI(TAG, "WiFi: %s, Golioth: %s", 
                 (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) ? "Connected" : "Disconnected",
                 s_golioth_connected ? "Connected" : "Disconnected");
        
#ifdef CONFIG_LAN_CLIENT_ENABLED
        bool lan_conn;
        uint32_t lan_tx, lan_rx, lan_reconn;
        lan_client_get_stats(&lan_conn, &lan_tx, &lan_rx, &lan_reconn);
        ESP_LOGI(TAG, "LAN: %s (TX:%lu RX:%lu)", 
                 lan_conn ? "Connected" : "Disconnected", lan_tx, lan_rx);
#endif
        
        ESP_LOGI(TAG, "106-H: %lu samples, %lu errors", total_samples, parse_errors);
        ESP_LOGI(TAG, "Peripherals: DAC=%s, LabO3=%s, Thermo=%s",
                 pstatus.dac_initialized ? "OK" : "N/A",
                 pstatus.lab_o3_initialized ? "OK" : "N/A",
                 pstatus.thermocouple_initialized ? "OK" : "N/A");
        ESP_LOGI(TAG, "Power: %d%%, Flow: %.1f LPM", o3_power_get(), s_current_flow_lpm);
        ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        
        // Check room O3 alarm
        if (sensor_aggregator_room_o3_alarm()) {
            ESP_LOGW(TAG, "*** ROOM O3 ALARM ACTIVE ***");
        }
    }
}

/**
 * @brief Application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "   BlockSI Control System v%s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    
    // Initialize NVS (required for WiFi and calibration storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize relays FIRST (ensure known OFF state from boot)
    // Use GPIO values from blocksi_pins.h via Kconfig defaults
    ESP_LOGI(TAG, "Initializing relay control...");
    relay_config_t relay_config = {
        .ozone_gen_gpio = CONFIG_RELAY_OZONE_GEN_GPIO,
        .o2_conc_gpio = CONFIG_RELAY_O2_CONC_GPIO,
        .active_high = CONFIG_RELAY_ACTIVE_HIGH,
    };
    if (relay_init(&relay_config) != ESP_OK) {
        ESP_LOGE(TAG, "Relay initialization failed!");
    }
    
    // Initialize backup storage (SPIFFS)
    ESP_LOGI(TAG, "Initializing backup storage...");
    if (backup_storage_init() != ESP_OK) {
        ESP_LOGW(TAG, "Backup storage init failed (continuing without backup)");
    }
    
    // Initialize all peripherals (I2C, SPI, DAC, sensors)
    ESP_LOGI(TAG, "Initializing peripherals...");
    peripherals_init_all();
    
    // Initialize dosimetry
    ESP_LOGI(TAG, "Initializing dosimetry...");
    dosimetry_init(NULL);
    dosimetry_set_flow_rate(s_current_flow_lpm);
    
    // Initialize O3 power control (uses MCP4725 DAC)
    ESP_LOGI(TAG, "Initializing O3 power control...");
    if (o3_power_init() != ESP_OK) {
        ESP_LOGW(TAG, "O3 power control not available (DAC not found)");
    }
    
    // Initialize sensor aggregator (starts background sampling)
    ESP_LOGI(TAG, "Initializing sensor aggregator...");
    sensor_aggregator_init(SENSOR_SAMPLE_INTERVAL_MS);
    
    // Initialize WiFi and block until connected
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed!");
    }
    
    // Initialize Golioth client
    ESP_LOGI(TAG, "Initializing Golioth...");
    if (golioth_init() != ESP_OK) {
        ESP_LOGW(TAG, "Golioth initialization failed (continuing without cloud)");
    }
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    // Initialize LAN client
    ESP_LOGI(TAG, "Initializing LAN client...");
    lan_client_config_t lan_config = {
        .server_ip = LAN_SERVER_IP,
        .server_port = LAN_SERVER_PORT,
        .reconnect_interval_ms = LAN_RECONNECT_MS,
        .cmd_handler = lan_command_handler,
    };
    
    if (lan_client_init(&lan_config) != ESP_OK) {
        ESP_LOGW(TAG, "LAN client initialization failed");
    }
#endif
    
    // Initialize 106-H interface
    ESP_LOGI(TAG, "Initializing 106-H interface...");
    m106h_config_t m106h_config = {
        .uart_num = M106H_UART_NUM,
        .tx_gpio = M106H_TX_GPIO,
        .rx_gpio = M106H_RX_GPIO,
        .baud_rate = M106H_BAUD_RATE,
        .callback = on_106h_sample,
    };
    
    if (m106h_init(&m106h_config) != ESP_OK) {
        ESP_LOGE(TAG, "106-H initialization failed!");
    }
    
    // Start status monitoring task
    xTaskCreate(status_task, "status", 3072, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "   Initialization Complete");
    ESP_LOGI(TAG, "=========================================");
    
    // Main loop - not much to do here, everything is event/task driven
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
