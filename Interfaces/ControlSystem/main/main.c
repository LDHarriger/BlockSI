/**
 * @file main.c
 * @brief BlockSI Control System - Main Application
 * 
 * Integrates:
 * - 106-H ozone monitor interface (RS232/UART)
 * - WiFi connectivity
 * - Golioth cloud connection (PSK authentication)
 * - LightDB Stream for telemetry data
 */

#include <string.h>
#include <stdlib.h>
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

static const char *TAG = "BLOCKSI";

// Configuration from menuconfig
#define WIFI_SSID               CONFIG_WIFI_SSID
#define WIFI_PASSWORD           CONFIG_WIFI_PASSWORD
#define GOLIOTH_PSK_ID          CONFIG_GOLIOTH_PSK_ID
#define GOLIOTH_PSK             CONFIG_GOLIOTH_PSK

#define M106H_UART_NUM          CONFIG_M106H_UART_NUM
#define M106H_TX_GPIO           CONFIG_M106H_TX_GPIO
#define M106H_RX_GPIO           CONFIG_M106H_RX_GPIO
#define M106H_BAUD_RATE         19200  // Fixed for 106-H

// Relay GPIO configuration
#define RELAY_OZONE_GEN_GPIO    CONFIG_RELAY_OZONE_GEN_GPIO
#define RELAY_O2_CONC_GPIO      CONFIG_RELAY_O2_CONC_GPIO
#define RELAY_ACTIVE_HIGH       CONFIG_RELAY_ACTIVE_HIGH

// LAN client configuration
#ifdef CONFIG_LAN_CLIENT_ENABLED
#define LAN_SERVER_IP           CONFIG_LAN_SERVER_IP
#define LAN_SERVER_PORT         CONFIG_LAN_SERVER_PORT
#define LAN_RECONNECT_MS        CONFIG_LAN_RECONNECT_INTERVAL_MS
#endif

// WiFi event bits
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// Golioth client handle and state
static struct golioth_client *s_client = NULL;
static volatile bool s_golioth_connected = false;

// Synchronization primitives
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t s_golioth_connected_sem = NULL;

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
 * @brief Initialize WiFi in station mode
 */
static esp_err_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_got_ip));
    
    // Configure WiFi
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
    
    ESP_LOGI(TAG, "WiFi initialization complete, SSID: %s", WIFI_SSID);
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed");
        return ESP_FAIL;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Golioth client event handler
 */
static void golioth_on_event(struct golioth_client *client, 
                              enum golioth_client_event event,
                              void *arg)
{
    switch (event) {
        case GOLIOTH_CLIENT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Golioth client connected");
            s_golioth_connected = true;
            // Signal that connection is established
            xSemaphoreGive(s_golioth_connected_sem);
            break;
        case GOLIOTH_CLIENT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Golioth client disconnected");
            s_golioth_connected = false;
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize Golioth client
 */
static esp_err_t golioth_init(void)
{
    ESP_LOGI(TAG, "Initializing Golioth client");
    ESP_LOGI(TAG, "PSK-ID: %s", GOLIOTH_PSK_ID);

    // Create connection semaphore
    s_golioth_connected_sem = xSemaphoreCreateBinary();
    if (!s_golioth_connected_sem) {
        ESP_LOGE(TAG, "Failed to create Golioth connection semaphore");
        return ESP_FAIL;
    }

    // Create Golioth client configuration
    // Note: SDK expects PSK as hex string - it handles conversion internally
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
    
    // Create client - this spawns the Golioth client task
    s_client = golioth_client_create(&config);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to create Golioth client");
        vSemaphoreDelete(s_golioth_connected_sem);
        return ESP_FAIL;
    }
    
    // Register event callback
    golioth_client_register_event_callback(s_client, golioth_on_event, NULL);
    
    ESP_LOGI(TAG, "Golioth client created, waiting for connection...");
    return ESP_OK;
}

/**
 * @brief Stream push callback (optional - for debugging)
 */
static void stream_push_handler(struct golioth_client *client,
                                 enum golioth_status status,
                                 const struct golioth_coap_rsp_code *coap_rsp_code,
                                 const char *path,
                                 void *arg)
{
    if (status != GOLIOTH_OK) {
        ESP_LOGW(TAG, "Stream push failed: status=%d", status);
    } else {
        ESP_LOGD(TAG, "Stream push acknowledged");
    }
}

/**
 * @brief Publish sample data to Golioth LightDB Stream using CBOR
 */
static void publish_sample_to_golioth(const m106h_sample_t *sample)
{
    if (!s_golioth_connected || !s_client) {
        ESP_LOGW(TAG, "Golioth not connected, skipping publish");
        return;
    }
    
    // CBOR encoding buffer - sized for our payload
    uint8_t cbor_buf[256];
    
    // Create zcbor encoder state
    ZCBOR_STATE_E(zse, 1, cbor_buf, sizeof(cbor_buf), 1);
    
    // Build timestamp string
    char timestamp[24];
    snprintf(timestamp, sizeof(timestamp), "%02u/%02u/%02u %02u:%02u:%02u",
             (unsigned)sample->day, (unsigned)sample->month, (unsigned)sample->year,
             (unsigned)sample->hour, (unsigned)sample->minute, (unsigned)sample->second);
    
    // Encode CBOR map with 6 key-value pairs
    bool ok = zcbor_map_start_encode(zse, 6);
    
    // ozone_wt_pct
    ok = ok && zcbor_tstr_put_lit(zse, "ozone_wt_pct");
    ok = ok && zcbor_float32_put(zse, sample->ozone_wt_pct);
    
    // temperature_c
    ok = ok && zcbor_tstr_put_lit(zse, "temperature_c");
    ok = ok && zcbor_float32_put(zse, sample->temperature_c);
    
    // pressure_mbar
    ok = ok && zcbor_tstr_put_lit(zse, "pressure_mbar");
    ok = ok && zcbor_float32_put(zse, sample->pressure_mbar);
    
    // sample_pdv_v
    ok = ok && zcbor_tstr_put_lit(zse, "sample_pdv_v");
    ok = ok && zcbor_float32_put(zse, sample->sample_pdv_v);
    
    // ref_pdv_v
    ok = ok && zcbor_tstr_put_lit(zse, "ref_pdv_v");
    ok = ok && zcbor_float32_put(zse, sample->ref_pdv_v);
    
    // timestamp
    ok = ok && zcbor_tstr_put_lit(zse, "timestamp");
    ok = ok && zcbor_tstr_put_term(zse, timestamp, sizeof(timestamp));
    
    // Close the map
    ok = ok && zcbor_map_end_encode(zse, 6);
    
    if (!ok) {
        ESP_LOGE(TAG, "CBOR encoding failed");
        return;
    }
    
    // Calculate actual payload size
    size_t payload_size = (size_t)((uintptr_t)zse->payload - (uintptr_t)cbor_buf);
    
    // Stream to root path
    enum golioth_status status = golioth_stream_set_async(
        s_client,
        "",                          // Empty path = stream root
        GOLIOTH_CONTENT_TYPE_CBOR,   // CBOR content type
        cbor_buf,
        payload_size,
        stream_push_handler,
        NULL);
    
    if (status != GOLIOTH_OK) {
        ESP_LOGW(TAG, "Failed to queue stream publish: %d", status);
    } else {
        ESP_LOGI(TAG, "Queued: O3=%.2f wt%%, T=%.1f°C, P=%.1f mbar",
                 sample->ozone_wt_pct, sample->temperature_c, sample->pressure_mbar);
    }
}

#ifdef CONFIG_LAN_CLIENT_ENABLED
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
        
        relay_set(relay, state ? RELAY_ON : RELAY_OFF);
        snprintf(response, response_size, "%s=%d", relay_name, state);
        return true;
        
    } else if (strcmp(cmd, "relay_get") == 0) {
        snprintf(response, response_size, "ozone_gen=%d,o2_conc=%d",
                 relay_get_state(RELAY_OZONE_GEN) == RELAY_ON ? 1 : 0,
                 relay_get_state(RELAY_O2_CONC) == RELAY_ON ? 1 : 0);
        return true;
        
    } else if (strcmp(cmd, "relay_all_off") == 0) {
        relay_all_off();
        snprintf(response, response_size, "all_off");
        return true;
    
    // =========================================================================
    // Status command
    // =========================================================================
    } else if (strcmp(cmd, "status") == 0) {
        uint32_t samples, errors;
        m106h_get_stats(&samples, &errors);
        snprintf(response, response_size, "heap=%lu,samples=%lu,errors=%lu",
                 (unsigned long)esp_get_free_heap_size(), 
                 (unsigned long)samples, 
                 (unsigned long)errors);
        return true;
    
    // =========================================================================
    // Backup storage commands
    // =========================================================================
    } else if (strcmp(cmd, "backup_start") == 0) {
        if (!args || !args[0]) {
            snprintf(response, response_size, "missing_sequence_type");
            return false;
        }
        
        esp_err_t err = backup_start_sequence(args);
        if (err == ESP_OK) {
            snprintf(response, response_size, "started=%s", args);
            
            // Notify PC of file change
            char notify[64];
            snprintf(notify, sizeof(notify), "FILE,START,%s", args);
            lan_client_send_message(notify);
            return true;
        }
        snprintf(response, response_size, "start_failed");
        return false;
        
    } else if (strcmp(cmd, "backup_stop") == 0) {
        const char *seq_name = backup_get_sequence_name();
        esp_err_t err = backup_stop_sequence();
        if (err == ESP_OK) {
            snprintf(response, response_size, "stopped=%s", seq_name ? seq_name : "none");
            lan_client_send_message("FILE,STOP");
            return true;
        }
        snprintf(response, response_size, "stop_failed");
        return false;
        
    } else if (strcmp(cmd, "backup_status") == 0) {
        backup_storage_status_t status;
        if (backup_get_status(&status) != ESP_OK) {
            snprintf(response, response_size, "status_failed");
            return false;
        }
        snprintf(response, response_size, 
                 "total=%lu,used=%lu,free=%lu,files=%u/%u,recording=%d",
                 (unsigned long)status.total_bytes,
                 (unsigned long)status.used_bytes,
                 (unsigned long)status.free_bytes,
                 status.file_count,
                 status.max_files,
                 backup_is_recording() ? 1 : 0);
        return true;
        
    } else if (strcmp(cmd, "backup_list") == 0) {
        backup_file_info_t files[BACKUP_MAX_FILES];
        uint8_t count = 0;
        
        if (backup_list_files(files, BACKUP_MAX_FILES, &count) != ESP_OK) {
            snprintf(response, response_size, "list_failed");
            return false;
        }
        
        if (count == 0) {
            snprintf(response, response_size, "no_files");
            return true;
        }
        
        // Format: file1.csv:1234:50;file2.csv:5678:100
        char *p = response;
        size_t remaining = response_size;
        
        for (uint8_t i = 0; i < count && remaining > 50; i++) {
            int len = snprintf(p, remaining, "%s%s:%lu:%lu",
                              (i > 0) ? ";" : "",
                              files[i].filename,
                              (unsigned long)files[i].size_bytes,
                              (unsigned long)files[i].sample_count);
            if (len > 0 && len < (int)remaining) {
                p += len;
                remaining -= len;
            }
        }
        return true;
        
    } else if (strcmp(cmd, "backup_delete") == 0) {
        if (!args || !args[0]) {
            snprintf(response, response_size, "missing_filename");
            return false;
        }
        
        esp_err_t err = backup_delete_file(args);
        if (err == ESP_OK) {
            snprintf(response, response_size, "deleted=%s", args);
            return true;
        } else if (err == ESP_ERR_INVALID_STATE) {
            snprintf(response, response_size, "cannot_delete_active");
            return false;
        }
        snprintf(response, response_size, "not_found");
        return false;
        
    } else if (strcmp(cmd, "backup_delete_all") == 0) {
        esp_err_t err = backup_delete_all();
        if (err == ESP_OK) {
            snprintf(response, response_size, "all_deleted");
            return true;
        }
        snprintf(response, response_size, "delete_failed");
        return false;
        
    } else if (strcmp(cmd, "backup_download") == 0) {
        if (!args || !args[0]) {
            snprintf(response, response_size, "missing_filename");
            return false;
        }
        
        backup_file_info_t info;
        if (backup_get_file_info(args, &info) != ESP_OK) {
            snprintf(response, response_size, "not_found");
            return false;
        }
        
        // Send file in chunks
        char chunk_buf[256];
        char send_buf[400];
        uint32_t offset = 0;
        uint32_t bytes_read;
        
        // Send start notification
        snprintf(send_buf, sizeof(send_buf), "CHUNK,START,%s,%lu",
                 args, (unsigned long)info.size_bytes);
        lan_client_send_message(send_buf);
        
        while (backup_read_chunk(args, offset, chunk_buf, sizeof(chunk_buf), &bytes_read) == ESP_OK 
               && bytes_read > 0) {
            
            // Send raw chunk (for simplicity - production could use base64)
            snprintf(send_buf, sizeof(send_buf), "CHUNK,%lu,%lu,",
                     (unsigned long)offset, (unsigned long)bytes_read);
            
            size_t header_len = strlen(send_buf);
            if (header_len + bytes_read < sizeof(send_buf) - 1) {
                memcpy(send_buf + header_len, chunk_buf, bytes_read);
                send_buf[header_len + bytes_read] = '\0';
            }
            
            lan_client_send_message(send_buf);
            offset += bytes_read;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        lan_client_send_message("CHUNK,END");
        snprintf(response, response_size, "sent=%lu", (unsigned long)offset);
        return true;
    
    // =========================================================================
    // 106-H Ozone Monitor Commands
    // =========================================================================
    } else if (strcmp(cmd, "106h_avg") == 0) {
        // Set averaging time: 1=2s, 2=10s, 3=1min, 4=5min, 5=1hr
        if (!args) {
            snprintf(response, response_size, "missing_option");
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
        
    } else if (strcmp(cmd, "106h_log_status") == 0) {
        snprintf(response, response_size, "logging=%s", 
                 m106h_is_logging() ? "active" : "inactive");
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
    // Publish to Golioth cloud
    publish_sample_to_golioth(sample);
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    // Send to local PC
    lan_client_send_sample(sample);
#endif

    // Write to backup storage if recording a sequence
    if (backup_is_recording()) {
        backup_write_sample(sample);
    }
}

/**
 * @brief Status monitoring task
 * 
 * Periodically logs system status and statistics.
 */
static void status_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));  // Every 60 seconds
        
        uint32_t total_samples, parse_errors;
        m106h_get_stats(&total_samples, &parse_errors);
        
        ESP_LOGI(TAG, "=== Status ===");
        ESP_LOGI(TAG, "WiFi: %s", (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) 
                 ? "Connected" : "Disconnected");
        ESP_LOGI(TAG, "Golioth: %s", s_golioth_connected ? "Connected" : "Disconnected");
        
#ifdef CONFIG_LAN_CLIENT_ENABLED
        bool lan_conn;
        uint32_t lan_tx, lan_rx, lan_reconn;
        lan_client_get_stats(&lan_conn, &lan_tx, &lan_rx, &lan_reconn);
        ESP_LOGI(TAG, "LAN: %s (TX:%lu RX:%lu reconn:%lu)", 
                 lan_conn ? "Connected" : "Disconnected", lan_tx, lan_rx, lan_reconn);
#endif
        
        ESP_LOGI(TAG, "106-H Samples: %lu total, %lu errors", total_samples, parse_errors);
        ESP_LOGI(TAG, "Relays: O3_Gen=%s, O2_Conc=%s",
                 relay_get_state(RELAY_OZONE_GEN) == RELAY_ON ? "ON" : "OFF",
                 relay_get_state(RELAY_O2_CONC) == RELAY_ON ? "ON" : "OFF");
        
        // Backup storage status
        backup_storage_status_t backup_status;
        if (backup_get_status(&backup_status) == ESP_OK) {
            ESP_LOGI(TAG, "Backup: %lu/%lu bytes, %u files, %s",
                     (unsigned long)backup_status.used_bytes,
                     (unsigned long)backup_status.total_bytes,
                     backup_status.file_count,
                     backup_is_recording() ? "RECORDING" : "idle");
        }
        
        ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    }
}

/**
 * @brief Application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "=== BlockSI Control System ===");
    ESP_LOGI(TAG, "Firmware version: 1.4.0");
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize relays FIRST (ensure known OFF state from boot)
    ESP_LOGI(TAG, "Initializing relay control...");
    relay_config_t relay_config = {
        .ozone_gen_gpio = RELAY_OZONE_GEN_GPIO,
        .o2_conc_gpio = RELAY_O2_CONC_GPIO,
        .active_high = RELAY_ACTIVE_HIGH,
    };
    if (relay_init(&relay_config) != ESP_OK) {
        ESP_LOGE(TAG, "Relay initialization failed, halting");
        return;
    }
    
    // Initialize backup storage (SPIFFS)
    ESP_LOGI(TAG, "Initializing backup storage...");
    if (backup_storage_init() != ESP_OK) {
        ESP_LOGW(TAG, "Backup storage init failed (continuing without backup)");
    }
    
    // Initialize WiFi and block until connected
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed, halting");
        return;
    }
    
    // Initialize Golioth client (starts connection process)
    ESP_LOGI(TAG, "Initializing Golioth...");
    if (golioth_init() != ESP_OK) {
        ESP_LOGE(TAG, "Golioth initialization failed, halting");
        return;
    }
    
    // *** CRITICAL: Wait for Golioth connection before starting sensor ***
    // The DTLS handshake takes several seconds
    ESP_LOGI(TAG, "Waiting for Golioth connection...");
    if (xSemaphoreTake(s_golioth_connected_sem, pdMS_TO_TICKS(30000)) != pdTRUE) {
        ESP_LOGE(TAG, "Golioth connection timeout after 30s, halting");
        return;
    }
    ESP_LOGI(TAG, "Golioth connected successfully");
    
    // Initialize RPC and register handlers
    ESP_LOGI(TAG, "Registering RPC handlers...");
    struct golioth_rpc *rpc = golioth_rpc_init(s_client);
    if (!rpc) {
        ESP_LOGE(TAG, "Failed to initialize Golioth RPC");
        return;
    }
    
    if (relay_register_rpc(rpc) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register relay RPC handlers");
        return;
    }
    ESP_LOGI(TAG, "RPC handlers registered");
    
#ifdef CONFIG_LAN_CLIENT_ENABLED
    // Initialize LAN client for local PC communication
    ESP_LOGI(TAG, "Initializing LAN client...");
    lan_client_config_t lan_config = {
        .server_ip = LAN_SERVER_IP,
        .server_port = LAN_SERVER_PORT,
        .cmd_handler = lan_command_handler,
        .reconnect_interval_ms = LAN_RECONNECT_MS,
    };
    if (lan_client_init(&lan_config) != ESP_OK) {
        ESP_LOGW(TAG, "LAN client initialization failed (continuing without)");
    } else {
        ESP_LOGI(TAG, "LAN client started, connecting to %s:%d", LAN_SERVER_IP, LAN_SERVER_PORT);
    }
#endif
    
    // Now it's safe to start the 106-H interface
    ESP_LOGI(TAG, "Initializing 106-H interface...");
    m106h_config_t m106h_config = {
        .uart_num = M106H_UART_NUM,
        .tx_gpio = M106H_TX_GPIO,
        .rx_gpio = M106H_RX_GPIO,
        .baud_rate = M106H_BAUD_RATE,
        .callback = on_106h_sample,
    };
    
    if (m106h_init(&m106h_config) != ESP_OK) {
        ESP_LOGE(TAG, "106-H initialization failed, halting");
        return;
    }
    
    // Create status monitoring task
    xTaskCreate(status_task, "status", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "=== System Ready ===");
    ESP_LOGI(TAG, "Streaming to: Golioth Cloud");
#ifdef CONFIG_LAN_CLIENT_ENABLED
    ESP_LOGI(TAG, "Streaming to: LAN PC (%s:%d)", LAN_SERVER_IP, LAN_SERVER_PORT);
#endif
    ESP_LOGI(TAG, "Backup storage: SPIFFS (sequence recording)");
    ESP_LOGI(TAG, "RPC commands: relay_set, relay_get, relay_all_off");
    ESP_LOGI(TAG, "LAN commands: backup_*, relay_*, 106h_avg, 106h_log_*");
    
    // Keep main task alive - Golioth client task handles CoAP I/O
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
