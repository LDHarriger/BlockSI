/**
 * @file sensor_aggregator.c
 * @brief Sensor aggregation implementation
 */

#include "sensor_aggregator.h"
#include "dfrobot_ozone.h"
#include "max31855_thermocouple.h"
#include "peripherals.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "SENSOR_AGG";

// Module state
static struct {
    bool initialized;
    uint32_t sample_interval_ms;
    TaskHandle_t sample_task;
    SemaphoreHandle_t mutex;
    
    // Room O3 accumulator
    float room_o3_sum;
    float room_o3_min;
    float room_o3_max;
    uint16_t room_o3_count;
    float room_o3_last;
    bool room_o3_connected;
    
    // Vessel temperature accumulator
    float vessel_temp_sum;
    float vessel_temp_min;
    float vessel_temp_max;
    uint16_t vessel_temp_count;
    float vessel_temp_last;
    float ambient_temp_last;
    bool vessel_temp_connected;
    
} s_agg = {0};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Sample room O3 sensor
 */
static void sample_room_o3(void)
{
    float ppm;
    esp_err_t ret = dfrobot_o3_read(&ppm);
    
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    
    if (ret == ESP_OK) {
        s_agg.room_o3_connected = true;
        s_agg.room_o3_last = ppm;
        s_agg.room_o3_sum += ppm;
        s_agg.room_o3_count++;
        
        if (s_agg.room_o3_count == 1) {
            s_agg.room_o3_min = ppm;
            s_agg.room_o3_max = ppm;
        } else {
            if (ppm < s_agg.room_o3_min) s_agg.room_o3_min = ppm;
            if (ppm > s_agg.room_o3_max) s_agg.room_o3_max = ppm;
        }
        
        ESP_LOGD(TAG, "Room O3: %.3f ppm (n=%d)", ppm, s_agg.room_o3_count);
    } else {
        s_agg.room_o3_connected = false;
        ESP_LOGD(TAG, "Room O3 read failed");
    }
    
    xSemaphoreGive(s_agg.mutex);
}

/**
 * @brief Sample thermocouple
 */
static void sample_thermocouple(void)
{
    max31855_reading_t reading;
    esp_err_t ret = max31855_read_full(&reading);
    
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    
    if (ret == ESP_OK && reading.valid) {
        s_agg.vessel_temp_connected = true;
        s_agg.vessel_temp_last = reading.thermocouple_c;
        s_agg.ambient_temp_last = reading.cold_junction_c;
        s_agg.vessel_temp_sum += reading.thermocouple_c;
        s_agg.vessel_temp_count++;
        
        if (s_agg.vessel_temp_count == 1) {
            s_agg.vessel_temp_min = reading.thermocouple_c;
            s_agg.vessel_temp_max = reading.thermocouple_c;
        } else {
            if (reading.thermocouple_c < s_agg.vessel_temp_min) 
                s_agg.vessel_temp_min = reading.thermocouple_c;
            if (reading.thermocouple_c > s_agg.vessel_temp_max) 
                s_agg.vessel_temp_max = reading.thermocouple_c;
        }
        
        ESP_LOGD(TAG, "Vessel temp: %.2f°C (n=%d)", reading.thermocouple_c, s_agg.vessel_temp_count);
    } else {
        s_agg.vessel_temp_connected = false;
        if (ret == ESP_OK && !reading.valid) {
            ESP_LOGD(TAG, "Thermocouple fault: %s", max31855_fault_to_string(reading.fault));
        } else {
            ESP_LOGD(TAG, "Thermocouple read failed");
        }
    }
    
    xSemaphoreGive(s_agg.mutex);
}

/**
 * @brief Background sampling task
 */
static void sample_task(void *arg)
{
    peripherals_status_t status;
    peripherals_get_status(&status);
    
    ESP_LOGI(TAG, "Sensor sampling task started");
    ESP_LOGI(TAG, "  Lab O3 sensor: %s", status.lab_o3_initialized ? "available" : "not available");
    ESP_LOGI(TAG, "  Thermocouple: %s", status.thermocouple_initialized ? "available" : "not available");
    
    while (1) {
        // Sample room O3 if available
        if (status.lab_o3_initialized) {
            sample_room_o3();
        }
        
        // Sample thermocouple if available
        if (status.thermocouple_initialized) {
            sample_thermocouple();
        }
        
        vTaskDelay(pdMS_TO_TICKS(s_agg.sample_interval_ms));
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t sensor_aggregator_init(uint32_t sample_interval_ms)
{
    if (s_agg.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing sensor aggregator");
    ESP_LOGI(TAG, "  Sample interval: %lu ms", (unsigned long)sample_interval_ms);
    
    s_agg.mutex = xSemaphoreCreateMutex();
    if (!s_agg.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_agg.sample_interval_ms = sample_interval_ms;
    
    // Initialize accumulators
    s_agg.room_o3_sum = 0;
    s_agg.room_o3_count = 0;
    s_agg.room_o3_min = 0;
    s_agg.room_o3_max = 0;
    s_agg.room_o3_last = -1;
    s_agg.room_o3_connected = false;
    
    s_agg.vessel_temp_sum = 0;
    s_agg.vessel_temp_count = 0;
    s_agg.vessel_temp_min = 0;
    s_agg.vessel_temp_max = 0;
    s_agg.vessel_temp_last = NAN;
    s_agg.ambient_temp_last = NAN;
    s_agg.vessel_temp_connected = false;
    
    // Start sampling task
    BaseType_t ret = xTaskCreate(sample_task, "sensor_agg", 3072, NULL, 4, &s_agg.sample_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampling task");
        vSemaphoreDelete(s_agg.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    s_agg.initialized = true;
    ESP_LOGI(TAG, "Sensor aggregator initialized");
    
    return ESP_OK;
}

void sensor_aggregator_deinit(void)
{
    if (!s_agg.initialized) {
        return;
    }
    
    if (s_agg.sample_task) {
        vTaskDelete(s_agg.sample_task);
        s_agg.sample_task = NULL;
    }
    
    if (s_agg.mutex) {
        vSemaphoreDelete(s_agg.mutex);
        s_agg.mutex = NULL;
    }
    
    s_agg.initialized = false;
    ESP_LOGI(TAG, "Sensor aggregator deinitialized");
}

esp_err_t sensor_aggregator_get_and_reset(aggregated_sensors_t *sensors)
{
    if (!sensors) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_agg.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    
    // Room O3
    sensors->room_o3_valid = s_agg.room_o3_connected && (s_agg.room_o3_count > 0);
    if (sensors->room_o3_valid) {
        sensors->room_o3_ppm = s_agg.room_o3_sum / s_agg.room_o3_count;
        sensors->room_o3_min = s_agg.room_o3_min;
        sensors->room_o3_max = s_agg.room_o3_max;
        sensors->room_o3_count = s_agg.room_o3_count;
    } else {
        sensors->room_o3_ppm = 0;
        sensors->room_o3_min = 0;
        sensors->room_o3_max = 0;
        sensors->room_o3_count = 0;
    }
    
    // Vessel temperature
    sensors->vessel_temp_valid = s_agg.vessel_temp_connected && (s_agg.vessel_temp_count > 0);
    if (sensors->vessel_temp_valid) {
        sensors->vessel_temp_c = s_agg.vessel_temp_sum / s_agg.vessel_temp_count;
        sensors->vessel_temp_min = s_agg.vessel_temp_min;
        sensors->vessel_temp_max = s_agg.vessel_temp_max;
        sensors->vessel_temp_count = s_agg.vessel_temp_count;
        sensors->ambient_temp_c = s_agg.ambient_temp_last;
        sensors->ambient_temp_valid = true;
    } else {
        sensors->vessel_temp_c = 0;
        sensors->vessel_temp_min = 0;
        sensors->vessel_temp_max = 0;
        sensors->vessel_temp_count = 0;
        sensors->ambient_temp_c = 0;
        sensors->ambient_temp_valid = false;
    }
    
    // Reset accumulators
    s_agg.room_o3_sum = 0;
    s_agg.room_o3_count = 0;
    s_agg.room_o3_min = 0;
    s_agg.room_o3_max = 0;
    
    s_agg.vessel_temp_sum = 0;
    s_agg.vessel_temp_count = 0;
    s_agg.vessel_temp_min = 0;
    s_agg.vessel_temp_max = 0;
    
    xSemaphoreGive(s_agg.mutex);
    
    return ESP_OK;
}

esp_err_t sensor_aggregator_peek(aggregated_sensors_t *sensors)
{
    if (!sensors) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_agg.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    
    // Room O3
    sensors->room_o3_valid = s_agg.room_o3_connected && (s_agg.room_o3_count > 0);
    if (sensors->room_o3_valid) {
        sensors->room_o3_ppm = s_agg.room_o3_sum / s_agg.room_o3_count;
        sensors->room_o3_min = s_agg.room_o3_min;
        sensors->room_o3_max = s_agg.room_o3_max;
        sensors->room_o3_count = s_agg.room_o3_count;
    }
    
    // Vessel temperature
    sensors->vessel_temp_valid = s_agg.vessel_temp_connected && (s_agg.vessel_temp_count > 0);
    if (sensors->vessel_temp_valid) {
        sensors->vessel_temp_c = s_agg.vessel_temp_sum / s_agg.vessel_temp_count;
        sensors->vessel_temp_min = s_agg.vessel_temp_min;
        sensors->vessel_temp_max = s_agg.vessel_temp_max;
        sensors->vessel_temp_count = s_agg.vessel_temp_count;
        sensors->ambient_temp_c = s_agg.ambient_temp_last;
        sensors->ambient_temp_valid = true;
    }
    
    xSemaphoreGive(s_agg.mutex);
    
    return ESP_OK;
}

bool sensor_aggregator_room_o3_alarm(void)
{
    return dfrobot_o3_alarm_is_active();
}

float sensor_aggregator_get_room_o3(void)
{
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    float val = s_agg.room_o3_connected ? s_agg.room_o3_last : -1.0f;
    xSemaphoreGive(s_agg.mutex);
    return val;
}

float sensor_aggregator_get_vessel_temp(void)
{
    xSemaphoreTake(s_agg.mutex, portMAX_DELAY);
    float val = s_agg.vessel_temp_connected ? s_agg.vessel_temp_last : NAN;
    xSemaphoreGive(s_agg.mutex);
    return val;
}
