/**
 * @file dfrobot_ozone.c
 * @brief DFRobot Gravity I2C Ozone Sensor implementation
 * 
 * Based on DFRobot SEN0321 sensor protocol.
 */

#include "dfrobot_ozone.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LAB_O3";

// DFRobot ozone sensor registers
#define REG_PASSIVE_MODE        0x00    // Set passive mode
#define REG_ACTIVE_MODE         0x01    // Set active mode  
#define REG_READ_O3             0x03    // Read O3 concentration
#define REG_AUTO_ACTIVE         0x04    // Auto-active mode
#define REG_SET_ADDR            0x05    // Set I2C address

// Conversion factor: raw value to ppm
// Based on DFRobot documentation: 1 LSB = 10 ppb = 0.01 ppm
#define RAW_TO_PPM              0.01f

// Module state
static struct {
    bool initialized;
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    uint16_t sample_interval_ms;
    dfrobot_o3_alarm_callback_t alarm_callback;
    
    // Calibration
    float offset_ppm;
    
    // Statistics
    float last_reading_ppm;
    uint32_t last_reading_time_ms;
    float max_reading_ppm;
    float min_reading_ppm;
    uint32_t reading_count;
    
    // Alarm state
    bool alarm_enabled;
    float alarm_warning;
    float alarm_danger;
    float alarm_critical;
    float current_alarm_level;
    bool alarm_active;
    
    // Auto-sample task
    TaskHandle_t sample_task;
} s_sensor = {0};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Write command to sensor
 */
static esp_err_t sensor_write_cmd(uint8_t cmd)
{
    ESP_LOGD(TAG, "Write cmd: 0x%02X", cmd);
    
    esp_err_t ret = i2c_master_write_to_device(s_sensor.i2c_port, s_sensor.i2c_addr,
                                                &cmd, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Read data from sensor
 */
static esp_err_t sensor_read_data(uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_read_from_device(s_sensor.i2c_port, s_sensor.i2c_addr,
                                                 data, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Check alarm thresholds
 */
static void check_alarms(float ppm)
{
    if (!s_sensor.alarm_enabled) {
        return;
    }
    
    float prev_level = s_sensor.current_alarm_level;
    s_sensor.current_alarm_level = 0;
    s_sensor.alarm_active = false;
    
    if (ppm >= s_sensor.alarm_critical) {
        s_sensor.current_alarm_level = s_sensor.alarm_critical;
        s_sensor.alarm_active = true;
        ESP_LOGE(TAG, "*** CRITICAL O3 LEVEL: %.3f ppm ***", ppm);
    } else if (ppm >= s_sensor.alarm_danger) {
        s_sensor.current_alarm_level = s_sensor.alarm_danger;
        s_sensor.alarm_active = true;
        ESP_LOGW(TAG, "** DANGER O3 LEVEL: %.3f ppm **", ppm);
    } else if (ppm >= s_sensor.alarm_warning) {
        s_sensor.current_alarm_level = s_sensor.alarm_warning;
        s_sensor.alarm_active = true;
        ESP_LOGW(TAG, "* WARNING O3 LEVEL: %.3f ppm *", ppm);
    }
    
    // Call alarm callback if threshold crossed
    if (s_sensor.alarm_callback && s_sensor.current_alarm_level != prev_level) {
        s_sensor.alarm_callback(ppm, s_sensor.current_alarm_level);
    }
}

/**
 * @brief Auto-sample task
 */
static void sample_task(void *arg)
{
    ESP_LOGI(TAG, "Auto-sample task started (interval=%ums)", s_sensor.sample_interval_ms);
    
    while (1) {
        float ppm;
        if (dfrobot_o3_read(&ppm) == ESP_OK) {
            ESP_LOGD(TAG, "Auto-sample: %.3f ppm", ppm);
        }
        
        vTaskDelay(pdMS_TO_TICKS(s_sensor.sample_interval_ms));
    }
}

// ============================================================================
// Public API - Initialization
// ============================================================================

esp_err_t dfrobot_o3_init(const dfrobot_o3_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_sensor.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing DFRobot O3 sensor (lab safety monitor)");
    ESP_LOGI(TAG, "  I2C port: %d", config->i2c_port);
    ESP_LOGI(TAG, "  I2C address: 0x%02X", config->i2c_addr);
    ESP_LOGI(TAG, "  Sample interval: %u ms", config->sample_interval_ms);
    
    s_sensor.i2c_port = config->i2c_port;
    s_sensor.i2c_addr = config->i2c_addr;
    s_sensor.sample_interval_ms = config->sample_interval_ms;
    s_sensor.alarm_callback = config->alarm_callback;
    
    // Set default alarm levels
    s_sensor.alarm_warning = O3_ALARM_WARNING;
    s_sensor.alarm_danger = O3_ALARM_DANGER;
    s_sensor.alarm_critical = O3_ALARM_CRITICAL;
    s_sensor.alarm_enabled = true;
    
    // Initialize statistics
    s_sensor.offset_ppm = 0;
    s_sensor.last_reading_ppm = 0;
    s_sensor.max_reading_ppm = 0;
    s_sensor.min_reading_ppm = 999.0f;
    s_sensor.reading_count = 0;
    
    // Check if device is present
    if (!dfrobot_o3_is_present()) {
        ESP_LOGE(TAG, "Sensor not found at address 0x%02X", config->i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Sensor detected at address 0x%02X", config->i2c_addr);
    
    // Set passive mode (read on demand)
    esp_err_t ret = sensor_write_cmd(REG_PASSIVE_MODE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not set passive mode, continuing anyway");
    }
    
    // Allow sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Take initial reading
    float ppm;
    ret = dfrobot_o3_read(&ppm);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial reading: %.3f ppm", ppm);
    } else {
        ESP_LOGW(TAG, "Initial reading failed: %s", esp_err_to_name(ret));
    }
    
    s_sensor.initialized = true;
    
    // Start auto-sample task if interval specified
    if (config->sample_interval_ms > 0) {
        xTaskCreate(sample_task, "lab_o3", 2048, NULL, 3, &s_sensor.sample_task);
    }
    
    ESP_LOGI(TAG, "DFRobot O3 sensor initialized");
    ESP_LOGI(TAG, "Safety thresholds: Warning=%.2f, Danger=%.2f, Critical=%.2f ppm",
             s_sensor.alarm_warning, s_sensor.alarm_danger, s_sensor.alarm_critical);
    
    return ESP_OK;
}

void dfrobot_o3_deinit(void)
{
    if (!s_sensor.initialized) {
        return;
    }
    
    if (s_sensor.sample_task) {
        vTaskDelete(s_sensor.sample_task);
        s_sensor.sample_task = NULL;
    }
    
    s_sensor.initialized = false;
    ESP_LOGI(TAG, "DFRobot O3 sensor deinitialized");
}

bool dfrobot_o3_is_present(void)
{
    uint8_t data;
    esp_err_t ret = i2c_master_read_from_device(s_sensor.i2c_port, s_sensor.i2c_addr,
                                                 &data, 1, pdMS_TO_TICKS(50));
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Device present, probe returned: 0x%02X", data);
        return true;
    }
    return false;
}

// ============================================================================
// Public API - Measurement
// ============================================================================

esp_err_t dfrobot_o3_read(float *ppm)
{
    if (!ppm) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send read command
    esp_err_t ret = sensor_write_cmd(REG_READ_O3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Small delay for conversion
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Read 2 bytes (16-bit value)
    uint8_t data[2];
    ret = sensor_read_data(data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes (big-endian)
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    
    // Convert to ppm and apply offset
    float reading = (raw * RAW_TO_PPM) + s_sensor.offset_ppm;
    if (reading < 0) reading = 0;  // Can't have negative O3
    
    ESP_LOGD(TAG, "Raw: %u → %.3f ppm", raw, reading);
    
    // Update statistics
    s_sensor.last_reading_ppm = reading;
    s_sensor.last_reading_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_sensor.reading_count++;
    
    if (reading > s_sensor.max_reading_ppm) {
        s_sensor.max_reading_ppm = reading;
    }
    if (reading < s_sensor.min_reading_ppm) {
        s_sensor.min_reading_ppm = reading;
    }
    
    // Check alarm thresholds
    check_alarms(reading);
    
    *ppm = reading;
    return ESP_OK;
}

esp_err_t dfrobot_o3_read_raw(uint16_t *raw)
{
    if (!raw) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = sensor_write_cmd(REG_READ_O3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t data[2];
    ret = sensor_read_data(data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *raw = ((uint16_t)data[0] << 8) | data[1];
    ESP_LOGD(TAG, "Raw value: %u", *raw);
    
    return ESP_OK;
}

float dfrobot_o3_get_last(void)
{
    if (s_sensor.reading_count == 0) {
        return -1.0f;
    }
    return s_sensor.last_reading_ppm;
}

esp_err_t dfrobot_o3_get_status(dfrobot_o3_status_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    status->last_reading_ppm = s_sensor.last_reading_ppm;
    status->last_reading_time_ms = s_sensor.last_reading_time_ms;
    status->max_reading_ppm = s_sensor.max_reading_ppm;
    status->min_reading_ppm = s_sensor.min_reading_ppm;
    status->reading_count = s_sensor.reading_count;
    status->alarm_active = s_sensor.alarm_active;
    
    return ESP_OK;
}

void dfrobot_o3_reset_stats(void)
{
    s_sensor.max_reading_ppm = s_sensor.last_reading_ppm;
    s_sensor.min_reading_ppm = s_sensor.last_reading_ppm;
    ESP_LOGI(TAG, "Statistics reset");
}

// ============================================================================
// Public API - Calibration
// ============================================================================

esp_err_t dfrobot_o3_set_offset(float offset_ppm)
{
    s_sensor.offset_ppm = offset_ppm;
    ESP_LOGI(TAG, "Offset set to %.3f ppm", offset_ppm);
    return ESP_OK;
}

float dfrobot_o3_get_offset(void)
{
    return s_sensor.offset_ppm;
}

esp_err_t dfrobot_o3_calibrate_zero(void)
{
    ESP_LOGI(TAG, "Performing zero calibration...");
    
    // Take multiple readings and average
    float sum = 0;
    int count = 0;
    
    for (int i = 0; i < 10; i++) {
        float ppm;
        s_sensor.offset_ppm = 0;  // Temporarily remove offset
        
        if (dfrobot_o3_read(&ppm) == ESP_OK) {
            sum += ppm;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    if (count == 0) {
        ESP_LOGE(TAG, "Zero calibration failed: no valid readings");
        return ESP_FAIL;
    }
    
    float avg = sum / count;
    s_sensor.offset_ppm = -avg;  // Negate to zero out
    
    ESP_LOGI(TAG, "Zero calibration complete: offset = %.3f ppm", s_sensor.offset_ppm);
    
    return ESP_OK;
}

// ============================================================================
// Public API - Alarms
// ============================================================================

void dfrobot_o3_alarm_enable(bool enable)
{
    s_sensor.alarm_enabled = enable;
    ESP_LOGI(TAG, "Alarms %s", enable ? "enabled" : "disabled");
}

void dfrobot_o3_set_alarm_levels(float warning, float danger, float critical)
{
    s_sensor.alarm_warning = warning;
    s_sensor.alarm_danger = danger;
    s_sensor.alarm_critical = critical;
    ESP_LOGI(TAG, "Alarm levels: Warning=%.2f, Danger=%.2f, Critical=%.2f ppm",
             warning, danger, critical);
}

bool dfrobot_o3_alarm_is_active(void)
{
    return s_sensor.alarm_active;
}

float dfrobot_o3_get_alarm_level(void)
{
    return s_sensor.current_alarm_level;
}
