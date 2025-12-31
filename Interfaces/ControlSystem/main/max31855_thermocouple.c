/**
 * @file max31855_thermocouple.c
 * @brief MAX31855 K-type thermocouple interface implementation
 */

#include "max31855_thermocouple.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "THERMO";

// SPI configuration
#define MAX31855_SPI_CLOCK_HZ   1000000  // 1 MHz (max 5 MHz)

// Module state
static struct {
    bool initialized;
    spi_device_handle_t spi_handle;
    max31855_reading_t last_reading;
    uint32_t reading_count;
    uint32_t fault_count;
} s_thermo = {0};

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Read 32-bit data from MAX31855
 */
static esp_err_t read_32bit(uint32_t *data)
{
    uint8_t rx_data[4] = {0};
    
    spi_transaction_t trans = {
        .length = 32,       // 32 bits to receive
        .rxlength = 32,
        .rx_buffer = rx_data,
        .tx_buffer = NULL,
    };
    
    esp_err_t ret = spi_device_transmit(s_thermo.spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Combine bytes (MSB first)
    *data = ((uint32_t)rx_data[0] << 24) |
            ((uint32_t)rx_data[1] << 16) |
            ((uint32_t)rx_data[2] << 8) |
            ((uint32_t)rx_data[3]);
    
    ESP_LOGD(TAG, "Raw data: 0x%08lX", (unsigned long)*data);
    
    return ESP_OK;
}

/**
 * @brief Parse 32-bit data into reading structure
 * 
 * Data format:
 * Bits 31-18: 14-bit thermocouple temperature (0.25°C resolution)
 * Bit 17: Reserved
 * Bit 16: Fault bit
 * Bits 15-4: 12-bit cold junction temperature (0.0625°C resolution)
 * Bit 3: Reserved
 * Bit 2: SCV (short to VCC)
 * Bit 1: SCG (short to GND)
 * Bit 0: OC (open circuit)
 */
static void parse_reading(uint32_t raw, max31855_reading_t *reading)
{
    // Clear reading
    memset(reading, 0, sizeof(*reading));
    
    // Check fault bit (bit 16)
    if (raw & 0x00010000) {
        reading->valid = false;
        
        // Parse fault bits
        reading->fault = MAX31855_FAULT_NONE;
        if (raw & 0x01) reading->fault |= MAX31855_FAULT_OPEN;
        if (raw & 0x02) reading->fault |= MAX31855_FAULT_SHORT_GND;
        if (raw & 0x04) reading->fault |= MAX31855_FAULT_SHORT_VCC;
        
        ESP_LOGW(TAG, "Fault detected: %s", max31855_fault_to_string(reading->fault));
        return;
    }
    
    reading->valid = true;
    reading->fault = MAX31855_FAULT_NONE;
    
    // Parse thermocouple temperature (bits 31-18)
    // 14-bit signed value, 0.25°C resolution
    int16_t tc_raw = (raw >> 18) & 0x3FFF;
    if (tc_raw & 0x2000) {
        // Negative temperature (sign extend)
        tc_raw |= 0xC000;
    }
    reading->thermocouple_c = tc_raw * 0.25f;
    
    // Parse cold junction temperature (bits 15-4)
    // 12-bit signed value, 0.0625°C resolution
    int16_t cj_raw = (raw >> 4) & 0x0FFF;
    if (cj_raw & 0x0800) {
        // Negative temperature (sign extend)
        cj_raw |= 0xF000;
    }
    reading->cold_junction_c = cj_raw * 0.0625f;
    
    ESP_LOGD(TAG, "TC: %.2f°C, CJ: %.2f°C", 
             reading->thermocouple_c, reading->cold_junction_c);
}

// ============================================================================
// Public API - Initialization
// ============================================================================

esp_err_t max31855_init(const max31855_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_thermo.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing MAX31855 thermocouple interface");
    ESP_LOGI(TAG, "  SPI host: %d", config->spi_host);
    ESP_LOGI(TAG, "  CS GPIO: %d", config->cs_gpio);
    
    // Configure SPI bus if pins specified
    if (config->sck_gpio >= 0 && config->miso_gpio >= 0) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = -1,  // Not used (MAX31855 is read-only)
            .miso_io_num = config->miso_gpio,
            .sclk_io_num = config->sck_gpio,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4,
        };
        
        esp_err_t ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_DISABLED);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            // ESP_ERR_INVALID_STATE means bus already initialized, which is OK
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "  SCK GPIO: %d", config->sck_gpio);
        ESP_LOGI(TAG, "  MISO GPIO: %d", config->miso_gpio);
    }
    
    // Add device to SPI bus
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = MAX31855_SPI_CLOCK_HZ,
        .mode = 0,  // CPOL=0, CPHA=0
        .spics_io_num = config->cs_gpio,
        .queue_size = 1,
        .flags = 0,
    };
    
    esp_err_t ret = spi_bus_add_device(config->spi_host, &dev_cfg, &s_thermo.spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize last reading
    s_thermo.last_reading.valid = false;
    s_thermo.last_reading.thermocouple_c = NAN;
    s_thermo.last_reading.cold_junction_c = NAN;
    s_thermo.reading_count = 0;
    s_thermo.fault_count = 0;
    
    s_thermo.initialized = true;
    
    // Check if device is present
    if (!max31855_is_present()) {
        ESP_LOGW(TAG, "MAX31855 not responding or thermocouple not connected");
    } else {
        // Take initial reading
        max31855_reading_t reading;
        if (max31855_read_full(&reading) == ESP_OK && reading.valid) {
            ESP_LOGI(TAG, "Initial reading: TC=%.2f°C, CJ=%.2f°C",
                     reading.thermocouple_c, reading.cold_junction_c);
        }
    }
    
    ESP_LOGI(TAG, "MAX31855 initialized");
    return ESP_OK;
}

void max31855_deinit(void)
{
    if (!s_thermo.initialized) {
        return;
    }
    
    if (s_thermo.spi_handle) {
        spi_bus_remove_device(s_thermo.spi_handle);
        s_thermo.spi_handle = NULL;
    }
    
    s_thermo.initialized = false;
    ESP_LOGI(TAG, "MAX31855 deinitialized");
}

bool max31855_is_present(void)
{
    if (!s_thermo.initialized) {
        return false;
    }
    
    uint32_t raw;
    if (read_32bit(&raw) != ESP_OK) {
        return false;
    }
    
    // Check for valid data pattern
    // All 1s (0xFFFFFFFF) or all 0s (0x00000000) indicate no device
    if (raw == 0xFFFFFFFF || raw == 0x00000000) {
        ESP_LOGD(TAG, "Invalid data pattern: 0x%08lX", (unsigned long)raw);
        return false;
    }
    
    return true;
}

// ============================================================================
// Public API - Temperature Reading
// ============================================================================

esp_err_t max31855_read_temp(float *temp_c)
{
    if (!s_thermo.initialized || !temp_c) {
        return ESP_ERR_INVALID_STATE;
    }
    
    max31855_reading_t reading;
    esp_err_t ret = max31855_read_full(&reading);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (!reading.valid) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    *temp_c = reading.thermocouple_c;
    return ESP_OK;
}

esp_err_t max31855_read_full(max31855_reading_t *reading)
{
    if (!s_thermo.initialized || !reading) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t raw;
    esp_err_t ret = read_32bit(&raw);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse the raw data
    parse_reading(raw, reading);
    
    // Update statistics
    s_thermo.reading_count++;
    if (!reading->valid) {
        s_thermo.fault_count++;
    }
    
    // Store last reading
    s_thermo.last_reading = *reading;
    
    return ESP_OK;
}

esp_err_t max31855_read_raw(uint32_t *raw)
{
    if (!s_thermo.initialized || !raw) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return read_32bit(raw);
}

float max31855_get_last(void)
{
    if (!s_thermo.last_reading.valid) {
        return NAN;
    }
    return s_thermo.last_reading.thermocouple_c;
}

max31855_fault_t max31855_get_last_fault(void)
{
    return s_thermo.last_reading.fault;
}

const char* max31855_fault_to_string(max31855_fault_t fault)
{
    if (fault == MAX31855_FAULT_NONE) {
        return "No fault";
    }
    
    static char buf[64];
    buf[0] = '\0';
    
    if (fault & MAX31855_FAULT_OPEN) {
        strcat(buf, "Open circuit ");
    }
    if (fault & MAX31855_FAULT_SHORT_GND) {
        strcat(buf, "Short to GND ");
    }
    if (fault & MAX31855_FAULT_SHORT_VCC) {
        strcat(buf, "Short to VCC ");
    }
    
    return buf;
}
