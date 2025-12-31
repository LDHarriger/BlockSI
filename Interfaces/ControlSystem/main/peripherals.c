/**
 * @file peripherals.c
 * @brief Peripheral initialization implementation
 */

#include "peripherals.h"
#include "blocksi_pins.h"
#include "mcp4725_dac.h"
#include "dfrobot_ozone.h"
#include "max31855_thermocouple.h"

#include <string.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "PERIPH";

// Module state
static peripherals_status_t s_status = {0};

// ============================================================================
// I2C Bus Initialization
// ============================================================================

esp_err_t peripherals_init_i2c(void)
{
    if (s_status.i2c_initialized) {
        ESP_LOGD(TAG, "I2C already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing I2C bus");
    ESP_LOGI(TAG, "  SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO);
    ESP_LOGI(TAG, "  Frequency: %d Hz", I2C_MASTER_FREQ_HZ);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    s_status.i2c_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized");
    
    // Scan for devices
    peripherals_scan_i2c();
    
    return ESP_OK;
}

// ============================================================================
// SPI Bus Initialization
// ============================================================================

esp_err_t peripherals_init_spi(void)
{
    if (s_status.spi_initialized) {
        ESP_LOGD(TAG, "SPI already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing SPI bus");
    ESP_LOGI(TAG, "  SCK: GPIO%d, MISO: GPIO%d, MOSI: GPIO%d",
             SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .sclk_io_num = SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI_HOST_DEVICE, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    s_status.spi_initialized = true;
    ESP_LOGI(TAG, "SPI bus initialized");
    
    return ESP_OK;
}

// ============================================================================
// Device Initialization
// ============================================================================

esp_err_t peripherals_init_dac(void)
{
    if (!s_status.i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized, cannot init DAC");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_status.dac_initialized) {
        ESP_LOGD(TAG, "DAC already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing MCP4725 DAC for O3 power control");
    
    mcp4725_config_t dac_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .i2c_addr = I2C_ADDR_MCP4725,
        .vdd_voltage = DAC_VDD_VOLTAGE,
        .divider_ratio = DAC_DIVIDER_RATIO,
    };
    
    esp_err_t ret = mcp4725_init(&dac_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MCP4725 DAC not found at address 0x%02X", I2C_ADDR_MCP4725);
        return ret;
    }
    
    s_status.dac_initialized = true;
    ESP_LOGI(TAG, "MCP4725 DAC initialized");
    ESP_LOGI(TAG, "  Output voltage range: 0 - %.2fV", DAC_MAX_OUTPUT_V);
    
    return ESP_OK;
}

/**
 * @brief Alarm callback for lab O3 sensor
 */
static void lab_o3_alarm_handler(float level_ppm, float alarm_level)
{
    if (alarm_level >= LAB_O3_ALARM_CRITICAL) {
        ESP_LOGE(TAG, "!!! CRITICAL: Lab O3 = %.3f ppm - EVACUATE !!!", level_ppm);
        // TODO: Could trigger relay to shut off O3 generator
        // TODO: Could trigger external alarm
    } else if (alarm_level >= LAB_O3_ALARM_DANGER) {
        ESP_LOGW(TAG, "!! DANGER: Lab O3 = %.3f ppm - Increase ventilation !!", level_ppm);
    } else if (alarm_level >= LAB_O3_ALARM_WARNING) {
        ESP_LOGW(TAG, "* WARNING: Lab O3 = %.3f ppm - Check for leaks *", level_ppm);
    }
}

esp_err_t peripherals_init_lab_o3(void)
{
    if (!s_status.i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized, cannot init lab O3 sensor");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_status.lab_o3_initialized) {
        ESP_LOGD(TAG, "Lab O3 sensor already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing DFRobot lab O3 sensor");
    
    dfrobot_o3_config_t o3_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .i2c_addr = I2C_ADDR_DFROBOT_O3,
        .sample_interval_ms = LAB_O3_SAMPLE_INTERVAL_MS,
        .alarm_callback = lab_o3_alarm_handler,
    };
    
    esp_err_t ret = dfrobot_o3_init(&o3_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DFRobot O3 sensor not found at address 0x%02X", I2C_ADDR_DFROBOT_O3);
        return ret;
    }
    
    s_status.lab_o3_initialized = true;
    ESP_LOGI(TAG, "Lab O3 sensor initialized");
    
    return ESP_OK;
}

esp_err_t peripherals_init_thermocouple(void)
{
    if (!s_status.spi_initialized) {
        // Initialize SPI if not already done
        esp_err_t ret = peripherals_init_spi();
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    if (s_status.thermocouple_initialized) {
        ESP_LOGD(TAG, "Thermocouple already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing MAX31855 thermocouple");
    
    max31855_config_t thermo_cfg = {
        .spi_host = SPI_HOST_DEVICE,
        .cs_gpio = SPI_CS_THERMOCOUPLE,
        .sck_gpio = -1,     // Already configured by spi_bus_initialize
        .miso_gpio = -1,    // Already configured by spi_bus_initialize
    };
    
    esp_err_t ret = max31855_init(&thermo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MAX31855 thermocouple not responding");
        return ret;
    }
    
    s_status.thermocouple_initialized = true;
    ESP_LOGI(TAG, "Thermocouple initialized");
    
    return ESP_OK;
}

// ============================================================================
// Initialization All
// ============================================================================

esp_err_t peripherals_init_all(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing all peripherals");
    ESP_LOGI(TAG, "========================================");
    
    esp_err_t overall_ret = ESP_OK;
    
    // Initialize I2C bus first
    esp_err_t ret = peripherals_init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed - I2C devices will not work");
        overall_ret = ESP_ERR_NOT_FOUND;
    }
    
    // Initialize SPI bus
    ret = peripherals_init_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed - SPI devices will not work");
        overall_ret = ESP_ERR_NOT_FOUND;
    }
    
    // Initialize MCP4725 DAC
    ret = peripherals_init_dac();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DAC init failed - O3 power control unavailable");
        overall_ret = ESP_ERR_NOT_FOUND;
    }
    
    // Initialize lab O3 sensor
    ret = peripherals_init_lab_o3();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Lab O3 sensor init failed - safety monitoring unavailable");
        overall_ret = ESP_ERR_NOT_FOUND;
    }
    
    // Initialize thermocouple
    ret = peripherals_init_thermocouple();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Thermocouple init failed - temperature monitoring unavailable");
        overall_ret = ESP_ERR_NOT_FOUND;
    }
    
    // Print summary
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Peripheral initialization summary:");
    ESP_LOGI(TAG, "  I2C bus:      %s", s_status.i2c_initialized ? "OK" : "FAIL");
    ESP_LOGI(TAG, "  SPI bus:      %s", s_status.spi_initialized ? "OK" : "FAIL");
    ESP_LOGI(TAG, "  MCP4725 DAC:  %s", s_status.dac_initialized ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  Lab O3:       %s", s_status.lab_o3_initialized ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  Thermocouple: %s", s_status.thermocouple_initialized ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "========================================");
    
    return overall_ret;
}

// ============================================================================
// Status and Utilities
// ============================================================================

esp_err_t peripherals_get_status(peripherals_status_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *status = s_status;
    return ESP_OK;
}

void peripherals_scan_i2c(void)
{
    if (!s_status.i2c_initialized) {
        ESP_LOGW(TAG, "I2C not initialized, cannot scan");
        return;
    }
    
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    int devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        // Try to read one byte from address
        uint8_t data;
        esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_PORT, addr, &data, 1, 
                                                     pdMS_TO_TICKS(50));
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            devices_found++;
            
            // Identify known devices
            if (addr == I2C_ADDR_MCP4725) {
                ESP_LOGI(TAG, "    → MCP4725 DAC");
            } else if (addr == I2C_ADDR_MCP4725 + 1) {
                ESP_LOGI(TAG, "    → MCP4725 DAC (A0=1)");
            } else if (addr == I2C_ADDR_DFROBOT_O3) {
                ESP_LOGI(TAG, "    → DFRobot O3 Sensor");
            }
        }
    }
    
    ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", devices_found);
}

void peripherals_deinit_all(void)
{
    ESP_LOGI(TAG, "Deinitializing all peripherals");
    
    if (s_status.thermocouple_initialized) {
        max31855_deinit();
        s_status.thermocouple_initialized = false;
    }
    
    if (s_status.lab_o3_initialized) {
        dfrobot_o3_deinit();
        s_status.lab_o3_initialized = false;
    }
    
    if (s_status.dac_initialized) {
        mcp4725_deinit();
        s_status.dac_initialized = false;
    }
    
    if (s_status.spi_initialized) {
        spi_bus_free(SPI_HOST_DEVICE);
        s_status.spi_initialized = false;
    }
    
    if (s_status.i2c_initialized) {
        i2c_driver_delete(I2C_MASTER_PORT);
        s_status.i2c_initialized = false;
    }
    
    ESP_LOGI(TAG, "All peripherals deinitialized");
}
