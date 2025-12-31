# BlockSI Control System - Main Component CMakeLists.txt
# Place this file in: main/CMakeLists.txt

idf_component_register(
    SRCS 
        "main.c"
        "model_106h_interface.c"
        "relay_control.c"
        "lan_client.c"
        "backup_storage.c"
        "dosimetry.c"
        "o3_power_control.c"
        "peripherals.c"
        "mcp4725_dac.c"
        "dfrobot_ozone.c"
        "max31855_thermocouple.c"
        "sensor_aggregator.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash 
        esp_wifi 
        esp_netif 
        esp_driver_uart 
        esp_driver_gpio
        esp_driver_spi
        driver 
        esp_timer 
        spiffs
    PRIV_REQUIRES 
        golioth_sdk
)
