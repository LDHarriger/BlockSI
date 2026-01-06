# BlockSI Control System - Main Component CMakeLists.txt
# Add this to Interfaces/ControlSystem/main/CMakeLists.txt

idf_component_register(
    SRCS 
        "main.c"
        "model_106h_interface.c"
        "relay_control.c"
        "lan_client.c"
        "backup_storage.c"
        "peripherals.c"
        "ds3502_digipot.c"          # NEW: DS3502 driver
        "o3_power_control.c"        # NEW: Power control using DS3502
        "dfrobot_ozone.c"
        "max31855_thermocouple.c"
        "sensor_aggregator.c"
        "dosimetry.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver 
        esp_timer 
        nvs_flash 
        esp_wifi 
        esp_netif 
        golioth_sdk
        spiffs
        esp_partition
)

# Note: Removed mcp4725_dac.c (replaced by ds3502_digipot.c)
# Note: Removed dac_power_test.c (standalone test, not part of main app)
