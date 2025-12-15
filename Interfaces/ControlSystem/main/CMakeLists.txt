# Main component CMakeLists.txt

idf_component_register(
    SRCS 
        "main.c"
        "model_106h_interface.c"
        "relay_control.c"
        "lan_client.c"
        "backup_storage.c"
    INCLUDE_DIRS 
        "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_driver_uart
        driver
        esp_timer
        spiffs
    PRIV_REQUIRES
        golioth_sdk
)
