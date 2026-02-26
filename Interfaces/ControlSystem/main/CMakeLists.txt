# BlockSI Control System - Main Component CMakeLists.txt

idf_component_register(
    SRCS 
        "main.c"
        "model_106h_interface.c"
        "relay_control.c"
        "lan_client.c"
        "backup_storage.c"
        "peripherals.c"
        "motor_pot.c"               # Motorized potentiometer driver (PRM162 + DRV8833)
        "o3_power_control.c"        # Power control API (uses motor_pot)
        "dfrobot_ozone.c"
        "max31855_thermocouple.c"
        "sensor_aggregator.c"
        "dosimetry.c"
        "blocksi_state.c"           # Unified system state management
        "power_calibration_v2.c"    # Power calibration sweep (motor pot version)
        "sequence_runner.c"         # Generic autonomous sequence framework
        "seq_executor.c"            # Recipe-based step executor (replaces seq_power_cal/seq_airflow_val)
        "seq_sensor_adapter.c"      # 106-H sample counter for executor
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
        esp_adc                     # Required for ADC oneshot driver
)

# Hardware change log:
# - Removed ds3502_digipot.c (replaced by motor_pot.c due to ground isolation issues)
# - Removed mcp4725_dac.c (original DAC design, replaced by DS3502, now motor pot)
# 
# The motor pot solution uses a dual-gang PRM162 potentiometer where:
# - Section 1 floats with MP-8000 control circuit (~10V above earth ground)
# - Section 2 provides position feedback to ESP32 ADC (0-3.3V)
# - Motor driven by DRV8833 H-bridge with PWM speed control
