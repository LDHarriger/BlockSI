# BlockSI Control System Firmware

Production firmware for BlockSI sterilization monitoring system.

## Features

- **106-H Ozone Monitor Interface**: RS232/UART communication at 19200 baud
- **WiFi Connectivity**: Automatic connection with retry logic
- **Golioth Cloud Integration**: PSK-authenticated MQTT/TLS connection
- **Real-time Data Streaming**: JSON telemetry to LightDB Stream
- **Status Monitoring**: Periodic statistics and health reporting

## Hardware Requirements

### Components
- ESP32-WROOM-32 (30-pin)
- 2B Technologies Model 106-H Ozone Monitor
- RS232 Pal (or equivalent MAX3232 level shifter)
- 5V power supply for ESP32

### Connections

```
106-H D9 Connector ←→ RS232 Pal ←→ ESP32
    Pin 3 (TX)     →    RXD      →   GPIO16 (UART2_RX)
    Pin 2 (RX)     ←    TXD      ←   GPIO17 (UART2_TX)
    Pin 5 (GND)    ←→   GND      ←→  GND
```

**Important**: 106-H has a male D9 connector with *female pinout* (non-standard). The provided cable or RS232 Pal handles this correctly.

### 106-H Configuration
- Set output mode to "Ser" (RS232 output)
- Baud rate: 19200 (fixed, cannot be changed)
- Data format: 8N1
- Averaging period: 2-10 seconds recommended

## Software Requirements

- ESP-IDF v5.4.0 or later (tested with v5.4.3)
- Python 3.8+ (included with ESP-IDF)
- Git (for component manager)

## Quick Start

### 1. Clone Repository

```powershell
cd C:\Users\<username>\Documents\
git clone <repository-url> BlockSI_Firmware
cd BlockSI_Firmware
```

### 2. Set Up Golioth Account

1. Create account at [console.golioth.io](https://console.golioth.io)
2. Create a new project
3. Add a device with PSK authentication
4. Copy the PSK-ID and PSK values (you'll need them for configuration)

### 3. Configure Project

```powershell
idf.py menuconfig
```

Navigate through menus and configure:

**BlockSI Configuration → WiFi Configuration**
- WiFi SSID: Your network name
- WiFi Password: Your network password

**BlockSI Configuration → Golioth Configuration**
- Golioth PSK-ID: `device-name@project-name` (from Golioth console)
- Golioth PSK: Hex string (from Golioth console)

**BlockSI Configuration → 106-H Configuration**
- UART Port Number: 2 (UART2)
- TX GPIO Pin: 17
- RX GPIO Pin: 16

Save and exit (S, then Enter, then Q).

### 4. Build

```powershell
idf.py build
```

### 5. Flash

```powershell
idf.py -p COM3 flash
```

Replace `COM3` with your actual COM port.

### 6. Monitor

```powershell
idf.py -p COM3 monitor
```

Exit monitor with `Ctrl+]`

## Data Format

### 106-H CSV Input
```
0.22,27.8,1019.9,1.15174,1.05427,24/11/25,19:08:41
```

Fields:
1. Ozone (wt%)
2. Temperature (°C)
3. Pressure (mbar)
4. Sample PDV (V)
5. Reference PDV (V)
6. Date (DD/MM/YY)
7. Time (HH:MM:SS)

### Golioth JSON Output

Published to `sensor/ozone` stream:

```json
{
  "ozone_wt_pct": 0.220,
  "temperature_c": 27.80,
  "pressure_mbar": 1019.90,
  "sample_pdv_v": 1.15174,
  "ref_pdv_v": 1.05427,
  "timestamp": "24/11/25 19:08:41"
}
```

## Monitoring Data in Golioth

1. Log into [console.golioth.io](https://console.golioth.io)
2. Navigate to your project
3. Click on your device
4. Go to "LightDB Stream" tab
5. View real-time data under `sensor/ozone` path

You should see new data points appearing every 2-10 seconds (depending on 106-H averaging setting).

## Expected Serial Output

```
I (234) BLOCKSI: === BlockSI Control System ===
I (234) BLOCKSI: Firmware version: 1.0.0
I (234) BLOCKSI: Initializing WiFi...
I (1234) BLOCKSI: WiFi connected, IP: 192.168.1.100
I (1240) BLOCKSI: Initializing Golioth...
I (1245) BLOCKSI: PSK-ID: blocksi-001@my-project
I (2456) BLOCKSI: Golioth client connected
I (2460) BLOCKSI: Initializing 106-H interface...
I (2465) 106H: UART2: TX=GPIO17, RX=GPIO16, Baud=19200
I (2470) 106H: 106-H interface initialized successfully
I (2475) BLOCKSI: === System Ready ===
I (4567) 106H: O3: 0.22 wt%, Temp: 27.8°C, Press: 1019.9 mbar
I (4568) BLOCKSI: Published: O3=0.22 wt%, T=27.8°C, P=1019.9 mbar
```

## Troubleshooting

### No Data from 106-H

**Check hardware:**
- Verify RS232 Pal is powered (3.3V or 5V depending on model)
- Check wiring: GPIO16 → RX, GPIO17 → TX
- Verify ground connection
- Test 106-H USB output to confirm device is working

**Check 106-H configuration:**
- Output mode must be "Ser" (not USB only)
- Baud rate: 19200
- Use provided cable or verify pinout

**Check ESP32 logs:**
```powershell
idf.py -p COM3 monitor
```
Look for "UART event task started" and parse errors.

### WiFi Connection Failed

- Verify SSID and password in menuconfig
- Check WiFi signal strength
- Ensure 2.4 GHz network (ESP32 doesn't support 5 GHz)
- Check for special characters in password

### Golioth Connection Failed

- Verify PSK-ID format: `device-name@project-name`
- Ensure PSK is correct hex string (no spaces or dashes)
- Check device is not deleted in Golioth console
- Verify WiFi is connected first

### Parse Errors

If you see "Parse error" in logs:
- Check baud rate matches (19200)
- Verify 106-H is in "Ser" mode
- Check for loose connections
- Ensure proper ground connection

## System Architecture

```
┌─────────────────────────────────────────┐
│         106-H Ozone Monitor             │
│     (CSV @ 19200 baud, RS232)           │
└────────────┬────────────────────────────┘
             │
             │ RS232 (±12V logic)
             ↓
┌─────────────────────────────────────────┐
│          RS232 Pal Board                │
│      (MAX3232 level shifter)            │
└────────────┬────────────────────────────┘
             │
             │ TTL (0-3.3V logic)
             ↓
┌─────────────────────────────────────────┐
│          ESP32-WROOM-32                 │
│                                          │
│  ┌──────────────────────────────────┐  │
│  │  UART2 (GPIO16/17)               │  │
│  │    ↓                              │  │
│  │  CSV Parser                       │  │
│  │    ↓                              │  │
│  │  Sample Callback                  │  │
│  │    ↓                              │  │
│  │  JSON Builder                     │  │
│  │    ↓                              │  │
│  │  Golioth SDK                      │  │
│  └──────────────────────────────────┘  │
│             │                            │
│             │ WiFi / MQTT+TLS            │
└─────────────┼────────────────────────────┘
              │
              ↓
┌─────────────────────────────────────────┐
│         Golioth Cloud                   │
│      (LightDB Stream API)               │
└─────────────────────────────────────────┘
              │
              ↓
┌─────────────────────────────────────────┐
│    Dashboard / Data Analysis            │
└─────────────────────────────────────────┘
```

## File Structure

```
blocksi_firmware/
├── CMakeLists.txt                 # Project build configuration
├── README.md                      # This file
└── main/
    ├── CMakeLists.txt             # Component build configuration
    ├── idf_component.yml          # Golioth SDK dependency
    ├── Kconfig.projbuild          # Configuration menu
    ├── main.c                     # Application entry point
    ├── model_106h_interface.h     # 106-H driver interface
    └── model_106h_interface.c     # 106-H driver implementation
```

## Performance Notes

- **Sample rate**: Matches 106-H averaging period (2-10 seconds typical)
- **Network overhead**: ~512 bytes per sample (JSON + MQTT headers)
- **Memory usage**: ~50 KB for WiFi stack, minimal for application
- **CPU usage**: <5% average (dominated by WiFi/TLS)

## Future Enhancements

Planned features for subsequent sprints:
- Offline data queueing (RAM-based circular buffer)
- Additional sensors (temperature, humidity, flow)
- Ozone generator control (MP-8000)
- Remote configuration via Golioth
- OTA firmware updates
- SD card logging (optional)

## License

Copyright 2025 BlockSI / Shroom-E Co.

## Support

For issues or questions:
- 106-H hardware: support@twobtech.com
- Golioth platform: https://docs.golioth.io/
- ESP-IDF framework: https://docs.espressif.com/

## Version History

- **1.0.0** (2025-12-04): Initial release
  - 106-H RS232 interface
  - WiFi connectivity
  - Golioth cloud integration
  - LightDB Stream publishing
