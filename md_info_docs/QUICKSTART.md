# Quick Start

## 1. Install PlatformIO
```bash
pip install platformio
```
The ESP8266 platform and the ArduinoJson / WebSockets libraries download automatically on the first build.

## 2. Wire the hardware
- CD74HC4067 mux: COM→A0, S0-S3→D1-D4, VCC+EN→3.3 V, GND
- Voltage source → mux C0 (0-1 V scale in software)
- Thermistor divider → mux C1: 3.3 V → 100 kΩ → C1 → 100k NTC → GND
- microSD module: SCK→D5, MISO→D6, MOSI→D7, CS→D8, VCC 3.3 V, GND
- Heater (optional): D0 → AOD4144 gate (10 kΩ gate→GND), 12 V heater on the drain, common ground

Full details: [WIRING_GUIDE.md](WIRING_GUIDE.md)

## 3. Upload firmware and web page
```bash
make deploy                     # NodeMCU (env esp8266)
make deploy ENV=wemos_d1_mini   # Wemos D1 mini
```
This uploads the firmware, uploads the web page to internal flash, and opens the serial monitor at 115200 baud.

## 4. Insert the SD card and boot
Use a FAT32 card (Class 10 or faster). The serial monitor should show:
```
SD card initialized.
Created new data file: data_N.log
```

## 5. Connect and log
- WiFi: `ESP8266_VoltageLogger`, password `voltage123`
- Browser: `http://192.168.4.1`
- Logging starts **paused**. Press **Start Logging**, then **Stop Logging** when done.
- **Download Data** gets the current log file. The **Log Files on SD Card** panel lists all files.

## 6. Heater (optional)
- Set a target (default 45 °C, max 130 °C) and press **Enable PID**, or use manual ON/OFF
- Keep logging running while the heater is on so the over-temperature check stays active
- After an emergency shutdown, press **System Reset**

## Where to change settings
| Setting | File |
|---|---|
| WiFi SSID / password | `src/wifi_manager.cpp` |
| Sample / web update intervals | `src/main.cpp` |
| Mux pins, thermistor constants | `src/sensor_manager.cpp` |
| Buffer size, file rotation size | `src/data_manager.cpp` |
| PWM, PID gains, target, safety limits | `src/heater_controller.cpp` |

`config.h` lists all of these values for reference but is not compiled into the firmware.
