# Quick Start

## 1. Install PlatformIO
```bash
pip install platformio
```
The ESP8266 platform and the ArduinoJson / WebSockets / NeoPixelBus libraries download automatically on the first build.

## 2. Assemble and connect the PCB
- Fit the NodeMCU v2 in its socket (check the orientation against the silkscreen)
- PID Temp Sensor (100k NTC) → J4, AUX Temp Sensor (MF55 10k NTC) → J5 (selectable as the PID control sensor)
- microSD card in J6 (bottom side, push until it clicks)
- Heater (optional): 12 V supply → J1, heater → J2. The logic still runs from the NodeMCU's USB

Full details and bring-up checks: [WIRING_GUIDE.md](../WIRING_GUIDE.md)

## 3. Upload firmware and web page
```bash
make deploy                     # NodeMCU v2 on the PCB (env esp8266)
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
- The status LED shows the heater power (blue = 0 % → red = 100 %); it flashes orange after a heater timeout and yellow after any other emergency shutdown
- After an emergency shutdown, press **System Reset**

## Where to change settings
| Setting | File |
|---|---|
| WiFi SSID / password | `src/wifi_manager.cpp` |
| Sample / web update intervals | `src/main.cpp` |
| ADC pins, thermistor constants | `src/sensor_manager.cpp` |
| Status LED colours / brightness | `src/status_led.cpp` |
| Buffer size, file rotation size | `src/data_manager.cpp` |
| PWM, PID gains, target, safety limits | `src/heater_controller.cpp` |

`config.h` lists all of these values for reference but is not compiled into the firmware.
