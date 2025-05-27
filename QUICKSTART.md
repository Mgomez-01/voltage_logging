# Quick Start Guide - ESP8266 Voltage Logger

## 1. Install Required Libraries
Open Arduino IDE → Tools → Manage Libraries, then install:
- **WebSocketsServer** by Markus Sattler
- **ArduinoJson** by Benoit Blanchon

## 2. Hardware Setup
- Connect voltage source (0-1V) to pin A0 on ESP8266
- Connect ESP8266 to computer via USB

## 3. Upload Code
- Open `voltage_logger.ino` in Arduino IDE
- Select your ESP8266 board (Tools → Board)
- Select correct COM port (Tools → Port) 
- Click Upload

## 4. Connect & Use
- ESP8266 creates WiFi network: `ESP8266_VoltageLogger`
- Password: `voltage123`
- Connect your device to this network
- Open browser to: `http://192.168.4.1`

## 5. Web Interface Features
- **Real-time voltage display** and statistics
- **Live chart** showing voltage over time
- **Data logging** with timestamps
- **Download data** as CSV file
- **Clear data** button to reset

## Troubleshooting
- Check Serial Monitor (115200 baud) for status messages
- Try different browser if interface doesn't load
- Ensure voltage is within 0-1V range for accurate readings

## Customization
Edit these values in `voltage_logger.ino`:
```cpp
// WiFi credentials
const char* ssid = "ESP8266_VoltageLogger";
const char* password = "voltage123";

// Sampling rate (milliseconds)
const unsigned long SAMPLE_INTERVAL = 1;

// Voltage conversion (for different ranges)
float voltage = (adcValue / 1024.0); // 0-1V range
```

That's it! Your voltage logger should now be running and accessible via web browser.
