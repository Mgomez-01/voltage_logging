# ESP8266 Dual Sensor Logger - Voltage + Temperature

A professional-grade dual sensor data logging system using ESP8266 with CD74HC4067 multiplexer for simultaneous voltage and temperature monitoring.

## 🌟 Features

- **Dual Sensor Logging**: Voltage and temperature simultaneously
- **Fast Sampling**: 500Hz per sensor (1000Hz total) with time-multiplexed reading
- **Real-time Web Interface**: Live dual sensor display with interactive charts
- **Data Persistence**: Stores readings to LittleFS filesystem, survives power cycles  
- **WiFi Access Point**: Creates its own WiFi network for easy connection
- **WebSocket Communication**: Real-time updates without page refresh
- **Controlled Logging**: Start/stop data collection on demand
- **CSV Export**: Download timestamped dual sensor data
- **Advanced Statistics**: Live min/max/average for both sensors
- **Responsive Design**: Works on desktop and mobile browsers
- **Hardware Expandable**: CD74HC4067 supports up to 16 analog sensors

## 📊 Sensor Capabilities

### **Voltage Measurement (Channel 0)**
- **Range**: 0-1V (configurable with voltage dividers)
- **Resolution**: 10-bit ADC (1024 levels)
- **Accuracy**: ±0.01V
- **Sample Rate**: 500Hz

### **Temperature Measurement (Channel 1)**  
- **Sensor**: 100kΩ NTC Thermistor (Ender 3 compatible)
- **Range**: -40°C to +125°C
- **Resolution**: 0.1°C
- **Accuracy**: ±1°C (with calibration)
- **Sample Rate**: 500Hz
- **Conversion**: Steinhart-Hart equation for precision

## Hardware Requirements

### **Core Components**
- **ESP8266MOD board** (NodeMCU, Wemos D1 Mini, or similar)
- **CD74HC4067** 16-channel analog multiplexer (~$2-3)
- **100kΩ resistor** (for thermistor voltage divider)

### **Sensors**
- **Voltage source** to measure (0-1V range, expandable with voltage dividers)
- **100kΩ NTC thermistor** (Ender 3 printer compatible, β=3950K)

### **Optional**
- **Breadboard and jumper wires** for prototyping
- **Pull-up resistors** if using longer cable runs
- **Capacitors** for noise filtering in high-EMI environments

## 🔌 Wiring Overview

```
ESP8266 → CD74HC4067
├─ A0 → COM (analog output)
├─ D1 → S0 (channel select bit 0)
├─ D2 → S1 (channel select bit 1)
├─ D3 → S2 (channel select bit 2)
├─ D4 → S3 (channel select bit 3)
├─ 3.3V → VCC + EN
└─ GND → GND

Sensors → CD74HC4067
├─ C0 → Voltage sensor (0-1V)
└─ C1 → Thermistor circuit:
    3.3V ─── [100kΩ] ─── C1 ─── [Thermistor] ─── GND
```

**📄 See [WIRING_GUIDE.md](WIRING_GUIDE.md) for detailed connection diagrams**

## Software Requirements

### Arduino IDE Libraries

Install these libraries through the Arduino IDE Library Manager:

1. **ESP8266WiFi** (usually included with ESP8266 board package)
2. **ESP8266WebServer** (usually included with ESP8266 board package)  
3. **WebSocketsServer** by Markus Sattler
4. **ArduinoJson** by Benoit Blanchon

### ESP8266 Board Package

1. In Arduino IDE, go to File → Preferences
2. Add this URL to "Additional Board Manager URLs":
   ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
3. Go to Tools → Board → Board Manager
4. Search for "ESP8266" and install the ESP8266 Community package

## 🚀 Quick Start

**You can test the software immediately** - it works with or without the CD74HC4067:

### **Method 1: PlatformIO (Recommended)**
```bash
pip install platformio
cd voltage_logger
make deploy
```

### **Method 2: Arduino CLI**
```bash
# Install Arduino CLI and ESP8266 core
arduino-cli core install esp8266:esp8266
arduino-cli lib install "ArduinoJson" "WebSockets"

# Compile and upload
arduino-cli compile --fqbn esp8266:esp8266:generic src/main.cpp
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:generic
```

## 🌐 Usage Instructions

### **1. Connect to WiFi**
- **Network**: `ESP8266_VoltageLogger`
- **Password**: `voltage123`
- **IP Address**: `192.168.4.1`

### **2. Open Web Interface**
- Open browser to: `http://192.168.4.1`
- **Status**: Should show "PAUSED" (logging starts paused)

### **3. Start Data Collection**
- **Click "▶ Start Logging"** when ready to collect data
- **Watch live readings**: Voltage and temperature update in real-time
- **Monitor statistics**: Min/max/average for both sensors

### **4. Download Data**
- **Click "Download Data"** for CSV export
- **Format**: `timestamp,voltage,temperature`
- **Example**: `1234,0.8213,23.45`

## Web Interface

The web interface provides:

- **Current Reading**: Large display of current voltage
- **Statistics**: Min, Max, Average, and Sample Count
- **Real-time Chart**: Scrolling plot of voltage vs time (last 500 points)
- **Data Log**: Text log of all readings with timestamps
- **Controls**:
  - Download Data: Get CSV file of all stored readings
  - Clear Data: Reset all stored data
  - Pause/Resume: Temporarily stop logging new data

## Technical Details

### Sampling Rate
- ADC sampled every 1ms (1000 Hz)
- Web updates every 100ms (10 Hz) for smooth display
- Buffer size: 100 readings before writing to file

### Data Storage
- Uses SPIFFS filesystem on ESP8266 flash memory
- Data stored in CSV format: `timestamp,voltage`
- Persists across power cycles and resets
- Automatic file creation and management

### Network Configuration
- **Access Point Mode**: ESP8266 creates its own WiFi network
- **IP Address**: 192.168.4.1 (default AP gateway)
- **WebSocket Port**: 81 (for real-time communication)
- **HTTP Port**: 80 (for web interface)

### Memory Management
- Circular buffer prevents memory overflow
- Chart limited to 500 points for browser performance
- Log display limited to 1000 entries
- Automatic cleanup of old data

## Customization

### Voltage Range
To change the voltage range, modify this line in the code:
```cpp
float voltage = (adcValue / 1024.0); // Currently 0-1V
```

For 0-3.3V range:
```cpp
float voltage = (adcValue / 1024.0) * 3.3;
```

### Sampling Rate
Adjust these constants:
```cpp
const unsigned long SAMPLE_INTERVAL = 1; // ADC sampling (ms)
const unsigned long WEB_UPDATE_INTERVAL = 100; // Web updates (ms)
```

### WiFi Credentials
Change these constants:
```cpp
const char* ssid = "ESP8266_VoltageLogger";
const char* password = "voltage123";
```

### Buffer Size
Modify for different memory usage:
```cpp
const int BUFFER_SIZE = 100; // Number of readings before file write
```

## Troubleshooting

### Upload Issues
- Check that correct board and port are selected
- Try holding FLASH button during upload if available
- Ensure ESP8266 board package is properly installed

### WiFi Connection Issues
- Look for the access point name in WiFi settings
- Default IP is always 192.168.4.1 in AP mode
- Try different devices if connection fails

### Web Interface Issues
- Clear browser cache and refresh
- Check JavaScript console for errors
- Try different browser (Chrome recommended)

### Data Issues
- If data isn't persisting, SPIFFS may not be initialized
- Check Serial Monitor for SPIFFS error messages
- Try clearing data and starting fresh

## Serial Monitor Output

Connect to ESP8266 at 115200 baud to see:
- Startup messages
- WiFi configuration details
- WebSocket connection status
- Error messages and debugging info

## File Structure

```
voltage_logger/
├── voltage_logger.ino    # Main Arduino sketch
├── README.md            # This documentation
└── libraries.txt        # Required library list
```

## Performance Notes

- Maximum practical sampling rate depends on WiFi activity
- Chart performance may degrade with very long sessions
- Browser memory usage grows with session length
- Recommend sessions under 1 hour for best performance

## License

This project is open source. Feel free to modify and distribute.
