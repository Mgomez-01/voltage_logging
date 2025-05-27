# ESP8266 Voltage Logger

A real-time voltage logging system using ESP8266 with web-based monitoring and data export capabilities.

## Features

- **Fast ADC Sampling**: Samples voltage every 1ms for capturing rapid changes
- **Real-time Web Interface**: Live voltage display with interactive chart
- **Data Persistence**: Stores readings to SPIFFS filesystem, survives power cycles  
- **WiFi Access Point**: Creates its own WiFi network for easy connection
- **WebSocket Communication**: Real-time updates without page refresh
- **Data Export**: Download all logged data as CSV file
- **Statistics**: Live min/max/average calculations
- **Responsive Design**: Works on desktop and mobile browsers

## Hardware Requirements

- ESP8266MOD board (like Hiletgo development board)
- Voltage source to measure (0-1V range)
- Connection to ADC pin (A0)

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

## Setup Instructions

1. **Hardware Connection**:
   - Connect your voltage source to the A0 (ADC) pin
   - Ensure voltage is within 0-1V range
   - Connect power to ESP8266 (via USB or external supply)

2. **Upload Code**:
   - Open `voltage_logger.ino` in Arduino IDE
   - Select your ESP8266 board (e.g., "NodeMCU 1.0" or "Generic ESP8266")
   - Select the correct COM port
   - Upload the sketch

3. **Connect to Device**:
   - After upload, the ESP8266 will create a WiFi access point
   - Network Name: `ESP8266_VoltageLogger`
   - Password: `voltage123`
   - Connect your device to this network
   - Open browser to: `http://192.168.4.1`

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
