// ESP8266 Voltage Logger Configuration
// Modify these values to customize your voltage logger

// =============================================================================
// WIFI CONFIGURATION
// =============================================================================
// Access Point credentials (what you connect to)
const char* WIFI_SSID = "ESP8266_VoltageLogger";
const char* WIFI_PASSWORD = "voltage123";

// =============================================================================
// SAMPLING CONFIGURATION  
// =============================================================================
// How often to read the ADC (milliseconds)
const unsigned long ADC_SAMPLE_INTERVAL = 1;  // 1ms = 1000 Hz sampling

// How often to update the web interface (milliseconds)
const unsigned long WEB_UPDATE_INTERVAL = 100;  // 100ms = 10 Hz updates

// Buffer size - how many readings to store before writing to file
const int READING_BUFFER_SIZE = 100;

// =============================================================================
// VOLTAGE CONFIGURATION
// =============================================================================
// ADC voltage range - modify based on your setup
// Option 1: 0-1V (default for bare ESP8266)
#define VOLTAGE_RANGE_0_1V
float convertADCtoVoltage(int adcValue) {
    return (adcValue / 1024.0);  // 0-1V
}

// Option 2: 0-3.3V (if using voltage divider)
// Uncomment this and comment above for 3.3V range
// #define VOLTAGE_RANGE_0_3V3
// float convertADCtoVoltage(int adcValue) {
//     return (adcValue / 1024.0) * 3.3;  // 0-3.3V
// }

// Option 3: Custom range
// Uncomment and modify for custom voltage range
// #define VOLTAGE_RANGE_CUSTOM
// const float VOLTAGE_MIN = 0.0;
// const float VOLTAGE_MAX = 5.0;
// float convertADCtoVoltage(int adcValue) {
//     return VOLTAGE_MIN + (adcValue / 1024.0) * (VOLTAGE_MAX - VOLTAGE_MIN);
// }

// =============================================================================
// WEB INTERFACE CONFIGURATION
// =============================================================================
// Maximum number of data points to show on chart (for performance)
const int MAX_CHART_POINTS = 500;

// Maximum number of log entries to show in text log
const int MAX_LOG_ENTRIES = 1000;

// Chart Y-axis limits (set to match your voltage range)
const float CHART_Y_MIN = 0.0;
const float CHART_Y_MAX = 1.0;

// =============================================================================
// FILE SYSTEM CONFIGURATION
// =============================================================================
// Name of the data file stored on SPIFFS
const char* DATA_FILENAME = "/voltage_data.csv";

// Whether to include timestamp in filename (creates unique files)
const bool USE_TIMESTAMP_IN_FILENAME = false;

// =============================================================================
// SERIAL DEBUGGING
// =============================================================================
// Baud rate for serial communication
const long SERIAL_BAUD_RATE = 115200;

// Enable/disable debug output
const bool ENABLE_DEBUG_OUTPUT = true;

// =============================================================================
// NETWORK CONFIGURATION
// =============================================================================
// Web server port
const int HTTP_PORT = 80;

// WebSocket port  
const int WEBSOCKET_PORT = 81;

// Access Point IP configuration
// Default: 192.168.4.1 (standard ESP8266 AP)
// Modify only if you need different IP range
IPAddress AP_LOCAL_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1); 
IPAddress AP_SUBNET(255, 255, 255, 0);

// =============================================================================
// ADVANCED CONFIGURATION
// =============================================================================
// ADC pin (A0 is the only ADC pin on ESP8266)
const int ADC_INPUT_PIN = A0;

// Watchdog timer reset interval (milliseconds)
const unsigned long WATCHDOG_INTERVAL = 1000;

// Maximum file size before rotation (bytes) - 0 to disable
const long MAX_FILE_SIZE = 0;  // Disabled by default

// =============================================================================
// USAGE NOTES
// =============================================================================
/*
QUICK START:
1. Modify WIFI_SSID and WIFI_PASSWORD above
2. Choose your voltage range (0-1V, 0-3.3V, or custom)
3. Upload to ESP8266
4. Connect to the WiFi network
5. Open browser to http://192.168.4.1

PERFORMANCE TUNING:
- Increase ADC_SAMPLE_INTERVAL to reduce CPU load
- Decrease WEB_UPDATE_INTERVAL for faster web updates
- Adjust READING_BUFFER_SIZE based on available memory
- Reduce MAX_CHART_POINTS if web interface is slow

TROUBLESHOOTING:
- Check Serial Monitor at 115200 baud for debug messages
- Ensure all required libraries are installed
- Verify voltage range matches your hardware setup
- Try different browsers if web interface has issues
*/
