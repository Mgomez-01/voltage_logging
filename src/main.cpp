#include <Arduino.h>          // Core Arduino functions and pin definitions
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>  // For thermistor calculations
#include <Ticker.h>  // For hardware timer-based safety system

// Fallback definition for A0 if not defined (for IDE/linter support)
#ifndef A0
#define A0 17  // ESP8266 ADC pin
#endif

// Debug configuration
#define DEBUG_SERIAL 1
#define DEBUG_ADC 1
#define DEBUG_WEBSOCKET 1
#define DEBUG_WIFI 1
#define DEBUG_HEATER 1
#define DEBUG_PID 1

// WiFi Access Point Configuration  
const char* ssid = "ESP8266_VoltageLogger";
const char* password = "voltage123";

// Web server and WebSocket
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Multiplexer control pins (CD74HC4067)
const int MUX_S0 = 5;  // D1
const int MUX_S1 = 4;  // D2  
const int MUX_S2 = 0;  // D3
const int MUX_S3 = 2;  // D4

// Sensor channels on multiplexer
const int VOLTAGE_CHANNEL = 0;    // Channel 0 for voltage measurement
const int THERMISTOR_CHANNEL = 1; // Channel 1 for thermistor

// Heater control pins and configuration
const int RELAY_PIN = 16; // GPIO16 (D0) for relay control
bool heaterEnabled = false;
bool relayState = false;
unsigned long relayOnTime = 0;
const unsigned long MAX_HEATER_TIME = 600000; // 10 min safety timeout
const float MAX_SAFE_TEMPERATURE = 80.0; // Maximum safe temperature in °C

// PID Controller variables
float targetTemperature = 25.0; // Default target temperature
bool pidEnabled = false;
float pidKp = 2.0;  // Proportional gain
float pidKi = 0.5;  // Integral gain  
float pidKd = 0.1;  // Derivative gain
float pidOutput = 0.0;
float pidError = 0.0;
float pidLastError = 0.0;
float pidIntegral = 0.0;
const unsigned long PID_INTERVAL = 1000; // PID update interval in ms
unsigned long lastPIDUpdate = 0;

// ADC and timing variables
const int ADC_PIN = A0;
unsigned long lastSample = 0;
unsigned long lastWebUpdate = 0;
unsigned long lastDebugPrint = 0;
const unsigned long SAMPLE_INTERVAL = 2; // Sample every 2ms (500Hz per channel, 1000Hz total)
const unsigned long WEB_UPDATE_INTERVAL = 100; // Update web every 100ms
const unsigned long DEBUG_INTERVAL = 1000; // Debug print every 1 second

// Data collection control
bool dataLoggingEnabled = false;  // Start with logging PAUSED

// Thermistor configuration (typical 100k NTC thermistor)
const float THERMISTOR_NOMINAL = 100000.0;   // 100k ohm at 25°C
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C
const float B_COEFFICIENT = 3950.0;          // Beta coefficient for typical 100k thermistor
const float SERIES_RESISTOR = 100000.0;      // 100k series resistor

// Sensor data structure with heater control
struct SensorReading {
  unsigned long timestamp;
  float voltage;
  float temperature;
  int currentChannel; // Which channel was being sampled
  bool heaterState;   // Heater relay state
  float pidValue;     // PID controller output
  float targetTemp;   // Target temperature at time of reading
};

const int BUFFER_SIZE = 100;
SensorReading readings[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;
int currentSensorChannel = VOLTAGE_CHANNEL; // Start with voltage channel

// File system
const char* DATA_FILE = "/sensor_data.csv";
File dataFile;

// Statistics for debugging
unsigned long totalReadings = 0;
unsigned long totalWebSocketMessages = 0;
unsigned long connectedClients = 0;

// Hardware-based Safety System
Ticker safetyTimer;           // Hardware timer for safety checks
Ticker watchdogTimer;         // Hardware watchdog timer
volatile bool systemAlive = true;     // Watchdog heartbeat flag
volatile unsigned long lastSafetyCheck = 0;  // Last safety check timestamp
volatile bool emergencyShutdown = false;     // Emergency shutdown flag
const unsigned long SAFETY_CHECK_INTERVAL = 500;  // Safety check every 500ms
const unsigned long WATCHDOG_TIMEOUT = 10000;     // 10 second watchdog timeout

// Function declarations
void setupMultiplexer();
void selectMuxChannel(int channel);
void readSensors();
float convertThermistorToTemperature(int adcValue);
void writeBufferToFile();
void sendWebUpdate();
void initializeDataFile();
void handleRoot();
void handleDataDownload();
void handleClearData();
void handleStatus();
void handleStartLogging();
void handleStopLogging();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
String getIndexHTML();
void printDebugStats();

// Heater control functions
void initializeRelay();
void setRelayState(bool state);
void checkSafetyTimeout();
void updatePIDController();
void handleRelayOn();
void handleRelayOff();
void handleSetTemperature();
void handlePIDEnable();
void handlePIDDisable();
void handlePIDParams();
void handleHeaterStatus();

// Hardware Safety System functions
void ICACHE_RAM_ATTR hardwareSafetyCheck();
void ICACHE_RAM_ATTR watchdogCheck();
void initializeSafetySystem();
void feedWatchdog();
void emergencyShutdownSystem();

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println();
  Serial.println("===============================================");
  Serial.println("ESP8266 Dual Sensor Logger - Voltage + Temperature");
  Serial.println("CD74HC4067 Multiplexer Version");
  Serial.println("===============================================");
  
  // Setup multiplexer control pins
  setupMultiplexer();
  
  // Setup heater relay control
  initializeRelay();
  
  // Debug: Show what A0 maps to and test both channels
  Serial.print("A0 pin number: ");
  Serial.println(A0);
  
  Serial.println("Testing sensor channels:");
  selectMuxChannel(VOLTAGE_CHANNEL);
  delay(10);
  int voltageReading = analogRead(A0);
  Serial.print("Channel 0 (Voltage): ");
  Serial.print(voltageReading);
  Serial.print(" -> ");
  Serial.print((voltageReading / 1024.0), 4);
  Serial.println("V");
  
  selectMuxChannel(THERMISTOR_CHANNEL);
  delay(10);
  int tempReading = analogRead(A0);
  float temperature = convertThermistorToTemperature(tempReading);
  Serial.print("Channel 1 (Thermistor): ");
  Serial.print(tempReading);
  Serial.print(" -> ");
  if (temperature > -50 && temperature < 150) { // Reasonable range check
    Serial.print(temperature, 2);
    Serial.println("°C");
  } else {
    Serial.println("N/A (no thermistor connected)");
  }
  
  // Initialize LittleFS
  Serial.print("Initializing LittleFS... ");
  if (!LittleFS.begin()) {
    Serial.println("FAILED!");
    Serial.println("ERROR: LittleFS initialization failed!");
    return;
  }
  Serial.println("OK");
  
  // Setup WiFi Access Point
  Serial.print("Setting up WiFi Access Point... ");
  WiFi.mode(WIFI_AP);
  bool apResult = WiFi.softAP(ssid, password);
  if (apResult) {
    Serial.println("OK");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.softAPmacAddress());
  } else {
    Serial.println("FAILED!");
    Serial.println("ERROR: Could not create WiFi Access Point!");
  }
  
  // Setup web server routes
  Serial.print("Setting up web server routes... ");
  server.on("/", handleRoot);
  server.on("/data.csv", handleDataDownload);
  server.on("/clear", handleClearData);
  server.on("/status", handleStatus);
  server.on("/start", handleStartLogging);
  server.on("/stop", handleStopLogging);
  
  // Heater control routes
  server.on("/relay/on", handleRelayOn);
  server.on("/relay/off", handleRelayOff);
  server.on("/temp/set", handleSetTemperature);
  server.on("/pid/enable", handlePIDEnable);
  server.on("/pid/disable", handlePIDDisable);
  server.on("/pid/params", handlePIDParams);
  server.on("/heater/status", handleHeaterStatus);
  
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("OK");
  Serial.println("Web server listening on port 80");
  
  // Setup WebSocket
  Serial.print("Setting up WebSocket server... ");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("OK");
  Serial.print("WebSocket server listening on port 81 at IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket URL should be: ws://" + WiFi.softAPIP().toString() + ":81");
  
  // Initialize data file
  Serial.print("Initializing data file... ");
  initializeDataFile();
  Serial.println("OK");
  
  // Initialize Hardware Safety System
  Serial.print("Initializing hardware safety system... ");
  initializeSafetySystem();
  Serial.println("OK");
  Serial.println("Hardware safety: Independent timer-based checks active");
  
  Serial.println("===============================================");
  Serial.println("SETUP COMPLETE - READY FOR CONNECTIONS");
  Serial.println("===============================================");
  Serial.print("Connect to WiFi: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Open browser to: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("===============================================");
  Serial.println("⚠️  DATA LOGGING IS PAUSED BY DEFAULT");
  Serial.println("   Click 'Start Logging' in web interface to begin");
  Serial.println("   This prevents stale data from previous sessions");
  Serial.println("   Voltage Channel 0, Temperature Channel 1");
  Serial.println("🔥 HEATER CONTROL READY");
  Serial.println("   Relay on GPIO16 (D0)");
  Serial.println("   PID control available");
  Serial.println("   Safety timeout: 10 minutes");
  Serial.println("   ⚡ HARDWARE SAFETY SYSTEM ACTIVE");
  Serial.println("   ⚡ Independent timer-based monitoring");
  Serial.println("   ⚡ Watchdog protection enabled");
  Serial.println("===============================================");
  Serial.println();
}

void loop() {
  // Feed watchdog to show system is alive
  feedWatchdog();
  
  // Check for emergency shutdown
  if (emergencyShutdown) {
    emergencyShutdownSystem();
    return; // Skip normal operations during emergency
  }
  
  server.handleClient();
  webSocket.loop();
  
  // Legacy safety checks (backup to hardware timer)
  checkSafetyTimeout();
  
  // Only sample when logging is enabled
  if (dataLoggingEnabled) {
    // Dual sensor sampling
    if (millis() - lastSample >= SAMPLE_INTERVAL) {
      readSensors();
      lastSample = millis();
    }
    
    // PID control loop
    if (pidEnabled && millis() - lastPIDUpdate >= PID_INTERVAL) {
      updatePIDController();
      lastPIDUpdate = millis();
    }
    
    // Web updates
    if (millis() - lastWebUpdate >= WEB_UPDATE_INTERVAL) {
      sendWebUpdate();
      lastWebUpdate = millis();
    }
  }
  
  // Debug output (always enabled)
  if (millis() - lastDebugPrint >= DEBUG_INTERVAL) {
    printDebugStats();
    lastDebugPrint = millis();
  }
}

void setupMultiplexer() {
  Serial.print("Setting up CD74HC4067 multiplexer... ");
  
  // Configure multiplexer control pins as outputs
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  
  // Start with voltage channel
  selectMuxChannel(VOLTAGE_CHANNEL);
  
  Serial.println("OK");
  Serial.println("Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4");
  Serial.println("Channel 0: Voltage sensor");
  Serial.println("Channel 1: Thermistor (100k NTC)");
}

void selectMuxChannel(int channel) {
  // Set multiplexer channel using binary representation
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  
  // Small delay to allow multiplexer to settle
  delayMicroseconds(10);
}

void readSensors() {
  static float lastVoltage = 0.0;
  static float lastTemperature = 0.0;
  
  // Alternate between channels each sample
  if (currentSensorChannel == VOLTAGE_CHANNEL) {
    // Read voltage
    selectMuxChannel(VOLTAGE_CHANNEL);
    delayMicroseconds(50); // Allow settling time
    int adcValue = analogRead(ADC_PIN);
    lastVoltage = (adcValue / 1024.0); // Convert to 0-1V range
    
    currentSensorChannel = THERMISTOR_CHANNEL; // Next time read temperature
  } else {
    // Read temperature
    selectMuxChannel(THERMISTOR_CHANNEL);
    delayMicroseconds(50); // Allow settling time
    int adcValue = analogRead(ADC_PIN);
    lastTemperature = convertThermistorToTemperature(adcValue);
    
    // Store the combined reading with heater state
    readings[bufferIndex].timestamp = millis();
    readings[bufferIndex].voltage = lastVoltage;
    readings[bufferIndex].temperature = lastTemperature;
    readings[bufferIndex].currentChannel = THERMISTOR_CHANNEL; // Mark as complete cycle
    readings[bufferIndex].heaterState = relayState;
    readings[bufferIndex].pidValue = pidOutput;
    readings[bufferIndex].targetTemp = targetTemperature;
    
    totalReadings++;
    
    #if DEBUG_ADC
    // Debug every 500th reading to avoid spam
    if (totalReadings % 500 == 0) {
      Serial.print("Sensor Reading #");
      Serial.print(totalReadings);
      Serial.print(": V=");
      Serial.print(lastVoltage, 4);
      Serial.print("V, T=");
      Serial.print(lastTemperature, 2);
      Serial.print("°C (Buffer: ");
      Serial.print(bufferIndex);
      Serial.println(")");
    }
    #endif
    
    bufferIndex++;
    if (bufferIndex >= BUFFER_SIZE) {
      Serial.print("Buffer full, writing to file... ");
      bufferIndex = 0;
      bufferFull = true;
      writeBufferToFile();
      Serial.println("done");
    }
    
    currentSensorChannel = VOLTAGE_CHANNEL; // Next time read voltage
  }
}

float convertThermistorToTemperature(int adcValue) {
  if (adcValue == 0) return -999; // Error value for no reading
  
  // Convert ADC reading to resistance
  float voltage = (adcValue / 1024.0) * 3.3; // Assuming 3.3V reference
  if (voltage >= 3.29) return -999; // Open circuit
  
  float resistance = SERIES_RESISTOR * voltage / (3.3 - voltage);
  
  // Steinhart-Hart equation for NTC thermistor
  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                      // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                    // Invert
  steinhart -= 273.15;                            // Convert to Celsius
  
  return steinhart;
}

void writeBufferToFile() {
  dataFile = LittleFS.open(DATA_FILE, "a");
  if (dataFile) {
    int startIndex = bufferFull ? bufferIndex : 0;
    int endIndex = bufferFull ? BUFFER_SIZE : bufferIndex;
    
    // Calculate stats for this write batch
    float batchVoltageMin = 1.0, batchVoltageMax = 0.0, batchVoltageSum = 0.0;
    float batchTempMin = 999.0, batchTempMax = -999.0, batchTempSum = 0.0;
    int batchCount = 0;
    
    for (int i = startIndex; i < endIndex; i++) {
      dataFile.print(readings[i].timestamp);
      dataFile.print(",");
      dataFile.print(readings[i].voltage, 6);
      dataFile.print(",");
      dataFile.print(readings[i].temperature, 3);
      dataFile.print(",");
      dataFile.print(readings[i].heaterState ? 1 : 0);
      dataFile.print(",");
      dataFile.print(readings[i].targetTemp, 2);
      dataFile.print(",");
      dataFile.println(readings[i].pidValue, 2);
      
      // Track batch statistics
      float v = readings[i].voltage;
      float t = readings[i].temperature;
      
      if (v < batchVoltageMin) batchVoltageMin = v;
      if (v > batchVoltageMax) batchVoltageMax = v;
      batchVoltageSum += v;
      
      if (t > -50 && t < 150) { // Only count reasonable temperatures
        if (t < batchTempMin) batchTempMin = t;
        if (t > batchTempMax) batchTempMax = t;
        batchTempSum += t;
      }
      batchCount++;
    }
    dataFile.close();
    
    Serial.print("Wrote ");
    Serial.print(batchCount);
    Serial.println(" sensor readings to file");
    Serial.print("  Voltage - Min=");
    Serial.print(batchVoltageMin, 4);
    Serial.print("V, Max=");
    Serial.print(batchVoltageMax, 4);
    Serial.print("V, Avg=");
    Serial.print(batchVoltageSum / batchCount, 4);
    Serial.println("V");
    if (batchTempMin < 999) {
      Serial.print("  Temperature - Min=");
      Serial.print(batchTempMin, 2);
      Serial.print("°C, Max=");
      Serial.print(batchTempMax, 2);
      Serial.print("°C, Avg=");
      Serial.print(batchTempSum / batchCount, 2);
      Serial.println("°C");
    } else {
      Serial.println("  Temperature - No valid readings (thermistor not connected?)");
    }
  } else {
    Serial.println("ERROR: Could not open data file for writing!");
  }
}

void sendWebUpdate() {
  connectedClients = webSocket.connectedClients();
  
  if (connectedClients > 0) {
    // Get current reading
    if (bufferIndex > 0 || bufferFull) {
      int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
      
      // Use JsonDocument for newer ArduinoJson
      JsonDocument doc;
      doc["timestamp"] = readings[idx].timestamp;
      doc["voltage"] = readings[idx].voltage;
      doc["temperature"] = readings[idx].temperature;
      doc["type"] = "reading";
      doc["heaterState"] = readings[idx].heaterState;
      doc["targetTemp"] = readings[idx].targetTemp;
      doc["pidOutput"] = readings[idx].pidValue;
      
      String jsonString;
      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
      totalWebSocketMessages++;
      
      #if DEBUG_WEBSOCKET
      // Debug every 100th message
      if (totalWebSocketMessages % 100 == 0) {
        Serial.print("WebSocket message #");
        Serial.print(totalWebSocketMessages);
        Serial.print(" sent to ");
        Serial.print(connectedClients);
        Serial.print(" clients: ");
        Serial.print(jsonString);
        Serial.print(" (buffer idx: ");
        Serial.print(idx);
        Serial.print(", V: ");
        Serial.print(readings[idx].voltage, 4);
        Serial.print("V, T: ");
        Serial.print(readings[idx].temperature, 2);
        Serial.println("°C)");
      }
      #endif
    }
  }
}

void initializeDataFile() {
  // Check if file exists, if not create with header
  if (!LittleFS.exists(DATA_FILE)) {
    dataFile = LittleFS.open(DATA_FILE, "w");
    if (dataFile) {
      dataFile.println("timestamp,voltage,temperature,heater_state,target_temp,pid_output");
      dataFile.close();
      Serial.println("Created new data file with heater control header");
    } else {
      Serial.println("ERROR: Could not create data file!");
    }
  } else {
    Serial.println("Data file exists, will append new data with heater control");
  }
}

void handleRoot() {
  Serial.println("HTTP: Serving root page to client");
  server.send(200, "text/html", getIndexHTML());
}

void handleDataDownload() {
  Serial.println("HTTP: Data download requested");
  if (LittleFS.exists(DATA_FILE)) {
    // First, flush current buffer to file
    writeBufferToFile();
    
    File file = LittleFS.open(DATA_FILE, "r");
    if (file) {
      server.sendHeader("Content-Disposition", "attachment; filename=sensor_data.csv");
      server.streamFile(file, "text/csv");
      file.close();
      Serial.println("HTTP: Dual sensor data file sent successfully");
    } else {
      server.send(404, "text/plain", "File not found");
      Serial.println("HTTP: ERROR - Could not open data file");
    }
  } else {
    server.send(404, "text/plain", "No data file exists");
    Serial.println("HTTP: ERROR - Data file does not exist");
  }
}

void handleClearData() {
  Serial.println("HTTP: Clear data requested");
  
  // Show file info before clearing
  if (LittleFS.exists(DATA_FILE)) {
    File file = LittleFS.open(DATA_FILE, "r");
    if (file) {
      Serial.print("File size before clear: ");
      Serial.print(file.size());
      Serial.println(" bytes");
      file.close();
    }
  }
  
  LittleFS.remove(DATA_FILE);
  initializeDataFile();
  bufferIndex = 0;
  bufferFull = false;
  totalReadings = 0;
  server.send(200, "text/plain", "Data cleared");
  Serial.println("HTTP: Data cleared successfully - fresh CSV will contain only new dual sensor readings");
}

void handleStatus() {
  Serial.println("HTTP: Status requested (polling mode)");
  
  // Get current readings from both channels
  selectMuxChannel(VOLTAGE_CHANNEL);
  delay(5);
  int voltageADC = analogRead(A0);
  float currentVoltage = (voltageADC / 1024.0);
  
  selectMuxChannel(THERMISTOR_CHANNEL);
  delay(5);
  int tempADC = analogRead(A0);
  float currentTemperature = convertThermistorToTemperature(tempADC);
  
  // Create JSON response
  JsonDocument doc;
  doc["timestamp"] = millis();
  doc["voltage"] = currentVoltage;
  doc["temperature"] = currentTemperature;
  doc["voltageADC"] = voltageADC;
  doc["temperatureADC"] = tempADC;
  doc["type"] = "reading";
  doc["totalReadings"] = totalReadings;
  doc["connectedClients"] = webSocket.connectedClients();
  doc["uptime"] = millis() / 1000;
  doc["loggingEnabled"] = dataLoggingEnabled;
  doc["heaterState"] = relayState;
  doc["targetTemp"] = targetTemperature;
  doc["pidEnabled"] = pidEnabled;
  doc["pidOutput"] = pidOutput;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
  Serial.print("HTTP: Status sent - V:");
  Serial.print(currentVoltage, 4);
  Serial.print("V, T:");
  Serial.print(currentTemperature, 2);
  Serial.print("°C, Logging:");
  Serial.println(dataLoggingEnabled ? "ON" : "OFF");
}

void handleStartLogging() {
  Serial.println("HTTP: Start logging requested");
  dataLoggingEnabled = true;
  
  // Reset timing for clean start
  lastSample = millis();
  lastWebUpdate = millis();
  currentSensorChannel = VOLTAGE_CHANNEL; // Start with voltage
  
  server.send(200, "text/plain", "Dual sensor logging started");
  Serial.println("HTTP: Dual sensor logging STARTED - now collecting voltage and temperature readings");
}

void handleStopLogging() {
  Serial.println("HTTP: Stop logging requested");
  dataLoggingEnabled = false;
  
  // Flush any remaining buffer to file
  if (bufferIndex > 0) {
    Serial.println("Flushing remaining buffer to file...");
    writeBufferToFile();
    bufferIndex = 0;
  }
  
  server.send(200, "text/plain", "Dual sensor logging stopped");
  Serial.println("HTTP: Dual sensor logging STOPPED - readings paused");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("WebSocket[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      Serial.printf("WebSocket: Now %u clients connected\n", webSocket.connectedClients());
      
      // Send a welcome message to confirm connection
      JsonDocument doc;
      doc["type"] = "welcome";
      doc["message"] = "Dual sensor WebSocket connected successfully";
      doc["sensors"] = "voltage+temperature";
      doc["timestamp"] = millis();
      String welcomeJson;
      serializeJson(doc, welcomeJson);
      webSocket.sendTXT(num, welcomeJson);
      Serial.println("WebSocket: Welcome message sent for dual sensor logger");
      break;
    }
    case WStype_TEXT:
      Serial.printf("WebSocket[%u] Received text: %s\n", num, payload);
      break;
    case WStype_ERROR:
      Serial.printf("WebSocket[%u] Error occurred\n", num);
      break;
    case WStype_BIN:
      Serial.printf("WebSocket[%u] Binary data received\n", num);
      break;
    case WStype_FRAGMENT_TEXT_START:
      Serial.printf("WebSocket[%u] Fragment text start\n", num);
      break;
    case WStype_FRAGMENT_BIN_START:
      Serial.printf("WebSocket[%u] Fragment binary start\n", num);
      break;
    case WStype_FRAGMENT:
      Serial.printf("WebSocket[%u] Fragment\n", num);
      break;
    case WStype_FRAGMENT_FIN:
      Serial.printf("WebSocket[%u] Fragment finished\n", num);
      break;
    default:
      Serial.printf("WebSocket[%u] Unknown event type: %d\n", num, type);
      break;
  }
}

// Heater Control Implementation Functions

void initializeRelay() {
  Serial.print("Initializing heater relay control... ");
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Start with relay OFF
  relayState = false;
  Serial.println("OK");
  Serial.print("Relay pin: GPIO");
  Serial.print(RELAY_PIN);
  Serial.println(" (D0)");
}

void setRelayState(bool state) {
  if (state != relayState) {
    relayState = state;
    digitalWrite(RELAY_PIN, state ? HIGH : LOW);
    
    if (state) {
      relayOnTime = millis();
      #if DEBUG_HEATER
      Serial.println("HEATER: Relay turned ON");
      #endif
    } else {
      #if DEBUG_HEATER
      Serial.print("HEATER: Relay turned OFF (was on for ");
      Serial.print((millis() - relayOnTime) / 1000);
      Serial.println(" seconds)");
      #endif
    }
  }
}

void checkSafetyTimeout() {
  // Enhanced safety checks that work with current sensor readings
  // This is a backup to the hardware timer-based safety system
  
  // Check if hardware safety system is working
  if (millis() - lastSafetyCheck > SAFETY_CHECK_INTERVAL * 3) {
    Serial.println("WARNING: Hardware safety system not responding!");
    // Force emergency shutdown as backup
    emergencyShutdown = true;
  }
  
  // Temperature safety (requires sensor readings)
  if (bufferIndex > 0 || bufferFull) {
    int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
    float currentTemp = readings[idx].temperature;
    
    // Over-temperature protection
    if (currentTemp > MAX_SAFE_TEMPERATURE) {
      Serial.print("SAFETY: Over-temperature shutdown! Temp=");
      Serial.print(currentTemp);
      Serial.println("°C");
      emergencyShutdown = true;
    }
    
    // Sensor failure detection
    if (relayState && (isnan(currentTemp) || currentTemp < -50 || currentTemp > 200)) {
      Serial.println("SAFETY: Temperature sensor failure - emergency shutdown");
      emergencyShutdown = true;
    }
  }
  
  // If emergency shutdown was triggered, ensure immediate action
  if (emergencyShutdown) {
    digitalWrite(RELAY_PIN, LOW);  // Force heater OFF immediately
    relayState = false;
    heaterEnabled = false;
    pidEnabled = false;
  }
}

void updatePIDController() {
  if (!pidEnabled || bufferIndex == 0) return;
  
  // Get current temperature
  int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
  float currentTemp = readings[idx].temperature;
  
  // Calculate error
  pidError = targetTemperature - currentTemp;
  
  // Proportional term
  float proportional = pidKp * pidError;
  
  // Integral term (with windup protection)
  pidIntegral += pidError * (PID_INTERVAL / 1000.0);
  if (pidIntegral > 100) pidIntegral = 100;
  if (pidIntegral < -100) pidIntegral = -100;
  float integral = pidKi * pidIntegral;
  
  // Derivative term
  float derivative = pidKd * (pidError - pidLastError) / (PID_INTERVAL / 1000.0);
  pidLastError = pidError;
  
  // Calculate output (0-100%)
  pidOutput = proportional + integral + derivative;
  
  // Clamp output
  if (pidOutput > 100) pidOutput = 100;
  if (pidOutput < 0) pidOutput = 0;
  
  // Apply output to relay (bang-bang control for now)
  if (pidOutput > 50.0) {
    setRelayState(true);
  } else {
    setRelayState(false);
  }
  
  #if DEBUG_PID
  Serial.print("PID: Target=");
  Serial.print(targetTemperature);
  Serial.print("°C, Current=");
  Serial.print(currentTemp);
  Serial.print("°C, Error=");
  Serial.print(pidError);
  Serial.print(", Output=");
  Serial.print(pidOutput);
  Serial.print("%, Relay=");
  Serial.println(relayState ? "ON" : "OFF");
  #endif
}

void handleRelayOn() {
  Serial.println("HTTP: Relay ON requested");
  heaterEnabled = true;
  setRelayState(true);
  server.send(200, "text/plain", "Relay turned ON");
}

void handleRelayOff() {
  Serial.println("HTTP: Relay OFF requested");
  heaterEnabled = false;
  pidEnabled = false; // Also disable PID when manually turning off
  setRelayState(false);
  server.send(200, "text/plain", "Relay turned OFF");
}

void handleSetTemperature() {
  if (server.hasArg("temp")) {
    float newTemp = server.arg("temp").toFloat();
    if (newTemp >= 0 && newTemp <= MAX_SAFE_TEMPERATURE) {
      targetTemperature = newTemp;
      Serial.print("HTTP: Target temperature set to ");
      Serial.print(targetTemperature);
      Serial.println("°C");
      server.send(200, "text/plain", "Target temperature set to " + String(targetTemperature) + "°C");
    } else {
      server.send(400, "text/plain", "Invalid temperature (0-" + String(MAX_SAFE_TEMPERATURE) + "°C)");
    }
  } else {
    server.send(400, "text/plain", "Missing temp parameter");
  }
}

void handlePIDEnable() {
  Serial.println("HTTP: PID control ENABLED");
  pidEnabled = true;
  heaterEnabled = true;
  pidIntegral = 0; // Reset integral term
  pidLastError = 0;
  server.send(200, "text/plain", "PID control enabled");
}

void handlePIDDisable() {
  Serial.println("HTTP: PID control DISABLED");
  pidEnabled = false;
  server.send(200, "text/plain", "PID control disabled");
}

void handlePIDParams() {
  bool updated = false;
  
  if (server.hasArg("kp")) {
    pidKp = server.arg("kp").toFloat();
    updated = true;
  }
  if (server.hasArg("ki")) {
    pidKi = server.arg("ki").toFloat();
    updated = true;
  }
  if (server.hasArg("kd")) {
    pidKd = server.arg("kd").toFloat();
    updated = true;
  }
  
  if (updated) {
    Serial.print("HTTP: PID parameters updated - Kp=");
    Serial.print(pidKp);
    Serial.print(", Ki=");
    Serial.print(pidKi);
    Serial.print(", Kd=");
    Serial.println(pidKd);
    
    // Reset PID state when parameters change
    pidIntegral = 0;
    pidLastError = 0;
    
    server.send(200, "text/plain", "PID parameters updated");
  } else {
    server.send(400, "text/plain", "No parameters provided");
  }
}

void handleHeaterStatus() {
  JsonDocument doc;
  
  doc["heaterEnabled"] = heaterEnabled;
  doc["relayState"] = relayState;
  doc["targetTemp"] = targetTemperature;
  doc["pidEnabled"] = pidEnabled;
  doc["pidOutput"] = pidOutput;
  doc["pidError"] = pidError;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
  
  if (relayState) {
    doc["heaterRuntime"] = (millis() - relayOnTime) / 1000; // seconds
  } else {
    doc["heaterRuntime"] = 0;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

// Hardware Safety System Implementation

void ICACHE_RAM_ATTR hardwareSafetyCheck() {
  // This runs in hardware timer - keep it fast and simple!
  lastSafetyCheck = millis();
  
  // Critical safety check: heater timeout
  if (relayState && millis() - relayOnTime > MAX_HEATER_TIME) {
    digitalWrite(RELAY_PIN, LOW);  // Direct GPIO write - fastest way
    relayState = false;
    heaterEnabled = false;
    pidEnabled = false;
    emergencyShutdown = true;
  }
  
  // Temperature safety will be checked when we have current readings
  // Cannot safely read ADC from timer interrupt
}

void ICACHE_RAM_ATTR watchdogCheck() {
  // Check if main loop is feeding the watchdog
  if (!systemAlive) {
    // Main loop is not responding - emergency shutdown
    digitalWrite(RELAY_PIN, LOW);  // Force heater OFF
    emergencyShutdown = true;
    // System will reset via ESP8266 hardware watchdog
  }
  systemAlive = false;  // Reset flag - main loop must set it
}

void initializeSafetySystem() {
  // Enable ESP8266 hardware watchdog (last resort)
  ESP.wdtEnable(WATCHDOG_TIMEOUT);
  
  // Start hardware timer for safety checks
  safetyTimer.attach_ms(SAFETY_CHECK_INTERVAL, hardwareSafetyCheck);
  
  // Start watchdog check timer
  watchdogTimer.attach_ms(1000, watchdogCheck);  // Check every second
  
  // Initialize safety state
  systemAlive = true;
  emergencyShutdown = false;
  lastSafetyCheck = millis();
}

void feedWatchdog() {
  // Called from main loop to show system is alive
  systemAlive = true;
  ESP.wdtFeed();  // Feed hardware watchdog
}

void emergencyShutdownSystem() {
  // Handle emergency shutdown state
  static unsigned long lastShutdownMessage = 0;
  
  // Ensure heater is OFF
  digitalWrite(RELAY_PIN, LOW);
  relayState = false;
  heaterEnabled = false;
  pidEnabled = false;
  
  // Print emergency message (rate limited)
  if (millis() - lastShutdownMessage > 5000) {
    Serial.println("*** EMERGENCY SHUTDOWN ACTIVE ***");
    Serial.println("*** HEATER DISABLED - SYSTEM SAFE ***");
    Serial.println("*** Restart required to resume operation ***");
    lastShutdownMessage = millis();
  }
  
  // Still handle basic web requests to show status
  server.handleClient();
}

void printDebugStats() {
  Serial.println("=== DUAL SENSOR DEBUG STATS ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  // Safety System Status
  Serial.println("HARDWARE SAFETY SYSTEM:");
  Serial.print("  Status: ");
  if (emergencyShutdown) {
    Serial.println("*** EMERGENCY SHUTDOWN ACTIVE ***");
  } else {
    Serial.println("Active and monitoring");
  }
  Serial.print("  Last safety check: ");
  Serial.print((millis() - lastSafetyCheck) / 1000);
  Serial.println(" seconds ago");
  Serial.print("  Watchdog: ");
  Serial.println(systemAlive ? "Fed" : "STARVED");
  
  Serial.print("Data Logging: ");
  Serial.println(dataLoggingEnabled ? "ENABLED" : "PAUSED");
  
  Serial.print("WiFi Status: ");
  if (WiFi.getMode() == WIFI_AP) {
    Serial.print("AP Mode - Connected stations: ");
    Serial.println(WiFi.softAPgetStationNum());
  } else {
    Serial.println("Not in AP mode!");
  }
  
  Serial.print("WebSocket clients: ");
  Serial.println(webSocket.connectedClients());
  
  Serial.print("Total sensor readings: ");
  Serial.println(totalReadings);
  
  Serial.print("Current buffer index: ");
  Serial.print(bufferIndex);
  Serial.print("/");
  Serial.print(BUFFER_SIZE);
  Serial.print(" (");
  Serial.print((bufferIndex * 100) / BUFFER_SIZE);
  Serial.println("% full)");
  
  Serial.print("WebSocket messages sent: ");
  Serial.println(totalWebSocketMessages);
  
  // Show current live readings from both channels
  Serial.println("HEATER CONTROL STATUS:");
  Serial.print("  Heater: ");
  Serial.print(heaterEnabled ? "ENABLED" : "DISABLED");
  Serial.print(", Relay: ");
  Serial.print(relayState ? "ON" : "OFF");
  if (relayState) {
    Serial.print(" (Runtime: ");
    Serial.print((millis() - relayOnTime) / 1000);
    Serial.print("s)");
  }
  Serial.println();
  
  Serial.print("  PID Control: ");
  Serial.print(pidEnabled ? "ACTIVE" : "INACTIVE");
  if (pidEnabled) {
    Serial.print(" (Target: ");
    Serial.print(targetTemperature);
    Serial.print("°C, Output: ");
    Serial.print(pidOutput, 1);
    Serial.print("%, Error: ");
    Serial.print(pidError, 2);
    Serial.print("°C)");
  }
  Serial.println();
  
  Serial.print("  PID Parameters: Kp=");
  Serial.print(pidKp);
  Serial.print(", Ki=");
  Serial.print(pidKi);
  Serial.print(", Kd=");
  Serial.println(pidKd);
  
  Serial.println("LIVE sensor readings:");
  
  selectMuxChannel(VOLTAGE_CHANNEL);
  delay(5);
  int voltageADC = analogRead(A0);
  float currentVoltage = (voltageADC / 1024.0);
  Serial.print("  Voltage (Ch0): ");
  Serial.print(voltageADC);
  Serial.print(" -> ");
  Serial.print(currentVoltage, 6);
  Serial.println("V");
  
  selectMuxChannel(THERMISTOR_CHANNEL);
  delay(5);
  int tempADC = analogRead(A0);
  float currentTemperature = convertThermistorToTemperature(tempADC);
  Serial.print("  Temperature (Ch1): ");
  Serial.print(tempADC);
  Serial.print(" -> ");
  if (currentTemperature > -50 && currentTemperature < 150) {
    Serial.print(currentTemperature, 3);
    Serial.println("°C");
  } else {
    Serial.println("N/A (no thermistor or bad reading)");
  }
  
  // Show recent buffered readings (only if logging enabled)
  if (dataLoggingEnabled && (bufferIndex > 0 || bufferFull)) {
    Serial.println("Last 3 BUFFERED sensor readings:");
    for (int i = 0; i < 3; i++) {
      int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      if (bufferFull || idx < bufferIndex) {
        Serial.print("  [" + String(idx) + "] ");
        Serial.print(readings[idx].voltage, 4);
        Serial.print("V | ");
        Serial.print(readings[idx].temperature, 2);
        Serial.print("°C @ ");
        Serial.print(readings[idx].timestamp);
        Serial.println("ms");
      }
    }
    
    // Calculate statistics from current buffer
    float voltageMin = 1.0, voltageMax = 0.0, voltageSum = 0.0;
    float tempMin = 999.0, tempMax = -999.0, tempSum = 0.0;
    int validReadings = bufferFull ? BUFFER_SIZE : bufferIndex;
    int validTempReadings = 0;
    
    for (int i = 0; i < validReadings; i++) {
      float v = readings[i].voltage;
      float t = readings[i].temperature;
      
      if (v < voltageMin) voltageMin = v;
      if (v > voltageMax) voltageMax = v;
      voltageSum += v;
      
      if (t > -50 && t < 150) { // Only count reasonable temperatures
        if (t < tempMin) tempMin = t;
        if (t > tempMax) tempMax = t;
        tempSum += t;
        validTempReadings++;
      }
    }
    
    if (validReadings > 0) {
      Serial.print("Buffer stats - Voltage: Min=");
      Serial.print(voltageMin, 4);
      Serial.print("V, Max=");
      Serial.print(voltageMax, 4);
      Serial.print("V, Avg=");
      Serial.print(voltageSum / validReadings, 4);
      Serial.print("V (n=");
      Serial.print(validReadings);
      Serial.println(")");
      
      if (validTempReadings > 0) {
        Serial.print("             Temperature: Min=");
        Serial.print(tempMin, 2);
        Serial.print("°C, Max=");
        Serial.print(tempMax, 2);
        Serial.print("°C, Avg=");
        Serial.print(tempSum / validTempReadings, 2);
        Serial.print("°C (n=");
        Serial.print(validTempReadings);
        Serial.println(")");
      } else {
        Serial.println("             Temperature: No valid readings in buffer");
      }
    }
  } else if (!dataLoggingEnabled) {
    Serial.println("No buffered readings (logging paused)");
  }
  
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.println("====================================");
  Serial.println();
}

// Forward declarations for HTML generation functions
String getHTMLHeader();
String getHTMLStyles();
String getHTMLBody();
String getHTMLScript();

String getIndexHTML() {
  // Use reserve to pre-allocate memory and reduce fragmentation
  String html;
  html.reserve(8192); // Reserve reasonable initial size
  
  html = getHTMLHeader();
  html += getHTMLStyles();
  html += getHTMLBody();
  html += getHTMLScript();
  html += "</html>";
  
  return html;
}

String getHTMLHeader() {
  String header;
  header.reserve(512);
  
  header = "<!DOCTYPE html><html><head>";
  header += "<title>ESP8266 Dual Sensor Logger with Heater Control</title>";
  header += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  header += "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>";
  header += "</head>";
  
  return header;
}

String getHTMLStyles() {
  String styles;
  styles.reserve(2048);
  
  styles = "<style>";
  styles += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }";
  styles += ".container { max-width: 1200px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  styles += ".header { text-align: center; margin-bottom: 30px; color: #333; }";
  styles += ".controls { text-align: center; margin-bottom: 20px; }";
  styles += ".btn { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; margin: 0 5px; text-decoration: none; display: inline-block; }";
  styles += ".btn:hover { background-color: #45a049; }";
  styles += ".btn.danger { background-color: #f44336; }";
  styles += ".btn.danger:hover { background-color: #da190b; }";
  styles += ".btn.warning { background-color: #ff9800; }";
  styles += ".btn.warning:hover { background-color: #e68900; }";
  styles += ".btn:disabled { background-color: #cccccc; cursor: not-allowed; }";
  styles += ".status { text-align: center; margin-bottom: 20px; padding: 10px; border-radius: 4px; }";
  styles += ".status.connected { background-color: #d4edda; color: #155724; border: 1px solid #c3e6cb; }";
  styles += ".status.disconnected { background-color: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }";
  styles += ".chart-container { width: 100%; height: 400px; margin-bottom: 30px; }";
  styles += ".log-container { height: 300px; overflow-y: auto; border: 1px solid #ddd; padding: 10px; background-color: #f9f9f9; font-family: monospace; font-size: 12px; }";
  styles += ".current-readings { text-align: center; margin-bottom: 20px; }";
  styles += ".reading { display: inline-block; margin: 0 20px; padding: 15px; background-color: #f8f9fa; border-radius: 8px; border: 2px solid #dee2e6; }";
  styles += ".reading-value { font-size: 24px; font-weight: bold; color: #2196F3; }";
  styles += ".reading-label { font-size: 14px; color: #666; margin-top: 5px; }";
  styles += ".stats { display: flex; justify-content: space-around; margin-bottom: 20px; flex-wrap: wrap; }";
  styles += ".stat { text-align: center; margin: 5px; }";
  styles += ".stat-value { font-size: 18px; font-weight: bold; color: #4CAF50; }";
  styles += ".stat-label { font-size: 12px; color: #666; }";
  styles += ".debug { background-color: #f0f0f0; padding: 10px; margin: 10px 0; border-radius: 4px; font-family: monospace; font-size: 12px; }";
  styles += ".control-panel { background-color: #f8f9fa; border: 2px solid #dee2e6; border-radius: 8px; padding: 20px; margin: 20px 0; }";
  styles += ".control-panel h3 { margin-top: 0; color: #495057; }";
  styles += ".heater-status { display: inline-block; width: 20px; height: 20px; border-radius: 50%; margin-left: 10px; vertical-align: middle; }";
  styles += ".heater-on { background-color: #ff4444; box-shadow: 0 0 10px #ff4444; }";
  styles += ".heater-off { background-color: #666666; }";
  styles += ".pid-params { margin-top: 10px; }";
  styles += ".pid-params input { width: 60px; margin: 0 5px; padding: 5px; border: 1px solid #ccc; border-radius: 4px; }";
  styles += ".temp-input { width: 80px; padding: 5px; margin: 0 10px; border: 1px solid #ccc; border-radius: 4px; font-size: 16px; }";
  styles += "</style>";
  
  return styles;
}

String getHTMLBody() {
  String body;
  body.reserve(4096);
  
  body = "<body>";
  
  body += "<div class=\"container\">";
  body += "<div class=\"header\">";
  body += "<h1>ESP8266 Dual Sensor Logger with Heater Control</h1>";
  body += "<p>Real-time voltage and temperature monitoring with PID heater control</p>";
  body += "</div>";
  
  // Add debug info section
  body += "<div class=\"debug\">";
  body += "<strong>Debug Info:</strong><br>";
  body += "ESP8266 IP: " + WiFi.softAPIP().toString() + "<br>";
  body += "WebSocket Port: 81<br>";
  body += "ADC Pin: A0 (pin " + String(A0) + ") via CD74HC4067<br>";
  body += "Channel 0: Voltage Sensor | Channel 1: 100k Thermistor<br>";
  body += "Heater Relay: GPIO16 (D0)<br>";
  body += "Uptime: <span id=\"uptime\">0</span> seconds<br>";
  body += "Chart Status: <span id=\"chartStatus\">Checking...</span><br>";
  body += "Logging Status: <span id=\"loggingStatus\">" + String(dataLoggingEnabled ? "ACTIVE" : "PAUSED") + "</span><br>";
  body += "Connection Status: <span id=\"connectionDebug\">Initializing...</span>";
  body += "</div>";
  
  // Add prominent logging control section
  body += "<div style=\"text-align:center;margin:20px 0;padding:20px;background-color:#f8f9fa;border-radius:8px;border:2px solid #dee2e6;\">";
  body += "<h3 style=\"margin-top:0;color:#495057;\">Data Logging Control</h3>";
  body += "<div style=\"font-size:18px;margin:10px 0;\">Status: <span id=\"mainLoggingStatus\" style=\"font-weight:bold;color:" + String(dataLoggingEnabled ? "#28a745" : "#dc3545") + ";\">" + String(dataLoggingEnabled ? "ACTIVE" : "PAUSED") + "</span></div>";
  body += "<button onclick=\"startLogging()\" class=\"btn\" id=\"startBtn\" style=\"background-color:#28a745;margin:5px;\" " + String(dataLoggingEnabled ? "disabled" : "") + ">▶ Start Logging</button>";
  body += "<button onclick=\"stopLogging()\" class=\"btn\" id=\"stopBtn\" style=\"background-color:#dc3545;margin:5px;\" " + String(dataLoggingEnabled ? "" : "disabled") + ">⏸ Stop Logging</button>";
  body += "<div style=\"font-size:14px;color:#6c757d;margin-top:10px;\">" + String(dataLoggingEnabled ? "Currently collecting dual sensor readings at 500Hz" : "Click 'Start Logging' to begin data collection") + "</div>";
  body += "</div>";
  
  body += "<div id=\"status\" class=\"status disconnected\">Connecting to WebSocket...</div>";
  
  // Add Heater Control Panel
  body += "<div class=\"control-panel\">";
  body += "<h3>🔥 Heater Control</h3>";
  body += "<div style=\"margin-bottom: 15px;\">";
  body += "Relay Status: <span id=\"relayStatus\" style=\"font-weight: bold;\">" + String(relayState ? "ON" : "OFF") + "</span>";
  body += "<span id=\"heaterIndicator\" class=\"heater-status " + String(relayState ? "heater-on" : "heater-off") + "\"></span>";
  body += "</div>";
  body += "<div style=\"margin-bottom: 15px;\">";
  body += "Runtime: <span id=\"heaterRuntime\">0:00:00</span>";
  body += " | Safety Timeout: 10 minutes";
  body += "</div>";
  body += "<button onclick=\"relayOn()\" class=\"btn\" id=\"relayOnBtn\">🔥 Turn ON</button>";
  body += "<button onclick=\"relayOff()\" class=\"btn danger\" id=\"relayOffBtn\">⏹ Turn OFF</button>";
  body += "<button onclick=\"emergencyStop()\" class=\"btn danger\" style=\"margin-left: 20px;\">⚠️ EMERGENCY STOP</button>";
  body += "</div>";
  
  // Add Temperature Control Panel
  body += "<div class=\"control-panel\">";
  body += "<h3>🌡️ Temperature Control</h3>";
  body += "<div style=\"margin-bottom: 15px;\">";
  body += "Current: <span id=\"currentTemp\" style=\"font-size: 20px; font-weight: bold; color: #2196F3;\">--</span>°C";
  body += " | Target: <input type=\"number\" id=\"targetTempInput\" class=\"temp-input\" value=\"" + String(targetTemperature, 1) + "\" min=\"0\" max=\"" + String(MAX_SAFE_TEMPERATURE) + "\" step=\"0.5\">";
  body += "<button onclick=\"setTargetTemp()\" class=\"btn\" style=\"padding: 5px 15px;\">Set</button>";
  body += "</div>";
  body += "<div style=\"margin-bottom: 15px;\">";
  body += "PID Control: <span id=\"pidStatus\" style=\"font-weight: bold;\">" + String(pidEnabled ? "ENABLED" : "DISABLED") + "</span>";
  body += " | Output: <span id=\"pidOutputValue\">" + String(pidOutput, 1) + "</span>%";
  body += "</div>";
  body += "<button onclick=\"enablePID()\" class=\"btn\" id=\"pidEnableBtn\">▶ Enable PID</button>";
  body += "<button onclick=\"disablePID()\" class=\"btn warning\" id=\"pidDisableBtn\">⏸ Manual Mode</button>";
  body += "<div class=\"pid-params\">";
  body += "<strong>PID Parameters:</strong>";
  body += " Kp: <input type=\"number\" id=\"kpInput\" value=\"" + String(pidKp, 2) + "\" step=\"0.1\">";
  body += " Ki: <input type=\"number\" id=\"kiInput\" value=\"" + String(pidKi, 2) + "\" step=\"0.1\">";
  body += " Kd: <input type=\"number\" id=\"kdInput\" value=\"" + String(pidKd, 2) + "\" step=\"0.1\">";
  body += "<button onclick=\"updatePIDParams()\" class=\"btn\" style=\"margin-left: 10px;\">Update</button>";
  body += "</div>";
  body += "</div>";
  
  // Current readings section
  body += "<div class=\"current-readings\">";
  body += "<div class=\"reading\">";
  body += "<div class=\"reading-value\" id=\"currentVoltage\">--</div>";
  body += "<div class=\"reading-label\">Voltage (V)</div>";
  body += "</div>";
  body += "<div class=\"reading\">";
  body += "<div class=\"reading-value\" id=\"currentTemperature\">--</div>";
  body += "<div class=\"reading-label\">Temperature (°C)</div>";
  body += "</div>";
  body += "</div>";
  
  // Statistics section
  body += "<div class=\"stats\">";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"minVoltage\">--</div><div class=\"stat-label\">Min Voltage (V)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"maxVoltage\">--</div><div class=\"stat-label\">Max Voltage (V)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"avgVoltage\">--</div><div class=\"stat-label\">Avg Voltage (V)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"minTemperature\">--</div><div class=\"stat-label\">Min Temp (°C)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"maxTemperature\">--</div><div class=\"stat-label\">Max Temp (°C)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"avgTemperature\">--</div><div class=\"stat-label\">Avg Temp (°C)</div></div>";
  body += "<div class=\"stat\"><div class=\"stat-value\" id=\"sampleCount\">0</div><div class=\"stat-label\">Samples</div></div>";
  body += "</div>";
  
  body += "<div class=\"controls\">";
  body += "<a href=\"/data.csv\" class=\"btn\">Download Data</a>";
  body += "<button onclick=\"clearData()\" class=\"btn danger\">Clear Data</button>";
  body += "<button onclick=\"toggleLogging()\" class=\"btn\" id=\"logToggle\">Pause Logging</button>";
  body += "</div>";
  
  body += "<div class=\"chart-container\"><canvas id=\"sensorChart\"></canvas></div>";
  body += "<div class=\"log-container\" id=\"logContainer\"><div>Sensor readings will appear here...</div></div>";
  body += "</div>";
  body += "</body>";
  
  return body;
}

String getHTMLScript() {
  String script;
  script.reserve(8192);
  
  script = "<script>";
  
  // JavaScript section with dual sensor support and heater control
  script += "let ws, chart, loggingEnabled = true, startTime = Date.now();";
  script += "let connectionAttempts = 0, lastMessageTime = 0, maxRetries = 10;";
  script += "let voltageStats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  script += "let tempStats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  script += "let heaterState = false, pidEnabled = false, heaterStartTime = 0;";
  
  script += "function updateDebugInfo() {";
  script += "document.getElementById('uptime').textContent = Math.floor((Date.now() - startTime) / 1000);";
  script += "let status = 'Attempts: ' + connectionAttempts + '/' + maxRetries;";
  script += "if (lastMessageTime > 0) status += ', Last message: ' + Math.floor((Date.now() - lastMessageTime) / 1000) + 's ago';";
  script += "document.getElementById('connectionDebug').textContent = status;";
  script += "}";
  
  script += "function logDebug(msg) { console.log('[DEBUG] ' + msg); }";
  
  script += "function initChart() {";
  script += "logDebug('Checking if Chart.js is available...');";
  script += "if (typeof Chart === 'undefined') {";
  script += "logDebug('Chart.js not loaded - chart will be disabled');";
  script += "document.getElementById('chartStatus').textContent = 'Disabled (no internet)';";
  script += "document.getElementById('chartStatus').style.color = 'orange';";
  script += "document.getElementById('sensorChart').style.display = 'none';";
  script += "const chartContainer = document.querySelector('.chart-container');";
  script += "if (chartContainer) chartContainer.innerHTML = '<div style=\"text-align:center;padding:20px;background:#f0f0f0;border-radius:4px;color:#666;\"><strong>Real-time Chart Unavailable</strong><br>Chart.js requires internet connection<br>Sensor readings still work below</div>';";
  script += "return;";
  script += "}";
  script += "logDebug('Chart.js loaded successfully, initializing dual sensor chart...');";
  script += "try {";
  script += "const ctx = document.getElementById('sensorChart').getContext('2d');";
  script += "chart = new Chart(ctx, {";
  script += "type: 'line',";
  script += "data: { labels: [], datasets: [";
  script += "{ label: 'Voltage (V)', data: [], borderColor: '#2196F3', backgroundColor: 'rgba(33, 150, 243, 0.1)', borderWidth: 2, fill: false, yAxisID: 'voltage' },";
  script += "{ label: 'Temperature (°C)', data: [], borderColor: '#FF6384', backgroundColor: 'rgba(255, 99, 132, 0.1)', borderWidth: 2, fill: false, yAxisID: 'temperature' }";
  script += "] },";
  script += "options: { responsive: true, maintainAspectRatio: false, scales: { ";
  script += "x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (ms)' } }, ";
  script += "voltage: { type: 'linear', position: 'left', title: { display: true, text: 'Voltage (V)' }, min: 0, max: 1 }, ";
  script += "temperature: { type: 'linear', position: 'right', title: { display: true, text: 'Temperature (°C)' } }";
  script += "}, plugins: { legend: { display: true } }, animation: { duration: 0 } }";
  script += "});";
  script += "document.getElementById('chartStatus').textContent = 'Active (Dual Sensor)';";
  script += "document.getElementById('chartStatus').style.color = 'green';";
  script += "logDebug('Dual sensor chart initialized successfully');";
  script += "} catch(e) {";
  script += "logDebug('Error initializing chart: ' + e.message);";
  script += "document.getElementById('chartStatus').textContent = 'Error: ' + e.message;";
  script += "document.getElementById('chartStatus').style.color = 'red';";
  script += "chart = null;";
  script += "}";
  script += "}";
  
  // Continue with WebSocket and other functions...
  script += "function initWebSocket() {";
  script += "if (connectionAttempts >= maxRetries) {";
  script += "logDebug('Max WebSocket connection attempts reached, switching to polling mode');";
  script += "document.getElementById('status').textContent = 'Using HTTP Polling (WebSocket failed)';";
  script += "document.getElementById('status').className = 'status connected';";
  script += "startPolling();";
  script += "return;";
  script += "}";
  
  script += "connectionAttempts++;";
  script += "logDebug('WebSocket connection attempt #' + connectionAttempts);";
  
  script += "const hostname = window.location.hostname;";
  script += "const wsUrl = 'ws://' + hostname + ':81';";
  script += "logDebug('Hostname: ' + hostname);";
  script += "logDebug('WebSocket URL: ' + wsUrl);";
  script += "logDebug('Current page URL: ' + window.location.href);";
  
  script += "try {";
  script += "ws = new WebSocket(wsUrl);";
  script += "logDebug('WebSocket object created');";
  script += "} catch(e) {";
  script += "logDebug('Error creating WebSocket: ' + e.message);";
  script += "setTimeout(initWebSocket, 3000);";
  script += "return;";
  script += "}";
  
  script += "ws.onopen = function() { ";
  script += "logDebug('WebSocket connected successfully!'); ";
  script += "document.getElementById('status').textContent = 'WebSocket Connected (Dual Sensor)'; ";
  script += "document.getElementById('status').className = 'status connected'; ";
  script += "};";
  
  script += "ws.onclose = function(event) { ";
  script += "logDebug('WebSocket closed - Code: ' + event.code + ', Reason: ' + event.reason + ', Clean: ' + event.wasClean); ";
  script += "document.getElementById('status').textContent = 'Disconnected - Retrying...'; ";
  script += "document.getElementById('status').className = 'status disconnected'; ";
  script += "setTimeout(initWebSocket, 2000); ";
  script += "};";
  
  script += "ws.onerror = function(error) { ";
  script += "logDebug('WebSocket error occurred: ' + JSON.stringify(error)); ";
  script += "document.getElementById('status').textContent = 'WebSocket Error - Retrying...'; ";
  script += "document.getElementById('status').className = 'status disconnected'; ";
  script += "};";
  
  script += "ws.onmessage = function(event) { ";
  script += "lastMessageTime = Date.now(); ";
  script += "logDebug('WebSocket message received: ' + event.data); ";
  script += "if (loggingEnabled) { ";
  script += "try { ";
  script += "const data = JSON.parse(event.data); ";
  script += "if (data.type === 'reading') addSensorReading(data.timestamp, data.voltage, data.temperature, data); ";
  script += "} catch(e) { logDebug('Error parsing message: ' + e.message); } ";
  script += "} ";
  script += "};"
  html += "}";
  
  html += "function startPolling() {";
  html += "logDebug('Starting HTTP polling mode...');";
  html += "setInterval(function() {";
  html += "fetch('/status').then(function(response) {";
  html += "return response.json();";
  html += "}).then(function(data) {";
  html += "if (data.voltage !== undefined && data.temperature !== undefined) {";
  html += "addSensorReading(data.timestamp, data.voltage, data.temperature, data);";
  html += "}";
  html += "}).catch(function(error) {";
  html += "logDebug('Polling error: ' + error.message);";
  html += "});";
  html += "}, 500);";
  html += "}";
  
  html += "function addSensorReading(timestamp, voltage, temperature, data) {";
  html += "document.getElementById('currentVoltage').textContent = voltage.toFixed(4);";
  html += "document.getElementById('currentTemperature').textContent = temperature.toFixed(2);";
  html += "document.getElementById('currentTemp').textContent = temperature.toFixed(2);";
  
  html += "if (data) {";
  html += "updateHeaterStatus(data.heaterState, data.targetTemp, data.pidEnabled, data.pidOutput);";
  html += "}";
  
  html += "voltageStats.min = Math.min(voltageStats.min, voltage);";
  html += "voltageStats.max = Math.max(voltageStats.max, voltage);";
  html += "voltageStats.sum += voltage;";
  html += "voltageStats.count++;";
  
  html += "if (temperature > -50 && temperature < 150) {";
  html += "tempStats.min = Math.min(tempStats.min, temperature);";
  html += "tempStats.max = Math.max(tempStats.max, temperature);";
  html += "tempStats.sum += temperature;";
  html += "tempStats.count++;";
  html += "}";
  
  html += "document.getElementById('minVoltage').textContent = voltageStats.min.toFixed(4);";
  html += "document.getElementById('maxVoltage').textContent = voltageStats.max.toFixed(4);";
  html += "document.getElementById('avgVoltage').textContent = (voltageStats.sum / voltageStats.count).toFixed(4);";
  html += "document.getElementById('sampleCount').textContent = voltageStats.count;";
  
  html += "if (tempStats.count > 0) {";
  html += "document.getElementById('minTemperature').textContent = tempStats.min.toFixed(2);";
  html += "document.getElementById('maxTemperature').textContent = tempStats.max.toFixed(2);";
  html += "document.getElementById('avgTemperature').textContent = (tempStats.sum / tempStats.count).toFixed(2);";
  html += "} else {";
  html += "document.getElementById('minTemperature').textContent = 'N/A';";
  html += "document.getElementById('maxTemperature').textContent = 'N/A';";
  html += "document.getElementById('avgTemperature').textContent = 'N/A';";
  html += "}";
  
  html += "if (chart && typeof chart.update === 'function') {";
  html += "if (chart.data.labels.length > 500) { ";
  html += "chart.data.labels.shift(); ";
  html += "chart.data.datasets[0].data.shift(); ";
  html += "chart.data.datasets[1].data.shift(); ";
  html += "}";
  html += "chart.data.labels.push(timestamp);";
  html += "chart.data.datasets[0].data.push(voltage);";
  html += "chart.data.datasets[1].data.push(temperature);";
  html += "chart.update('none');";
  html += "} else {";
  html += "logDebug('Chart not available, skipping chart update');";
  html += "}";
  
  html += "const logContainer = document.getElementById('logContainer');";
  html += "const logEntry = document.createElement('div');";
  html += "let logText = new Date(timestamp).toLocaleTimeString() + ' - ' + voltage.toFixed(4) + 'V | ' + temperature.toFixed(2) + '°C';";
  html += "if (data && data.heaterState !== undefined) {";
  html += "logText += ' | Heater: ' + (data.heaterState ? 'ON' : 'OFF');";
  html += "if (data.pidEnabled) logText += ' | PID: ' + data.pidOutput.toFixed(1) + '%';";
  html += "}";
  html += "logEntry.textContent = logText;";
  html += "logContainer.appendChild(logEntry);";
  html += "while (logContainer.children.length > 1000) logContainer.removeChild(logContainer.firstChild);";
  html += "logContainer.scrollTop = logContainer.scrollHeight;";
  html += "}";
  
  html += "function clearData() {";
  html += "if (confirm('Are you sure you want to clear all sensor data?')) {";
  html += "fetch('/clear').then(function() {";
  html += "if (chart && typeof chart.update === 'function') {";
  html += "chart.data.labels = []; chart.data.datasets[0].data = []; chart.data.datasets[1].data = []; chart.update();";
  html += "} else {";
  html += "logDebug('Chart not available for clearing');";
  html += "}";
  html += "voltageStats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  html += "tempStats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  html += "['minVoltage', 'maxVoltage', 'avgVoltage', 'minTemperature', 'maxTemperature', 'avgTemperature'].forEach(id => document.getElementById(id).textContent = '--');";
  html += "document.getElementById('sampleCount').textContent = '0';";
  html += "document.getElementById('currentVoltage').textContent = '--';";
  html += "document.getElementById('currentTemperature').textContent = '--';";
  html += "document.getElementById('logContainer').innerHTML = '<div>Data cleared. New sensor readings will appear here...</div>';";
  html += "alert('Sensor data cleared successfully');";
  html += "}).catch(function() { alert('Error clearing data'); });";
  html += "}";
  html += "}";
  
  html += "function toggleLogging() {";
  html += "loggingEnabled = !loggingEnabled;";
  html += "const btn = document.getElementById('logToggle');";
  html += "btn.textContent = loggingEnabled ? 'Pause Logging' : 'Resume Logging';";
  html += "btn.style.backgroundColor = loggingEnabled ? '#4CAF50' : '#ff9800';";
  html += "}";
  
  html += "function startLogging() {";
  html += "logDebug('Starting dual sensor logging...');";
  html += "fetch('/start').then(function(response) {";
  html += "return response.text();";
  html += "}).then(function(data) {";
  html += "logDebug('Start logging response: ' + data);";
  html += "document.getElementById('mainLoggingStatus').textContent = 'ACTIVE';";
  html += "document.getElementById('mainLoggingStatus').style.color = '#28a745';";
  html += "document.getElementById('loggingStatus').textContent = 'ACTIVE';";
  html += "document.getElementById('startBtn').disabled = true;";
  html += "document.getElementById('stopBtn').disabled = false;";
  html += "alert('Dual sensor logging started');";
  html += "}).catch(function(error) {";
  html += "logDebug('Error starting logging: ' + error.message);";
  html += "alert('Error starting logging');";
  html += "});";
  html += "}";
  
  html += "function stopLogging() {";
  html += "logDebug('Stopping dual sensor logging...');";
  html += "fetch('/stop').then(function(response) {";
  html += "return response.text();";
  html += "}).then(function(data) {";
  html += "logDebug('Stop logging response: ' + data);";
  html += "document.getElementById('mainLoggingStatus').textContent = 'PAUSED';";
  html += "document.getElementById('mainLoggingStatus').style.color = '#dc3545';";
  html += "document.getElementById('loggingStatus').textContent = 'PAUSED';";
  html += "document.getElementById('startBtn').disabled = false;";
  html += "document.getElementById('stopBtn').disabled = true;";
  html += "alert('Dual sensor logging stopped');";
  html += "}).catch(function(error) {";
  html += "logDebug('Error stopping logging: ' + error.message);";
  html += "alert('Error stopping logging');";
  html += "});";
  html += "}";
  
  html += "function updateHeaterStatus(state, target, pidOn, output) {";
  html += "heaterState = state;";
  html += "pidEnabled = pidOn;";
  html += "document.getElementById('relayStatus').textContent = state ? 'ON' : 'OFF';";
  html += "document.getElementById('heaterIndicator').className = 'heater-status ' + (state ? 'heater-on' : 'heater-off');";
  html += "document.getElementById('pidStatus').textContent = pidOn ? 'ENABLED' : 'DISABLED';";
  html += "document.getElementById('pidOutputValue').textContent = output.toFixed(1);";
  html += "if (state && heaterStartTime === 0) heaterStartTime = Date.now();";
  html += "if (!state) heaterStartTime = 0;";
  html += "}";
  
  html += "function updateHeaterRuntime() {";
  html += "if (heaterState && heaterStartTime > 0) {";
  html += "const runtime = Math.floor((Date.now() - heaterStartTime) / 1000);";
  html += "const hours = Math.floor(runtime / 3600);";
  html += "const minutes = Math.floor((runtime % 3600) / 60);";
  html += "const seconds = runtime % 60;";
  html += "document.getElementById('heaterRuntime').textContent = ";
  html += "hours + ':' + minutes.toString().padStart(2, '0') + ':' + seconds.toString().padStart(2, '0');";
  html += "} else {";
  html += "document.getElementById('heaterRuntime').textContent = '0:00:00';";
  html += "}";
  html += "}";
  
  html += "function relayOn() {"; 
  html += "if (heaterState) { alert('Heater is already ON'); return; }"; 
  html += "if (!confirm('Turn heater ON? This will enable manual heater control.')) return;"; 
  html += "logDebug('Turning relay ON...');"; 
  html += "const onBtn = document.getElementById('relayOnBtn');"; 
  html += "const offBtn = document.getElementById('relayOffBtn');"; 
  html += "if (onBtn) onBtn.disabled = true;"; 
  html += "fetch('/relay/on').then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('Relay ON response: ' + data);"; 
  html += "heaterState = true;"; 
  html += "heaterStartTime = Date.now();"; 
  html += "const statusEl = document.getElementById('relayStatus');"; 
  html += "const indicatorEl = document.getElementById('heaterIndicator');"; 
  html += "if (statusEl) statusEl.textContent = 'ON';"; 
  html += "if (indicatorEl) indicatorEl.className = 'heater-status heater-on';"; 
  html += "alert('Heater turned ON successfully');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error turning relay on: ' + error.message);"; 
  html += "if (onBtn) onBtn.disabled = false;"; 
  html += "alert('Error turning heater on: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "function relayOff() {"; 
  html += "if (!heaterState) { alert('Heater is already OFF'); return; }"; 
  html += "logDebug('Turning relay OFF...');"; 
  html += "const onBtn = document.getElementById('relayOnBtn');"; 
  html += "const offBtn = document.getElementById('relayOffBtn');"; 
  html += "if (offBtn) offBtn.disabled = true;"; 
  html += "fetch('/relay/off').then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('Relay OFF response: ' + data);"; 
  html += "heaterState = false;"; 
  html += "heaterStartTime = 0;"; 
  html += "const statusEl = document.getElementById('relayStatus');"; 
  html += "const indicatorEl = document.getElementById('heaterIndicator');"; 
  html += "if (statusEl) statusEl.textContent = 'OFF';"; 
  html += "if (indicatorEl) indicatorEl.className = 'heater-status heater-off';"; 
  html += "alert('Heater turned OFF successfully');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error turning relay off: ' + error.message);"; 
  html += "if (offBtn) offBtn.disabled = false;"; 
  html += "alert('Error turning heater off: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "function emergencyStop() {"; 
  html += "if (!confirm('EMERGENCY STOP: Turn off heater and disable PID?')) return;"; 
  html += "logDebug('EMERGENCY STOP initiated...');"; 
  html += "const onBtn = document.getElementById('relayOnBtn');"; 
  html += "const offBtn = document.getElementById('relayOffBtn');"; 
  html += "const enableBtn = document.getElementById('pidEnableBtn');"; 
  html += "const disableBtn = document.getElementById('pidDisableBtn');"; 
  html += "[onBtn, offBtn, enableBtn, disableBtn].forEach(btn => { if (btn) btn.disabled = true; });"; 
  html += "Promise.all(["; 
  html += "fetch('/relay/off').then(r => { if (!r.ok) throw new Error('Relay OFF failed: ' + r.status); return r.text(); }),"; 
  html += "fetch('/pid/disable').then(r => { if (!r.ok) throw new Error('PID disable failed: ' + r.status); return r.text(); })"; 
  html += "]).then(function(responses) {"; 
  html += "logDebug('Emergency stop responses: ' + JSON.stringify(responses));"; 
  html += "heaterState = false;"; 
  html += "pidEnabled = false;"; 
  html += "heaterStartTime = 0;"; 
  html += "const statusEl = document.getElementById('relayStatus');"; 
  html += "const pidStatusEl = document.getElementById('pidStatus');"; 
  html += "const indicatorEl = document.getElementById('heaterIndicator');"; 
  html += "if (statusEl) statusEl.textContent = 'OFF';"; 
  html += "if (pidStatusEl) pidStatusEl.textContent = 'DISABLED';"; 
  html += "if (indicatorEl) indicatorEl.className = 'heater-status heater-off';"; 
  html += "if (enableBtn) enableBtn.disabled = false;"; 
  html += "if (disableBtn) disableBtn.disabled = true;"; 
  html += "[onBtn, offBtn].forEach(btn => { if (btn) btn.disabled = false; });"; 
  html += "alert('⚠️ EMERGENCY STOP COMPLETE\\nHeater: OFF\\nPID: DISABLED\\nSystem is safe.');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Emergency stop error: ' + error.message);"; 
  html += "[onBtn, offBtn, enableBtn, disableBtn].forEach(btn => { if (btn) btn.disabled = false; });"; 
  html += "alert('⚠️ EMERGENCY STOP ERROR: ' + error.message + '\\nCheck system manually!');"; 
  html += "});"; 
  html += "}";
  
  html += "function setTargetTemp() {"; 
  html += "const tempInput = document.getElementById('targetTempInput');"; 
  html += "if (!tempInput) { alert('Temperature input not found'); return; }"; 
  html += "const temp = parseFloat(tempInput.value);"; 
  html += "if (isNaN(temp) || temp < 0 || temp > " + String(MAX_SAFE_TEMPERATURE) + ") {"; 
  html += "alert('Invalid temperature. Must be between 0 and " + String(MAX_SAFE_TEMPERATURE) + "°C');"; 
  html += "return;"; 
  html += "}"; 
  html += "logDebug('Setting target temperature to: ' + temp);"; 
  html += "fetch('/temp/set?temp=' + temp).then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('Set temperature response: ' + data);"; 
  html += "alert('Target temperature set to ' + temp + '°C');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error setting temperature: ' + error.message);"; 
  html += "alert('Error setting temperature: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "function enablePID() {"; 
  html += "if (pidEnabled) { alert('PID is already enabled'); return; }"; 
  html += "logDebug('Enabling PID control...');"; 
  html += "const enableBtn = document.getElementById('pidEnableBtn');"; 
  html += "const disableBtn = document.getElementById('pidDisableBtn');"; 
  html += "if (enableBtn) enableBtn.disabled = true;"; 
  html += "fetch('/pid/enable').then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('PID enable response: ' + data);"; 
  html += "pidEnabled = true;"; 
  html += "if (enableBtn) enableBtn.disabled = true;"; 
  html += "if (disableBtn) disableBtn.disabled = false;"; 
  html += "const statusEl = document.getElementById('pidStatus');"; 
  html += "if (statusEl) statusEl.textContent = 'ENABLED';"; 
  html += "alert('PID control enabled successfully');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error enabling PID: ' + error.message);"; 
  html += "if (enableBtn) enableBtn.disabled = false;"; 
  html += "alert('Error enabling PID: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "function disablePID() {"; 
  html += "if (!pidEnabled) { alert('PID is already disabled'); return; }"; 
  html += "logDebug('Disabling PID control...');"; 
  html += "const enableBtn = document.getElementById('pidEnableBtn');"; 
  html += "const disableBtn = document.getElementById('pidDisableBtn');"; 
  html += "if (disableBtn) disableBtn.disabled = true;"; 
  html += "fetch('/pid/disable').then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('PID disable response: ' + data);"; 
  html += "pidEnabled = false;"; 
  html += "if (enableBtn) enableBtn.disabled = false;"; 
  html += "if (disableBtn) disableBtn.disabled = true;"; 
  html += "const statusEl = document.getElementById('pidStatus');"; 
  html += "if (statusEl) statusEl.textContent = 'DISABLED';"; 
  html += "alert('PID control disabled successfully');"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error disabling PID: ' + error.message);"; 
  html += "if (disableBtn) disableBtn.disabled = false;"; 
  html += "alert('Error disabling PID: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "function updatePIDParams() {"; 
  html += "const kpInput = document.getElementById('kpInput');"; 
  html += "const kiInput = document.getElementById('kiInput');"; 
  html += "const kdInput = document.getElementById('kdInput');"; 
  html += "if (!kpInput || !kiInput || !kdInput) {"; 
  html += "alert('PID input fields not found'); return;"; 
  html += "}"; 
  html += "const kp = parseFloat(kpInput.value);"; 
  html += "const ki = parseFloat(kiInput.value);"; 
  html += "const kd = parseFloat(kdInput.value);"; 
  html += "if (isNaN(kp) || isNaN(ki) || isNaN(kd)) {"; 
  html += "alert('Invalid PID parameters. All values must be numbers.'); return;"; 
  html += "}"; 
  html += "if (kp < 0 || ki < 0 || kd < 0) {"; 
  html += "alert('PID parameters cannot be negative'); return;"; 
  html += "}"; 
  html += "if (kp > 100 || ki > 100 || kd > 100) {"; 
  html += "alert('PID parameters too large (max 100 each)'); return;"; 
  html += "}"; 
  html += "logDebug('Updating PID params: Kp=' + kp + ', Ki=' + ki + ', Kd=' + kd);"; 
  html += "fetch('/pid/params?kp=' + kp + '&ki=' + ki + '&kd=' + kd).then(function(response) {"; 
  html += "if (!response.ok) throw new Error('HTTP ' + response.status);"; 
  html += "return response.text();"; 
  html += "}).then(function(data) {"; 
  html += "logDebug('PID params update response: ' + data);"; 
  html += "alert('PID parameters updated successfully: Kp=' + kp + ', Ki=' + ki + ', Kd=' + kd);"; 
  html += "}).catch(function(error) {"; 
  html += "logDebug('Error updating PID params: ' + error.message);"; 
  html += "alert('Error updating PID parameters: ' + error.message);"; 
  html += "});"; 
  html += "}";
  
  html += "window.onload = function() { ";
  html += "logDebug('Page loaded, initializing dual sensor components...'); ";
  html += "try { initChart(); } catch(e) { logDebug('Chart initialization failed: ' + e.message); }";
  html += "logDebug('Starting WebSocket initialization...'); ";
  html += "initWebSocket(); ";
  html += "logDebug('Starting debug info updates...'); ";
  html += "setInterval(updateDebugInfo, 1000); ";
  html += "setInterval(updateHeaterRuntime, 1000); ";
  html += "document.getElementById('pidEnableBtn').disabled = " + String(pidEnabled) + ";";
  html += "document.getElementById('pidDisableBtn').disabled = " + String(!pidEnabled) + ";";
  html += "logDebug('All dual sensor and heater control initialization complete'); ";
  html += "};";
  
  script += "</script>";
  
  return script;
}
