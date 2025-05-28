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
const float MAX_SAFE_TEMPERATURE = 120.0; // Maximum safe temperature in °C

// PID Controller variables
float targetTemperature = 65.0; // Default target temperature
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
void sendTemplatedPage(const char* filename);
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
  Serial.println("CD74HC4067 Multiplexer Version - **WATCHDOG FIX**");
  Serial.println("===============================================");

  // ... (Rest of setup remains the same) ...
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

  Serial.println("OK");

  server.begin();
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
  feedWatchdog(); // Feed at the start of every loop

  if (emergencyShutdown) {
    emergencyShutdownSystem();
    return;
  }
  server.handleClient();
  webSocket.loop();
  checkSafetyTimeout();
  if (dataLoggingEnabled) {
    if (millis() - lastSample >= SAMPLE_INTERVAL) {
      readSensors();
      lastSample = millis();
    }
    if (pidEnabled && millis() - lastPIDUpdate >= PID_INTERVAL) {
      updatePIDController();
      lastPIDUpdate = millis();
    }
    if (millis() - lastWebUpdate >= WEB_UPDATE_INTERVAL) {
      sendWebUpdate();
      lastWebUpdate = millis();
    }
  }
  if (millis() - lastDebugPrint >= DEBUG_INTERVAL) {
    printDebugStats();
    lastDebugPrint = millis();
  }
}

// ... (Keep all functions from setupMultiplexer to printDebugStats as they were, *except* sendTemplatedPage) ...
void setupMultiplexer() {
  Serial.print("Setting up CD74HC4067 multiplexer... ");
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  selectMuxChannel(VOLTAGE_CHANNEL);
  Serial.println("OK");
  Serial.println("Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4");
  Serial.println("Channel 0: Voltage sensor");
  Serial.println("Channel 1: Thermistor (100k NTC)");
}

void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(10);
}

void readSensors() {
  static float lastVoltage = 0.0;
  static float lastTemperature = 0.0;
  if (currentSensorChannel == VOLTAGE_CHANNEL) {
    selectMuxChannel(VOLTAGE_CHANNEL);
    delayMicroseconds(50);
    int adcValue = analogRead(ADC_PIN);
    lastVoltage = (adcValue / 1024.0);
    currentSensorChannel = THERMISTOR_CHANNEL;
  } else {
    selectMuxChannel(THERMISTOR_CHANNEL);
    delayMicroseconds(50);
    int adcValue = analogRead(ADC_PIN);
    lastTemperature = convertThermistorToTemperature(adcValue);
    readings[bufferIndex].timestamp = millis();
    readings[bufferIndex].voltage = lastVoltage;
    readings[bufferIndex].temperature = lastTemperature;
    readings[bufferIndex].currentChannel = THERMISTOR_CHANNEL;
    readings[bufferIndex].heaterState = relayState;
    readings[bufferIndex].pidValue = pidOutput;
    readings[bufferIndex].targetTemp = targetTemperature;
    totalReadings++;
    #if DEBUG_ADC
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
    currentSensorChannel = VOLTAGE_CHANNEL;
  }
}

float convertThermistorToTemperature(int adcValue) {
  if (adcValue <= 1) return -999; // Error value for no reading (check 0 and 1)
  float voltage = (adcValue / 1024.0) * 3.3; // Assuming 3.3V reference
  if (voltage >= 3.29) return -999; // Open circuit
  float resistance = SERIES_RESISTOR * voltage / (3.3 - voltage);
  if (resistance <= 0) return -999; // Avoid log(0) or log(-)
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
    if (!dataFile) {
        Serial.println("ERROR: Could not open data file for writing!");
        return; // Exit if file cannot be opened
    }

    // Determine how many readings to write. If buffer is full, write all.
    // Otherwise, write up to the current index.
    int readingsToWrite = bufferFull ? BUFFER_SIZE : bufferIndex;
    if (readingsToWrite == 0) {
        dataFile.close();
        return; // Nothing to write
    }

    float batchVoltageMin = 1.0, batchVoltageMax = 0.0, batchVoltageSum = 0.0;
    float batchTempMin = 999.0, batchTempMax = -999.0, batchTempSum = 0.0;
    int batchCount = 0;
    int validTempCount = 0;

    for (int i = 0; i < readingsToWrite; i++) {
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

        float v = readings[i].voltage;
        float t = readings[i].temperature;

        if (v < batchVoltageMin) batchVoltageMin = v;
        if (v > batchVoltageMax) batchVoltageMax = v;
        batchVoltageSum += v;

        if (t > -50 && t < 150) {
            if (t < batchTempMin) batchTempMin = t;
            if (t > batchTempMax) batchTempMax = t;
            batchTempSum += t;
            validTempCount++;
        }
        batchCount++;
    }
    dataFile.close();

    Serial.print("Wrote ");
    Serial.print(batchCount);
    Serial.println(" sensor readings to file");
    Serial.print("  Voltage - Min="); Serial.print(batchVoltageMin, 4);
    Serial.print("V, Max="); Serial.print(batchVoltageMax, 4);
    Serial.print("V, Avg="); Serial.print(batchVoltageSum / batchCount, 4); Serial.println("V");

    if (validTempCount > 0) {
      Serial.print("  Temperature - Min="); Serial.print(batchTempMin, 2);
      Serial.print("°C, Max="); Serial.print(batchTempMax, 2);
      Serial.print("°C, Avg="); Serial.print(batchTempSum / validTempCount, 2); Serial.println("°C");
    } else {
      Serial.println("  Temperature - No valid readings (thermistor not connected?)");
    }

    // Reset buffer index only after writing
    bufferIndex = 0;
    bufferFull = false; // Reset full flag
}


void sendWebUpdate() {
  connectedClients = webSocket.connectedClients();
  if (connectedClients > 0) {
    if (bufferIndex > 0 || bufferFull) {
      int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
      JsonDocument doc;
      doc["timestamp"] = readings[idx].timestamp;
      doc["voltage"] = readings[idx].voltage;
      doc["temperature"] = readings[idx].temperature;
      doc["type"] = "reading";
      doc["heaterState"] = readings[idx].heaterState;
      doc["targetTemp"] = readings[idx].targetTemp;
      doc["pidOutput"] = readings[idx].pidValue;
      doc["loggingEnabled"] = dataLoggingEnabled; // Send current logging state
      String jsonString;
      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
      totalWebSocketMessages++;
      #if DEBUG_WEBSOCKET
      if (totalWebSocketMessages % 100 == 0) {
        Serial.print("WebSocket message #"); Serial.print(totalWebSocketMessages);
        Serial.print(" sent to "); Serial.print(connectedClients);
        Serial.print(" clients: "); Serial.print(jsonString);
        Serial.print(" (buffer idx: "); Serial.print(idx);
        Serial.print(", V: "); Serial.print(readings[idx].voltage, 4);
        Serial.print("V, T: "); Serial.print(readings[idx].temperature, 2); Serial.println("°C)");
      }
      #endif
    }
  }
}

void initializeDataFile() {
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

void handleDataDownload() {
  Serial.println("HTTP: Data download requested");
  if (LittleFS.exists(DATA_FILE)) {
    Serial.println("Flushing buffer before download...");
    writeBufferToFile(); // **Ensure buffer is flushed**
    File file = LittleFS.open(DATA_FILE, "r");
    if (file) {
        if (file.size() <= 80) { // Check if only header (or very small)
            Serial.println("WARNING: Data file seems empty or contains only headers.");
        }
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
  selectMuxChannel(VOLTAGE_CHANNEL);
  delay(5);
  int voltageADC = analogRead(A0);
  float currentVoltage = (voltageADC / 1024.0);
  selectMuxChannel(THERMISTOR_CHANNEL);
  delay(5);
  int tempADC = analogRead(A0);
  float currentTemperature = convertThermistorToTemperature(tempADC);
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
  lastSample = millis();
  lastWebUpdate = millis();
  currentSensorChannel = VOLTAGE_CHANNEL;
  server.send(200, "text/plain", "Dual sensor logging started");
  Serial.println("HTTP: Dual sensor logging STARTED - now collecting voltage and temperature readings");
}

void handleStopLogging() {
  Serial.println("HTTP: Stop logging requested");
  dataLoggingEnabled = false;
  if (bufferIndex > 0) {
    Serial.println("Flushing remaining buffer to file...");
    writeBufferToFile(); // Ensure buffer is flushed
  } else {
    Serial.println("Buffer empty, nothing to flush.");
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
    // ... (Keep other WebSocket cases) ...
  }
}

// ... (Keep Heater Control functions) ...
void initializeRelay() {
  Serial.print("Initializing heater relay control... ");
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
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
  if (millis() - lastSafetyCheck > SAFETY_CHECK_INTERVAL * 3 && !emergencyShutdown) {
    Serial.println("WARNING: Hardware safety system not responding!");
    emergencyShutdown = true;
  }
  if (!emergencyShutdown && (bufferIndex > 0 || bufferFull)) {
    int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
    float currentTemp = readings[idx].temperature;
    if (currentTemp > MAX_SAFE_TEMPERATURE) {
      Serial.print("SAFETY: Over-temperature shutdown! Temp=");
      Serial.print(currentTemp);
      Serial.println("°C");
      emergencyShutdown = true;
    }
    if (relayState && (isnan(currentTemp) || currentTemp < -50 || currentTemp > 200)) {
      Serial.println("SAFETY: Temperature sensor failure - emergency shutdown");
      emergencyShutdown = true;
    }
  }
  if (emergencyShutdown) {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    heaterEnabled = false;
    pidEnabled = false;
  }
}

void updatePIDController() {
  if (!pidEnabled || bufferIndex == 0) return;
  int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
  float currentTemp = readings[idx].temperature;
  pidError = targetTemperature - currentTemp;
  float proportional = pidKp * pidError;
  pidIntegral += pidError * (PID_INTERVAL / 1000.0);
  if (pidIntegral > 100) pidIntegral = 100;
  if (pidIntegral < -100) pidIntegral = -100;
  float integral = pidKi * pidIntegral;
  float derivative = pidKd * (pidError - pidLastError) / (PID_INTERVAL / 1000.0);
  pidLastError = pidError;
  pidOutput = proportional + integral + derivative;
  if (pidOutput > 100) pidOutput = 100;
  if (pidOutput < 0) pidOutput = 0;
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
  pidEnabled = false;
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
  pidIntegral = 0;
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
  if (server.hasArg("kp")) { pidKp = server.arg("kp").toFloat(); updated = true; }
  if (server.hasArg("ki")) { pidKi = server.arg("ki").toFloat(); updated = true; }
  if (server.hasArg("kd")) { pidKd = server.arg("kd").toFloat(); updated = true; }
  if (updated) {
    Serial.print("HTTP: PID parameters updated - Kp=");
    Serial.print(pidKp);
    Serial.print(", Ki=");
    Serial.print(pidKi);
    Serial.print(", Kd=");
    Serial.println(pidKd);
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
  doc["heaterRuntime"] = relayState ? (millis() - relayOnTime) / 1000 : 0;
  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}


// ... (Keep Hardware Safety System functions) ...
void ICACHE_RAM_ATTR hardwareSafetyCheck() {
  lastSafetyCheck = millis();
  if (relayState && millis() - relayOnTime > MAX_HEATER_TIME) {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    heaterEnabled = false;
    pidEnabled = false;
    emergencyShutdown = true;
  }
}

void ICACHE_RAM_ATTR watchdogCheck() {
  if (!systemAlive && !emergencyShutdown) { // Only trigger if not already shut down
    Serial.println("!!! WATCHDOG STARVED - EMERGENCY SHUTDOWN !!!");
    digitalWrite(RELAY_PIN, LOW);
    emergencyShutdown = true;
    // Don't rely on hardware WDT, manage via flag
  }
  systemAlive = false;
}

void initializeSafetySystem() {
  // We will manage our own watchdog, but keep ESP WDT as a final backup
  ESP.wdtEnable(WATCHDOG_TIMEOUT);
  safetyTimer.attach_ms(SAFETY_CHECK_INTERVAL, hardwareSafetyCheck);
  watchdogTimer.attach_ms(1000, watchdogCheck); // Check every second
  systemAlive = true;
  emergencyShutdown = false;
  lastSafetyCheck = millis();
}

void feedWatchdog() {
  systemAlive = true;
  ESP.wdtFeed();
}

void emergencyShutdownSystem() {
  static unsigned long lastShutdownMessage = 0;
  digitalWrite(RELAY_PIN, LOW);
  relayState = false;
  heaterEnabled = false;
  pidEnabled = false;
  dataLoggingEnabled = false; // Stop logging
  if (millis() - lastShutdownMessage > 5000) {
    Serial.println("*** EMERGENCY SHUTDOWN ACTIVE ***");
    Serial.println("*** HEATER DISABLED - SYSTEM SAFE ***");
    Serial.println("*** Restart required to resume operation ***");
    lastShutdownMessage = millis();
  }
  server.handleClient(); // Keep server minimally responsive
  webSocket.loop();      // Keep WS minimally responsive
  delay(10); // Prevent tight loop in shutdown
}

// ... (Keep printDebugStats) ...
void printDebugStats() {
    Serial.println("=== DUAL SENSOR DEBUG STATS ===");
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.println("HARDWARE SAFETY SYSTEM:");
    Serial.print("  Status: "); Serial.println(emergencyShutdown ? "*** EMERGENCY SHUTDOWN ACTIVE ***" : "Active and monitoring");
    Serial.print("  Last safety check: "); Serial.print((millis() - lastSafetyCheck) / 1000); Serial.println(" seconds ago");
    Serial.print("  Watchdog: "); Serial.println(systemAlive ? "Fed" : "STARVED");
    Serial.print("Data Logging: "); Serial.println(dataLoggingEnabled ? "ENABLED" : "PAUSED");
    Serial.print("WiFi Status: AP Mode - Connected stations: "); Serial.println(WiFi.softAPgetStationNum());
    Serial.print("WebSocket clients: "); Serial.println(webSocket.connectedClients());
    Serial.print("Total sensor readings: "); Serial.println(totalReadings);
    Serial.print("Current buffer index: "); Serial.print(bufferIndex); Serial.print("/"); Serial.print(BUFFER_SIZE); Serial.print(" ("); Serial.print((bufferIndex * 100) / BUFFER_SIZE); Serial.println("% full)");
    Serial.print("WebSocket messages sent: "); Serial.println(totalWebSocketMessages);
    Serial.println("HEATER CONTROL STATUS:");
    Serial.print("  Heater: "); Serial.print(heaterEnabled ? "ENABLED" : "DISABLED"); Serial.print(", Relay: "); Serial.print(relayState ? "ON" : "OFF");
    if (relayState) { Serial.print(" (Runtime: "); Serial.print((millis() - relayOnTime) / 1000); Serial.print("s)"); } Serial.println();
    Serial.print("  PID Control: "); Serial.print(pidEnabled ? "ACTIVE" : "INACTIVE");
    if (pidEnabled) { Serial.print(" (Target: "); Serial.print(targetTemperature); Serial.print("°C, Output: "); Serial.print(pidOutput, 1); Serial.print("%, Error: "); Serial.print(pidError, 2); Serial.print("°C)"); } Serial.println();
    Serial.print("  PID Parameters: Kp="); Serial.print(pidKp); Serial.print(", Ki="); Serial.print(pidKi); Serial.print(", Kd="); Serial.println(pidKd);
    Serial.println("LIVE sensor readings:");
    selectMuxChannel(VOLTAGE_CHANNEL); delay(5); int voltageADC = analogRead(A0); float currentVoltage = (voltageADC / 1024.0);
    Serial.print("  Voltage (Ch0): "); Serial.print(voltageADC); Serial.print(" -> "); Serial.print(currentVoltage, 6); Serial.println("V");
    selectMuxChannel(THERMISTOR_CHANNEL); delay(5); int tempADC = analogRead(A0); float currentTemperature = convertThermistorToTemperature(tempADC);
    Serial.print("  Temperature (Ch1): "); Serial.print(tempADC); Serial.print(" -> ");
    if (currentTemperature > -50 && currentTemperature < 150) { Serial.print(currentTemperature, 3); Serial.println("°C"); } else { Serial.println("N/A (no thermistor or bad reading)"); }
    if (dataLoggingEnabled && (bufferIndex > 0 || bufferFull)) {
        Serial.println("Last 3 BUFFERED sensor readings:");
        for (int i = 0; i < 3; i++) {
            int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
            if (bufferFull || idx < bufferIndex) {
                Serial.print("  [" + String(idx) + "] "); Serial.print(readings[idx].voltage, 4); Serial.print("V | "); Serial.print(readings[idx].temperature, 2); Serial.print("°C @ "); Serial.print(readings[idx].timestamp); Serial.println("ms");
            }
        }
    } else if (!dataLoggingEnabled) { Serial.println("No buffered readings (logging paused)"); }
    Serial.print("Free heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
    Serial.println("===================================="); Serial.println();
}

void sendTemplatedPage(const char* filename) {
    File templateFile = LittleFS.open(filename, "r");
    if (!templateFile) {
        Serial.println("Failed to open template file for reading");
        server.send(500, "text/plain", "Internal Server Error: Template file not found.");
        return;
    }

    Serial.print("Reading template file (");
    Serial.print(templateFile.size());
    Serial.println(" bytes)...");
    Serial.println("Starting chunked template response...");

    server.chunkedResponseModeStart(200, "text/html");

    String line;
    while (templateFile.available()) {
        line = templateFile.readStringUntil('\n');

        // Perform replacements
        line.replace("{{WIFI_IP}}", WiFi.softAPIP().toString());
        line.replace("{{ADC_PIN_NUM}}", String(A0));
        line.replace("{{LOGGING_STATUS_TEXT_INITIAL}}", dataLoggingEnabled ? "ACTIVE" : "PAUSED");
        line.replace("{{LOGGING_STATUS_COLOR_INITIAL}}", dataLoggingEnabled ? "#28a745" : "#dc3545");
        line.replace("{{START_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "disabled" : "");
        line.replace("{{STOP_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "" : "disabled");
        line.replace("{{LOGGING_DESCRIPTION_INITIAL}}", dataLoggingEnabled ? "Currently collecting dual sensor readings at 500Hz" : "Click 'Start Logging' to begin data collection");
        line.replace("{{RELAY_STATUS_TEXT_INITIAL}}", relayState ? "ON" : "OFF");
        line.replace("{{HEATER_INDICATOR_CLASS_INITIAL}}", relayState ? "heater-on" : "heater-off");
        line.replace("{{TARGET_TEMP_INITIAL}}", String(targetTemperature, 1));
        line.replace("{{MAX_SAFE_TEMP_VALUE}}", String(MAX_SAFE_TEMPERATURE));
        line.replace("{{MAX_SAFE_TEMP_VALUE_JS}}", String(MAX_SAFE_TEMPERATURE));
        line.replace("{{PID_STATUS_TEXT_INITIAL}}", pidEnabled ? "ENABLED" : "DISABLED");
        line.replace("{{PID_OUTPUT_INITIAL}}", String(pidOutput, 1));
        line.replace("{{PID_KP_INITIAL}}", String(pidKp, 2));
        line.replace("{{PID_KI_INITIAL}}", String(pidKi, 2));
        line.replace("{{PID_KD_INITIAL}}", String(pidKd, 2));
        line.replace("{{JS_HEATER_STATE_INITIAL}}", relayState ? "true" : "false");
        line.replace("{{JS_PID_ENABLED_INITIAL}}", pidEnabled ? "true" : "false");

        server.sendContent(line + "\n");

        // **FIX:** Feed watchdog during long operation
        feedWatchdog();
        delay(1);
        yield();
    }

    templateFile.close();
    server.sendContent(""); // End chunked response
    Serial.println("Chunked template response finished.");
}

void handleRoot() {
  Serial.println("HTTP: Serving root page to client (STREAMING)");
  sendTemplatedPage("/index.template.html");
}
