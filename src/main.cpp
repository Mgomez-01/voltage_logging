#include <Arduino.h>          // Core Arduino functions and pin definitions
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>  // For thermistor calculations
#include <Ticker.h>  // For hardware timer-based safety system
#include <SD.h>  // For file size monitoring
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "heater_controller.h"
#include "data_manager.h"
#include "web_interface.h"
#include "safety_system.h"

// Fallback definition for A0 if not defined (for IDE/linter support)
#ifndef A0
#define A0 17  // ESP8266 ADC pin
#endif

// Debug configuration
#define DEBUG_SERIAL 1      // General status - keep enabled
#define DEBUG_ADC 0         // Turn off to reduce memory usage
#define DEBUG_WEBSOCKET 0   // Turn off to reduce memory usage
#undef DEBUG_WIFI  // Undefine ESP8266WiFi library's version
#define DEBUG_WIFI 1        // WiFi status - keep enabled
#define DEBUG_HEATER 1      // Safety-critical - keep enabled
#define DEBUG_PID 0         // Turn off to reduce memory usage (enable only when tuning)



// Web server and WebSocket
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);





// Timing variables
unsigned long lastSample = 0;
unsigned long lastWebUpdate = 0;
unsigned long lastDebugPrint = 0;
const unsigned long SAMPLE_INTERVAL = 2; // Sample every 2ms (500Hz per channel, 1000Hz total)
const unsigned long WEB_UPDATE_INTERVAL = 500; // Update web every 500ms (reduced from 100ms to save memory)
const unsigned long DEBUG_INTERVAL = 1000; // Debug print every 1 second

// Data collection control
bool dataLoggingEnabled = false;  // Start with logging PAUSED






// Function declarations
void readSensors();
void printDebugStats();
void checkSafetyTimeout();

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

  // Setup heater PWM control
  initializeHeaterPWM();

  // Debug: Show what A0 maps to and test both channels
  Serial.print("A0 pin number: ");
  Serial.println(A0);

  testSensorChannels();

  // Initialize LittleFS
  Serial.print("Initializing LittleFS... ");
  if (!LittleFS.begin()) {
    Serial.println("FAILED!");
    Serial.println("ERROR: LittleFS initialization failed!");
    return;
  }
  Serial.println("OK");

  // Setup WiFi Access Point
  initializeWiFi();

  // Setup web server routes
  setupWebServer();

  // Setup WebSocket
  setupWebSocket();

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
  Serial.println("DATA LOGGING IS PAUSED BY DEFAULT");
  Serial.println("   Click 'Start Logging' in web interface to begin");
  Serial.println("   Voltage Channel 0, Temperature Channel 1");
  Serial.println("  PWM HEATER CONTROL READY");
  Serial.println("   PWM pin: GPIO16 (D0) - 10 Hz frequency");
  Serial.println("   MOSFET: AOD4144 N-channel");
  Serial.println("   PID control with smooth 0-100% power regulation");
  Serial.println("   Safety timeout: 30 minutes continuous operation");
  Serial.println("     HARDWARE SAFETY SYSTEM ACTIVE");
  Serial.println("     Independent timer-based monitoring");
  Serial.println("     Watchdog protection enabled");
  Serial.println("===============================================");
  Serial.println();
}

void loop() {
  feedWatchdog(); // Feed at the start of every loop

  if (emergencyShutdown) {
    emergencyShutdownSystem();
    return;
  }
  
  // Automatic memory protection - stop logging if heap gets critically low
  if (ESP.getFreeHeap() < 8000 && dataLoggingEnabled) {
    Serial.println("MEMORY: Auto-stopping logging due to critically low heap!");
    Serial.print("MEMORY: Free heap = ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    dataLoggingEnabled = false;
    flushDataBuffer();
  }
  
  server.handleClient();
  webSocket.loop();
  checkSafetyTimeout(); // General safety system monitoring
  
  // PID control runs independently of data logging
  if (pidEnabled && millis() - lastPIDUpdate >= PID_INTERVAL) {
    updatePIDController();
    lastPIDUpdate = millis();
  }
  
  // Data logging and sensor reading
  if (dataLoggingEnabled) {
    if (millis() - lastSample >= SAMPLE_INTERVAL) {
      readSensors();
      lastSample = millis();
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
    readings[bufferIndex].heaterState = (heaterDutyCycle > 0);  // Store as boolean: is heater active?
    readings[bufferIndex].pidValue = heaterDutyCycle;  // Store actual PWM duty cycle percentage
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

void checkSafetyTimeout() {
  if (millis() - lastSafetyCheck > SAFETY_CHECK_INTERVAL * 3 && !emergencyShutdown) {
    Serial.println("WARNING: Hardware safety system not responding!");
    emergencyShutdown = true;
  }
  // Call heater controller's safety check function
  checkHeaterSafety();
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
    
    // File size monitoring
    if (SD.exists(logFileName)) {
        File sizeCheck = SD.open(logFileName, FILE_READ);
        if (sizeCheck) {
            unsigned long fileSize = sizeCheck.size();
            sizeCheck.close();
            float fileSizeMB = fileSize / 1000000.0;
            float percentFull = (fileSize * 100.0) / MAX_FILE_SIZE;
            Serial.print("Current log file: ");
            Serial.print(logFileName);
            Serial.print(" (");
            Serial.print(fileSizeMB, 2);
            Serial.print("MB / ");
            Serial.print(MAX_FILE_SIZE / 1000000);
            Serial.print("MB max, ");
            Serial.print(percentFull, 1);
            Serial.println("% full)");
            
            if (percentFull > 90) {
                Serial.println("  ⚠️  File will rotate soon (>90% full)");
            }
        }
    }
    Serial.println("HEATER CONTROL STATUS:");
    Serial.print("  Heater: "); Serial.print(heaterEnabled ? "ENABLED" : "DISABLED"); 
    Serial.print(", PWM: "); Serial.print(heaterDutyCycle, 1); Serial.print("%");
    if (heaterDutyCycle > 0) { 
      Serial.print(" (Active for: "); 
      Serial.print((millis() - heaterStartTime) / 1000); 
      Serial.print("s)"); 
    } 
    Serial.println();
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
    
    // Memory warning
    if (ESP.getFreeHeap() < 10000) {
        Serial.println("⚠️  WARNING: Low memory! Free heap < 10KB");
        Serial.println("⚠️  Consider stopping data logging or reducing buffer size");
    } else if (ESP.getFreeHeap() < 15000) {
        Serial.println("⚠️  NOTICE: Memory getting low (< 15KB)");
    }
    
    Serial.println("===================================="); Serial.println();
}


