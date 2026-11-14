#include <Arduino.h>          // Core Arduino functions and pin definitions
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>  // For thermistor calculations
#include <Ticker.h>  // For hardware timer-based safety system
#include <SD.h>  // SD card log files (also used for file size monitoring)
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "heater_controller.h"
#include "data_manager.h"
#include "web_interface.h"
#include "safety_system.h"
#include "tuning.h"
#include "program.h"
#include "status_led.h"

// Debug configuration for main.cpp only. Each .cpp file is compiled separately,
// so these do not affect heater_controller.cpp or web_interface.cpp, which set
// their own DEBUG_* values.
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
// One sample cycle (PID + AUX Temp Sensor, one MCP3208 conversion each, ~60 us)
// every 6 ms: up to ~167 cycles/s, the same row rate as the old board.
// Cycles run all the time, logging or not. The PID uses the mean of every
// cycle since its last update (~83 cycles per 500 ms update), so its input is
// far quieter than any single row; logging faster would not help it. The rate
// is set by the SD card instead: raw rows are ~110 bytes, so 167 rows/s is
// ~66 MB/h (6 h: ~400 MB, in 50 MB files). Turn averaging on for long runs.
const unsigned long SAMPLE_INTERVAL = 6;
const unsigned long WEB_UPDATE_INTERVAL = 500; // Update web every 500ms (reduced from 100ms to save memory)
const unsigned long DEBUG_INTERVAL = 1000; // Debug print every 1 second

// Data collection control
bool dataLoggingEnabled = false;  // Start with logging PAUSED






// Function declarations
void readSensors();
void printDebugStats();
void checkSafetyTimeout();

void setup() {
  // TX only: RX (GPIO3) is the status LED's data line on the PCB. The firmware
  // never reads from Serial, so only output is needed.
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  // Heater off as early as possible (R3 also holds the gate low until here)
  initializeHeaterPWM();
  // Status LED after Serial.begin(): the LED driver takes over the RX pin
  setupStatusLed();
  delay(1000); // Give serial time to initialize

  Serial.println();
  Serial.println("===============================================");
  Serial.println(F("ESP8266 Temperature Logger (PID + AUX Temp Sensors)"));
  Serial.println(F("PCB version - MCP3208 12-bit ADC"));
  Serial.println("===============================================");

  // MCP3208 ADC (shares SPI with the SD card, initialised later)
  setupAdc();

  testSensorChannels();

  // Initialize LittleFS
  Serial.print("Initializing LittleFS... ");
  if (!LittleFS.begin()) {
    Serial.println("FAILED!");
    Serial.println("ERROR: LittleFS initialization failed!");
    return;
  }
  Serial.println("OK");

  // PID gains saved from the /tune page, if any
  loadSavedPidGains();

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
  Serial.println(F("   MCP3208 CH1: PID Temp Sensor, CH2: AUX Temp Sensor"));
  Serial.println("  PWM HEATER CONTROL READY");
  Serial.println("   PWM pin: GPIO16 (D0) - 32 Hz");
  Serial.println("   MOSFET: AOD4144 N-channel");
  Serial.println("   PID control with smooth 0-100% power regulation");
  Serial.print(F("   Safety timeout: ")); Serial.print(MAX_HEATER_TIME / 60000UL); Serial.println(F(" min continuous heating"));
  Serial.println("     HARDWARE SAFETY SYSTEM ACTIVE");
  Serial.println("     Independent timer-based monitoring");
  Serial.println("     Watchdog protection enabled");
  Serial.println("===============================================");
  Serial.println();
}

void loop() {
  feedWatchdog(); // Feed at the start of every loop
  statusLedLoop(); // Also runs during an emergency shutdown (flashing)

  if (emergencyShutdown) {
    stopTuning("emergency shutdown");  // No-op unless a tuning test is running
    stopProgram("emergency shutdown"); // No-op unless a setpoint program is running
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
  
  // PID tuning tests (/tune page): sample and advance, if one is running
  tuningLoop();

  // Setpoint program (/program page): advance the steps, if one is running
  programLoop();

  // PID control runs independently of data logging
  if (pidEnabled && millis() - lastPIDUpdate >= PID_INTERVAL) {
    updatePIDController();
    lastPIDUpdate = millis();
  }
  
  // Sensor sampling runs all the time: it feeds the PID and the safety checks,
  // and the log when logging is on
  if (millis() - lastSample >= SAMPLE_INTERVAL) {
    lastSample = millis();
    readSensors();
  }
  if (dataLoggingEnabled) {
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
  // One sample cycle: one conversion of each sensor, back to back (~60 us).
  // Updates the live readings (safety checks, status), adds the codes to the
  // PID's average, and, when logging, hands the cycle to addSampleCycle():
  // with averaging off it is logged as one row, with averaging on
  // averageSamples cycles become one row.
  uint16_t pidAdc = readAdcChannel(THERMISTOR_CHANNEL);
  uint16_t auxAdc = readAdcChannel(AUX_THERMISTOR_CHANNEL);
  unsigned long now = millis();
  float pidTemp = convertThermistorToTemperature(pidAdc);
  livePidTemperature = pidTemp;
  liveAuxTemperature = convertAuxThermistorToTemperature(auxAdc);
  liveSampleTime = now ? now : 1;   // 0 means "no sample yet"
  addControlSample(pidAdc, auxAdc); // PID input: mean of the cycles since its last update

  if (!dataLoggingEnabled) return;

  SensorReading cycle;
  cycle.timestamp = now;
  cycle.voltage = 0;                    // No voltage input on the PCB (column kept for old scripts)
  cycle.voltageAdc = 0;
  cycle.temperature = pidTemp;
  cycle.thermistorAdc = pidAdc;
  cycle.auxThermistorAdc = auxAdc;
  cycle.heaterState = (heaterDutyCycle > 0);  // Is the heater active?
  cycle.pidEnabled = pidEnabled;        // PID in control (vs manual/off)
  cycle.pidValue = heaterDutyCycle;     // Actual PWM duty cycle percentage
  cycle.targetTemp = targetTemperature;
  cycle.samples = 1;
  cycle.control = controlSensor;        // Sensor the PID controls (0 = PID, 1 = AUX)
  cycle.step = programStepNumber();     // Setpoint program step (0 = none)
  #if DEBUG_ADC
  static unsigned long cycleCount = 0;
  if (++cycleCount % 500 == 0) {
    Serial.print("Sample cycle #");
    Serial.print(cycleCount);
    Serial.print(": PID ");
    Serial.print(pidAdc);
    Serial.print(" -> ");
    Serial.print(pidTemp, 2);
    Serial.print("°C, AUX ");
    Serial.print(auxAdc);
    Serial.print(" (Buffer: ");
    Serial.print(bufferIndex);
    Serial.println(")");
  }
  #endif
  // Logged as a raw row, or added to the running average (data_manager.cpp)
  addSampleCycle(cycle);
}

void checkSafetyTimeout() {
  if (millis() - lastSafetyCheck > SAFETY_CHECK_INTERVAL * 3 && !emergencyShutdown) {
    Serial.println("WARNING: Hardware safety system not responding!");
    triggerEmergencyShutdown(SHUTDOWN_SAFETY_TIMER);
  }
  // Call heater controller's safety check function
  checkHeaterSafety();
}

// ... (Keep printDebugStats) ...
void printDebugStats() {
    Serial.println("=== SENSOR DEBUG STATS ===");
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.println("HARDWARE SAFETY SYSTEM:");
    Serial.print("  Status: ");
    if (emergencyShutdown) { Serial.print(F("*** EMERGENCY SHUTDOWN ACTIVE *** (")); Serial.print(shutdownReasonText()); Serial.println(")"); }
    else Serial.println("Active and monitoring");
    Serial.print("  Last safety check: "); Serial.print((millis() - lastSafetyCheck) / 1000); Serial.println(" seconds ago");
    Serial.print("  Watchdog: "); Serial.println(systemAlive ? "Fed" : "STARVED");
    Serial.print("Data Logging: "); Serial.println(dataLoggingEnabled ? "ENABLED" : "PAUSED");
    Serial.print("WiFi Status: AP Mode - Connected stations: "); Serial.println(WiFi.softAPgetStationNum());
    Serial.print("WebSocket clients: "); Serial.println(webSocket.connectedClients());
    Serial.print("Total logged rows: "); Serial.println(totalReadings);
    Serial.print("Averaging: ");
    if (averagingEnabled) { Serial.print("ON, "); Serial.print(averageSamples); Serial.println(" sample cycles per row"); }
    else { Serial.println("OFF (raw rows)"); }
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
    int tempADC = readAdcChannel(THERMISTOR_CHANNEL); float currentTemperature = convertThermistorToTemperature(tempADC);
    Serial.print(F("  PID Temp Sensor (CH1): ")); Serial.print(tempADC); Serial.print(" -> ");
    if (currentTemperature > -50 && currentTemperature < 150) { Serial.print(currentTemperature, 3); Serial.println("°C"); } else { Serial.println("N/A (no thermistor or bad reading)"); }
    int auxADC = readAdcChannel(AUX_THERMISTOR_CHANNEL); float currentAuxTemperature = convertAuxThermistorToTemperature(auxADC);
    Serial.print(F("  AUX Temp Sensor (CH2): ")); Serial.print(auxADC); Serial.print(" -> ");
    if (currentAuxTemperature > -50 && currentAuxTemperature < 150) { Serial.print(currentAuxTemperature, 3); Serial.println("°C"); } else { Serial.println("N/A (no thermistor or bad reading)"); }
    if (dataLoggingEnabled && (bufferIndex > 0 || bufferFull)) {
        Serial.println("Last 3 BUFFERED sensor readings:");
        for (int i = 0; i < 3; i++) {
            int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
            if (bufferFull || idx < bufferIndex) {
                Serial.print("  [" + String(idx) + "] PID "); Serial.print(readings[idx].temperature, 2); Serial.print("°C | AUX "); Serial.print(convertAuxThermistorToTemperature(readings[idx].auxThermistorAdc), 2); Serial.print("°C @ "); Serial.print(readings[idx].timestamp); Serial.println("ms");
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


