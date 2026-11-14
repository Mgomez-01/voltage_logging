#include "web_interface.h"
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "heater_controller.h"
#include "data_manager.h"
#include "safety_system.h"
#include "tuning.h"
#include "program.h"

// Statistics for debugging
unsigned long totalWebSocketMessages = 0;
unsigned long connectedClients = 0;

// Debug configuration for this file only (the DEBUG_* defines in main.cpp
// do not reach other .cpp files)
#define DEBUG_WEBSOCKET 1

void setupWebServer() {
  Serial.print("Setting up web server routes... ");
  server.on("/", handleRoot);
  server.on("/data.csv", handleDataDownload);
  server.on("/clear", handleClearData);
  server.on("/status", handleStatus);
  server.on("/start", handleStartLogging);
  server.on("/stop", handleStopLogging);
  server.on("/averaging", handleAveraging);

  // Heater control routes
  server.on("/relay/on", handleRelayOn);
  server.on("/relay/off", handleRelayOff);
  server.on("/temp/set", handleSetTemperature);
  server.on("/pid/enable", handlePIDEnable);
  server.on("/pid/disable", handlePIDDisable);
  server.on("/pid/params", handlePIDParams);
  server.on("/pid/sensor", handlePIDSensor);
  server.on("/heater/status", handleHeaterStatus);

  // SD card routes
  server.on("/logs", handleListLogs);
  server.on("/logs/delete", handleDeleteLog);
  
  // System control routes
  server.on("/system/reset", handleSystemReset);

  // PID tuning page and tests
  setupTuningRoutes();

  // Setpoint program (experiment) page
  setupProgramRoutes();

  Serial.println("OK");
  server.begin();
  Serial.println("Web server listening on port 80");
}

void setupWebSocket() {
  Serial.print("Setting up WebSocket server... ");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("OK");
  Serial.print("WebSocket server listening on port 81 at IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("WebSocket URL should be: ws://" + WiFi.softAPIP().toString() + ":81");
}

void sendWebUpdate() {
  connectedClients = webSocket.connectedClients();
  if (connectedClients > 0) {
    if (bufferIndex > 0 || bufferFull) {
      int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
      
      // Skip redundant updates to reduce memory allocations
      static float lastSentTemp = -999;
      static float lastSentAuxTemp = -999;
      static bool lastSentHeaterState = false;
      static float lastSentDuty = -1;
      static float lastSentTarget = -999;
      static bool lastSentPidEnabled = false;
      static uint8_t lastSentControl = 255;
      static unsigned long lastSentMs = 0;
      float currentTemp = readings[idx].temperature;
      float currentAuxTemp = convertAuxThermistorToTemperature(readings[idx].auxThermistorAdc);
      bool currentHeaterState = readings[idx].heaterState;

      // Only send if something changed noticeably (temperatures, heater
      // power, setpoint, PID on/off), or every 5 s so the live plot keeps moving
      if (abs(currentTemp - lastSentTemp) < 0.1 &&
          abs(currentAuxTemp - lastSentAuxTemp) < 0.1 &&
          currentHeaterState == lastSentHeaterState &&
          abs(readings[idx].pidValue - lastSentDuty) < 1.0 &&
          readings[idx].targetTemp == lastSentTarget &&
          readings[idx].pidEnabled == lastSentPidEnabled &&
          readings[idx].control == lastSentControl &&
          millis() - lastSentMs < 5000) {
        return;  // Skip this update - no significant change
      }

      lastSentTemp = currentTemp;
      lastSentAuxTemp = currentAuxTemp;
      lastSentHeaterState = currentHeaterState;
      lastSentDuty = readings[idx].pidValue;
      lastSentTarget = readings[idx].targetTemp;
      lastSentPidEnabled = readings[idx].pidEnabled;
      lastSentControl = readings[idx].control;
      lastSentMs = millis();
      
      // Use static document to avoid heap fragmentation
      static StaticJsonDocument<320> doc;
      doc.clear();
      
      doc["timestamp"] = readings[idx].timestamp;
      doc["temperature"] = readings[idx].temperature;   // PID Temp Sensor
      doc["auxTemperature"] = currentAuxTemp;            // AUX Temp Sensor
      doc["type"] = "reading";
      doc["heaterState"] = readings[idx].heaterState;
      doc["targetTemp"] = readings[idx].targetTemp;
      doc["pidOutput"] = readings[idx].pidValue;           // Heater PWM duty when sampled
      doc["pidEnabled"] = readings[idx].pidEnabled;
      doc["ctl"] = readings[idx].control;                   // 0 = PID sensor controls, 1 = AUX
      if (sheetLimited) doc["sheetLimited"] = true;
      doc["loggingEnabled"] = dataLoggingEnabled;
      
      char jsonBuffer[320];
      serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
      webSocket.broadcastTXT(jsonBuffer);
      
      totalWebSocketMessages++;
      #if DEBUG_WEBSOCKET
      if (totalWebSocketMessages % 100 == 0) {
        Serial.print("WebSocket message #"); Serial.print(totalWebSocketMessages);
        Serial.print(" sent to "); Serial.print(connectedClients);
        Serial.print(" clients: "); Serial.print(jsonBuffer);
        Serial.print(" (buffer idx: "); Serial.print(idx);
        Serial.print(", PID T: "); Serial.print(readings[idx].temperature, 2);
        Serial.print(F("°C, AUX T: ")); Serial.print(currentAuxTemp, 2); Serial.println("°C)");
      }
      #endif
    }
  }
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
      doc["message"] = "Sensor WebSocket connected successfully";
      doc["sensors"] = "pid_temp+aux_temp";
      doc["timestamp"] = millis();
      String welcomeJson;
      serializeJson(doc, welcomeJson);
      webSocket.sendTXT(num, welcomeJson);
      Serial.println("WebSocket: Welcome message sent for sensor logger");
      break;
    }
    case WStype_TEXT:
      Serial.printf("WebSocket[%u] Received text: %s\n", num, payload);
      break;
    case WStype_ERROR:
      Serial.printf("WebSocket[%u] Error occurred\n", num);
      break;
    default:
      break;
  }
}

void sendTemplatedPage(const char* filename) {
    // Feed watchdog at start of potentially long operation
    feedWatchdog();
    
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
        line.replace("{{ADC_PIN_NUM}}", "MCP3208");
        line.replace("{{LOGGING_STATUS_TEXT_INITIAL}}", dataLoggingEnabled ? "ACTIVE" : "PAUSED");
        line.replace("{{LOGGING_STATUS_COLOR_INITIAL}}", dataLoggingEnabled ? "#28a745" : "#dc3545");
        line.replace("{{START_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "disabled" : "");
        line.replace("{{STOP_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "" : "disabled");
        line.replace("{{LOGGING_DESCRIPTION_INITIAL}}", dataLoggingEnabled ? "Currently collecting sensor readings (up to ~167 rows per second)" : "Click 'Start Logging' to begin data collection");
        line.replace("{{AVG_CHECKED_ATTR}}", averagingEnabled ? "checked" : "");
        line.replace("{{CTL_PID_CHECKED}}", controlSensor == CONTROL_PID_SENSOR ? "checked" : "");
        line.replace("{{CTL_AUX_CHECKED}}", controlSensor == CONTROL_AUX_SENSOR ? "checked" : "");
        line.replace("{{SHEET_LIMIT_INITIAL}}", String(sheetLimit, 0));
        line.replace("{{SHEET_LIMIT_MIN}}", String(MIN_SHEET_LIMIT, 0));
        line.replace("{{SHEET_LIMIT_MAX}}", String(MAX_SHEET_LIMIT, 0));
        line.replace("{{SAFETY_TIMEOUT_MIN}}", String(MAX_HEATER_TIME / 60000UL));
        line.replace("{{AVG_SAMPLES_INITIAL}}", String(averageSamples));
        line.replace("{{AVG_SAMPLES_MIN}}", String(MIN_AVERAGE_SAMPLES));
        line.replace("{{AVG_SAMPLES_MAX}}", String(MAX_AVERAGE_SAMPLES));
        line.replace("{{RELAY_STATUS_TEXT_INITIAL}}", heaterDutyCycle > 0 ? "ON" : "OFF");
        line.replace("{{HEATER_INDICATOR_CLASS_INITIAL}}", heaterDutyCycle > 0 ? "heater-on" : "heater-off");
        line.replace("{{PWM_DUTY_CYCLE_INITIAL}}", String(heaterDutyCycle, 1));
        line.replace("{{TARGET_TEMP_INITIAL}}", String(targetTemperature, 1));
        line.replace("{{MAX_SAFE_TEMP_VALUE}}", String(MAX_SAFE_TEMPERATURE));
        line.replace("{{MAX_SAFE_TEMP_VALUE_JS}}", String(MAX_SAFE_TEMPERATURE));
        line.replace("{{PID_STATUS_TEXT_INITIAL}}", pidEnabled ? "ENABLED" : "DISABLED");
        line.replace("{{PID_OUTPUT_INITIAL}}", String(pidOutput, 1));
        line.replace("{{PID_KP_INITIAL}}", String(pidKp, 2));
        line.replace("{{PID_KI_INITIAL}}", String(pidKi, 2));
        line.replace("{{PID_KD_INITIAL}}", String(pidKd, 2));
        line.replace("{{JS_HEATER_STATE_INITIAL}}", heaterDutyCycle > 0 ? "true" : "false");
        line.replace("{{JS_PWM_DUTY_CYCLE_INITIAL}}", String(heaterDutyCycle, 1));
        line.replace("{{JS_PID_ENABLED_INITIAL}}", pidEnabled ? "true" : "false");

        server.sendContent(line + "\n");

        // Feed watchdog during long operation to prevent emergency shutdown
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

void handleDataDownload() {
  Serial.println("HTTP: Data download requested");
  if (!initializeSDCard()) {
    server.send(500, "text/plain", "SD card not found");
    return;
  }
  // No "file" argument (e.g. the main "Download Data" button): send the log
  // file currently being written. Its name changes every boot and on rotation.
  String filename = server.hasArg("file") ? server.arg("file") : String(logFileName);
  if (filename.length() > 0) {
    if (filename == logFileName) {
      flushDataBuffer(); // include rows still waiting in the RAM buffer
    }
    Serial.print("Downloading file: ");
    Serial.println(filename);

    if (SD.exists(filename)) {
      File file = SD.open(filename, "r");
      if (file) {
        server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
        server.streamFile(file, "text/csv");
        file.close();
        Serial.println("HTTP: Log file sent successfully");
      } else {
        server.send(404, "text/plain", "File not found");
        Serial.println("HTTP: ERROR - Could not open log file");
      }
    } else {
      server.send(404, "text/plain", "No data file exists");
      Serial.println("HTTP: ERROR - Log file does not exist");
    }
  } else {
    // Only reached if no file was given and no log file was created at boot
    server.send(404, "text/plain", "No current log file (SD card was not ready at boot)");
  }
}

void handleClearData() {
  Serial.println("HTTP: Clear data requested");
  if (!initializeSDCard()) {
    server.send(500, "text/plain", "SD card not found");
    return;
  }
  clearDataFile();
  server.send(200, "text/plain", "Data cleared");
  Serial.println("HTTP: Data cleared successfully");
}

void handleStatus() {
  Serial.println("HTTP: Status requested (polling mode)");
  // 16 conversions per sensor (~1 ms): a steadier reading than one sample
  float tempADC = readAdcChannelAveraged(THERMISTOR_CHANNEL, 16);
  float currentTemperature = convertThermistorToTemperature(tempADC);
  float auxADC = readAdcChannelAveraged(AUX_THERMISTOR_CHANNEL, 16);
  float currentAuxTemperature = convertAuxThermistorToTemperature(auxADC);
  JsonDocument doc;
  doc["timestamp"] = millis();
  doc["temperature"] = currentTemperature;            // PID Temp Sensor
  doc["auxTemperature"] = currentAuxTemperature;      // AUX Temp Sensor
  doc["adcFullScale"] = 4096;                          // MCP3208
  doc["temperatureADC"] = tempADC;
  doc["thermistorVoltage"] = thermistorAdcToVoltage(tempADC);
  doc["auxTemperatureADC"] = auxADC;
  doc["auxThermistorVoltage"] = thermistorAdcToVoltage(auxADC);
  doc["type"] = "reading";
  doc["totalReadings"] = totalReadings;
  doc["connectedClients"] = webSocket.connectedClients();
  doc["uptime"] = millis() / 1000;
  doc["loggingEnabled"] = dataLoggingEnabled;
  doc["heaterState"] = (heaterDutyCycle > 0);
  doc["heaterDutyCycle"] = heaterDutyCycle;
  doc["targetTemp"] = targetTemperature;
  doc["pidEnabled"] = pidEnabled;
  doc["pidOutput"] = pidOutput;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
  doc["control"] = controlSensorName(controlSensor);
  doc["sheetLimit"] = sheetLimit;
  doc["sheetLimited"] = sheetLimited;
  doc["pidFault"] = pidFaultMessage;
  doc["maxHeaterMinutes"] = MAX_HEATER_TIME / 60000UL;
  doc["averagingEnabled"] = averagingEnabled;
  doc["averageSamples"] = averageSamples;
  doc["emergencyShutdown"] = (bool)emergencyShutdown;
  doc["shutdownReason"] = shutdownReasonText();
  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
  Serial.print("HTTP: Status sent - PID T:");
  Serial.print(currentTemperature, 2);
  Serial.print(F("°C, AUX T:"));
  Serial.print(currentAuxTemperature, 2);
  Serial.print("°C, Logging:");
  Serial.println(dataLoggingEnabled ? "ON" : "OFF");
}

// Start logging to the SD card with the current settings (averaging etc.).
// Retries the SD setup if no log file exists yet. Returns false if logging
// can't start (no SD card / no log file). Used by Start Logging and by the
// setpoint program.
bool beginDataLogging() {
  // If the SD card was not ready at boot, no log file was created and every
  // buffer write would fail. Try again now (e.g. after the card was inserted).
  if (logFileName[0] == '\0') {
    Serial.println("LOGGING: No log file yet - retrying SD card setup");
    initializeDataFile();
  }
  if (logFileName[0] == '\0') {
    Serial.println("LOGGING: NOT started - SD card not ready");
    return false;
  }
  if (!dataLoggingEnabled) {
    dataLoggingEnabled = true;
    lastWebUpdate = millis();
    resetRowAverage();  // Don't mix cycles from before the pause into the first averaged row
  }
  Serial.print("LOGGING: STARTED - writing to ");
  Serial.println(logFileName);
  return true;
}

void handleStartLogging() {
  Serial.println("HTTP: Start logging requested");
  if (!beginDataLogging()) {
    server.send(503, "text/plain", "Cannot start logging: SD card not ready (no log file). Check the card and try again.");
    return;
  }
  server.send(200, "text/plain", "Sensor logging started");
}

void handleStopLogging() {
  Serial.println("HTTP: Stop logging requested");
  stopProgram("logging stopped from the main page");
  dataLoggingEnabled = false;
  flushDataBuffer();
  server.send(200, "text/plain", "Sensor logging stopped");
  Serial.println("HTTP: Sensor logging STOPPED - readings paused");
}

// /averaging?enabled=0|1[&n=2..100]
// Turns averaging (oversampling) on or off. Off: every sample cycle is logged
// raw (samples = 1). On: n sample cycles are averaged into each row. Rows
// already in the buffer keep their own samples value, so nothing is flushed;
// only a partly filled average is dropped.
void handleAveraging() {
  if (!server.hasArg("enabled") && !server.hasArg("n")) {
    server.send(400, "text/plain", "Missing enabled or n parameter");
    return;
  }
  if (server.hasArg("n")) {
    long n = server.arg("n").toInt();
    if (n < MIN_AVERAGE_SAMPLES || n > MAX_AVERAGE_SAMPLES) {
      server.send(400, "text/plain", "n must be " + String(MIN_AVERAGE_SAMPLES) + "-" + String(MAX_AVERAGE_SAMPLES));
      return;
    }
    averageSamples = (uint8_t)n;
  }
  if (server.hasArg("enabled")) {
    averagingEnabled = server.arg("enabled") == "1" || server.arg("enabled") == "true";
  }
  resetRowAverage();
  String msg = averagingEnabled
      ? "Averaging ON: " + String(averageSamples) + " sample cycles per row"
      : String("Averaging OFF: every sample cycle logged raw");
  Serial.print("HTTP: ");
  Serial.println(msg);
  server.send(200, "text/plain", msg);
}

void handleRelayOn() {
  Serial.println("HTTP: Heater ON requested (manual 100% PWM)");
  stopTuning("heater turned on from the main page");
  stopProgram("heater turned on from the main page");
  heaterEnabled = true;
  pidEnabled = false;  // Disable PID for manual control
  setHeaterPower(100.0);  // Set to 100% PWM
  server.send(200, "text/plain", "Heater turned ON at 100%");
}

void handleRelayOff() {
  Serial.println("HTTP: Heater OFF requested");
  stopTuning("heater turned off from the main page");
  stopProgram("heater turned off from the main page");
  heaterEnabled = false;
  pidEnabled = false;
  setHeaterPower(0.0);  // Set to 0% PWM
  server.send(200, "text/plain", "Heater turned OFF");
}

void handleSetTemperature() {
  if (server.hasArg("temp")) {
    stopTuning("setpoint changed from the main page");
    stopProgram("setpoint changed from the main page");
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
  stopTuning("PID enabled from the main page");
  stopProgram("PID enabled from the main page");
  pidEnabled = true;
  heaterEnabled = true;
  resetPIDState();
  lastPIDUpdate = millis() - PID_INTERVAL; // Force immediate PID update
  Serial.println("PID: Taking control of heater PWM");
  server.send(200, "text/plain", "PID control enabled - taking over heater control");
}

void handlePIDDisable() {
  Serial.println("HTTP: PID control DISABLED");
  stopTuning("PID disabled from the main page");
  stopProgram("PID disabled from the main page");
  pidEnabled = false;
  server.send(200, "text/plain", "PID control disabled");
}

void handlePIDParams() {
  bool updated = false;
  // The kp/ki/kd columns are written from the current gains when the buffer is
  // saved, so save the rows sampled under the old gains before changing them.
  if (server.hasArg("kp") || server.hasArg("ki") || server.hasArg("kd")) {
    stopTuning("PID gains changed");
    flushDataBuffer();
    if (bufferIndex > 0 || bufferFull) {
      Serial.println(F("WARNING: Buffer flush failed - unsaved rows will be logged with the new PID gains"));
    }
  }
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
    rememberActiveGains();  // These are now the selected control sensor's gains
    resetPIDState();
    server.send(200, "text/plain", "PID parameters updated");
  } else {
    server.send(400, "text/plain", "No parameters provided");
  }
}

// /pid/sensor?s=pid|aux[&limit=120]
// Selects the sensor the PID controls, and the sheet limit used while it
// controls on AUX. Each sensor keeps its own gains; the response has the gains
// now in use so the page can show them.
void handlePIDSensor() {
  bool changeSensor = server.hasArg("s");
  ControlSensor wanted = controlSensor;
  if (changeSensor) {
    String s = server.arg("s");
    if (s == "aux") wanted = CONTROL_AUX_SENSOR;
    else if (s == "pid") wanted = CONTROL_PID_SENSOR;
    else { server.send(400, "text/plain", "s must be pid or aux"); return; }
  }
  if (server.hasArg("limit")) {
    float limit = server.arg("limit").toFloat();
    if (limit < MIN_SHEET_LIMIT || limit > MAX_SHEET_LIMIT) {
      server.send(400, "text/plain", "Sheet limit must be " + String(MIN_SHEET_LIMIT, 0) + "-" + String(MAX_SHEET_LIMIT, 0) + " C");
      return;
    }
    sheetLimit = limit;
  }
  if (changeSensor && wanted != controlSensor) {
    stopTuning("control sensor changed");
    stopProgram("control sensor changed");
    // kp/ki/kd are written from the current gains when the buffer is saved:
    // save the rows sampled under the old sensor's gains first
    flushDataBuffer();
    setControlSensor(wanted);
  }
  JsonDocument doc;
  doc["control"] = controlSensorName(controlSensor);
  doc["sheetLimit"] = sheetLimit;
  doc["kp"] = pidKp; doc["ki"] = pidKi; doc["kd"] = pidKd;
  doc["pidEnabled"] = pidEnabled;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleHeaterStatus() {
  JsonDocument doc;
  doc["heaterEnabled"] = heaterEnabled;
  doc["heaterDutyCycle"] = heaterDutyCycle;
  doc["heaterState"] = (heaterDutyCycle > 0);
  doc["targetTemp"] = targetTemperature;
  doc["pidEnabled"] = pidEnabled;
  doc["pidOutput"] = pidOutput;
  doc["pidError"] = pidError;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
  doc["control"] = controlSensorName(controlSensor);
  doc["sheetLimit"] = sheetLimit;
  doc["sheetLimited"] = sheetLimited;
  doc["pidFault"] = pidFaultMessage;
  doc["heaterRuntime"] = (heaterDutyCycle > 0) ? (millis() - heaterStartTime) / 1000 : 0;
  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void handleListLogs() {
  Serial.println("HTTP: List logs requested");
  if (initializeSDCard()) {
    File root = SD.open("/");
    JsonDocument doc;
    JsonArray logs = doc.to<JsonArray>();
    while (true) {
      File entry =  root.openNextFile();
      if (! entry) {
        break;
      }
      if (strstr(entry.name(), "data") != NULL && strstr(entry.name(), ".log") != NULL) {
        JsonObject log = logs.add<JsonObject>();
        log["name"] = entry.name();
        log["size"] = entry.size();
      }
      entry.close();
    }
    root.close();
    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
  } else {
    server.send(500, "text/plain", "SD card not found");
  }
}

void handleDeleteLog() {
  if (!initializeSDCard()) {
    server.send(500, "text/plain", "SD card not found");
    return;
  }
  if (server.hasArg("file")) {
    String filename = server.arg("file");
    if (SD.remove(filename)) {
      server.send(200, "text/plain", "File deleted");
    } else {
      server.send(500, "text/plain", "Error deleting file");
    }
  } else {
    server.send(400, "text/plain", "Missing file parameter");
  }
}

void handleSystemReset() {
  Serial.println("HTTP: System reset requested");
  stopTuning("system reset");
  stopProgram("system reset");
  
  // Safety check - verify temperature is reasonable before allowing reset
  float currentTemp = convertThermistorToTemperature(readAdcChannelAveraged(THERMISTOR_CHANNEL, 16));
  
  // Don't allow reset if temperature is dangerously high
  if (currentTemp > MAX_SAFE_TEMPERATURE - 10) {
    Serial.print("RESET DENIED: Temperature too high (");
    Serial.print(currentTemp);
    Serial.println("°C)");
    server.send(403, "text/plain", "Cannot reset - temperature still too high. Wait for cooling.");
    return;
  }
  
  // Check if temperature sensor is working
  if (isnan(currentTemp) || currentTemp < -50 || currentTemp > 200) {
    Serial.println("RESET DENIED: Temperature sensor not reading properly");
    server.send(403, "text/plain", "Cannot reset - temperature sensor failure. Check thermistor connection.");
    return;
  }
  
  Serial.println("RESET: Safety checks passed, clearing emergency state");
  Serial.print("RESET: Current temperature: ");
  Serial.print(currentTemp);
  Serial.println("°C (safe)");
  
  // CRITICAL: Clear the data buffer FIRST to remove stale temperature readings
  // This prevents old high-temp readings from triggering another emergency
  // Do this BEFORE clearing emergency flag to prevent race condition
  bufferIndex = 0;
  bufferFull = false;
  Serial.println("RESET: Data buffer cleared (removed stale readings)");
  // Sampling stops during the shutdown, so the live reading is the one from
  // before it (maybe the over-temperature itself). Replace it with this read.
  livePidTemperature = currentTemp;
  liveSampleTime = millis() ? millis() : 1;
  
  // Small delay to ensure any in-flight safety checks see the cleared buffer
  delay(10);
  
  // Clear emergency shutdown flag (safe now that buffer is cleared)
  emergencyShutdown = false;
  shutdownReason = SHUTDOWN_NONE;
  Serial.println("RESET: Emergency shutdown flag cleared");
  
  // Reset heater state
  heaterEnabled = false;
  pidEnabled = false;
  setHeaterPower(0.0);
  
  // Reset PID state
  resetPIDState();
  pidOutput = 0;
  
  // Reset safety system state
  systemAlive = true;
  
  // Reset emergency cleanup flag (in safety_system.cpp)
  // This is handled by clearing emergencyShutdown flag which resets the static in emergencyShutdownSystem()
  
  // Keep data logging paused (user must manually restart)
  dataLoggingEnabled = false;
  
  Serial.println("RESET: System state cleared");
  Serial.println("RESET: All heater control disabled");
  Serial.println("RESET: Data logging paused");
  Serial.println("RESET: System ready for normal operation");
  Serial.println("RESET: User must manually re-enable desired features");
  
  server.send(200, "text/plain", "System reset successful. Emergency state cleared. Temperature: " + String(currentTemp, 1) + "°C");
}
