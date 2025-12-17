#include "web_interface.h"
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "heater_controller.h"
#include "data_manager.h"
#include "safety_system.h"

// Statistics for debugging
unsigned long totalWebSocketMessages = 0;
unsigned long connectedClients = 0;

// Debug configuration (accessing from main file defines)
#define DEBUG_WEBSOCKET 1

void setupWebServer() {
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

  // SD card routes
  server.on("/logs", handleListLogs);
  server.on("/logs/delete", handleDeleteLog);

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
        line.replace("{{ADC_PIN_NUM}}", String(A0));
        line.replace("{{LOGGING_STATUS_TEXT_INITIAL}}", dataLoggingEnabled ? "ACTIVE" : "PAUSED");
        line.replace("{{LOGGING_STATUS_COLOR_INITIAL}}", dataLoggingEnabled ? "#28a745" : "#dc3545");
        line.replace("{{START_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "disabled" : "");
        line.replace("{{STOP_LOGGING_DISABLED_ATTR}}", dataLoggingEnabled ? "" : "disabled");
        line.replace("{{LOGGING_DESCRIPTION_INITIAL}}", dataLoggingEnabled ? "Currently collecting dual sensor readings at 500Hz" : "Click 'Start Logging' to begin data collection");
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
  if (server.hasArg("file")) {
    String filename = server.arg("file");
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
    server.send(400, "text/plain", "Missing file parameter");
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
  doc["heaterState"] = (heaterDutyCycle > 0);
  doc["heaterDutyCycle"] = heaterDutyCycle;
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
  flushDataBuffer();
  server.send(200, "text/plain", "Dual sensor logging stopped");
  Serial.println("HTTP: Dual sensor logging STOPPED - readings paused");
}

void handleRelayOn() {
  Serial.println("HTTP: Heater ON requested (manual 100% PWM)");
  heaterEnabled = true;
  pidEnabled = false;  // Disable PID for manual control
  setHeaterPower(100.0);  // Set to 100% PWM
  server.send(200, "text/plain", "Heater turned ON at 100%");
}

void handleRelayOff() {
  Serial.println("HTTP: Heater OFF requested");
  heaterEnabled = false;
  pidEnabled = false;
  setHeaterPower(0.0);  // Set to 0% PWM
  server.send(200, "text/plain", "Heater turned OFF");
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
  doc["heaterDutyCycle"] = heaterDutyCycle;
  doc["heaterState"] = (heaterDutyCycle > 0);
  doc["targetTemp"] = targetTemperature;
  doc["pidEnabled"] = pidEnabled;
  doc["pidOutput"] = pidOutput;
  doc["pidError"] = pidError;
  doc["pidKp"] = pidKp;
  doc["pidKi"] = pidKi;
  doc["pidKd"] = pidKd;
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
