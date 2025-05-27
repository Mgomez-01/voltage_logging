#include <Arduino.h>          // Core Arduino functions and pin definitions
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// Fallback definition for A0 if not defined (for IDE/linter support)
#ifndef A0
#define A0 17  // ESP8266 ADC pin
#endif

// Debug configuration
#define DEBUG_SERIAL 1
#define DEBUG_ADC 1
#define DEBUG_WEBSOCKET 1
#define DEBUG_WIFI 1

// WiFi Access Point Configuration
const char* ssid = "ESP8266_VoltageLogger";
const char* password = "voltage123";

// Web server and WebSocket
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ADC and timing variables
const int ADC_PIN = A0;
unsigned long lastSample = 0;
unsigned long lastWebUpdate = 0;
unsigned long lastDebugPrint = 0;
const unsigned long SAMPLE_INTERVAL = 1; // Sample every 1ms for fast sampling
const unsigned long WEB_UPDATE_INTERVAL = 100; // Update web every 100ms
const unsigned long DEBUG_INTERVAL = 1000; // Debug print every 1 second

// Data collection control
bool dataLoggingEnabled = false;  // Start with logging PAUSED

// Data storage
struct VoltageReading {
  unsigned long timestamp;
  float voltage;
};

const int BUFFER_SIZE = 100;
VoltageReading readings[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// File system
const char* DATA_FILE = "/voltage_data.csv";
File dataFile;

// Statistics for debugging
unsigned long totalReadings = 0;
unsigned long totalWebSocketMessages = 0;
unsigned long connectedClients = 0;

// Function declarations
void readVoltage();
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

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println();
  Serial.println("===============================================");
  Serial.println("ESP8266 Voltage Logger - DEBUG MODE");
  Serial.println("===============================================");
  
  // Debug: Show what A0 maps to
  Serial.print("A0 pin number: ");
  Serial.println(A0);
  Serial.print("Initial ADC reading: ");
  int initialReading = analogRead(A0);
  Serial.print(initialReading);
  Serial.print(" (");
  Serial.print((initialReading / 1024.0), 4);
  Serial.println("V)");
  
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
  server.on("/status", handleStatus);  // Add status endpoint for polling fallback
  server.on("/start", handleStartLogging);  // Start data logging
  server.on("/stop", handleStopLogging);    // Stop data logging
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
  Serial.println("===============================================");
  Serial.println();
  
  // Take a few test readings
  Serial.println("Taking initial ADC readings:");
  for (int i = 0; i < 5; i++) {
    int reading = analogRead(A0);
    float voltage = (reading / 1024.0);
    Serial.print("Reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(reading);
    Serial.print(" -> ");
    Serial.print(voltage, 4);
    Serial.println("V");
    delay(100);
  }
  Serial.println();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  // Only sample when logging is enabled
  if (dataLoggingEnabled) {
    // Fast ADC sampling
    if (millis() - lastSample >= SAMPLE_INTERVAL) {
      readVoltage();
      lastSample = millis();
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

void readVoltage() {
  int adcValue = analogRead(ADC_PIN);
  float voltage = (adcValue / 1024.0); // Convert to 0-1V range
  
  // Store in buffer
  readings[bufferIndex].timestamp = millis();
  readings[bufferIndex].voltage = voltage;
  
  totalReadings++;
  
  #if DEBUG_ADC
  // Debug every 1000th reading to avoid spam
  if (totalReadings % 1000 == 0) {
    Serial.print("ADC Reading #");
    Serial.print(totalReadings);
    Serial.print(": ");
    Serial.print(adcValue);
    Serial.print(" -> ");
    Serial.print(voltage, 4);
    Serial.print("V (Buffer: ");
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
}

void writeBufferToFile() {
  dataFile = LittleFS.open(DATA_FILE, "a");
  if (dataFile) {
    int startIndex = bufferFull ? bufferIndex : 0;
    int endIndex = bufferFull ? BUFFER_SIZE : bufferIndex;
    
    // Calculate stats for this write batch
    float batchMin = 1.0, batchMax = 0.0, batchSum = 0.0;
    int batchCount = 0;
    
    for (int i = startIndex; i < endIndex; i++) {
      dataFile.print(readings[i].timestamp);
      dataFile.print(",");
      dataFile.println(readings[i].voltage, 6);
      
      // Track batch statistics
      float v = readings[i].voltage;
      if (v < batchMin) batchMin = v;
      if (v > batchMax) batchMax = v;
      batchSum += v;
      batchCount++;
    }
    dataFile.close();
    
    Serial.print("Wrote ");
    Serial.print(batchCount);
    Serial.print(" readings to file - Batch stats: Min=");
    Serial.print(batchMin, 6);
    Serial.print("V, Max=");
    Serial.print(batchMax, 6);
    Serial.print("V, Avg=");
    Serial.print(batchSum / batchCount, 6);
    Serial.println("V");
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
      doc["type"] = "reading";
      
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
        Serial.print(", buffered voltage: ");
        Serial.print(readings[idx].voltage, 6);
        Serial.println("V)");
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
      dataFile.println("timestamp,voltage");
      dataFile.close();
      Serial.println("Created new data file with header");
    } else {
      Serial.println("ERROR: Could not create data file!");
    }
  } else {
    Serial.println("Data file exists, will append new data");
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
      server.sendHeader("Content-Disposition", "attachment; filename=voltage_data.csv");
      server.streamFile(file, "text/csv");
      file.close();
      Serial.println("HTTP: Data file sent successfully");
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
  Serial.println("HTTP: Data cleared successfully - fresh CSV will contain only new readings");
}

void handleStatus() {
  Serial.println("HTTP: Status requested (polling mode)");
  
  // Get current reading
  int currentADC = analogRead(A0);
  float currentVoltage = (currentADC / 1024.0);
  
  // Create JSON response
  JsonDocument doc;
  doc["timestamp"] = millis();
  doc["voltage"] = currentVoltage;
  doc["adc"] = currentADC;
  doc["type"] = "reading";
  doc["totalReadings"] = totalReadings;
  doc["connectedClients"] = webSocket.connectedClients();
  doc["uptime"] = millis() / 1000;
  doc["loggingEnabled"] = dataLoggingEnabled;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  server.send(200, "application/json", jsonString);
  Serial.print("HTTP: Status sent - V:");
  Serial.print(currentVoltage, 4);
  Serial.print(", ADC:");
  Serial.print(currentADC);
  Serial.print(", Logging:");
  Serial.println(dataLoggingEnabled ? "ON" : "OFF");
}

void handleStartLogging() {
  Serial.println("HTTP: Start logging requested");
  dataLoggingEnabled = true;
  
  // Reset timing for clean start
  lastSample = millis();
  lastWebUpdate = millis();
  
  server.send(200, "text/plain", "Data logging started");
  Serial.println("HTTP: Data logging STARTED - now collecting readings");
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
  
  server.send(200, "text/plain", "Data logging stopped");
  Serial.println("HTTP: Data logging STOPPED - readings paused");
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
      doc["message"] = "WebSocket connected successfully";
      doc["timestamp"] = millis();
      String welcomeJson;
      serializeJson(doc, welcomeJson);
      webSocket.sendTXT(num, welcomeJson);
      Serial.println("WebSocket: Welcome message sent");
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

void printDebugStats() {
  Serial.println("=== DEBUG STATS ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
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
  
  Serial.print("Total ADC readings: ");
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
  
  // Compare live ADC vs buffered data
  int currentADC = analogRead(A0);
  float currentVoltage = (currentADC / 1024.0);
  Serial.print("LIVE ADC reading: ");
  Serial.print(currentADC);
  Serial.print(" -> ");
  Serial.print(currentVoltage, 6);
  Serial.println("V");
  
  // Show recent buffered readings for comparison (only if logging enabled)
  if (dataLoggingEnabled && (bufferIndex > 0 || bufferFull)) {
    Serial.println("Last 5 BUFFERED readings:");
    for (int i = 0; i < 5; i++) {
      int idx = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      if (bufferFull || idx < bufferIndex) {
        Serial.print("  [" + String(idx) + "] ");
        Serial.print(readings[idx].voltage, 6);
        Serial.print("V @ ");
        Serial.print(readings[idx].timestamp);
        Serial.println("ms");
      }
    }
    
    // Calculate statistics from current buffer
    float bufferMin = 1.0, bufferMax = 0.0, bufferSum = 0.0;
    int validReadings = bufferFull ? BUFFER_SIZE : bufferIndex;
    
    for (int i = 0; i < validReadings; i++) {
      float v = readings[i].voltage;
      if (v < bufferMin) bufferMin = v;
      if (v > bufferMax) bufferMax = v;
      bufferSum += v;
    }
    
    if (validReadings > 0) {
      Serial.print("Buffer stats - Min: ");
      Serial.print(bufferMin, 6);
      Serial.print("V, Max: ");
      Serial.print(bufferMax, 6);
      Serial.print("V, Avg: ");
      Serial.print(bufferSum / validReadings, 6);
      Serial.print("V (n=");
      Serial.print(validReadings);
      Serial.println(")");
    }
  } else if (!dataLoggingEnabled) {
    Serial.println("No buffered readings (logging paused)");
  }
  
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.println("==================");
  Serial.println();
}

String getIndexHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP8266 Voltage Logger</title>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }";
  html += ".container { max-width: 1200px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".header { text-align: center; margin-bottom: 30px; color: #333; }";
  html += ".controls { text-align: center; margin-bottom: 20px; }";
  html += ".btn { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; margin: 0 5px; text-decoration: none; display: inline-block; }";
  html += ".btn:hover { background-color: #45a049; }";
  html += ".btn.danger { background-color: #f44336; }";
  html += ".btn.danger:hover { background-color: #da190b; }";
  html += ".status { text-align: center; margin-bottom: 20px; padding: 10px; border-radius: 4px; }";
  html += ".status.connected { background-color: #d4edda; color: #155724; border: 1px solid #c3e6cb; }";
  html += ".status.disconnected { background-color: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }";
  html += ".chart-container { width: 100%; height: 400px; margin-bottom: 30px; }";
  html += ".log-container { height: 300px; overflow-y: auto; border: 1px solid #ddd; padding: 10px; background-color: #f9f9f9; font-family: monospace; font-size: 12px; }";
  html += ".current-reading { text-align: center; font-size: 24px; font-weight: bold; margin-bottom: 20px; color: #2196F3; }";
  html += ".stats { display: flex; justify-content: space-around; margin-bottom: 20px; }";
  html += ".stat { text-align: center; }";
  html += ".stat-value { font-size: 20px; font-weight: bold; color: #4CAF50; }";
  html += ".stat-label { font-size: 14px; color: #666; }";
  html += ".debug { background-color: #f0f0f0; padding: 10px; margin: 10px 0; border-radius: 4px; font-family: monospace; font-size: 12px; }";
  html += "</style></head><body>";
  
  html += "<div class=\"container\">";
  html += "<div class=\"header\">";
  html += "<h1>ESP8266 Voltage Logger</h1>";
  html += "<p>Real-time voltage monitoring and logging</p>";
  html += "</div>";
  
  // Add debug info section
  html += "<div class=\"debug\">";
  html += "<strong>Debug Info:</strong><br>";
  html += "ESP8266 IP: " + WiFi.softAPIP().toString() + "<br>";
  html += "WebSocket Port: 81<br>";
  html += "ADC Pin: A0 (pin " + String(A0) + ")<br>";
  html += "Uptime: <span id=\"uptime\">0</span> seconds<br>";
  html += "Chart Status: <span id=\"chartStatus\">Checking...</span><br>";
  html += "Logging Status: <span id=\"loggingStatus\">" + String(dataLoggingEnabled ? "ACTIVE" : "PAUSED") + "</span><br>";
  html += "Connection Status: <span id=\"connectionDebug\">Initializing...</span>";
  html += "</div>";
  
  // Add prominent logging control section
  html += "<div style=\"text-align:center;margin:20px 0;padding:20px;background-color:#f8f9fa;border-radius:8px;border:2px solid #dee2e6;\">";
  html += "<h3 style=\"margin-top:0;color:#495057;\">Data Logging Control</h3>";
  html += "<div style=\"font-size:18px;margin:10px 0;\">Status: <span id=\"mainLoggingStatus\" style=\"font-weight:bold;color:" + String(dataLoggingEnabled ? "#28a745" : "#dc3545") + ";\">" + String(dataLoggingEnabled ? "ACTIVE" : "PAUSED") + "</span></div>";
  html += "<button onclick=\"startLogging()\" class=\"btn\" id=\"startBtn\" style=\"background-color:#28a745;margin:5px;\" " + String(dataLoggingEnabled ? "disabled" : "") + ">▶ Start Logging</button>";
  html += "<button onclick=\"stopLogging()\" class=\"btn\" id=\"stopBtn\" style=\"background-color:#dc3545;margin:5px;\" " + String(dataLoggingEnabled ? "" : "disabled") + ">⏸ Stop Logging</button>";
  html += "<div style=\"font-size:14px;color:#6c757d;margin-top:10px;\">" + String(dataLoggingEnabled ? "Currently collecting voltage readings at 1000 Hz" : "Click 'Start Logging' to begin data collection") + "</div>";
  html += "</div>";
  
  html += "<div id=\"status\" class=\"status disconnected\">Connecting to WebSocket...</div>";
  html += "<div class=\"current-reading\">Current: <span id=\"currentVoltage\">--</span> V</div>";
  
  html += "<div class=\"stats\">";
  html += "<div class=\"stat\"><div class=\"stat-value\" id=\"minVoltage\">--</div><div class=\"stat-label\">Min (V)</div></div>";
  html += "<div class=\"stat\"><div class=\"stat-value\" id=\"maxVoltage\">--</div><div class=\"stat-label\">Max (V)</div></div>";
  html += "<div class=\"stat\"><div class=\"stat-value\" id=\"avgVoltage\">--</div><div class=\"stat-label\">Avg (V)</div></div>";
  html += "<div class=\"stat\"><div class=\"stat-value\" id=\"sampleCount\">0</div><div class=\"stat-label\">Samples</div></div>";
  html += "</div>";
  
  html += "<div class=\"controls\">";
  html += "<a href=\"/data.csv\" class=\"btn\">Download Data</a>";
  html += "<button onclick=\"clearData()\" class=\"btn danger\">Clear Data</button>";
  html += "<button onclick=\"toggleLogging()\" class=\"btn\" id=\"logToggle\">Pause Logging</button>";
  html += "</div>";
  
  html += "<div class=\"chart-container\"><canvas id=\"voltageChart\"></canvas></div>";
  html += "<div class=\"log-container\" id=\"logContainer\"><div>Voltage readings will appear here...</div></div>";
  html += "</div>";
  
  // Enhanced JavaScript with debugging
  html += "<script>";
  html += "let ws, chart, loggingEnabled = true, startTime = Date.now();";
  html += "let stats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  html += "let connectionAttempts = 0, lastMessageTime = 0, maxRetries = 10;";
  
  html += "function updateDebugInfo() {";
  html += "document.getElementById('uptime').textContent = Math.floor((Date.now() - startTime) / 1000);";
  html += "let status = 'Attempts: ' + connectionAttempts + '/' + maxRetries;";
  html += "if (lastMessageTime > 0) status += ', Last message: ' + Math.floor((Date.now() - lastMessageTime) / 1000) + 's ago';";
  html += "document.getElementById('connectionDebug').textContent = status;";
  html += "}";
  
  html += "function logDebug(msg) { console.log('[DEBUG] ' + msg); }";
  
  html += "function initChart() {";
  html += "logDebug('Checking if Chart.js is available...');";
  html += "if (typeof Chart === 'undefined') {";
  html += "logDebug('Chart.js not loaded - chart will be disabled');";
  html += "document.getElementById('chartStatus').textContent = 'Disabled (no internet)';";
  html += "document.getElementById('chartStatus').style.color = 'orange';";
  html += "document.getElementById('voltageChart').style.display = 'none';";
  html += "const chartContainer = document.querySelector('.chart-container');";
  html += "if (chartContainer) chartContainer.innerHTML = '<div style=\"text-align:center;padding:20px;background:#f0f0f0;border-radius:4px;color:#666;\"><strong>Real-time Chart Unavailable</strong><br>Chart.js requires internet connection<br>Voltage readings still work below</div>';";
  html += "return;";
  html += "}";
  html += "logDebug('Chart.js loaded successfully, initializing chart...');";
  html += "try {";
  html += "const ctx = document.getElementById('voltageChart').getContext('2d');";
  html += "chart = new Chart(ctx, {";
  html += "type: 'line',";
  html += "data: { labels: [], datasets: [{ label: 'Voltage (V)', data: [], borderColor: '#2196F3', backgroundColor: 'rgba(33, 150, 243, 0.1)', borderWidth: 2, fill: true, tension: 0.1 }] },";
  html += "options: { responsive: true, maintainAspectRatio: false, scales: { x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (ms)' } }, y: { title: { display: true, text: 'Voltage (V)' }, min: 0, max: 1 } }, plugins: { legend: { display: true } }, animation: { duration: 0 } }";
  html += "});";
  html += "document.getElementById('chartStatus').textContent = 'Active';";
  html += "document.getElementById('chartStatus').style.color = 'green';";
  html += "logDebug('Chart initialized successfully');";
  html += "} catch(e) {";
  html += "logDebug('Error initializing chart: ' + e.message);";
  html += "document.getElementById('chartStatus').textContent = 'Error: ' + e.message;";
  html += "document.getElementById('chartStatus').style.color = 'red';";
  html += "chart = null;";
  html += "}";
  html += "}";
  
  html += "function initWebSocket() {";
  html += "if (connectionAttempts >= maxRetries) {";
  html += "logDebug('Max WebSocket connection attempts reached, switching to polling mode');";
  html += "document.getElementById('status').textContent = 'Using HTTP Polling (WebSocket failed)';";
  html += "document.getElementById('status').className = 'status connected';";
  html += "startPolling();";
  html += "return;";
  html += "}";
  
  html += "connectionAttempts++;";
  html += "logDebug('WebSocket connection attempt #' + connectionAttempts);";
  
  html += "const hostname = window.location.hostname;";
  html += "const wsUrl = 'ws://' + hostname + ':81';";
  html += "logDebug('Hostname: ' + hostname);";
  html += "logDebug('WebSocket URL: ' + wsUrl);";
  html += "logDebug('Current page URL: ' + window.location.href);";
  
  html += "try {";
  html += "ws = new WebSocket(wsUrl);";
  html += "logDebug('WebSocket object created');";
  html += "} catch(e) {";
  html += "logDebug('Error creating WebSocket: ' + e.message);";
  html += "setTimeout(initWebSocket, 3000);";
  html += "return;";
  html += "}";
  
  html += "ws.onopen = function() { ";
  html += "logDebug('WebSocket connected successfully!'); ";
  html += "document.getElementById('status').textContent = 'WebSocket Connected'; ";
  html += "document.getElementById('status').className = 'status connected'; ";
  html += "};";
  
  html += "ws.onclose = function(event) { ";
  html += "logDebug('WebSocket closed - Code: ' + event.code + ', Reason: ' + event.reason + ', Clean: ' + event.wasClean); ";
  html += "document.getElementById('status').textContent = 'Disconnected - Retrying...'; ";
  html += "document.getElementById('status').className = 'status disconnected'; ";
  html += "setTimeout(initWebSocket, 2000); ";
  html += "};";
  
  html += "ws.onerror = function(error) { ";
  html += "logDebug('WebSocket error occurred: ' + JSON.stringify(error)); ";
  html += "document.getElementById('status').textContent = 'WebSocket Error - Retrying...'; ";
  html += "document.getElementById('status').className = 'status disconnected'; ";
  html += "};";
  
  html += "ws.onmessage = function(event) { ";
  html += "lastMessageTime = Date.now(); ";
  html += "logDebug('WebSocket message received: ' + event.data); ";
  html += "if (loggingEnabled) { ";
  html += "try { ";
  html += "const data = JSON.parse(event.data); ";
  html += "if (data.type === 'reading') addVoltageReading(data.timestamp, data.voltage); ";
  html += "} catch(e) { logDebug('Error parsing message: ' + e.message); } ";
  html += "} ";
  html += "};";
  html += "}";
  
  html += "function startPolling() {";
  html += "logDebug('Starting HTTP polling mode...');";
  html += "setInterval(function() {";
  html += "fetch('/status').then(function(response) {";
  html += "return response.json();";
  html += "}).then(function(data) {";
  html += "if (data.voltage !== undefined) {";
  html += "addVoltageReading(data.timestamp, data.voltage);";
  html += "}";
  html += "}).catch(function(error) {";
  html += "logDebug('Polling error: ' + error.message);";
  html += "});";
  html += "}, 500);";
  html += "}";
  
  html += "function addVoltageReading(timestamp, voltage) {";
  html += "document.getElementById('currentVoltage').textContent = voltage.toFixed(4);";
  html += "stats.min = Math.min(stats.min, voltage); stats.max = Math.max(stats.max, voltage); stats.sum += voltage; stats.count++;";
  html += "document.getElementById('minVoltage').textContent = stats.min.toFixed(4);";
  html += "document.getElementById('maxVoltage').textContent = stats.max.toFixed(4);";
  html += "document.getElementById('avgVoltage').textContent = (stats.sum / stats.count).toFixed(4);";
  html += "document.getElementById('sampleCount').textContent = stats.count;";
  
  html += "if (chart && typeof chart.update === 'function') {";
  html += "if (chart.data.labels.length > 500) { chart.data.labels.shift(); chart.data.datasets[0].data.shift(); }";
  html += "chart.data.labels.push(timestamp); chart.data.datasets[0].data.push(voltage); chart.update('none');";
  html += "} else {";
  html += "logDebug('Chart not available, skipping chart update');";
  html += "}";
  
  html += "const logContainer = document.getElementById('logContainer');";
  html += "const logEntry = document.createElement('div');";
  html += "logEntry.textContent = new Date(timestamp).toLocaleTimeString() + ' - ' + voltage.toFixed(4) + ' V';";
  html += "logContainer.appendChild(logEntry);";
  html += "while (logContainer.children.length > 1000) logContainer.removeChild(logContainer.firstChild);";
  html += "logContainer.scrollTop = logContainer.scrollHeight;";
  html += "}";
  
  html += "function clearData() {";
  html += "if (confirm('Are you sure you want to clear all data?')) {";
  html += "fetch('/clear').then(function() {";
  html += "if (chart && typeof chart.update === 'function') {";
  html += "chart.data.labels = []; chart.data.datasets[0].data = []; chart.update();";
  html += "} else {";
  html += "logDebug('Chart not available for clearing');";
  html += "}";
  html += "stats = { min: Infinity, max: -Infinity, sum: 0, count: 0 };";
  html += "document.getElementById('minVoltage').textContent = '--';";
  html += "document.getElementById('maxVoltage').textContent = '--';";
  html += "document.getElementById('avgVoltage').textContent = '--';";
  html += "document.getElementById('sampleCount').textContent = '0';";
  html += "document.getElementById('currentVoltage').textContent = '--';";
  html += "document.getElementById('logContainer').innerHTML = '<div>Data cleared. New readings will appear here...</div>';";
  html += "alert('Data cleared successfully');";
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
  html += "logDebug('Starting data logging...');";
  html += "fetch('/start').then(function(response) {";
  html += "return response.text();";
  html += "}).then(function(data) {";
  html += "logDebug('Start logging response: ' + data);";
  html += "document.getElementById('mainLoggingStatus').textContent = 'ACTIVE';";
  html += "document.getElementById('mainLoggingStatus').style.color = '#28a745';";
  html += "document.getElementById('loggingStatus').textContent = 'ACTIVE';";
  html += "document.getElementById('startBtn').disabled = true;";
  html += "document.getElementById('stopBtn').disabled = false;";
  html += "alert('Data logging started');";
  html += "}).catch(function(error) {";
  html += "logDebug('Error starting logging: ' + error.message);";
  html += "alert('Error starting logging');";
  html += "});";
  html += "}";
  
  html += "function stopLogging() {";
  html += "logDebug('Stopping data logging...');";
  html += "fetch('/stop').then(function(response) {";
  html += "return response.text();";
  html += "}).then(function(data) {";
  html += "logDebug('Stop logging response: ' + data);";
  html += "document.getElementById('mainLoggingStatus').textContent = 'PAUSED';";
  html += "document.getElementById('mainLoggingStatus').style.color = '#dc3545';";
  html += "document.getElementById('loggingStatus').textContent = 'PAUSED';";
  html += "document.getElementById('startBtn').disabled = false;";
  html += "document.getElementById('stopBtn').disabled = true;";
  html += "alert('Data logging stopped');";
  html += "}).catch(function(error) {";
  html += "logDebug('Error stopping logging: ' + error.message);";
  html += "alert('Error stopping logging');";
  html += "});";
  html += "}";
  
  html += "window.onload = function() { ";
  html += "logDebug('Page loaded, initializing components...'); ";
  html += "try { initChart(); } catch(e) { logDebug('Chart initialization failed: ' + e.message); }";
  html += "logDebug('Starting WebSocket initialization...'); ";
  html += "initWebSocket(); ";
  html += "logDebug('Starting debug info updates...'); ";
  html += "setInterval(updateDebugInfo, 1000); ";
  html += "logDebug('All initialization complete'); ";
  html += "};";
  
  html += "</script></body></html>";
  
  return html;
}
