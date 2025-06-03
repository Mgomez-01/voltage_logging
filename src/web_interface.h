#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// External objects
extern ESP8266WebServer server;
extern WebSocketsServer webSocket;

// External variables needed by web interface
extern unsigned long totalWebSocketMessages;
extern unsigned long connectedClients;
extern bool dataLoggingEnabled;
extern unsigned long lastSample;
extern unsigned long lastWebUpdate;
extern int currentSensorChannel;

// External buffer references
extern const int BUFFER_SIZE;
extern struct SensorReading readings[];
extern int bufferIndex;
extern bool bufferFull;
extern unsigned long totalReadings;

// Function declarations
void setupWebServer();
void setupWebSocket();
void sendWebUpdate();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendTemplatedPage(const char* filename);

// HTTP handler functions
void handleRoot();
void handleDataDownload();
void handleClearData();
void handleStatus();
void handleStartLogging();
void handleStopLogging();
void handleRelayOn();
void handleRelayOff();
void handleSetTemperature();
void handlePIDEnable();
void handlePIDDisable();
void handlePIDParams();
void handleHeaterStatus();

#endif // WEB_INTERFACE_H
