#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

// External objects that will be defined elsewhere
extern ESP8266WebServer server;
extern WebSocketsServer webSocket;

// WiFi configuration
extern const char* ssid;
extern const char* password;

// Function declarations
void initializeWiFi();
void printWiFiStatus();

#endif // WIFI_MANAGER_H
