#include "wifi_manager.h"

// WiFi configuration constants
const char* ssid = "ESP8266_VoltageLogger";
const char* password = "voltage123";

void initializeWiFi() {
  Serial.print("Setting up WiFi Access Point... ");
  WiFi.mode(WIFI_AP);
  bool apResult = WiFi.softAP(ssid, password);
  
  if (apResult) {
    Serial.println("OK");
    printWiFiStatus();
  } else {
    Serial.println("FAILED!");
    Serial.println("ERROR: Could not create WiFi Access Point!");
  }
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.softAPmacAddress());
}
