#include "heater_controller.h"

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

// Debug configuration (accessing from main file defines)
#define DEBUG_HEATER 1
#define DEBUG_PID 1

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

void checkHeaterSafety() {
  // This function needs access to the safety system variables and buffer
  // The actual safety logic will be called from main loop
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
