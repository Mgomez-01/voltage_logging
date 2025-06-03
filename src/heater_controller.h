#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>

// Heater control pins and configuration
extern const int RELAY_PIN; // GPIO16 (D0) for relay control
extern bool heaterEnabled;
extern bool relayState;
extern unsigned long relayOnTime;
extern const unsigned long MAX_HEATER_TIME; // 10 min safety timeout
extern const float MAX_SAFE_TEMPERATURE; // Maximum safe temperature in °C

// PID Controller variables
extern float targetTemperature; // Default target temperature
extern bool pidEnabled;
extern float pidKp;  // Proportional gain
extern float pidKi;  // Integral gain
extern float pidKd;  // Derivative gain
extern float pidOutput;
extern float pidError;
extern float pidLastError;
extern float pidIntegral;
extern const unsigned long PID_INTERVAL; // PID update interval in ms
extern unsigned long lastPIDUpdate;

// External references needed by heater controller
extern int bufferIndex;
extern bool bufferFull;
extern const int BUFFER_SIZE;
extern struct SensorReading readings[];
extern volatile bool emergencyShutdown;

// Function declarations
void initializeRelay();
void setRelayState(bool state);
void checkHeaterSafety();
void updatePIDController();

#endif // HEATER_CONTROLLER_H
