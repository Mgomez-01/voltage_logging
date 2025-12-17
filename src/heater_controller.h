#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>
#include "data_manager.h"  // For SensorReading struct and buffer access

// Heater control pins and configuration
extern const int HEATER_PWM_PIN; // GPIO16 (D0) for PWM control of MOSFET
extern bool heaterEnabled;
extern float heaterDutyCycle; // Current PWM duty cycle (0-100%)
extern unsigned long heaterStartTime; // Time when heater was enabled
extern const unsigned long MAX_HEATER_TIME; // 10 min safety timeout
extern const float MAX_SAFE_TEMPERATURE; // Maximum safe temperature in °C

// PWM configuration
extern const int PWM_FREQUENCY; // PWM frequency in Hz
extern const int PWM_RANGE;     // PWM range (0-1000 for 0.1% resolution)

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

// Function declarations
void initializeHeaterPWM();
void setHeaterPower(float dutyCyclePercent);
void checkHeaterSafety();
void updatePIDController();

#endif // HEATER_CONTROLLER_H
