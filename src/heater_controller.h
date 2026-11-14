#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>
#include "data_manager.h"  // For SensorReading struct and buffer access

// Heater control pins and configuration
extern const int HEATER_PWM_PIN; // GPIO16 (D0) for PWM control of MOSFET
extern bool heaterEnabled;
extern float heaterDutyCycle; // Current PWM duty cycle (0-100%)
extern unsigned long heaterStartTime; // Time when heater was enabled
extern const unsigned long MAX_HEATER_TIME; // 6 h continuous-heating limit
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
extern float pidLastInput;  // Last measured temperature, for derivative on measurement
extern const unsigned long PID_INTERVAL; // PID update interval in ms
extern unsigned long lastPIDUpdate;

// Which sensor the PID controls (selectable from the web page)
enum ControlSensor : uint8_t {
  CONTROL_PID_SENSOR = 0,  // PID Temp Sensor, channel 1 (sheet, next to the heater)
  CONTROL_AUX_SENSOR = 1   // AUX Temp Sensor, channel 2 (sample)
};
extern ControlSensor controlSensor;
extern float sensorGains[2][3];       // [sensor][Kp, Ki, Kd]; the active sensor's are pidKp/Ki/Kd
extern float sheetLimit;              // Sheet cap (°C) while controlling on AUX
extern const float SHEET_LIMIT_BAND;  // Power tapers to 0 over this many °C below sheetLimit
extern const float MIN_SHEET_LIMIT;
extern const float MAX_SHEET_LIMIT;
extern bool sheetLimited;             // PID output currently held back by the sheet limit
extern char pidFaultMessage[64];      // Why the PID stopped itself; empty if it didn't

// Function declarations
void initializeHeaterPWM();
void setControlSensor(ControlSensor sensor);  // Swaps in that sensor's gains, resets the PID state
void rememberActiveGains();                   // Store pidKp/Ki/Kd as the active sensor's gains
const char* controlSensorName(ControlSensor sensor);  // "pid" or "aux"
void setHeaterPower(float dutyCyclePercent);
void checkHeaterSafety();
void updatePIDController();
void resetPIDState();      // Clear integral/derivative memory (on enable or gain change)
// Called by readSensors() every sample cycle with the raw MCP3208 codes. The
// PID's input is the mean of all cycles since its previous update.
void addControlSample(uint16_t pidAdc, uint16_t auxAdc);

#endif // HEATER_CONTROLLER_H
