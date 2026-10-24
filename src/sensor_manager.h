#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

// Multiplexer control pins (CD74HC4067)
extern const int MUX_S0;  // D1
extern const int MUX_S1;  // D2
extern const int MUX_S2;  // D3
extern const int MUX_S3;  // D4

// Sensor channels on multiplexer
extern const int VOLTAGE_CHANNEL;        // Channel 0: voltage measurement
extern const int THERMISTOR_CHANNEL;     // Channel 1: PID Temp Sensor
extern const int AUX_THERMISTOR_CHANNEL; // Channel 2: AUX Temp Sensor (not in the PID)

// ADC pin
extern const int ADC_PIN;

// PID Temp Sensor configuration (channel 1)
extern const float THERMISTOR_NOMINAL;   // 100k ohm at 25°C
extern const float TEMPERATURE_NOMINAL;  // 25°C
extern const float B_COEFFICIENT;        // Beta coefficient for typical 100k thermistor
extern const float SERIES_RESISTOR;      // 100k series resistor

// AUX Temp Sensor configuration (channel 2)
extern const float AUX_THERMISTOR_NOMINAL;   // 10k ohm at 25°C
extern const float AUX_TEMPERATURE_NOMINAL;  // 25°C
extern const float AUX_B_COEFFICIENT;        // 3950
extern const float AUX_SERIES_RESISTOR;      // 2.0k series resistor

// Current sensor channel being read
extern int currentSensorChannel;

// Function declarations
void setupMultiplexer();
void selectMuxChannel(int channel);
float thermistorAdcToVoltage(int adcValue);  // Divider voltage used by the conversion
float convertThermistorToTemperature(int adcValue);     // PID Temp Sensor
float convertAuxThermistorToTemperature(int adcValue);  // AUX Temp Sensor
void testSensorChannels();

#endif // SENSOR_MANAGER_H
