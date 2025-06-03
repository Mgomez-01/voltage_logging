#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

// Multiplexer control pins (CD74HC4067)
extern const int MUX_S0;  // D1
extern const int MUX_S1;  // D2
extern const int MUX_S2;  // D3
extern const int MUX_S3;  // D4

// Sensor channels on multiplexer
extern const int VOLTAGE_CHANNEL;    // Channel 0 for voltage measurement
extern const int THERMISTOR_CHANNEL; // Channel 1 for thermistor

// ADC pin
extern const int ADC_PIN;

// Thermistor configuration
extern const float THERMISTOR_NOMINAL;   // 100k ohm at 25°C
extern const float TEMPERATURE_NOMINAL;  // 25°C
extern const float B_COEFFICIENT;        // Beta coefficient for typical 100k thermistor
extern const float SERIES_RESISTOR;      // 100k series resistor

// Current sensor channel being read
extern int currentSensorChannel;

// Function declarations
void setupMultiplexer();
void selectMuxChannel(int channel);
float convertThermistorToTemperature(int adcValue);
void testSensorChannels();

#endif // SENSOR_MANAGER_H
