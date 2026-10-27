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

// Measurement circuit (see sensor_manager.cpp)
extern const float THERMISTOR_SUPPLY_VOLTAGE; // Rail feeding both dividers (measured 3.32 V)
extern const float ADC_FULL_SCALE_VOLTAGE;    // 3.2 V: A0 full scale with the board's 220k/100k divider
extern const float ADC_INPUT_RESISTANCE;      // 320k board divider loading the selected channel
extern const float ADC_CALIBRATION_GAIN;      // Calibration multiplier for the A0 voltage (1.0 = uncalibrated)

// PID Temp Sensor configuration (channel 1)
extern const float THERMISTOR_NOMINAL;   // R25, calibrated (datasheet 100k)
extern const float TEMPERATURE_NOMINAL;  // 25°C
extern const float B_COEFFICIENT;        // Beta, calibrated (datasheet 3950)
extern const float SERIES_RESISTOR;      // Series resistor (measured 98k)

// AUX Temp Sensor configuration (channel 2)
extern const float AUX_THERMISTOR_NOMINAL;   // R25, calibrated (datasheet 10k)
extern const float AUX_TEMPERATURE_NOMINAL;  // 25°C
extern const float AUX_B_COEFFICIENT;        // Beta, calibrated (datasheet 3950)
extern const float AUX_SERIES_RESISTOR;      // 2.0k series resistor

// Current sensor channel being read
extern int currentSensorChannel;

// Function declarations
void setupMultiplexer();
void selectMuxChannel(int channel);
// ADC counts are float so averaged (oversampled) counts keep their fraction;
// a raw analogRead() count converts implicitly.
float thermistorAdcToVoltage(float adcValue);  // A0 voltage (0-3.2 V, calibrated) used by the conversion
float convertThermistorToTemperature(float adcValue);     // PID Temp Sensor
float convertAuxThermistorToTemperature(float adcValue);  // AUX Temp Sensor
void testSensorChannels();

#endif // SENSOR_MANAGER_H
