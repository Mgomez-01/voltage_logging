#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

// PCB version (pcb_voltage_logging): MCP3208 12-bit SPI ADC (U2).
// No multiplexer and no A0. The ADC shares the SPI bus with the SD card.

// MCP3208 chip select: D1 / GPIO5 (10k pull-up R12 on the board)
extern const int ADC_CS_PIN;

// MCP3208 channels (single-ended)
extern const int THERMISTOR_CHANNEL;     // CH1: PID Temp Sensor (J4)
extern const int AUX_THERMISTOR_CHANNEL; // CH2: AUX Temp Sensor (J5, selectable as the PID sensor)
// CH0 is not connected. CH3-CH7 go to the J7 spare header (not used yet).

extern const float ADC_COUNTS;           // 4096: code = 4096 * Vin / Vref

// Measurement circuit (see sensor_manager.cpp)
extern const float THERMISTOR_SUPPLY_VOLTAGE; // +3V3 rail; only for the logged thermistor_v / aux_v

// PID Temp Sensor configuration (channel 1)
extern const float THERMISTOR_NOMINAL;   // R25 (datasheet 100k)
extern const float TEMPERATURE_NOMINAL;  // 25°C
extern const float B_COEFFICIENT;        // Beta (datasheet 3950)
extern const float SERIES_RESISTOR;      // R1 on the PCB (100k)

// AUX Temp Sensor configuration (channel 2)
extern const float AUX_THERMISTOR_NOMINAL;   // R25 (datasheet 10k)
extern const float AUX_TEMPERATURE_NOMINAL;  // 25°C
extern const float AUX_B_COEFFICIENT;        // Beta (datasheet 3950)
extern const float AUX_SERIES_RESISTOR;      // R2 on the PCB (2k)

// Most recent raw sample of each sensor (°C, -999 = open/short sensor, NAN =
// no sample yet). Updated by readSensors() in main.cpp every sample cycle,
// whether or not logging is on. Used by the safety checks and the status page.
extern float livePidTemperature;
extern float liveAuxTemperature;
extern unsigned long liveSampleTime;   // millis() of that sample (0 = none yet)

// ADC
void setupAdc();
uint16_t readAdcChannel(uint8_t channel);                 // One conversion, 0-4095
float readAdcChannelAveraged(uint8_t channel, int count);  // Mean of `count` conversions
// Both temperatures now, from `count` conversions per sensor (for code that
// needs a reading outside the sample loop)
void readTemperaturesNow(float& pidTemp, float& auxTemp, int count = 16);

// ADC counts are float so averaged (oversampled) counts keep their fraction.
float thermistorAdcToVoltage(float adcValue);  // Node voltage (0-3.3 V), logged as thermistor_v / aux_v
float convertThermistorToTemperature(float adcValue);     // PID Temp Sensor
float convertAuxThermistorToTemperature(float adcValue);  // AUX Temp Sensor
void testSensorChannels();

#endif // SENSOR_MANAGER_H
