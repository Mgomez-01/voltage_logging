#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>

// One logged row (36 bytes in RAM). With averaging off, a row is one sample
// cycle (channels 0 -> 1 -> 2) and the ADC fields hold whole raw counts. With
// averaging on, a row is the mean of `samples` cycles and the ADC fields hold
// the mean counts (fractional). Some logged values are not stored per row to
// save RAM:
//  - thermistor_v / aux_v are computed from the raw ADC counts at write time
//    with the same function the temperature conversions use
//  - aux_temp (AUX Temp Sensor) is computed from auxThermistorAdc at write time
//  - the PID gains are written from their current values (the buffer is
//    flushed before the gains change, see handlePIDParams())
struct SensorReading {
  unsigned long timestamp;   // millis() at the PID Temp Sensor sample
  float voltage;             // Channel 0, voltageAdc / 1024 (0-1 V scale)
  float temperature;         // Channel 1, PID Temp Sensor, deg C (-999 = open/short sensor)
  float pidValue;            // Heater PWM duty cycle (0-100%), logged as pid_output
  float targetTemp;          // Target temperature at time of reading
  float voltageAdc;          // ADC count (0-1024), channel 0; mean count when averaged
  float thermistorAdc;       // ADC count, channel 1 (PID Temp Sensor); mean when averaged
  float auxThermistorAdc;    // ADC count, channel 2 (AUX Temp Sensor); mean when averaged
  bool heaterState;          // True if heater PWM duty > 0
  bool pidEnabled;           // True if the PID was controlling the heater
  uint8_t samples;           // Sample cycles in this row: 1 = raw, N = averaged
};

// Averaging (oversampling). Off by default: every sample cycle is logged raw.
// When on, `averageSamples` consecutive cycles are combined into one row.
// Set from the web page (/averaging); not saved across reboots.
extern bool averagingEnabled;
extern uint8_t averageSamples;
extern const uint8_t MIN_AVERAGE_SAMPLES;
extern const uint8_t MAX_AVERAGE_SAMPLES;
extern const uint8_t DEFAULT_AVERAGE_SAMPLES;

// CSV header written at the top of every log file (stored in flash/PROGMEM)
extern const char CSV_HEADER[];

// File system (SD card)
extern File dataFile;
extern char logFileName[20];

// Statistics for debugging
extern unsigned long totalReadings;

// Buffer constants and variables
extern const int BUFFER_SIZE;
extern SensorReading readings[];
extern int bufferIndex;
extern bool bufferFull;

// File rotation settings
extern const unsigned long MAX_FILE_SIZE;

// Function declarations
// Hand over one raw sample cycle (samples = 1, whole ADC counts, temperature
// already converted). Stores it as a row, or adds it to the running average
// and stores the averaged row once averageSamples cycles are in.
void addSampleCycle(const SensorReading& cycle);
void resetRowAverage();   // Drop a partly filled average (mode change, logging start)
void writeBufferToFile();
void initializeDataFile();
void clearDataFile();
void flushDataBuffer();
bool initializeSDCard();
void getNewLogFileName();
void emergencyFileCleanup();

#endif // DATA_MANAGER_H
