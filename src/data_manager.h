#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>

// One logged row (28 bytes in RAM). Some logged values are not stored per row
// to save RAM:
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
  uint16_t voltageAdc;       // Raw ADC count (0-1023), channel 0
  uint16_t thermistorAdc;    // Raw ADC count (0-1023), channel 1 (PID Temp Sensor)
  uint16_t auxThermistorAdc; // Raw ADC count (0-1023), channel 2 (AUX Temp Sensor)
  bool heaterState;          // True if heater PWM duty > 0
  bool pidEnabled;           // True if the PID was controlling the heater
};

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
void writeBufferToFile();
void initializeDataFile();
void clearDataFile();
void flushDataBuffer();
bool initializeSDCard();
void getNewLogFileName();
void emergencyFileCleanup();

#endif // DATA_MANAGER_H
