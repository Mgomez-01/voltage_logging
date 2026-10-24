#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>

// Sensor data structure with heater control
struct SensorReading {
  unsigned long timestamp;
  float voltage;
  float temperature;
  int currentChannel; // Which channel was being sampled
  bool heaterState;   // True if heater PWM duty > 0
  float pidValue;     // Heater PWM duty cycle (0-100%), logged as pid_output
  float targetTemp;   // Target temperature at time of reading
};

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
