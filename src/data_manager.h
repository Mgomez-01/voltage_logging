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
  bool heaterState;   // Heater relay state
  float pidValue;     // PID controller output
  float targetTemp;   // Target temperature at time of reading
};

// File system
extern const char* DATA_FILE;
extern File dataFile;
extern char logFileName[20];

// Statistics for debugging
extern unsigned long totalReadings;

// Buffer constants and variables
extern const int BUFFER_SIZE;
extern SensorReading readings[];
extern int bufferIndex;
extern bool bufferFull;

// Function declarations
void writeBufferToFile();
void initializeDataFile();
void clearDataFile();
void flushDataBuffer();
bool initializeSDCard();
void getNewLogFileName();

#endif // DATA_MANAGER_H
