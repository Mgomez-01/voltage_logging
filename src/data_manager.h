#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <LittleFS.h>

// File system
extern const char* DATA_FILE;
extern File dataFile;

// Statistics for debugging
extern unsigned long totalReadings;

// External references needed by data manager
extern const int BUFFER_SIZE;
extern struct SensorReading readings[];
extern int bufferIndex;
extern bool bufferFull;

// Function declarations
void writeBufferToFile();
void initializeDataFile();
void clearDataFile();
void flushDataBuffer();

#endif // DATA_MANAGER_H
