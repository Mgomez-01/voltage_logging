#include "data_manager.h"
#include "safety_system.h"

// File system
const char* DATA_FILE = "/sensor_data.csv";
File dataFile;

// Statistics for debugging
unsigned long totalReadings = 0;

// Buffer constants and variables
const int BUFFER_SIZE = 100;
SensorReading readings[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

void writeBufferToFile() {
    // Feed watchdog before potentially long write operation
    feedWatchdog();
    
    dataFile = LittleFS.open(DATA_FILE, "a");
    if (!dataFile) {
        Serial.println("ERROR: Could not open data file for writing!");
        return; // Exit if file cannot be opened
    }

    // // Determine how many readings to write. If buffer is full, write all.
    // // Otherwise, write up to the current index.
    int readingsToWrite = bufferFull ? BUFFER_SIZE : bufferIndex;
    if (readingsToWrite == 0) {
        dataFile.close();
        return; // Nothing to write
    }

    float batchVoltageMin = 1.0, batchVoltageMax = 0.0, batchVoltageSum = 0.0;
    float batchTempMin = 999.0, batchTempMax = -999.0, batchTempSum = 0.0;
    int batchCount = 0;
    int validTempCount = 0;

    for (int i = 0; i < readingsToWrite; i++) {
        // Feed watchdog every 50 writes during large operations
        if (i > 0 && i % 50 == 0) {
            feedWatchdog();
        }
        
        dataFile.print(readings[i].timestamp);
        dataFile.print(",");
        dataFile.print(readings[i].voltage, 6);
        dataFile.print(",");
        dataFile.print(readings[i].temperature, 3);
        dataFile.print(",");
        dataFile.print(readings[i].heaterState ? 1 : 0);
        dataFile.print(",");
        dataFile.print(readings[i].targetTemp, 2);
        dataFile.print(",");
        dataFile.println(readings[i].pidValue, 2);

        float v = readings[i].voltage;
        float t = readings[i].temperature;

        if (v < batchVoltageMin) batchVoltageMin = v;
        if (v > batchVoltageMax) batchVoltageMax = v;
        batchVoltageSum += v;

        if (t > -50 && t < 150) {
            if (t < batchTempMin) batchTempMin = t;
            if (t > batchTempMax) batchTempMax = t;
            batchTempSum += t;
            validTempCount++;
        }
        batchCount++;
    }
    dataFile.close();

    Serial.print("Wrote ");
    Serial.print(batchCount);
    Serial.println(" sensor readings to file");
    Serial.print("  Voltage - Min="); Serial.print(batchVoltageMin, 4);
    Serial.print("V, Max="); Serial.print(batchVoltageMax, 4);
    Serial.print("V, Avg="); Serial.print(batchVoltageSum / batchCount, 4); Serial.println("V");

    if (validTempCount > 0) {
      Serial.print("  Temperature - Min="); Serial.print(batchTempMin, 2);
      Serial.print("°C, Max="); Serial.print(batchTempMax, 2);
      Serial.print("°C, Avg="); Serial.print(batchTempSum / validTempCount, 2); Serial.println("°C");
    } else {
      Serial.println("  Temperature - No valid readings (thermistor not connected?)");
    }

    // Reset buffer index only after writing
    bufferIndex = 0;
    bufferFull = false; // Reset full flag
}

void initializeDataFile() {
  if (!LittleFS.exists(DATA_FILE)) {
    dataFile = LittleFS.open(DATA_FILE, "w");
    if (dataFile) {
      dataFile.println("timestamp,voltage,temperature,heater_state,target_temp,pid_output");
      dataFile.close();
      Serial.println("Created new data file with heater control header");
    } else {
      Serial.println("ERROR: Could not create data file!");
    }
  } else {
    Serial.println("Data file exists, will append new data with heater control");
  }
}

void clearDataFile() {
  if (LittleFS.exists(DATA_FILE)) {
    File file = LittleFS.open(DATA_FILE, "r");
    if (file) {
      Serial.print("File size before clear: ");
      Serial.print(file.size());
      Serial.println(" bytes");
      file.close();
    }
  }
  LittleFS.remove(DATA_FILE);
  initializeDataFile();
  bufferIndex = 0;
  bufferFull = false;
  totalReadings = 0;
  Serial.println("Data cleared successfully - fresh CSV will contain only new dual sensor readings");
}

void flushDataBuffer() {
  if (bufferIndex > 0) {
    Serial.println("Flushing remaining buffer to file...");
    writeBufferToFile(); // Ensure buffer is flushed
  } else {
    Serial.println("Buffer empty, nothing to flush.");
  }
}
