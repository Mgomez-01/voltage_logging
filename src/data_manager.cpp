#include "data_manager.h"
#include "safety_system.h"

// File system
const char* DATA_FILE = "/sensor_data.csv";
File dataFile;
char logFileName[20];

// Statistics for debugging
unsigned long totalReadings = 0;

// Buffer constants and variables
const int BUFFER_SIZE = 50;  // Reduced from 100 to save 1,400 bytes RAM
SensorReading readings[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

bool initializeSDCard() {
  if (!SD.begin(SS)) {
    Serial.println("ERROR: SD card initialization failed!");
    return false;
  }
  Serial.println("SD card initialized.");
  return true;
}

void getNewLogFileName() {
  strcpy(logFileName, "data.log");
  if (SD.exists(logFileName)) {
    int n = 1;
    do {
      sprintf(logFileName, "data_%d.log", n++);
    } while (SD.exists(logFileName));
  }
}

void writeBufferToFile() {
    // Feed watchdog before potentially long write operation
    feedWatchdog();
    Serial.println("[WATCHDOG] Fed before file write operation");
    
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (!dataFile) {
        Serial.println("ERROR: Could not open data file for writing!");
        return; // Exit if file cannot be opened
    }
    
    // Feed watchdog after successfully opening file
    feedWatchdog();

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
        // Feed watchdog every 10 writes during large operations
        if (i > 0 && i % 10 == 0) {
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
    
    // Feed watchdog before closing file
    feedWatchdog();
    dataFile.close();
    Serial.println("[WATCHDOG] Fed after file write completion");

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
  if (initializeSDCard()) {
    getNewLogFileName();
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("timestamp,voltage,temperature,heater_state,target_temp,pid_output");
      dataFile.close();
      Serial.print("Created new data file: ");
      Serial.println(logFileName);
    } else {
      Serial.println("ERROR: Could not create data file!");
    }
  }
}

void clearDataFile() {
  if (initializeSDCard()) {
    File root = SD.open("/");
    while (true) {
      File entry =  root.openNextFile();
      if (! entry) {
        break;
      }
      if (strstr(entry.name(), "data") != NULL && strstr(entry.name(), ".log") != NULL) {
        SD.remove(entry.name());
        Serial.print("Removed: ");
        Serial.println(entry.name());
      }
      entry.close();
    }
    root.close();
  }
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
