#include "data_manager.h"
#include "safety_system.h"
#include "heater_controller.h"  // PID gains written with each row
#include "sensor_manager.h"     // thermistorAdcToVoltage()

// CSV header. The first six columns are the original format; the rest were
// appended so older analysis scripts that read by column name keep working.
// temperature / thermistor_adc / thermistor_v = PID Temp Sensor (channel 1)
// aux_temp / aux_adc / aux_v                  = AUX Temp Sensor (channel 2)
// Stored in flash (PROGMEM) to save RAM; print it with printCsvHeader().
const char CSV_HEADER[] PROGMEM =
    "timestamp,voltage,temperature,heater_state,target_temp,pid_output,"
    "voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,"
    "aux_temp,aux_adc,aux_v";

static void printCsvHeader(File& f) {
  f.println(FPSTR(CSV_HEADER));
}

// File system (SD card)
File dataFile;
char logFileName[20] = "";  // Current log file; stays empty if the SD card was not ready at boot

// Statistics for debugging
unsigned long totalReadings = 0;

// Buffer constants and variables
const int BUFFER_SIZE = 50;  // Reduced from 100 to save 1,400 bytes RAM
SensorReading readings[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// File rotation settings
const unsigned long MAX_FILE_SIZE = 50000000;  // 50MB max per file (prevents 4GB FAT32 limit)

// Write operation safety
bool writeInProgress = false;
unsigned long writeStartTime = 0;
const unsigned long MAX_WRITE_TIME = 5000;  // 5 second max write time (less than 8 sec watchdog)

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
    // Check if a write is already in progress (prevent concurrent writes)
    if (writeInProgress) {
        unsigned long elapsed = millis() - writeStartTime;
        if (elapsed > MAX_WRITE_TIME) {
            Serial.println("ERROR: Previous write operation timed out!");
            Serial.print("ERROR: Write took ");
            Serial.print(elapsed);
            Serial.println("ms (max allowed: 5000ms)");
            
            // Force close any open file handle
            if (dataFile) {
                dataFile.close();
                Serial.println("ERROR: Forced file close after timeout");
            }
            
            writeInProgress = false;
            
            // Clear buffer to prevent repeated failed writes
            bufferIndex = 0;
            bufferFull = false;
            Serial.println("ERROR: Buffer cleared to prevent corruption loop");
        } else {
            Serial.println("SKIP: Write already in progress, skipping this batch");
        }
        return;
    }
    
    // Mark write as in progress
    writeInProgress = true;
    writeStartTime = millis();
    
    // Feed watchdog before potentially long write operation
    feedWatchdog();
    Serial.println("[WATCHDOG] Fed before file write operation");
    
    // Check if we need to rotate to a new file (prevent 4GB limit crash)
    if (SD.exists(logFileName)) {
        feedWatchdog();  // Feed during rotation check
        File sizeCheck = SD.open(logFileName, FILE_READ);
        if (sizeCheck) {
            unsigned long currentSize = sizeCheck.size();
            sizeCheck.close();
            feedWatchdog();  // Feed after size check
            
            if (currentSize >= MAX_FILE_SIZE) {
                Serial.print("FILE ROTATION: Current file reached ");
                Serial.print(currentSize / 1000000);
                Serial.print("MB (limit: ");
                Serial.print(MAX_FILE_SIZE / 1000000);
                Serial.println("MB)");
                
                feedWatchdog();  // Feed before creating new file
                
                // Get next available log filename
                getNewLogFileName();
                
                // Create new file with header
                File newFile = SD.open(logFileName, FILE_WRITE);
                if (newFile) {
                    printCsvHeader(newFile);
                    newFile.close();
                    feedWatchdog();  // Feed after creating new file
                    Serial.print("FILE ROTATION: Created new log file: ");
                    Serial.println(logFileName);
                } else {
                    Serial.println("FILE ROTATION: ERROR - Could not create new file!");
                    writeInProgress = false;
                    return;
                }
            }
        }
    }
    
    // Open file for appending with timeout protection
    unsigned long openStart = millis();
    dataFile = SD.open(logFileName, FILE_WRITE);
    
    if (!dataFile) {
        Serial.println("ERROR: Could not open data file for writing!");
        writeInProgress = false;
        return; // Exit if file cannot be opened
    }
    
    unsigned long openTime = millis() - openStart;
    if (openTime > 1000) {
        Serial.print("WARNING: File open took ");
        Serial.print(openTime);
        Serial.println("ms (slow SD card?)");
    }
    
    // Feed watchdog after successfully opening file
    feedWatchdog();

    // Determine how many readings to write
    int readingsToWrite = bufferFull ? BUFFER_SIZE : bufferIndex;
    if (readingsToWrite == 0) {
        dataFile.close();
        writeInProgress = false;
        return; // Nothing to write
    }

    float batchVoltageMin = 1.0, batchVoltageMax = 0.0, batchVoltageSum = 0.0;
    float batchTempMin = 999.0, batchTempMax = -999.0, batchTempSum = 0.0;
    float batchAuxMin = 999.0, batchAuxMax = -999.0, batchAuxSum = 0.0;
    int batchCount = 0;
    int validTempCount = 0;
    int validAuxCount = 0;

    // Write with frequent watchdog feeding and timeout checking
    for (int i = 0; i < readingsToWrite; i++) {
        // Check for timeout during write
        if (millis() - writeStartTime > MAX_WRITE_TIME) {
            Serial.print("ERROR: Write timeout after ");
            Serial.print(i);
            Serial.print("/");
            Serial.print(readingsToWrite);
            Serial.println(" readings");
            dataFile.close();
            writeInProgress = false;
            // Don't clear buffer - will retry next time
            return;
        }
        
        // Feed watchdog EVERY 5 writes (more frequent than before)
        if (i % 5 == 0) {
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
        dataFile.print(readings[i].pidValue, 2);
        // Columns added after the original six (see CSV_HEADER)
        dataFile.print(",");
        dataFile.print(readings[i].voltageAdc);
        dataFile.print(",");
        dataFile.print(readings[i].thermistorAdc);
        dataFile.print(",");
        dataFile.print(thermistorAdcToVoltage(readings[i].thermistorAdc), 4);
        dataFile.print(",");
        dataFile.print(readings[i].pidEnabled ? 1 : 0);
        dataFile.print(",");
        dataFile.print(pidKp, 3);
        dataFile.print(",");
        dataFile.print(pidKi, 3);
        dataFile.print(",");
        dataFile.print(pidKd, 3);
        // AUX Temp Sensor (channel 2), computed from the raw count
        float aux = convertAuxThermistorToTemperature(readings[i].auxThermistorAdc);
        dataFile.print(",");
        dataFile.print(aux, 3);
        dataFile.print(",");
        dataFile.print(readings[i].auxThermistorAdc);
        dataFile.print(",");
        dataFile.println(thermistorAdcToVoltage(readings[i].auxThermistorAdc), 4);

        if (aux > -50 && aux < 150) {
            if (aux < batchAuxMin) batchAuxMin = aux;
            if (aux > batchAuxMax) batchAuxMax = aux;
            batchAuxSum += aux;
            validAuxCount++;
        }

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
    
    // Close with timeout tracking
    unsigned long closeStart = millis();
    dataFile.close();
    unsigned long closeTime = millis() - closeStart;
    
    if (closeTime > 1000) {
        Serial.print("WARNING: File close took ");
        Serial.print(closeTime);
        Serial.println("ms (slow SD card?)");
    }
    
    feedWatchdog();
    Serial.println("[WATCHDOG] Fed after file write completion");
    
    // Mark write as complete
    writeInProgress = false;
    unsigned long totalWriteTime = millis() - writeStartTime;

    Serial.print("Wrote ");
    Serial.print(batchCount);
    Serial.print(" sensor readings to file in ");
    Serial.print(totalWriteTime);
    Serial.println("ms");
    Serial.print("  Voltage - Min="); Serial.print(batchVoltageMin, 4);
    Serial.print("V, Max="); Serial.print(batchVoltageMax, 4);
    Serial.print("V, Avg="); Serial.print(batchVoltageSum / batchCount, 4); Serial.println("V");

    if (validTempCount > 0) {
      Serial.print(F("  PID Temp Sensor - Min=")); Serial.print(batchTempMin, 2);
      Serial.print("°C, Max="); Serial.print(batchTempMax, 2);
      Serial.print("°C, Avg="); Serial.print(batchTempSum / validTempCount, 2); Serial.println("°C");
    } else {
      Serial.println(F("  PID Temp Sensor - No valid readings (thermistor not connected?)"));
    }
    if (validAuxCount > 0) {
      Serial.print(F("  AUX Temp Sensor - Min=")); Serial.print(batchAuxMin, 2);
      Serial.print("°C, Max="); Serial.print(batchAuxMax, 2);
      Serial.print("°C, Avg="); Serial.print(batchAuxSum / validAuxCount, 2); Serial.println("°C");
    } else {
      Serial.println(F("  AUX Temp Sensor - No valid readings (thermistor not connected?)"));
    }
    
    // Check if write took too long (warning threshold)
    if (totalWriteTime > 2000) {
        Serial.print("⚠️  WARNING: Slow SD write (");
        Serial.print(totalWriteTime);
        Serial.println("ms). Consider replacing SD card.");
    }

    // Reset buffer index only after successful write
    bufferIndex = 0;
    bufferFull = false; // Reset full flag
}

void initializeDataFile() {
  if (initializeSDCard()) {
    getNewLogFileName();
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      printCsvHeader(dataFile);
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
  Serial.println("Data cleared successfully - fresh CSV will contain only new sensor readings");
}

void flushDataBuffer() {
  if (bufferIndex > 0) {
    Serial.println("Flushing remaining buffer to file...");
    writeBufferToFile(); // Ensure buffer is flushed
  } else {
    Serial.println("Buffer empty, nothing to flush.");
  }
}

void emergencyFileCleanup() {
  // Force close any open file handle
  if (dataFile) {
    Serial.println("EMERGENCY: Forcing file close");
    dataFile.close();
  }
  
  // Reset write-in-progress flag
  if (writeInProgress) {
    Serial.println("EMERGENCY: Clearing write-in-progress flag");
    writeInProgress = false;
  }
  
  Serial.println("EMERGENCY: File cleanup complete");
}
