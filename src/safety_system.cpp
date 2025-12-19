#include "safety_system.h"

// Hardware-based Safety System
Ticker safetyTimer;           // Hardware timer for safety checks
Ticker watchdogTimer;         // Hardware watchdog timer
volatile bool systemAlive = true;     // Watchdog heartbeat flag
volatile unsigned long lastSafetyCheck = 0;  // Last safety check timestamp
volatile bool emergencyShutdown = false;     // Emergency shutdown flag
const unsigned long SAFETY_CHECK_INTERVAL = 500;  // Safety check every 500ms
const unsigned long WATCHDOG_TIMEOUT = 10000;     // 10 second watchdog timeout

void IRAM_ATTR hardwareSafetyCheck() {
  lastSafetyCheck = millis();
  if (heaterDutyCycle > 0 && millis() - heaterStartTime > MAX_HEATER_TIME) {
    analogWrite(HEATER_PWM_PIN, 0);  // Turn off PWM
    heaterDutyCycle = 0.0;
    heaterEnabled = false;
    pidEnabled = false;
    emergencyShutdown = true;
  }
}

void IRAM_ATTR watchdogCheck() {
  if (!systemAlive && !emergencyShutdown) { // Only trigger if not already shut down
    Serial.println("!!! WATCHDOG STARVED - EMERGENCY SHUTDOWN !!!");
    analogWrite(HEATER_PWM_PIN, 0);  // Turn off PWM
    emergencyShutdown = true;
    // Don't rely on hardware WDT, manage via flag
  }
  systemAlive = false;
}

void initializeSafetySystem() {
  // We will manage our own watchdog, but keep ESP WDT as a final backup
  ESP.wdtEnable(WATCHDOG_TIMEOUT);
  safetyTimer.attach_ms(SAFETY_CHECK_INTERVAL, hardwareSafetyCheck);
  watchdogTimer.attach_ms(8000, watchdogCheck); // Check every 8 seconds (allows time for file operations)
  systemAlive = true;
  emergencyShutdown = false;
  lastSafetyCheck = millis();
}

void feedWatchdog() {
  systemAlive = true;
  ESP.wdtFeed();
}

void emergencyShutdownSystem() {
  static unsigned long lastShutdownMessage = 0;
  analogWrite(HEATER_PWM_PIN, 0);  // Turn off PWM
  heaterDutyCycle = 0.0;
  heaterEnabled = false;
  pidEnabled = false;
  dataLoggingEnabled = false; // Stop logging
  if (millis() - lastShutdownMessage > 5000) {
    Serial.println("*** EMERGENCY SHUTDOWN ACTIVE ***");
    Serial.println("*** HEATER DISABLED - SYSTEM SAFE ***");
    Serial.println("*** Use 'System Reset' button in web interface to recover ***");
    Serial.println("*** Or send GET request to /system/reset ***");
    lastShutdownMessage = millis();
  }
  server.handleClient(); // Keep server responsive for reset button
  webSocket.loop();      // Keep WebSocket responsive
  delay(10); // Prevent tight loop in shutdown
}
