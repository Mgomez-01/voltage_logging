#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>
#include <Ticker.h>
#include "heater_controller.h"
#include "web_interface.h"

// Hardware-based Safety System
extern Ticker safetyTimer;           // Hardware timer for safety checks
extern Ticker watchdogTimer;         // Hardware watchdog timer
extern volatile bool systemAlive;     // Watchdog heartbeat flag
extern volatile unsigned long lastSafetyCheck;  // Last safety check timestamp
extern volatile bool emergencyShutdown;     // Emergency shutdown flag

// Why the emergency shutdown happened (first cause wins; cleared by /system/reset).
// The status LED flashes orange for SHUTDOWN_TIMEOUT and yellow for the rest.
enum ShutdownReason : uint8_t {
  SHUTDOWN_NONE = 0,
  SHUTDOWN_TIMEOUT,        // Heater on longer than MAX_HEATER_TIME
  SHUTDOWN_OVER_TEMP,      // PID Temp Sensor above MAX_SAFE_TEMPERATURE
  SHUTDOWN_SENSOR,         // PID Temp Sensor fault while the heater was on
  SHUTDOWN_WATCHDOG,       // Main loop stopped feeding the watchdog
  SHUTDOWN_SAFETY_TIMER    // Safety timer stopped running
};
extern volatile uint8_t shutdownReason;
extern const unsigned long SAFETY_CHECK_INTERVAL;  // Safety check every 500ms
extern const unsigned long WATCHDOG_TIMEOUT;     // 10 second watchdog timeout

// Function declarations
void IRAM_ATTR hardwareSafetyCheck();
void IRAM_ATTR watchdogCheck();
void initializeSafetySystem();
void feedWatchdog();
void emergencyShutdownSystem();
void triggerEmergencyShutdown(ShutdownReason reason);  // Sets the flag (and the reason, if first)
const char* shutdownReasonText();                      // Reason as text ("" if none)

#endif // SAFETY_SYSTEM_H
