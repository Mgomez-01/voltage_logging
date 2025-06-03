#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>
#include <Ticker.h>

// Hardware-based Safety System
extern Ticker safetyTimer;           // Hardware timer for safety checks
extern Ticker watchdogTimer;         // Hardware watchdog timer
extern volatile bool systemAlive;     // Watchdog heartbeat flag
extern volatile unsigned long lastSafetyCheck;  // Last safety check timestamp
extern volatile bool emergencyShutdown;     // Emergency shutdown flag
extern const unsigned long SAFETY_CHECK_INTERVAL;  // Safety check every 500ms
extern const unsigned long WATCHDOG_TIMEOUT;     // 10 second watchdog timeout

// External references needed by safety system
extern bool dataLoggingEnabled;
extern bool relayState;
extern bool heaterEnabled;
extern bool pidEnabled;
extern unsigned long relayOnTime;
extern const unsigned long MAX_HEATER_TIME;
extern const int RELAY_PIN;

// Function declarations
void ICACHE_RAM_ATTR hardwareSafetyCheck();
void ICACHE_RAM_ATTR watchdogCheck();
void initializeSafetySystem();
void feedWatchdog();
void emergencyShutdownSystem();

#endif // SAFETY_SYSTEM_H
