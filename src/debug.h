#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// ============================================================================
// MASTER DEBUG CONTROL - Set to 0 for RELEASE builds
// ============================================================================
#define DEBUG_MASTER 1  // Set to 0 to disable ALL debug output

// ============================================================================
// INDIVIDUAL DEBUG CATEGORIES
// ============================================================================
// Only active if DEBUG_MASTER is enabled
#if DEBUG_MASTER

  // General system debug
  #define DEBUG_SERIAL 1        // General status messages, uptime, stats
  #define DEBUG_STARTUP 1       // Initialization messages
  
  // Component-specific debug
  #define DEBUG_ADC 0           // Sensor ADC readings (VERY verbose)
  #define DEBUG_WEBSOCKET 0     // WebSocket traffic (verbose)
  #define DEBUG_WIFI 1          // WiFi connection status
  #define DEBUG_HEATER 1        // Heater control operations
  #define DEBUG_PID 0           // PID controller details (verbose)
  #define DEBUG_SAFETY 1        // Safety system checks
  #define DEBUG_FILE 1          // File operations and timing
  #define DEBUG_WATCHDOG 1      // Watchdog feed messages
  #define DEBUG_MEMORY 1        // Memory warnings and stats
  #define DEBUG_BUFFER 1        // Buffer operations
  
#else
  // All debug disabled for RELEASE builds
  #define DEBUG_SERIAL 0
  #define DEBUG_STARTUP 0
  #define DEBUG_ADC 0
  #define DEBUG_WEBSOCKET 0
  #define DEBUG_WIFI 0
  #define DEBUG_HEATER 0
  #define DEBUG_PID 0
  #define DEBUG_SAFETY 0
  #define DEBUG_FILE 0
  #define DEBUG_WATCHDOG 0
  #define DEBUG_MEMORY 0
  #define DEBUG_BUFFER 0
#endif

// ============================================================================
// CRITICAL MESSAGES - ALWAYS ENABLED (even in release)
// ============================================================================
// These are always printed regardless of debug settings
// Use for safety-critical messages, errors, and warnings

#define CRITICAL(x) Serial.print(x)
#define CRITICALLN(x) Serial.println(x)

// ============================================================================
// DEBUG MACROS - Conditional compilation based on flags
// ============================================================================

// General serial debug
#if DEBUG_SERIAL
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Startup debug
#if DEBUG_STARTUP
  #define STARTUP_PRINT(x) Serial.print(x)
  #define STARTUP_PRINTLN(x) Serial.println(x)
#else
  #define STARTUP_PRINT(x)
  #define STARTUP_PRINTLN(x)
#endif

// ADC debug
#if DEBUG_ADC
  #define ADC_PRINT(x) Serial.print(x)
  #define ADC_PRINTLN(x) Serial.println(x)
#else
  #define ADC_PRINT(x)
  #define ADC_PRINTLN(x)
#endif

// WebSocket debug
#if DEBUG_WEBSOCKET
  #define WS_PRINT(x) Serial.print(x)
  #define WS_PRINTLN(x) Serial.println(x)
#else
  #define WS_PRINT(x)
  #define WS_PRINTLN(x)
#endif

// WiFi debug
#if DEBUG_WIFI
  #define WIFI_PRINT(x) Serial.print(x)
  #define WIFI_PRINTLN(x) Serial.println(x)
#else
  #define WIFI_PRINT(x)
  #define WIFI_PRINTLN(x)
#endif

// Heater debug
#if DEBUG_HEATER
  #define HEATER_PRINT(x) Serial.print(x)
  #define HEATER_PRINTLN(x) Serial.println(x)
#else
  #define HEATER_PRINT(x)
  #define HEATER_PRINTLN(x)
#endif

// PID debug
#if DEBUG_PID
  #define PID_PRINT(x) Serial.print(x)
  #define PID_PRINTLN(x) Serial.println(x)
#else
  #define PID_PRINT(x)
  #define PID_PRINTLN(x)
#endif

// Safety debug
#if DEBUG_SAFETY
  #define SAFETY_PRINT(x) Serial.print(x)
  #define SAFETY_PRINTLN(x) Serial.println(x)
#else
  #define SAFETY_PRINT(x)
  #define SAFETY_PRINTLN(x)
#endif

// File operations debug
#if DEBUG_FILE
  #define FILE_PRINT(x) Serial.print(x)
  #define FILE_PRINTLN(x) Serial.println(x)
#else
  #define FILE_PRINT(x)
  #define FILE_PRINTLN(x)
#endif

// Watchdog debug
#if DEBUG_WATCHDOG
  #define WATCHDOG_PRINT(x) Serial.print(x)
  #define WATCHDOG_PRINTLN(x) Serial.println(x)
#else
  #define WATCHDOG_PRINT(x)
  #define WATCHDOG_PRINTLN(x)
#endif

// Memory debug
#if DEBUG_MEMORY
  #define MEM_PRINT(x) Serial.print(x)
  #define MEM_PRINTLN(x) Serial.println(x)
#else
  #define MEM_PRINT(x)
  #define MEM_PRINTLN(x)
#endif

// Buffer debug
#if DEBUG_BUFFER
  #define BUFFER_PRINT(x) Serial.print(x)
  #define BUFFER_PRINTLN(x) Serial.println(x)
#else
  #define BUFFER_PRINT(x)
  #define BUFFER_PRINTLN(x)
#endif

// ============================================================================
// HELPER MACROS
// ============================================================================

// Print with condition (for inline conditionals)
#if DEBUG_MASTER
  #define DEBUG_IF(condition, x) if (condition) Serial.print(x)
  #define DEBUG_IFLN(condition, x) if (condition) Serial.println(x)
#else
  #define DEBUG_IF(condition, x)
  #define DEBUG_IFLN(condition, x)
#endif

// Print build configuration at startup
inline void printDebugConfig() {
#if DEBUG_MASTER
  Serial.println("=== DEBUG BUILD ===");
  Serial.print("  DEBUG_SERIAL: "); Serial.println(DEBUG_SERIAL);
  Serial.print("  DEBUG_ADC: "); Serial.println(DEBUG_ADC);
  Serial.print("  DEBUG_WEBSOCKET: "); Serial.println(DEBUG_WEBSOCKET);
  Serial.print("  DEBUG_WIFI: "); Serial.println(DEBUG_WIFI);
  Serial.print("  DEBUG_HEATER: "); Serial.println(DEBUG_HEATER);
  Serial.print("  DEBUG_PID: "); Serial.println(DEBUG_PID);
  Serial.print("  DEBUG_SAFETY: "); Serial.println(DEBUG_SAFETY);
  Serial.print("  DEBUG_FILE: "); Serial.println(DEBUG_FILE);
  Serial.print("  DEBUG_WATCHDOG: "); Serial.println(DEBUG_WATCHDOG);
  Serial.print("  DEBUG_MEMORY: "); Serial.println(DEBUG_MEMORY);
  Serial.println("===================");
#else
  Serial.println("=== RELEASE BUILD ===");
  Serial.println("Debug output disabled");
  Serial.println("=====================");
#endif
}

#endif // DEBUG_H
