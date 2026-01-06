# Memory Analysis and Fixes

## Current Memory Usage

### ESP8266 Constraints
- **Total RAM**: ~80KB
- **WiFi Stack**: ~20-25KB
- **Web Server**: ~10-15KB
- **WebSocket Server**: ~5-10KB
- **Available for app**: ~30-40KB

### Your Current Allocations

**1. Data Buffer** (BIGGEST CONSUMER)
```cpp
const int BUFFER_SIZE = 100;
SensorReading readings[BUFFER_SIZE];
```

Each SensorReading:
- timestamp: 4 bytes
- voltage: 4 bytes
- temperature: 4 bytes
- currentChannel: 4 bytes
- heaterState: 4 bytes (padded)
- pidValue: 4 bytes
- targetTemp: 4 bytes
**Total per reading**: ~28 bytes

**100 readings = 2,800 bytes!** 🚨

**2. WebSocket Updates** (MEMORY FRAGMENTATION)
```cpp
// Called every 100ms when logging!
void sendWebUpdate() {
  JsonDocument doc;  // Dynamic allocation on stack
  // ... create message
  String jsonString;  // Dynamic string
  webSocket.broadcastTXT(jsonString);  // Makes copy
}
```

At 500Hz sampling rate:
- Creates JSON doc 10 times/second
- Each creates temporary Strings
- Fragments heap memory over time

**3. String Operations** (FRAGMENTATION)
```cpp
// In sendTemplatedPage - happens on every page load
line.replace("{{WIFI_IP}}", WiFi.softAPIP().toString());
// Multiple String copies and reallocations
```

---

## Problems Identified

### 1. Buffer Too Large
- 100 readings at 500Hz = only 0.2 seconds of data
- Writing to SD every 0.2 seconds is excessive
- Each write takes time, causes jitter

### 2. WebSocket Spam
- 10 updates/second is way too frequent
- User can't see changes that fast
- Creates 10 JSON documents/second
- Each allocation fragments heap

### 3. No Memory Monitoring
- No checks for low memory
- No recovery when memory gets low
- System just becomes unresponsive

### 4. String Fragmentation
- Repeated String operations fragment heap
- Eventually can't allocate even small blocks
- System hangs or crashes

---

## Recommended Fixes

### Fix 1: Reduce Buffer Size ✅

**Change BUFFER_SIZE from 100 to 50**

**File**: `src/data_manager.cpp` and `src/data_manager.h`

```cpp
// OLD:
const int BUFFER_SIZE = 100;

// NEW:
const int BUFFER_SIZE = 50;  // Saves 1,400 bytes!
```

**Benefits**:
- Saves 1,400 bytes of RAM
- Still captures 0.1 seconds at 500Hz
- Still provides good data continuity
- Writes to SD twice as often (still totally fine)

**Could even go to 25 if needed** (saves 2,100 bytes total)

### Fix 2: Reduce WebSocket Update Rate ✅

**Change from 100ms to 500ms**

**File**: `src/main.cpp`

```cpp
// OLD:
const unsigned long WEB_UPDATE_INTERVAL = 100; // Update web every 100ms

// NEW:
const unsigned long WEB_UPDATE_INTERVAL = 500; // Update web every 500ms
```

**Benefits**:
- 10 updates/sec → 2 updates/sec (80% reduction!)
- Still plenty responsive for user
- Massive reduction in memory allocations
- Less heap fragmentation

### Fix 3: Add Memory Monitoring ✅

**Add heap monitoring to debug stats**

**File**: `src/main.cpp` in `printDebugStats()`

Already there:
```cpp
Serial.print("Free heap: "); 
Serial.print(ESP.getFreeHeap()); 
Serial.println(" bytes");
```

**Add warning threshold**:
```cpp
// At the end of printDebugStats()
if (ESP.getFreeHeap() < 10000) {
  Serial.println("⚠️ WARNING: Low memory! Free heap < 10KB");
  Serial.println("⚠️ Consider stopping data logging or reducing buffer");
}
```

### Fix 4: Reduce Debug Output When Running ✅

**Turn off excessive debug messages**

**File**: `src/main.cpp`

```cpp
// OLD:
#define DEBUG_ADC 1

// NEW:
#define DEBUG_ADC 0  // Turn off during normal operation
```

Same for:
```cpp
#define DEBUG_WEBSOCKET 0  // Reduce WebSocket spam
#define DEBUG_PID 0         // Reduce PID spam (or only show errors)
```

**Keep these on**:
```cpp
#define DEBUG_SERIAL 1    // General status - keep
#define DEBUG_HEATER 1    // Safety-critical - keep
```

### Fix 5: Skip Redundant WebSocket Updates ✅

**Only send if data actually changed**

**File**: `src/web_interface.cpp`

```cpp
void sendWebUpdate() {
  connectedClients = webSocket.connectedClients();
  if (connectedClients > 0) {
    if (bufferIndex > 0 || bufferFull) {
      int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
      
      // Skip if no significant change
      static float lastSentTemp = -999;
      static float lastSentVoltage = -999;
      float currentTemp = readings[idx].temperature;
      float currentVoltage = readings[idx].voltage;
      
      // Only send if changed by > 0.1°C or 0.01V
      if (abs(currentTemp - lastSentTemp) < 0.1 && 
          abs(currentVoltage - lastSentVoltage) < 0.01) {
        return;  // Skip this update
      }
      
      lastSentTemp = currentTemp;
      lastSentVoltage = currentVoltage;
      
      // Original code continues...
      JsonDocument doc;
      // ...
    }
  }
}
```

### Fix 6: Use Static JsonDocument ✅

**Pre-allocate JSON buffer to avoid fragmentation**

**File**: `src/web_interface.cpp`

```cpp
void sendWebUpdate() {
  connectedClients = webSocket.connectedClients();
  if (connectedClients > 0) {
    if (bufferIndex > 0 || bufferFull) {
      int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
      
      // Use static document to avoid heap fragmentation
      static StaticJsonDocument<256> doc;
      doc.clear();  // Clear previous contents
      
      doc["timestamp"] = readings[idx].timestamp;
      doc["voltage"] = readings[idx].voltage;
      doc["temperature"] = readings[idx].temperature;
      doc["type"] = "reading";
      doc["heaterState"] = readings[idx].heaterState;
      doc["targetTemp"] = readings[idx].targetTemp;
      doc["pidOutput"] = readings[idx].pidValue;
      doc["loggingEnabled"] = dataLoggingEnabled;
      
      char jsonBuffer[256];
      serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
      webSocket.broadcastTXT(jsonBuffer);
      
      totalWebSocketMessages++;
    }
  }
}
```

---

## Memory-Optimized Configuration

### Recommended Settings

**For Normal Operation**:
```cpp
// data_manager.cpp
const int BUFFER_SIZE = 50;  // Was 100

// main.cpp
const unsigned long WEB_UPDATE_INTERVAL = 500;  // Was 100
#define DEBUG_ADC 0           // Was 1
#define DEBUG_WEBSOCKET 0     // Was 1
#define DEBUG_PID 0           // Was 1 (or keep at 1 if tuning)
```

**For Heavy Logging**:
```cpp
const int BUFFER_SIZE = 25;  // Even smaller
const unsigned long WEB_UPDATE_INTERVAL = 1000;  // 1 second updates
```

**For Testing/Development**:
```cpp
const int BUFFER_SIZE = 50;
const unsigned long WEB_UPDATE_INTERVAL = 500;
// Enable all debug as needed
```

---

## Expected Memory Improvements

| Change | Memory Saved | Impact |
|--------|-------------|--------|
| Buffer 100→50 | 1,400 bytes | -50% buffer RAM |
| WebSocket 100ms→500ms | ~500 bytes/sec | -80% allocations |
| Static JsonDocument | ~200 bytes | No fragmentation |
| Reduce debug output | ~100 bytes | Less Serial buffer |
| Skip redundant updates | Varies | Less processing |
| **TOTAL SAVINGS** | **~2,200 bytes** | **Much more stable** |

---

## How to Test Memory Issues

### Before Making Changes

Monitor free heap in serial output:
```
Free heap: 31712 bytes  ← Good, plenty of RAM
Free heap: 28450 bytes  ← Still OK
Free heap: 15234 bytes  ← Getting low
Free heap: 8102 bytes   ← DANGER! System will hang soon
Free heap: 4567 bytes   ← CRITICAL! About to crash
```

### Symptoms of Memory Issues

1. **System becomes unresponsive**
   - Web interface stops responding
   - WebSocket disconnects
   - Serial output freezes

2. **Heap fragmentation**
   - Free heap shows available memory
   - But can't allocate even small blocks
   - Functions fail randomly

3. **Watchdog resets**
   - System appears to reset
   - Loses connection
   - "Watchdog starved" messages

### Testing Procedure

1. Start with logging OFF
2. Note free heap (should be 30-35KB)
3. Start logging
4. Watch heap decrease over time
5. If drops below 15KB → memory leak or too much buffering
6. If drops below 10KB → imminent crash

---

## Quick Fixes You Can Do Right Now

### Priority 1: Reduce Buffer (CRITICAL)
```cpp
// In src/data_manager.cpp line 12:
const int BUFFER_SIZE = 50;  // Change from 100
```

### Priority 2: Reduce WebSocket Rate (IMPORTANT)
```cpp
// In src/main.cpp line 43:
const unsigned long WEB_UPDATE_INTERVAL = 500;  // Change from 100
```

### Priority 3: Disable Verbose Debug (HELPFUL)
```cpp
// In src/main.cpp lines 22-27:
#define DEBUG_SERIAL 1       // Keep
#define DEBUG_ADC 0          // Turn off (was 1)
#define DEBUG_WEBSOCKET 0    // Turn off (was 1)
#define DEBUG_WIFI 1         // Keep
#define DEBUG_HEATER 1       // Keep (safety)
#define DEBUG_PID 0          // Turn off unless tuning
```

Just these 3 changes will dramatically improve stability!

---

## Long-Term Memory Management

### Add Automatic Protection

```cpp
// In main.cpp loop(), add memory check:
void loop() {
  feedWatchdog();
  
  // Memory protection
  if (ESP.getFreeHeap() < 8000 && dataLoggingEnabled) {
    Serial.println("MEMORY: Auto-stopping logging due to low heap");
    dataLoggingEnabled = false;
    flushDataBuffer();
  }
  
  // ... rest of loop
}
```

### Add Memory Stats to Web Interface

Show free heap on the status page so users can monitor it.

---

## Summary: What's Causing Your Issue

**Root Causes**:
1. ⚠️ **100-entry buffer too large** (2,800 bytes)
2. ⚠️ **WebSocket spam** (10 updates/second)
3. ⚠️ **Heap fragmentation** from repeated allocations
4. ⚠️ **No memory monitoring** to catch issues early

**Quick Wins**:
1. ✅ Reduce BUFFER_SIZE to 50 (saves 1,400 bytes)
2. ✅ Reduce WEB_UPDATE_INTERVAL to 500ms (80% less spam)
3. ✅ Turn off DEBUG_ADC and DEBUG_WEBSOCKET

**After fixes, you should have**:
- Free heap stays above 20KB during logging
- System runs for hours without issues
- WebSocket stays connected
- No random hangs or resets

Would you like me to create the actual code changes for you to apply?
