# Safety Parameter Configuration Guide

## Where to Change Safety Settings

**File**: `src/heater_controller.cpp`  
**Lines**: 10-11

```cpp
const unsigned long MAX_HEATER_TIME = 600000; // 10 min safety timeout
const float MAX_SAFE_TEMPERATURE = 120.0; // Maximum safe temperature in °C
```

---

## Safety Parameters Explained

### 1. MAX_HEATER_TIME (Maximum Runtime)

**Purpose**: Prevents heater from running continuously for too long, protecting against:
- Stuck-on conditions
- Runaway heating
- PID tuning errors
- Software bugs

**Current Value**: `600000` milliseconds = **10 minutes**

**How to Calculate**:
```cpp
// Format: milliseconds
1 minute  = 60000
5 minutes = 300000
10 minutes = 600000
15 minutes = 900000
20 minutes = 1200000
30 minutes = 1800000
1 hour = 3600000
```

**Recommended Values**:
| Application | Recommended Time | Value (ms) | Rationale |
|-------------|------------------|------------|-----------|
| Testing/Development | 5 minutes | `300000` | Quick safety for initial tests |
| Small thermal mass | 10 minutes | `600000` | Current setting - good default |
| Medium thermal mass | 20 minutes | `1200000` | Allow time to heat up |
| Large thermal mass | 30 minutes | `1800000` | Heavy objects take longer |
| Continuous operation | 1 hour | `3600000` | Well-tuned PID, monitored |

**⚠️ Safety Notes**:
- **Never disable** (don't set to 0 or extremely high values)
- Consider thermal mass of what you're heating
- Shorter is safer, longer allows reaching temperature
- If timeout triggers often, PID tuning or target temp may be wrong

**Example Changes**:
```cpp
// For quick testing (5 minutes)
const unsigned long MAX_HEATER_TIME = 300000;

// For larger projects (20 minutes)
const unsigned long MAX_HEATER_TIME = 1200000;

// For continuous well-monitored operation (1 hour)
const unsigned long MAX_HEATER_TIME = 3600000;
```

---

### 2. MAX_SAFE_TEMPERATURE (Over-Temperature Shutdown)

**Purpose**: Emergency shutdown if temperature exceeds safe limit, protecting against:
- Sensor failures
- PID malfunction
- Target temperature set too high
- Thermal runaway
- Fire hazard

**Current Value**: `120.0°C`

**Recommended Values**:
| Material/Application | Max Safe Temp | Value | Rationale |
|---------------------|---------------|-------|-----------|
| Water heating | 90°C | `90.0` | Below boiling, safe for containers |
| General lab use | 100°C | `100.0` | Water boiling point |
| PCB reflow (low) | 120°C | `120.0` | Current setting |
| Plastic work | 150°C | `150.0` | Below most plastic melting points |
| High temp applications | 200°C | `200.0` | **Maximum sensor range** |

**⚠️ CRITICAL Safety Notes**:
- **Never exceed 200°C** - that's the sensor's maximum rating
- Consider what you're heating:
  - **PCBs**: Max 150-180°C (component damage)
  - **Plastics**: Varies 80-200°C (melting point)
  - **Water**: Max 100°C (boiling)
  - **Enclosure materials**: Check their rating
- Add margin of safety (10-20°C below actual danger point)
- This is a **last-resort** protection, not an operating limit

**Example Changes**:
```cpp
// For water heating (safe margin below boiling)
const float MAX_SAFE_TEMPERATURE = 90.0;

// For PCB/electronics work
const float MAX_SAFE_TEMPERATURE = 150.0;

// For high temperature applications (use cautiously)
const float MAX_SAFE_TEMPERATURE = 200.0;
```

---

## Other Configurable Safety Parameters

### 3. Watchdog Check Interval
**File**: `src/safety_system.cpp`  
**Line**: ~37

```cpp
watchdogTimer.attach_ms(8000, watchdogCheck); // Check every 8 seconds
```

**Change to**:
```cpp
watchdogTimer.attach_ms(5000, watchdogCheck);  // More frequent (5 seconds)
watchdogTimer.attach_ms(10000, watchdogCheck); // Less frequent (10 seconds)
```

**Recommendation**: Keep at 8000ms (8 seconds) - allows for file write operations without false triggers

---

### 4. Safety Check Interval
**File**: `src/safety_system.cpp`  
**Line**: ~7

```cpp
const unsigned long SAFETY_CHECK_INTERVAL = 500;  // Safety check every 500ms
```

**Recommendation**: Keep at 500ms (0.5 seconds) - good balance between responsiveness and overhead

---

### 5. PID Update Interval
**File**: `src/heater_controller.cpp`  
**Line**: ~28

```cpp
const unsigned long PID_INTERVAL = 500; // PID update interval in ms
```

**Options**:
```cpp
const unsigned long PID_INTERVAL = 250;  // Faster response (4 Hz)
const unsigned long PID_INTERVAL = 500;  // Default (2 Hz)
const unsigned long PID_INTERVAL = 1000; // Slower, more stable (1 Hz)
```

**Recommendation**: 500ms works well for most heater applications

---

### 6. Default Target Temperature
**File**: `src/heater_controller.cpp`  
**Line**: ~20

```cpp
float targetTemperature = 65.0; // Default target temperature
```

**Change to your typical operating temperature**:
```cpp
float targetTemperature = 45.0;  // Cooler default
float targetTemperature = 80.0;  // Hotter default
```

---

## Complete Safety System Hierarchy

```
Layer 1: Software Limits (First Line)
├── MAX_SAFE_TEMPERATURE (120°C) - Temperature too high
├── Sensor validation (-50 to 200°C) - Bad readings
└── Target temperature limit (0 to MAX_SAFE_TEMPERATURE)

Layer 2: Runtime Protection (Time-Based)
├── MAX_HEATER_TIME (10 min) - Continuous operation limit
└── Safety check interval (500ms) - Regular monitoring

Layer 3: Hardware Watchdog (Independent Timer)
├── Watchdog timer (8 sec) - Software must feed regularly
└── Auto-shutdown if not fed - Protects against hangs

Layer 4: Emergency Shutdown (Last Resort)
└── Immediately disables all heater control
```

---

## Safety Testing Checklist

After changing safety parameters, test them:

### Test 1: Over-Temperature Shutdown
1. Set MAX_SAFE_TEMPERATURE to something low (e.g., 40°C)
2. Heat past that temperature
3. Verify emergency shutdown triggers
4. Reset and restore proper value

### Test 2: Maximum Runtime
1. Set MAX_HEATER_TIME to something short (e.g., 60000 = 1 minute)
2. Let heater run continuously
3. Verify shutdown after 1 minute
4. Reset and restore proper value

### Test 3: Watchdog
1. Add a delay(10000) in the main loop temporarily
2. Verify watchdog triggers emergency shutdown
3. Remove the delay

### Test 4: Sensor Failure
1. Disconnect thermistor temporarily
2. Enable heater
3. Verify sensor failure detection
4. Reconnect thermistor

---

## Quick Reference Table

| Parameter | Location | Current | Typical Range | Purpose |
|-----------|----------|---------|---------------|---------|
| MAX_HEATER_TIME | heater_controller.cpp:10 | 600000ms (10m) | 300000-3600000ms (5m-1h) | Runtime limit |
| MAX_SAFE_TEMPERATURE | heater_controller.cpp:11 | 120.0°C | 80-200°C | Over-temp shutdown |
| PID_INTERVAL | heater_controller.cpp:28 | 500ms | 250-1000ms | Control speed |
| targetTemperature | heater_controller.cpp:20 | 65.0°C | 20-150°C | Default setpoint |
| SAFETY_CHECK_INTERVAL | safety_system.cpp:7 | 500ms | 250-1000ms | Monitor frequency |
| Watchdog interval | safety_system.cpp:37 | 8000ms | 5000-10000ms | Hang detection |

---

## Example: Conservative Settings (Maximum Safety)

```cpp
// heater_controller.cpp
const unsigned long MAX_HEATER_TIME = 300000;     // 5 minutes
const float MAX_SAFE_TEMPERATURE = 80.0;          // 80°C
const unsigned long PID_INTERVAL = 1000;          // 1 second (slower)
float targetTemperature = 50.0;                   // 50°C default

// safety_system.cpp  
const unsigned long SAFETY_CHECK_INTERVAL = 250;  // 4 times per second
watchdogTimer.attach_ms(5000, watchdogCheck);     // 5 second watchdog
```

**Use for**: Initial testing, high-risk materials, unattended operation

---

## Example: Performance Settings (Well-Tested System)

```cpp
// heater_controller.cpp
const unsigned long MAX_HEATER_TIME = 1800000;    // 30 minutes
const float MAX_SAFE_TEMPERATURE = 150.0;         // 150°C
const unsigned long PID_INTERVAL = 500;           // 0.5 seconds (default)
float targetTemperature = 80.0;                   // 80°C default

// safety_system.cpp
const unsigned long SAFETY_CHECK_INTERVAL = 500;  // 2 times per second (default)
watchdogTimer.attach_ms(8000, watchdogCheck);     // 8 second watchdog (default)
```

**Use for**: Proven setup, attended operation, higher temperature needs

---

## ⚠️ Safety Philosophy

**Always err on the side of caution:**
- Start with conservative values
- Test safety systems work as expected
- Gradually relax limits only after testing
- Never disable safety features
- Monitor first few operations closely
- Keep fire extinguisher nearby for high-temp work

**Remember**: These limits are your last line of defense. Good PID tuning means you should never hit them during normal operation!
