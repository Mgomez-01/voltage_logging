# Voltage Logger Testing Specification

## Document Version: 1.0
**Last Updated:** 2025-01-02  
**Target Hardware:** NodeMCU ESP-12E (Hitlego board)  
**Project:** ESP8266 Dual Sensor Logger with Temperature Control

---

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [Testing Priorities](#testing-priorities)
3. [Hardware Configuration](#hardware-configuration)
4. [Testing Phases](#testing-phases)
5. [Unit Tests](#unit-tests)
6. [Integration Tests](#integration-tests)
7. [End-to-End Tests](#end-to-end-tests)
8. [Test Environment Setup](#test-environment-setup)
9. [Success Criteria](#success-criteria)
10. [Failure Scenarios](#failure-scenarios)

---

## Executive Summary

### Primary Goals
1. **Ensure system stability** - Device must run indefinitely without memory exhaustion
2. **Validate safety systems** - Emergency shutdown and watchdog must be bulletproof
3. **Prevent data corruption** - File system and SD card operations must be robust
4. **Guarantee thermal safety** - Heater control must fail-safe under all conditions

### Testing Approach
- **Primary:** Mock/simulation testing for rapid development
- **Secondary:** Hardware validation on actual board with sensors
- **Focus:** Unit tests first, then integration, finally end-to-end

---

## Testing Priorities

### Priority Level 1: CRITICAL (Must Pass)
1. **Memory Management**
   - No heap exhaustion after 24+ hours continuous operation
   - Memory leak detection in all subsystems
   - Buffer overflow prevention
   - Heap fragmentation monitoring

2. **Safety Systems**
   - Hardware watchdog functionality
   - Emergency shutdown triggers
   - Temperature runaway prevention
   - Heater timeout enforcement
   - PID controller stability under edge cases

3. **File System Robustness**
   - SD card write failures don't crash system
   - File rotation works reliably
   - Buffer flush on low memory
   - Recovery from corrupted files

### Priority Level 2: HIGH (Should Pass)
4. **Sensor Reading Accuracy**
   - Multiplexer channel switching reliability
   - ADC reading consistency
   - Temperature calculation accuracy
   - Voltage measurement precision

5. **Data Logging Integrity**
   - No lost samples during normal operation
   - Correct timestamp ordering
   - Buffer management under load
   - File write verification

### Priority Level 3: MEDIUM (Nice to Have)
6. **Web Interface & WebSocket**
   - Client disconnect/reconnect handling
   - Multiple simultaneous clients
   - Message queue management
   - JSON serialization/deserialization

7. **WiFi Connectivity**
   - AP mode stability
   - Client connection handling
   - Network recovery after interference

---

## Hardware Configuration

### NodeMCU ESP-12E Pinout (Hitlego Board)
```
Target Board: NodeMCU ESP-12E Module
Chip: ESP8266MOD
Flash: 4MB
RAM: ~80KB heap available

Pin Assignments (from your configuration):
├─ GPIO16 (D0) → Heater PWM (MOSFET gate)
├─ GPIO5  (D1) → MUX_S0
├─ GPIO4  (D2) → MUX_S1
├─ GPIO0  (D3) → MUX_S2
├─ GPIO2  (D4) → MUX_S3
├─ A0 (ADC0)   → Multiplexer COM pin
└─ GPIO15 (D8) → SD Card CS (if used)

CD74HC4067 Multiplexer:
├─ Channel 0 → Voltage sensor (0-1V)
├─ Channel 1 → 100kΩ NTC Thermistor
└─ COM → ESP8266 A0 (ADC)

Heater Circuit:
└─ GPIO16 → AOD4144 MOSFET → Heating element
```

### Test Configurations

#### Configuration A: Full Hardware
- All sensors connected
- SD card mounted
- MOSFET and heater connected
- Real-time operation

#### Configuration B: Mock Hardware (Development)
- Simulated sensor readings
- Virtual file system
- Mock heater control
- Terminal-based testing

---

## Testing Phases

### Phase 1: Unit Tests (Week 1)
**Goal:** Validate individual components in isolation

**Test Duration per component:** 5-15 minutes  
**Total Estimated Time:** 2-3 days  
**Environment:** Mock/Terminal

### Phase 2: Integration Tests (Week 2)
**Goal:** Validate component interactions

**Test Duration per scenario:** 30-60 minutes  
**Total Estimated Time:** 3-5 days  
**Environment:** Mock + Limited Hardware

### Phase 3: End-to-End Tests (Week 3+)
**Goal:** Validate complete system behavior

**Test Duration per scenario:** 2-24 hours  
**Total Estimated Time:** 1-2 weeks  
**Environment:** Full Hardware Setup

---

## Unit Tests

### UT-001: Memory Management - Heap Monitoring
**Priority:** CRITICAL  
**Duration:** 15 minutes continuous  
**Environment:** Mock

**Test Steps:**
1. Initialize system with minimal heap usage baseline
2. Run all subsystems for 15 minutes
3. Sample heap every 1 second
4. Check for downward trends or fragmentation

**Success Criteria:**
- Free heap remains > 10KB at all times
- No monotonic decrease in available memory
- Heap variance < 5KB over entire test
- No memory allocation failures

**Failure Modes to Test:**
- WebSocket message accumulation
- Data buffer growth
- String concatenation leaks
- JSON document memory

**Measurement Points:**
```cpp
// Collect at 1Hz:
- ESP.getFreeHeap()
- ESP.getHeapFragmentation()
- bufferIndex value
- totalReadings count
```

---

### UT-002: Memory Management - Buffer Overflow Protection
**Priority:** CRITICAL  
**Duration:** 5 minutes  
**Environment:** Mock

**Test Steps:**
1. Fill sensor reading buffer to capacity
2. Continue adding readings beyond BUFFER_SIZE
3. Verify buffer wraps or flushes correctly
4. Check no out-of-bounds writes occur

**Success Criteria:**
- bufferIndex never exceeds BUFFER_SIZE
- No heap corruption detected
- Automatic flush triggers before overflow
- System remains responsive

**Input Conditions:**
- Rapid sensor readings (> 1000Hz)
- Disabled file writes (to force buffer fill)
- Memory pressure simulation

---

### UT-003: Memory Management - String Memory Leaks
**Priority:** CRITICAL  
**Duration:** 10 minutes  
**Environment:** Mock

**Test Steps:**
1. Exercise all String operations in codebase:
   - JSON serialization (web_interface.cpp)
   - File path construction (data_manager.cpp)
   - Debug output (main.cpp)
2. Monitor heap before/after each operation
3. Repeat operations 1000+ times

**Success Criteria:**
- No memory leaks from String operations
- Heap returns to baseline after operations
- No accumulation over iterations

**Code Locations to Test:**
```cpp
// web_interface.cpp: sendWebUpdate()
String message = "{...}";  // JSON construction

// data_manager.cpp: getNewLogFileName()
sprintf(logFileName, ...);  // String formatting

// main.cpp: printDebugStats()
Serial.print("Free heap: " + String(ESP.getFreeHeap()));
```

---

### UT-004: Safety Systems - Hardware Watchdog
**Priority:** CRITICAL  
**Duration:** 30 seconds  
**Environment:** Mock

**Test Steps:**
1. Initialize watchdog timer
2. Simulate main loop freeze (delay blocking)
3. Verify watchdog triggers emergency shutdown
4. Test watchdog feed mechanism

**Success Criteria:**
- Watchdog triggers within WATCHDOG_TIMEOUT (10s)
- Emergency shutdown flag sets to true
- Heater immediately disabled
- System enters safe state

**Test Cases:**
```cpp
TC-004.1: Normal operation - watchdog fed regularly
TC-004.2: Missed feed - single timeout
TC-004.3: Complete freeze - no feeds at all
TC-004.4: Intermittent feeds - sporadic feeding
```

---

### UT-005: Safety Systems - Temperature Limit Enforcement
**Priority:** CRITICAL  
**Duration:** 2 minutes  
**Environment:** Mock

**Test Steps:**
1. Set target temperature below MAX_SAFE_TEMPERATURE
2. Simulate temperature rising above limit
3. Verify emergency shutdown triggers
4. Test hysteresis behavior

**Success Criteria:**
- Heater disabled when T > MAX_SAFE_TEMPERATURE
- Emergency shutdown triggered
- System cannot be re-enabled until temperature drops
- No overshoot beyond safety margin

**Temperature Scenarios:**
```
Current: 20°C  → Target: 80°C  → Safety: 100°C
Current: 95°C  → Target: 80°C  → Safety: 100°C (approaching)
Current: 101°C → Target: 80°C  → Safety: 100°C (exceeded)
Current: 150°C → Target: 80°C  → Safety: 100°C (sensor error)
```

---

### UT-006: Safety Systems - Heater Timeout
**Priority:** CRITICAL  
**Duration:** 12 minutes  
**Environment:** Mock

**Test Steps:**
1. Enable heater with valid target temperature
2. Run for MAX_HEATER_TIME duration (10 minutes)
3. Verify automatic shutdown at timeout
4. Test cannot re-enable without manual reset

**Success Criteria:**
- Heater runs for exactly MAX_HEATER_TIME
- Automatic shutdown at timeout
- heaterEnabled flag = false
- Warning logged to Serial
- Emergency shutdown flag may be set

**Edge Cases:**
```
- Timeout during active PID control
- Timeout while temperature rising
- Timeout while temperature stable
- Multiple enable/disable cycles
```

---

### UT-007: Safety Systems - PID Controller Stability
**Priority:** CRITICAL  
**Duration:** 10 minutes  
**Environment:** Mock

**Test Steps:**
1. Initialize PID with standard gains (Kp=2.0, Ki=0.5, Kd=1.0)
2. Test response to step inputs
3. Test response to ramp inputs
4. Verify no integral windup
5. Check derivative kick prevention

**Success Criteria:**
- PID output bounded to 0-100%
- No integral windup (pidIntegral clamped)
- Stable at setpoint (±1°C)
- No oscillations or overshoot > 10°C
- Derivative term handles noise

**PID Test Scenarios:**
```cpp
TC-007.1: Step response: 20°C → 60°C
TC-007.2: Steady state: Hold 60°C for 5 minutes
TC-007.3: Disturbance rejection: Drop 10°C suddenly
TC-007.4: Setpoint tracking: Ramp 20°C → 80°C over 2 minutes
TC-007.5: Noise immunity: Add ±2°C sensor noise
```

---

### UT-008: File System - Write Failure Recovery
**Priority:** HIGH  
**Duration:** 3 minutes  
**Environment:** Mock

**Test Steps:**
1. Fill SD card to near capacity
2. Attempt buffer writes
3. Simulate write failures
4. Verify graceful degradation

**Success Criteria:**
- System continues running on write failure
- Error logged but doesn't crash
- Buffer can be flushed when space available
- Data logging can be disabled/re-enabled

**Failure Scenarios:**
```
- SD card full
- SD card removed during write
- Corrupted FAT table
- Write timeout
```

---

### UT-009: File System - File Rotation
**Priority:** HIGH  
**Duration:** 5 minutes  
**Environment:** Mock

**Test Steps:**
1. Create log file approaching MAX_FILE_SIZE
2. Continue writing data
3. Verify new file created automatically
4. Check filename increment logic

**Success Criteria:**
- New file created at threshold
- Filename format: `sensor_log_NNN.csv`
- Old file remains intact
- No data loss during rotation
- Metadata updated correctly

**File Naming Tests:**
```
sensor_log_000.csv → sensor_log_001.csv → ...
Verify: No overwrites, sequential numbering, max 999 files
```

---

### UT-010: Sensor Manager - Multiplexer Channel Switching
**Priority:** HIGH  
**Duration:** 1 minute  
**Environment:** Mock/Hardware

**Test Steps:**
1. Rapidly switch between Channel 0 and Channel 1
2. Verify stabilization delay (50µs) is adequate
3. Check channel select pins (S0-S3) driven correctly
4. Measure crosstalk between channels

**Success Criteria:**
- Clean channel transitions
- No crosstalk (< 1% adjacent channel)
- Consistent readings from same channel
- Switching time < 100µs total

**Switching Pattern:**
```
Ch0 → Ch1 → Ch0 → Ch1 (1000 times)
Random switching
All 16 channels test (if more sensors added)
```

---

### UT-011: Sensor Manager - Temperature Calculation
**Priority:** HIGH  
**Duration:** 2 minutes  
**Environment:** Mock

**Test Steps:**
1. Test Steinhart-Hart equation with known ADC values
2. Verify temperature calculation accuracy
3. Test boundary conditions (0°C, 25°C, 100°C)
4. Check for invalid readings (open/short circuit)

**Success Criteria:**
- Accuracy ±1°C at 25°C
- Accuracy ±2°C across 0-100°C range
- Returns reasonable value or error code
- No divide-by-zero or NaN results

**Test ADC Values:**
```
ADC 0    → Should detect open circuit
ADC 512  → ~25°C (nominal)
ADC 1023 → Should detect short circuit
```

---

### UT-012: Sensor Manager - Voltage Reading Accuracy
**Priority:** MEDIUM  
**Duration:** 1 minute  
**Environment:** Hardware

**Test Steps:**
1. Apply known voltages to Channel 0
2. Compare ADC readings to expected values
3. Check linearity across 0-1V range
4. Measure noise and stability

**Success Criteria:**
- Linearity R² > 0.999
- Accuracy ±0.01V
- Noise < 0.005V RMS
- Stable readings (σ < 0.01V)

**Test Voltages:**
```
0.000V, 0.250V, 0.500V, 0.750V, 1.000V
```

---

### UT-013: Heater Controller - PWM Output
**Priority:** HIGH  
**Duration:** 2 minutes  
**Environment:** Mock/Hardware

**Test Steps:**
1. Set duty cycles from 0% to 100%
2. Measure PWM frequency with oscilloscope
3. Verify smooth transitions
4. Check for glitches or spikes

**Success Criteria:**
- Frequency = PWM_FREQUENCY (10 Hz)
- Duty cycle accuracy ±1%
- No glitches on transitions
- Clean waveform (no ringing)

**Duty Cycle Tests:**
```
0%, 25%, 50%, 75%, 100%
Sweep 0%→100% in 10% steps
```

---

### UT-014: Heater Controller - Power Ramping
**Priority:** MEDIUM  
**Duration:** 3 minutes  
**Environment:** Mock

**Test Steps:**
1. Ramp heater power from 0% to 100%
2. Verify smooth transitions (no jumps)
3. Test rate limiting if implemented
4. Check for instability

**Success Criteria:**
- Smooth ramp (no discontinuities)
- Rate < 10%/second (if limited)
- No oscillations
- Predictable behavior

---

### UT-015: Web Interface - JSON Serialization
**Priority:** MEDIUM  
**Duration:** 2 minutes  
**Environment:** Mock

**Test Steps:**
1. Serialize sensor data to JSON
2. Check message size
3. Verify no memory leaks
4. Test edge cases (NaN, infinity, very large numbers)

**Success Criteria:**
- Valid JSON format
- Message size < 512 bytes
- No memory leaks after 1000 iterations
- Edge cases handled gracefully

**Test Data:**
```json
{
  "voltage": 0.5432,
  "temperature": 25.6,
  "heater": true,
  "pidValue": 45.3,
  "timestamp": 123456789
}
```

---

### UT-016: WiFi Manager - AP Mode Initialization
**Priority:** MEDIUM  
**Duration:** 30 seconds  
**Environment:** Hardware

**Test Steps:**
1. Start WiFi in AP mode
2. Verify SSID broadcasts
3. Check IP configuration
4. Test client can connect

**Success Criteria:**
- AP starts within 5 seconds
- SSID visible to clients
- IP = 192.168.4.1
- Client can associate and get DHCP

---

## Integration Tests

### IT-001: Memory Under Load - Combined Systems
**Priority:** CRITICAL  
**Duration:** 1 hour continuous  
**Environment:** Mock

**Test Steps:**
1. Enable all subsystems simultaneously:
   - Data logging at 500Hz per channel
   - WebSocket updates at 2Hz
   - PID controller active
   - File writes every 10 seconds
2. Monitor heap every second
3. Check for memory leaks

**Success Criteria:**
- Free heap stable for entire duration
- No monotonic decrease
- System remains responsive
- All subsystems functional

**Load Conditions:**
```
- 3600 sensor readings/channel (1.8M total over 1 hour)
- 7200 WebSocket messages
- 360 file writes
- Continuous PID calculations
```

---

### IT-002: Safety System Integration - Cascading Failures
**Priority:** CRITICAL  
**Duration:** 5 minutes  
**Environment:** Mock

**Test Steps:**
1. Trigger multiple safety conditions simultaneously:
   - High temperature
   - Watchdog timeout
   - Heater timeout
2. Verify system enters safe state
3. Check recovery procedures

**Success Criteria:**
- All safety systems respond
- Most critical takes precedence
- Heater disabled immediately
- System logs all events
- Cannot restart without clearing flags

---

### IT-003: Data Logging - End-to-End Pipeline
**Priority:** HIGH  
**Duration:** 10 minutes  
**Environment:** Mock + SD Card

**Test Steps:**
1. Collect sensor data → Buffer → File
2. Verify data integrity throughout
3. Check file format correctness
4. Test recovery from interruptions

**Success Criteria:**
- No lost samples
- Timestamps monotonic
- CSV format valid
- File readable after test
- Buffer cleared after writes

---

### IT-004: WebSocket Communication - Multi-Client
**Priority:** MEDIUM  
**Duration:** 5 minutes  
**Environment:** Mock + WiFi

**Test Steps:**
1. Connect 3 WebSocket clients simultaneously
2. Verify all receive updates
3. Disconnect/reconnect clients randomly
4. Check for memory leaks

**Success Criteria:**
- All clients receive identical data
- Disconnect doesn't crash server
- Reconnect successful
- No memory leaks from client management

---

### IT-005: Sensor Reading → Display Pipeline
**Priority:** HIGH  
**Duration:** 10 minutes  
**Environment:** Hardware

**Test Steps:**
1. Read sensors via multiplexer
2. Process data (voltage, temperature conversion)
3. Buffer readings
4. Send to WebSocket clients
5. Verify display matches sensor input

**Success Criteria:**
- End-to-end latency < 500ms
- Displayed values within ±1% of actual
- No dropped updates
- Smooth real-time display

---

### IT-006: PID → Heater → Sensor Feedback Loop
**Priority:** CRITICAL  
**Duration:** 15 minutes  
**Environment:** Hardware

**Test Steps:**
1. Set target temperature
2. Monitor PID calculations
3. Verify heater power adjusts
4. Check temperature response
5. Confirm stable at setpoint

**Success Criteria:**
- Reaches setpoint within 5 minutes
- Stable within ±1°C
- No oscillations
- PID output reasonable (not saturated)

---

### IT-007: File Rotation Under Load
**Priority:** HIGH  
**Duration:** 20 minutes  
**Environment:** Mock + SD Card

**Test Steps:**
1. Generate data rapidly to trigger rotation
2. Verify rotation occurs smoothly
3. Check no data loss during transition
4. Verify old file integrity

**Success Criteria:**
- Rotation completes in < 1 second
- No samples lost
- Both files readable and valid
- System continues logging immediately

---

### IT-008: Low Memory Recovery
**Priority:** CRITICAL  
**Duration:** 10 minutes  
**Environment:** Mock

**Test Steps:**
1. Simulate low memory condition (< 10KB heap)
2. Verify automatic buffer flush
3. Check data logging pause if needed
4. Test recovery when memory available

**Success Criteria:**
- System doesn't crash
- Auto-flush triggered
- Logging pauses gracefully
- Resumes when heap recovers
- Warning logged

---

## End-to-End Tests

### E2E-001: 24-Hour Continuous Operation
**Priority:** CRITICAL  
**Duration:** 24 hours  
**Environment:** Full Hardware

**Test Steps:**
1. Start system with all features enabled
2. Monitor remotely (log files + web interface)
3. Check stability metrics every hour
4. Verify no degradation over time

**Success Criteria:**
- System runs for full 24 hours
- Free heap stable (variance < 10%)
- All sensors reading correctly
- Data logging continuous
- Web interface responsive
- No crashes or resets

**Monitoring Points:**
```
Every 1 hour:
- Free heap
- Total readings collected
- File sizes
- Temperature stability
- WebSocket clients connected
- Any error messages
```

---

### E2E-002: Temperature Control Accuracy
**Priority:** HIGH  
**Duration:** 2 hours  
**Environment:** Full Hardware

**Test Steps:**
1. Set target temperature to 60°C
2. Monitor temperature over time
3. Verify PID maintains setpoint
4. Test disturbance rejection

**Success Criteria:**
- Steady-state error < ±1°C
- Rise time < 10 minutes
- No overshoot > 5°C
- Stable for 90+ minutes

**Temperature Profile:**
```
T=0min:   Start at 20°C
T=10min:  Should reach 60°C
T=60min:  Add cold disturbance (fan)
T=90min:  Remove disturbance
T=120min: Verify still stable at 60°C
```

---

### E2E-003: Safety System Validation - Thermal Runaway
**Priority:** CRITICAL  
**Duration:** 30 minutes  
**Environment:** Full Hardware (with supervision)

**Test Steps:**
1. Enable heater with normal target (60°C)
2. Simulate PID failure (force output 100%)
3. Verify safety timeout triggers
4. Check emergency shutdown

**Success Criteria:**
- Heater disabled at MAX_HEATER_TIME
- Emergency shutdown flag set
- Temperature doesn't exceed MAX_SAFE_TEMPERATURE
- Cannot restart without reset

**SAFETY NOTE:** Test with low power heater or in controlled environment!

---

### E2E-004: Data Integrity - Long Term Logging
**Priority:** HIGH  
**Duration:** 8 hours  
**Environment:** Full Hardware

**Test Steps:**
1. Log continuously for 8 hours
2. Download all log files
3. Verify data integrity:
   - No gaps in timestamps
   - No corrupted entries
   - File rotation successful
4. Check statistical properties

**Success Criteria:**
- Zero lost samples
- All timestamps monotonic increasing
- CSV format valid throughout
- File sizes predictable
- Data matches expected patterns

**Expected Data Volume:**
```
8 hours × 500Hz per channel × 2 channels = 28,800,000 readings
Estimated file size: ~2GB (depending on CSV format)
Number of files: Depends on MAX_FILE_SIZE
```

---

### E2E-005: Network Stress Test
**Priority:** MEDIUM  
**Duration:** 30 minutes  
**Environment:** Full Hardware + Multiple Clients

**Test Steps:**
1. Connect 5 WebSocket clients
2. Request data updates at maximum rate
3. Monitor system performance
4. Check for degradation

**Success Criteria:**
- All clients receive updates
- Update rate >= 1Hz per client
- Free heap remains > 10KB
- Sensor sampling not impacted
- No clients disconnected unexpectedly

---

### E2E-006: Power Cycle Recovery
**Priority:** HIGH  
**Duration:** 1 hour (multiple cycles)  
**Environment:** Full Hardware

**Test Steps:**
1. Start logging
2. After 10 minutes, power cycle
3. Verify system restarts correctly
4. Check data file integrity
5. Repeat 6 times

**Success Criteria:**
- Restarts successfully every time
- No file corruption
- Data recoverable from each session
- Settings persist (if EEPROM used)
- No memory leaks across restarts

---

### E2E-007: Sensor Failure Handling
**Priority:** MEDIUM  
**Duration:** 15 minutes  
**Environment:** Full Hardware

**Test Steps:**
1. Disconnect thermistor (open circuit)
2. Verify error detection
3. Short thermistor to ground
4. Verify error detection
5. Reconnect and verify recovery

**Success Criteria:**
- Open circuit detected (temp = invalid)
- Short circuit detected (temp = invalid)
- Error logged
- Heater disabled on sensor failure
- System continues running
- Recovery on reconnection

---

## Test Environment Setup

### Mock Testing Environment

#### Software Requirements
```bash
# PlatformIO for building
pip install platformio

# Unity test framework (built into PlatformIO)
# Already included in PlatformIO

# Optional: Native unit testing
pio test --environment native
```

#### Mock Implementation Strategy
```cpp
// test/mocks/mock_arduino.h
// Provide Arduino.h functions for native testing

// test/mocks/mock_wifi.h
// Mock ESP8266WiFi without hardware

// test/mocks/mock_filesystem.h
// Mock LittleFS/SD operations in memory

// test/mocks/mock_sensors.h
// Generate synthetic sensor data
```

#### Directory Structure
```
test/
├── unit/
│   ├── test_memory/
│   ├── test_safety/
│   ├── test_sensors/
│   ├── test_heater/
│   ├── test_filesystem/
│   └── test_webinterface/
├── integration/
│   ├── test_sensor_pipeline/
│   ├── test_safety_integration/
│   └── test_logging_pipeline/
├── e2e/
│   └── test_scenarios/
├── mocks/
│   ├── mock_arduino.h
│   ├── mock_wifi.h
│   ├── mock_filesystem.h
│   └── mock_sensors.h
└── helpers/
    ├── test_utilities.h
    └── memory_monitor.h
```

---

### Hardware Testing Environment

#### Required Equipment
- NodeMCU ESP-12E (Hitlego board)
- CD74HC4067 multiplexer breakout
- 100kΩ NTC thermistor
- AOD4144 MOSFET or equivalent
- Low-power heater element (< 50W for safety)
- Adjustable power supply (0-1V for voltage testing)
- Micro SD card (8GB+ recommended)
- Oscilloscope (for PWM verification)
- Digital multimeter
- Temperature measurement (thermocouple or IR thermometer)
- USB serial adapter for logging

#### Test Bench Setup
```
┌─────────────────────────────────────────────────┐
│  Test Bench Configuration                       │
├─────────────────────────────────────────────────┤
│                                                 │
│  [Power Supply] → [NodeMCU ESP-12E]            │
│                         ↓                       │
│                   [CD74HC4067]                  │
│                    ↙          ↘                 │
│         [Voltage Source]  [Thermistor]          │
│              ↓                  ↓               │
│         [Multimeter]       [Temp Sensor]        │
│                                                 │
│  [GPIO16] → [MOSFET] → [Heater] → [Power]     │
│                                                 │
│  [SD Card Module] ← [SPI Bus]                  │
│                                                 │
│  [Laptop] ←─ USB ─→ [ESP8266] (serial monitor) │
│     ↓                                           │
│  [WiFi] ←─ wireless ─→ [ESP8266 AP]           │
│                                                 │
└─────────────────────────────────────────────────┘
```

#### Safety Precautions for Hardware Testing
1. Use low-power heater (< 50W)
2. Temperature-rated insulation
3. Fire extinguisher nearby
4. Don't leave unattended during thermal tests
5. Thermal cutoff fuse in heater circuit
6. Emergency power cutoff switch
7. Well-ventilated area

---

## Success Criteria

### System-Level Success Criteria

#### Memory Stability
- [ ] Free heap > 10KB at all times
- [ ] No monotonic decrease over 24 hours
- [ ] Heap fragmentation < 30%
- [ ] No memory allocation failures
- [ ] Automatic recovery from low memory

#### Safety System Reliability
- [ ] Watchdog triggers within timeout window
- [ ] Emergency shutdown in < 1 second
- [ ] Temperature limit enforcement 100% reliable
- [ ] Heater timeout enforced
- [ ] PID stable with no runaway

#### Data Integrity
- [ ] Zero lost samples in normal operation
- [ ] File writes succeed or fail gracefully
- [ ] File rotation seamless
- [ ] CSV format always valid
- [ ] Recovery from interruptions

#### Performance
- [ ] Sensor sampling rate: 500Hz per channel sustained
- [ ] Web update rate: 2Hz minimum
- [ ] PID update rate: 10Hz
- [ ] WebSocket latency: < 500ms
- [ ] File write latency: < 100ms

#### Operational Requirements
- [ ] 24+ hour continuous operation
- [ ] Temperature control ±1°C steady-state
- [ ] Survives power cycles
- [ ] Recovers from sensor failures
- [ ] Web interface always responsive

---

### Per-Test Success Criteria

Each test must document:
1. **Pass Condition:** What specifically must happen
2. **Fail Condition:** What indicates failure
3. **Measurement Method:** How to verify
4. **Acceptable Variance:** Tolerance limits

Example:
```
Test: UT-001 Heap Monitoring
Pass: Free heap > 10KB for 15 minutes, variance < 5KB
Fail: Heap < 10KB at any point, or monotonic decrease
Measurement: ESP.getFreeHeap() logged every 1s
Variance: ±5KB acceptable due to buffer cycling
```

---

## Failure Scenarios

### Critical Failure Scenarios (Must Handle Gracefully)

#### FS-001: Memory Exhaustion
**Trigger:** Heap < 8KB  
**Expected Behavior:**
1. Auto-stop data logging
2. Flush current buffer to file
3. Log warning message
4. System continues safety monitoring
5. Recovery when heap > 15KB

**Test:** Artificially consume heap until threshold

---

#### FS-002: SD Card Failure
**Trigger:** SD card removed or full  
**Expected Behavior:**
1. Detect write failure
2. Log error to Serial
3. Continue sensor monitoring
4. Buffer data in RAM (if space)
5. Retry writes periodically
6. Graceful degradation (logging disabled)

**Test:** Remove SD card mid-write

---

#### FS-003: Temperature Sensor Failure
**Trigger:** Thermistor disconnected  
**Expected Behavior:**
1. Detect invalid reading (open/short)
2. Disable heater immediately
3. Set emergency flag
4. Log error
5. Continue voltage monitoring
6. Recovery on reconnection

**Test:** Disconnect thermistor wire

---

#### FS-004: Temperature Runaway
**Trigger:** T > MAX_SAFE_TEMPERATURE  
**Expected Behavior:**
1. Emergency shutdown in < 1 second
2. Heater disabled and latched off
3. Emergency flag set
4. Cannot restart without manual reset
5. Log critical error
6. Visual indication (if LED present)

**Test:** Override PID to force 100% power

---

#### FS-005: Watchdog Timeout
**Trigger:** Main loop stalls > 10 seconds  
**Expected Behavior:**
1. Watchdog timer triggers
2. Emergency shutdown
3. Heater disabled
4. System reset or safe state
5. Log preserved if possible

**Test:** Insert blocking delay() in loop()

---

#### FS-006: WiFi AP Failure
**Trigger:** AP mode fails to start  
**Expected Behavior:**
1. Log error to Serial
2. Continue data logging
3. Retry AP initialization
4. Fallback to standalone mode
5. Data still persists to SD card

**Test:** Simulate WiFi initialization failure

---

#### FS-007: WebSocket Buffer Overflow
**Trigger:** Client disconnects without draining queue  
**Expected Behavior:**
1. Detect buffer full
2. Drop oldest messages
3. Log warning
4. Continue sending new data
5. No memory leak

**Test:** Disconnect client without closing connection

---

#### FS-008: File Write Timeout
**Trigger:** SD card slow response  
**Expected Behavior:**
1. Timeout after reasonable period (5s)
2. Retry once
3. On failure, log error and continue
4. Data lost but system stable

**Test:** Use very slow/damaged SD card

---

#### FS-009: Multiplexer Channel Stuck
**Trigger:** Hardware failure in channel switching  
**Expected Behavior:**
1. Detect same ADC value repeatedly
2. Log warning
3. Attempt channel reset
4. Mark sensor as faulty
5. Continue with remaining sensors

**Test:** Disconnect MUX control pins

---

#### FS-010: Heater Stuck On
**Trigger:** MOSFET or circuit failure  
**Expected Behavior:**
1. Safety timer still triggers
2. Temperature limit enforced
3. Watchdog provides final backstop
4. System enters emergency shutdown
5. Physical thermal fuse as last resort

**Test:** Requires careful hardware setup - simulate with GPIO override

---

### Failure Detection Methods

#### Memory Leak Detection
```cpp
void checkMemoryLeak() {
  static uint32_t lastHeap = ESP.getFreeHeap();
  uint32_t currentHeap = ESP.getFreeHeap();
  
  if (currentHeap < lastHeap - 1000) {
    Serial.println("MEMORY LEAK DETECTED!");
    Serial.printf("Heap dropped: %u → %u\n", lastHeap, currentHeap);
  }
  
  lastHeap = currentHeap;
}
```

#### Watchdog Verification
```cpp
void testWatchdog() {
  Serial.println("Testing watchdog - blocking loop...");
  delay(15000); // Exceed watchdog timeout
  // Should never reach here
  Serial.println("ERROR: Watchdog failed!");
}
```

#### Safety System Validation
```cpp
void testSafetyShutdown() {
  Serial.println("Triggering emergency shutdown...");
  emergencyShutdown = true;
  
  // Verify heater disabled
  if (heaterDutyCycle > 0) {
    Serial.println("ERROR: Heater still active!");
  }
}
```

---

## Test Execution Plan

### Week 1: Unit Tests (Mock Environment)
**Day 1-2:** Memory management tests (UT-001 to UT-003)  
**Day 3-4:** Safety system tests (UT-004 to UT-007)  
**Day 5:** File system tests (UT-008 to UT-009)  
**Day 6:** Sensor and heater tests (UT-010 to UT-014)  
**Day 7:** Web interface tests (UT-015 to UT-016)

### Week 2: Integration Tests (Mock + Limited Hardware)
**Day 1-2:** Memory under load tests (IT-001 to IT-002)  
**Day 3:** Data logging pipeline (IT-003 to IT-004)  
**Day 4:** Sensor to display pipeline (IT-005)  
**Day 5:** PID feedback loop (IT-006)  
**Day 6-7:** File rotation and recovery tests (IT-007 to IT-008)

### Week 3+: End-to-End Tests (Full Hardware)
**Day 1:** 24-hour continuous operation (E2E-001)  
**Day 2:** Temperature control accuracy (E2E-002)  
**Day 3:** Safety validation - supervised (E2E-003)  
**Day 4-5:** Long-term data logging (E2E-004)  
**Day 6:** Network stress testing (E2E-005)  
**Day 7:** Power cycle and sensor failure tests (E2E-006, E2E-007)

---

## Test Reporting

### Per-Test Report Template
```
TEST ID: UT-001
TEST NAME: Memory Management - Heap Monitoring
DATE: YYYY-MM-DD
TESTER: [Name]
ENVIRONMENT: Mock/Hardware

RESULT: PASS / FAIL / PARTIAL

MEASUREMENTS:
- Starting heap: [value]
- Ending heap: [value]
- Minimum heap: [value]
- Heap variance: [value]

OBSERVATIONS:
[Detailed notes]

ISSUES FOUND:
[Any problems discovered]

ATTACHMENTS:
- logs/UT-001_heap_log.txt
- screenshots/UT-001_graph.png
```

### Summary Report Requirements
- Test coverage percentage
- Pass/fail breakdown
- Critical issues discovered
- Risk assessment
- Recommendations for improvement
- Sign-off for production readiness

---

## Continuous Integration

### Automated Testing (Future Enhancement)
```yaml
# .github/workflows/test.yml
name: PlatformIO Test

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Set up Python
        uses: actions/setup-python@v2
      - name: Install PlatformIO
        run: pip install platformio
      - name: Run unit tests
        run: pio test --environment native
      - name: Upload test results
        uses: actions/upload-artifact@v2
        with:
          name: test-results
          path: test-results/
```

---

## Conclusion

This testing specification provides a comprehensive framework for validating the ESP8266 Voltage Logger system. The focus on **memory stability** and **safety systems** ensures the device can operate reliably for extended periods without human intervention.

**Key Priorities:**
1. Memory must never exhaust
2. Safety systems must be bulletproof
3. Data integrity must be maintained
4. System must fail gracefully, never catastrophically

**Next Steps:**
1. Review and approve this specification
2. Set up test environment (mock framework)
3. Implement unit tests (Week 1)
4. Execute integration tests (Week 2)
5. Perform hardware validation (Week 3+)
6. Document results and iterate

---

**Document Control**
- Version: 1.0
- Author: Test Specification Team
- Approved by: [Pending]
- Last Review: 2025-01-02
- Next Review: After Phase 1 completion
