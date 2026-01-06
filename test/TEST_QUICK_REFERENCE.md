# Testing Quick Reference Guide

## 🎯 Priority Testing Checklist

### ✅ CRITICAL Tests (Must Pass Before Deployment)

#### Memory Management
- [ ] **UT-001**: Heap monitoring (15 min) - Free heap > 10KB
- [ ] **UT-002**: Buffer overflow protection (5 min)
- [ ] **UT-003**: String memory leaks (10 min)
- [ ] **IT-001**: Memory under full load (1 hour)
- [ ] **E2E-001**: 24-hour continuous operation

#### Safety Systems  
- [ ] **UT-004**: Hardware watchdog (30 sec)
- [ ] **UT-005**: Temperature limit enforcement (2 min)
- [ ] **UT-006**: Heater timeout (12 min)
- [ ] **UT-007**: PID controller stability (10 min)
- [ ] **IT-002**: Cascading safety failures (5 min)
- [ ] **E2E-003**: Thermal runaway prevention (30 min)

#### File System
- [ ] **UT-008**: Write failure recovery (3 min)
- [ ] **UT-009**: File rotation (5 min)
- [ ] **IT-007**: File rotation under load (20 min)

---

## 🚀 Quick Start: Running Your First Test

### Option 1: Mock Testing (No Hardware Required)
```bash
cd voltage_logger
pio test --environment native --filter "test_memory"
```

### Option 2: Hardware Testing
```bash
# Upload test firmware
pio test --environment esp12e --filter "test_sensors"

# Monitor serial output
pio device monitor --baud 115200
```

---

## 📊 Test Execution Order

### Phase 1: Unit Tests (Mock) - Week 1
```
Day 1-2: Memory tests (UT-001 to UT-003)
Day 3-4: Safety tests (UT-004 to UT-007)  
Day 5:   File system tests (UT-008 to UT-009)
Day 6:   Sensor tests (UT-010 to UT-012)
Day 7:   Interface tests (UT-013 to UT-016)
```

### Phase 2: Integration Tests - Week 2
```
Day 1-2: Load testing (IT-001 to IT-002)
Day 3-4: Pipeline testing (IT-003 to IT-005)
Day 5:   Feedback loops (IT-006)
Day 6-7: Recovery testing (IT-007 to IT-008)
```

### Phase 3: End-to-End - Week 3+
```
Day 1:   24-hour stability test (E2E-001)
Day 2:   Temperature accuracy (E2E-002)
Day 3:   Safety validation (E2E-003) ⚠️ SUPERVISED
Day 4-5: Data logging integrity (E2E-004)
Day 6:   Stress testing (E2E-005)
Day 7:   Recovery tests (E2E-006, E2E-007)
```

---

## 🔍 Common Test Commands

### Run All Unit Tests
```bash
pio test --environment native
```

### Run Specific Test
```bash
pio test --environment native --filter "test_safety"
```

### Run on Hardware
```bash
pio test --environment esp12e --filter "*"
```

### Monitor During Test
```bash
pio device monitor --baud 115200 --filter log2file
```

### Check Code Coverage
```bash
pio test --environment native --with-coverage
```

---

## 📈 Success Criteria Summary

| Category | Metric | Target | Critical? |
|----------|--------|--------|-----------|
| **Memory** | Free Heap | > 10KB always | ✅ YES |
| **Memory** | Heap Variance | < 5KB over 24h | ✅ YES |
| **Safety** | Watchdog Response | < 10 seconds | ✅ YES |
| **Safety** | Temp Limit Response | < 1 second | ✅ YES |
| **Safety** | PID Stability | ±1°C steady-state | ✅ YES |
| **Data** | Lost Samples | 0 in normal ops | ✅ YES |
| **Performance** | Sampling Rate | 500Hz per channel | ⚠️ HIGH |
| **Performance** | Web Updates | 2Hz minimum | ⚠️ HIGH |
| **Reliability** | Uptime | 24+ hours | ✅ YES |

---

## ⚠️ Safety Testing Precautions

### Before Running Hardware Tests:
1. ✅ Low-power heater (< 50W)
2. ✅ Fire extinguisher nearby
3. ✅ Temperature monitoring
4. ✅ Emergency power cutoff
5. ✅ Well-ventilated area
6. ✅ Never leave unattended

### E2E-003 Thermal Runaway Test:
**⚠️ REQUIRES SUPERVISION AT ALL TIMES**
- Have manual power cutoff ready
- Monitor temperature continuously
- Abort if T > 110°C
- Use thermal camera if available

---

## 🐛 Debugging Failed Tests

### Memory Test Failed?
```cpp
// Add to setup():
ESP.setWatchdog(WDTO_8S);

// Add to loop():
if (millis() % 1000 == 0) {
  Serial.printf("Heap: %u\n", ESP.getFreeHeap());
}
```

### Safety Test Failed?
```cpp
// Check safety flags:
Serial.printf("Emergency: %d, Heater: %d\n", 
              emergencyShutdown, heaterEnabled);

// Verify watchdog fed:
Serial.printf("Last feed: %lu ms ago\n", 
              millis() - lastWatchdogFeed);
```

### Sensor Test Failed?
```cpp
// Check multiplexer:
selectMuxChannel(0);
delay(10);
Serial.printf("Ch0 ADC: %d\n", analogRead(A0));

selectMuxChannel(1);
delay(10);
Serial.printf("Ch1 ADC: %d\n", analogRead(A0));
```

---

## 📝 Test Report Template (Quick)

```markdown
## Test Session: [Date]

**Tests Run:** [count]
**Passed:** [count]  
**Failed:** [count]
**Duration:** [time]

### Failed Tests:
- [Test ID]: [Reason]

### Critical Issues:
- [Issue description]

### Recommendations:
- [Action items]
```

---

## 🔧 Test Framework Setup

### First Time Setup
```bash
# Install PlatformIO
pip install platformio

# Initialize test environment
cd voltage_logger
pio test --environment native --verbose

# Should see:
# Unity Test Framework ready
# Running tests...
```

### Directory Structure
```
voltage_logger/
├── test/
│   ├── test_memory/           # Memory tests
│   │   └── test_heap.cpp
│   ├── test_safety/           # Safety system tests
│   │   ├── test_watchdog.cpp
│   │   └── test_temperature.cpp
│   ├── test_sensors/          # Sensor tests
│   │   └── test_multiplexer.cpp
│   └── test_integration/      # Integration tests
│       └── test_pipeline.cpp
└── platformio.ini             # Test configurations
```

---

## 💡 Tips for Effective Testing

### 1. Start Small
Begin with **UT-001** (heap monitoring). It's simple and catches critical issues.

### 2. Use Serial Logging
Add debug output liberally during testing:
```cpp
#define DEBUG_TEST 1
#if DEBUG_TEST
  Serial.printf("Test checkpoint: %d\n", __LINE__);
#endif
```

### 3. Automate Where Possible
Use PlatformIO's built-in test framework instead of manual testing.

### 4. Document Failures Immediately
Don't rely on memory - write down what failed and why.

### 5. Test One Thing at a Time
Don't enable all features when debugging. Isolate problems.

---

## 📞 Need Help?

### Common Issues:

**Q: Tests won't compile**
```bash
# Clean and rebuild
pio run --target clean
pio test --environment native --verbose
```

**Q: Hardware tests timeout**
```bash
# Check USB connection
ls /dev/ttyUSB*

# Verify board detected
pio device list
```

**Q: Memory tests show leaks**
```bash
# Enable detailed memory tracking
build_flags = -DDEBUG_ESP_CORE
```

**Q: Safety tests don't trigger**
```bash
# Verify watchdog enabled
Serial.printf("Watchdog: %d\n", systemAlive);
```

---

## 🎓 Learning Resources

### PlatformIO Testing
- [Official Docs](https://docs.platformio.org/en/latest/plus/unit-testing.html)
- [Unity Framework](http://www.throwtheswitch.org/unity)

### ESP8266 Debugging
- [ESP8266 Arduino Core Docs](https://arduino-esp8266.readthedocs.io/)
- [Memory Debugging Guide](https://arduino-esp8266.readthedocs.io/en/latest/faq/a02-my-esp-crashes.html)

### Embedded Testing Best Practices
- [Embedded Testing Guide](https://interrupt.memfault.com/blog/unit-testing-basics)

---

**Quick Reference Version 1.0**  
**Last Updated:** 2025-01-02

For complete details, see **TESTING_SPECIFICATION.md**
