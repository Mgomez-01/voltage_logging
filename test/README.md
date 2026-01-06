# Voltage Logger Test Suite

This directory contains the comprehensive test suite for the ESP8266 Voltage Logger project.

## Quick Start

### Run All Tests (Mock Environment)
```bash
cd voltage_logger
pio test --environment native
```

### Run Specific Test Group
```bash
# Memory tests
pio test --environment native --filter "test_memory"

# Safety tests
pio test --environment native --filter "test_safety"
```

### Run on Hardware
```bash
# Upload and run tests on ESP8266
pio test --environment test_esp12e --filter "test_safety"
```

## Test Organization

```
test/
├── test_memory/              # UT-001 to UT-003: Memory management
│   └── test_heap_monitoring.cpp
├── test_safety/              # UT-004 to UT-007: Safety systems
│   ├── test_watchdog.cpp
│   ├── test_temperature_limits.cpp
│   └── test_pid_controller.cpp (TODO)
├── test_sensors/             # UT-010 to UT-012: Sensor reading
│   └── (TODO)
├── test_filesystem/          # UT-008 to UT-009: File operations
│   └── (TODO)
├── test_heater/              # UT-013 to UT-014: Heater control
│   └── (TODO)
├── test_webinterface/        # UT-015 to UT-016: Web interface
│   └── (TODO)
└── TEST_QUICK_REFERENCE.md   # Quick reference guide
```

## Test Status

### ✅ Implemented
- **UT-001**: Heap Monitoring (5 tests)
- **UT-004**: Watchdog Timer (6 tests)
- **UT-005**: Temperature Limits (10 tests)

### 🔨 TODO (Priority Order)
1. **UT-006**: Heater Timeout (CRITICAL)
2. **UT-007**: PID Controller Stability (CRITICAL)
3. **UT-002**: Buffer Overflow Protection (CRITICAL)
4. **UT-003**: String Memory Leaks (CRITICAL)
5. **UT-008**: File Write Failure Recovery (HIGH)
6. **UT-009**: File Rotation (HIGH)
7. Integration tests
8. End-to-end tests

## Running Tests

### Prerequisites
```bash
# Install PlatformIO
pip install platformio

# Or using homebrew on macOS
brew install platformio
```

### Test Commands

```bash
# Run all native tests
pio test --environment native

# Run with verbose output
pio test --environment native --verbose

# Run specific test file
pio test --environment native --filter "test_heap_monitoring"

# Run on hardware
pio test --environment test_esp12e

# Monitor serial output during hardware test
pio test --environment test_esp12e && pio device monitor
```

## Test Results

Tests will output results in Unity format:

```
test/test_memory/test_heap_monitoring.cpp:45:test_heap_stable_over_time:PASS
test/test_memory/test_heap_monitoring.cpp:68:test_no_monotonic_heap_decrease:PASS
...
-----------------------
21 Tests 0 Failures 0 Ignored
OK
```

## Writing New Tests

### Template for Unit Test

```cpp
#include <unity.h>

#ifdef UNIT_TEST

// Your test code here

void test_feature_name() {
    // Arrange
    int expected = 5;
    
    // Act
    int actual = someFunction();
    
    // Assert
    TEST_ASSERT_EQUAL(expected, actual);
}

void setUp(void) {
    // Runs before each test
}

void tearDown(void) {
    // Runs after each test
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_feature_name);
    return UNITY_END();
}

#endif // UNIT_TEST
```

### Unity Assertions Reference

```cpp
// Basic assertions
TEST_ASSERT(condition)
TEST_ASSERT_TRUE(condition)
TEST_ASSERT_FALSE(condition)

// Equality
TEST_ASSERT_EQUAL(expected, actual)
TEST_ASSERT_EQUAL_INT(expected, actual)
TEST_ASSERT_EQUAL_FLOAT(expected, actual, delta)

// Comparison
TEST_ASSERT_GREATER_THAN(threshold, actual)
TEST_ASSERT_LESS_THAN(threshold, actual)

// Pointers
TEST_ASSERT_NULL(pointer)
TEST_ASSERT_NOT_NULL(pointer)

// With messages
TEST_ASSERT_EQUAL_MESSAGE(expected, actual, "Custom error message")
```

## Test Coverage Goals

| Category | Target Coverage | Current |
|----------|----------------|---------|
| Memory Management | 100% | 60% |
| Safety Systems | 100% | 50% |
| Sensor Reading | 90% | 0% |
| File Operations | 90% | 0% |
| Web Interface | 80% | 0% |

## Debugging Failed Tests

### Enable Debug Output
```cpp
#define DEBUG_TEST 1

void test_something() {
    #if DEBUG_TEST
    printf("Debug: variable = %d\n", variable);
    #endif
    TEST_ASSERT(condition);
}
```

### Common Issues

**Test won't compile:**
```bash
pio run --target clean
pio test --environment native --verbose
```

**Test passes locally but fails on hardware:**
- Check timing assumptions (hardware is slower)
- Verify hardware initialization
- Add delays for sensor stabilization

**Memory tests show false positives:**
- Increase sample size
- Check for timer-based allocations
- Consider ESP8266 background tasks

## Integration with CI/CD

### GitHub Actions Example
```yaml
name: PlatformIO Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
      - name: Install PlatformIO
        run: pip install platformio
      - name: Run Tests
        run: pio test --environment native
```

## Performance Benchmarks

Expected test execution times:

| Test Group | Mock Time | Hardware Time |
|------------|-----------|---------------|
| Memory | 30 seconds | N/A |
| Safety | 1 minute | 5 minutes |
| Sensors | 30 seconds | 2 minutes |
| File System | 1 minute | 5 minutes |
| **Total** | **3-5 minutes** | **15-20 minutes** |

## Contributing Tests

When adding new tests:

1. Follow the UT-XXX naming convention
2. Add test to appropriate directory
3. Update this README
4. Ensure test passes on native environment
5. Document expected results
6. Add to test execution plan

## Support

For issues or questions:
- See `TESTING_SPECIFICATION.md` for detailed requirements
- See `TEST_QUICK_REFERENCE.md` for quick tips
- Check PlatformIO docs: https://docs.platformio.org/en/latest/plus/unit-testing.html

## License

Same as main project.

---

**Last Updated:** 2025-01-02  
**Version:** 1.0
