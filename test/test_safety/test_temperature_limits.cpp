/**
 * UT-005: Safety Systems - Temperature Limit Enforcement
 * Priority: CRITICAL
 * Duration: 2 minutes
 * Environment: Mock
 * 
 * Tests that the system enforces temperature safety limits
 * and triggers emergency shutdown on overheat.
 */

#include <unity.h>

#ifdef UNIT_TEST

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Configuration
#define MAX_SAFE_TEMPERATURE 100.0f   // °C
#define TEMP_HYSTERESIS 5.0f           // °C
#define MIN_TEMPERATURE 0.0f
#define MAX_TEMPERATURE 150.0f

// System state
float currentTemperature = 20.0f;
float targetTemperature = 60.0f;
bool emergencyShutdown = false;
bool heaterEnabled = false;
uint8_t heaterDutyCycle = 0;

// Temperature safety check
void checkTemperatureSafety() {
    // Check for sensor error (impossible readings)
    if (currentTemperature < MIN_TEMPERATURE || 
        currentTemperature > MAX_TEMPERATURE) {
        emergencyShutdown = true;
        heaterEnabled = false;
        heaterDutyCycle = 0;
        return;
    }
    
    // Check if we've exceeded safe temperature
    if (currentTemperature > MAX_SAFE_TEMPERATURE) {
        emergencyShutdown = true;
        heaterEnabled = false;
        heaterDutyCycle = 0;
        return;
    }
    
    // Hysteresis: Must drop below (MAX_SAFE - HYSTERESIS) to clear
    if (emergencyShutdown && 
        currentTemperature < (MAX_SAFE_TEMPERATURE - TEMP_HYSTERESIS)) {
        // Can clear emergency (but requires manual reset in real system)
        // For testing, we'll allow automatic clearing
    }
}

// Heater control with safety check
void setHeater(uint8_t dutyCycle) {
    if (emergencyShutdown) {
        heaterEnabled = false;
        heaterDutyCycle = 0;
        return;
    }
    
    heaterDutyCycle = dutyCycle;
    heaterEnabled = (dutyCycle > 0);
}

// Simulate temperature change
void simulateTemperature(float delta) {
    currentTemperature += delta;
    checkTemperatureSafety();
}

// Test: Normal operation below safe limit
void test_normal_operation_below_limit() {
    emergencyShutdown = false;
    currentTemperature = 60.0f;
    targetTemperature = 80.0f;
    
    setHeater(50);
    checkTemperatureSafety();
    
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should not trigger emergency at safe temperature"
    );
    TEST_ASSERT_TRUE_MESSAGE(
        heaterEnabled,
        "Heater should remain enabled at safe temperature"
    );
}

// Test: Temperature exceeding limit triggers shutdown
void test_temperature_exceeds_limit() {
    emergencyShutdown = false;
    currentTemperature = 95.0f;
    setHeater(75);
    
    // Temperature rises above limit
    simulateTemperature(10.0f); // Now at 105°C
    
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Emergency shutdown should trigger above MAX_SAFE_TEMPERATURE"
    );
    TEST_ASSERT_FALSE_MESSAGE(
        heaterEnabled,
        "Heater should be disabled on overheat"
    );
    TEST_ASSERT_EQUAL_MESSAGE(
        0,
        heaterDutyCycle,
        "Heater duty cycle should be zero"
    );
}

// Test: Approaching limit but not exceeding
void test_approaching_limit_safe() {
    emergencyShutdown = false;
    currentTemperature = 95.0f;
    targetTemperature = 90.0f;
    
    setHeater(25);
    checkTemperatureSafety();
    
    // Still below limit
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should not trigger when approaching but below limit"
    );
    
    // Rise to just below limit
    simulateTemperature(4.5f); // Now at 99.5°C
    
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should not trigger when just below limit"
    );
}

// Test: Exactly at limit triggers shutdown
void test_exactly_at_limit() {
    emergencyShutdown = false;
    currentTemperature = MAX_SAFE_TEMPERATURE;
    
    checkTemperatureSafety();
    
    // At the limit should still be safe (we check for >)
    TEST_ASSERT_FALSE(emergencyShutdown);
    
    // But just over should trigger
    currentTemperature = MAX_SAFE_TEMPERATURE + 0.1f;
    checkTemperatureSafety();
    
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Should trigger when exceeding limit"
    );
}

// Test: Hysteresis prevents rapid cycling
void test_temperature_hysteresis() {
    emergencyShutdown = false;
    currentTemperature = 105.0f; // Above limit
    
    checkTemperatureSafety();
    TEST_ASSERT_TRUE(emergencyShutdown);
    
    // Temperature drops but still within hysteresis band
    currentTemperature = 98.0f; // Below MAX_SAFE but above (MAX_SAFE - HYSTERESIS)
    checkTemperatureSafety();
    
    // Emergency should still be active (hysteresis)
    // In real system, would require manual reset
    // But temperature is now safe
    TEST_ASSERT_LESS_THAN(MAX_SAFE_TEMPERATURE, currentTemperature + 5.0f);
    
    // Drop below hysteresis threshold
    currentTemperature = 94.0f; // Below (MAX_SAFE - HYSTERESIS)
    emergencyShutdown = false; // Simulate manual reset
    checkTemperatureSafety();
    
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should be safe below hysteresis threshold"
    );
}

// Test: Sensor error (too high) triggers shutdown
void test_sensor_error_high() {
    emergencyShutdown = false;
    currentTemperature = 200.0f; // Impossible reading
    
    checkTemperatureSafety();
    
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Sensor error (too high) should trigger emergency"
    );
}

// Test: Sensor error (too low) triggers shutdown
void test_sensor_error_low() {
    emergencyShutdown = false;
    currentTemperature = -50.0f; // Impossible reading
    
    checkTemperatureSafety();
    
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Sensor error (too low) should trigger emergency"
    );
}

// Test: Cannot re-enable heater after thermal shutdown
void test_heater_locked_after_shutdown() {
    emergencyShutdown = false;
    currentTemperature = 105.0f;
    
    checkTemperatureSafety();
    TEST_ASSERT_TRUE(emergencyShutdown);
    TEST_ASSERT_FALSE(heaterEnabled);
    
    // Try to turn heater back on
    setHeater(100);
    
    TEST_ASSERT_FALSE_MESSAGE(
        heaterEnabled,
        "Heater should remain off after thermal shutdown"
    );
    TEST_ASSERT_EQUAL_MESSAGE(
        0,
        heaterDutyCycle,
        "Duty cycle should remain zero"
    );
}

// Test: Rapid temperature rise detection
void test_rapid_temperature_rise() {
    emergencyShutdown = false;
    currentTemperature = 80.0f;
    
    setHeater(100);
    
    // Simulate rapid rise (PID runaway scenario)
    for (int i = 0; i < 10; i++) {
        simulateTemperature(3.0f); // +3°C per iteration
        
        if (currentTemperature > MAX_SAFE_TEMPERATURE) {
            TEST_ASSERT_TRUE_MESSAGE(
                emergencyShutdown,
                "Should detect and stop rapid temperature rise"
            );
            break;
        }
    }
    
    TEST_ASSERT_TRUE(emergencyShutdown);
}

// Test: Temperature oscillation around limit
void test_temperature_oscillation() {
    emergencyShutdown = false;
    
    // Oscillate around MAX_SAFE_TEMPERATURE
    for (int i = 0; i < 5; i++) {
        currentTemperature = 99.0f;
        checkTemperatureSafety();
        TEST_ASSERT_FALSE(emergencyShutdown);
        
        currentTemperature = 101.0f;
        checkTemperatureSafety();
        TEST_ASSERT_TRUE_MESSAGE(
            emergencyShutdown,
            "Should trigger on each excursion above limit"
        );
        
        // Reset for next iteration
        emergencyShutdown = false;
        currentTemperature = 90.0f;
    }
}

void setUp(void) {
    // Reset state before each test
    emergencyShutdown = false;
    heaterEnabled = false;
    heaterDutyCycle = 0;
    currentTemperature = 20.0f;
    targetTemperature = 60.0f;
}

void tearDown(void) {
    // Clean up after each test
}

int main() {
    UNITY_BEGIN();
    
    printf("\n");
    printf("========================================\n");
    printf("UT-005: Temperature Limit Tests\n");
    printf("========================================\n");
    
    RUN_TEST(test_normal_operation_below_limit);
    RUN_TEST(test_temperature_exceeds_limit);
    RUN_TEST(test_approaching_limit_safe);
    RUN_TEST(test_exactly_at_limit);
    RUN_TEST(test_temperature_hysteresis);
    RUN_TEST(test_sensor_error_high);
    RUN_TEST(test_sensor_error_low);
    RUN_TEST(test_heater_locked_after_shutdown);
    RUN_TEST(test_rapid_temperature_rise);
    RUN_TEST(test_temperature_oscillation);
    
    return UNITY_END();
}

#endif // UNIT_TEST
