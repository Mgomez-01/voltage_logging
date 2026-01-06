/**
 * UT-004: Safety Systems - Hardware Watchdog
 * Priority: CRITICAL
 * Duration: 30 seconds
 * Environment: Mock
 * 
 * Tests that the watchdog timer triggers emergency shutdown
 * when the main loop is blocked or stalled.
 */

#include <unity.h>

#ifdef UNIT_TEST

#include <stdint.h>
#include <stdbool.h>

// System state variables
bool emergencyShutdown = false;
bool heaterEnabled = false;
bool systemAlive = false;
uint32_t lastWatchdogFeed = 0;
uint32_t currentTime = 0;

// Configuration
#define WATCHDOG_TIMEOUT 10000  // 10 seconds in ms

// Watchdog functions
void feedWatchdog() {
    systemAlive = true;
    lastWatchdogFeed = currentTime;
}

void checkWatchdog() {
    if (currentTime - lastWatchdogFeed > WATCHDOG_TIMEOUT) {
        // Watchdog timeout - trigger emergency shutdown
        emergencyShutdown = true;
        heaterEnabled = false;
        systemAlive = false;
    }
}

void resetWatchdog() {
    lastWatchdogFeed = 0;
    systemAlive = false;
}

// Heater control
void setHeaterPower(uint8_t dutyCycle) {
    if (emergencyShutdown) {
        heaterEnabled = false;
        return;
    }
    heaterEnabled = (dutyCycle > 0);
}

// Test: Watchdog triggers on timeout
void test_watchdog_triggers_on_timeout() {
    // Initialize system
    emergencyShutdown = false;
    heaterEnabled = true;
    currentTime = 0;
    feedWatchdog();
    
    // Simulate main loop running normally for 5 seconds
    for (int i = 0; i < 50; i++) {
        currentTime += 100; // Advance 100ms
        feedWatchdog();
        checkWatchdog();
        
        TEST_ASSERT_FALSE_MESSAGE(
            emergencyShutdown,
            "Emergency shutdown triggered prematurely"
        );
        TEST_ASSERT_TRUE_MESSAGE(
            systemAlive,
            "System should be alive when watchdog is fed"
        );
    }
    
    // Now simulate a freeze - stop feeding watchdog
    printf("Simulating main loop freeze...\n");
    currentTime += WATCHDOG_TIMEOUT + 1000; // Exceed timeout
    checkWatchdog();
    
    // Watchdog should have triggered
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Emergency shutdown should be triggered after watchdog timeout"
    );
    TEST_ASSERT_FALSE_MESSAGE(
        heaterEnabled,
        "Heater should be disabled after watchdog timeout"
    );
    TEST_ASSERT_FALSE_MESSAGE(
        systemAlive,
        "System should not be alive after timeout"
    );
}

// Test: Watchdog fed regularly keeps system running
void test_regular_feeding_prevents_timeout() {
    emergencyShutdown = false;
    heaterEnabled = true;
    currentTime = 0;
    feedWatchdog();
    
    // Feed watchdog every second for 30 seconds
    for (int i = 0; i < 30; i++) {
        currentTime += 1000; // Advance 1 second
        feedWatchdog();
        checkWatchdog();
        
        TEST_ASSERT_FALSE_MESSAGE(
            emergencyShutdown,
            "Emergency shutdown should not trigger with regular feeding"
        );
    }
}

// Test: Missed single feed doesn't trigger (grace period)
void test_single_missed_feed_has_grace() {
    emergencyShutdown = false;
    currentTime = 0;
    feedWatchdog();
    
    // Feed normally for a bit
    for (int i = 0; i < 5; i++) {
        currentTime += 1000;
        feedWatchdog();
        checkWatchdog();
    }
    
    // Miss one feed cycle (but still within timeout)
    currentTime += 5000; // 5 seconds later (total 10s from last feed = timeout)
    checkWatchdog();
    
    // Should NOT have triggered yet (exactly at timeout)
    // Feed it just in time
    feedWatchdog();
    checkWatchdog();
    
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should have grace period for missed feeds"
    );
}

// Test: Emergency shutdown latches heater off
void test_emergency_shutdown_latches_heater() {
    emergencyShutdown = false;
    heaterEnabled = true;
    currentTime = 0;
    feedWatchdog();
    
    // Trigger watchdog timeout
    currentTime += WATCHDOG_TIMEOUT + 1000;
    checkWatchdog();
    
    TEST_ASSERT_TRUE(emergencyShutdown);
    TEST_ASSERT_FALSE(heaterEnabled);
    
    // Try to turn heater back on - should fail
    setHeaterPower(100);
    
    TEST_ASSERT_FALSE_MESSAGE(
        heaterEnabled,
        "Heater should remain off after emergency shutdown"
    );
}

// Test: Watchdog timing accuracy
void test_watchdog_timing_accuracy() {
    emergencyShutdown = false;
    currentTime = 0;
    feedWatchdog();
    
    // Advance just before timeout
    currentTime += WATCHDOG_TIMEOUT - 100; // 9.9 seconds
    checkWatchdog();
    
    TEST_ASSERT_FALSE_MESSAGE(
        emergencyShutdown,
        "Should not trigger before timeout"
    );
    
    // Advance past timeout
    currentTime += 200; // Now at 10.1 seconds
    checkWatchdog();
    
    TEST_ASSERT_TRUE_MESSAGE(
        emergencyShutdown,
        "Should trigger after timeout"
    );
}

// Test: Multiple timeout cycles
void test_multiple_timeout_cycles() {
    // First cycle
    emergencyShutdown = false;
    heaterEnabled = true;
    currentTime = 0;
    feedWatchdog();
    
    currentTime += WATCHDOG_TIMEOUT + 1000;
    checkWatchdog();
    TEST_ASSERT_TRUE(emergencyShutdown);
    
    // Reset system (simulating manual reset)
    emergencyShutdown = false;
    heaterEnabled = false;
    currentTime = 0;
    resetWatchdog();
    feedWatchdog();
    
    // Second cycle - should work normally
    for (int i = 0; i < 10; i++) {
        currentTime += 1000;
        feedWatchdog();
        checkWatchdog();
        TEST_ASSERT_FALSE(emergencyShutdown);
    }
}

void setUp(void) {
    // Reset state before each test
    emergencyShutdown = false;
    heaterEnabled = false;
    systemAlive = false;
    lastWatchdogFeed = 0;
    currentTime = 0;
}

void tearDown(void) {
    // Clean up after each test
}

int main() {
    UNITY_BEGIN();
    
    printf("\n");
    printf("========================================\n");
    printf("UT-004: Watchdog Timer Tests\n");
    printf("========================================\n");
    
    RUN_TEST(test_watchdog_triggers_on_timeout);
    RUN_TEST(test_regular_feeding_prevents_timeout);
    RUN_TEST(test_single_missed_feed_has_grace);
    RUN_TEST(test_emergency_shutdown_latches_heater);
    RUN_TEST(test_watchdog_timing_accuracy);
    RUN_TEST(test_multiple_timeout_cycles);
    
    return UNITY_END();
}

#endif // UNIT_TEST
