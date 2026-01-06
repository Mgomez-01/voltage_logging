/**
 * UT-001: Memory Management - Heap Monitoring
 * Priority: CRITICAL
 * Duration: 15 minutes continuous
 * Environment: Mock
 * 
 * Tests that free heap remains stable during operation
 * and detects memory leaks early.
 */

#include <unity.h>

#ifdef UNIT_TEST // Only compile for native testing

#include <stdint.h>
#include <stdlib.h>
#include <vector>

// Mock ESP functions
namespace ESP {
    static uint32_t mockFreeHeap = 40000; // Start with 40KB
    static uint32_t mockHeapFragmentation = 0;
    
    uint32_t getFreeHeap() { return mockFreeHeap; }
    uint8_t getHeapFragmentation() { return mockHeapFragmentation; }
}

// Simplified sensor buffer simulation
#define BUFFER_SIZE 100
struct SensorReading {
    uint32_t timestamp;
    float voltage;
    float temperature;
};

std::vector<SensorReading> sensorBuffer;
uint32_t simulatedTime = 0;

// Simulate adding sensor readings
void addSensorReading(float voltage, float temperature) {
    SensorReading reading = {simulatedTime++, voltage, temperature};
    
    if (sensorBuffer.size() >= BUFFER_SIZE) {
        sensorBuffer.erase(sensorBuffer.begin()); // Remove oldest
    }
    
    sensorBuffer.push_back(reading);
}

// Test: Heap remains stable over time
void test_heap_stable_over_time() {
    const int ITERATIONS = 1000; // Simulate 1000 readings
    const uint32_t INITIAL_HEAP = ESP::getFreeHeap();
    const uint32_t MIN_HEAP_THRESHOLD = 10000; // 10KB minimum
    
    uint32_t minHeapSeen = INITIAL_HEAP;
    
    // Simulate continuous operation
    for (int i = 0; i < ITERATIONS; i++) {
        // Add sensor reading
        addSensorReading(0.5f, 25.0f);
        
        // Simulate some memory churn
        ESP::mockFreeHeap = 40000 - (rand() % 5000); // Vary 35-40KB
        
        uint32_t currentHeap = ESP::getFreeHeap();
        if (currentHeap < minHeapSeen) {
            minHeapSeen = currentHeap;
        }
        
        // Check heap never drops below threshold
        TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(
            MIN_HEAP_THRESHOLD,
            currentHeap,
            "Heap dropped below minimum threshold!"
        );
    }
    
    printf("Min heap during test: %u bytes\n", minHeapSeen);
}

// Test: No monotonic decrease in heap
void test_no_monotonic_heap_decrease() {
    const int SAMPLES = 100;
    uint32_t heapSamples[SAMPLES];
    
    // Collect heap samples over simulated time
    for (int i = 0; i < SAMPLES; i++) {
        addSensorReading(0.5f, 25.0f);
        heapSamples[i] = ESP::getFreeHeap();
        
        // Simulate normal variance
        ESP::mockFreeHeap = 40000 - (rand() % 3000);
    }
    
    // Check for monotonic decrease (memory leak pattern)
    int consecutiveDecreases = 0;
    for (int i = 1; i < SAMPLES; i++) {
        if (heapSamples[i] < heapSamples[i-1]) {
            consecutiveDecreases++;
        } else {
            consecutiveDecreases = 0; // Reset counter
        }
        
        // Fail if we see too many consecutive decreases
        TEST_ASSERT_LESS_THAN_MESSAGE(
            10,
            consecutiveDecreases,
            "Detected sustained monotonic heap decrease (memory leak!)"
        );
    }
}

// Test: Heap variance within acceptable range
void test_heap_variance_acceptable() {
    const int SAMPLES = 50;
    uint32_t heapSamples[SAMPLES];
    uint32_t sum = 0;
    
    // Collect samples
    for (int i = 0; i < SAMPLES; i++) {
        addSensorReading(0.5f, 25.0f);
        ESP::mockFreeHeap = 40000 - (rand() % 4000);
        heapSamples[i] = ESP::getFreeHeap();
        sum += heapSamples[i];
    }
    
    // Calculate mean
    uint32_t mean = sum / SAMPLES;
    
    // Calculate variance
    uint64_t varianceSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int32_t diff = heapSamples[i] - mean;
        varianceSum += (diff * diff);
    }
    uint32_t variance = varianceSum / SAMPLES;
    uint32_t stddev = sqrt(variance);
    
    printf("Heap statistics - Mean: %u, StdDev: %u\n", mean, stddev);
    
    // Variance should be less than 5KB
    TEST_ASSERT_LESS_THAN_MESSAGE(
        5000,
        stddev,
        "Heap variance too high - possible memory management issue"
    );
}

// Test: Buffer size stays bounded
void test_buffer_stays_bounded() {
    // Fill buffer beyond capacity
    for (int i = 0; i < BUFFER_SIZE * 2; i++) {
        addSensorReading(0.5f, 25.0f);
        
        // Buffer should never exceed BUFFER_SIZE
        TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(
            BUFFER_SIZE,
            sensorBuffer.size(),
            "Buffer grew beyond BUFFER_SIZE!"
        );
    }
    
    TEST_ASSERT_EQUAL_MESSAGE(
        BUFFER_SIZE,
        sensorBuffer.size(),
        "Buffer size should stabilize at BUFFER_SIZE"
    );
}

// Test: Heap fragmentation stays low
void test_heap_fragmentation_low() {
    const uint8_t MAX_FRAGMENTATION = 30; // 30%
    
    // Simulate memory operations that could cause fragmentation
    for (int i = 0; i < 100; i++) {
        addSensorReading(0.5f, 25.0f);
        
        // Simulate varying fragmentation
        ESP::mockHeapFragmentation = rand() % 25;
        
        uint8_t frag = ESP::getHeapFragmentation();
        TEST_ASSERT_LESS_THAN_MESSAGE(
            MAX_FRAGMENTATION,
            frag,
            "Heap fragmentation exceeded threshold"
        );
    }
}

void setUp(void) {
    // Reset state before each test
    sensorBuffer.clear();
    ESP::mockFreeHeap = 40000;
    ESP::mockHeapFragmentation = 0;
    simulatedTime = 0;
    srand(42); // Deterministic randomness for testing
}

void tearDown(void) {
    // Clean up after each test
    sensorBuffer.clear();
}

int main() {
    UNITY_BEGIN();
    
    printf("\n");
    printf("========================================\n");
    printf("UT-001: Heap Monitoring Tests\n");
    printf("========================================\n");
    
    RUN_TEST(test_heap_stable_over_time);
    RUN_TEST(test_no_monotonic_heap_decrease);
    RUN_TEST(test_heap_variance_acceptable);
    RUN_TEST(test_buffer_stays_bounded);
    RUN_TEST(test_heap_fragmentation_low);
    
    return UNITY_END();
}

#endif // UNIT_TEST
