#include "heater_controller.h"
#include "safety_system.h"
#include "sensor_manager.h"  // For direct sensor access

// Heater control pins and configuration
const int HEATER_PWM_PIN = 16; // GPIO16 (D0) for PWM control of MOSFET
bool heaterEnabled = false;
float heaterDutyCycle = 0.0; // Current PWM duty cycle (0-100%)
unsigned long heaterStartTime = 0; // Time when heater was enabled
const unsigned long MAX_HEATER_TIME = 600000; // 10 min safety timeout
const float MAX_SAFE_TEMPERATURE = 120.0; // Maximum safe temperature in °C

// PWM configuration
// Lower frequency (5-10 Hz) is better for heater control to reduce MOSFET switching losses
// and provide more accurate average power delivery
const int PWM_FREQUENCY = 10; // 10 Hz PWM frequency
const int PWM_RANGE = 1000;   // 0-1000 range for 0.1% resolution

// PID Controller variables
float targetTemperature = 65.0; // Default target temperature
bool pidEnabled = false;
float pidKp = 2.2;  // Proportional gain
float pidKi = 0.25;  // Integral gain
float pidKd = 0.15;  // Derivative gain - these values have been tested and work well at 50°C. 
float pidOutput = 0.0;
float pidError = 0.0;
float pidLastError = 0.0;
float pidIntegral = 0.0;
const unsigned long PID_INTERVAL = 500; // PID update interval in ms
unsigned long lastPIDUpdate = 0;

// Debug configuration (accessing from main file defines)
#define DEBUG_HEATER 1
#define DEBUG_PID 1

void initializeHeaterPWM() {
  Serial.print("Initializing heater PWM control... ");
  
  // Configure PWM frequency and range
  analogWriteFreq(PWM_FREQUENCY);
  analogWriteRange(PWM_RANGE);
  
  // Set pin as output and initialize to 0% duty cycle
  pinMode(HEATER_PWM_PIN, OUTPUT);
  analogWrite(HEATER_PWM_PIN, 0);
  heaterDutyCycle = 0.0;
  
  Serial.println("OK");
  Serial.print("PWM pin: GPIO");
  Serial.print(HEATER_PWM_PIN);
  Serial.print(" (D0), Frequency: ");
  Serial.print(PWM_FREQUENCY);
  Serial.print(" Hz, Range: 0-");
  Serial.println(PWM_RANGE);
  Serial.println("PWM control via MOSFET (AOD4144) for smooth heater power regulation");
}

void setHeaterPower(float dutyCyclePercent) {
  // Clamp duty cycle to 0-100%
  if (dutyCyclePercent < 0) dutyCyclePercent = 0;
  if (dutyCyclePercent > 100) dutyCyclePercent = 100;
  
  // Convert percentage to PWM value (0-1000)
  int pwmValue = (int)((dutyCyclePercent / 100.0) * PWM_RANGE);
  
  // Update PWM output
  analogWrite(HEATER_PWM_PIN, pwmValue);
  
  // Track state changes for logging
  bool wasActive = (heaterDutyCycle > 0);
  bool nowActive = (dutyCyclePercent > 0);
  
  heaterDutyCycle = dutyCyclePercent;
  
  #if DEBUG_HEATER
  // Log significant changes
  if (!wasActive && nowActive) {
    Serial.print("HEATER: PWM enabled at ");
    Serial.print(dutyCyclePercent, 1);
    Serial.print("% (PWM value: ");
    Serial.print(pwmValue);
    Serial.println(")");
    heaterStartTime = millis();
  } else if (wasActive && !nowActive) {
    Serial.print("HEATER: PWM disabled (was active for ");
    Serial.print((millis() - heaterStartTime) / 1000);
    Serial.println(" seconds)");
  } else if (abs(heaterDutyCycle - dutyCyclePercent) > 5.0) {
    // Log if duty cycle changes by more than 5%
    Serial.print("HEATER: Power adjusted to ");
    Serial.print(dutyCyclePercent, 1);
    Serial.println("%");
  }
  #endif
}

void checkHeaterSafety() {
  // This function needs access to the safety system variables and buffer
  // The actual safety logic will be called from main loop
  if (!emergencyShutdown && (bufferIndex > 0 || bufferFull)) {
    int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
    float currentTemp = readings[idx].temperature;
    
    // Check for over-temperature
    if (currentTemp > MAX_SAFE_TEMPERATURE) {
      Serial.print("SAFETY: Over-temperature shutdown! Temp=");
      Serial.print(currentTemp);
      Serial.println("°C");
      emergencyShutdown = true;
    }
    
    // Check for sensor failure (only if heater is active)
    if (heaterDutyCycle > 0 && (isnan(currentTemp) || currentTemp < -50 || currentTemp > 200)) {
      Serial.println("SAFETY: Temperature sensor failure - emergency shutdown");
      emergencyShutdown = true;
    }
    
    // Check for maximum runtime (10 minutes continuous heating)
    if (heaterDutyCycle > 0 && (millis() - heaterStartTime) > MAX_HEATER_TIME) {
      Serial.println("SAFETY: Maximum heater runtime exceeded - emergency shutdown");
      emergencyShutdown = true;
    }
  }
  
  // Execute emergency shutdown if triggered
  if (emergencyShutdown) {
    analogWrite(HEATER_PWM_PIN, 0);  // Turn off PWM
    heaterDutyCycle = 0.0;
    heaterEnabled = false;
    pidEnabled = false;
    Serial.println("SAFETY: All heater control disabled");
  }
}

void updatePIDController() {
  if (!pidEnabled) return;
  
  // Find the most recent valid temperature reading
  float currentTemp = NAN;
  int idx = -1;
  
  if (bufferIndex > 0) {
    // Use the most recent reading in the buffer
    idx = bufferIndex - 1;
    currentTemp = readings[idx].temperature;
  } else {
    // Buffer was reset or no readings yet - try to get current temperature directly
    selectMuxChannel(THERMISTOR_CHANNEL);
    delayMicroseconds(100);  // Small delay for stability
    int adcValue = analogRead(ADC_PIN);
    currentTemp = convertThermistorToTemperature(adcValue);
    idx = -1;  // Indicate direct reading
    
    #if DEBUG_PID
    Serial.print("PID: Using direct sensor reading, ADC=");
    Serial.print(adcValue);
    Serial.print(", Temp=");
    Serial.println(currentTemp);
    #endif
  }
  
  // Validate temperature reading before using it for PID
  if (isnan(currentTemp) || currentTemp < -50 || currentTemp > 200) {
    #if DEBUG_PID
    Serial.print("PID: Invalid temperature reading: ");
    Serial.println(currentTemp);
    #endif
    return;  // Skip PID update with invalid temperature
  }
  
  #if DEBUG_PID
  Serial.print("PID DEBUG: enabled=");
  Serial.print(pidEnabled);
  Serial.print(", bufferIndex=");
  Serial.print(bufferIndex);
  Serial.print(", totalReadings=");
  Serial.print(totalReadings);
  Serial.print(", bufferFull=");
  Serial.print(bufferFull);
  if (idx >= 0) {
    Serial.print(", idx=");
    Serial.print(idx);
  } else {
    Serial.print(", direct_sensor");
  }
  Serial.print(", temp=");
  Serial.println(currentTemp);
  #endif
  
  pidError = targetTemperature - currentTemp;
  
  // Calculate PID terms
  float proportional = pidKp * pidError;
  
  // Update integral with anti-windup
  pidIntegral += pidError * (PID_INTERVAL / 1000.0);
  // Anti-windup: limit integral to prevent excessive buildup
  if (pidIntegral > 100) pidIntegral = 100;
  if (pidIntegral < -100) pidIntegral = -100;
  float integral = pidKi * pidIntegral;
  
  // Calculate derivative
  float derivative = pidKd * (pidError - pidLastError) / (PID_INTERVAL / 1000.0);
  pidLastError = pidError;
  
  // Compute PID output (0-100%)
  pidOutput = proportional + integral + derivative;
  
  // Clamp output to 0-100%
  if (pidOutput > 100) pidOutput = 100;
  if (pidOutput < 0) pidOutput = 0;
  
  // Apply PID output directly to heater PWM
  // No hysteresis needed with PWM - smooth continuous control
  setHeaterPower(pidOutput);
  
  #if DEBUG_PID
  // Detailed PID debug output every 10 updates
  static int debugCounter = 0;
  if (++debugCounter >= 10) {
    debugCounter = 0;
    Serial.print("PID DETAIL: P=");
    Serial.print(proportional, 2);
    Serial.print(", I=");
    Serial.print(integral, 2);
    Serial.print(", D=");
    Serial.print(derivative, 2);
    Serial.print(", Sum=");
    Serial.print(pidOutput, 2);
    Serial.println("%");
  }
  #endif
  #if DEBUG_PID
  Serial.print("PID: Target=");
  Serial.print(targetTemperature);
  Serial.print("°C, Current=");
  Serial.print(currentTemp);
  Serial.print("°C, Error=");
  Serial.print(pidError);
  Serial.print(", Output=");
  Serial.print(pidOutput, 1);
  Serial.print("%, PWM=");
  Serial.print(heaterDutyCycle, 1);
  Serial.print("%, BufIdx=");
  Serial.print(bufferIndex);
  Serial.print(", ReadIdx=");
  Serial.println(idx);
  #endif
}
