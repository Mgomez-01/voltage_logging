#include "heater_controller.h"
#include "safety_system.h"
#include "sensor_manager.h"  // For direct sensor access

// Heater control pins and configuration
const int HEATER_PWM_PIN = 16; // GPIO16 (D0) for PWM control of MOSFET
bool heaterEnabled = false;
float heaterDutyCycle = 0.0; // Current PWM duty cycle (0-100%)
unsigned long heaterStartTime = 0; // Time when heater was enabled
const unsigned long MAX_HEATER_TIME = 3600000UL; // 1 h continuous-heating safety limit (was 30 min)
const float MAX_SAFE_TEMPERATURE = 130.0; // Maximum safe temperature in °C

// PWM configuration
// A low frequency suits a heater (fewer MOSFET switching losses).
// The ESP8266 core docs list 100 Hz as the analogWriteFreq() minimum, but the
// default PWM engine in core 3.1.2 uses the requested 32 Hz directly (only the
// fallback waveform path clamps to 100 Hz).
const int PWM_FREQUENCY = 32; // PWM frequency in Hz
const int PWM_RANGE = 1000;   // 0-1000 range for 0.1% resolution

// PID Controller variables
float targetTemperature = 45.0; // Default target temperature
bool pidEnabled = false;
// Default gains: interim values from a model of the sheet fitted to the
// 2026-10-26 run (about 2 °C per % power, 60 s time constant, 3 s delay),
// chosen for no overshoot. Run the /tune page to measure the real fixture.
// Gains saved from /tune (/pid/save) are loaded from LittleFS at boot and
// replace these. (Before the anti-windup fix the gains were 2.2 / 0.25 / 0.15;
// with the old integral cap they overshoot ~10 °C now.)
float pidKp = 2.3;    // Proportional gain (% per °C)   } active gains, for the
float pidKi = 0.045;  // Integral gain (% per °C·s)      } sensor selected in
float pidKd = 0.0;    // Derivative gain (% per °C/s)    } controlSensor

// Which sensor the PID controls. Each sensor keeps its own gains: the sample
// (AUX) responds much more slowly than the sheet, so it needs gentler gains.
// AUX defaults are from a model fitted to the 2026-10-26 run (sample settles
// ~63 % of the way to the sheet temperature with an ~80 s lag), chosen for no
// overshoot. Tune them on /tune with AUX selected.
ControlSensor controlSensor = CONTROL_PID_SENSOR;
float sensorGains[2][3] = {
  {2.3, 0.045, 0.0},   // CONTROL_PID_SENSOR (sheet)
  {1.0, 0.008, 0.0}    // CONTROL_AUX_SENSOR (sample)
};

// Sheet limit while the PID controls on the AUX sensor. The sheet runs ~25 °C
// above the sample near 70 °C, so it needs its own cap. Heater power is tapered
// to 0 over the last SHEET_LIMIT_BAND degrees below the limit.
float sheetLimit = 120.0;
const float SHEET_LIMIT_BAND = 5.0;
const float MIN_SHEET_LIMIT = 40.0;
const float MAX_SHEET_LIMIT = 125.0;   // MAX_SAFE_TEMPERATURE - 5
bool sheetLimited = false;             // Last PID update was held back by the sheet limit
char pidFaultMessage[64] = "";         // Why the PID last stopped itself (empty = no fault)
float pidOutput = 0.0;
float pidError = 0.0;
float pidLastError = 0.0;
float pidIntegral = 0.0;
float pidLastInput = NAN;  // Last measured temperature (derivative on measurement); NAN = none yet
const unsigned long PID_INTERVAL = 500; // PID update interval in ms
unsigned long lastPIDUpdate = 0;

// Debug configuration for this file only (the DEBUG_* defines in main.cpp
// do not reach other .cpp files)
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
    
    // Check for maximum runtime (MAX_HEATER_TIME = 1 h continuous heating)
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

// Clear the PID's memory (integral, last error/temperature). Call whenever the
// PID is (re)enabled or its gains change.
void rememberActiveGains() {
  sensorGains[controlSensor][0] = pidKp;
  sensorGains[controlSensor][1] = pidKi;
  sensorGains[controlSensor][2] = pidKd;
}

void setControlSensor(ControlSensor sensor) {
  if (sensor != CONTROL_PID_SENSOR && sensor != CONTROL_AUX_SENSOR) return;
  rememberActiveGains();                 // Keep the old sensor's gains
  controlSensor = sensor;
  pidKp = sensorGains[sensor][0];
  pidKi = sensorGains[sensor][1];
  pidKd = sensorGains[sensor][2];
  resetPIDState();                       // Start the integral fresh on the new sensor
  sheetLimited = false;
  Serial.print(F("PID: now controlling on the "));
  Serial.println(sensor == CONTROL_AUX_SENSOR ? F("AUX Temp Sensor (sample)") : F("PID Temp Sensor (sheet)"));
}

const char* controlSensorName(ControlSensor sensor) {
  return sensor == CONTROL_AUX_SENSOR ? "aux" : "pid";
}

static void pidFault(const char* message) {
  strncpy(pidFaultMessage, message, sizeof(pidFaultMessage) - 1);
  pidFaultMessage[sizeof(pidFaultMessage) - 1] = '\0';
  pidEnabled = false;
  heaterEnabled = false;
  setHeaterPower(0.0);
  Serial.print(F("PID: stopped, heater off - "));
  Serial.println(pidFaultMessage);
}

void resetPIDState() {
  pidFaultMessage[0] = '\0';
  pidIntegral = 0;
  pidLastError = 0;
  pidLastInput = NAN;
  pidError = 0;
}

void updatePIDController() {
  if (!pidEnabled) return;
  
  #if DEBUG_PID
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {  // Log every 5 seconds that PID is active
    Serial.println("PID: Controller active and running");
    lastDebug = millis();
  }
  #endif
  
  // Most recent readings of both sensors: the latest logged row, or direct
  // reads when the buffer is empty (logging off, or just written to the card)
  float sheetTemp = NAN;   // PID Temp Sensor (sheet)
  float auxTemp = NAN;     // AUX Temp Sensor (sample)
  int idx = -1;
  
  if (bufferIndex > 0) {
    idx = bufferIndex - 1;
    sheetTemp = readings[idx].temperature;
    auxTemp = convertAuxThermistorToTemperature(readings[idx].auxThermistorAdc);
  } else {
    selectMuxChannel(THERMISTOR_CHANNEL);
    delayMicroseconds(100);  // Small delay for stability
    sheetTemp = convertThermistorToTemperature(analogRead(ADC_PIN));
    if (controlSensor == CONTROL_AUX_SENSOR) {
      selectMuxChannel(AUX_THERMISTOR_CHANNEL);
      delayMicroseconds(100);
      auxTemp = convertAuxThermistorToTemperature(analogRead(ADC_PIN));
    }
    idx = -1;  // Indicate direct reading
  }
  bool sheetValid = !(isnan(sheetTemp) || sheetTemp < -50 || sheetTemp > 200);
  bool auxValid = !(isnan(auxTemp) || auxTemp < -50 || auxTemp > 200);
  
  float currentTemp;
  if (controlSensor == CONTROL_AUX_SENSOR) {
    // Controlling on the sample. Both sensors must work: AUX is the feedback,
    // and the sheet reading enforces the sheet limit. The safety system only
    // watches the PID Temp Sensor, so a failed AUX sensor stops the PID here.
    if (!auxValid) { pidFault("AUX Temp Sensor fault while controlling on AUX"); return; }
    if (!sheetValid) { pidFault("PID Temp Sensor (sheet) fault: sheet limit can't be enforced"); return; }
    currentTemp = auxTemp;
  } else {
    // Controlling on the sheet: skip an invalid reading (the safety system
    // shuts down on a sensor fault while the heater is on)
    if (!sheetValid) {
      #if DEBUG_PID
      Serial.print("PID: Invalid temperature reading: ");
      Serial.println(sheetTemp);
      #endif
      return;
    }
    currentTemp = sheetTemp;
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
  const float dt = PID_INTERVAL / 1000.0;
  
  // Proportional term
  float proportional = pidKp * pidError;
  
  // Derivative on measurement: -Kd * dT/dt. Same as Kd * d(error)/dt while the
  // setpoint is constant, but a setpoint step doesn't cause an output spike.
  float derivative = 0;
  if (!isnan(pidLastInput)) {
    derivative = -pidKd * (currentTemp - pidLastInput) / dt;
  }
  pidLastInput = currentTemp;
  pidLastError = pidError;
  
  // Integral with anti-windup:
  //  1. Conditional integration: don't integrate further while the output is
  //     already saturated in the direction the error is pushing.
  //  2. Clamp the integral term itself to the output range (0-100 %), so it can
  //     supply whatever steady power a setpoint needs. (The old +/-100 clamp on
  //     the raw integral capped it at 25 % output with Ki = 0.25, which left the
  //     sheet several degrees short of high setpoints.)
  // Upper output limit: 100 %, or less when controlling on AUX and the sheet
  // is within SHEET_LIMIT_BAND of the sheet limit (0 % at the limit)
  float highLimit = 100.0;
  if (controlSensor == CONTROL_AUX_SENSOR) {
    float cap = (sheetLimit - sheetTemp) * 100.0 / SHEET_LIMIT_BAND;
    if (cap < 0) cap = 0;
    if (cap < highLimit) highLimit = cap;
  }
  float candidate = pidIntegral + pidError * dt;
  float unsaturated = proportional + pidKi * candidate + derivative;
  // Report "sheet limited" for 5 s after the limit last held the output back:
  // near the limit, sheet sensor noise switches the taper on and off from one
  // update to the next
  static unsigned long lastLimitedMs = 0;
  bool limitedNow = (controlSensor == CONTROL_AUX_SENSOR) && highLimit < 100.0 &&
                    unsaturated >= highLimit - 0.5;
  if (limitedNow) lastLimitedMs = millis();
  sheetLimited = (controlSensor == CONTROL_AUX_SENSOR) && lastLimitedMs != 0 &&
                 millis() - lastLimitedMs < 5000;
  bool saturatedHigh = unsaturated > highLimit && pidError > 0;
  bool saturatedLow = unsaturated < 0 && pidError < 0;
  if (!saturatedHigh && !saturatedLow) {
    pidIntegral = candidate;
  }
  if (pidKi > 0) {
    float maxIntegral = 100.0 / pidKi;
    if (pidIntegral > maxIntegral) pidIntegral = maxIntegral;
    if (pidIntegral < -maxIntegral) pidIntegral = -maxIntegral;
  }
  float integral = pidKi * pidIntegral;
  
  // Compute PID output (0-100%)
  pidOutput = proportional + integral + derivative;
  
  // Clamp output to 0-100 % (and the sheet limit when controlling on AUX)
  if (pidOutput > highLimit) pidOutput = highLimit;
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
  // Always show PID updates when active
  Serial.print("PID: Target=");
  Serial.print(targetTemperature);
  Serial.print("°C, Current=");
  Serial.print(currentTemp);
  Serial.print("°C, Error=");
  Serial.print(pidError, 2);
  Serial.print(", Output=");
  Serial.print(pidOutput, 1);
  Serial.print("%, PWM=");
  Serial.print(heaterDutyCycle, 1);
  Serial.print("%, BufIdx=");
  Serial.print(bufferIndex);
  Serial.print(", ReadIdx=");
  Serial.print(idx);
  Serial.print(", Logging=");
  Serial.println(dataLoggingEnabled ? "ON" : "OFF");
  #endif
}
