#ifndef TUNING_H
#define TUNING_H

#include <Arduino.h>

// PID tuning tests, run by the firmware and shown on the /tune page.
//
// Power step test (open loop, PID off):
//   1. Hold the heater at power1 until the PID Temp Sensor settles.
//   2. Step to power2 and record the response until it settles again.
//   3. Heater off.
//   The page fits delay / time constant / gain to step 2 and suggests gains.
//
// Setpoint test (closed loop, PID on the PID Temp Sensor):
//   Run the PID with the given gains through up to 4 setpoints, holding each
//   for a set time, then heater off. The page reports overshoot and settling.
//
// Both tests record the PID and AUX temperatures and heater power into a RAM
// buffer (independent of SD logging) and stop the heater if anything goes
// wrong: sensor fault, sheet above TUNE_MAX_TEMP, emergency shutdown, total
// time over TUNE_MAX_TOTAL_MS, or heater/PID changed from the main page.

enum TuneState : uint8_t {
  TUNE_IDLE = 0,
  TUNE_STEP_SETTLE,    // Power step test: holding power1, waiting to settle
  TUNE_STEP_RESPONSE,  // Power step test: at power2, recording the response
  TUNE_SETPOINT,       // Setpoint test running
  TUNE_DONE,           // Finished normally (heater off)
  TUNE_ABORTED         // Stopped early (heater off); see tuneMessage
};

enum TuneTest : uint8_t { TEST_NONE = 0, TEST_STEP, TEST_SETPOINT };

// One recorded sample (8 bytes)
struct TuneSample {
  uint16_t t;       // Time since test start, 0.1 s units
  int16_t pidT;     // PID Temp Sensor, 0.01 °C units (-32768 = fault)
  int16_t auxT;     // AUX Temp Sensor, 0.01 °C units (-32768 = fault / none)
  uint16_t duty;    // Heater power, 0.1 % units
};

extern const uint16_t TUNE_MAX_SAMPLES;
extern const float TUNE_MAX_TEMP;
extern const unsigned long TUNE_MAX_TOTAL_MS;
extern const uint8_t TUNE_MAX_SETPOINTS;

extern TuneState tuneState;
extern TuneTest tuneTest;

bool tuningActive();   // A test is running (heater under tuning control)
void tuningLoop();     // Call from loop(); samples and advances the test
void stopTuning(const char* reason);  // Heater off, test aborted (no-op if idle)

void setupTuningRoutes();  // /tune page and /tune/* endpoints, /pid/save
void loadSavedPidGains();  // Read /pid.json from LittleFS (after LittleFS.begin())

#endif // TUNING_H
