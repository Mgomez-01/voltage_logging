#ifndef PROGRAM_H
#define PROGRAM_H

#include <Arduino.h>

// Setpoint program (experiments), run by the firmware and controlled from the
// /program page.
//
// A program is a list of up to PROGRAM_MAX_STEPS setpoints, each held for a
// set time. Starting it turns on SD logging (with the current logging
// settings, e.g. averaging), sets the first setpoint and turns the PID on with
// the selected control sensor and its gains. Each step lasts exactly its hold
// time, counted from when the step starts; there is no wait for the
// temperature to settle. When the last step ends the program either turns the
// heater off or keeps holding the last setpoint, and optionally stops logging.
//
// Every log row records the running step number in the "step" column
// (0 = no program running).
//
// The program stops (heater off, logging left running) if: the Stop button is
// pressed, the heater/PID/setpoint/control sensor is changed from the main
// page, the PID stops itself (sensor fault), logging stops (from the page or
// the low-memory auto-stop), or an emergency shutdown happens. PID gains can
// be adjusted during a program.

extern const uint8_t PROGRAM_MAX_STEPS;

bool programRunning();          // Steps are running (not finished / holding the last setpoint)
bool programActive();           // Running, or holding the last setpoint after the last step
uint8_t programStepNumber();    // 1-based step being run or held (0 if no program is active)
void programLoop();             // Call from loop(): advances the steps
void stopProgram(const char* reason);  // Heater off, program stopped (no-op if idle)
void setupProgramRoutes();      // /program page and /program/* endpoints

#endif // PROGRAM_H
