#include "program.h"
#include "heater_controller.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "web_interface.h"
#include "tuning.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

const uint8_t PROGRAM_MAX_STEPS = 100;
static const float PROGRAM_MIN_TEMP = 20.0;
static const float PROGRAM_MAX_TEMP = 120.0;     // MAX_SAFE_TEMPERATURE - 10
static const float PROGRAM_MIN_HOLD_MIN = 0.5;
static const float PROGRAM_MAX_HOLD_MIN = 360.0;   // Per step; the total must also fit MAX_HEATER_TIME
static const char* PROGRAM_FILE = "/program.json";  // Last program started (for the page to reload)

enum ProgramState : uint8_t {
  PROGRAM_IDLE = 0,
  PROGRAM_RUNNING,     // Stepping through the list
  PROGRAM_HOLDING,     // Finished, holding the last setpoint ("hold" end action)
  PROGRAM_DONE,        // Finished, heater off
  PROGRAM_STOPPED      // Stopped early, heater off; see programMessage
};

struct ProgramStep {
  float temp;          // Setpoint (°C), for the selected control sensor
  uint16_t holdS;      // Hold time (s)
};

static ProgramStep steps[100];          // 100 x 8 bytes
static uint8_t stepCount = 0;
static float defaultHoldMin = 10;       // Default hold of the current/last program (min)
static uint16_t listId = 0;             // Changes each time a program starts (page reloads the list)
static uint8_t stepIndex = 0;           // 0-based step being run
static ProgramState state = PROGRAM_IDLE;
static unsigned long programStartMs = 0;
static unsigned long stepStartMs = 0;
static unsigned long finishedMs = 0;
static bool holdLastAtEnd = false;      // End action: keep holding the last setpoint
static bool stopLoggingAtEnd = true;    // End action: stop SD logging when the steps finish
static char programMessage[112] = "No program running";

bool programRunning() { return state == PROGRAM_RUNNING; }
bool programActive() { return state == PROGRAM_RUNNING || state == PROGRAM_HOLDING; }
uint8_t programStepNumber() { return programActive() ? stepIndex + 1 : 0; }

static void setMessage(const char* m) {
  strncpy(programMessage, m, sizeof(programMessage) - 1);
  programMessage[sizeof(programMessage) - 1] = '\0';
  Serial.print(F("PROGRAM: "));
  Serial.println(programMessage);
}

static void heaterOff() {
  pidEnabled = false;
  heaterEnabled = false;
  setHeaterPower(0.0);
}

void stopProgram(const char* reason) {
  if (!programActive()) return;
  heaterOff();
  state = PROGRAM_STOPPED;
  finishedMs = millis();
  char msg[112];
  snprintf_P(msg, sizeof(msg), PSTR("Stopped at step %u of %u: %s. Heater off, logging left as it was."),
             stepIndex + 1, stepCount, reason);
  setMessage(msg);
}

static void startStep(uint8_t i) {
  stepIndex = i;
  stepStartMs = millis();
  targetTemperature = steps[i].temp;
  char msg[112];
  snprintf_P(msg, sizeof(msg), PSTR("Step %u of %u: %.2f C for %.1f min"),
             i + 1, stepCount, steps[i].temp, steps[i].holdS / 60.0);
  setMessage(msg);
}

void programLoop() {
  if (!programActive()) return;

  if (emergencyShutdown) { stopProgram("emergency shutdown"); return; }
  if (!pidEnabled) {
    stopProgram(pidFaultMessage[0] ? pidFaultMessage : "PID turned off");
    return;
  }
  if (state == PROGRAM_HOLDING) return;   // Holding the last setpoint until stopped
  if (!dataLoggingEnabled) { stopProgram("logging stopped"); return; }

  unsigned long now = millis();
  if (now - stepStartMs < (unsigned long)steps[stepIndex].holdS * 1000UL) return;

  if (stepIndex + 1 < stepCount) {
    startStep(stepIndex + 1);
    return;
  }

  // Last step finished
  finishedMs = now;
  if (holdLastAtEnd) {
    state = PROGRAM_HOLDING;
    char msg[112];
    snprintf_P(msg, sizeof(msg), PSTR("All steps done (%u). Holding %.2f C until stopped."), stepCount, steps[stepIndex].temp);
    setMessage(msg);
  } else {
    heaterOff();
    state = PROGRAM_DONE;
    setMessage("All steps done. Heater off.");
  }
  if (stopLoggingAtEnd && dataLoggingEnabled) {
    dataLoggingEnabled = false;
    flushDataBuffer();
    Serial.println(F("PROGRAM: logging stopped (end of program)"));
  }
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------
static void sendText(int code, const String& msg) { server.send(code, "text/plain", msg); }

static void handleProgramPage() {
  File f = LittleFS.open("/program.html", "r");
  if (!f) { sendText(500, F("program.html not found: upload the filesystem image (make filesystem / pio run -t uploadfs)")); return; }
  server.streamFile(f, "text/html");
  f.close();
}

// The program as JSON: {"listId":..,"hold":..,"end":"off|hold","stopLog":true,
// "steps":[[temp,min],...]}. Written piece by piece (up to 100 steps) instead
// of building a JsonDocument, to keep RAM use small.
template <typename Out>
static void writeProgramJson(Out& out) {
  char buf[48];
  snprintf_P(buf, sizeof(buf), PSTR("{\"listId\":%u,\"hold\":%.2f,\"end\":\""), listId, defaultHoldMin);
  out.print(buf);
  out.print(holdLastAtEnd ? F("hold") : F("off"));
  out.print(stopLoggingAtEnd ? F("\",\"stopLog\":true,\"steps\":[") : F("\",\"stopLog\":false,\"steps\":["));
  for (uint8_t i = 0; i < stepCount; i++) {
    snprintf_P(buf, sizeof(buf), PSTR("%s[%.2f,%.2f]"), i ? "," : "", steps[i].temp, steps[i].holdS / 60.0);
    out.print(buf);
  }
  out.print(F("]}"));
}

static void saveProgramFile() {
  File f = LittleFS.open(PROGRAM_FILE, "w");
  if (!f) return;
  writeProgramJson(f);
  f.close();
}

// Collects text for a chunked HTTP response, sending it in ~1 KB pieces
struct ChunkWriter {
  String buf;
  void print(const char* s) { buf += s; flushIfBig(); }
  void print(const __FlashStringHelper* s) { buf += s; flushIfBig(); }
  void flushIfBig() { if (buf.length() > 1000) { server.sendContent(buf); buf = ""; } }
  void finish() { if (buf.length()) server.sendContent(buf); server.sendContent(""); }
};

// Parse "temp[:minutes],..." into steps (when store is true). Returns the
// number of steps, or -1 with err set.
static int parseStepList(const String& list, float defaultHold, bool store, float& totalMin, String& err) {
  int n = 0;
  totalMin = 0;
  int start = 0;
  while (start <= (int)list.length()) {
    int comma = list.indexOf(',', start);
    String item = list.substring(start, comma < 0 ? list.length() : comma);
    item.trim();
    if (item.length() > 0) {
      if (n >= PROGRAM_MAX_STEPS) { err = String(F("At most ")) + String(PROGRAM_MAX_STEPS) + " steps"; return -1; }
      int colon = item.indexOf(':');
      float temp = (colon < 0 ? item : item.substring(0, colon)).toFloat();
      float hold = colon < 0 ? defaultHold : item.substring(colon + 1).toFloat();
      if (temp < PROGRAM_MIN_TEMP || temp > PROGRAM_MAX_TEMP) {
        err = String(F("Step ")) + String(n + 1) + ": setpoint must be " + String(PROGRAM_MIN_TEMP, 0) + "-" + String(PROGRAM_MAX_TEMP, 0) + " C";
        return -1;
      }
      if (hold < PROGRAM_MIN_HOLD_MIN || hold > PROGRAM_MAX_HOLD_MIN) {
        err = String(F("Step ")) + String(n + 1) + ": hold must be 0.5-360 minutes";
        return -1;
      }
      uint16_t holdS = (uint16_t)lroundf(hold * 60.0f);
      if (store) { steps[n].temp = temp; steps[n].holdS = holdS; }
      totalMin += holdS / 60.0f;
      n++;
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  return n;
}

// /program/start?steps=60:10,61,62:5&hold=10&end=off|hold&stoplog=1
//   steps: up to 100 entries "temp" or "temp:minutes"; entries without minutes
//   use hold (minutes). Total time must fit within the heater time limit.
static void handleProgramStart() {
  if (emergencyShutdown) { sendText(409, F("Emergency shutdown active: reset the system first")); return; }
  if (programActive()) { sendText(409, F("A program is already running: stop it first")); return; }
  if (tuningActive()) { sendText(409, F("A PID tuning test is running: stop it first")); return; }
  if (!server.hasArg("steps")) { sendText(400, F("Missing steps")); return; }

  float defaultHold = server.hasArg("hold") ? server.arg("hold").toFloat() : 10;
  if (defaultHold < PROGRAM_MIN_HOLD_MIN || defaultHold > PROGRAM_MAX_HOLD_MIN) {
    sendText(400, F("Default hold must be 0.5-360 minutes"));
    return;
  }

  // Two passes over the list: check it all first, then store it, so a
  // rejected list doesn't overwrite the last program and no second buffer is
  // needed
  String list = server.arg("steps");
  String err;
  float totalMin = 0;
  int n = parseStepList(list, defaultHold, false, totalMin, err);
  if (n < 0) { sendText(400, err); return; }
  if (n == 0) { sendText(400, F("No steps given")); return; }
  float limitMin = MAX_HEATER_TIME / 60000.0f;
  if (totalMin > limitMin) {
    sendText(400, String(F("Total time ")) + String(totalMin, 1) + " min is over the " + String(limitMin, 0) +
                  " min continuous-heating limit");
    return;
  }

  // Logging first: a program without a log isn't useful, and logging keeps
  // the over-temperature checks running
  if (!beginDataLogging()) {
    sendText(503, F("Cannot start: logging can't start (SD card not ready / no log file)"));
    return;
  }

  parseStepList(list, defaultHold, true, totalMin, err);
  stepCount = n;
  holdLastAtEnd = server.hasArg("end") && server.arg("end") == "hold";
  stopLoggingAtEnd = !server.hasArg("stoplog") || server.arg("stoplog") == "1";
  defaultHoldMin = defaultHold;
  listId++;
  saveProgramFile();

  programStartMs = millis();
  state = PROGRAM_RUNNING;
  startStep(0);
  resetPIDState();
  heaterEnabled = true;
  pidEnabled = true;
  lastPIDUpdate = millis() - PID_INTERVAL;  // Run the PID right away

  sendText(200, String(programMessage));
}

static void handleProgramStop() {
  if (programActive()) {
    stopProgram("stopped from the program page");
    sendText(200, String(programMessage));
  } else {
    sendText(200, F("No program running"));
  }
}

static const char* stateName() {
  switch (state) {
    case PROGRAM_RUNNING: return "running";
    case PROGRAM_HOLDING: return "holding";
    case PROGRAM_DONE: return "done";
    case PROGRAM_STOPPED: return "stopped";
    default: return "idle";
  }
}

// Latest readings of both sensors: the last logged row, or direct reads
static void latestTemps(float& pidT, float& auxT) {
  if (bufferIndex > 0 || bufferFull) {
    int idx = bufferIndex == 0 ? BUFFER_SIZE - 1 : bufferIndex - 1;
    pidT = readings[idx].temperature;
    auxT = convertAuxThermistorToTemperature(readings[idx].auxThermistorAdc);
  } else {
    selectMuxChannel(THERMISTOR_CHANNEL);
    delayMicroseconds(100);
    pidT = convertThermistorToTemperature(analogRead(ADC_PIN));
    selectMuxChannel(AUX_THERMISTOR_CHANNEL);
    delayMicroseconds(100);
    auxT = convertAuxThermistorToTemperature(analogRead(ADC_PIN));
  }
}

// /program/status: program state, plus live readings and the logging/heater
// settings the page shows. The step list itself is at /program/steps (fetched
// when listId changes), so this stays small with 100 steps.
static void handleProgramStatus() {
  JsonDocument doc;
  unsigned long now = millis();
  doc["state"] = stateName();
  doc["message"] = programMessage;
  doc["running"] = programRunning();
  doc["active"] = programActive();
  doc["step"] = stepCount ? stepIndex + 1 : 0;
  doc["count"] = stepCount;
  if (stepCount) {
    float total = 0, before = 0;
    for (uint8_t i = 0; i < stepCount; i++) {
      total += steps[i].holdS;
      if (i < stepIndex) before += steps[i].holdS;
    }
    float stepElapsed = programRunning() ? (now - stepStartMs) / 1000.0 : 0;
    doc["stepElapsed"] = stepElapsed;
    doc["stepRemaining"] = programRunning() ? steps[stepIndex].holdS - stepElapsed : 0;
    doc["totalTime"] = total;
    doc["totalRemaining"] = programRunning() ? total - before - stepElapsed : 0;
    doc["elapsed"] = programActive() ? (now - programStartMs) / 1000.0 : (finishedMs - programStartMs) / 1000.0;
  }
  doc["listId"] = listId;
  doc["endAction"] = holdLastAtEnd ? "hold" : "off";
  doc["stopLog"] = stopLoggingAtEnd;
  // Live state
  float pidT, auxT;
  latestTemps(pidT, auxT);
  if (pidT > -50 && pidT < 200) doc["pidTemp"] = pidT;
  if (auxT > -50 && auxT < 200) doc["auxTemp"] = auxT;
  doc["timestamp"] = now;
  doc["target"] = targetTemperature;
  doc["duty"] = heaterDutyCycle;
  doc["pidEnabled"] = pidEnabled;
  doc["control"] = controlSensorName(controlSensor);
  doc["kp"] = pidKp; doc["ki"] = pidKi; doc["kd"] = pidKd;
  doc["sheetLimit"] = sheetLimit;
  doc["sheetLimited"] = sheetLimited;
  doc["logging"] = dataLoggingEnabled;
  doc["logFile"] = logFileName;
  doc["averaging"] = averagingEnabled;
  doc["averageSamples"] = averageSamples;
  doc["maxHeaterMinutes"] = MAX_HEATER_TIME / 60000UL;
  doc["heaterRuntime"] = heaterDutyCycle > 0 ? (now - heaterStartTime) / 1000 : 0;
  doc["emergency"] = (bool)emergencyShutdown;
  doc["tuning"] = tuningActive();
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// /program/steps: the current (or last) program's step list
static void handleProgramSteps() {
  server.chunkedResponseModeStart(200, "application/json");
  ChunkWriter w;
  writeProgramJson(w);
  w.finish();
}

// /program/saved: the last program started (for the page to reload)
static void handleProgramSaved() {
  File f = LittleFS.open(PROGRAM_FILE, "r");
  if (!f) { server.send(404, "application/json", "{}"); return; }
  server.streamFile(f, "application/json");
  f.close();
}

void setupProgramRoutes() {
  server.on("/program", handleProgramPage);
  server.on("/program/start", handleProgramStart);
  server.on("/program/stop", handleProgramStop);
  server.on("/program/status", handleProgramStatus);
  server.on("/program/saved", handleProgramSaved);
  server.on("/program/steps", handleProgramSteps);
}
