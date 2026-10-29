#include "tuning.h"
#include "heater_controller.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "web_interface.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------
const uint16_t TUNE_MAX_SAMPLES = 600;            // 600 x 8 bytes = 4.8 KB of RAM
const float TUNE_MAX_TEMP = 120.0;                // Stop if the sheet (PID Temp Sensor) exceeds this
                                                  // (MAX_SAFE_TEMPERATURE 130 is the emergency limit)
const unsigned long TUNE_MAX_TOTAL_MS = 25UL * 60UL * 1000UL;  // 25 min, under the 30 min heater limit
const uint8_t TUNE_MAX_SETPOINTS = 4;
static const float TUNE_MAX_POWER = 80.0;         // Highest heater power a step test may use (%)
static const unsigned long MIN_PHASE_MS = 120000; // Hold each step test phase at least 2 min
static const unsigned long SETTLE_WINDOW_DS = 600;  // Settling judged over the last 60 s (0.1 s units)
static const float SETTLE_SLOPE_C_PER_MIN = 0.15; // Settled when the trend is flatter than this
static const uint16_t BASE_INTERVAL_DS = 10;      // Record every 1.0 s (doubles if the buffer fills)
static const int ADC_AVERAGE = 8;                 // ADC reads averaged per sensor per sample
static const char* PID_GAINS_FILE = "/pid.json";

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
TuneState tuneState = TUNE_IDLE;
TuneTest tuneTest = TEST_NONE;

static TuneSample samples[TUNE_MAX_SAMPLES];
static uint16_t sampleCount = 0;
static uint16_t sampleGen = 0;          // Changes when the buffer is restarted or compacted
static uint16_t sampleIntervalDs = BASE_INTERVAL_DS;
static unsigned long testStartMs = 0;
static unsigned long phaseStartMs = 0;
static unsigned long lastSampleMs = 0;
static char tuneMessage[112] = "Idle";

// Power step test
static float stepPower1 = 0, stepPower2 = 0;
static unsigned long maxPhaseMs = 0;
static int32_t stepAtDs = -1;           // Time of the power step (0.1 s), -1 = not yet
static bool settled1 = false, settled2 = false;

// Setpoint test
static float spList[4];
static uint8_t spCount = 0, spIndex = 0;
static unsigned long holdMs = 0;
static int32_t spStartDs[4];

static const int16_t FAULT = -32768;

static void sendText(int code, const String& msg) {
  server.send(code, "text/plain", msg);
}

bool tuningActive() {
  return tuneState == TUNE_STEP_SETTLE || tuneState == TUNE_STEP_RESPONSE || tuneState == TUNE_SETPOINT;
}

static void heaterOff() {
  pidEnabled = false;
  heaterEnabled = false;
  setHeaterPower(0.0);
}

static void finish(TuneState endState, const char* message) {
  heaterOff();
  tuneState = endState;
  strncpy(tuneMessage, message, sizeof(tuneMessage) - 1);
  tuneMessage[sizeof(tuneMessage) - 1] = '\0';
  Serial.print(F("TUNE: "));
  Serial.println(tuneMessage);
}

void stopTuning(const char* reason) {
  if (!tuningActive()) return;
  char msg[112];
  snprintf_P(msg, sizeof(msg), PSTR("Stopped: %s. Heater off."), reason);
  finish(TUNE_ABORTED, msg);
}

static float readAveragedAdc(int channel) {
  selectMuxChannel(channel);
  delayMicroseconds(60);
  uint32_t sum = 0;
  for (int i = 0; i < ADC_AVERAGE; i++) sum += analogRead(ADC_PIN);
  return sum / (float)ADC_AVERAGE;
}

static int16_t toCentiC(float t) {
  if (t == -999 || isnan(t) || t < -300 || t > 320) return FAULT;
  return (int16_t)lroundf(t * 100.0f);
}

static uint16_t nowDs() {
  return (uint16_t)((millis() - testStartMs) / 100UL);
}

// Halve the buffer (keep every other sample) when it fills, so a long test
// still fits. The page sees the generation change and reloads the data.
static void compactSamples() {
  for (uint16_t i = 0; i < sampleCount / 2; i++) samples[i] = samples[i * 2];
  sampleCount /= 2;
  sampleIntervalDs *= 2;
  sampleGen++;
}

static void startRecording() {
  sampleCount = 0;
  sampleGen++;
  sampleIntervalDs = BASE_INTERVAL_DS;
  testStartMs = millis();
  phaseStartMs = testStartMs;
  lastSampleMs = 0;
}

// Linear-regression slope of the PID Temp Sensor over the last 60 s, in °C/min.
// Returns NAN if there are too few samples.
static float recentSlopeCPerMin() {
  if (sampleCount < 5) return NAN;
  uint16_t tEnd = samples[sampleCount - 1].t;
  double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = sampleCount - 1; i >= 0; i--) {
    if ((unsigned long)(tEnd - samples[i].t) > SETTLE_WINDOW_DS) break;
    if (samples[i].pidT == FAULT) continue;
    double x = samples[i].t / 600.0;        // minutes
    double y = samples[i].pidT / 100.0;
    n++; sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  if (n < 5) return NAN;
  double den = n * sxx - sx * sx;
  if (den <= 0) return NAN;
  return (float)((n * sxy - sx * sy) / den);
}

static bool phaseSettled() {
  if (millis() - phaseStartMs < MIN_PHASE_MS) return false;
  float slope = recentSlopeCPerMin();
  return !isnan(slope) && fabsf(slope) < SETTLE_SLOPE_C_PER_MIN;
}

void tuningLoop() {
  if (!tuningActive()) return;

  if (emergencyShutdown) { stopTuning("emergency shutdown"); return; }

  unsigned long now = millis();
  if (lastSampleMs != 0 && now - lastSampleMs < (unsigned long)sampleIntervalDs * 100UL) return;
  lastSampleMs = now;

  // Read both sensors (averaged). This switches the mux, which the logging
  // state machine re-selects on its next read anyway.
  float pidT = convertThermistorToTemperature(readAveragedAdc(THERMISTOR_CHANNEL));
  float auxT = convertAuxThermistorToTemperature(readAveragedAdc(AUX_THERMISTOR_CHANNEL));

  // Safety guards
  if (pidT == -999 || isnan(pidT)) { stopTuning("PID Temp Sensor fault"); return; }
  if (pidT > TUNE_MAX_TEMP) {
    char msg[64];
    snprintf_P(msg, sizeof(msg), PSTR("sheet reached %.1f C (limit %.0f C)"), pidT, TUNE_MAX_TEMP);
    stopTuning(msg);
    return;
  }
  if (now - testStartMs > TUNE_MAX_TOTAL_MS) { stopTuning("25 min test time limit"); return; }
  if (tuneTest == TEST_STEP && (!heaterEnabled || pidEnabled)) { stopTuning("heater changed from the main page"); return; }
  if (tuneTest == TEST_SETPOINT && !pidEnabled) { stopTuning("PID turned off from the main page"); return; }

  // Record
  if (sampleCount >= TUNE_MAX_SAMPLES) compactSamples();
  TuneSample& s = samples[sampleCount++];
  s.t = nowDs();
  s.pidT = toCentiC(pidT);
  s.auxT = toCentiC(auxT);
  s.duty = (uint16_t)lroundf(heaterDutyCycle * 10.0f);

  // Advance the test
  unsigned long phaseMs = now - phaseStartMs;
  if (tuneState == TUNE_STEP_SETTLE) {
    bool ok = phaseSettled();
    if (ok || phaseMs >= maxPhaseMs) {
      settled1 = ok;
      stepAtDs = nowDs();
      setHeaterPower(stepPower2);
      phaseStartMs = now;
      tuneState = TUNE_STEP_RESPONSE;
      snprintf_P(tuneMessage, sizeof(tuneMessage), PSTR("Stepped %.1f%% -> %.1f%%%s, recording response"),
               stepPower1, stepPower2, ok ? "" : " (start had not fully settled)");
      Serial.print(F("TUNE: ")); Serial.println(tuneMessage);
    }
  } else if (tuneState == TUNE_STEP_RESPONSE) {
    bool ok = phaseSettled();
    if (ok || phaseMs >= maxPhaseMs) {
      settled2 = ok;
      finish(TUNE_DONE, ok ? "Power step test done: response settled. Heater off."
                           : "Power step test done: response had not fully settled (time limit). Heater off.");
    }
  } else if (tuneState == TUNE_SETPOINT) {
    if (phaseMs >= holdMs) {
      spIndex++;
      if (spIndex >= spCount) {
        finish(TUNE_DONE, "Setpoint test done. Heater and PID off.");
      } else {
        targetTemperature = spList[spIndex];
        spStartDs[spIndex] = nowDs();
        phaseStartMs = now;
        snprintf_P(tuneMessage, sizeof(tuneMessage), PSTR("Setpoint %u of %u: %.1f C"),
                 spIndex + 1, spCount, spList[spIndex]);
        Serial.print(F("TUNE: ")); Serial.println(tuneMessage);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Saved PID gains (LittleFS /pid.json)
// ---------------------------------------------------------------------------
static bool validGain(float g) { return !isnan(g) && g >= 0 && g <= 100; }

void loadSavedPidGains() {
  File f = LittleFS.open(PID_GAINS_FILE, "r");
  if (!f) {
    Serial.println(F("PID gains: no saved gains, using firmware defaults"));
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  float kp = doc["kp"] | NAN, ki = doc["ki"] | NAN, kd = doc["kd"] | NAN;
  if (err || !validGain(kp) || !validGain(ki) || !validGain(kd)) {
    Serial.println(F("PID gains: /pid.json unreadable, using firmware defaults"));
    return;
  }
  pidKp = kp; pidKi = ki; pidKd = kd;
  Serial.print(F("PID gains loaded from /pid.json: Kp=")); Serial.print(pidKp, 3);
  Serial.print(F(", Ki=")); Serial.print(pidKi, 4);
  Serial.print(F(", Kd=")); Serial.println(pidKd, 3);
}

static void handlePidSave() {
  File f = LittleFS.open(PID_GAINS_FILE, "w");
  if (!f) { sendText(500, F("Could not write /pid.json")); return; }
  JsonDocument doc;
  doc["kp"] = pidKp; doc["ki"] = pidKi; doc["kd"] = pidKd;
  serializeJson(doc, f);
  f.close();
  String msg = "Saved as boot defaults: Kp=" + String(pidKp, 3) + ", Ki=" + String(pidKi, 4) + ", Kd=" + String(pidKd, 3);
  Serial.println(msg);
  sendText(200, msg);
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static void handleTunePage() {
  File f = LittleFS.open("/tune.html", "r");
  if (!f) { sendText(500, F("tune.html not found: upload the filesystem image (make filesystem / pio run -t uploadfs)")); return; }
  server.streamFile(f, "text/html");
  f.close();
}

static bool refuseStart() {
  if (emergencyShutdown) { sendText(409, F("Emergency shutdown active: reset the system first")); return true; }
  if (tuningActive()) { sendText(409, F("A test is already running")); return true; }
  return false;
}

// /tune/step?p1=20&p2=30&max=8  (powers in %, max minutes per phase 3-12)
static void handleTuneStep() {
  if (refuseStart()) return;
  if (!server.hasArg("p1") || !server.hasArg("p2")) { sendText(400, F("Missing p1 or p2")); return; }
  float p1 = server.arg("p1").toFloat(), p2 = server.arg("p2").toFloat();
  float maxMin = server.hasArg("max") ? server.arg("max").toFloat() : 8;
  if (p1 < 0 || p2 < 0 || p1 > TUNE_MAX_POWER || p2 > TUNE_MAX_POWER || fabsf(p2 - p1) < 2) {
    sendText(400, String(F("Powers must be 0-")) + String((int)TUNE_MAX_POWER) + " % and differ by at least 2 %");
    return;
  }
  if (maxMin < 3 || maxMin > 12) { sendText(400, F("max must be 3-12 minutes")); return; }

  stepPower1 = p1; stepPower2 = p2;
  maxPhaseMs = (unsigned long)(maxMin * 60000UL);
  stepAtDs = -1; settled1 = settled2 = false;
  tuneTest = TEST_STEP;
  startRecording();
  pidEnabled = false;
  heaterEnabled = true;
  setHeaterPower(p1);
  tuneState = TUNE_STEP_SETTLE;
  snprintf_P(tuneMessage, sizeof(tuneMessage), PSTR("Holding %.1f%% until the sheet settles"), p1);
  Serial.print(F("TUNE: power step test started: ")); Serial.println(tuneMessage);
  sendText(200, String(tuneMessage));
}

// /tune/setpoint?sp=60,65,70&hold=5[&kp=&ki=&kd=]
static void handleTuneSetpoint() {
  if (refuseStart()) return;
  if (!server.hasArg("sp")) { sendText(400, F("Missing sp")); return; }
  String list = server.arg("sp");
  uint8_t n = 0;
  int start = 0;
  while (start <= (int)list.length() && n < TUNE_MAX_SETPOINTS) {
    int comma = list.indexOf(',', start);
    String item = list.substring(start, comma < 0 ? list.length() : comma);
    item.trim();
    if (item.length() > 0) {
      float v = item.toFloat();
      if (v < 20 || v > TUNE_MAX_TEMP - 5) {
        sendText(400, String(F("Setpoints must be 20-")) + String((int)(TUNE_MAX_TEMP - 5)) + " C");
        return;
      }
      spList[n++] = v;
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  if (n == 0) { sendText(400, F("No setpoints given")); return; }
  float holdMin = server.hasArg("hold") ? server.arg("hold").toFloat() : 5;
  if (holdMin < 1 || holdMin > 10 || holdMin * n > 24) {
    sendText(400, F("hold must be 1-10 min, and setpoints x hold at most 24 min"));
    return;
  }
  float kp = server.hasArg("kp") ? server.arg("kp").toFloat() : pidKp;
  float ki = server.hasArg("ki") ? server.arg("ki").toFloat() : pidKi;
  float kd = server.hasArg("kd") ? server.arg("kd").toFloat() : pidKd;
  if (!validGain(kp) || !validGain(ki) || !validGain(kd)) { sendText(400, F("Gains must be 0-100")); return; }

  pidKp = kp; pidKi = ki; pidKd = kd;
  spCount = n; spIndex = 0;
  holdMs = (unsigned long)(holdMin * 60000UL);
  for (uint8_t i = 0; i < 4; i++) spStartDs[i] = -1;
  tuneTest = TEST_SETPOINT;
  startRecording();
  spStartDs[0] = 0;
  targetTemperature = spList[0];
  resetPIDState();
  heaterEnabled = true;
  pidEnabled = true;
  lastPIDUpdate = millis() - PID_INTERVAL;  // Run the PID right away
  tuneState = TUNE_SETPOINT;
  snprintf_P(tuneMessage, sizeof(tuneMessage), PSTR("Setpoint 1 of %u: %.1f C"), spCount, spList[0]);
  Serial.print(F("TUNE: setpoint test started: ")); Serial.println(tuneMessage);
  sendText(200, String(tuneMessage));
}

static void handleTuneStop() {
  if (tuningActive()) {
    stopTuning("stopped from the tuning page");
    sendText(200, String(tuneMessage));
  } else {
    sendText(200, F("No test running"));
  }
}

static const char* stateName(TuneState s) {
  switch (s) {
    case TUNE_STEP_SETTLE: return "step_settle";
    case TUNE_STEP_RESPONSE: return "step_response";
    case TUNE_SETPOINT: return "setpoint";
    case TUNE_DONE: return "done";
    case TUNE_ABORTED: return "aborted";
    default: return "idle";
  }
}

static void handleTuneStatus() {
  JsonDocument doc;
  doc["state"] = stateName(tuneState);
  doc["test"] = tuneTest == TEST_STEP ? "step" : (tuneTest == TEST_SETPOINT ? "setpoint" : "none");
  doc["message"] = tuneMessage;
  doc["running"] = tuningActive();
  doc["elapsed"] = tuningActive() ? (millis() - testStartMs) / 1000.0 : (sampleCount ? samples[sampleCount - 1].t / 10.0 : 0);
  doc["count"] = sampleCount;
  doc["gen"] = sampleGen;
  doc["interval"] = sampleIntervalDs / 10.0;
  doc["maxSamples"] = TUNE_MAX_SAMPLES;
  float slope = recentSlopeCPerMin();
  if (!isnan(slope) && tuningActive()) doc["slope"] = slope;
  doc["duty"] = heaterDutyCycle;
  doc["pidEnabled"] = pidEnabled;
  doc["target"] = targetTemperature;
  doc["kp"] = pidKp; doc["ki"] = pidKi; doc["kd"] = pidKd;
  doc["maxTemp"] = TUNE_MAX_TEMP;
  doc["emergency"] = (bool)emergencyShutdown;
  if (tuneTest == TEST_STEP) {
    JsonObject st = doc["step"].to<JsonObject>();
    st["p1"] = stepPower1; st["p2"] = stepPower2;
    st["at"] = stepAtDs < 0 ? -1.0 : stepAtDs / 10.0;
    st["settled1"] = settled1; st["settled2"] = settled2;
    st["maxPhase"] = maxPhaseMs / 60000.0;
  } else if (tuneTest == TEST_SETPOINT) {
    JsonObject sp = doc["sp"].to<JsonObject>();
    JsonArray l = sp["list"].to<JsonArray>();
    JsonArray st = sp["starts"].to<JsonArray>();
    for (uint8_t i = 0; i < spCount; i++) {
      l.add(spList[i]);
      st.add(spStartDs[i] < 0 ? -1.0 : spStartDs[i] / 10.0);
    }
    sp["hold"] = holdMs / 60000.0;
    sp["index"] = spIndex;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// /tune/data?from=N -> up to 150 samples as [t_s, pidC, auxC|null, duty%]
static void handleTuneData() {
  uint16_t from = server.hasArg("from") ? (uint16_t)server.arg("from").toInt() : 0;
  if (from > sampleCount) from = sampleCount;
  uint16_t to = from + 150;
  if (to > sampleCount) to = sampleCount;
  server.chunkedResponseModeStart(200, "application/json");
  char buf[80];
  snprintf_P(buf, sizeof(buf), PSTR("{\"gen\":%u,\"from\":%u,\"count\":%u,\"s\":["), sampleGen, from, sampleCount);
  String chunk = buf;
  for (uint16_t i = from; i < to; i++) {
    const TuneSample& s = samples[i];
    char aux[12];
    if (s.auxT == FAULT) strcpy(aux, "null"); else snprintf_P(aux, sizeof(aux), PSTR("%.2f"), s.auxT / 100.0);
    char pid[12];
    if (s.pidT == FAULT) strcpy(pid, "null"); else snprintf_P(pid, sizeof(pid), PSTR("%.2f"), s.pidT / 100.0);
    snprintf_P(buf, sizeof(buf), PSTR("%s[%.1f,%s,%s,%.1f]"), i == from ? "" : ",", s.t / 10.0, pid, aux, s.duty / 10.0);
    chunk += buf;
    if (chunk.length() > 900) { server.sendContent(chunk); chunk = ""; }
  }
  chunk += "]}";
  server.sendContent(chunk);
  server.sendContent("");
}

void setupTuningRoutes() {
  server.on("/tune", handleTunePage);
  server.on("/tune/status", handleTuneStatus);
  server.on("/tune/data", handleTuneData);
  server.on("/tune/step", handleTuneStep);
  server.on("/tune/setpoint", handleTuneSetpoint);
  server.on("/tune/stop", handleTuneStop);
  server.on("/pid/save", handlePidSave);
}
