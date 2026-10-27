#include "sensor_manager.h"
#include <math.h>

// Multiplexer control pins (CD74HC4067)
const int MUX_S0 = 5;  // D1
const int MUX_S1 = 4;  // D2
const int MUX_S2 = 0;  // D3
const int MUX_S3 = 2;  // D4

// Sensor channels on multiplexer
const int VOLTAGE_CHANNEL = 0;        // Channel 0: voltage measurement
const int THERMISTOR_CHANNEL = 1;     // Channel 1: PID Temp Sensor (controls the heater)
const int AUX_THERMISTOR_CHANNEL = 2; // Channel 2: AUX Temp Sensor (logged only, not in the PID)

// ADC pin
const int ADC_PIN = A0;

// ---------------------------------------------------------------------------
// Measurement circuit (shared by both thermistor channels)
//
//   VS ---[Rs]---+--- mux --- A0 ---[220k]---+--- ESP8266 ADC (0-1.0 V)
//                |                           |
//              [NTC]                       [100k]
//                |                           |
//               GND                         GND
//
// The NodeMCU / Wemos D1 mini board has a 220k/100k divider in front of the
// ESP8266's 1.0 V ADC. That makes A0's full scale 3.2 V (not 3.3 V), and it
// puts a 320k load on whichever divider the mux has selected. Both effects are
// modelled below; ignoring them made the PID sensor read ~5 °C hot and the AUX
// sensor ~4 °C cold at room temperature.
// ---------------------------------------------------------------------------
const float THERMISTOR_SUPPLY_VOLTAGE = 3.32;  // VS: measured rail feeding both dividers (V)
const float ADC_FULL_SCALE_VOLTAGE = 3.2;      // A0 voltage that reads 1024 (220k/100k board divider)
const float ADC_INPUT_RESISTANCE = 320000.0;   // Board divider seen from A0 (220k + 100k), loads the
                                               // selected divider. Without a capacitor on C1/C2 the
                                               // load applies at the moment of the sample. If you add
                                               // a filter capacitor on C1/C2, the node averages instead:
                                               // use ~960000 (320k x 3 channels) and recalibrate.
const float ADC_CALIBRATION_GAIN = 1.0;        // Multiplier on the A0 voltage. Leave at 1.0: the ice
                                               // bath errors had opposite signs on the two sensors, so
                                               // one gain can't fix both. Calibration is done per sensor
                                               // with R25/B below instead (tools/calibrate_thermistors.py).
                                               // Changing this invalidates that calibration.

// Readings beyond these limits mean an open or shorted sensor, and return -999
// so the safety system's sensor-fault check sees them. Because of the 320k load,
// a disconnected PID thermistor reads ~2.54 V (ADC ~813), never near the rail,
// so "open" is detected as an implausibly high NTC resistance instead.
const int ADC_SATURATED = 1023;                // ADC at/near full scale: can't tell the value (AUX open, or < ~-11 °C)
const float NTC_OPEN_RATIO = 20.0;             // R > 20 x R25 (colder than ~-30 °C) = open sensor

// Thermistor calibration (2026-10-25): R25 and B below are fitted, not datasheet
// values. Two points in stirred water with both sensors suspended mid-depth:
//   ice bath 0.0 °C:          PID 660.8 counts, AUX 1002.5 counts
//   boiling water 95.7 °C:    PID  77.7 counts, AUX  293.1 counts
//   (boiling point for Salt Lake City, ~1300 m; settled plateau of data_4.log)
// Refit with: python3 tools/calibrate_thermistors.py --point 0:660.8:1002.5 --point 95.7:77.7:293.1
// Accuracy is approximately +/-1.5-2 °C between the two points: the ESP8266 ADC
// is non-linear (~1.7 % high at mid-scale, ~0.6 % near full scale, measured
// with a multimeter at 45 °C), and a two-point fit can't follow it.
// The fits absorb the thermistor tolerance and any remaining ADC/circuit error,
// so they only hold with the circuit constants above and the resistors fitted.

// PID Temp Sensor (channel 1): 100k NTC, Beta 3950 (Ender 3 type)
// Divider: VS -- SERIES_RESISTOR -- C1 -- NTC -- GND
const float THERMISTOR_NOMINAL = 97886.0;    // R25, fitted (datasheet: 100000)
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C
const float B_COEFFICIENT = 3910.0;          // Beta, fitted (datasheet: 3950)
const float SERIES_RESISTOR = 98000.0;       // Series resistor, measured on the board (100k part, reads 98k)

// AUX Temp Sensor (channel 2): MF55 ultra-thin 10k NTC, Beta 3950
// Divider: VS -- AUX_SERIES_RESISTOR -- C2 -- NTC -- GND
// AUX_SERIES_RESISTOR must match the resistor actually fitted. ~2 kOhm puts the
// best resolution (~9 ADC counts/°C) around 60-70°C, where the NTC itself is
// ~2.5k-1.8k. 2.0k-2.7k all give about the same resolution there; 2.7k heats
// the NTC less (self-heating ~1.4-1.9°C in still air with 2.0k at 65°C).
const float AUX_THERMISTOR_NOMINAL = 10631.0;  // R25, fitted (datasheet: 10000)
const float AUX_TEMPERATURE_NOMINAL = 25.0;    // 25°C
const float AUX_B_COEFFICIENT = 4096.0;        // Beta, fitted (datasheet B25/50: 3950, MF55 103F3950)
const float AUX_SERIES_RESISTOR = 2000.0;      // 2.0k series resistor (measured 2.00k on the board)

// Current sensor channel being read
int currentSensorChannel = VOLTAGE_CHANNEL; // Start with voltage channel

void setupMultiplexer() {
  Serial.print("Setting up CD74HC4067 multiplexer... ");
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  selectMuxChannel(VOLTAGE_CHANNEL);
  Serial.println("OK");
  Serial.println("Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4");
  Serial.println("Channel 0: Voltage sensor");
  Serial.println(F("Channel 1: PID Temp Sensor (100k NTC)"));
  Serial.println(F("Channel 2: AUX Temp Sensor (MF55 10k NTC, not used by PID)"));
}

void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(10);
}

// Voltage at A0 / the selected divider node (0-3.2 V), with the calibration gain
// applied. Used by both temperature conversions below and logged as
// thermistor_v / aux_v, so each logged voltage is exactly the one its
// temperature was calculated from.
float thermistorAdcToVoltage(float adcValue) {
  return (adcValue / 1024.0) * ADC_FULL_SCALE_VOLTAGE * ADC_CALIBRATION_GAIN;
}

// Beta-equation conversion for a divider VS -- seriesResistor -- node -- NTC -- GND,
// with the board's ADC_INPUT_RESISTANCE in parallel with the NTC while sampling.
// Returns -999 for an open, shorted or saturated sensor.
static float ntcAdcToCelsius(float adcValue, float seriesResistor, float nominalResistance,
                             float nominalTempC, float beta) {
  if (adcValue <= 1) return -999;                  // Shorted NTC (or no reading)
  if (adcValue >= ADC_SATURATED) return -999;      // At full scale: value unknown
  float voltage = thermistorAdcToVoltage(adcValue);
  if (voltage >= THERMISTOR_SUPPLY_VOLTAGE) return -999;
  // Resistance below the node (NTC in parallel with the ADC input load)
  float parallel = seriesResistor * voltage / (THERMISTOR_SUPPLY_VOLTAGE - voltage);
  if (parallel >= ADC_INPUT_RESISTANCE) return -999;  // Only the ADC load is there: open NTC
  // Remove the ADC load: 1/R = 1/parallel - 1/ADC_INPUT_RESISTANCE
  float resistance = parallel * ADC_INPUT_RESISTANCE / (ADC_INPUT_RESISTANCE - parallel);
  if (resistance <= 0) return -999;                    // Avoid log(0) or log(-)
  if (resistance > nominalResistance * NTC_OPEN_RATIO) return -999;  // Open NTC
  float steinhart;
  steinhart = resistance / nominalResistance;      // (R/Ro)
  steinhart = log(steinhart);                      // ln(R/Ro)
  steinhart /= beta;                               // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nominalTempC + 273.15);      // + (1/To)
  steinhart = 1.0 / steinhart;                     // Invert
  steinhart -= 273.15;                             // Convert to Celsius
  return steinhart;
}

// PID Temp Sensor (channel 1)
float convertThermistorToTemperature(float adcValue) {
  return ntcAdcToCelsius(adcValue, SERIES_RESISTOR, THERMISTOR_NOMINAL,
                         TEMPERATURE_NOMINAL, B_COEFFICIENT);
}

// AUX Temp Sensor (channel 2)
float convertAuxThermistorToTemperature(float adcValue) {
  return ntcAdcToCelsius(adcValue, AUX_SERIES_RESISTOR, AUX_THERMISTOR_NOMINAL,
                         AUX_TEMPERATURE_NOMINAL, AUX_B_COEFFICIENT);
}

static void printTestTemperature(const char* label, int channel, float (*convert)(float)) {
  selectMuxChannel(channel);
  delay(10);
  int reading = analogRead(A0);
  float temperature = convert(reading);
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(label);
  Serial.print("): ");
  Serial.print(reading);
  Serial.print(" -> ");
  if (temperature > -50 && temperature < 150) { // Reasonable range check
    Serial.print(temperature, 2);
    Serial.println("°C");
  } else {
    Serial.println(F("N/A (no thermistor connected)"));
  }
}

void testSensorChannels() {
  Serial.println("Testing sensor channels:");
  selectMuxChannel(VOLTAGE_CHANNEL);
  delay(10);
  int voltageReading = analogRead(A0);
  Serial.print("Channel 0 (Voltage): ");
  Serial.print(voltageReading);
  Serial.print(" -> ");
  Serial.print((voltageReading / 1024.0), 4);
  Serial.println("V");

  printTestTemperature("PID Temp Sensor", THERMISTOR_CHANNEL, convertThermistorToTemperature);
  printTestTemperature("AUX Temp Sensor", AUX_THERMISTOR_CHANNEL, convertAuxThermistorToTemperature);
}
