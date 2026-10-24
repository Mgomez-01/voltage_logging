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

// PID Temp Sensor (channel 1): 100k NTC, Beta 3950 (Ender 3 type)
// Divider: 3.3V -- SERIES_RESISTOR -- C1 -- NTC -- GND
const float THERMISTOR_NOMINAL = 100000.0;   // 100k ohm at 25°C
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C
const float B_COEFFICIENT = 3950.0;          // Beta coefficient for typical 100k thermistor
const float SERIES_RESISTOR = 100000.0;      // 100k series resistor

// AUX Temp Sensor (channel 2): MF55 ultra-thin 10k NTC, Beta 3950
// Divider: 3.3V -- AUX_SERIES_RESISTOR -- C2 -- NTC -- GND
// AUX_SERIES_RESISTOR must match the resistor actually fitted. ~2 kOhm puts the
// best resolution (~9 ADC counts/°C) around 60-70°C, where the NTC itself is
// ~2.5k-1.8k. 2.0k-2.7k all give about the same resolution there; 2.7k heats
// the NTC less (self-heating ~1.4-1.9°C in still air with 2.0k at 65°C).
const float AUX_THERMISTOR_NOMINAL = 10000.0;  // 10k ohm at 25°C
const float AUX_TEMPERATURE_NOMINAL = 25.0;    // 25°C
const float AUX_B_COEFFICIENT = 3950.0;        // B25/50 = 3950 K (MF55 103F3950)
const float AUX_SERIES_RESISTOR = 2000.0;      // 2.0k series resistor

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

// Thermistor divider voltage (0-3.3 V ADC range). Used by both temperature
// conversions below and logged as thermistor_v / aux_v, so each logged voltage
// is exactly the one its temperature was calculated from.
float thermistorAdcToVoltage(int adcValue) {
  return (adcValue / 1024.0) * 3.3; // Assuming 3.3V reference
}

// Beta-equation conversion for a divider 3.3V -- seriesResistor -- ADC -- NTC -- GND.
// Returns -999 for an open or shorted sensor.
static float ntcAdcToCelsius(int adcValue, float seriesResistor, float nominalResistance,
                             float nominalTempC, float beta) {
  if (adcValue <= 1) return -999; // Error value for no reading (check 0 and 1)
  float voltage = thermistorAdcToVoltage(adcValue);
  if (voltage >= 3.29) return -999; // Open circuit
  float resistance = seriesResistor * voltage / (3.3 - voltage);
  if (resistance <= 0) return -999; // Avoid log(0) or log(-)
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
float convertThermistorToTemperature(int adcValue) {
  return ntcAdcToCelsius(adcValue, SERIES_RESISTOR, THERMISTOR_NOMINAL,
                         TEMPERATURE_NOMINAL, B_COEFFICIENT);
}

// AUX Temp Sensor (channel 2)
float convertAuxThermistorToTemperature(int adcValue) {
  return ntcAdcToCelsius(adcValue, AUX_SERIES_RESISTOR, AUX_THERMISTOR_NOMINAL,
                         AUX_TEMPERATURE_NOMINAL, AUX_B_COEFFICIENT);
}

static void printTestTemperature(const char* label, int channel, float (*convert)(int)) {
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
