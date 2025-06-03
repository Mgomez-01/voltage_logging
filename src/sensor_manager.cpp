#include "sensor_manager.h"
#include <math.h>

// Multiplexer control pins (CD74HC4067)
const int MUX_S0 = 5;  // D1
const int MUX_S1 = 4;  // D2
const int MUX_S2 = 0;  // D3
const int MUX_S3 = 2;  // D4

// Sensor channels on multiplexer
const int VOLTAGE_CHANNEL = 0;    // Channel 0 for voltage measurement
const int THERMISTOR_CHANNEL = 1; // Channel 1 for thermistor

// ADC pin
const int ADC_PIN = A0;

// Thermistor configuration (typical 100k NTC thermistor)
const float THERMISTOR_NOMINAL = 100000.0;   // 100k ohm at 25°C
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C
const float B_COEFFICIENT = 3950.0;          // Beta coefficient for typical 100k thermistor
const float SERIES_RESISTOR = 100000.0;      // 100k series resistor

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
  Serial.println("Channel 1: Thermistor (100k NTC)");
}

void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(10);
}

float convertThermistorToTemperature(int adcValue) {
  if (adcValue <= 1) return -999; // Error value for no reading (check 0 and 1)
  float voltage = (adcValue / 1024.0) * 3.3; // Assuming 3.3V reference
  if (voltage >= 3.29) return -999; // Open circuit
  float resistance = SERIES_RESISTOR * voltage / (3.3 - voltage);
  if (resistance <= 0) return -999; // Avoid log(0) or log(-)
  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                      // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                    // Invert
  steinhart -= 273.15;                            // Convert to Celsius
  return steinhart;
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

  selectMuxChannel(THERMISTOR_CHANNEL);
  delay(10);
  int tempReading = analogRead(A0);
  float temperature = convertThermistorToTemperature(tempReading);
  Serial.print("Channel 1 (Thermistor): ");
  Serial.print(tempReading);
  Serial.print(" -> ");
  if (temperature > -50 && temperature < 150) { // Reasonable range check
    Serial.print(temperature, 2);
    Serial.println("°C");
  } else {
    Serial.println("N/A (no thermistor connected)");
  }
}
