#include "sensor_manager.h"
#include <SPI.h>
#include <math.h>

// PCB version (pcb_voltage_logging): both thermistors are read by an MCP3208
// 12-bit SPI ADC (U2). It shares SCK/MISO/MOSI (D5/D6/D7) with the SD card;
// each has its own chip select (ADC: D1/GPIO5, SD: D8/GPIO15).

const int ADC_CS_PIN = 5;                    // D1 / GPIO5 -> MCP3208 /CS (pin 10), 10k pull-up R12

// MCP3208 channels. CH0 is not connected (floating: never log it as data).
// CH3-CH7 go to the J7 spare header, wired in reverse (J7 pin 1 = CH7 ...
// pin 5 = CH3); not used yet.
const int THERMISTOR_CHANNEL = 1;            // CH1: PID Temp Sensor (sheet; default control sensor, safety checks)
const int AUX_THERMISTOR_CHANNEL = 2;        // CH2: AUX Temp Sensor (sample; PID can control on it if selected)

const float ADC_COUNTS = 4096.0;             // MCP3208: code = 4096 * Vin / Vref

// The MCP3208's maximum clock is 1 MHz at 2.7 V supply (2 MHz at 5 V); at
// 3.3 V stay at 1 MHz. The SD library uses its own (faster) settings, and each
// device gets its own SPI transaction, so the two never interfere.
static const SPISettings MCP3208_SPI(1000000, MSBFIRST, SPI_MODE0);

// ---------------------------------------------------------------------------
// Measurement circuit (same for both thermistor channels)
//
//   +3V3 ---[Rs]---+--- MCP3208 CHn          MCP3208 VREF = VDD = +3V3
//                  |
//            [NTC] + [100 nF] (C8 / C9)
//                  |
//                 GND
//
// Ratiometric: the divider and the ADC reference are the same +3V3, so
//   code / 4096 = R_ntc / (Rs + R_ntc)
// and the rail voltage cancels out. The MCP3208 input draws only nA (no
// loading correction, unlike the old A0 input's 320k). The 100 nF caps hold
// the node steady while the ADC samples (time constant ~5 ms for the PID
// sensor at 25 °C, ~0.2 ms for AUX), so no settling delay is needed.
// ---------------------------------------------------------------------------
const float THERMISTOR_SUPPLY_VOLTAGE = 3.32;  // +3V3 rail, only used for the logged thermistor_v / aux_v

// Readings at these limits mean an open or shorted sensor, and return -999 so
// the safety system's sensor-fault check sees them. An open sensor is pulled
// to +3V3 through Rs and reads ~4095; a shorted one reads ~0.
static const float ADC_SHORTED = 1;          // code <= 1: shorted NTC
static const float ADC_SATURATED = 4094;     // code >= 4094: open NTC
static const float NTC_OPEN_RATIO = 20.0;    // R > 20 x R25 (colder than ~-30 °C) = open sensor

// Thermistor constants. NOT YET CALIBRATED ON THE PCB.
// R25/B below are the fits from the old board (2026-10-25 ice bath + boiling
// water). Those fits also absorbed the old ESP8266 ADC's errors and its 320k
// load model, so on the PCB they are only a starting point (expect a degree or
// two of error). Recalibrate with an ice bath (0 °C) and boiling water
// (~95.7 °C in Salt Lake City), logging with averaging on, then:
//   python3 tools/calibrate_thermistors.py --average ice.csv --timeline   (find the flat part)
//   python3 tools/calibrate_thermistors.py --average ice.csv --from S --to S
//   (same for the boiling log), then fit both points:
//   python3 tools/calibrate_thermistors.py --point 0:PID:AUX --point 95.7:PID:AUX
// (the tool checks adc_fs in the log, so old 10-bit logs are not mixed in).
// Measure R1 and R2 on the board with the sensors unplugged and the board
// unpowered, and set SERIES_RESISTOR / AUX_SERIES_RESISTOR first: the fit
// depends on them.

// PID Temp Sensor (channel 1): 100k NTC, Beta 3950 (Ender 3 type), J4
// Divider: +3V3 -- R1 (SERIES_RESISTOR) -- CH1 -- NTC -- GND
const float THERMISTOR_NOMINAL = 97886.0;    // R25: old-board fit, recalibrate (datasheet: 100000)
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C
const float B_COEFFICIENT = 3910.0;          // Beta: old-board fit, recalibrate (datasheet: 3950)
const float SERIES_RESISTOR = 100000.0;      // R1, 100k nominal: replace with the measured value

// AUX Temp Sensor (channel 2): MF55 ultra-thin 10k NTC, Beta 3950, J5
// Divider: +3V3 -- R2 (AUX_SERIES_RESISTOR) -- CH2 -- NTC -- GND
// 2 kOhm puts the best resolution (~37 codes/°C on the MCP3208) around
// 60-70 °C, where the NTC itself is ~2.5k-1.8k. Self-heating in still air is
// ~1.4-1.9 °C at 65 °C with 2.0k (2.7k heats it less, at about the same resolution).
const float AUX_THERMISTOR_NOMINAL = 10631.0;  // R25: old-board fit, recalibrate (datasheet: 10000)
const float AUX_TEMPERATURE_NOMINAL = 25.0;    // 25°C
const float AUX_B_COEFFICIENT = 4096.0;        // Beta: old-board fit, recalibrate (datasheet B25/50: 3950)
const float AUX_SERIES_RESISTOR = 2000.0;      // R2, 2k nominal: replace with the measured value

// ---------------------------------------------------------------------------
// Hotplate correction (2026-10-28, OLD BOARD). After the Beta conversion, each
// sensor's reading is corrected by subtracting an error looked up from its
// reading (linear interpolation between points; 0 outside 0-95.7 °C).
// Measured with both sensors side by side on a hotplate surface (Kapton tape +
// thermal compound), 20 setpoints from 29 to 84 °C, referenced to the
// hotplate's displayed temperature and smoothed (the display reads whole
// degrees). Ice (0 °C) and boiling water (95.7 °C) stay exact (correction 0).
// After correction both sensors match the hotplate to 0.35 °C RMS (max 0.9 °C)
// over 29-84 °C. Absolute accuracy is limited by the hotplate itself: a
// multimeter check in water at 45 °C pointed ~2-4 °C higher than the hotplate
// scale, so treat absolute temperatures as about +/-1.5 °C; the difference
// between the two sensors is good to a few tenths of a degree.
// OFF ON THE PCB: these tables were measured with the old ESP8266 ADC, and part
// of what they corrected was that ADC's non-linearity, which the MCP3208 does
// not have. The rest was the fixture (sensor placement and contact), which is
// still there. Re-measure on the PCB after the ice/boiling recalibration and
// only turn this back on with new tables. Note: the PID-vs-AUX difference in
// use (e.g. 67 vs 54.5 °C) is heat flow through the sapphire, not ADC error, so
// it will not go away with the new ADC.
// ---------------------------------------------------------------------------
const bool APPLY_TEMPERATURE_CORRECTION = false;

struct CorrectionPoint {
  float reading;     // Beta-equation reading (°C)
  float correction;  // Subtracted from the reading (°C)
};

// PID Temp Sensor (channel 1)
static const CorrectionPoint PID_CORRECTION[] = {
  {0.0, 0.00}, {30.0, 2.29}, {40.0, 2.01}, {50.0, 1.61}, {55.0, 1.33},
  {60.0, 1.06}, {65.0, 0.95}, {70.0, 1.01}, {75.0, 1.13}, {80.0, 1.23},
  {85.0, 1.25}, {95.7, 0.00}
};

// AUX Temp Sensor (channel 2)
static const CorrectionPoint AUX_CORRECTION[] = {
  {0.0, 0.00}, {30.0, 3.99}, {40.0, 3.87}, {50.0, 3.61}, {55.0, 3.39},
  {60.0, 3.13}, {65.0, 2.95}, {70.0, 2.86}, {75.0, 2.80}, {80.0, 2.74},
  {85.0, 2.67}, {95.7, 0.00}
};

static float applyCorrection(float reading, const CorrectionPoint* table, int count) {
  if (!APPLY_TEMPERATURE_CORRECTION || reading == -999) return reading;
  if (reading <= table[0].reading || reading >= table[count - 1].reading) return reading;
  for (int i = 1; i < count; i++) {
    if (reading <= table[i].reading) {
      const CorrectionPoint& a = table[i - 1];
      const CorrectionPoint& b = table[i];
      float f = (reading - a.reading) / (b.reading - a.reading);
      return reading - (a.correction + f * (b.correction - a.correction));
    }
  }
  return reading;
}

// Latest raw sample of each sensor (see sensor_manager.h)
float livePidTemperature = NAN;
float liveAuxTemperature = NAN;
unsigned long liveSampleTime = 0;

void setupAdc() {
  Serial.print(F("Setting up MCP3208 ADC... "));
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);   // Deselected before any SPI traffic
  SPI.begin();                      // SD.begin() calls this again later; that is fine
  Serial.println(F("OK"));
  Serial.println(F("MCP3208 on SPI (D5 SCK, D6 MISO, D7 MOSI), CS = D1/GPIO5, 1 MHz"));
  Serial.println(F("CH1: PID Temp Sensor (100k NTC, J4)"));
  Serial.println(F("CH2: AUX Temp Sensor (MF55 10k NTC, J5, selectable as the PID sensor)"));
}

// One single-ended conversion of CH0-CH7, returns 0-4095. About 30 us.
// Command (MCP3208 datasheet, figure 6-1): start bit, SGL/DIFF = 1, D2 D1 D0.
uint16_t readAdcChannel(uint8_t channel) {
  channel &= 0x07;
  SPI.beginTransaction(MCP3208_SPI);
  digitalWrite(ADC_CS_PIN, LOW);
  SPI.transfer(0x06 | (channel >> 2));               // 0 0 0 0 0 START SGL D2
  uint8_t hi = SPI.transfer((channel & 0x03) << 6);  // D1 D0 x x x x x x -> ? ? ? 0 B11..B8
  uint8_t lo = SPI.transfer(0x00);                   // B7..B0
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.endTransaction();
  return ((uint16_t)(hi & 0x0F) << 8) | lo;
}

float readAdcChannelAveraged(uint8_t channel, int count) {
  if (count < 1) count = 1;
  uint32_t sum = 0;
  for (int i = 0; i < count; i++) sum += readAdcChannel(channel);
  return sum / (float)count;
}

void readTemperaturesNow(float& pidTemp, float& auxTemp, int count) {
  pidTemp = convertThermistorToTemperature(readAdcChannelAveraged(THERMISTOR_CHANNEL, count));
  auxTemp = convertAuxThermistorToTemperature(readAdcChannelAveraged(AUX_THERMISTOR_CHANNEL, count));
}

// Divider node voltage (0-3.3 V), logged as thermistor_v / aux_v for continuity
// with the old logs. Not used by the temperature conversion (that is ratiometric).
float thermistorAdcToVoltage(float adcValue) {
  return adcValue / ADC_COUNTS * THERMISTOR_SUPPLY_VOLTAGE;
}

// Beta-equation conversion for a divider +3V3 -- seriesResistor -- node -- NTC -- GND
// read ratiometrically. Returns -999 for an open or shorted sensor.
static float ntcAdcToCelsius(float adcValue, float seriesResistor, float nominalResistance,
                             float nominalTempC, float beta) {
  if (adcValue <= ADC_SHORTED) return -999;        // Shorted NTC
  if (adcValue >= ADC_SATURATED) return -999;      // Open NTC (pulled to +3V3)
  float ratio = adcValue / ADC_COUNTS;             // = R / (Rs + R)
  float resistance = seriesResistor * ratio / (1.0f - ratio);
  if (resistance <= 0) return -999;                // Avoid log(0) or log(-)
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

// PID Temp Sensor (channel 1): Beta equation, then the hotplate correction (if on)
float convertThermistorToTemperature(float adcValue) {
  float t = ntcAdcToCelsius(adcValue, SERIES_RESISTOR, THERMISTOR_NOMINAL,
                            TEMPERATURE_NOMINAL, B_COEFFICIENT);
  return applyCorrection(t, PID_CORRECTION, sizeof(PID_CORRECTION) / sizeof(PID_CORRECTION[0]));
}

// AUX Temp Sensor (channel 2): Beta equation, then the hotplate correction (if on)
float convertAuxThermistorToTemperature(float adcValue) {
  float t = ntcAdcToCelsius(adcValue, AUX_SERIES_RESISTOR, AUX_THERMISTOR_NOMINAL,
                            AUX_TEMPERATURE_NOMINAL, AUX_B_COEFFICIENT);
  return applyCorrection(t, AUX_CORRECTION, sizeof(AUX_CORRECTION) / sizeof(AUX_CORRECTION[0]));
}

static void printTestTemperature(const char* label, int channel, float (*convert)(float)) {
  uint16_t reading = readAdcChannel(channel);
  float temperature = convert(reading);
  Serial.print(F("CH"));
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(label);
  Serial.print("): ");
  Serial.print(reading);
  Serial.print(" -> ");
  if (temperature > -50 && temperature < 150) { // Reasonable range check
    Serial.print(temperature, 2);
    Serial.println("°C");
  } else if (reading >= ADC_SATURATED) {
    Serial.println(F("N/A (open: no thermistor connected)"));
  } else if (reading <= ADC_SHORTED) {
    Serial.println(F("N/A (shorted)"));
  } else {
    Serial.println(F("N/A (out of range)"));
  }
}

void testSensorChannels() {
  Serial.println(F("Testing sensor channels (MCP3208, 0-4095):"));
  printTestTemperature("PID Temp Sensor", THERMISTOR_CHANNEL, convertThermistorToTemperature);
  printTestTemperature("AUX Temp Sensor", AUX_THERMISTOR_CHANNEL, convertAuxThermistorToTemperature);
}
