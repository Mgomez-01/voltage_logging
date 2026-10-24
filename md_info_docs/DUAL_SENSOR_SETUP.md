# Sensor Setup (CD74HC4067 Multiplexer)

The ESP8266 has a single analog input (A0). A CD74HC4067 16-channel multiplexer puts the voltage sensor and both thermistors on that one pin, and the firmware switches between them.

(This file kept its original name from when there were two channels. There are now three.)

## Components
- **CD74HC4067** 16-channel analog multiplexer
- **Voltage source** on channel 0
- **PID Temp Sensor** on channel 1: 100k NTC thermistor (β = 3950 K, Ender 3 type) with a **100 kΩ** series resistor. The PID and the safety checks use this sensor.
- **AUX Temp Sensor** on channel 2: MF55 ultra-thin 10k NTC thermistor (β = 3950 K) with a **2.0 kΩ** series resistor. A second sensor at another location for redundancy and sanity checks. It is logged and displayed only, and not used by the PID or the safety checks.

## Wiring
```
ESP8266 -> CD74HC4067
├─ A0          -> COM (common analog output)
├─ D1 (GPIO5)  -> S0
├─ D2 (GPIO4)  -> S1
├─ D3 (GPIO0)  -> S2
├─ D4 (GPIO2)  -> S3
├─ 3.3V        -> VCC and EN
└─ GND         -> GND

CD74HC4067 channels:
├─ C0 -> voltage source
├─ C1 -> PID Temp Sensor divider: 3.3V --[100k]-- C1 --[100k NTC]-- GND
└─ C2 -> AUX Temp Sensor divider: 3.3V --[2.0k]-- C2 --[10k NTC]--  GND
```
The SD card and heater wiring are in [WIRING_GUIDE.md](../WIRING_GUIDE.md).

## How sampling works
- One ADC read every 2 ms, cycling channel 0 → 1 → 2
- Each channel is read up to ~167 times per second
- After each switch the firmware waits 10 µs + 50 µs for the signal to settle
- Each AUX read completes one row (voltage, PID Temp Sensor and AUX Temp Sensor readings with their raw ADC counts, and the heater/PID state). That gives up to ~167 CSV rows per second, with the three readings in a row taken within ~4 ms of each other.
- The row's timestamp and heater/PID state are taken at the PID Temp Sensor read, the sample the PID uses
- Rows collect in a 50-row RAM buffer, then are appended to the current log file on the SD card

Real rates are a little lower because sampling pauses during SD writes and the 1 s serial debug print.

## Conversions
**Voltage (C0):** `voltage = ADC / 1024` (0-1 V scale, ~1 mV per step).
On NodeMCU / D1 mini boards A0 has an onboard divider that makes its range 0-3.3 V, so this value is the real voltage ÷ 3.3.

**Temperatures (C1, C2):** both use the Beta equation (simplified Steinhart-Hart) with their own constants:
```
V = ADC / 1024 * 3.3
R = Rs * V / (3.3 - V)
1/T = 1/298.15 + ln(R / R25) / B      (T in kelvin)

PID Temp Sensor: Rs = 100k, R25 = 100k, B = 3950
AUX Temp Sensor: Rs = 2.0k, R25 = 10k,  B = 3950
```
-999 is logged when a sensor reads as open or shorted, including when the AUX Temp Sensor is not connected.

Each `V` is logged (`thermistor_v`, `aux_v`) along with the raw count (`thermistor_adc`, `aux_adc`), so every logged temperature can be checked or recalculated afterwards.

Constants, in `src/sensor_manager.cpp`:
```cpp
// PID Temp Sensor (channel 1)
const float THERMISTOR_NOMINAL = 100000.0;   // 100 kΩ at 25 °C
const float TEMPERATURE_NOMINAL = 25.0;      // 25 °C reference
const float B_COEFFICIENT = 3950.0;          // Beta coefficient
const float SERIES_RESISTOR = 100000.0;      // 100 kΩ series resistor

// AUX Temp Sensor (channel 2)
const float AUX_THERMISTOR_NOMINAL = 10000.0;  // 10 kΩ at 25 °C
const float AUX_TEMPERATURE_NOMINAL = 25.0;    // 25 °C reference
const float AUX_B_COEFFICIENT = 3950.0;        // B25/50 (3435 for a 103F3435 part)
const float AUX_SERIES_RESISTOR = 2000.0;      // 2.0 kΩ series resistor
```
Change these if you fit a different thermistor or resistor. The series-resistor constant must match the resistor actually fitted.

### Why 2.0 kΩ for the AUX Temp Sensor
The divider is most sensitive where the series resistor equals the NTC resistance. The MF55 10k is ~2.5 kΩ at 60 °C and ~1.8 kΩ at 70 °C, so ~2 kΩ gives ~9 ADC counts per °C (~0.11 °C per count) in that range. For comparison, a 10 kΩ resistor would give ~5 counts/°C there. Anything from 2.0 kΩ to 2.7 kΩ performs about the same at 60-65 °C.

Trade-off: the low resistance passes more current through the NTC. At 65 °C it dissipates ~1.4 mW, which can read up to ~1.9 °C high in still air (dissipation constant 0.7 mW/°C), and less when it is pressed against a surface. 2.7 kΩ reduces this by about a quarter.

## CSV format
Each boot creates a new file on the SD card (`data.log`, `data_1.log`, ...):
```csv
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,aux_temp,aux_adc,aux_v
120004,0.822266,25.000,1,45.00,18.40,842,512,1.6500,1,2.200,0.250,0.150,25.053,853,2.7489
```
`temperature` is the PID Temp Sensor and `aux_temp` is the AUX Temp Sensor. Column details are in [README.md](../README.md#data-format).

## Expected serial output at boot
```
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: PID Temp Sensor (100k NTC)
Channel 2: AUX Temp Sensor (MF55 10k NTC, not used by PID)
...
Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V
Channel 1 (PID Temp Sensor): 512 -> 25.00°C
Channel 2 (AUX Temp Sensor): 853 -> 25.05°C
```
A channel with no thermistor connected shows `N/A (no thermistor connected)`.

## Checks
- ~1.65 V at C1 at room temperature (25 °C gives exactly half of 3.3 V with the 100k/100k divider)
- ~2.75 V at C2 at room temperature, ~1.7 V at 65 °C
- Touch each thermistor: its temperature should rise within a few seconds and the other should not
- At room temperature, with both sensors in the same air, they should agree within about 1 °C
- If channels read the same value, check S0-S3 and that EN is tied to 3.3 V

## Expansion
Channels C3-C15 are free for more analog sensors (extra thermistors, light, current sensors). The firmware currently reads C0, C1 and C2.
