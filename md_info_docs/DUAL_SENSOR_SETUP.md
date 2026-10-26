# Sensor Setup (CD74HC4067 Multiplexer)

The ESP8266 has a single analog input (A0). A CD74HC4067 16-channel multiplexer puts the voltage sensor and both thermistors on that one pin, and the firmware switches between them.

(This file kept its original name from when there were two channels. There are now three.)

## Components
- **CD74HC4067** 16-channel analog multiplexer
- **Voltage source** on channel 0
- **PID Temp Sensor** on channel 1: 100k NTC thermistor (β = 3950 K, Ender 3 type) with a **100 kΩ** series resistor (measures 98 kΩ on this board). The PID and the safety checks use this sensor.
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
├─ C1 -> PID Temp Sensor divider: 3.3V --[100k, meas. 98k]-- C1 --[100k NTC]-- GND
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
On NodeMCU / D1 mini boards A0 has an onboard divider that makes its range 0-3.2 V, so this value is the real voltage ÷ 3.2.

**Temperatures (C1, C2):** the board's 220k/100k A0 divider makes A0's full scale 3.2 V, and it loads the selected thermistor divider with 320 kΩ. Both conversions model that, then use the Beta equation (simplified Steinhart-Hart) with their own constants:
```
V = ADC / 1024 * 3.2 * gain          A0 voltage (gain = ADC_CALIBRATION_GAIN, 1.0 until calibrated)
P = Rs * V / (VS - V)                NTC in parallel with the 320k A0 load (VS = 3.32 V rail)
R = P * RL / (RL - P)                NTC alone (RL = 320k)
1/T = 1/298.15 + ln(R / R25) / B     (T in kelvin)

PID Temp Sensor: Rs = 98k (measured), R25 = 100k, B = 3950
AUX Temp Sensor: Rs = 2.0k,           R25 = 10k,  B = 3950
```
The load matters mostly for the PID Temp Sensor, whose ~50 kΩ divider is pulled ~14 % low. Leaving it out made that sensor read ~5 °C hot, while the 3.3 V vs 3.2 V full scale made the AUX Temp Sensor read ~4 °C cold. With both modelled, the two sensors agree to within a few hundredths of a °C at room temperature.

-999 is logged when a sensor reads as shorted (ADC ≤ 1), saturated (ADC ≥ 1023: AUX unplugged, or below about -11 °C), or open (NTC resistance above 20 × R25, colder than about -30 °C). The last case catches an unplugged PID Temp Sensor, which reads ~2.54 V (ADC ~813) because of the A0 load, not the rail voltage.

Each `V` is logged (`thermistor_v`, `aux_v`) along with the raw count (`thermistor_adc`, `aux_adc`), so every logged temperature can be checked or recalculated afterwards.

Constants, in `src/sensor_manager.cpp`:
```cpp
// Measurement circuit (both channels)
const float THERMISTOR_SUPPLY_VOLTAGE = 3.32;  // measured rail
const float ADC_FULL_SCALE_VOLTAGE = 3.2;      // A0 full scale (220k/100k board divider)
const float ADC_INPUT_RESISTANCE = 320000.0;   // A0 load on the selected divider
const float ADC_CALIBRATION_GAIN = 1.0;        // set from the ice bath / reference calibration

// PID Temp Sensor (channel 1)
const float THERMISTOR_NOMINAL = 100000.0;   // 100 kΩ at 25 °C (datasheet R25)
const float TEMPERATURE_NOMINAL = 25.0;      // 25 °C reference
const float B_COEFFICIENT = 3950.0;          // Beta coefficient
const float SERIES_RESISTOR = 98000.0;       // series resistor, measured

// AUX Temp Sensor (channel 2)
const float AUX_THERMISTOR_NOMINAL = 10000.0;  // 10 kΩ at 25 °C
const float AUX_TEMPERATURE_NOMINAL = 25.0;    // 25 °C reference
const float AUX_B_COEFFICIENT = 3950.0;        // B25/50 (3435 for a 103F3435 part)
const float AUX_SERIES_RESISTOR = 2000.0;      // 2.0 kΩ series resistor
```
Change these if you fit a different thermistor or resistor. The series-resistor constants must match the resistors actually fitted, and the supply constant the measured rail. Keep the R25 values at the datasheet numbers: a thermistor reading 104.5 kΩ or 10.41 kΩ at room temperature is just at ~24 °C.

### Why 2.0 kΩ for the AUX Temp Sensor
The divider is most sensitive where the series resistor equals the NTC resistance. The MF55 10k is ~2.5 kΩ at 60 °C and ~1.8 kΩ at 70 °C, so ~2 kΩ gives ~9 ADC counts per °C (~0.11 °C per count) in that range. For comparison, a 10 kΩ resistor would give ~5 counts/°C there. Anything from 2.0 kΩ to 2.7 kΩ performs about the same at 60-65 °C.

Trade-off: the low resistance passes more current through the NTC. At 65 °C it dissipates ~1.4 mW, which can read up to ~1.9 °C high in still air (dissipation constant 0.7 mW/°C), and less when it is pressed against a surface. 2.7 kΩ reduces this by about a quarter.

## CSV format
Each boot creates a new file on the SD card (`data.log`, `data_1.log`, ...):
```csv
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,aux_temp,aux_adc,aux_v
64659,0.016602,22.591,0,45.00,0.00,17,486,1.5188,0,2.200,0.250,0.150,22.745,895,2.7969
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
Channel 1 (PID Temp Sensor): 465 -> 24.97°C
Channel 2 (AUX Temp Sensor): 881 -> 24.96°C
```
(Counts at 25 °C for this board, before calibration.) A channel with no thermistor connected shows `N/A (no thermistor connected)`.

## Checks
- C1 at 25 °C: ~1.68 V unloaded, ~1.45 V while sampled; a meter shows something in between while the ESP8266 runs
- C2 at 25 °C: ~2.75 V; ~1.7 V at 65 °C
- Touch each thermistor: its temperature should rise within a few seconds and the other should not
- At room temperature, with both sensors in the same air, they should agree within about 1 °C
- If channels read the same value, check S0-S3 and that EN is tied to 3.3 V

## Expansion
Channels C3-C15 are free for more analog sensors (extra thermistors, light, current sensors). The firmware currently reads C0, C1 and C2.
