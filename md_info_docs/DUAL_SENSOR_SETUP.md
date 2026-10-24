# Dual Sensor Setup (CD74HC4067 Multiplexer)

The ESP8266 has a single analog input (A0). A CD74HC4067 16-channel multiplexer puts the voltage sensor and the thermistor on that one pin, and the firmware switches between them.

## Components
- **CD74HC4067** 16-channel analog multiplexer
- **100 kΩ resistor** (top of the thermistor divider)
- **Voltage source** on channel 0
- **100k NTC thermistor** (β = 3950 K, Ender 3 type) on channel 1

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
└─ C1 -> thermistor divider: 3.3V --[100k]-- C1 --[NTC]-- GND
```
The SD card and heater wiring are in [WIRING_GUIDE.md](../WIRING_GUIDE.md).

## How sampling works
- One ADC read every 2 ms, alternating channel 0 and channel 1
- Each channel is read up to 250 times per second
- After each switch the firmware waits 10 µs + 50 µs for the signal to settle
- Each thermistor read completes one row (latest voltage + temperature, both raw ADC counts, and the heater/PID state), so up to 250 CSV rows per second
- Rows collect in a 50-row RAM buffer, then are appended to the current log file on the SD card

Real rates are a little lower because sampling pauses during SD writes and the 1 s serial debug print.

## Conversions
**Voltage (C0):** `voltage = ADC / 1024` (0-1 V scale, ~1 mV per step).
On NodeMCU / D1 mini boards A0 has an onboard divider that makes its range 0-3.3 V, so this value is the real voltage ÷ 3.3.

**Temperature (C1):** Beta equation (simplified Steinhart-Hart):
```
V = ADC / 1024 * 3.3
R = 100k * V / (3.3 - V)
1/T = 1/298.15 + ln(R / 100k) / 3950      (T in kelvin)
```
-999 is logged when the sensor reads as open or shorted.

The voltage `V` above is logged as `thermistor_v` (and the raw count as `thermistor_adc`), so each logged temperature can be checked or recalculated afterwards.

Constants, in `src/sensor_manager.cpp`:
```cpp
const float THERMISTOR_NOMINAL = 100000.0;   // 100 kΩ at 25 °C
const float TEMPERATURE_NOMINAL = 25.0;      // 25 °C reference
const float B_COEFFICIENT = 3950.0;          // Beta coefficient
const float SERIES_RESISTOR = 100000.0;      // 100 kΩ series resistor
```
Change these if you use a different thermistor.

## CSV format
Each boot creates a new file on the SD card (`data.log`, `data_1.log`, ...):
```csv
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd
120004,0.822266,25.000,1,45.00,18.40,842,512,1.6500,1,2.200,0.250,0.150
```
Column details are in [README.md](../README.md#data-format).

## Expected serial output at boot
```
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: Thermistor (100k NTC)
...
Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V
Channel 1 (Thermistor): 512 -> 25.00°C
```
With no thermistor connected, channel 1 shows `N/A (no thermistor connected)`.

## Checks
- ~1.65 V at C1 at room temperature (25 °C gives exactly half of 3.3 V)
- Touch the thermistor: the temperature should rise within a few seconds
- If both channels read the same value, check S0-S3 and that EN is tied to 3.3 V

## Expansion
Channels C2-C15 are free for more analog sensors (extra thermistors, light, current sensors). The firmware currently reads only C0 and C1.
