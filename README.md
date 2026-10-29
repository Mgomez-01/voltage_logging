# ESP8266 Voltage + Temperature Logger with SD Card and PWM Heater Control

A data logger built on the ESP8266. It reads a voltage and two NTC thermistors through a CD74HC4067 analog multiplexer, logs them to a microSD card, and drives a heater through a MOSFET with a PWM PID controller. Everything is controlled from a web page served over the ESP8266's own WiFi access point.

## Features

- **Sensor logging**: voltage (mux channel 0), **PID Temp Sensor** (channel 1, controls the heater) and **AUX Temp Sensor** (channel 2, a second thermistor at another location for redundancy and sanity checks; not used by the PID or the safety checks)
- **SD card storage**: CSV-formatted log files on a microSD card, a new file every boot, automatic rotation at 50 MB
- **Log file manager**: list, download and delete log files from the web page
- **PWM heater control**: AOD4144 N-channel MOSFET driven at 0-100% duty from GPIO16
- **PID temperature control**: runs every 500 ms, independent of data logging
- **Real-time web interface**: WebSocket updates, live charts and statistics
- **WiFi access point**: no router needed
- **Safety system**: over-temperature, sensor-fault and max-runtime shutdown, software and hardware watchdogs, SD write timeouts, low-memory guard, and a web "System Reset" to recover

## Hardware

### Core components
- **ESP8266 board**: NodeMCU v2 or Wemos D1 mini (ESP-12E/F)
- **CD74HC4067** 16-channel analog multiplexer
- **microSD card module** (SPI, 3.3 V) or Wemos D1 mini micro-SD shield, with a FAT16/FAT32-formatted card (Class 10 or faster recommended)
- **100 kΩ resistor** for the PID Temp Sensor divider (measures 98 kΩ on this board; the firmware uses the measured value)
- **2.0 kΩ resistor** for the AUX Temp Sensor divider (see below)

### Sensors
- **Voltage source** to measure (0-1 V scale in software, see Notes)
- **PID Temp Sensor**: 100 kΩ NTC thermistor, β = 3950 K (Ender 3 compatible). Calibrated: R25 = 97.89 kΩ, B = 3910
- **AUX Temp Sensor**: MF55 ultra-thin 10 kΩ NTC thermistor, β = 3950 K (B25/50). Calibrated: R25 = 10.63 kΩ, B = 4096

The AUX series resistor is sized for resolution around 60-70 °C, where the 10k NTC measures about 2.5-1.8 kΩ. That gives ~9 ADC counts per °C (~0.11 °C per count); anything from 2.0 kΩ to 2.7 kΩ gives about the same there. With ~2 kΩ the NTC dissipates ~1.4 mW at 65 °C, which can warm it by up to ~1.9 °C in still air (0.7 mW/°C), and less when it is pressed against a surface. A 2.7 kΩ resistor cuts that by about a quarter. `AUX_SERIES_RESISTOR` in `src/sensor_manager.cpp` must match the resistor fitted.

### How the temperatures are calculated
Both dividers run from the 3.3 V rail (measured 3.32 V). On NodeMCU / D1 mini boards, A0 goes through an onboard 220k/100k divider to the ESP8266's 1.0 V ADC. That has two effects, and the firmware models both:
- **A0 full scale is 3.2 V**, not 3.3 V (ADC count 1024 = 3.2 V)
- **A0 loads the selected divider with 320 kΩ.** This matters for the PID Temp Sensor (~50 kΩ divider impedance): its node is pulled ~14 % low. It barely affects the AUX Temp Sensor (~1.7 kΩ).

Before this was modelled, the PID Temp Sensor read ~5 °C hot and the AUX Temp Sensor ~4 °C cold at room temperature. With it, they agree to within a few hundredths of a °C at room temperature.

The circuit constants are in `src/sensor_manager.cpp`: `THERMISTOR_SUPPLY_VOLTAGE` (3.32), `ADC_FULL_SCALE_VOLTAGE` (3.2), `ADC_INPUT_RESISTANCE` (320000), the measured series resistors, and `ADC_CALIBRATION_GAIN` (leave at 1.0).

### Calibration
Each thermistor's R25 and B are fitted from two reference points in stirred water, with both sensors suspended mid-depth: an ice bath (0 °C) and boiling water (95.7 °C, the boiling point at Salt Lake City's ~1300 m). The fitted values are in `src/sensor_manager.cpp`, with the datasheet values in comments:

| Sensor | Ice bath | Boiling | Fitted R25 | Fitted B | Datasheet |
|---|---|---|---|---|---|
| PID | 660.8 counts | 77.7 counts | 97 886 Ω | 3910 | 100 kΩ / 3950 |
| AUX | 1002.5 counts | 293.1 counts | 10 631 Ω | 4096 | 10 kΩ / 3950 |

With datasheet values the PID sensor read +0.6 °C in ice and 95.5 °C at boiling, and the AUX sensor −2.0 °C and 96.8 °C. A single ADC gain can't fix both, because the errors have opposite signs. That's why the calibration is per sensor.

To recalibrate (new thermistor, new resistor, or a different boiling point), average the raw counts and refit:
```
python3 tools/calibrate_thermistors.py --average data_N.log          # mean counts from a log or serial capture
python3 tools/calibrate_thermistors.py --elevation 1300              # boiling point for your elevation
python3 tools/calibrate_thermistors.py --point 0:660.8:1002.5 --point 95.7:77.7:293.1
```
The tool reads the circuit constants from `src/sensor_manager.cpp` and prints the new R25/B lines. The fit only holds for the circuit constants and resistors it was made with. Change any of them and you need to recalibrate.

Between those two points the ESP8266 ADC is not linear: a multimeter check at ~45 °C found it reading ~1.7 % high at mid-scale and only ~0.6 % high near full scale. A two-point fit can't follow that, so the hotplate correction below takes out what remains.

### Hotplate correction
After the Beta equation, each reading is corrected by subtracting an error looked up from the reading itself (linear interpolation between points, no correction outside 0–95.7 °C). The tables come from a hotplate sweep (2026-10-28): both sensors side by side on the plate surface under Kapton tape in thermal compound, 20 setpoints from 29 °C to 84 °C, referenced to the hotplate's displayed temperature and smoothed (the display reads whole degrees). Ice and boiling water stay exact.

| Reading (°C) | 0 | 30 | 40 | 50 | 55 | 60 | 65 | 70 | 75 | 80 | 85 | 95.7 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| PID correction | 0 | 2.29 | 2.01 | 1.61 | 1.33 | 1.06 | 0.95 | 1.01 | 1.13 | 1.23 | 1.25 | 0 |
| AUX correction | 0 | 3.99 | 3.87 | 3.61 | 3.39 | 3.13 | 2.95 | 2.86 | 2.80 | 2.74 | 2.67 | 0 |

`PID_CORRECTION` / `AUX_CORRECTION` in `src/sensor_manager.cpp` hold these values. `APPLY_TEMPERATURE_CORRECTION = false` turns the correction off.

**Accuracy:**
- **Between the two sensors: a few tenths of a °C.** Before the correction AUX read 1.87 ± 0.24 °C above PID at every hotplate step, and in the 45 °C water test. After it, both match the hotplate to 0.35 °C RMS (worst 0.9 °C) over 29–84 °C.
- **Absolute: about ±1.5 °C**, limited by the hotplate as a reference. Ice and boiling water agree with the corrected readings, but a multimeter check in water at ~45 °C pointed 2–4 °C higher than the hotplate scale. The hotplate could not be cross-checked before it was returned.
- `tools/calibrate_thermistors.py` fits only the Beta part (R25/B). Its "now reads" values don't include this correction.

If you add a filter capacitor on C1 or C2, the node no longer sees the full 320 kΩ load at the moment of the sample. Set `ADC_INPUT_RESISTANCE` to ~960000 (320k × 3 channels) and recalibrate.

An NTC resistance above 20 × R25 (colder than about -30 °C) is treated as an open sensor, and an ADC reading at full scale (1023-1024) as unknown; both log -999. This matters for the PID Temp Sensor: because of the 320 kΩ load, a disconnected PID thermistor reads ~2.54 V (ADC ~813) rather than near the rail. Without this check it would read as a cold temperature and the PID would heat at full power. Returning -999 trips the sensor-fault shutdown instead.

### Heater
- **AOD4144** N-channel logic-level MOSFET (low-side switch)
- **10 kΩ** gate pull-down resistor
- **12 V heater element** and a 12 V supply

## Wiring Overview

```
ESP8266        Connected to          Function
-------        ------------          --------
A0             Mux COM               Analog input
D0 (GPIO16)    MOSFET gate           Heater PWM
D1 (GPIO5)     Mux S0                Channel select bit 0
D2 (GPIO4)     Mux S1                Channel select bit 1
D3 (GPIO0)     Mux S2                Channel select bit 2
D4 (GPIO2)     Mux S3                Channel select bit 3
D5 (GPIO14)    SD SCK                SPI clock
D6 (GPIO12)    SD MISO               SPI data in
D7 (GPIO13)    SD MOSI               SPI data out
D8 (GPIO15)    SD CS                 SPI chip select
3V3            Mux VCC + EN, SD VCC, both thermistor dividers
GND            Mux, SD, MOSFET source, 12 V supply ground

Mux channels:
  C0 -> voltage source
  C1 -> PID Temp Sensor: 3.3V --[100k, meas. 98k]-- C1 --[100k NTC]-- GND
  C2 -> AUX Temp Sensor: 3.3V --[2.0k]------------- C2 --[10k NTC]--  GND
```

See [WIRING_GUIDE.md](WIRING_GUIDE.md) for the full diagram, heater circuit and checks.

## Software Setup

### PlatformIO (recommended)
```bash
pip install platformio
make deploy                   # build + upload firmware + upload web page + open monitor (NodeMCU)
make deploy ENV=wemos_d1_mini # same for a Wemos D1 mini
```

Or step by step:
```bash
pio run -e esp8266 -t upload       # firmware (use -e wemos_d1_mini for a D1 mini)
pio run -e esp8266 -t uploadfs     # web pages (data/ folder: main page + /tune page)
pio device monitor -b 115200
```

Both steps are needed: the firmware and the web page are uploaded separately.

### Libraries
- ESP8266 Arduino core (ESP8266WiFi, ESP8266WebServer, SD, SPI, LittleFS, Ticker)
- ArduinoJson ^7.0.0
- WebSockets ^2.4.0 (Markus Sattler)

PlatformIO installs the two external libraries automatically from `platformio.ini`.

### What the two storage areas hold
| Storage | Holds | How it gets there |
|---|---|---|
| **microSD card** | All sensor data logs (`data*.log`) | Written by the firmware while logging |
| Internal flash (LittleFS) | The web pages (`data/index.template.html`, `data/tune.html`) and saved PID gains (`/pid.json`, written by the board) | Pages: `pio run -t uploadfs` / `make filesystem`. This also erases `/pid.json` |

Sensor data is never stored in internal flash.

## Usage

1. **Insert the SD card** and power the board. The serial monitor should show `SD card initialized.` and `Created new data file: data_N.log`.
2. **Connect to WiFi**: SSID `ESP8266_VoltageLogger`, password `voltage123`.
3. **Open** `http://192.168.4.1`.
4. **Start Logging**. Logging is paused at boot. Tick **Average** first to log averaged rows instead of every raw sample (see Averaging below). If the SD card wasn't ready at boot, Start Logging tries to set it up again and shows an error if it still can't create a log file.
5. **Heater control** (optional):
   - Set the target temperature (0-130 °C, default 45 °C)
   - Enable PID, or use manual ON (100%) / OFF (0%)
   - Keep logging running while the heater is on (see Safety)
6. **Stop Logging** to flush the RAM buffer to the card.
7. **Download Data** gets the current log file. The **Log Files on SD Card** panel lists every file so you can download or delete them individually.

## Data Format

Each boot creates a new file: `data.log`, then `data_1.log`, `data_2.log`, ... When a file reaches 50 MB, logging continues in the next free name. Every file starts with the CSV header: if the current log file is deleted while the board is running, the next write recreates it with a header. The files are CSV text:

```csv
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,aux_temp,aux_adc,aux_v,samples
64659,0.016602,20.404,0,45.00,0.00,17,486,1.5188,0,2.200,0.250,0.150,20.928,895,2.7969,1
259599,0.016602,40.939,1,45.00,9.08,17,312,0.9750,1,2.250,0.320,0.240,39.606,746,2.3312,1
3391012,0.017422,42.844,0,45.00,0.00,17.84,298.41,0.9325,0,2.200,0.250,0.150,42.821,717.53,2.2423,32
```
The first two rows are raw (`samples` = 1). The third is an averaged row (32 sample cycles), so its ADC columns are mean counts with 2 decimals.

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | timestamp | ms | `millis()` since boot, taken at the PID Temp Sensor sample |
| 2 | voltage | V | Channel 0, `voltage_adc`/1024 (6 decimals) |
| 3 | temperature | °C | **PID Temp Sensor**, channel 1 (3 decimals, -999 = open/short) |
| 4 | heater_state | 0/1 | 1 if heater PWM duty > 0 |
| 5 | target_temp | °C | PID setpoint |
| 6 | pid_output | % | Heater PWM duty (PID output, or 100 in manual ON) |
| 7 | voltage_adc | counts | ADC reading for channel 0 (0-1023). Mean of the row's sample cycles when averaged (2 decimals) |
| 8 | thermistor_adc | counts | ADC reading for channel 1, PID Temp Sensor (0-1023). Mean when averaged |
| 9 | thermistor_v | V | PID Temp Sensor voltage at A0, `thermistor_adc`/1024 × 3.2 × `ADC_CALIBRATION_GAIN` (4 decimals). This is the exact value `temperature` is calculated from. |
| 10 | pid_enabled | 0/1 | 1 if the PID was controlling the heater, 0 for manual ON/OFF. Tells PID-at-100% apart from manual ON. |
| 11 | kp | - | PID proportional gain in effect (3 decimals) |
| 12 | ki | - | PID integral gain in effect |
| 13 | kd | - | PID derivative gain in effect |
| 14 | aux_temp | °C | **AUX Temp Sensor**, channel 2 (3 decimals, -999 = open/short or not connected) |
| 15 | aux_adc | counts | ADC reading for channel 2, AUX Temp Sensor (0-1023). Mean when averaged |
| 16 | aux_v | V | AUX Temp Sensor voltage at A0, `aux_adc`/1024 × 3.2 × `ADC_CALIBRATION_GAIN` (4 decimals). This is the exact value `aux_temp` is calculated from. |
| 17 | samples | cycles | Sample cycles in the row: 1 = raw, N = averaged (see Averaging) |

Columns 1-6 are the original format. Columns 7-17 were added after them, so scripts that read by column name keep working.

A raw row is one pass over channels 0 → 1 → 2, so its three readings are taken within ~4 ms of each other.

### Averaging (oversampling)
The **Average** checkbox under Data Logging Control sets what each row holds:
- **Off (default):** every sample cycle is logged raw, as above, up to ~167 rows/s. `samples` = 1.
- **On:** N consecutive sample cycles (default 32, settable 2-100 next to the checkbox) are combined into one row. The ADC counts are averaged, and `temperature`, `aux_temp`, `voltage` and the `*_v` columns are calculated from the mean counts. Averaging the counts (not the temperatures) lets the ADC noise resolve fractions of a count, as with the boiling-water mean of 77.73. `timestamp` is the middle of the window. `heater_state`, `pid_output`, `target_temp` and `pid_enabled` are from the last cycle in it. `samples` = N.

With N = 32 a row covers ~0.2 s, at up to ~5 rows/s. Averaging improves resolution and noise, but not accuracy: the ±1.5 °C absolute accuracy (see Calibration) still applies. The setting can change while logging (a partly filled average is dropped), and it resets to off at boot. The PID and the safety checks use the latest logged row, so with averaging on they also see averaged temperatures. `/averaging?enabled=1&n=32` sets it over HTTP, and `/status` reports `averagingEnabled` and `averageSamples`.

**Checking the temperatures:** with VS = 3.32 V and RL = 320 kΩ (the A0 load), both use:
```
P = Rs × V / (VS − V)        resistance below the node (NTC in parallel with RL)
R = P × RL / (RL − P)        NTC resistance
1/T = 1/298.15 + ln(R/R25)/B      (T in kelvin)
```
- PID Temp Sensor: V = `thermistor_v`, Rs = 98 kΩ, R25 = 97 886 Ω, B = 3910
- AUX Temp Sensor: V = `aux_v`, Rs = 2.0 kΩ, R25 = 10 631 Ω, B = 4096

then subtract the hotplate correction for that reading (see Calibration). That reproduces `temperature` and `aux_temp` to within a few thousandths of a °C. Logs written before the 2026-10-25 calibration used R25 = 100 kΩ / 10 kΩ and B = 3950. The raw counts let you redo either conversion with a different calibration.

**Older log files** (before the circuit model was added) used V = adc/1024 × 3.3, R = 100k × V / (3.3 − V) with no A0 load, and `thermistor_v` / `aux_v` on that 3.3 V scale. Recalculate their temperatures from `thermistor_adc` and `aux_adc` with the formulas above.

**Gains:** the kp/ki/kd columns hold the gains when the row was sampled. Changing them with `/pid/params` saves the buffered rows first, so rows never show gains they weren't sampled under. The serial monitor warns if that save fails.

With averaging off, the maximum rate is about 93 bytes × 167 rows/s ≈ 56 MB per hour, so a new 50 MB file starts about every 54 minutes. With averaging on at N = 32 it is about 2 MB per hour.

## Technical Details

### Sampling
- One ADC read every 2 ms, cycling channels 0 → 1 → 2: up to ~167 readings/s per channel and ~167 CSV rows/s (one row per cycle), or ~167/N rows/s with averaging on
- 50-row RAM buffer, appended to the SD card when full
- WebSocket update every 500 ms
- Real rates are a little lower because sampling pauses during SD writes and the 1 s debug print
- Logging stops automatically if free heap drops below 8 KB

### SD card writes
- Each write opens the file, checks its size (rotating at 50 MB), appends the buffered rows and closes the file
- A write is abandoned after 5 s. The watchdog is fed every 5 rows so slow cards don't trip it.
- Slow cards are reported in the serial monitor (> 1 s open/close, > 2 s total). See [SD_WRITE_TIMEOUT_FIX.md](md_info_docs/SD_WRITE_TIMEOUT_FIX.md).

### Heater / PID
| Setting | Value |
|---|---|
| PWM pin / range | GPIO16 (D0), 0-1000 (0.1%) |
| PWM frequency | 32 Hz (`analogWriteFreq(32)`). The core docs list 100 Hz as the minimum, but core 3.1.2's default PWM engine uses 32 Hz. |
| Kp / Ki / Kd | 2.3 / 0.045 / 0 (interim, from a model; replaced by gains saved from the tuning page) |
| PID interval | 500 ms |
| Anti-windup | Integral term limited to the 0–100 % output range, and frozen while the output is saturated |
| Derivative | On the measurement (no kick when the setpoint changes) |
| Default target | 45 °C |
| Max safe temperature | 130 °C |
| Max continuous heater time | 30 min |

The PID uses the latest logged PID Temp Sensor reading, or reads that thermistor directly when the buffer is empty, so it works with logging on or off. Its output (0-100%) sets the PWM duty directly. The AUX Temp Sensor is never used by the PID.

Before 2026-10-29 the integral was clamped to ±100, which with Ki = 0.25 capped it at 25 % output. Above ~60 °C the sheet needs more power than that to hold, so it settled several degrees below the setpoint (e.g. 88 °C for a 92 °C setpoint). With the old gains (2.2 / 0.25 / 0.15) the corrected PID overshoots ~10 °C, so the defaults changed and the gains should be tuned on the fixture.

### PID tuning page (`/tune`)
Open `http://192.168.4.1/tune` (or the link on the main page), with the whole fixture set up as for a measurement. Close the main page while tuning: each open page keeps its own connection to the board. The tests run on the board, so a dropped Wi-Fi link can't leave the heater on.

1. **Power step test (PID off).** The heater holds a start power until the sheet settles (at least 2 min, trend flatter than 0.15 °C/min), steps to a second power, records the response until it settles, then turns off. Pick powers around the working range: about 25 → 35 % holds the sheet near 85–100 °C.
2. **Model and gains.** The page fits a delay, time constant and gain (°C per % power) to the step response, and suggests PI gains (Skogestad SIMC rule) for three speeds. "Critically damped (recommended)" aims for no overshoot.
3. **Setpoint test (PID on).** Runs the PID with chosen gains through up to 4 setpoints, holding each, then turns the heater off. It reports overshoot, settling time (±0.5 °C on a 10 s average) and the remaining error for each step.
4. **Apply / Save.** "Apply" sets the running gains. "Save" also stores them on the board's flash (`/pid.json`), loaded at boot. Uploading the filesystem image (`make filesystem`) erases the saved gains: note them first.

Safety during tests: the heater turns off if the sheet passes 120 °C, the PID sensor faults, an emergency shutdown happens, a test runs over 25 min (under the 30 min heater limit), or the heater/PID is changed from the main page. "Stop test" turns the heater off at any time. Test data (1 s samples, PID and AUX temperatures, heater power) is kept in RAM and can be downloaded as CSV. Turn logging on as well to keep a full record on the SD card.

Endpoints: `/tune/step?p1=&p2=&max=`, `/tune/setpoint?sp=80,90,95&hold=&kp=&ki=&kd=`, `/tune/stop`, `/tune/status`, `/tune/data?from=`, `/pid/save`.

### Safety
- Over-temperature (> 130 °C) and sensor-fault shutdown, based on the PID Temp Sensor. These checks use the latest logged reading, so they only run while logging is on. Keep logging running whenever the heater is on. The AUX Temp Sensor is logged and displayed only; it does not trigger a shutdown.
- 30-minute continuous-heating limit (Ticker, every 500 ms). Under PID the duty rarely drops to 0%, so a long hold will reach this limit.
- Software watchdog (checked every 8 s) + ESP hardware watchdog
- Logging auto-stops when free heap < 8 KB
- On shutdown the heater PWM goes to 0, PID and logging stop, and any open log file is closed
- After a shutdown, use **System Reset** (`/system/reset`). It is refused if the temperature is ≥ 120 °C or the sensor is faulty. It also clears the RAM buffer so old readings can't re-trigger a shutdown. See [RESET_BUFFER_FIX.md](md_info_docs/RESET_BUFFER_FIX.md).

### Network / API
| Route | Action |
|---|---|
| `/` | Web page |
| `/start`, `/stop` | Start / stop logging (`/start` returns 503 if no log file can be created) |
| `/status` | One-shot JSON reading and settings: `temperature` (PID Temp Sensor), `auxTemperature` (AUX Temp Sensor), raw ADC counts and divider voltages |
| `/logs` | JSON list of log files `{name, size}` |
| `/data.csv` | Download the current log file (flushes the RAM buffer first) |
| `/data.csv?file=<name>` | Download a specific log file |
| `/logs/delete?file=<name>` | Delete a log file |
| `/clear` | Delete all `data*.log` files and start a new `data.log` |
| `/relay/on`, `/relay/off` | Manual heater 100% / 0% (names kept from the relay version) |
| `/temp/set?temp=X` | Set target temperature |
| `/pid/enable`, `/pid/disable` | PID on/off |
| `/pid/params?kp=&ki=&kd=` | Set PID gains |
| `/heater/status` | Heater/PID JSON |
| `/system/reset` | Clear emergency shutdown |

HTTP on port 80, WebSocket on port 81, AP IP 192.168.4.1.

## Code Layout

```
src/
├── main.cpp               # setup(), loop(), sensor sampling, debug stats
├── sensor_manager.*       # mux control, PID/AUX thermistor conversion and constants
├── data_manager.*         # RAM buffer, SD card, log files, rotation, write timeouts
├── heater_controller.*    # PWM output, PID, heater safety checks
├── safety_system.*        # Ticker safety check, watchdog, emergency shutdown
├── web_interface.*        # HTTP routes, WebSocket, page templating
├── wifi_manager.*         # access point
└── debug_config.h         # centralized debug macros (not used yet, see md_info_docs/DEBUG_MIGRATION_GUIDE.md)
data/
└── index.template.html    # web page (uploaded to LittleFS)
config.h                   # reference of key settings (not compiled in)
```

Debug output is set per file: `main.cpp`, `heater_controller.cpp` and `web_interface.cpp` each define their own `DEBUG_*` flags.

## Troubleshooting

### "SD card initialization failed" / "SD card not found"
- Check the SPI wiring: CS=D8, MOSI=D7, MISO=D6, SCK=D5, VCC=3.3 V
- Format the card FAT32 (or FAT16 for ≤ 2 GB)
- Make sure the module works at 3.3 V
- If the board won't boot with the SD module attached, D8 (GPIO15) must be LOW at power-up
- If the card wasn't ready at boot, insert it and press Start Logging, or reboot

### "Slow SD write" or "Write timeout" in the serial monitor
- Use a faster card (Class 10 / UHS-1) and reformat it FAT32

### Web page doesn't load or says "Template file not found"
- The web page lives in internal flash. Run `make filesystem` (`pio run -t uploadfs`).

### Heater doesn't turn on
- Check the MOSFET: source to GND, drain to heater (-), heater (+) to 12 V
- Measure the gate: ~3.3 V at 100% duty
- Check for an active emergency shutdown in the serial monitor, and use System Reset

### Temperature reads N/A or -999
- PID Temp Sensor: check the divider on C1. At 25 °C it is ~1.68 V unloaded, ~1.45 V while the ESP8266 samples it (a meter shows something in between). A disconnected PID thermistor reads -999 and trips the sensor-fault shutdown if the heater is on.
- AUX Temp Sensor: check the divider on C2 (~2.75 V at 25 °C, ~1.7 V at 65 °C). With nothing connected to C2 it reads N/A, and logging continues.

### PID and AUX Temp Sensors disagree
- They measure different spots, so some difference is expected. In air the AUX Temp Sensor also self-heats by ~1 °C at room temperature (calibration was done in water, where self-heating is negligible), so it reads high there. To check the calibration, put both sensors in the same stirred water: they should agree within a few tenths of a °C. If they don't, check `AUX_SERIES_RESISTOR` matches the resistor fitted and recalibrate (see Calibration).

For PlatformIO install/build problems, see [TROUBLESHOOTING.md](md_info_docs/TROUBLESHOOTING.md).

## Notes

- The voltage channel is scaled as ADC/1024 (0-1 V). On NodeMCU / D1 mini boards, A0 has an onboard divider that makes its range 0-3.2 V, so the logged value is the actual voltage ÷ 3.2. The thermistor conversions model this (see "How the temperatures are calculated"). Because `voltage_adc` is logged, the actual voltage can be recovered afterwards (`voltage_adc` / 1024 × 3.2 on those boards).
- Timestamps restart at 0 on each boot. Each boot also starts a new log file.
- Further guides and change notes from earlier versions (relay control, internal-flash logging) are in `md_info_docs/`. Each change note starts with a note saying what has changed since. This README, WIRING_GUIDE.md and config.h describe the current firmware.

## License

This project is open source. Feel free to modify and distribute.
