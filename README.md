# ESP8266 Temperature Logger with SD Card and PWM Heater Control (PCB version)

A data logger built on the ESP8266. It reads two NTC thermistors with an MCP3208 12-bit SPI ADC, logs them to a microSD card, and drives a heater through a MOSFET with a PWM PID controller. Everything is controlled from a web page served over the ESP8266's own WiFi access point.

**This branch (`pcb_update`) is for the `pcb_voltage_logging` PCB** (NodeMCU v2 in a socket, MCP3208, on-board microSD, WS2812B status LED). The earlier breadboard build (CD74HC4067 mux into A0, voltage input on channel 0) stays on the `main` branch. Fabrication notes are in [PCB_IMPLEMENTATION.md](PCB_IMPLEMENTATION.md); changes the next board revision needs are in [PCB_REVISIONS.md](PCB_REVISIONS.md).

## Features

- **Sensor logging**: **PID Temp Sensor** (MCP3208 CH1, J4, controls the heater) and **AUX Temp Sensor** (CH2, J5, a second thermistor at another location, normally on the sample). Either sensor can be the one the PID controls, selected on the web page; the safety checks always use the PID Temp Sensor
- **SD card storage**: CSV-formatted log files on a microSD card, a new file every boot, automatic rotation at 50 MB
- **Log file manager**: list, download and delete log files from the web page
- **PWM heater control**: AOD4144 N-channel MOSFET driven at 0-100% duty from GPIO16
- **PID temperature control**: runs every 500 ms, independent of data logging
- **Real-time web interface**: WebSocket updates, a live plot and statistics. The plot is drawn by the page itself (no internet needed): PID and AUX temperatures, the setpoint while the PID is on, and heater power on the right axis, over a 2 min to 6 h window, downloadable as CSV
- **PID tuning page** (`/tune`): power step and setpoint tests with suggested gains (see Heater / PID)
- **Setpoint program page** (`/program`): run an experiment through a list of up to 100 setpoints, each held for a set time, with logging on (see Setpoint program)
- **WiFi access point**: no router needed
- **Safety system**: over-temperature, sensor-fault and max-runtime shutdown, software and hardware watchdogs, SD write timeouts, low-memory guard, and a web "System Reset" to recover
- **Status LED** (WS2812B): off when the heater is off, blue → red with heater power, flashing orange after a heater timeout, flashing yellow after any other emergency shutdown

## Hardware

### The PCB (`pcb_voltage_logging`)
| Ref | Part | Function |
|---|---|---|
| U1 | NodeMCU v2 (in 0.9″ socket rows) | ESP8266, USB, 3.3 V regulator |
| U2 | MCP3208-CI/SL | 12-bit 8-channel SPI ADC. VREF = VDD = +3V3 |
| J4 / R1 / C8 | 2-pin JST PH, 100k, 100 nF | PID Temp Sensor input (CH1) |
| J5 / R2 / C9 | 2-pin JST PH, 2k, 100 nF | AUX Temp Sensor input (CH2) |
| J6 | Hirose DM3AT microSD (bottom side) | Log storage, shares SPI with U2. No card-detect wired |
| R9 / R10 / R11 | 33 Ω, 33 Ω, 47 Ω | Series resistors on SCK, MOSI and SD CS |
| R5 / R6 / R7 | 10k | Pull-ups on SD DO (MISO), DAT1, DAT2 |
| R12 | 10k | Pull-up on the MCP3208 CS |
| D2 / U3 / R8 | WS2812B, 74AHCT1G125, 330 Ω | Status LED, data from RX/GPIO3 level-shifted to 5 V |
| Q1 / R4 / R3 | AOD4144, 100 Ω, 10k | Heater low-side switch, gate resistor, gate pull-down |
| D1 | SS34 | Flyback across the heater |
| F1 | 1.5 A hold PTC | Heater supply fuse |
| J1 / J2 | Screw terminals | 12 V in / heater out |
| J7 | 1×7 header | MCP3208 CH3–CH7 (spare, not used yet), +3V3, GND |

**Power:** the logic (ESP8266, MCP3208, SD, LED) runs from the NodeMCU's USB. The 12 V input on J1 only feeds the heater. There is no 5 V regulator on this revision (see [PCB_REVISIONS.md](PCB_REVISIONS.md)), so keep USB connected.

### Sensors
- **PID Temp Sensor** (J4): 100 kΩ NTC thermistor, β = 3950 K (Ender 3 compatible)
- **AUX Temp Sensor** (J5): MF55 ultra-thin 10 kΩ NTC thermistor, β = 3950 K (B25/50)

The AUX series resistor (2 kΩ) is sized for resolution around 60-70 °C, where the 10k NTC measures about 2.5-1.8 kΩ. With ~2 kΩ the NTC dissipates ~1.4 mW at 65 °C, which can warm it by up to ~1.9 °C in still air (0.7 mW/°C), and less when it is pressed against a surface.

| Resolution at 65 °C | Old board (A0, 10-bit) | PCB (MCP3208, 12-bit) |
|---|---|---|
| PID Temp Sensor | ~5 counts/°C | ~20 codes/°C |
| AUX Temp Sensor | ~9.5 counts/°C | ~37 codes/°C |

### How the temperatures are calculated
Each NTC divider runs from +3V3, and the same +3V3 is the MCP3208's reference, so the reading is **ratiometric**:
```
code / 4096 = R_ntc / (Rs + R_ntc)      ->   R_ntc = Rs × code / (4096 − code)
1/T = 1/298.15 + ln(R_ntc / R25) / B     (T in kelvin)
```
The rail voltage cancels out, and the MCP3208 input draws only nA, so there is no loading correction (the old board's A0 put 320 kΩ across the divider and needed a model for it). The 100 nF caps (C8/C9) hold each node steady while the ADC samples. A code ≥ 4094 means an open sensor (pulled up to +3V3 through Rs), a code ≤ 1 a short, and an NTC resistance above 20 × R25 (colder than about −30 °C) is also treated as open. All of these log -999, which trips the sensor-fault shutdown if the heater is on.

The constants are in `src/sensor_manager.cpp`: `SERIES_RESISTOR` (R1), `AUX_SERIES_RESISTOR` (R2), and each sensor's R25 and B.

### Calibration
**The PCB is not calibrated yet.** The R25/B values in `src/sensor_manager.cpp` are the old board's fits (PID 97 886 Ω / 3910, AUX 10 631 Ω / 4096). Those fits also absorbed the old ESP8266 ADC's errors and its 320 kΩ load model, so on the PCB expect a degree or two of error until it is recalibrated:

1. With the board unpowered and the sensors unplugged, measure R1 and R2 and set `SERIES_RESISTOR` / `AUX_SERIES_RESISTOR`.
2. Log with **Average** on in a stirred ice bath (0 °C) and in boiling water (~95.7 °C at Salt Lake City's ~1300 m), both sensors suspended mid-depth.
3. Average each settled stretch and fit:
```
python3 tools/calibrate_thermistors.py --average ice.log --timeline       # find the flat part
python3 tools/calibrate_thermistors.py --average ice.log --from 120 --to 300
python3 tools/calibrate_thermistors.py --elevation 1300                   # boiling point for your elevation
python3 tools/calibrate_thermistors.py --point 0:PID:AUX --point 95.7:PID:AUX
```
The tool reads the series resistors from `src/sensor_manager.cpp`, checks the log's `adc_fs` column so old 10-bit logs aren't mixed in, and prints the new R25/B lines.

The MCP3208 is linear (±1 LSB), unlike the ESP8266 ADC (~1.7 % high at mid-scale), so a two-point fit should hold much better between the points than it did on the old board.

### Hotplate correction (off on the PCB)
`PID_CORRECTION` / `AUX_CORRECTION` in `src/sensor_manager.cpp` are the old board's hotplate tables (2026-10-28). Part of what they corrected was the ESP8266 ADC's non-linearity, which the PCB doesn't have, so `APPLY_TEMPERATURE_CORRECTION` is `false`. Re-measure on the PCB after the ice/boiling calibration and turn it back on only with new tables. The difference between the PID and AUX sensors in use (e.g. 67 °C on the sheet against 54.5 °C on the sample) is heat flow through the sapphire, not ADC error, so it will not go away with the new ADC.

**Accuracy:** until the PCB is recalibrated, treat absolute temperatures as about ±2 °C. The old board reached about ±1.5 °C absolute and a few tenths of a °C between the two sensors.

### Status LED (WS2812B)
| LED | Meaning |
|---|---|
| Off | Heater off (no PID, no manual power) |
| Blue → red | Heater on: blue = 0 % PWM, red = 100 %, purple in between |
| Flashing orange | Emergency shutdown: the heater ran longer than `MAX_HEATER_TIME` (6 h) |
| Flashing yellow | Emergency shutdown for any other reason (over-temperature, sensor fault, watchdog) |

The LED is on RX/GPIO3, so the serial port is output-only (`SERIAL_TX_ONLY`); the serial monitor still works. It may flicker random colours while the board is being flashed. Brightness is `LED_BRIGHTNESS` in `src/status_led.cpp`.

## Wiring Overview

```
NodeMCU        PCB net               Function
-------        -------               --------
D0 (GPIO16)    HEATER_PWM            Q1 gate via R4 100 Ω (R3 10k pull-down)
D1 (GPIO5)     ADC_CS                MCP3208 /CS (R12 10k pull-up)
D2, D3, D4     -                     Not connected (D3/D4 are boot pins; left floating)
D5 (GPIO14)    SPI_SCK               R9 33 Ω -> MCP3208 CLK + SD CLK
D6 (GPIO12)    SD_MISO               MCP3208 DOUT + SD DO (R5 10k pull-up)
D7 (GPIO13)    SPI_MOSI              R10 33 Ω -> MCP3208 DIN + SD CMD
D8 (GPIO15)    SD CS                 R11 47 Ω -> SD DAT3/CS
RX (GPIO3)     LED_DATA              U3 74AHCT1G125 -> R8 330 Ω -> WS2812B DIN
TX (GPIO1)     -                     USB serial output only
A0             -                     Not connected
3V3            +3V3                  MCP3208 VDD/VREF, SD, NTC dividers, J7
VIN            +5V_VIN               WS2812B, U3 (5 V from USB through the NodeMCU)

MCP3208 channels:
  CH0      not connected (floating: never logged)
  CH1  ->  PID Temp Sensor: +3V3 --[R1 100k]-- CH1 --[100k NTC on J4]-- GND   (C8 100 nF to GND)
  CH2  ->  AUX Temp Sensor: +3V3 --[R2 2k]---- CH2 --[10k NTC on J5]--- GND   (C9 100 nF to GND)
  CH3-7 -> J7 pins 5..1 (reversed: J7 pin 1 = CH7), spare
```

See [WIRING_GUIDE.md](WIRING_GUIDE.md) for connectors, the heater circuit and bring-up checks.

## Software Setup

### PlatformIO (recommended)
```bash
pip install platformio
make deploy                   # build + upload firmware + upload web page + open monitor
```

Or step by step:
```bash
pio run -e esp8266 -t upload       # firmware
pio run -e esp8266 -t uploadfs     # web pages (data/ folder: main page, /tune, /program)
pio device monitor -b 115200
```

Both steps are needed: the firmware and the web page are uploaded separately.

### Libraries
- ESP8266 Arduino core (ESP8266WiFi, ESP8266WebServer, SD, SPI, LittleFS, Ticker)
- ArduinoJson ^7.0.0
- WebSockets ^2.4.0 (Markus Sattler)
- NeoPixelBus ^2.8.0 (Makuna), for the status LED

PlatformIO installs the external libraries automatically from `platformio.ini`.

### What the two storage areas hold
| Storage | Holds | How it gets there |
|---|---|---|
| **microSD card** | All sensor data logs (`data*.log`) | Written by the firmware while logging |
| Internal flash (LittleFS) | The web pages (`data/index.template.html`, `data/tune.html`, `data/program.html`), the saved PID settings (`/pid.json`) and the last setpoint program (`/program.json`), both written by the board | Pages: `pio run -t uploadfs` / `make filesystem`. This also erases `/pid.json` and `/program.json` |

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
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,aux_temp,aux_adc,aux_v,samples,control,step,adc_fs
64659,0.000000,20.404,0,45.00,0.00,0,2236,1.8123,0,2.300,0.045,0.000,20.928,3545,2.8731,1,0,0,4096
259599,0.000000,40.939,1,45.00,9.08,0,1371,1.1113,1,2.300,0.045,0.000,39.606,3018,2.4459,1,0,0,4096
3391012,0.000000,42.844,0,45.00,0.00,0.00,1303.47,1.0565,0,2.300,0.045,0.000,42.821,2908.50,2.3575,32,0,0,4096
```
The first two rows are raw (`samples` = 1). The third is an averaged row (32 sample cycles), so its ADC columns are mean codes with 2 decimals.

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | timestamp | ms | `millis()` since boot, taken at the PID Temp Sensor sample |
| 2 | voltage | V | Always 0 on the PCB (no voltage input). Kept so scripts for the old logs still parse |
| 3 | temperature | °C | **PID Temp Sensor**, MCP3208 CH1 (3 decimals, -999 = open/short) |
| 4 | heater_state | 0/1 | 1 if heater PWM duty > 0 |
| 5 | target_temp | °C | PID setpoint |
| 6 | pid_output | % | Heater PWM duty (PID output, or 100 in manual ON) |
| 7 | voltage_adc | codes | Always 0 on the PCB (kept for the column layout) |
| 8 | thermistor_adc | codes | MCP3208 code for CH1, PID Temp Sensor (0-4095). Mean of the row's sample cycles when averaged (2 decimals) |
| 9 | thermistor_v | V | PID Temp Sensor node voltage, `thermistor_adc`/4096 × 3.32 (4 decimals). For reference only: the temperature is calculated ratiometrically from the code, not from this |
| 10 | pid_enabled | 0/1 | 1 if the PID was controlling the heater, 0 for manual ON/OFF. Tells PID-at-100% apart from manual ON. |
| 11 | kp | - | PID proportional gain in effect (3 decimals) |
| 12 | ki | - | PID integral gain in effect |
| 13 | kd | - | PID derivative gain in effect |
| 14 | aux_temp | °C | **AUX Temp Sensor**, MCP3208 CH2 (3 decimals, -999 = open/short or not connected) |
| 15 | aux_adc | codes | MCP3208 code for CH2, AUX Temp Sensor (0-4095). Mean when averaged |
| 16 | aux_v | V | AUX Temp Sensor node voltage, `aux_adc`/4096 × 3.32 (4 decimals), for reference |
| 17 | samples | cycles | Sample cycles in the row: 1 = raw, N = averaged (see Averaging) |
| 18 | control | 0/1 | Sensor the PID controls: 0 = PID Temp Sensor, 1 = AUX Temp Sensor. `target_temp` applies to it |
| 19 | step | - | Setpoint program step (1-100) when the row was sampled; 0 = no program running |
| 20 | adc_fs | codes | ADC full scale for the `*_adc` columns: 4096 (MCP3208). Old-board logs have no `adc_fs` column and are 0-1023 |

Columns 1-6 are the original format. Columns 7-20 were added after them, so scripts that read by column name keep working. To tell old and new logs apart, check for the `adc_fs` column.

A raw row is one conversion of CH1 and one of CH2, taken back to back (~30 µs apart).

### Averaging (oversampling)
The **Average** checkbox under Data Logging Control sets what each row holds:
- **Off (default):** every sample cycle is logged raw, as above, up to ~167 rows/s. `samples` = 1.
- **On:** N consecutive sample cycles (default 32, settable 2-100 next to the checkbox) are combined into one row. The ADC codes are averaged, and `temperature`, `aux_temp` and the `*_v` columns are calculated from the mean codes. Averaging the codes (not the temperatures) lets the ADC noise resolve fractions of a code. `timestamp` is the middle of the window. `heater_state`, `pid_output`, `target_temp` and `pid_enabled` are from the last cycle in it. `samples` = N.

With N = 32 a row covers ~0.2 s, at up to ~5 rows/s. Averaging improves resolution and noise, but not accuracy (see Calibration). The setting can change while logging (a partly filled average is dropped), and it resets to off at boot. It only affects the log: the PID always uses its own average of every sample cycle (see Sampling). `/averaging?enabled=1&n=32` sets it over HTTP, and `/status` reports `averagingEnabled` and `averageSamples`.

**Checking the temperatures:** both use the ratiometric formula from "How the temperatures are calculated":
```
R = Rs × code / (4096 − code)
1/T = 1/298.15 + ln(R/R25)/B      (T in kelvin)
```
- PID Temp Sensor: code = `thermistor_adc`, Rs = `SERIES_RESISTOR`, R25 / B = `THERMISTOR_NOMINAL` / `B_COEFFICIENT`
- AUX Temp Sensor: code = `aux_adc`, Rs = `AUX_SERIES_RESISTOR`, R25 / B = `AUX_THERMISTOR_NOMINAL` / `AUX_B_COEFFICIENT`

That reproduces `temperature` and `aux_temp` to within a few thousandths of a °C (plus the hotplate correction, if it is ever turned back on). The raw codes let you redo the conversion with a later calibration.

**Old-board logs** (no `adc_fs` column) are 10-bit A0 counts read through the 320 kΩ A0 load. Convert them with the formulas in the `main` branch's README, not these.

**Gains:** the kp/ki/kd columns hold the gains when the row was sampled. Changing them with `/pid/params` saves the buffered rows first, so rows never show gains they weren't sampled under. The serial monitor warns if that save fails.

With averaging off, the maximum rate is about 110 bytes × 167 rows/s ≈ 66 MB per hour (a 6 h program ≈ 400 MB, in 50 MB files). With averaging on at N = 32 it is about 2 MB per hour, which is the better choice for long runs.

## Technical Details

### Sampling
- One sample cycle every 6 ms: one MCP3208 conversion of each sensor (~60 µs at 1 MHz SPI). Up to ~167 cycles/s, so ~167 CSV rows/s raw, or ~167/N rows/s with averaging on
- Sampling runs all the time, logging or not. Each cycle updates the live readings used by the safety checks, and is added to the PID's input average
- **PID input:** every PID update (500 ms) uses the mean of all the cycles since the last one (~83), which cuts the noise ~9× against a single sample. A window that contains an open/shorted reading counts as a sensor fault rather than being averaged in. Logging faster would not help the PID: the plant responds over seconds to minutes, so the row rate is set by SD card space instead
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
| Kp / Ki / Kd | Controlling on the PID Temp Sensor: 2.3 / 0.045 / 0. On AUX: 1.0 / 0.008 / 0. Interim values from a model; replaced by gains saved from the tuning page |
| Control sensor | PID Temp Sensor (default) or AUX Temp Sensor, selectable on the main page and `/tune` |
| Sheet limit (AUX control) | 120 °C default (40–125 °C); heater power tapers to 0 over the last 5 °C |
| PID interval | 500 ms |
| Anti-windup | Integral term limited to the 0–100 % output range, and frozen while the output is saturated |
| Derivative | On the measurement (no kick when the setpoint changes) |
| Default target | 45 °C |
| Max safe temperature | 130 °C |
| Max continuous heater time | 6 h (`MAX_HEATER_TIME`) |

The PID uses the mean of the sample cycles since its last update (or a 16-conversion direct read if none came in), so it works the same with logging on or off. Its output (0-100%) sets the PWM duty directly. The ADC resolution and noise floor are better than on the old board, so re-run a `/tune` step test on the PCB; the old gains are a reasonable start.

### Choosing the control sensor
The **Control sensor** setting under Temperature Control picks which thermistor the PID holds at the setpoint. You can switch at any time without an upload, so you can try either location for a sample.
- **PID Temp Sensor (sheet, default):** controls the temperature next to the heater, which responds quickly. The sample sits below the setpoint by however much the fixture drops (~20–25 °C near 70 °C on the 2026-10-26 run).
- **AUX Temp Sensor (sample):** holds the sample itself at the setpoint. The sheet runs hotter to get there, so a **sheet limit** (default 120 °C) caps it. Heater power tapers to 0 over the last 5 °C below the limit. It is a soft limit: in a fast rise the sheet can pass it by ~2 °C (its reading lags the real temperature by a few seconds), then settles just below it. The 130 °C over-temperature shutdown still applies. If the sheet limit stops the sample reaching its setpoint, the page shows "Sheet limit reached". If either sensor fails while controlling on AUX, the PID stops and the heater turns off, and the page shows why.
- **Each sensor has its own gains.** The sample responds far more slowly than the sheet (on the model, ~27 s delay and ~130 s time constant against ~3 s and ~60 s), so it needs gentler gains. Switching swaps them in, and the integral restarts. If the PID is running, the page asks before switching, because the setpoint then applies to the other sensor.
- The setting is lost at reboot unless saved: **Save sensor, gains + limit as boot defaults** (or Save on `/tune`) stores both gain sets, the control sensor and the sheet limit in `/pid.json`.
- Every log row records the control sensor in the `control` column (0 = PID Temp Sensor, 1 = AUX), and `target_temp` applies to that sensor.

Before 2026-10-29 the integral was clamped to ±100, which with Ki = 0.25 capped it at 25 % output. Above ~60 °C the sheet needs more power than that to hold, so it settled several degrees below the setpoint (e.g. 88 °C for a 92 °C setpoint). With the old gains (2.2 / 0.25 / 0.15) the corrected PID overshoots ~10 °C, so the defaults changed and the gains should be tuned on the fixture.

### Setpoint program (`/program`)
For experiments: run through a list of setpoints automatically, logging everything. Open `http://192.168.4.1/program` (or the link on the main page).

- **The list:** up to 100 steps, one per line: a setpoint in °C, optionally followed by its own hold time in minutes (`62.5`, `62.5 15`, `62.5, 15` or `62.5:15`; 0.5–360 min). Lines without a time use the **default hold**. The total must fit within the 6 h heater limit.
- **Generate steps** *appends* a run from a start to an end setpoint in a given step size (optionally back down again) to the end of the list, so a pattern can be built from pieces, e.g. 60→70 in 0.5 °C steps, then 70→65, then 65→72. If the first new setpoint equals the last one in the list it is skipped, so a turn-around point isn't held twice. **Clear the list** empties it.
- **Timing only:** each step lasts exactly its hold time from when it starts. The program doesn't wait for the temperature to settle; it just changes the setpoint when the time is up.
- **What Start does:** turns on SD logging (into the current log file, with the main page's settings such as averaging), sets the first setpoint and turns the PID on with the selected **control sensor** (PID or AUX) and its gains. The steps then run on the board, so the page can be closed. Choose the control sensor, gains and averaging on the main page before starting; PID gains can be adjusted while it runs.
- **At the end:** either turn the heater off or keep holding the last setpoint until stopped, and optionally stop logging.
- **It stops (heater off, logging left as it was)** on Stop, if the heater, PID, setpoint or control sensor is changed from the main page, if logging stops (including the low-memory auto-stop), if the PID stops itself (a sensor fault) or on an emergency shutdown. A tuning test and a program can't run at the same time.
- **In the log:** the `step` column (19) gives the running step number (0 outside a program), and `target_temp` the setpoint, so each step can be cut out of the file afterwards.
- The page shows the current step, time left, the step list with progress, the current logging/control settings, and a live plot from when the page was opened. The last program started is saved on the board (`/program.json`) and reloaded into the editor.

### PID tuning page (`/tune`)
Open `http://192.168.4.1/tune` (or the link on the main page), with the whole fixture set up as for a measurement. Close the main page while tuning: each open page keeps its own connection to the board. The tests run on the board, so a dropped Wi-Fi link can't leave the heater on.

Tests and the gain fit use the selected control sensor (switch it at the top of the page). Tune each sensor once and save.

1. **Power step test (PID off).** The heater holds a start power until the selected sensor settles (at least 2 min, trend flatter than 0.15 °C/min), steps to a second power, records the response until it settles, then turns off. Pick powers around the working range: about 25 → 35 % holds the sheet near 85–100 °C and the sample near 60–70 °C. Allow 15–20 min per phase for AUX.
2. **Model and gains.** The page fits a delay, time constant and gain (°C per % power) to the step response, and suggests PI gains (Skogestad SIMC rule) for three speeds. "Critically damped (recommended)" aims for no overshoot.
3. **Setpoint test (PID on).** Runs the PID with chosen gains through up to 4 setpoints, holding each, then turns the heater off. It reports overshoot, settling time (±0.5 °C on a 10 s average) and the remaining error for each step.
4. **Apply / Save.** "Apply" sets the selected sensor's running gains. "Save" also stores both sensors' gains, the control sensor and the sheet limit on the board's flash (`/pid.json`), loaded at boot. Uploading the filesystem image (`make filesystem` or `make deploy`) erases them: note them first. `make upload` (firmware only) keeps them.

Safety during tests: the heater turns off if the sheet passes 120 °C, the PID sensor (or, when tuning on AUX, the AUX sensor) faults, an emergency shutdown happens, a test runs over 55 min, or the heater/PID is changed from the main page. "Stop test" turns the heater off at any time. Test data (1 s samples, PID and AUX temperatures, heater power) is kept in RAM and can be downloaded as CSV. Turn logging on as well to keep a full record on the SD card.

Endpoints: `/tune/step?p1=&p2=&max=` (max 3–20 min per phase), `/tune/setpoint?sp=80,90,95&hold=&kp=&ki=&kd=` (hold 1–15 min, at most 54 min in total), `/tune/stop`, `/tune/status`, `/tune/data?from=`, `/pid/save`, `/pid/sensor?s=pid|aux&limit=`.

### Safety
- Over-temperature (> 130 °C) and sensor-fault shutdown, based on the PID Temp Sensor's live sample (every 6 ms). These now run whether or not logging is on (on the old board they only ran while logging). The AUX Temp Sensor does not trigger a shutdown, but while it is the control sensor a fault on it stops the PID and turns the heater off.
- 6-hour continuous-heating limit (`MAX_HEATER_TIME`, Ticker every 500 ms). Under PID the duty rarely drops to 0%, so heating for longer than 6 hours reaches this limit and shuts down. Raised from 30 min (then 1 h, then 3 h) for long setpoint programs (e.g. ~70 half-degree steps of 5 min up and back down). The main page, `/program` and the boot message all read it from the firmware.
- Software watchdog (checked every 8 s) + ESP hardware watchdog
- Logging auto-stops when free heap < 8 KB
- On shutdown the heater PWM goes to 0, PID and logging stop, and any open log file is closed
- The status LED flashes orange after a heater timeout and yellow after any other shutdown. `/status` reports `emergencyShutdown` and `shutdownReason`, and the serial debug print shows the reason.
- After a shutdown, use **System Reset** (`/system/reset`). It is refused if the temperature is ≥ 120 °C or the sensor is faulty. It also clears the RAM buffer and replaces the live reading with a fresh one, so old readings can't re-trigger a shutdown. See [RESET_BUFFER_FIX.md](md_info_docs/RESET_BUFFER_FIX.md).

### Network / API
| Route | Action |
|---|---|
| `/` | Main web page |
| `/tune` | PID tuning page; tests via `/tune/step`, `/tune/setpoint`, `/tune/stop`, `/tune/status`, `/tune/data`; `/pid/save` stores the gains |
| `/averaging?enabled=&n=` | Averaging on/off and cycles per row |
| `/pid/sensor?s=pid\|aux&limit=` | Select the control sensor and/or the sheet limit; returns the gains now in use |
| `/program` | Setpoint program page; runs via `/program/start?steps=60:10,61,62:5&hold=10&end=off\|hold&stoplog=1`, `/program/stop`, `/program/status`, `/program/steps` (current list), `/program/saved` (last list started) |
| `/start`, `/stop` | Start / stop logging (`/start` returns 503 if no log file can be created) |
| `/status` | One-shot JSON reading (16 conversions per sensor) and settings: `temperature` (PID Temp Sensor), `auxTemperature` (AUX Temp Sensor), raw MCP3208 codes, divider voltages, `emergencyShutdown`, `shutdownReason` |
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
├── sensor_manager.*       # MCP3208 SPI reads, PID/AUX thermistor conversion and constants
├── status_led.*           # WS2812B status LED (NeoPixelBus, GPIO3)
├── data_manager.*         # RAM buffer, SD card, log files, rotation, write timeouts
├── heater_controller.*    # PWM output, PID, heater safety checks
├── safety_system.*        # Ticker safety check, watchdog, emergency shutdown
├── tuning.*               # PID tuning tests (power step, setpoint), saved gains
├── program.*              # setpoint program (experiment) runner
├── web_interface.*        # HTTP routes, WebSocket, page templating
├── wifi_manager.*         # access point
└── debug_config.h         # centralized debug macros (not used yet, see md_info_docs/DEBUG_MIGRATION_GUIDE.md)
data/
├── index.template.html    # main web page (uploaded to LittleFS)
├── tune.html              # PID tuning page (uploaded to LittleFS)
└── program.html           # setpoint program page (uploaded to LittleFS)
config.h                   # reference of key settings (not compiled in)
```

Debug output is set per file: `main.cpp`, `heater_controller.cpp` and `web_interface.cpp` each define their own `DEBUG_*` flags.

## Troubleshooting

### "SD card initialization failed" / "SD card not found"
- Push the card fully into J6 until it clicks (push-push socket; there is no card-detect, so the firmware can't tell it is missing)
- Format the card FAT32 (or FAT16 for ≤ 2 GB)
- If the board won't boot with a card in, D8 (GPIO15) must be LOW at power-up: check the NodeMCU's own pull-down on D8
- If the card wasn't ready at boot, insert it and press Start Logging, or reboot
- If temperatures go wrong only after SD writes, the card is holding the shared MISO line: try another card

### "Slow SD write" or "Write timeout" in the serial monitor
- Use a faster card (Class 10 / UHS-1) and reformat it FAT32

### Web page doesn't load or says "Template file not found"
- The web page lives in internal flash. Run `make filesystem` (`pio run -t uploadfs`).

### Heater doesn't turn on
- Check the MOSFET: source to GND, drain to heater (-), heater (+) to 12 V
- Measure the gate: ~3.3 V at 100% duty
- Check for an active emergency shutdown in the serial monitor, and use System Reset

### Temperature reads N/A or -999
- PID Temp Sensor: measure the top pad of C8. At 25 °C it is ~1.64 V; with nothing plugged into J4 it is ~3.3 V (code ~4095, logged as -999). A disconnected PID thermistor trips the sensor-fault shutdown if the heater is on.
- AUX Temp Sensor: measure C9 (~2.79 V at 25 °C, ~1.7 V at 65 °C). With nothing on J5 it reads N/A, and logging continues.
- Both -999 at once: check the MCP3208 (3.3 V on U2 pins 15/16, CS on D1). The boot message prints the raw codes of both channels.

### PID and AUX Temp Sensors disagree
- They measure different spots, so some difference is expected. In air the AUX Temp Sensor also self-heats by ~1 °C at room temperature (calibration was done in water, where self-heating is negligible), so it reads high there. To check the calibration, put both sensors in the same stirred water: they should agree within a few tenths of a °C. If they don't, check `AUX_SERIES_RESISTOR` matches the resistor fitted and recalibrate (see Calibration).

For PlatformIO install/build problems, see [TROUBLESHOOTING.md](md_info_docs/TROUBLESHOOTING.md).

## Notes

- The PCB has no voltage input. The `voltage` and `voltage_adc` columns are still written (as 0) so analysis scripts written for the old logs keep working.
- Timestamps restart at 0 on each boot. Each boot also starts a new log file.
- Further guides and change notes from earlier versions (relay control, internal-flash logging) are in `md_info_docs/`. Each change note starts with a note saying what has changed since. This README, WIRING_GUIDE.md and config.h describe the current (PCB) firmware; the other notes in `md_info_docs/` mostly describe the old board.

## License

This project is open source. Feel free to modify and distribute.
