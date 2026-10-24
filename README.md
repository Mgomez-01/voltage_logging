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
- **100 kΩ resistor** for the PID Temp Sensor divider
- **2.0 kΩ resistor** for the AUX Temp Sensor divider (see below)

### Sensors
- **Voltage source** to measure (0-1 V scale in software, see Notes)
- **PID Temp Sensor**: 100 kΩ NTC thermistor, β = 3950 K (Ender 3 compatible)
- **AUX Temp Sensor**: MF55 ultra-thin 10 kΩ NTC thermistor, β = 3950 K (B25/50)

The AUX series resistor is sized for resolution around 60-70 °C, where the 10k NTC measures about 2.5-1.8 kΩ. That gives ~9 ADC counts per °C (~0.11 °C per count); anything from 2.0 kΩ to 2.7 kΩ gives about the same there. With ~2 kΩ the NTC dissipates ~1.4 mW at 65 °C, which can warm it by up to ~1.9 °C in still air (0.7 mW/°C), and less when it is pressed against a surface. A 2.7 kΩ resistor cuts that by about a quarter. `AUX_SERIES_RESISTOR` in `src/sensor_manager.cpp` must match the resistor fitted.

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
  C1 -> PID Temp Sensor: 3.3V --[100k]-- C1 --[100k NTC]-- GND
  C2 -> AUX Temp Sensor: 3.3V --[2.0k]-- C2 --[10k NTC]--  GND
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
pio run -e esp8266 -t uploadfs     # web page template (data/ folder)
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
| Internal flash (LittleFS) | Only the web page template (`data/index.template.html`) | `pio run -t uploadfs` / `make filesystem` |

Sensor data is never stored in internal flash.

## Usage

1. **Insert the SD card** and power the board. The serial monitor should show `SD card initialized.` and `Created new data file: data_N.log`.
2. **Connect to WiFi**: SSID `ESP8266_VoltageLogger`, password `voltage123`.
3. **Open** `http://192.168.4.1`.
4. **Start Logging**. Logging is paused at boot. If the SD card wasn't ready at boot, Start Logging tries to set it up again and shows an error if it still can't create a log file.
5. **Heater control** (optional):
   - Set the target temperature (0-130 °C, default 45 °C)
   - Enable PID, or use manual ON (100%) / OFF (0%)
   - Keep logging running while the heater is on (see Safety)
6. **Stop Logging** to flush the RAM buffer to the card.
7. **Download Data** gets the current log file. The **Log Files on SD Card** panel lists every file so you can download or delete them individually.

## Data Format

Each boot creates a new file: `data.log`, then `data_1.log`, `data_2.log`, ... When a file reaches 50 MB, logging continues in the next free name. The files are CSV text:

```csv
timestamp,voltage,temperature,heater_state,target_temp,pid_output,voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,aux_temp,aux_adc,aux_v
120004,0.822266,25.000,1,45.00,18.40,842,512,1.6500,1,2.200,0.250,0.150,25.053,853,2.7489
120010,0.823242,25.088,1,45.00,18.40,843,511,1.6468,1,2.200,0.250,0.150,25.210,852,2.7457
```

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | timestamp | ms | `millis()` since boot, taken at the PID Temp Sensor sample |
| 2 | voltage | V | Channel 0, `voltage_adc`/1024 (6 decimals) |
| 3 | temperature | °C | **PID Temp Sensor**, channel 1 (3 decimals, -999 = open/short) |
| 4 | heater_state | 0/1 | 1 if heater PWM duty > 0 |
| 5 | target_temp | °C | PID setpoint |
| 6 | pid_output | % | Heater PWM duty (PID output, or 100 in manual ON) |
| 7 | voltage_adc | counts | Raw ADC reading for channel 0 (0-1023) |
| 8 | thermistor_adc | counts | Raw ADC reading for channel 1, PID Temp Sensor (0-1023) |
| 9 | thermistor_v | V | PID Temp Sensor divider voltage, `thermistor_adc`/1024 × 3.3 (4 decimals). This is the exact value `temperature` is calculated from. |
| 10 | pid_enabled | 0/1 | 1 if the PID was controlling the heater, 0 for manual ON/OFF. Tells PID-at-100% apart from manual ON. |
| 11 | kp | - | PID proportional gain in effect (3 decimals) |
| 12 | ki | - | PID integral gain in effect |
| 13 | kd | - | PID derivative gain in effect |
| 14 | aux_temp | °C | **AUX Temp Sensor**, channel 2 (3 decimals, -999 = open/short or not connected) |
| 15 | aux_adc | counts | Raw ADC reading for channel 2, AUX Temp Sensor (0-1023) |
| 16 | aux_v | V | AUX Temp Sensor divider voltage, `aux_adc`/1024 × 3.3 (4 decimals). This is the exact value `aux_temp` is calculated from. |

Columns 1-6 are the original format. Columns 7-16 were added after them, so scripts that read by column name keep working.

Each row is one pass over channels 0 → 1 → 2, so its three readings are taken within ~4 ms of each other.

**Checking the temperatures:** both use R = Rs × V / (3.3 − V) and 1/T = 1/298.15 + ln(R/R25)/3950 (T in kelvin):
- PID Temp Sensor: V = `thermistor_v`, Rs = 100 kΩ, R25 = 100 kΩ
- AUX Temp Sensor: V = `aux_v`, Rs = 2.0 kΩ, R25 = 10 kΩ

That reproduces `temperature` and `aux_temp` to within a few thousandths of a °C. The raw counts let you redo either conversion with a different calibration or ADC scaling.

**Gains:** the kp/ki/kd columns hold the gains when the row was sampled. Changing them with `/pid/params` saves the buffered rows first, so rows never show gains they weren't sampled under. The serial monitor warns if that save fails.

At the maximum rate this is about 91 bytes × 167 rows/s ≈ 55 MB per hour, so a new 50 MB file starts about every 55 minutes.

## Technical Details

### Sampling
- One ADC read every 2 ms, cycling channels 0 → 1 → 2: up to ~167 readings/s per channel and ~167 CSV rows/s (one row per cycle)
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
| Kp / Ki / Kd | 2.2 / 0.25 / 0.15 |
| PID interval | 500 ms |
| Integral clamp | ±100 (±25% output) |
| Default target | 45 °C |
| Max safe temperature | 130 °C |
| Max continuous heater time | 30 min |

The PID uses the latest logged PID Temp Sensor reading, or reads that thermistor directly when the buffer is empty, so it works with logging on or off. Its output (0-100%) sets the PWM duty directly. The AUX Temp Sensor is never used by the PID.

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
- PID Temp Sensor: check the divider on C1 (~1.65 V at room temperature)
- AUX Temp Sensor: check the divider on C2 (~2.75 V at room temperature with the 2.0 kΩ resistor, ~1.7 V at 65 °C). With nothing connected to C2 it reads N/A, and logging continues.

### PID and AUX Temp Sensors disagree
- They measure different spots, so some difference is expected. A steady offset at room temperature points to a calibration or resistor-value issue: check `AUX_SERIES_RESISTOR` matches the resistor fitted, and compare both against a reference thermometer.

For PlatformIO install/build problems, see [TROUBLESHOOTING.md](md_info_docs/TROUBLESHOOTING.md).

## Notes

- The voltage channel is scaled as ADC/1024 (0-1 V). On NodeMCU / D1 mini boards, A0 has an onboard divider that makes its range 0-3.3 V, so the logged value is the actual voltage ÷ 3.3. The thermistor conversion already assumes 0-3.3 V. Because `voltage_adc` is logged, the actual voltage can be recovered afterwards (`voltage_adc` / 1024 × 3.3 on those boards).
- Timestamps restart at 0 on each boot. Each boot also starts a new log file.
- Further guides and change notes from earlier versions (relay control, internal-flash logging) are in `md_info_docs/`. Each change note starts with a note saying what has changed since. This README, WIRING_GUIDE.md and config.h describe the current firmware.

## License

This project is open source. Feel free to modify and distribute.
