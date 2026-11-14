// ESP8266 Temperature Logger (PCB version) - Configuration Reference
//
// NOTE: This file is a REFERENCE ONLY. It is not included by any source file,
// and nothing here is compiled into the firmware. The live values are defined
// in the source files listed next to each setting. Change them there. If
// you do, update this file so it stays accurate.

// =============================================================================
// WIFI (src/wifi_manager.cpp)
// =============================================================================
// Access point the ESP8266 creates
//   ssid     = "ESP8266_VoltageLogger"
//   password = "voltage123"
//   IP       = 192.168.4.1 (ESP8266 default AP address)
//
// Ports (src/main.cpp)
//   HTTP      = 80
//   WebSocket = 81

// =============================================================================
// SAMPLING (src/main.cpp)
// =============================================================================
//   SAMPLE_INTERVAL     = 6 ms    One sample cycle (CH1 + CH2, one MCP3208
//                                 conversion each) every 6 ms: up to ~167
//                                 cycles/s, one CSV row per cycle when raw.
//                                 Runs all the time (logging or not): feeds
//                                 the live readings (safety checks) and the
//                                 PID's input average (heater_controller.cpp,
//                                 addControlSample(), mean of the ~83 cycles
//                                 between 500 ms PID updates)
//   WEB_UPDATE_INTERVAL = 500 ms  WebSocket push of latest reading
//   DEBUG_INTERVAL      = 1000 ms Serial stats print
//   Low-heap auto-stop  = free heap < 8000 bytes stops logging

// =============================================================================
// SENSORS (src/sensor_manager.cpp)
// =============================================================================
// MCP3208 12-bit SPI ADC (U2), shares SPI with the SD card
//   ADC_CS_PIN = GPIO5 (D1), 10k pull-up R12; SPI at 1 MHz, MODE0
//   SCK = D5, MISO = D6, MOSI = D7 (33 ohm series R9/R10 on SCK/MOSI)
//   readAdcChannel(ch) -> 0-4095 (~30 us); readAdcChannelAveraged(ch, n)
//   D2, D3, D4 and A0 are not connected on the PCB
//
// Channels
//   CH0                    = not connected (floating, never logged)
//   THERMISTOR_CHANNEL     = 1   PID Temp Sensor, J4 (sheet: default control
//                                sensor, safety checks, sheet limit)
//   AUX_THERMISTOR_CHANNEL = 2   AUX Temp Sensor, J5 (sample: selectable as
//                                the control sensor)
//   CH3-CH7                = J7 pins 5..1 (reversed), spare, not read
//
// Measurement circuit (both thermistor channels), ratiometric
//   +3V3 -- Rs -- CHn -- NTC -- GND, 100 nF (C8/C9) on each node,
//   MCP3208 VREF = VDD = +3V3, so the rail voltage cancels out
//   ADC_COUNTS                = 4096
//   THERMISTOR_SUPPLY_VOLTAGE = 3.32   V, only for the logged *_v columns
//   Conversion: R = Rs * code / (4096 - code)
//               1/T = 1/(T25 + 273.15) + ln(R/R25)/B
//   -999 when code <= 1 (short), code >= 4094 (open: pulled to +3V3),
//   or R > 20 x R25
//
// PID Temp Sensor (100k NTC, Beta equation)
//   THERMISTOR_NOMINAL  = 97886.0   ohm at 25 C (old-board fit; datasheet 100000)
//   TEMPERATURE_NOMINAL = 25.0      C
//   B_COEFFICIENT       = 3910.0    (old-board fit; datasheet 3950)
//   SERIES_RESISTOR     = 100000.0  ohm, R1 nominal: set to the measured value
//
// AUX Temp Sensor (MF55 ultra-thin 10k NTC, Beta equation)
//   AUX_THERMISTOR_NOMINAL  = 10631.0  ohm at 25 C (old-board fit; datasheet 10000)
//   AUX_TEMPERATURE_NOMINAL = 25.0     C
//   AUX_B_COEFFICIENT       = 4096.0   (old-board fit; datasheet B25/50 3950)
//   AUX_SERIES_RESISTOR     = 2000.0   ohm, R2 nominal: set to the measured value
//                             ~37 codes/C at 65 C (PID sensor: ~20 codes/C)
//
// Calibration: NOT YET DONE ON THE PCB. R25/B above are the old board's fits
// (2026-10-25, ice bath + boiling water at ~1300 m, SLC), which also absorbed
// the old A0 ADC's errors. Recalibrate on the PCB (ice bath ~3100 PID / ~3900
// AUX codes; boiling ~300 / ~1130) with tools/calibrate_thermistors.py.
//
// Hotplate correction (2026-10-28, OLD BOARD), applied after the Beta equation:
//   T = T - correction(T), linear interpolation, 0 outside 0-95.7 C
//   reading:  0   30   40   50   55   60   65   70   75   80   85  95.7
//   PID:      0 2.29 2.01 1.61 1.33 1.06 0.95 1.01 1.13 1.23 1.25  0
//   AUX:      0 3.99 3.87 3.61 3.39 3.13 2.95 2.86 2.80 2.74 2.67  0
//   (PID_CORRECTION / AUX_CORRECTION; APPLY_TEMPERATURE_CORRECTION = false
//   on the PCB: part of it corrected the old ESP8266 ADC's non-linearity.
//   Re-measure on the PCB before turning it back on.)

// =============================================================================
// DATA LOGGING - SD CARD (src/data_manager.cpp)
// =============================================================================
// On-board microSD (J6, Hirose DM3AT, no card-detect) on hardware SPI,
// shared with the MCP3208: SCK=D5 (GPIO14), MISO=D6 (GPIO12),
//                          MOSI=D7 (GPIO13), CS=D8 (GPIO15, SS, via R11 47 ohm)
//   SD.begin(SS)
//
//   BUFFER_SIZE   = 50 rows        RAM buffer before each SD write
//   MAX_FILE_SIZE = 50000000 bytes Rotate to a new file at 50 MB
//   File names    = data.log, data_1.log, data_2.log, ... (new file each boot)
//   CSV header    = timestamp,voltage,temperature,heater_state,target_temp,pid_output,
//                   voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,
//                   aux_temp,aux_adc,aux_v,samples,control,step,adc_fs
//                   (CSV_HEADER in data_manager.cpp; ~110 bytes per raw row)
//     voltage / voltage_adc = always 0 on the PCB (kept for old scripts)
//     temperature / thermistor_adc / thermistor_v = PID Temp Sensor (CH1)
//     aux_temp / aux_adc / aux_v                  = AUX Temp Sensor (CH2)
//     *_adc        = raw MCP3208 codes (0-4095), or mean codes (2 decimals)
//                    when averaged
//     adc_fs       = 4096 (old-board logs have no adc_fs column: 0-1023)
//     samples      = sample cycles in the row: 1 = raw, N = averaged
//     control      = sensor the PID controls: 0 = PID Temp Sensor, 1 = AUX
//     step         = setpoint program step (1-100), 0 = no program
//     *_v          = code / 4096 * 3.32, the node voltage (reference only;
//                    the temperature is calculated ratiometrically from the code)
//     pid_enabled  = 1 if the PID was controlling the heater
//     kp, ki, kd   = gains in effect; /pid/params flushes the buffer before
//                    changing them
//   Averaging (oversampling), set from the web page or /averaging?enabled=&n=
//     averagingEnabled        = false  off: log every sample cycle raw
//     DEFAULT_AVERAGE_SAMPLES = 32     cycles per averaged row (~0.2 s)
//     MIN/MAX_AVERAGE_SAMPLES = 2/100
//     Counts are averaged, then converted. Resets to off at boot.
//   MAX_WRITE_TIME = 5000 ms       A single SD write is abandoned after 5 s;
//                                  the watchdog is fed every 5 rows
//   Every log file starts with the header: a write to an empty or deleted
//   current file writes the header first.
//   If the SD card is not ready at boot, no log file is created. /start
//   (Start Logging) retries the SD setup and refuses with HTTP 503 if it
//   still fails. /data.csv with no ?file= downloads the current log file.
//
// Internal flash (LittleFS) holds the web pages (data/index.template.html,
// data/tune.html, uploaded with `pio run -t uploadfs`) and the PID gains saved
// from /tune (/pid.json, erased by uploadfs).
// No sensor data is stored in internal flash.

// =============================================================================
// HEATER PWM + PID (src/heater_controller.cpp)
// =============================================================================
//   HEATER_PWM_PIN       = GPIO16 (D0) -> AOD4144 MOSFET gate
//   PWM_FREQUENCY        = 32 Hz via analogWriteFreq(). The core docs list
//                          100 Hz as the minimum, but core 3.1.2's default
//                          PWM engine uses 32 Hz (only its fallback path
//                          clamps to 100 Hz).
//   PWM_RANGE            = 1000 (0.1% resolution)
//
//   targetTemperature    = 45.0 C default (settable 0 - MAX_SAFE_TEMPERATURE)
//   controlSensor        = CONTROL_PID_SENSOR (or CONTROL_AUX_SENSOR),
//                          switchable at runtime: /pid/sensor?s=pid|aux
//   sensorGains          = PID sensor 2.3 / 0.045 / 0, AUX 1.0 / 0.008 / 0
//                          (Kp / Ki / Kd; interim, from a model). The active
//                          sensor's gains are pidKp / pidKi / pidKd.
//   sheetLimit           = 120 C (40-125) while controlling on AUX; power
//                          tapers to 0 over SHEET_LIMIT_BAND = 5 C below it
//   Saved settings       = /pid.json on LittleFS (both gain sets, control
//                          sensor, sheet limit), loaded at boot
//   PID_INTERVAL         = 500 ms
//   Anti-windup          = integral term limited to +/-100 % output and not
//                          integrated while the output is saturated
//   Derivative           = on the measurement (-Kd dT/dt), no setpoint kick
//
//   PID tuning (src/tuning.cpp, /tune page)
//   TUNE_MAX_SAMPLES     = 600 x 8 bytes, 1 s (halves resolution when full)
//   TUNE_MAX_TEMP        = 120 C   test stops (heater off) above this
//   TUNE_MAX_TOTAL_MS    = 55 min  test stops after this
//
//   Setpoint program (src/program.cpp, /program page)
//   PROGRAM_MAX_STEPS    = 100     steps (setpoint 20-120 C, hold 0.5-360 min)
//   Total time           <= MAX_HEATER_TIME (6 h)
//   Start turns on logging + PID; steps advance on time only; end action
//   heater off or hold last; optional stop logging at the end
//   Step test settling   = >= 2 min and |trend| < 0.15 C/min over 60 s
//   Output               = 0-100% -> PWM duty
//
//   MAX_SAFE_TEMPERATURE = 130.0 C        Over-temp emergency shutdown
//   MAX_HEATER_TIME      = 21600000 ms    6 h continuous heating limit (was 30 min, 1 h, 3 h)
//
//   The over-temp and sensor-fault checks use the latest logged reading, so
//   they only run while logging is on. The PID itself reads the sensor
//   directly when the buffer is empty.

// =============================================================================
// SAFETY SYSTEM (src/safety_system.cpp)
// =============================================================================
//   SAFETY_CHECK_INTERVAL = 500 ms   Ticker heater-runtime check
//   Watchdog ticker       = 8000 ms  Software watchdog check
//   WATCHDOG_TIMEOUT      = 10000 ms ESP hardware watchdog
//   System reset (/system/reset) allowed only below MAX_SAFE_TEMPERATURE - 10
//   Emergency shutdown: PWM 0, PID off, logging off, open log file closed
//   Over-temperature / sensor-fault checks use the live PID Temp Sensor sample,
//   so they run whether or not logging is on
//   shutdownReason (safety_system.h): timeout, over-temp, sensor, watchdog,
//   safety timer; reported in /status and shown by the status LED
//
// =============================================================================
// STATUS LED (src/status_led.cpp)
// =============================================================================
//   WS2812B (D2) on RX/GPIO3 via 74AHCT1G125 (U3); NeoPixelBus DMA method
//   LED_BRIGHTNESS = 64 (of 255)
//   Off = heater off; blue -> red = heater on, 0 -> 100 % PWM;
//   flashing orange = heater timeout shutdown; flashing yellow = other shutdown

// =============================================================================
// SERIAL / DEBUG
// =============================================================================
//   Baud rate = 115200, TX only (SERIAL_TX_ONLY): RX/GPIO3 is the LED data line
//   DEBUG_* flags are set per file (main.cpp, heater_controller.cpp,
//   web_interface.cpp). src/debug_config.h holds centralized macros but is not
//   included yet (see md_info_docs/DEBUG_MIGRATION_GUIDE.md).
