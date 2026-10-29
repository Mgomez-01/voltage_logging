// ESP8266 Voltage + Temperature Logger - Configuration Reference
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
//   SAMPLE_INTERVAL     = 2 ms    One ADC read every 2 ms, cycling channels
//                                 0 -> 1 -> 2 (up to ~167 readings/s per
//                                 channel, one CSV row per cycle)
//   WEB_UPDATE_INTERVAL = 500 ms  WebSocket push of latest reading
//   DEBUG_INTERVAL      = 1000 ms Serial stats print
//   Low-heap auto-stop  = free heap < 8000 bytes stops logging

// =============================================================================
// SENSORS (src/sensor_manager.cpp)
// =============================================================================
// CD74HC4067 mux select pins
//   MUX_S0 = GPIO5  (D1)
//   MUX_S1 = GPIO4  (D2)
//   MUX_S2 = GPIO0  (D3)
//   MUX_S3 = GPIO2  (D4)
//   ADC_PIN = A0 (mux COM)
//
// Channels
//   VOLTAGE_CHANNEL        = 0   voltage = ADC / 1024 (0-1 V scale)
//   THERMISTOR_CHANNEL     = 1   PID Temp Sensor (PID + safety checks)
//   AUX_THERMISTOR_CHANNEL = 2   AUX Temp Sensor (logged/displayed only)
//
// Measurement circuit (both thermistor channels)
//   THERMISTOR_SUPPLY_VOLTAGE = 3.32     V, measured rail feeding both dividers
//   ADC_FULL_SCALE_VOLTAGE    = 3.2      V at A0 that reads 1024 (board's
//                                        220k/100k divider to the 1.0 V ADC)
//   ADC_INPUT_RESISTANCE      = 320000.0 ohm, that divider loading the selected
//                                        channel (use ~960000 if a filter
//                                        capacitor is added on C1/C2)
//   ADC_CALIBRATION_GAIN      = 1.0      multiplier on the A0 voltage; leave at
//                                        1.0 (calibration is per sensor, R25/B)
//   Conversion: V = adc/1024 * 3.2 * gain
//               P = Rs * V / (VS - V);  R = P * RL / (RL - P)
//               1/T = 1/(T25 + 273.15) + ln(R/R25)/B
//               T = T - correction(T)   (hotplate table, see below)
//   -999 when adc <= 1 (short), adc >= 1023 (saturated), or R > 20 x R25
//   (open: a disconnected PID NTC reads ~2.54 V because of the 320k load)
//
// PID Temp Sensor (100k NTC, Beta equation)
//   THERMISTOR_NOMINAL  = 97886.0   ohm at 25 C (R25, fitted; datasheet 100000)
//   TEMPERATURE_NOMINAL = 25.0      C
//   B_COEFFICIENT       = 3910.0    (fitted; datasheet 3950)
//   SERIES_RESISTOR     = 98000.0   ohm, measured (VS -- R -- C1 -- NTC -- GND)
//
// AUX Temp Sensor (MF55 ultra-thin 10k NTC, Beta equation)
//   AUX_THERMISTOR_NOMINAL  = 10631.0  ohm at 25 C (R25, fitted; datasheet 10000)
//   AUX_TEMPERATURE_NOMINAL = 25.0     C
//   AUX_B_COEFFICIENT       = 4096.0   (fitted; datasheet B25/50 3950)
//   AUX_SERIES_RESISTOR     = 2000.0   ohm, measured (VS -- R -- C2 -- NTC -- GND)
//                             Must match the resistor fitted. ~9 counts/C at
//                             60-70 C; 2.0k-2.7k give about the same.
//
// Calibration (2026-10-25): R25/B fitted from ice bath 0 C (PID 660.8, AUX
// 1002.5 counts) and boiling water 95.7 C at ~1300 m, SLC (PID 77.7, AUX 293.1).
// Refit with tools/calibrate_thermistors.py.
//
// Hotplate correction (2026-10-28), applied after the Beta equation:
//   T = T - correction(T), linear interpolation, 0 outside 0-95.7 C
//   reading:  0   30   40   50   55   60   65   70   75   80   85  95.7
//   PID:      0 2.29 2.01 1.61 1.33 1.06 0.95 1.01 1.13 1.23 1.25  0
//   AUX:      0 3.99 3.87 3.61 3.39 3.13 2.95 2.86 2.80 2.74 2.67  0
//   (PID_CORRECTION / AUX_CORRECTION; APPLY_TEMPERATURE_CORRECTION = true)
//   Sensors agree to a few tenths of a C; absolute about +/-1.5 C.

// =============================================================================
// DATA LOGGING - SD CARD (src/data_manager.cpp)
// =============================================================================
// SD card on hardware SPI: SCK=D5 (GPIO14), MISO=D6 (GPIO12),
//                          MOSI=D7 (GPIO13), CS=D8 (GPIO15, SS)
//   SD.begin(SS)
//
//   BUFFER_SIZE   = 50 rows        RAM buffer before each SD write
//   MAX_FILE_SIZE = 50000000 bytes Rotate to a new file at 50 MB
//   File names    = data.log, data_1.log, data_2.log, ... (new file each boot)
//   CSV header    = timestamp,voltage,temperature,heater_state,target_temp,pid_output,
//                   voltage_adc,thermistor_adc,thermistor_v,pid_enabled,kp,ki,kd,
//                   aux_temp,aux_adc,aux_v,samples
//                   (CSV_HEADER in data_manager.cpp; ~93 bytes per raw row)
//     temperature / thermistor_adc / thermistor_v = PID Temp Sensor (channel 1)
//     aux_temp / aux_adc / aux_v                  = AUX Temp Sensor (channel 2)
//     *_adc        = raw ADC counts (0-1023), or mean counts (2 decimals)
//                    when averaged
//     samples      = sample cycles in the row: 1 = raw, N = averaged
//     *_v          = adc / 1024 * 3.2 * ADC_CALIBRATION_GAIN, the A0 voltage
//                    each temperature is calculated from (thermistorAdcToVoltage())
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
//   pidKp                = 2.3    interim defaults from a model of the sheet;
//   pidKi                = 0.045  gains saved from the /tune page (/pid.json
//   pidKd                = 0.0    on LittleFS) replace them at boot
//   PID_INTERVAL         = 500 ms
//   Anti-windup          = integral term limited to +/-100 % output and not
//                          integrated while the output is saturated
//   Derivative           = on the measurement (-Kd dT/dt), no setpoint kick
//
//   PID tuning (src/tuning.cpp, /tune page)
//   TUNE_MAX_SAMPLES     = 600 x 8 bytes, 1 s (halves resolution when full)
//   TUNE_MAX_TEMP        = 120 C   test stops (heater off) above this
//   TUNE_MAX_TOTAL_MS    = 25 min  test stops after this
//   Step test settling   = >= 2 min and |trend| < 0.15 C/min over 60 s
//   Output               = 0-100% -> PWM duty
//
//   MAX_SAFE_TEMPERATURE = 130.0 C        Over-temp emergency shutdown
//   MAX_HEATER_TIME      = 1800000 ms     30 min continuous heating limit
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

// =============================================================================
// SERIAL / DEBUG
// =============================================================================
//   Baud rate = 115200
//   DEBUG_* flags are set per file (main.cpp, heater_controller.cpp,
//   web_interface.cpp). src/debug_config.h holds centralized macros but is not
//   included yet (see md_info_docs/DEBUG_MIGRATION_GUIDE.md).
