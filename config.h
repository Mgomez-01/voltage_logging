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
// PID Temp Sensor (100k NTC, Beta equation)
//   THERMISTOR_NOMINAL  = 100000.0  ohm at 25 C
//   TEMPERATURE_NOMINAL = 25.0      C
//   B_COEFFICIENT       = 3950.0
//   SERIES_RESISTOR     = 100000.0  ohm (3.3V -- R -- C1 -- NTC -- GND)
//
// AUX Temp Sensor (MF55 ultra-thin 10k NTC, Beta equation)
//   AUX_THERMISTOR_NOMINAL  = 10000.0  ohm at 25 C
//   AUX_TEMPERATURE_NOMINAL = 25.0     C
//   AUX_B_COEFFICIENT       = 3950.0   (B25/50; use 3435 for a 103F3435 part)
//   AUX_SERIES_RESISTOR     = 2000.0   ohm (3.3V -- R -- C2 -- NTC -- GND)
//                             Must match the resistor fitted. ~9 counts/C at
//                             60-70 C; 2.0k-2.7k give about the same.

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
//                   aux_temp,aux_adc,aux_v
//                   (CSV_HEADER in data_manager.cpp; ~91 bytes per row)
//     temperature / thermistor_adc / thermistor_v = PID Temp Sensor (channel 1)
//     aux_temp / aux_adc / aux_v                  = AUX Temp Sensor (channel 2)
//     *_adc        = raw ADC counts (0-1023)
//     *_v          = adc / 1024 * 3.3, the voltage each temperature is
//                    calculated from (thermistorAdcToVoltage())
//     pid_enabled  = 1 if the PID was controlling the heater
//     kp, ki, kd   = gains in effect; /pid/params flushes the buffer before
//                    changing them
//   MAX_WRITE_TIME = 5000 ms       A single SD write is abandoned after 5 s;
//                                  the watchdog is fed every 5 rows
//   If the SD card is not ready at boot, no log file is created. /start
//   (Start Logging) retries the SD setup and refuses with HTTP 503 if it
//   still fails. /data.csv with no ?file= downloads the current log file.
//
// Internal flash (LittleFS) holds ONLY the web page template
// (data/index.template.html, uploaded with `pio run -t uploadfs`).
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
//   pidKp                = 2.2
//   pidKi                = 0.25
//   pidKd                = 0.15
//   PID_INTERVAL         = 500 ms
//   Integral clamp       = +/-100 (+/-25% output with Ki = 0.25)
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
