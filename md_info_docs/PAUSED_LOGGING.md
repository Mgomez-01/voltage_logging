# Paused Logging and Session Control

## Behavior
Logging starts **paused** at every boot. Nothing is written to the SD card until you press **Start Logging**, so each log file only holds the measurements you meant to record.

Each boot also creates a new log file (`data.log`, `data_1.log`, ...). Runs from different boots never mix in one file, and timestamps restart at 0 in each file.

While logging is paused:
- The web page still connects, and `/status` returns live readings
- The PID controller can still run (it reads the thermistor directly)
- The over-temperature check does **not** run, because it uses logged readings. Keep logging on whenever the heater is on.

## Workflow
1. Connect to WiFi `ESP8266_VoltageLogger` / `voltage123` and open `http://192.168.4.1`
2. Check the live readings
3. Press **Start Logging**
   - If the SD card wasn't ready at boot, this retries the SD setup. If it still can't create a log file, logging stays paused and the page shows the error.
4. Run your measurement
5. Press **Stop Logging**. The RAM buffer (up to 50 rows) is flushed to the card.
6. Press **Download Data** for the current file, or use the **Log Files on SD Card** panel for older files

## HTTP endpoints
| Route | Action |
|---|---|
| `/start` | Start logging (503 if no log file can be created) |
| `/stop` | Stop logging and flush the buffer |
| `/status` | Current readings and settings, including `loggingEnabled` |
| `/data.csv` | Download the current log file (flushes the buffer first) |
| `/data.csv?file=<name>` | Download a specific log file |
| `/logs` | List log files |
| `/clear` | Delete all `data*.log` files and start a new `data.log` |

## Serial output
At boot:
```
DATA LOGGING IS PAUSED BY DEFAULT
   Click 'Start Logging' in web interface to begin
   Voltage Channel 0, Temperature Channel 1
```
Start:
```
HTTP: Start logging requested
HTTP: Dual sensor logging STARTED - writing to data_3.log
```
Stop:
```
HTTP: Stop logging requested
Flushing remaining buffer to file...
[WATCHDOG] Fed before file write operation
[WATCHDOG] Fed after file write completion
Wrote 43 sensor readings to file in 210ms
  Voltage - Min=0.8180V, Max=0.8220V, Avg=0.8210V
  Temperature - Min=44.70°C, Max=45.10°C, Avg=44.92°C
HTTP: Dual sensor logging STOPPED - readings paused
```

Logging also stops on its own if free heap drops below 8 KB, or on an emergency shutdown.
