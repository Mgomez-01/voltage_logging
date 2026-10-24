# No Arduino IDE Required

The project builds with PlatformIO from the command line or VS Code. The Arduino IDE isn't needed.

## Project structure
```
voltage_logging/
├── src/
│   ├── main.cpp                # setup(), loop(), sampling, debug stats
│   ├── sensor_manager.*        # multiplexer + thermistor
│   ├── data_manager.*          # RAM buffer + SD card log files
│   ├── heater_controller.*     # PWM heater + PID
│   ├── safety_system.*         # watchdog + emergency shutdown
│   ├── web_interface.*         # HTTP routes + WebSocket
│   └── wifi_manager.*          # access point
├── data/index.template.html    # web page (LittleFS)
├── platformio.ini              # PlatformIO configuration
├── Makefile                    # make commands
├── README.md                   # full documentation
├── QUICKSTART.md               # short setup guide
└── config.h                    # settings reference (not compiled)
```

## Quick setup

### PlatformIO + VS Code
1. Install VS Code and the "PlatformIO IDE" extension
2. Open the project folder
3. Pick the environment (esp8266 for NodeMCU, wemos_d1_mini for a D1 mini)
4. Run **Upload**, then **Upload Filesystem Image** (the web page)
5. Open the serial monitor (115200 baud)

### PlatformIO command line
```bash
pip install platformio
pio run -e esp8266 -t upload
pio run -e esp8266 -t uploadfs
pio device monitor -b 115200
```

### Make
```bash
pip install platformio
make deploy                     # build, upload firmware + web page, monitor
make deploy ENV=wemos_d1_mini   # for a D1 mini
make build                      # just compile
make monitor                    # serial monitor
```

## Arduino IDE vs PlatformIO
| Arduino IDE | PlatformIO |
|---|---|
| .ino sketch | `src/*.cpp` modules (same Arduino code) |
| Library Manager | `lib_deps` in `platformio.ini`, installed automatically |
| Separate filesystem upload plugin | `pio run -t uploadfs` |
| One board at a time | Several environments in one file |

## Still want the Arduino IDE?
Copy all of `src/` into a sketch folder with a matching (empty) `.ino` file, install ArduinoJson 7.x and WebSockets, and upload the `data/` folder with the LittleFS upload tool. See [libraries.txt](../libraries.txt).
