# Development Options (No Arduino IDE Required)

## Option 1: PlatformIO (recommended)

This is what the project is set up for (`platformio.ini`, `Makefile`).

### Setup
```bash
pip install platformio
# or install VS Code + the "PlatformIO IDE" extension
```

### Usage
```bash
make deploy                     # build, upload firmware, upload web page, monitor
make deploy ENV=wemos_d1_mini   # same for a Wemos D1 mini

# or directly with pio:
pio run -e esp8266                    # build
pio run -e esp8266 -t upload          # upload firmware
pio run -e esp8266 -t uploadfs        # upload web page (data/) to LittleFS
pio device monitor -b 115200          # serial monitor
pio run -e esp8266 -t clean           # clean
```
Always pass `-e <env>`. Without it, PlatformIO builds and uploads every environment in `platformio.ini`.

### VS Code
Open the project folder with the PlatformIO extension installed and use the PlatformIO toolbar. Pick the environment (esp8266 / wemos_d1_mini / esp12e) in the status bar, and run both "Upload" and "Upload Filesystem Image".

---

## Option 2: Arduino CLI

Possible, but more manual than PlatformIO.

```bash
arduino-cli core update-index --additional-urls http://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core install esp8266:esp8266 --additional-urls http://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli lib install "ArduinoJson@7.4.2" "WebSockets"
```
Arduino CLI needs a sketch folder with a `.ino` file named after the folder. Copy everything in `src/` into, for example, `voltage_logging/` next to an empty `voltage_logging.ino`, then:
```bash
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 voltage_logging
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 voltage_logging
```
The web page (`data/index.template.html`) still has to be uploaded to LittleFS separately (for example with PlatformIO's `uploadfs` or the mklittlefs tool).

---

## Option 3: ESP8266 RTOS SDK (advanced)
Espressif's own framework. It would mean rewriting the code without the Arduino libraries, so it isn't practical for this project.

---

## Project structure
```
voltage_logging/
├── platformio.ini          # build configuration (esp8266, esp12e, wemos_d1_mini)
├── Makefile                # make build / deploy / monitor (ENV=...)
├── src/                    # firmware modules (main.cpp + *_manager, heater, safety, web)
├── data/
│   └── index.template.html # web page, uploaded to LittleFS
├── config.h                # reference of settings (not compiled)
└── *.md                    # documentation
```

| Method | Pros | Cons |
|---|---|---|
| PlatformIO | Configured already, handles libraries and the LittleFS upload | Learning curve |
| Arduino CLI | Familiar Arduino tooling | Manual sketch folder and filesystem upload |
| RTOS SDK | Full control | Needs a rewrite |
