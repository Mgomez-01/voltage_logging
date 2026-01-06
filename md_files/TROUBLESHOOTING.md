# 🚨 PlatformIO Troubleshooting - ESP8266 Platform Issues

## **Issue: "Detected unknown package 'espressif8266'"**

This is a common PlatformIO setup issue. Here are the solutions:

## **🔧 Quick Fixes (Try in Order)**

### **Fix 1: Update PlatformIO**
```bash
# Update PlatformIO to latest version
pip install -U platformio

# Update platform registry
pio platform update
```

### **Fix 2: Replace platformio.ini**
Copy one of these configurations to your `platformio.ini`:

**Option A: Auto-detect platform**
```ini
[env:esp8266]
platform = espressif8266
board = esp12e
framework = arduino
monitor_speed = 115200
lib_deps = 
    bblanchon/ArduinoJson@^6.21.3
    links2004/WebSockets@^2.4.0
```

**Option B: Specific version**
```ini
[env:esp8266]
platform = espressif8266@4.2.0
board = esp12e
framework = arduino
monitor_speed = 115200
lib_deps = 
    bblanchon/ArduinoJson@^6.21.3
    links2004/WebSockets@^2.4.0
```

### **Fix 3: Try Different Board**
If you're not sure what ESP8266 board you have:

```bash
# List available ESP8266 boards
pio boards espressif8266
```

Common boards:
- `esp12e` - Generic ESP-12E module
- `nodemcuv2` - NodeMCU board
- `d1_mini` - Wemos D1 Mini
- `esp8285` - Generic ESP8285

### **Fix 4: Manual Platform Install**
```bash
# Install ESP8266 platform manually
pio platform install espressif8266

# Check if installed
pio platform list
```

## **🚀 Alternative: Use Arduino CLI Instead**

If PlatformIO keeps giving issues:

```bash
# Install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Add ESP8266 core
arduino-cli core update-index --additional-urls http://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core install esp8266:esp8266 --additional-urls http://arduino.esp8266.com/stable/package_esp8266com_index.json

# Install libraries
arduino-cli lib install "ArduinoJson@6.21.3"
arduino-cli lib install "WebSockets@2.4.0"

# Compile (from voltage_logger directory)
arduino-cli compile --fqbn esp8266:esp8266:generic src/main.cpp

# Upload (replace /dev/ttyUSB0 with your port)
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:generic src/
```

## **🎯 Updated Makefile Commands**

I'll update the Makefile to handle these issues:

```make
# Try different build environments
build-auto:
	pio run -e esp8266

build-stable:
	pio run -e esp8266_stable

build-nodemcu:
	pio run -e nodemcu

# Debug platform issues
debug-platform:
	pio platform list
	pio boards espressif8266
```

## **📋 Diagnostic Commands**

Run these to debug the issue:

```bash
# Check PlatformIO version
pio --version

# List installed platforms
pio platform list

# Check available ESP8266 boards
pio boards espressif8266

# Check if libraries can be found
pio lib search ArduinoJson
pio lib search WebSockets
```

## **✅ Working Configuration Template**

Here's a minimal `platformio.ini` that should work:

```ini
[env:esp8266]
platform = espressif8266
board = nodemcuv2
framework = arduino
monitor_speed = 115200
lib_deps = 
    ArduinoJson
    WebSockets
```

## **🚨 Last Resort: Docker**

If nothing works, use Docker:

```bash
# Run PlatformIO in Docker
docker run -it --rm -v $(pwd):/workspace -w /workspace platformio/platformio-core pio run
```

---

**Let me know which fix works, and I'll update the main configuration!**
