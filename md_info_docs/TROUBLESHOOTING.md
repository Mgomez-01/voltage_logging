# PlatformIO Troubleshooting

This page covers installing and building. For runtime problems (SD card, heater, sensors), see the Troubleshooting sections in [README.md](README.md) and [WIRING_GUIDE.md](WIRING_GUIDE.md).

## "Detected unknown package 'espressif8266'"

Try these in order:

### 1. Update PlatformIO
```bash
pip install -U platformio
pio platform update
```
Or run `make fix-platform`.

### 2. Install the platform manually
```bash
pio platform install espressif8266
pio platform list
```

### 3. Check the board name
```bash
pio boards espressif8266
```
The environments in `platformio.ini` use:
- `nodemcuv2` (env `esp8266`) - NodeMCU v2
- `esp12e` (env `esp12e`) - bare ESP-12E module
- `d1_mini` (env `wemos_d1_mini`) - Wemos D1 mini

Build a specific one with `make build ENV=wemos_d1_mini` or `pio run -e wemos_d1_mini`.

## Library errors

- The code needs **ArduinoJson 7.x** (it uses the v7 `JsonDocument` API). An old 6.x copy in the global library cache causes errors about `JsonDocument`. Clean it with:
  ```bash
  pio pkg uninstall -g -l ArduinoJson
  make clean && make build
  ```
- `WebSocketsServer was not declared`: make sure `WebSockets@^2.4.0` is in `lib_deps` (it is by default).

## Diagnostics
```bash
pio --version
pio platform list
pio pkg list
make troubleshoot
```

## Alternatives

`platformio_alternatives.ini` has a few other environment setups (pinned platform version, generic module). Copy the one you need into `platformio.ini`.

### Docker
```bash
docker run -it --rm -v $(pwd):/workspace -w /workspace platformio/platformio-core pio run -e esp8266
```
