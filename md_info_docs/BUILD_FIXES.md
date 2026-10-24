# ESP8266 Build Notes

## Current build setup
- **PlatformIO**, platform `espressif8266`, Arduino framework
- **Environments**: `esp8266` (NodeMCU v2, default), `wemos_d1_mini`, `esp12e`
- **Libraries**: ArduinoJson ^7.0.0, WebSockets ^2.4.0 (plus SD, SPI, LittleFS, Ticker from the core)
- **Filesystem**: LittleFS, used only for the web page template (`data/index.template.html`). Logs go to the SD card.
- **Build flags** (esp8266 and wemos_d1_mini): `PIO_FRAMEWORK_ARDUINO_LWIP2_LOW_MEMORY`, `VTABLES_IN_FLASH`, `MMU_IRAM_SIZE=0x8000`, `MMU_ICACHE_SIZE=0x8000`

Memory use with ESP8266 core 3.1.2 (NodeMCU): RAM ~42.8 KB of 80 KB, IRAM ~62 KB of 64 KB (94%), flash code ~366 KB.

## Build and upload
```bash
make build                     # or: pio run -e esp8266
make deploy                    # firmware + web page + serial monitor
make deploy ENV=wemos_d1_mini  # for a D1 mini
```

## Common errors

### "LittleFS.h: No such file"
The ESP8266 core is too old:
```bash
pio platform update espressif8266
```

### Errors about `JsonDocument`
The code uses the ArduinoJson **7** API. An old 6.x copy is being picked up:
```bash
pio pkg uninstall -g -l ArduinoJson
make clean && make build
```

### "WebSocketsServer was not declared"
Make sure `WebSockets@^2.4.0` (links2004) is in `lib_deps`.

### "ESP8266WiFi.h: No such file"
The ESP8266 platform isn't installed properly:
```bash
make fix-platform
```

### "Region 'iram1_0_seg' overflowed"
IRAM is already at ~94%. Keep the `MMU_IRAM_SIZE` / `MMU_ICACHE_SIZE` build flags, and avoid adding more `IRAM_ATTR` functions.

## Debugging the build
```bash
pio run -e esp8266 -v   # verbose output
pio pkg list            # installed libraries
make clean && rm -rf .pio/ && make build
make troubleshoot
```

## Expected result
```
Building ESP8266 Voltage Logger for esp8266...
...
SUCCESS
```

See also [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
