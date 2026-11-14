# Makefile for the ESP8266 Temperature Logger, PCB version (MCP3208 + SD card + PWM heater)
# Requires PlatformIO to be installed
#
# Board environment (from platformio.ini): esp8266 (NodeMCU v2 in the PCB socket),
# the only one on this branch.
ENV ?= esp8266

.PHONY: build upload monitor clean install help filesystem deploy

# Default target
all: build

# Build the project for the selected board
build:
	@echo "Building ESP8266 Voltage Logger for $(ENV)..."
	pio run -e $(ENV)

# Try specific board environments
build-nodemcu:
	@echo "Building for NodeMCU..."
	pio run -e esp8266

# Upload to ESP8266
upload: build
	@echo "Uploading firmware to ESP8266 ($(ENV))..."
	pio run -e $(ENV) --target upload

# Monitor serial output
monitor:
	@echo "Starting serial monitor (Ctrl+C to exit)..."
	pio device monitor --baud 115200

# Build, upload firmware, upload web page (LittleFS), and monitor in one command
deploy: upload filesystem
	@echo "Deployment complete (firmware + filesystem), starting monitor..."
	@sleep 2
	$(MAKE) monitor

# Clean build files
clean:
	@echo "Cleaning build files..."
	pio run -e $(ENV) --target clean

# Install/update dependencies
install:
	@echo "Installing PlatformIO dependencies..."
	pio pkg install

# Update libraries
update:
	@echo "Updating libraries..."
	pio pkg update

# List connected devices
devices:
	@echo "Connected devices:"
	pio device list

# Show project info
info:
	@echo "Project Information:"
	pio project config

# Upload the web page template (data/) to internal flash (LittleFS).
# Sensor logs go to the SD card, not here.
filesystem:
	@echo "Uploading web page to LittleFS ($(ENV))..."
	pio run -e $(ENV) --target uploadfs

# Generate compile_commands.json for language servers
compile-db:
	@echo "Generating compile commands database..."
	pio run -e $(ENV) --target compiledb || echo "Manual compile_commands.json generation needed"
	@echo "Language server database updated"

# Setup language server configuration
setup-lsp: compile-db
	@echo "Setting up language server configuration..."
	chmod +x generate_compile_commands.sh 2>/dev/null || true
	./generate_compile_commands.sh
	@echo "Language server setup complete - restart your editor"

# Troubleshooting commands
troubleshoot:
	@echo "=== PlatformIO Diagnostics ==="
	pio --version
	@echo ""
	@echo "=== Installed Platforms ==="
	pio platform list
	@echo ""
	@echo "=== Available ESP8266 Boards ==="
	pio boards espressif8266
	@echo ""
	@echo "=== Project Config ==="
	pio project config

# Fix common platform issues
fix-platform:
	@echo "Updating PlatformIO and installing ESP8266 platform..."
	pip install -U platformio
	pio platform update
	pio platform install espressif8266

# Help
help:
	@echo "ESP8266 Voltage Logger - Available targets:"
	@echo ""
	@echo "  Board: ENV=$(ENV) (NodeMCU v2 on the PCB)"
	@echo ""
	@echo "  build         - Compile the firmware for ENV"
	@echo "  build-nodemcu - Build for NodeMCU specifically"
	@echo "  upload        - Upload firmware to ESP8266"
	@echo "  monitor       - Start serial monitor"
	@echo "  deploy        - Build, upload firmware + web page, and monitor"
	@echo "  clean         - Clean build files"
	@echo "  install       - Install dependencies"
	@echo "  update        - Update libraries"
	@echo "  devices       - List connected devices"
	@echo "  info          - Show project information"
	@echo "  filesystem    - Upload web page template to LittleFS"
	@echo "  compile-db    - Generate compile_commands.json for language servers"
	@echo "  setup-lsp     - Setup language server configuration"
	@echo "  troubleshoot  - Run diagnostics"
	@echo "  fix-platform  - Fix platform issues"
	@echo "  help          - Show this help"
	@echo ""
	@echo "Quick start:"
	@echo "  make fix-platform  # Fix platform issues first"
	@echo "  make setup-lsp     # Setup language server (fixes undefined-symbol errors)"
	@echo "  make deploy        # Build, upload firmware + web page, and monitor"
	@echo ""
	@echo "If build fails:"
	@echo "  make troubleshoot  # Check what's wrong"
	@echo ""
	@echo "For IDE/Editor issues:"
	@echo "  make setup-lsp     # Fix language server undefined-symbol errors"
	@echo ""
	@echo "Prerequisites:"
	@echo "  pip install platformio"
