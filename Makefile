# Makefile for ESP8266 Voltage Logger
# Requires PlatformIO to be installed

.PHONY: build upload monitor clean install help

# Default target
all: build

# Build the project (try different environments if one fails)
build:
	@echo "Building ESP8266 Voltage Logger..."
	pio run -e esp8266 || pio run -e esp12e || pio run -e wemos_d1_mini

# Try specific board environments
build-nodemcu:
	@echo "Building for NodeMCU..."
	pio run -e esp8266

build-esp12e:
	@echo "Building for ESP12E..."
	pio run -e esp12e

build-wemos:
	@echo "Building for Wemos D1 Mini..."
	pio run -e wemos_d1_mini

# Upload to ESP8266
upload: build
	@echo "Uploading to ESP8266..."
	pio run --target upload

# Monitor serial output
monitor:
	@echo "Starting serial monitor (Ctrl+C to exit)..."
	pio device monitor --baud 115200

# Build, upload filesystem, upload firmware, and monitor in one command
deploy: upload filesystem
	@echo "Deployment complete (firmware + filesystem), starting monitor..."
	@sleep 2
	$(MAKE) monitor

# Clean build files
clean:
	@echo "Cleaning build files..."
	pio run --target clean

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

# Flash filesystem (SPIFFS/LittleFS)
filesystem:
	@echo "Uploading filesystem..."
	pio run --target uploadfs

# Generate compile_commands.json for language servers
compile-db:
	@echo "Generating compile commands database..."
	pio run --target compiledb || echo "Manual compile_commands.json generation needed"
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
	@echo "  build         - Compile the project (auto-detect board)"
	@echo "  build-nodemcu - Build for NodeMCU specifically"
	@echo "  build-esp12e  - Build for ESP12E specifically"
	@echo "  build-wemos   - Build for Wemos D1 Mini specifically"
	@echo "  upload        - Upload to ESP8266"  
	@echo "  monitor       - Start serial monitor"
	@echo "  deploy        - Build, upload firmware + filesystem, and monitor"
	@echo "  clean         - Clean build files"
	@echo "  install       - Install dependencies"
	@echo "  update        - Update libraries"
	@echo "  devices       - List connected devices"
	@echo "  info          - Show project information"
	@echo "  filesystem    - Upload filesystem"
	@echo "  compile-db    - Generate compile_commands.json for language servers"
	@echo "  setup-lsp     - Setup language server configuration"
	@echo "  troubleshoot  - Run diagnostics"
	@echo "  fix-platform  - Fix platform issues"
	@echo "  help          - Show this help"
	@echo ""
	@echo "Quick start:"
	@echo "  make fix-platform  # Fix platform issues first"
	@echo "  make setup-lsp     # Setup language server (fixes A0 errors)"
	@echo "  make deploy        # Build, upload firmware + filesystem, and monitor"
	@echo ""
	@echo "If build fails:"
	@echo "  make troubleshoot  # Check what's wrong"
	@echo "  make build-nodemcu # Try specific board"
	@echo ""
	@echo "For IDE/Editor issues:"
	@echo "  make setup-lsp     # Fix language server A0 undefined errors"
	@echo ""
	@echo "Prerequisites:"
	@echo "  pip install platformio"
