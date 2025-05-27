#!/bin/bash
# Generate compile_commands.json for language servers

echo "Generating compile commands database for language servers..."

# Method 1: Use PlatformIO built-in support
if command -v pio &> /dev/null; then
    echo "Using PlatformIO to generate compile_commands.json..."
    pio run --target compiledb
    if [ -f "compile_commands.json" ]; then
        echo "✅ Successfully generated compile_commands.json"
    else
        echo "⚠️  PlatformIO compile_commands.json generation failed, trying alternative..."
    fi
fi

# Method 2: Manual generation if PlatformIO method fails
if [ ! -f "compile_commands.json" ]; then
    echo "Creating manual compile_commands.json..."
    cat > compile_commands.json << 'EOF'
[
  {
    "directory": "'$(pwd)'",
    "command": "xtensa-lx106-elf-g++ -DARDUINO=10816 -DARDUINO_ESP8266_NODEMCU -DARDUINO_ARCH_ESP8266 -DESP8266 -DPLATFORMIO=60106 -DF_CPU=80000000L -DA0=17 -std=c++11 -I. -Isrc -I.pio/libdeps/esp8266 -I~/.platformio/packages/framework-arduinoespressif8266/cores/esp8266 -I~/.platformio/packages/framework-arduinoespressif8266/variants/nodemcu -c src/main.cpp",
    "file": "src/main.cpp"
  }
]
EOF
    echo "✅ Created manual compile_commands.json"
fi

echo ""
echo "Language server setup complete!"
echo "Restart your editor to pick up the new configuration."
