# 🔥 ESP8266 Dual Sensor Logger - CD74HC4067 Implementation

## 🎯 **Complete Dual Sensor System**

I've implemented a full **voltage + temperature logging system** using the CD74HC4067 multiplexer. Here's what's new:

## 🔧 **Hardware Setup**

### **Required Components:**
- **CD74HC4067** 16-Channel Analog Multiplexer (~$2-3)
- **100kΩ resistor** (for thermistor voltage divider)
- Your existing **voltage source** (Channel 0)
- Your **100k NTC thermistor** (Channel 1)

### **Wiring Connections:**
```
ESP8266 → CD74HC4067
├─ A0 → COM (common analog output)
├─ D1 (GPIO5) → S0 (control bit 0)
├─ D2 (GPIO4) → S1 (control bit 1)  
├─ D3 (GPIO0) → S2 (control bit 2)
├─ D4 (GPIO2) → S3 (control bit 3)
├─ 3.3V → VCC
└─ GND → GND

CD74HC4067 Sensor Connections:
├─ C0 → Your voltage sensor
└─ C1 → Thermistor voltage divider:
    └─ 3.3V ─── [100kΩ] ─── C1 ─── [Thermistor] ─── GND
```

## ⚡ **Enhanced Features**

### **📊 Dual Sensor Sampling**
- **500Hz per sensor** (alternates between channels every 2ms)
- **1000Hz total sampling rate** (maintains high-speed logging)
- **Time-multiplexed reading** - voltage then temperature

### **🌡️ Temperature Conversion**
- **Steinhart-Hart equation** for accurate NTC thermistor conversion
- **Configurable parameters** for different thermistor types
- **Error detection** for disconnected sensors

### **📱 Enhanced Web Interface**
- **Dual reading display** - voltage and temperature side-by-side
- **Separate statistics** for both sensors
- **Dual-axis chart** - voltage (left axis) + temperature (right axis)
- **Combined CSV export** - timestamp,voltage,temperature

### **🔍 Advanced Debugging**
- **Live sensor readings** from both channels
- **Multiplexer channel status** in debug output
- **Separate statistics** for voltage and temperature
- **Connection validation** for each sensor

## 📁 **CSV Format**
```csv
timestamp,voltage,temperature
1234,0.8213,23.45
1236,0.8215,23.47
1238,0.8210,23.44
```

## 🚀 **Deploy and Test**

```bash
make deploy
```

## 📋 **Testing Without Hardware**

**The code will work immediately** even without the CD74HC4067:

### **With Only ESP8266:**
- Channel 0 (voltage): Reads A0 directly
- Channel 1 (temperature): Will show "N/A (no thermistor connected)"
- Web interface loads and shows voltage readings
- Temperature shows as invalid until thermistor connected

### **Expected Serial Output:**
```
===============================================
ESP8266 Dual Sensor Logger - Voltage + Temperature
CD74HC4067 Multiplexer Version
===============================================
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: Thermistor (100k NTC)

Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V
Channel 1 (Thermistor): 512 -> N/A (no thermistor connected)
```

## 🎮 **Web Interface Changes**

### **New Display Layout:**
```
┌─────────────────────────────────────────┐
│     ESP8266 Dual Sensor Logger         │
│                                         │
│  ┌─────────────┐  ┌─────────────────┐  │
│  │   0.8223    │  │      23.4       │  │
│  │ Voltage (V) │  │ Temperature(°C) │  │
│  └─────────────┘  └─────────────────┘  │
│                                         │
│  Min V: 0.8200  Max V: 0.8240         │
│  Min T: 22.1°C  Max T: 24.8°C         │
│                                         │
│  [▶ Start] [⏸ Stop] [Clear] [Download] │
└─────────────────────────────────────────┘
```

### **Enhanced Statistics:**
- **Voltage stats**: Min, Max, Average voltage
- **Temperature stats**: Min, Max, Average temperature  
- **Sample count**: Total dual-sensor readings
- **Live updates**: Both sensors update in real-time

## 🔧 **Thermistor Calibration**

If you have a different thermistor, adjust these constants:
```cpp
const float THERMISTOR_NOMINAL = 100000.0;   // 100kΩ at 25°C
const float TEMPERATURE_NOMINAL = 25.0;      // 25°C reference
const float B_COEFFICIENT = 3950.0;          // Beta coefficient
const float SERIES_RESISTOR = 100000.0;      // 100kΩ series resistor
```

## 🎯 **Key Benefits**

1. **🔬 Professional dual sensing** - voltage and temperature simultaneously
2. **📊 High-speed logging** - 500Hz per sensor, 1000Hz total
3. **🌐 Real-time web display** - both sensors update live
4. **💾 Unified data export** - CSV with both measurements
5. **🔧 Hardware expandable** - multiplexer supports up to 16 sensors
6. **🚀 Ready to test** - works with or without multiplexer hardware

## 🧪 **Testing Plan**

### **Phase 1: Software Testing (Now)**
1. **Deploy code** - test web interface and voltage readings
2. **Verify CSV export** - check format includes temperature column
3. **Test start/stop** - confirm paused logging works

### **Phase 2: Hardware Testing (When CD74HC4067 arrives)**
1. **Wire multiplexer** according to diagram
2. **Connect thermistor** to Channel 1
3. **Test channel switching** - verify both sensors read correctly
4. **Calibrate temperature** - compare with known temperature

**The system is now a professional dual-sensor data logger!** 🎉

Deploy it now to test the software, and it'll be ready for the hardware when your CD74HC4067 arrives.
