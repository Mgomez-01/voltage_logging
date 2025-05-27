# 🔌 CD74HC4067 Multiplexer Wiring Guide

## 📋 **Pin Mapping**

### **ESP8266 NodeMCU → CD74HC4067**
```
ESP8266 Pin | GPIO | CD74HC4067 Pin | Function
------------|------|----------------|----------
A0          | ADC  | COM            | Analog output to ESP8266
D1          | 5    | S0             | Channel select bit 0
D2          | 4    | S1             | Channel select bit 1
D3          | 0    | S2             | Channel select bit 2
D4          | 2    | S3             | Channel select bit 3
3.3V        | 3.3V | VCC            | Power supply
GND         | GND  | GND            | Ground
            |      | EN             | Tie to 3.3V (enable)
```

### **CD74HC4067 Sensor Connections**
```
Channel | Pin | Connection
--------|-----|--------------------
C0      | C0  | Your voltage source
C1      | C1  | Thermistor voltage divider
C2-C15  | C2+ | Available for future sensors
```

## 🔥 **Complete Wiring Diagram**

```
                    ESP8266 NodeMCU
                    ┌─────────────┐
                    │     A0 ────┼──┐
                    │     D1 ────┼─┐│
                    │     D2 ────┼┐││
                    │     D3 ────┼│││
                    │     D4 ────┼││││
                    │    3.3V────┼│││││
                    │     GND────┼│││││
                    └─────────────┘│││││
                                  │││││
                    CD74HC4067    │││││
                    ┌─────────────┐│││││
              ┌─────┤ COM     VCC ├┼┘│││
              │     │ S0      EN  ├─┘│││  (EN to 3.3V)
              │  ┌──┤ S1      GND ├──┘││
              │  │┌─┤ S2          │   ││
              │  ││┌┤ S3          │   ││
              │  │││├─────────────┤   ││
              │  ││││ C0      C1  ├┐  ││
              │  │││└─────────────┘│  ││
              │  ││└────────────────┘  ││
              │  │└─────────────────────┘│
              │  └────────────────────────┘
              │
              │  Voltage Source        Thermistor Circuit
              │  ┌─────────────┐      ┌─────────────────┐
              └──┤ Your sensor │      │ 3.3V            │
                 │ (0-1V out)  │      │  │              │
                 └─────────────┘      │  ├── 100kΩ ──┐  │
                                      │  │           │  │
                                      │  └───────────┼──┤ C1
                                      │              │  │
                                      │         Thermistor
                                      │         (100k NTC)
                                      │              │  │
                                      │             GND ├──┘
                                      └─────────────────┘
```

## 🧰 **Component Details**

### **CD74HC4067 Multiplexer**
- **16 channels** (C0-C15) selectable via 4 control bits
- **Single supply**: 2V to 6V (perfect for 3.3V ESP8266)
- **Low on-resistance**: ~70Ω typical
- **Fast switching**: ~250ns propagation delay
- **Common part numbers**: CD74HC4067, 74HC4067, SN74HC4067

### **100k NTC Thermistor**
- **Typical specs**: 100kΩ at 25°C, β=3950K
- **Temperature range**: -40°C to +125°C
- **Ender 3 compatible**: Uses same thermistor type
- **3-wire connection**: Signal, 3.3V, GND (via voltage divider)

## 🔧 **Assembly Steps**

### **1. Breadboard Layout**
```
Power Rails:
- Red rail: 3.3V from ESP8266
- Blue rail: GND from ESP8266

CD74HC4067:
- Place multiplexer chip on breadboard
- Connect VCC and EN to 3.3V rail
- Connect GND to ground rail
- Wire control pins S0-S3 to ESP8266 D1-D4
- Connect COM to ESP8266 A0
```

### **2. Sensor Connections**
```
Channel 0 (Voltage):
- Connect your existing voltage source to C0
- Ensure voltage stays within 0-1V range

Channel 1 (Thermistor):
- 3.3V → 100kΩ resistor → C1 → Thermistor → GND
- This creates a voltage divider for temperature sensing
```

### **3. Verification**
```
With multimeter, check:
✅ 3.3V between VCC and GND
✅ 3.3V between EN and GND  
✅ Continuity from ESP8266 pins to multiplexer
✅ ~1.65V at C1 (thermistor midpoint at room temp)
✅ Your voltage source reading at C0
```

## 🧪 **Testing Procedure**

### **1. Power-On Test**
```
Expected Serial Output:
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: Thermistor (100k NTC)

Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V    ← Your voltage
Channel 1 (Thermistor): 512 -> 23.4°C  ← Room temperature
```

### **2. Channel Switching Test**
Touch the thermistor with your finger - temperature should rise within seconds:
```
LIVE sensor readings:
  Voltage (Ch0): 842 -> 0.822300V
  Temperature (Ch1): 567 -> 25.8°C  ← Higher when touched
```

### **3. Web Interface Test**
- Both voltage and temperature should update in real-time
- Temperature should be reasonable (15-30°C room temperature)
- Voltage should match your previous readings

## 🚨 **Troubleshooting**

### **Problem: "N/A (no thermistor connected)"**
**Solutions:**
- Check thermistor wiring - ensure voltage divider connected
- Verify 100kΩ series resistor is present
- Test thermistor resistance with multimeter (~100kΩ at room temp)
- Check C1 voltage should be ~1.6V at room temperature

### **Problem: Temperature readings way off**
**Solutions:**
- Verify thermistor specifications match code (100kΩ, β=3950)
- Check series resistor value (should be 100kΩ)
- Measure voltage at C1 - should vary with temperature
- Try different thermistor if available

### **Problem: Voltage readings stopped working**
**Solutions:**
- Check channel 0 wiring to C0
- Verify COM connection to ESP8266 A0
- Test multiplexer power (3.3V at VCC)
- Check control pin connections S0-S3

### **Problem: Both sensors show same reading**
**Solutions:**
- Verify control pins S0-S3 wired correctly
- Check multiplexer switching by manually changing channels
- Ensure EN pin tied to 3.3V (enables multiplexer)
- Replace multiplexer chip if faulty

## 📊 **Expected Performance**

### **Sampling Rate**
- **Total**: 1000 samples/second (500Hz per sensor)
- **Channel switching**: Every 2ms alternating
- **Settling time**: 50μs per channel switch

### **Accuracy**
- **Voltage**: ±0.01V (limited by ESP8266 ADC)
- **Temperature**: ±1°C (with proper calibration)
- **Timing**: ±1ms timestamp accuracy

### **Range**
- **Voltage**: 0-1V (expandable with voltage dividers)
- **Temperature**: -40°C to +125°C (thermistor dependent)
- **Channels**: 16 total available (only 2 used currently)

## 🔮 **Future Expansion Ideas**

The CD74HC4067 has 14 unused channels (C2-C15) for additional sensors:
- **More thermistors** for multi-point temperature monitoring
- **Light sensors** (LDRs) for ambient light logging
- **Pressure sensors** for barometric/altitude data
- **Humidity sensors** (analog output types)
- **Current sensors** for power monitoring

**Your dual sensor logger is ready to expand into a complete environmental monitoring station!** 🌡️📊⚡
