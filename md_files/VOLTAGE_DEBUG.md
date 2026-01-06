# 🔍 Enhanced Debugging - Voltage Discrepancy Analysis

## **🎯 What I Added**

### **1. Live vs Buffered Comparison**
- Shows current live ADC reading
- Shows last 5 buffered readings that go to CSV
- Calculates real-time buffer statistics

### **2. File Write Statistics**  
- Shows min/max/average of each 100-reading batch written to CSV
- Tracks exactly what values are being saved

### **3. WebSocket Message Details**
- Shows which buffered voltage values are sent to web interface
- Confirms data path from buffer → WebSocket → web display

## **🚀 Deploy and Monitor**

```bash
make deploy
```

## **📋 What to Watch For**

### **In Serial Monitor, look for:**

**Live vs Buffered readings:**
```
LIVE ADC reading: 842 -> 0.821300V
Last 5 BUFFERED readings:
  [99] 0.550000V @ 45000ms
  [98] 0.552000V @ 44999ms  
  [97] 0.548000V @ 44998ms
```

**File write statistics:**
```
Wrote 100 readings to file - Batch stats: Min=0.520000V, Max=0.580000V, Avg=0.550000V
```

**WebSocket messages:**
```
WebSocket message #100 sent to 1 clients: {"timestamp":45000,"voltage":0.550000,"type":"reading"} (buffer idx: 99, buffered voltage: 0.550000V)
```

## **🔬 Possible Scenarios**

### **Scenario 1: Rapid Voltage Fluctuation**
If you see:
- Live readings: consistently ~0.82V
- Buffered readings: varying from 0.5V to 0.8V  
- CSV average: ~0.55V

**Cause**: Your circuit voltage is fluctuating rapidly, and the 1ms sampling is catching the full range while occasional spot-checks hit the peaks.

### **Scenario 2: ADC Sampling Timing**
If you see:
- Live readings: one value
- Buffered readings: completely different values
- Pattern in the differences

**Cause**: Timing differences in when ADC is sampled during different operations.

### **Scenario 3: Circuit Loading**
If you see differences correlating with activity:
- Higher voltage when ESP8266 is less active
- Lower voltage during WiFi transmission
- Changes during file writes

**Cause**: Your circuit might be affected by ESP8266's power consumption changes.

## **🎯 Expected Debug Output**

After deploying, you should see much more detailed output like:
```
=== DEBUG STATS ===
Uptime: 120 seconds
WiFi Status: AP Mode - Connected stations: 1
WebSocket clients: 1
Total ADC readings: 12000
Current buffer index: 50/100 (50% full)
WebSocket messages sent: 1200

LIVE ADC reading: 842 -> 0.821300V

Last 5 BUFFERED readings:
  [49] 0.552000V @ 120450ms
  [48] 0.548000V @ 120449ms
  [47] 0.551000V @ 120448ms
  [46] 0.549000V @ 120447ms
  [45] 0.550000V @ 120446ms

Buffer stats - Min: 0.520000V, Max: 0.580000V, Avg: 0.550000V (n=50)

Free heap: 31112 bytes
==================
```

**Deploy this version and share the detailed debug output!** This will show us exactly where the 0.82V vs 0.55V discrepancy is coming from.

Most likely explanation: **Your voltage is actually fluctuating rapidly** and the fast sampling (1000 Hz) is capturing the true average, while occasional spot-checks happen to hit consistent peak values.
