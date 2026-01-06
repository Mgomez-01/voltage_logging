# PWM Frequency Configuration Guide

## Location
**File**: `src/heater_controller.cpp`  
**Line**: 16

```cpp
const int PWM_FREQUENCY = 10; // 10 Hz PWM frequency
```

---

## Quick Change Examples

### Option 1: Lower Frequency (Minimum Losses)
```cpp
const int PWM_FREQUENCY = 1;  // 1 Hz - ultimate low losses
const int PWM_FREQUENCY = 5;  // 5 Hz - very low losses, smoother
```

**Best for:**
- Large heated objects
- Slow temperature response acceptable
- Absolute minimum MOSFET heating

### Option 2: Current (Balanced)
```cpp
const int PWM_FREQUENCY = 10; // 10 Hz - current setting ✅
```

**Best for:**
- General purpose heater control
- Good balance of smoothness and efficiency
- Recommended starting point

### Option 3: Higher Frequency (Smoother)
```cpp
const int PWM_FREQUENCY = 50;  // 50 Hz - smoother, low losses
const int PWM_FREQUENCY = 100; // 100 Hz - very smooth, moderate losses
```

**Best for:**
- Small thermal mass
- Very responsive control needed
- MOSFET has good heatsinking

### Option 4: Very High (Not Recommended for Heaters)
```cpp
const int PWM_FREQUENCY = 1000; // 1 kHz - high losses ⚠️
```

**⚠️ Warning**: Only use for special cases with proper MOSFET heatsinking

---

## Understanding the Trade-offs

### 1. Switching Loss Calculation

**Power dissipated in MOSFET**:
```
P_switch ≈ Frequency × t_switch × V_supply × I_load / 2
```

For AOD4144 @ 12V, 1A load:
- **10 Hz**: P_switch ≈ 0.1W (cool MOSFET)
- **100 Hz**: P_switch ≈ 1W (warm MOSFET)
- **1000 Hz**: P_switch ≈ 10W (hot MOSFET, needs heatsink)

### 2. Thermal Time Constant

**What matters**: How fast your heated object responds

```
Thermal time constant (τ) = Mass × Specific_Heat / Heat_Transfer_Rate
```

**Examples**:
| Object | Thermal τ | Minimum PWM Freq | Recommended Freq |
|--------|-----------|------------------|------------------|
| Small PCB (50g) | ~5 seconds | 0.2 Hz | 1-10 Hz |
| Water (100ml) | ~30 seconds | 0.03 Hz | 1-5 Hz |
| Large thermal mass | minutes | 0.01 Hz | 1-2 Hz |

**Rule of thumb**: If thermal τ > 1 second, use 1-10 Hz PWM

### 3. Control Response

**PID update interval**: 500ms (2 Hz)

Your PID updates twice per second, so:
- PWM faster than 2 Hz: PID sees "averaged" power ✅
- PWM slower than 2 Hz: PID might see pulsing ⚠️

**Recommendation**: Keep PWM ≥ 2× PID rate = 4+ Hz

---

## Frequency Selection Decision Tree

```
START
  │
  ├─ Is thermal time constant < 1 second?
  │   YES → Use 50-100 Hz
  │   NO  → Continue
  │
  ├─ Is MOSFET getting hot?
  │   YES → Lower frequency (1-5 Hz)
  │   NO  → Continue
  │
  ├─ Need ultra-smooth control?
  │   YES → Use 50-100 Hz (with heatsink)
  │   NO  → Use 10 Hz (current setting) ✅
```

---

## Testing Different Frequencies

### Test Procedure:

1. **Start at 10 Hz** (current setting)
2. **Monitor**:
   - Temperature response
   - MOSFET temperature (should be barely warm)
   - Temperature oscillations
3. **Try variations**:
   - Lower (5 Hz): Cooler MOSFET, slightly less smooth
   - Higher (50 Hz): Smoother, slightly warmer MOSFET
4. **Choose best balance**

### What to Look For:

**Temperature Control Quality**:
```
Good: Smooth approach to setpoint, minimal oscillation
Bad: Hunting, oscillating around setpoint
```

**MOSFET Temperature** (touch test):
```
Good: Barely warm or cool
Concern: Noticeably warm
Bad: Hot to touch (reduce frequency!)
```

**PWM Effectiveness**:
```
Monitor actual duty cycle vs. temperature rise
Should be predictable linear relationship
```

---

## Advanced: Adaptive Frequency

If you want to get fancy, you could implement adaptive PWM:

```cpp
void setAdaptivePWMFrequency() {
  if (pidOutput > 80.0) {
    // High power - use low frequency to reduce losses
    analogWriteFreq(5);
  } else if (pidOutput > 20.0) {
    // Medium power - balanced
    analogWriteFreq(10);
  } else {
    // Low power - can use higher frequency
    analogWriteFreq(50);
  }
}
```

**Note**: Call this when PID output changes significantly

---

## Frequency Limits

### ESP8266 Hardware Limits:
- **Minimum**: ~1 Hz
- **Maximum**: ~40 kHz (40,000 Hz)
- **Practical for heaters**: 1-100 Hz
- **Sweet spot**: 5-20 Hz

### AOD4144 MOSFET Limits:
- **Maximum switching frequency**: ~1 MHz (datasheet)
- **Practical with your gate drive**: < 10 kHz
- **Recommended for low losses**: < 100 Hz
- **Optimal for heater control**: 5-20 Hz

---

## Common Misconceptions

### ❌ "Higher frequency = better control"
**Reality**: Control quality depends on PID tuning, not PWM frequency. Above ~10 Hz, no improvement for thermal systems.

### ❌ "1000 Hz is always better"
**Reality**: Wastes power as heat in MOSFET. No benefit for heaters. Use for motors/LEDs only.

### ❌ "Frequency doesn't matter"
**Reality**: It does for efficiency and MOSFET temperature. Too high = hot MOSFET, too low = possible oscillations.

---

## Practical Examples

### Example 1: General Lab Use (Recommended)
```cpp
const int PWM_FREQUENCY = 10; // ✅ Current setting
```
**Why**: Excellent balance, proven in practice

### Example 2: High-Power Heating (Efficiency Priority)
```cpp
const int PWM_FREQUENCY = 5;  // Lower losses
```
**Why**: Reduce MOSFET heating, thermal mass averages it anyway

### Example 3: Small Fast-Response Heater
```cpp
const int PWM_FREQUENCY = 50; // Higher responsiveness
```
**Why**: Small thermal mass benefits from smoother power delivery

### Example 4: Maximum Efficiency
```cpp
const int PWM_FREQUENCY = 2;  // Minimum practical frequency
```
**Why**: Absolute minimum switching losses, still above PID rate

---

## Recommended Settings by Application

| Application | PWM Freq | Rationale |
|-------------|----------|-----------|
| **PCB Reflow** | 10-20 Hz | Medium response, standard practice |
| **Water heating** | 1-5 Hz | Large thermal mass, efficiency priority |
| **3D printer bed** | 10 Hz | Proven effective, balanced |
| **Small elements** | 20-50 Hz | Faster response helpful |
| **Large thermal mass** | 1-5 Hz | Efficiency, smooth is automatic |
| **General purpose** | 10 Hz | Current setting ✅ |

---

## Monitoring & Optimization

### After Changing Frequency:

1. **Check MOSFET temperature** after 5 minutes at 50% duty cycle
   - Should be barely warm
   - If hot, lower frequency or add heatsink

2. **Monitor temperature control quality**
   - Smooth approach to setpoint?
   - Minimal overshoot?
   - Stable at setpoint?

3. **Verify PID performance**
   - No unusual oscillations?
   - Response time acceptable?
   - Duty cycle reasonable?

### Serial Monitor Output:
```
PWM Frequency: 10 Hz
MOSFET Temp: 35°C (normal)
Control Quality: Good
Temperature Stability: ±0.5°C
```

---

## Quick Reference Table

| Parameter | Current | Conservative | Aggressive | Notes |
|-----------|---------|--------------|------------|-------|
| PWM_FREQUENCY | 10 Hz | 5 Hz | 50 Hz | Heaters: favor low |
| MOSFET Temp | ~30°C | <40°C | <60°C | Touch test |
| Switching Loss | ~0.1W | <0.5W | 1-5W | Keep low |

---

## Bottom Line Recommendation

**For your heater control application:**

✅ **Keep it at 10 Hz** (current setting)

**Why:**
1. Proven effective for temperature control
2. Minimal MOSFET heating
3. Smooth enough for thermal systems
4. Standard practice in industry
5. No benefit from going higher

**Only change if:**
- MOSFET gets hot → Lower to 5 Hz
- Very small heater element → Raise to 20 Hz
- Special requirement → Test 5-50 Hz range

**Never go above 100 Hz for heater control** - pure waste!
