# PID Tuning Guide for Heater Control

## Understanding PID Parameters

### What Each Parameter Does

**Kp (Proportional)** - *The "main driver"*
- **Effect**: Immediate response to error
- **Higher Kp**: Faster response, more aggressive
- **Lower Kp**: Slower response, more gentle
- **Too high**: Oscillation, overshoot
- **Too low**: Slow to reach target, sluggish

**Ki (Integral)** - *The "steady-state fixer"*
- **Effect**: Eliminates steady-state error over time
- **Higher Ki**: Faster elimination of offset, but can cause overshoot
- **Lower Ki**: Slower but more stable
- **Too high**: Overshoot, oscillation, instability
- **Too low**: Never quite reaches target (steady-state error)

**Kd (Derivative)** - *The "anticipator"*
- **Effect**: Dampens overshoot, predicts future error
- **Higher Kd**: More damping, smoother response
- **Lower Kd**: Less damping
- **Too high**: Sluggish response, noise sensitivity
- **Too low**: Overshoot, oscillation

---

## Current Settings Analysis

Your current PID values from the code:
```cpp
float pidKp = 2.2;   // Proportional gain
float pidKi = 0.25;  // Integral gain
float pidKd = 0.15;  // Derivative gain
```

**Assessment**: These are reasonable starting values for heater control. They're conservative, which means:
- ✅ Stable operation
- ✅ Unlikely to oscillate
- ⚠️ Might be slow to reach target
- ⚠️ Might have small steady-state error

---

## Method 1: Manual Tuning (Recommended for Beginners)

### Step-by-Step Process

#### Phase 1: Find Kp (Proportional Only)

**Goal**: Get system responding without oscillation

1. **Disable I and D**:
   ```cpp
   float pidKp = 1.0;   // Start here
   float pidKi = 0.0;   // Turn off integral
   float pidKd = 0.0;   // Turn off derivative
   ```

2. **Set target temperature** (e.g., 50°C, well above ambient)

3. **Enable PID and observe**:
   - Does temperature rise toward target?
   - Does it stabilize below target? (Increase Kp)
   - Does it oscillate around target? (Decrease Kp)
   - Does it overshoot significantly? (Decrease Kp)

4. **Adjust Kp**:
   ```
   Kp too low:  Temperature rises slowly, stops below target
   Kp just right: Temperature approaches target, minor overshoot
   Kp too high: Wild oscillations, large overshoot
   ```

5. **Find the "critical Kp"**:
   - Keep increasing Kp until you see sustained oscillation
   - Note this value as **Ku (ultimate gain)**
   - Back off to about 50% of Ku for stable operation

**Example**:
```
Try Kp = 1.0  → Too slow, doesn't reach target
Try Kp = 2.0  → Better, small overshoot
Try Kp = 4.0  → Oscillates!
Critical Ku = 4.0
Final Kp = 2.0 (50% of Ku)
```

#### Phase 2: Add Ki (Integral)

**Goal**: Eliminate steady-state error

1. **Start with Kp from Phase 1**:
   ```cpp
   float pidKp = 2.0;   // From Phase 1
   float pidKi = 0.1;   // Start small
   float pidKd = 0.0;   // Still off
   ```

2. **Observe behavior**:
   - Does it reach exact target over time? (Good)
   - Does it overshoot then settle? (Ki slightly high)
   - Still doesn't reach target? (Ki too low)

3. **Gradually increase Ki**:
   ```
   Ki = 0.05  → Slow to eliminate error
   Ki = 0.1   → Moderate speed
   Ki = 0.25  → Faster elimination
   Ki = 0.5   → May cause overshoot
   ```

4. **Stop when**:
   - Temperature reaches exact target
   - Minimal overshoot (< 2°C)
   - Settling time acceptable

**Typical values for heaters**: Ki = 0.1 to 0.3

#### Phase 3: Add Kd (Derivative)

**Goal**: Reduce overshoot, smooth response

1. **Start with Kp and Ki from previous phases**:
   ```cpp
   float pidKp = 2.0;   // From Phase 1
   float pidKi = 0.2;   // From Phase 2
   float pidKd = 0.05;  // Start very small
   ```

2. **Observe behavior**:
   - Does overshoot decrease? (Good)
   - Does response become too slow? (Kd too high)
   - Still too much overshoot? (Increase Kd)

3. **Fine-tune Kd**:
   ```
   Kd = 0.0   → Some overshoot
   Kd = 0.05  → Reduced overshoot
   Kd = 0.15  → Smooth approach
   Kd = 0.3   → May be too slow
   ```

**Typical values for heaters**: Kd = 0.05 to 0.2

---

## Method 2: Ziegler-Nichols Tuning

### Classic Auto-Tune Method

**Step 1**: Find Critical Values

1. Set Ki = 0, Kd = 0
2. Increase Kp until sustained oscillation occurs
3. Note:
   - **Ku** = Kp value causing oscillation
   - **Tu** = Period of oscillation (seconds)

**Example**:
```
Kp = 4.0 causes oscillation
Tu = 60 seconds (peak to peak)
```

**Step 2**: Calculate PID Values

Use these formulas:

| Control Type | Kp | Ki | Kd |
|--------------|----|----|-----|
| **P only** | 0.5 × Ku | 0 | 0 |
| **PI** | 0.45 × Ku | 1.2 × Kp / Tu | 0 |
| **PID** | 0.6 × Ku | 2 × Kp / Tu | Kp × Tu / 8 |

**Example Calculation** (Ku = 4.0, Tu = 60s):
```cpp
// PID (Classic Ziegler-Nichols)
Kp = 0.6 × 4.0 = 2.4
Ki = (2 × 2.4) / 60 = 0.08
Kd = (2.4 × 60) / 8 = 18.0  // Often too high for heaters!

// Modified for heater control (gentler):
Kp = 2.4
Ki = 0.08
Kd = 0.15  // Manually reduced
```

**Note**: Ziegler-Nichols often produces aggressive values. For heater control, reduce Kd by 10-100×.

---

## Method 3: Practical "Quick Tune" for Heaters

### Fast Method for Typical Heaters

**Step 1**: Start Conservative
```cpp
float pidKp = 2.0;
float pidKi = 0.1;
float pidKd = 0.05;
```

**Step 2**: Test and adjust based on response

| Observed Problem | Solution |
|-----------------|----------|
| Slow to reach target (> 10 minutes) | Increase Kp by 0.5 |
| Overshoots target by > 5°C | Decrease Kp by 0.5 |
| Never quite reaches target | Increase Ki by 0.05 |
| Oscillates ±1-2°C around target | Decrease Kp, increase Kd |
| Oscillates wildly | Decrease all values by 50% |
| Very smooth but slow | Increase Kp and Ki slightly |

**Step 3**: Fine-tune
```cpp
// Good baseline for 50°C setpoint
float pidKp = 2.2;   // Responsive without overshoot
float pidKi = 0.25;  // Eliminates steady-state error
float pidKd = 0.15;  // Smooth approach to target
```

---

## Method 4: Automated Tuning (Advanced)

### Auto-Tune Algorithm

I can add an auto-tune function to your code that:
1. Performs a step response test
2. Analyzes system behavior
3. Calculates optimal PID values

**Would you like me to implement this?** It would:
- Run automatically when requested via web interface
- Take 5-10 minutes to complete
- Store the tuned values
- Show progress on serial monitor

---

## Tuning Tips & Troubleshooting

### Common Response Patterns

#### 1. **Sluggish Response** (Too Conservative)
```
Current: 25°C → 28°C → 31°C → 34°C → 37°C → 40°C (10+ minutes)
Target: 45°C
Problem: Takes forever to reach target
```
**Solution**: Increase Kp by 0.5-1.0

#### 2. **Overshoot** (Too Aggressive)
```
Current: 25°C → 35°C → 48°C → 42°C → 46°C → 44°C → 45°C
Target: 45°C
Problem: Overshoots by 3°C, oscillates
```
**Solution**: Decrease Kp by 0.5, increase Kd by 0.05

#### 3. **Steady-State Error** (Needs Integral)
```
Current: 25°C → 35°C → 40°C → 42°C → 43°C → 43.5°C → stays at 43.5°C
Target: 45°C
Problem: Never reaches target
```
**Solution**: Increase Ki by 0.05-0.1

#### 4. **Oscillation** (Too Much Gain)
```
Current: 44°C → 46°C → 44°C → 46°C → 44°C → 46°C (keeps bouncing)
Target: 45°C
Problem: Won't settle, ±1-2°C oscillation
```
**Solution**: Decrease Kp by 0.3, increase Kd by 0.05

#### 5. **Perfect Response** ✓
```
Current: 25°C → 32°C → 39°C → 44°C → 45.5°C → 45.0°C → stable
Target: 45°C
Problem: None! Small overshoot, quick settling
```
**You're done!** Maybe fine-tune Ki if you see tiny steady-state error.

### Visual Recognition Guide

```
Temperature vs Time Graph:

Kp too low (sluggish):
45°C ┤           _______________
     │          /
     │        /
     │      /
     │    /
25°C └──────────────────────────
     0    5    10   15   20 min

Kp too high (oscillation):
45°C ┤    /\  /\  /\  /\
     │   /  \/  \/  \/  \
     │  /
     │ /
25°C └──────────────────────────
     0    5    10   15   20 min

Good tuning:
45°C ┤       _/‾‾‾‾‾‾‾‾‾‾‾‾‾
     │      / 
     │     /
     │   /
25°C └──────────────────────────
     0    5    10   15   20 min
```

---

## Recommended Starting Points by Application

### Water Heating (0-80°C)
```cpp
float pidKp = 3.0;   // Higher Kp for liquid thermal mass
float pidKi = 0.15;  // Moderate integral
float pidKd = 0.2;   // Dampen liquid inertia
```

### PCB Reflow (0-150°C)
```cpp
float pidKp = 2.0;   // Moderate response
float pidKi = 0.25;  // Good steady-state accuracy
float pidKd = 0.15;  // Smooth temperature curve
```

### 3D Printer Hotend (0-250°C)
```cpp
float pidKp = 20.0;  // Very responsive (low thermal mass)
float pidKi = 1.0;   // Quick integral action
float pidKd = 0.5;   // Dampen overshoot
```

### Laboratory Oven (0-120°C)
```cpp
float pidKp = 1.5;   // Gentle for large thermal mass
float pidKi = 0.1;   // Slow integral (takes time to heat)
float pidKd = 0.3;   // Prevent overshoot
```

### Your Current Setup (General Purpose)
```cpp
float pidKp = 2.2;   // Good starting point
float pidKi = 0.25;  // Eliminate steady-state error
float pidKd = 0.15;  // Smooth approach
```

---

## Live Tuning via Web Interface

### Current Method
You can already tune PID via your web interface:
1. Go to `http://192.168.4.1` (or your ESP8266 IP)
2. Use the PID parameter inputs
3. Click "Update PID Parameters"
4. Watch serial output for response

### Serial Monitor Feedback
Your code already shows:
```
PID: Target=45.00°C, Current=38.29°C, Error=6.71, Output=14.8%, PWM=14.8%
PID DETAIL: P=14.76, I=2.15, D=0.23, Sum=17.1%
```

Use this to see how parameters affect P, I, and D contributions!

---

## Step-by-Step Tuning Session Example

### Session 1: Finding Kp

**Initial Setup**:
```cpp
pidKp = 1.0;
pidKi = 0.0;
pidKd = 0.0;
targetTemperature = 50.0;
```

**Test 1**: Kp = 1.0
```
Time  Temp  Error  Output  Observation
0:00  25°C  25.0   25.0%   Heating slowly
1:00  30°C  20.0   20.0%   
2:00  35°C  15.0   15.0%   
5:00  45°C   5.0    5.0%   Stops at 45°C - too low!
```
**Conclusion**: Increase Kp

**Test 2**: Kp = 2.0
```
Time  Temp  Error  Output  Observation
0:00  25°C  25.0   50.0%   Heating faster
1:00  35°C  15.0   30.0%   
2:00  43°C   7.0   14.0%   
3:00  49°C   1.0    2.0%   
3:30  50°C   0.0    0.0%   Perfect!
```
**Conclusion**: Kp = 2.0 is good, add Ki to eliminate any drift

### Session 2: Adding Ki

**Setup**:
```cpp
pidKp = 2.0;  // From Session 1
pidKi = 0.2;  // Adding integral
pidKd = 0.0;
```

**Test 3**: Ki = 0.2
```
Time  Temp  Error  Output  Observation
0:00  25°C  25.0   50.0%   Similar start
2:00  43°C   7.0   15.4%   
3:00  49°C   1.0    3.2%   I building up
3:30  50.2°C -0.2  -0.4%   Slight overshoot
4:00  50.0°C  0.0   0.8%   Settles perfectly!
```
**Conclusion**: Ki = 0.2 works well

### Session 3: Adding Kd

**Setup**:
```cpp
pidKp = 2.0;
pidKi = 0.2;
pidKd = 0.1;  // Adding derivative
```

**Test 4**: Kd = 0.1
```
Time  Temp  Error  Output  Observation
0:00  25°C  25.0   50.0%   
2:00  43°C   7.0   14.3%   Derivative reduces output
3:00  49°C   1.0    2.5%   Smoother approach
3:20  50.0°C  0.0   0.5%   No overshoot! Perfect!
```
**Conclusion**: Final values found!

**Final Tuned Values**:
```cpp
float pidKp = 2.0;
float pidKi = 0.2;
float pidKd = 0.1;
```

---

## PID Tuning Checklist

- [ ] Understand thermal mass of your system
- [ ] Start with conservative values (Kp=2, Ki=0.1, Kd=0.05)
- [ ] Test at typical operating temperature
- [ ] Tune Kp first (Ki=0, Kd=0)
- [ ] Add Ki for steady-state accuracy
- [ ] Add Kd to reduce overshoot
- [ ] Document your final values
- [ ] Test at different setpoints (40°C, 60°C, 80°C)
- [ ] Verify stability over 30+ minute run
- [ ] Check response to disturbances (opening enclosure, etc.)

---

## Quick Reference: What to Adjust

| Problem | Kp | Ki | Kd |
|---------|----|----|-----|
| Too slow to reach target | ↑ | — | — |
| Overshoots target | ↓ | — | ↑ |
| Never reaches target exactly | — | ↑ | — |
| Oscillates around target | ↓ | ↓ | ↑ |
| Reaches target then drifts away | — | ↑ | — |
| Response too aggressive | ↓ | ↓ | — |
| Response too sluggish | ↑ | ↑ | — |

**Legend**: ↑ = Increase, ↓ = Decrease, — = Keep same

---

## Advanced: Auto-Tune Implementation

Would you like me to add an auto-tune feature? It would:

**Features**:
- One-button start via web interface
- Performs relay test (bang-bang control)
- Measures system response
- Calculates optimal PID values
- Stores values automatically
- Shows progress in serial monitor

**Usage**:
1. Click "Auto-Tune PID" button
2. Wait 5-10 minutes
3. System tests itself and calculates values
4. New PID values applied automatically

**Let me know if you want this feature added!**

---

## Summary: Your Action Plan

**Current Values** (already good):
```cpp
pidKp = 2.2;
pidKi = 0.25;
pidKd = 0.15;
```

**Recommended Test**:
1. Set target to 50°C
2. Enable PID (data logging can be OFF)
3. Watch serial output:
   - Should reach 50°C in 5-8 minutes
   - Overshoot should be < 2°C
   - Should settle and stay at 50°C ± 0.5°C

**If not satisfactory**, follow Manual Tuning Method 1 above.

**Pro Tip**: Your current values are already well-tuned for general heater control. Unless you see problems (slow response, overshoot, oscillation), you can probably use them as-is!
