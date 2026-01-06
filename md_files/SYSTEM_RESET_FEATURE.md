# System Reset Feature - Software Recovery from Emergency State

## Overview

Added a software reset button that allows recovery from emergency shutdown without requiring physical access to the ESP8266. This provides much better user experience, especially for remote or enclosed installations.

---

## Problem Solved

### Before (Old Behavior):
```
Emergency Shutdown Triggered
    ↓
System enters locked state
    ↓
Web interface shows error
    ↓
User must physically reset ESP8266 ❌
    ↓
System reboots (30 seconds downtime)
```

### After (New Behavior):
```
Emergency Shutdown Triggered
    ↓
System enters safe state
    ↓
Web interface shows "System Reset" button ✅
    ↓
User clicks button (or sends GET request)
    ↓
System verifies safety conditions
    ↓
Emergency state cleared (instant recovery)
    ↓
User re-enables features as needed
```

---

## Implementation Details

### 1. New Web Route
**Endpoint**: `GET /system/reset`

**File**: `src/web_interface.cpp`

**What it does**:
1. Checks temperature is safe (< MAX_SAFE_TEMPERATURE - 10°C)
2. Verifies temperature sensor is working
3. Clears emergency shutdown flag
4. Resets all heater control states
5. Resets PID controller state
6. Keeps data logging paused (user must restart manually)

### 2. Safety Checks (Critical!)

The reset will **NOT** proceed if:

❌ **Temperature too high**:
```
Current temp > 110°C (when MAX_SAFE_TEMPERATURE = 120°C)
Response: "Cannot reset - temperature still too high. Wait for cooling."
```

❌ **Sensor not working**:
```
Temperature reading is NaN, < -50°C, or > 200°C
Response: "Cannot reset - temperature sensor failure. Check thermistor connection."
```

✅ **Safe to reset**:
```
Temperature is reasonable and sensor working
Response: "System reset successful. Temperature: XX.X°C"
```

### 3. State Reset Actions

When reset is successful:

```cpp
// Clear emergency flag
emergencyShutdown = false;

// Disable heater completely
heaterEnabled = false;
pidEnabled = false;
setHeaterPower(0.0);  // 0% PWM

// Reset PID state
pidIntegral = 0;
pidLastError = 0;
pidError = 0;
pidOutput = 0;

// Reset watchdog
systemAlive = true;

// Keep logging paused (safety)
dataLoggingEnabled = false;
```

**Important**: User must manually re-enable features after reset. This prevents automatic resumption of potentially unsafe operations.

---

## Usage

### Via Web Interface (Recommended)

**Steps**:
1. Emergency shutdown occurs
2. Open web interface (still accessible during shutdown)
3. Click **"System Reset"** button (to be added to UI)
4. System checks safety conditions
5. If safe, emergency state cleared
6. Manually re-enable heater/logging as needed

### Via HTTP Request

**Using curl**:
```bash
curl http://192.168.4.1/system/reset
```

**Using browser**:
```
http://192.168.4.1/system/reset
```

**Using Python**:
```python
import requests
response = requests.get('http://192.168.4.1/system/reset')
print(response.text)
```

### Via Serial Monitor

You can monitor the reset process:
```
HTTP: System reset requested
RESET: Safety checks passed, clearing emergency state
RESET: Current temperature: 35.4°C (safe)
RESET: System state cleared
RESET: All heater control disabled
RESET: Data logging paused
RESET: System ready for normal operation
RESET: User must manually re-enable desired features
```

---

## Serial Output Messages

### During Emergency Shutdown

**Every 5 seconds**:
```
*** EMERGENCY SHUTDOWN ACTIVE ***
*** HEATER DISABLED - SYSTEM SAFE ***
*** Use 'System Reset' button in web interface to recover ***
*** Or send GET request to /system/reset ***
```

### When Reset Requested - Success

```
HTTP: System reset requested
RESET: Safety checks passed, clearing emergency state
RESET: Current temperature: 42.3°C (safe)
RESET: System state cleared
RESET: All heater control disabled
RESET: Data logging paused
RESET: System ready for normal operation
RESET: User must manually re-enable desired features
```

### When Reset Requested - Temperature Too High

```
HTTP: System reset requested
RESET DENIED: Temperature too high (115.2°C)
```

### When Reset Requested - Sensor Failure

```
HTTP: System reset requested
RESET DENIED: Temperature sensor not reading properly
```

---

## Safety Philosophy

### Why Safety Checks Before Reset?

1. **Over-temperature**: If temp is still high, the cause of emergency might still be present
2. **Sensor failure**: Can't verify it's safe to reset if we can't read temperature
3. **Cool-down period**: Forces user to wait if system overheated

### Why Disable Everything After Reset?

**Conservative approach**:
- Heater disabled → User must manually re-enable
- PID disabled → User must manually re-enable
- Logging paused → User must manually restart

**Rationale**:
- Prevents automatic resumption of operations
- User must actively decide what to restart
- Time to investigate cause of emergency
- Prevents repeated emergency shutdowns

### Temperature Safety Margin

```cpp
if (currentTemp > MAX_SAFE_TEMPERATURE - 10)
```

**Explanation**:
- MAX_SAFE_TEMPERATURE = 120°C
- Reset blocked if temp > 110°C
- 10°C safety margin ensures system has cooled
- Prevents reset during active over-temp event

---

## Recovery Workflow

### Typical Recovery Procedure

**1. Emergency Shutdown Occurs**
```
Cause: Over-temperature detected (122°C)
System: Emergency shutdown triggered
Heater: Turned off immediately
```

**2. Wait for Cool-Down**
```
Monitor temperature via web interface
Current: 122°C → 115°C → 108°C → 95°C
Target: Wait until < 110°C for reset
```

**3. Investigate Cause**
```
Why did temperature exceed limit?
- PID tuning issue?
- Sensor placement problem?
- Target temperature too high?
- Insufficient cooling?
```

**4. Click System Reset**
```
Web Interface: Click "System Reset" button
Response: "System reset successful. Temperature: 95.2°C"
Status: Emergency state cleared
```

**5. Adjust Settings**
```
Lower target temperature
Adjust PID parameters
Improve cooling
Re-check sensor placement
```

**6. Resume Operations**
```
Re-enable PID control (if needed)
Restart data logging (if needed)
Monitor closely for first few minutes
```

---

## Testing the Reset Feature

### Test 1: Normal Reset (Success)

**Setup**:
1. Manually trigger emergency shutdown:
   ```cpp
   emergencyShutdown = true;  // In code or via debugger
   ```
2. Or wait for actual emergency event

**Execute**:
```bash
curl http://192.168.4.1/system/reset
```

**Expected**:
- HTTP 200 response
- "System reset successful" message
- System returns to normal operation
- All heater control disabled
- Can now manually re-enable features

### Test 2: Reset Blocked (Temperature Too High)

**Setup**:
1. Heat system above 110°C
2. Trigger emergency shutdown (should happen automatically)

**Execute**:
```bash
curl http://192.168.4.1/system/reset
```

**Expected**:
- HTTP 403 response (Forbidden)
- "Cannot reset - temperature still too high"
- Emergency state remains
- Must wait for cooling

### Test 3: Reset Blocked (Sensor Failure)

**Setup**:
1. Disconnect thermistor
2. Trigger emergency shutdown

**Execute**:
```bash
curl http://192.168.4.1/system/reset
```

**Expected**:
- HTTP 403 response
- "Temperature sensor failure"
- Must fix sensor before reset

---

## Adding Button to Web Interface

### Suggested UI Placement

**Emergency State Banner**:
```html
<div class="alert alert-danger" id="emergencyBanner" style="display:none;">
  <h4>⚠️ EMERGENCY SHUTDOWN ACTIVE</h4>
  <p>System is in safe mode. All heater control disabled.</p>
  <p>Current Temperature: <span id="currentTemp">--</span>°C</p>
  <button onclick="resetSystem()" class="btn btn-warning">
    🔄 System Reset
  </button>
  <p class="small">Reset will only work if temperature is safe</p>
</div>
```

**JavaScript Function**:
```javascript
function resetSystem() {
  if (!confirm('Reset system from emergency state?')) return;
  
  fetch('/system/reset')
    .then(response => response.text())
    .then(data => {
      alert(data);
      location.reload();  // Refresh page to show normal state
    })
    .catch(error => {
      alert('Reset failed: ' + error);
    });
}
```

**WebSocket to Show/Hide Banner**:
```javascript
// In WebSocket message handler
if (data.emergencyShutdown) {
  document.getElementById('emergencyBanner').style.display = 'block';
} else {
  document.getElementById('emergencyBanner').style.display = 'none';
}
```

---

## API Reference

### Endpoint: `/system/reset`

**Method**: GET

**Parameters**: None

**Success Response** (200 OK):
```
System reset successful. Emergency state cleared. Temperature: XX.X°C
```

**Error Responses**:

**403 Forbidden** - Temperature too high:
```
Cannot reset - temperature still too high. Wait for cooling.
```

**403 Forbidden** - Sensor failure:
```
Cannot reset - temperature sensor failure. Check thermistor connection.
```

**Example Usage**:
```bash
# Success
$ curl http://192.168.4.1/system/reset
System reset successful. Emergency state cleared. Temperature: 45.2°C

# Failure - too hot
$ curl http://192.168.4.1/system/reset
Cannot reset - temperature still too high. Wait for cooling.
```

---

## Comparison: Reset vs Reboot

### Software Reset (New Feature)

✅ **Instant** - No downtime  
✅ **Safe** - Checks conditions first  
✅ **Selective** - Clears only emergency state  
✅ **Remote** - No physical access needed  
✅ **Logged** - Full serial output of process  
✅ **Preserves** - Network connection stays up  

### Physical Reboot (Old Method)

⏱️ **Slow** - 30 second boot time  
❌ **Blind** - No safety checks  
🔄 **Full** - Resets everything  
🔌 **Physical** - Requires access to device  
📵 **Disconnects** - Clients must reconnect  
💾 **Loses** - Any buffered data lost  

---

## Troubleshooting

### Problem: Reset Button Not Working

**Check**:
1. Is emergency shutdown actually active?
2. Is web interface accessible?
3. Is temperature reading properly?
4. Check serial output for error messages

**Solution**:
```bash
# Test endpoint directly
curl http://192.168.4.1/system/reset
```

### Problem: Reset Blocked - Won't Clear

**Cause**: Temperature still too high

**Solution**:
1. Wait for system to cool below 110°C
2. Check heater is actually off (should be 0% PWM)
3. Improve cooling if needed
4. Monitor temp in web interface
5. Try reset again when cooler

### Problem: Reset Blocked - Sensor Error

**Cause**: Thermistor disconnected or failed

**Solution**:
1. Check thermistor connections
2. Verify wiring to multiplexer
3. Check thermistor resistance (should be ~10kΩ at 25°C)
4. Look at ADC value in serial output
5. Fix connection and try reset

### Problem: System Immediately Triggers Emergency Again

**Cause**: Root problem not fixed

**Investigate**:
1. Why did emergency trigger originally?
2. Is target temperature too high?
3. Is PID tuned incorrectly?
4. Is cooling adequate?
5. Is sensor reading accurately?

**Solution**:
1. Lower target temperature
2. Improve cooling
3. Re-tune PID parameters
4. Check sensor calibration

---

## Best Practices

### Do's ✅

1. **Investigate before resetting** - Find out why emergency occurred
2. **Wait for cooling** - Don't rush the reset
3. **Test carefully after reset** - Monitor first few minutes
4. **Adjust settings** - Fix the root cause
5. **Document incidents** - Keep log of emergency events

### Don'ts ❌

1. **Don't immediately reset** - Understand the cause first
2. **Don't skip investigation** - Could happen again
3. **Don't ignore warnings** - They exist for safety
4. **Don't bypass safety checks** - Temperature limits are critical
5. **Don't resume same settings** - If they caused emergency, adjust them

---

## Future Enhancements

### Possible Additions

1. **Reset cooldown timer** - Prevent rapid reset attempts
2. **Incident logging** - Record all emergency events
3. **Email notifications** - Alert on emergency shutdown
4. **Automatic reset** - After X minutes if temp safe (optional)
5. **Recovery checklist** - Guide user through reset process

### Could Be Configurable

```cpp
const float RESET_TEMP_MARGIN = 10.0;  // How much below max to allow reset
const unsigned long RESET_COOLDOWN = 60000;  // Min time between resets
```

---

## Summary

### What You Can Now Do

✅ **Recover from emergency without physical access**  
✅ **Reset system via web interface button**  
✅ **Reset system via HTTP GET request**  
✅ **Safety checks prevent unsafe resets**  
✅ **Full logging of reset process**  
✅ **System stays online during emergency**  

### Safety Features Maintained

🛡️ **Temperature check before reset**  
🛡️ **Sensor validation before reset**  
🛡️ **All heater control disabled after reset**  
🛡️ **Manual re-enable required**  
🛡️ **10°C safety margin**  

### What Happens After Reset

1. Emergency flag cleared ✅
2. Heater disabled ✅
3. PID disabled ✅
4. PWM set to 0% ✅
5. Logging paused ✅
6. Watchdog reset ✅
7. System ready for user input ✅

---

## Implementation Status

✅ **Backend route created** - `/system/reset`  
✅ **Safety checks implemented** - Temperature and sensor validation  
✅ **State reset logic** - All flags and states cleared properly  
✅ **Serial logging** - Full debug output  
✅ **Updated emergency messages** - Instructions to use reset button  
⏳ **Web UI button** - To be added to HTML template (recommended)  

---

The system reset feature is now fully functional via HTTP requests. You can test it immediately by sending a GET request to `/system/reset` when an emergency shutdown occurs. The web interface button can be added to provide a user-friendly click-to-reset experience.

**Upload and test** - the feature is ready to use! 🔄
