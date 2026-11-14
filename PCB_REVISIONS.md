# PCB revisions: changes for the next board (rev B)

What the `pcb_voltage_logging` board (rev A, fab outputs of 2026-11-13, see [PCB_IMPLEMENTATION.md](PCB_IMPLEMENTATION.md)) needs to work the way the project is used: a standalone logger and heater controller, run unattended for up to 6 hours, powered from its 12 V supply.

These come from a review of the rev A schematic and netlist (`pcb_voltage_logging.kicad_sch` / `.kicad_pcb`) against the firmware on the `pcb_update` branch. Rev A should still work for bring-up and bench use with USB connected; nothing here blocks testing it.

| # | Change | Priority | Firmware change |
|---|---|---|---|
| 1 | 12 V → 5 V buck regulator feeding VIN | **Must** (for standalone use) | None |
| 2 | Reverse-polarity protection on J1 | **Must** (with #1) | None |
| 3 | Hardware over-temperature cut-off in series with the heater | **Must** | None |
| 4 | Move SD chip select off GPIO15 (boot strap) | **Should** | `SD.begin(4)` |
| 5 | 12 V supply monitor on MCP3208 CH0 | Should | Read CH0, log it in `voltage` |
| 6 | Filtered analog +3V3 for VREF and the NTC dividers | Should | None |
| 7 | Fix the J7 spare header (labels, pin order, input protection) | Should | Channel map comment |
| 8 | Wire the microSD card-detect switch | Nice | Check the pin before SD access |
| 9 | Test points, power LED | Nice | None |
| 10 | Layout check: heater return current away from the ADC ground | Check | None |

---

## 1. 12 V → 5 V regulator: needed

**Determination: yes, add one.** Rev A can't run from its own 12 V supply.

**How rev A is powered.** In the netlist, +12V_IN (J1) only reaches F1, the heater (J2), D1, C4 and Q1. +5V_VIN connects only to the NodeMCU's VIN pin, the WS2812B (D2), the level shifter (U3) and C5/C6. So everything except the heater (ESP8266, MCP3208, SD card, LED) runs from the NodeMCU's micro-USB.

**What that means:**
- The board only works with a USB cable plugged into a PC, a USB charger or a power bank, as well as the 12 V supply. Two supplies for one instrument, and a loose USB cable stops a 6 h run.
- If USB drops mid-run, the ESP8266 resets and R3 turns the heater off. That is safe, but the run and the rest of the log are lost.
- When USB comes from a PC, the 12 V supply's ground is tied to the PC's ground through the board. That is normal, but it is one more ground path near the ADC.

**Why VIN can't simply be fed with 12 V:**
- The WS2812B (max ~5.3 V) and the 74AHCT1G125 (max 5.5 V) sit on VIN and would be destroyed.
- The NodeMCU's AMS1117-type 3.3 V regulator would have to drop 8.7 V at ~80 mA average and 300+ mA WiFi peaks, about 0.7 W average, which is too much for its SOT-223 with almost no copper.

**Suggested part: Diodes Inc. AP63205WU-7.** It is a synchronous buck with a fixed 5.0 V output, 3.8–32 V input, 2 A, 1.1 MHz, in TSOT-26. Datasheet parts list:
- 4.7 µH inductor
- 10 µF input capacitor (rated ≥ 25 V)
- 2 × 22 µF output capacitors
- 100 nF bootstrap capacitor (BST to SW)

The load is under 0.5 A worst case, so a small shielded inductor (≥ 1 A saturation) is plenty. Put it on +12V **after** F1 and the reverse-polarity protection (#2).

A linear regulator (7805 or LM1117-5.0 in DPAK) also works, but drops 7 V × 0.1–0.3 A = 0.7–2 W. That is warm, and the heat sits on the same board as the thermistor inputs.

**Feeding VIN without fighting USB.** Some NodeMCU v2 boards have a Schottky diode between USB 5 V and VIN, and some connect them directly.
- Add a Schottky (e.g. SS14 / B5819W, SOD-123) from the buck output to +5V_VIN. With USB also plugged in, whichever supply is higher feeds VIN, and the buck never drives current back into the PC's USB port.
- VIN then sits at ~4.7 V on the buck, which is fine for the NodeMCU regulator, the WS2812B (≥ 3.5 V) and the AHCT buffer.
- Check the NodeMCU actually used: with USB unplugged, VIN should not appear on the USB connector's VBUS pin.

**Keep USB working** for flashing and the serial monitor. With the buck fitted, USB stays optional.

## 2. Reverse-polarity protection on J1

There is nothing on rev A to stop a reversed 12 V supply. With J1 reversed:
- Current flows through Q1's body diode and then D1 (SS34) straight back to the supply: two forward diodes in series, effectively a short.
- F1 (1.5 A hold) is in that path and eventually trips, but D1 and Q1 carry several amps until it does.
- With the buck from #1 added, a reversed input would also destroy the buck IC.

**Fix:** a P-channel MOSFET in the +12 V line, after J1 and before F1:
- Drain to J1+, source to the board's +12V_IN, gate to GND through ~10k.
- 15 V Zener from gate to source, so the gate is protected if the supply spikes.
- Pick a P-FET rated ≥ 30 V with low Rds(on) at Vgs = −10 V. For example, the AO3401A (SOT-23, ~45 mΩ, ~0.1 W at 1.5 A) is fine for this heater; use a DPAK part for a larger one.

A series Schottky (another SS34) is simpler but drops ~0.5 V. That is ~0.75 W at 1.5 A, and the heater loses that voltage.

## 3. Hardware over-temperature cut-off

All the over-temperature protection on rev A is in firmware:
- the 130 °C shutdown
- the sensor-fault shutdown
- the 6 h limit
- the watchdog

None of it can help if Q1 fails short (the usual MOSFET failure mode) or the gate is stuck high. The heater then runs at full power from 12 V until something melts, during runs that are meant to be left alone for hours.

**Fix:** a normally-closed thermal cut-off in series with the heater, mounted on the heater sheet:
- e.g. a KSD9700 bimetal thermostat, opening at ~130–140 °C, above the 120 °C sheet limit and the 130 °C firmware limit
- or a one-shot thermal fuse
- On the board: a second 2-pin connector in series with J2 (heater → J2 → cut-off connector → Q1 drain), or a 3-pin heater connector.
- Fit a jumper footprint across it for bench testing.

**Optional:** route the cut-off's state to a spare GPIO (D2/GPIO4, or D3 with a pull-up), so the firmware can report "hardware cut-off open" instead of seeing the heater do nothing.

## 4. Move the SD chip select off GPIO15

GPIO15 (D8) must be LOW when the ESP8266 resets, or it doesn't boot from flash. On rev A it drives the card's CS (DAT3) directly through R11 (47 Ω).
- An SD card has an internal pull-up on CD/DAT3 at power-up. The spec allows 10–90 kΩ.
- That pull-up fights the NodeMCU's own GPIO15 pull-down (about 10–12 kΩ on most boards). With a card at the low end, D8 can sit near 1.5–1.8 V at reset, and the board won't boot with that card inserted.
- The old breadboard SD modules mostly had a 74LVC125 buffer between the card and the pins, which hid this. Rev A connects the card directly.

**Check on rev A:** boot with several different cards inserted. If any card stops it booting, this fix is needed.

**Fix (pick one):**
- **Preferred:** move SD CS to **D2 / GPIO4**. It is free, not a boot pin, and the trace is short. Leave D8 unconnected. Firmware: `SD.begin(4)` instead of `SD.begin(SS)` in `data_manager.cpp`.
- **Or:** keep D8 and add a 4.7k pull-down from D8 to GND on the board, so the divider against even a 10k card pull-up stays below ~0.9 V. No firmware change.

## 5. 12 V supply monitor on MCP3208 CH0

CH0 is floating on rev A. Using it to watch the heater supply turns a currently invisible failure into one the firmware can report. Today, if the 12 V supply is off, unplugged or its PTC has tripped, the PID just sits at 100 % with no temperature rise.

**Circuit:**
- +12V (after F1) → 100k → CH0 → 22k → GND, with 100 nF from CH0 to GND.
- That gives 12 V → 2.16 V (code ~2680), and full scale at ~18.3 V.
- Use 1 % resistors. The reading is relative to +3V3, so expect about ±2 % absolute, which is plenty to tell 12 V from 0 V or a sagging supply.

**Firmware:**
- read CH0 each sample cycle
- log the supply voltage in the existing `voltage` column (always 0 on rev A), so the column means something again
- warn, or stop the PID, if the heater is on and the supply is below ~10 V

**Minimum fix if not used:** tie CH0 to GND through 10k, so it doesn't float.

## 6. Filtered analog supply for VREF and the NTC dividers

On rev A, +3V3 comes straight from the NodeMCU's regulator. The same rail runs WiFi, with ~300 mA transmit bursts, and the SD card. The measurement is ratiometric, so slow rail changes cancel, but fast ones don't:
- VREF only has C7 (1 µF).
- The NTC nodes are low-pass filtered by C8/C9 (time constants 0.2–5 ms).
- So a burst can shift VREF without shifting the inputs.

The firmware averages ~83 samples per PID update, which hides most of this. Fixing it at the source makes single raw rows cleaner too.

**Fix:**
- Add a "+3V3A" node: +3V3 → ferrite bead (e.g. 600 Ω at 100 MHz) or 10 Ω → +3V3A, with 10 µF + 100 nF to GND.
- Feed the MCP3208 VREF and the top of R1 and R2 from +3V3A.
- VDD can stay on +3V3 or move to +3V3A as well. The MCP3208 draws < 0.5 mA, so 10 Ω drops < 5 mV.
- Keep VREF, the R1/R2 tops and C8/C9 grounds close together, so they all see the same node.

## 7. J7 spare header

On rev A, J7 pins 1–5 carry CH7…CH3, which is reversed from what you'd expect, and the silkscreen nets are still named after the old mux ("MUX_C3…C7"). The inputs go straight to the MCP3208 pins with no protection.

**Changes:**
- Rename the nets ADC_CH3…ADC_CH7 and silkscreen each pin with its channel.
- Either reverse the order (pin 1 = CH3) or label it clearly.
- Put a 1k series resistor and a 100 nF cap to GND (footprints at least) on each input. A sensor plugged in live, or a voltage above 3.3 V, can then only push limited current into the MCP3208's clamp diodes.
- Keep pin 6 = +3V3A (from #6) and pin 7 = GND, so a third thermistor divider can be built on J7.

## 8. microSD card-detect

The Hirose DM3AT has a card-detect switch, but it isn't connected on rev A, so the firmware can't tell whether a card is in. Today it finds out when `SD.begin()` fails. With D2 used for SD CS (#4), D3/GPIO0 or D4/GPIO2 could take card-detect, but both are boot pins: use the switch so it pulls the pin **high**, or not at all, at reset.

**Simplest:**
- Leave both detect pins on a footprint with a 10k pull-up to +3V3.
- Route to a free GPIO only if the firmware is going to use it, e.g. to show "no SD card" on the page and LED, or to refuse Start Logging straight away.

## 9. Test points and a power LED

- **Test points:**
  - +12V (after F1)
  - +5V_VIN
  - +3V3 / +3V3A
  - GND (two)
  - PID_NTC and AUX_NTC (the C8/C9 top pads are used today)
  - SPI SCK, MISO, MOSI, ADC_CS and SD CS, for a logic analyser
- **Power LED:** a small LED on +5V_VIN (and one on +12V after F1) shows at a glance which supply is present. It also shows when F1 has tripped, if added after F1.

## 10. Layout check: heater return current

The heater current (up to 1.5 A, switched at 32 Hz) returns to J1 through the board's ground. If that path runs under U2 or between U2's AGND and the J4/J5 ground pins, the PWM edges appear as a voltage between the NTC ground and the ADC ground. That shows up as heater-synchronous noise in the readings.

Check this on rev A by logging raw rows with the heater at 50 % and looking for a 32 Hz pattern in `thermistor_adc`. For rev B:
- Take the heater return from Q1's source straight to J1's GND pin with its own wide trace or pour region.
- Join it to the logic ground at one point near J1.

---

## Already fine on rev A (no change needed)

**Heater MOSFET drive at 3.3 V.** The AOD4144 (30 V, Vgs(th) 1.2–2.4 V, Rds(on) rated at 4.5 V) runs from GPIO16 through R4, with R3 holding it off at reset. That is enough at the ≤ 1.5 A F1 allows. A heater over ~18 W would need:
- a gate driver from the new 5 V rail
- a larger F1
- re-checking D1

**WS2812B data from RX/GPIO3** through the 74AHCT1G125 works with the firmware's NeoPixelBus DMA driver. The cost is a TX-only serial port and some LED flicker while flashing. If the RX pin is wanted back for serial input, move LED_DATA to D4/GPIO2 (not a boot problem for an output that idles high, through the buffer). The firmware would then need the UART1 method (`NeoEsp8266Uart1800KbpsMethod`), which is fixed to GPIO2.

**MCP3208 on the shared SPI bus with the SD card**, with series resistors on SCK, MOSI and SD CS, works. The firmware gives each device its own SPI settings (1 MHz for the ADC).

**D3/GPIO0 and D4/GPIO2 left floating** is fine. The NodeMCU pulls them up.

## Sources

- AP63205 datasheet (Diodes Inc.): input 3.8–32 V, fixed 5.0 V, 2 A, 1.1 MHz, TSOT-26; 4.7 µH, 10 µF in, 2 × 22 µF out, 100 nF bootstrap. https://www.mouser.com/datasheet/2/115/AP63200_AP63201_AP63203_AP63205-1525010.pdf
- AOD4144 datasheet (Alpha & Omega): VGS(th) 1.2 / 1.8 / 2.4 V, RDS(on) ≤ 8 mΩ at 10 V, ≤ 14 mΩ at 4.5 V. https://70ic.com/587270/AOD4144003.pdf
- NodeMCU board variants (VIN to USB 5 V, with or without a diode): https://homeding.github.io/boards/nodemcu.htm
- Rev A netlist: `../pcb_voltage_logging/pcb_voltage_logging.kicad_sch` and `.kicad_pcb` (2026-11-13)
