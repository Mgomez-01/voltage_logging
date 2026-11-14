# Wiring Guide: PCB (`pcb_voltage_logging`)

This branch runs on the `pcb_voltage_logging` board. The ADC, SD socket, status LED and heater switch are all on the PCB, so wiring is down to the connectors below. The breadboard build (CD74HC4067 mux into A0) is documented on the `main` branch. Fabrication and hand-assembly notes are in [PCB_IMPLEMENTATION.md](PCB_IMPLEMENTATION.md); fixes for the next revision are in [PCB_REVISIONS.md](PCB_REVISIONS.md).

## Connectors

| Ref | Type | Connect | Notes |
|---|---|---|---|
| U1 | 2 × 15 female header rows (0.9″ apart) | NodeMCU v2 | Check the orientation against the silkscreen before powering |
| USB on U1 | micro-USB | PC or 5 V USB supply | **Powers all the logic** (ESP8266, MCP3208, SD, LED). Required |
| J1 | 2-pin screw terminal | 12 V supply (+ / GND) | Feeds the heater only, through F1 (1.5 A hold PTC) |
| J2 | 2-pin screw terminal | Heater element | Pin 1 = +12 V, pin 2 = MOSFET drain (switched low side) |
| J4 | 2-pin JST PH | PID Temp Sensor, 100k NTC | Pin 1 = MCP3208 CH1 node, pin 2 = GND |
| J5 | 2-pin JST PH | AUX Temp Sensor, MF55 10k NTC | Pin 1 = MCP3208 CH2 node, pin 2 = GND |
| J6 | microSD (bottom side) | FAT32 card | Push-push socket, no card-detect wired |
| J7 | 1 × 7 pin header | Spare ADC inputs | Pins 1–5 = CH7…CH3 (reversed), pin 6 = +3V3, pin 7 = GND. Not used by the firmware yet |

Thermistors are polarity-free; either way round in J4/J5 is fine.

## NodeMCU pin map

```
Label | GPIO   | PCB net      | Connected to                            | Function
------|--------|--------------|-----------------------------------------|---------------------------
D0    | GPIO16 | HEATER_PWM   | R4 100 Ω -> Q1 gate (R3 10k to GND)     | Heater PWM, 32 Hz
D1    | GPIO5  | ADC_CS       | MCP3208 /CS (pin 10), R12 10k pull-up   | ADC chip select
D2    | GPIO4  | -            | not connected                           | free
D3    | GPIO0  | -            | not connected (boot pin)                | free, leave floating
D4    | GPIO2  | -            | not connected (boot pin, blue LED)      | free, leave floating
D5    | GPIO14 | SPI_SCK      | R9 33 Ω -> MCP3208 CLK + SD CLK         | SPI clock
D6    | GPIO12 | SD_MISO      | MCP3208 DOUT + SD DO, R5 10k pull-up    | SPI data in
D7    | GPIO13 | SPI_MOSI     | R10 33 Ω -> MCP3208 DIN + SD CMD        | SPI data out
D8    | GPIO15 | (unnamed)    | R11 47 Ω -> SD DAT3/CS                  | SD chip select (boot pin, must be LOW at reset)
RX    | GPIO3  | LED_DATA     | U3 74AHCT1G125 -> R8 330 Ω -> D2 DIN    | Status LED data
TX    | GPIO1  | -            | not connected on the PCB                | USB serial output
A0    | ADC    | -            | not connected                           | not used
3V3   | -      | +3V3         | MCP3208 VDD/VREF, SD, NTC dividers, J7  |
VIN   | -      | +5V_VIN      | WS2812B, U3                             | 5 V from USB (through the NodeMCU's diode)
GND   | -      | GND          | everything, incl. 12 V return           |
```

## Circuits

```
   PID Temp Sensor (CH1)        AUX Temp Sensor (CH2)        MCP3208 (U2)
   +3V3                         +3V3                         VDD (16), VREF (15) = +3V3, C7 1 µF
     │                            │                          AGND (14), DGND (9) = GND
   [R1 100k]                    [R2 2k]                      CLK (13) <- SCK, DOUT (12) -> MISO
     │                            │                          DIN (11) <- MOSI, /CS (10) <- D1
     ├── CH1 (pin 2)              ├── CH2 (pin 3)
     │       └─[C8 100n]─ GND     │       └─[C9 100n]─ GND
   [NTC 100k, J4]               [NTC 10k, J5]
     │                            │
    GND                          GND

   Heater (low-side switch)                        Status LED
   J1 +12V ─[F1 1.5 A PTC]─┬─ J2 pin 1 (+)         RX/GPIO3 ─ U3 (74AHCT1G125, 5 V, OE tied low)
                           │   [Heater]                     ─ R8 330 Ω ─ WS2812B DIN (D2, 5 V)
                       D1 SS34 (flyback)
                           │
                           └─ J2 pin 2 ── Q1 drain (AOD4144)
   D0 ─[R4 100 Ω]─┬─ Q1 gate                Q1 source ── GND
                  └─[R3 10k]─ GND
```

## Component notes

### MCP3208 (U2)
- 12-bit, 8 channels, SPI. The firmware runs it at 1 MHz (its limit at 2.7–3.3 V) and gives it its own SPI transaction, so it shares the bus with the SD card safely
- Reference = supply = +3V3, the same rail as the NTC dividers, so the reading is ratiometric and the rail voltage cancels out
- Input leakage is ~nA: no loading correction is needed (the old A0 input loaded the PID divider with 320 kΩ)
- C8/C9 hold the nodes steady during the sample, so no settling delay is needed between channels

### PID Temp Sensor: 100k NTC (J4, CH1)
- 100 kΩ at 25 °C, β = 3950 K (Ender 3 type), with R1 = 100 kΩ to +3V3
- ~20 codes per °C at 65 °C
- Used by the PID (by default) and by the over-temperature / sensor-fault safety checks
- Firmware constants (`src/sensor_manager.cpp`): `THERMISTOR_NOMINAL`, `B_COEFFICIENT`, `SERIES_RESISTOR`. Set `SERIES_RESISTOR` to R1's measured value and recalibrate (README.md, "Calibration")

### AUX Temp Sensor: MF55 ultra-thin 10k NTC (J5, CH2)
- 10 kΩ at 25 °C, β = 3950 K (B25/50), ±1 %; rated -30 to +125 °C; dissipation constant 0.7 mW/°C
- R2 = 2 kΩ to +3V3, chosen for resolution around 60–70 °C: ~37 codes per °C at 65 °C
- Self-heating: ~1.4 mW at 65 °C, up to ~1.9 °C high in still air, less when pressed against a surface
- Logged and displayed; selectable as the PID's control sensor. It does not trigger the safety shutdown, but a fault on it stops the PID while it is the control sensor

### microSD (J6)
- `SD.begin(SS)`, CS on D8 through R11. FAT32 (or FAT16 for ≤ 2 GB). Log files `data*.log` go in the card root
- No card-detect: if the card wasn't in at boot, insert it and press Start Logging (it retries), or reboot

### Status LED (D2)
- WS2812B on the 5 V VIN rail, data from GPIO3 (RX) through U3. The firmware uses NeoPixelBus's DMA method, which only works on GPIO3, and sets the serial port to TX only
- It may flicker random colours while the board is being flashed over USB (the USB-serial chip drives RX). Harmless

### AOD4144 MOSFET + heater
- 30 V N-channel, Vgs(th) 1.2–2.4 V, Rds(on) rated at 10 V and 4.5 V. Driven at 3.3 V from GPIO16 through R4; fine at the ≤ 1.5 A the PTC allows (see `md_info_docs/MOSFET_GATE_DRIVE_NOTES.md` for larger heaters)
- R3 keeps the gate low while the ESP8266 boots or resets
- F1 (1.5 A hold) limits the heater to about 18 W at 12 V. A larger heater trips it

## Bring-up

1. **Before fitting the NodeMCU:** check J1 polarity (there is no reverse-polarity protection on this revision) and that +3V3, +5V_VIN and GND aren't shorted to each other.
2. **Fit the NodeMCU and plug in USB only.** Measure:
   - +3V3 at U2 pins 15/16 (~3.3 V)
   - C8 top pad: ~1.64 V at 25 °C with the PID NTC in J4, ~3.3 V with J4 empty
   - C9 top pad: ~2.79 V at 25 °C with the AUX NTC in J5, ~3.3 V with J5 empty
   - 0 V at the Q1 gate
3. **Flash** (`make deploy`) and watch the boot output (115200 baud):
   ```
   Initializing heater PWM control... OK
   Status LED: WS2812B on GPIO3 (RX), NeoPixelBus DMA
   ===============================================
   ESP8266 Temperature Logger (PID + AUX Temp Sensors)
   PCB version - MCP3208 12-bit ADC
   ===============================================
   Setting up MCP3208 ADC... OK
   MCP3208 on SPI (D5 SCK, D6 MISO, D7 MOSI), CS = D1/GPIO5, 1 MHz
   CH1: PID Temp Sensor (100k NTC, J4)
   CH2: AUX Temp Sensor (MF55 10k NTC, J5, selectable as the PID sensor)
   Testing sensor channels (MCP3208, 0-4095):
   CH1 (PID Temp Sensor): 2026 -> 25.00°C
   CH2 (AUX Temp Sensor): 3447 -> 25.00°C
   ...
   Initializing data file... SD card initialized.
   ```
   (2026 and 3447 are the 25 °C codes with the current constants.) An empty J4/J5 shows ~4095 and "open"; a shorted one ~0.
4. **SD + ADC together:** start logging and check that rows appear and the temperatures stay right after the buffer writes (the bus is shared).
5. **Heater:** connect 12 V and the heater, turn on manual power and check the LED goes blue → red with the power. Turn it off: the LED goes off.
6. **Calibrate** (README.md, "Calibration"), then re-run a `/tune` step test.

## Troubleshooting

### Board won't boot (or only boots without the SD card)
- D8 (GPIO15) must be LOW at reset. The NodeMCU's own pull-down does this; a card holding DAT3 high can fight it. Try another card, or see PCB_REVISIONS.md.
- D3/D4 are left floating on this board, which is fine (the NodeMCU pulls them up).

### Both temperatures -999 / N/A
- No 3.3 V at U2, D1 (CS) not reaching U2 pin 10, or the MCP3208 fitted the wrong way round. The boot message shows the raw codes.

### Temperatures wrong only after SD writes
- The card is holding the shared MISO line. Try another card.

### LED never lights / random colours
- The LED needs USB 5 V on VIN. Random colours appear during flashing and stop when the firmware starts.

### Heater never turns on / stays on
- Gate should be 0 V off and ~3.3 V at 100 %. If the heater is on with the gate at 0 V, Q1 is damaged (check D1 is fitted the right way: cathode to +12 V).
- If the heater cuts out after a while, F1 may be tripping: the heater draws more than ~1.5 A.

## Expected performance

- Sampling: one cycle (CH1 + CH2) every 6 ms, up to ~167 rows/s; the PID averages all cycles between its 500 ms updates
- Resolution at 65 °C: ~20 codes/°C (PID sensor), ~37 codes/°C (AUX)
- Accuracy: not yet calibrated on the PCB (expect ~±2 °C until it is)
- Logging: 50-row buffer written to the SD card, ~110 bytes per row (20 columns): ~66 MB/h at the full rate, ~2 MB/h with averaging on (32 cycles per row)
