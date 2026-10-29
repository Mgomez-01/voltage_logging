# Wiring Guide: Multiplexer, Temperature Sensors, SD Card and PWM Heater

## Pin Mapping

### ESP8266 (NodeMCU / Wemos D1 mini)
```
Label | GPIO   | Connected to          | Function
------|--------|-----------------------|---------------------------------
A0    | ADC    | CD74HC4067 COM        | Analog input (all mux channels)
D0    | GPIO16 | AOD4144 gate          | Heater PWM
D1    | GPIO5  | CD74HC4067 S0         | Mux select bit 0
D2    | GPIO4  | CD74HC4067 S1         | Mux select bit 1
D3    | GPIO0  | CD74HC4067 S2         | Mux select bit 2 (boot pin)
D4    | GPIO2  | CD74HC4067 S3         | Mux select bit 3 (boot pin, LED)
D5    | GPIO14 | SD module SCK         | SPI clock
D6    | GPIO12 | SD module MISO        | SPI data in
D7    | GPIO13 | SD module MOSI        | SPI data out
D8    | GPIO15 | SD module CS          | SPI chip select (boot pin)
3V3   | -      | Mux VCC + EN, SD VCC, top of both thermistor dividers
GND   | -      | Mux GND, SD GND, both thermistors, MOSFET source, 12 V supply GND
```

All usable GPIOs are in use. Only RX/TX remain free.

### CD74HC4067 Channels
```
Channel | Connection
--------|---------------------------------------------------------------
C0      | Voltage source
C1      | PID Temp Sensor divider (100k NTC) - controls the heater
C2      | AUX Temp Sensor divider (MF55 10k NTC) - logged only, not used by the PID
C3-C15  | Free for more analog sensors
```

## Complete Wiring Diagram

```
                         ESP8266 (NodeMCU / D1 mini)
                        ┌───────────────────────────┐
     Mux COM  ──────────┤ A0                    3V3 ├────── 3.3 V rail
     Mux S0   ──────────┤ D1                    GND ├────── GND rail
     Mux S1   ──────────┤ D2                        │
     Mux S2   ──────────┤ D3                     D5 ├────── SD SCK
     Mux S3   ──────────┤ D4                     D6 ├────── SD MISO
     MOSFET gate ───────┤ D0                     D7 ├────── SD MOSI
                        │                        D8 ├────── SD CS
                        └───────────────────────────┘

   CD74HC4067                             microSD module
   ┌──────────────────────────────┐       ┌──────────────┐
   │ VCC, EN ── 3.3 V             │       │ VCC ── 3.3 V │
   │ GND ────── GND               │       │ GND ── GND   │
   │ COM ────── A0                │       │ SCK ── D5    │
   │ S0-S3 ──── D1-D4             │       │ MISO ─ D6    │
   │ C0 ─────── voltage source    │       │ MOSI ─ D7    │
   │ C1 ─────── PID Temp divider  │       │ CS ─── D8    │
   │ C2 ─────── AUX Temp divider  │       └──────────────┘
   └──────────────────────────────┘

   PID Temp Sensor (C1)     AUX Temp Sensor (C2)     Heater (low-side MOSFET switch)
   3.3 V                    3.3 V                    +12 V
     │                        │                        │
   [100 kΩ]                 [2.0 kΩ]                [Heater]
     │                        │                        │
     ├──── C1                 ├──── C2                 ├──── Drain
     │                        │                        │
   [NTC 100k]               [MF55 NTC 10k]  D0 ───┬──── Gate   AOD4144
     │                        │                   │    │
    GND                      GND               [10 kΩ] Source
                                                  │    │
                                                 GND  GND (shared with ESP8266 GND)
```

## Component Details

### CD74HC4067 Multiplexer
- 16 channels selected by 4 bits (S0-S3)
- Runs at 3.3 V (2-6 V supply), ~70 Ω on-resistance
- The firmware waits 10 µs after switching plus 50 µs settling before each read

### PID Temp Sensor: 100k NTC Thermistor (C1)
- 100 kΩ at 25 °C, β = 3950 K (Ender 3 type)
- 100 kΩ series resistor to 3.3 V (measures 98 kΩ on this board; `SERIES_RESISTOR` = 98000)
- Used by the PID and by the over-temperature / sensor-fault safety checks
- Firmware constants (`src/sensor_manager.cpp`): `THERMISTOR_NOMINAL`, `TEMPERATURE_NOMINAL`, `B_COEFFICIENT`, `SERIES_RESISTOR`. Calibrated: R25 = 97 886 Ω, B = 3910 (see README.md, "Calibration")
- High divider impedance (~50 kΩ), so the board's 320 kΩ A0 load pulls the sampled voltage ~14 % low. The firmware models this; see "A0 load and full scale" below.

### AUX Temp Sensor: MF55 Ultra-Thin 10k NTC Thermistor (C2)
- 10 kΩ at 25 °C, β = 3950 K (B25/50), ±1 %; rated -30 to +125 °C; dissipation constant 0.7 mW/°C
- 2.0 kΩ series resistor to 3.3 V, chosen for resolution around 60-70 °C (the NTC is ~2.5-1.8 kΩ there): ~9 ADC counts per °C. Anything from 2.0 kΩ to 2.7 kΩ gives about the same resolution at 60-65 °C.
- Self-heating: with 2.0 kΩ the NTC dissipates ~1.4 mW at 65 °C, which can read up to ~1.9 °C high in still air, and less when pressed flat against a surface. 2.7 kΩ cuts this by about a quarter.
- Second sensor at a different location, for redundancy and sanity checks. It is logged and displayed but **not** used by the PID or the safety checks.
- Firmware constants (`src/sensor_manager.cpp`): `AUX_THERMISTOR_NOMINAL`, `AUX_TEMPERATURE_NOMINAL`, `AUX_B_COEFFICIENT`, `AUX_SERIES_RESISTOR`. `AUX_SERIES_RESISTOR` must match the resistor fitted (measured 2.00 kΩ). Calibrated: R25 = 10 631 Ω, B = 4096. For a different part (e.g. a 103F3435), start from its datasheet values and recalibrate.
- Near 0 °C this divider sits close to the ADC ceiling (~997 of 1024 counts) with only ~3 counts per °C, so small ADC gain errors show up as several degrees there. That makes an ice bath a sensitive calibration point (it read 1002.5 counts in the calibration ice bath). It reads -999 below about -11 °C, where the ADC clips.

### A0 load and full scale (NodeMCU / D1 mini)
The board routes A0 through a 220k/100k divider to the ESP8266's 1.0 V ADC:
- A0 full scale is **3.2 V** (`ADC_FULL_SCALE_VOLTAGE`)
- A0 presents a **320 kΩ** load to whichever divider the mux selects (`ADC_INPUT_RESISTANCE`)
- The thermistor rail measures **3.32 V** (`THERMISTOR_SUPPLY_VOLTAGE`)

The conversion removes the load, so the NTC resistance it calculates is the real one. If you add a filter capacitor on C1 or C2, the node no longer sees the full load at the moment of the sample: set `ADC_INPUT_RESISTANCE` to ~960000 and recalibrate. `ADC_CALIBRATION_GAIN` stays at 1.0. Calibration is done per sensor through R25 and B (README.md, "Calibration"), and changing the gain would invalidate it.

Open-sensor detection: a disconnected PID thermistor leaves the 98 kΩ / 320 kΩ divider, ~2.54 V (ADC ~813), not the rail. The firmware treats an NTC resistance above 20 × R25 (colder than about -30 °C) as open and returns -999, which trips the sensor-fault shutdown when the heater is on.

### microSD Module
- SPI at 3.3 V, chip select on D8 (GPIO15), `SD.begin(SS)`
- Card formatted FAT32 (or FAT16 for ≤ 2 GB)
- A Wemos D1 mini micro-SD shield uses the same pins and plugs straight on
- Log files `data*.log` are written to the card root

### AOD4144 MOSFET + Heater
- N-channel logic-level MOSFET, Vgs(th) 2-3 V
- Gate driven directly from GPIO16 (3.3 V PWM), with a 10 kΩ pull-down so the heater stays off while the ESP8266 boots or resets
- 3.3 V gate drive turns it on but not fully (Rds(on) is rated at 4.5 V). For high heater currents, add a gate driver such as a TC4420 powered from 5-12 V. See `md_info_docs/MOSFET_GATE_DRIVE_NOTES.md`.
- Heater supply: 12 V. The supply ground must be tied to the ESP8266 ground.
- Fuse the 12 V heater supply.

## Assembly Steps

1. **Power rails**: 3.3 V and GND from the ESP8266 to the breadboard rails.
2. **Multiplexer**: VCC and EN to 3.3 V, GND to ground, S0-S3 to D1-D4, COM to A0.
3. **Sensors**:
   - Voltage source to C0
   - PID Temp Sensor divider to C1: 3.3 V → 100 kΩ (measured 98 kΩ) → C1 → 100k NTC → GND
   - AUX Temp Sensor divider to C2: 3.3 V → 2.0 kΩ → C2 → MF55 10k NTC → GND
4. **SD module**: VCC 3.3 V, GND, SCK→D5, MISO→D6, MOSI→D7, CS→D8. Insert a formatted card.
5. **Heater**: D0 → MOSFET gate, 10 kΩ gate→GND, source→GND, drain→heater(−), heater(+)→12 V, supply GND→common GND.

## Verification

With a multimeter:
- 3.3 V between mux VCC/EN and GND, and at the SD module VCC (this board measures 3.32 V; `THERMISTOR_SUPPLY_VOLTAGE` should match)
- PID Temp Sensor, C1 at 25 °C: ~1.68 V with nothing sampling it, ~1.45 V while the mux connects it to A0. With the ESP8266 running, a meter shows something in between.
- AUX Temp Sensor, C2 at 25 °C: ~2.75 V (the A0 load barely affects it); ~1.7 V at 65 °C
- Your source voltage at C0
- 0 V at the MOSFET gate with the heater off

Expected serial output at boot (115200 baud):
```
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: PID Temp Sensor (100k NTC)
Channel 2: AUX Temp Sensor (MF55 10k NTC, not used by PID)
Initializing heater PWM control... OK
PWM pin: GPIO16 (D0), Frequency: 32 Hz, Range: 0-1000
PWM control via MOSFET (AOD4144) for smooth heater power regulation
A0 pin number: 17
Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V
Channel 1 (PID Temp Sensor): 442 -> 25.03°C
Channel 2 (AUX Temp Sensor): 864 -> 24.95°C
...
Initializing data file... SD card initialized.
Created new data file: data_3.log
OK
```

(ADC 442 and 864 are the nearest counts to 25 °C for this board with the calibrated thermistor constants and hotplate correction: 98 kΩ and 2.0 kΩ dividers on a 3.32 V rail, with the 320 kΩ A0 load and 3.2 V full scale. The PWM runs at 32 Hz with ESP8266 core 3.1.2. You should see a ~31 ms period on a scope.)

## Troubleshooting

### Board won't boot with everything attached
- D3 (GPIO0) and D4 (GPIO2) must be HIGH and D8 (GPIO15) LOW at power-up. Check that the mux and SD module aren't pulling them the wrong way.

### "SD card initialization failed"
- Check CS on D8 and the SPI lines, use a 3.3 V-compatible module, and reformat the card FAT32.
- With no card at boot, no log file is created. Insert the card and press Start Logging (it retries the SD setup), or reboot.

### PID Temp Sensor shows N/A / -999
- Check the divider on C1 and the series resistor. The thermistor should measure ~100 kΩ near 25 °C (e.g. ~104.5 kΩ at 24 °C). A disconnected PID thermistor reads -999 and trips the sensor-fault shutdown if the heater is on.

### AUX Temp Sensor shows N/A / -999
- Check the divider on C2 and the 2.0 kΩ resistor. The MF55 should measure ~10 kΩ at room temperature.
- With the NTC unplugged, C2 is pulled up to 3.3 V and reads N/A. The heater and PID are unaffected.

### PID and AUX Temp Sensors disagree at room temperature
- Check that `AUX_SERIES_RESISTOR` and `AUX_B_COEFFICIENT` match the parts fitted, then compare both against a reference thermometer.

### Channels read the same value
- Check the S0-S3 wiring and that EN is tied active.

### Heater never turns on / stays on
- Gate should be 0 V off and 3.3 V at 100%. Check the pull-down, source to GND, and the common ground with the 12 V supply.

## Expected Performance

- Sampling: one ADC read every 2 ms, cycling channels 0 → 1 → 2, so up to ~167 readings/s per channel
- Voltage resolution: 10-bit (~1 mV per step on the 0-1 V scale)
- PID Temp Sensor resolution: ~4-6 ADC counts per °C at 60-70 °C (100k NTC with 100 kΩ)
- AUX Temp Sensor resolution: ~9 ADC counts per °C at 60-70 °C (10k NTC with 2.0 kΩ)
- Temperature accuracy: about ±1.5 °C absolute, a few tenths of a °C between the two sensors (ice/boiling calibration + hotplate correction; see README.md, "Calibration")
- Logging: 50-row buffer written to the SD card, ~93 bytes per row (17 columns). A new 50 MB file about every 54 min at the full rate, or ~2 MB/hour with averaging on (32 cycles per row).
