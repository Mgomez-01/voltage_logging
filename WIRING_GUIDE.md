# Wiring Guide: Multiplexer, SD Card and PWM Heater

## Pin Mapping

### ESP8266 (NodeMCU / Wemos D1 mini)
```
Label | GPIO   | Connected to          | Function
------|--------|-----------------------|---------------------------------
A0    | ADC    | CD74HC4067 COM        | Analog input (both sensors)
D0    | GPIO16 | AOD4144 gate          | Heater PWM
D1    | GPIO5  | CD74HC4067 S0         | Mux select bit 0
D2    | GPIO4  | CD74HC4067 S1         | Mux select bit 1
D3    | GPIO0  | CD74HC4067 S2         | Mux select bit 2 (boot pin)
D4    | GPIO2  | CD74HC4067 S3         | Mux select bit 3 (boot pin, LED)
D5    | GPIO14 | SD module SCK         | SPI clock
D6    | GPIO12 | SD module MISO        | SPI data in
D7    | GPIO13 | SD module MOSI        | SPI data out
D8    | GPIO15 | SD module CS          | SPI chip select (boot pin)
3V3   | -      | Mux VCC + EN, SD VCC, thermistor divider top
GND   | -      | Mux GND, SD GND, thermistor bottom, MOSFET source, 12 V supply GND
```

All usable GPIOs are in use. Only RX/TX remain free.

### CD74HC4067 Channels
```
Channel | Connection
--------|---------------------------
C0      | Voltage source
C1      | Thermistor voltage divider
C2-C15  | Free for more analog sensors
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

   CD74HC4067                         microSD module
   ┌──────────────┐                   ┌──────────────┐
   │ VCC, EN ── 3.3 V                 │ VCC ── 3.3 V │
   │ GND ────── GND                   │ GND ── GND   │
   │ COM ────── A0                    │ SCK ── D5    │
   │ S0-S3 ──── D1-D4                 │ MISO ─ D6    │
   │ C0 ─────── voltage source        │ MOSI ─ D7    │
   │ C1 ─────── thermistor divider    │ CS ─── D8    │
   └──────────────┘                   └──────────────┘

   Thermistor divider (C1)            Heater (low-side MOSFET switch)
   3.3 V                              +12 V
     │                                  │
   [100 kΩ]                          [Heater]
     │                                  │
     ├──── C1                           ├──── Drain
     │                                  │
   [NTC 100k]               D0 ───┬──── Gate   AOD4144
     │                            │     │
    GND                        [10 kΩ]  Source
                                  │     │
                                 GND   GND (shared with ESP8266 GND)
```

## Component Details

### CD74HC4067 Multiplexer
- 16 channels selected by 4 bits (S0-S3)
- Runs at 3.3 V (2-6 V supply), ~70 Ω on-resistance
- The firmware waits 10 µs after switching plus 50 µs settling before each read

### 100k NTC Thermistor
- 100 kΩ at 25 °C, β = 3950 K (Ender 3 type)
- 100 kΩ series resistor to 3.3 V
- Firmware constants (`src/sensor_manager.cpp`): `THERMISTOR_NOMINAL`, `TEMPERATURE_NOMINAL`, `B_COEFFICIENT`, `SERIES_RESISTOR`

### microSD Module
- SPI at 3.3 V, chip select on D8 (GPIO15), `SD.begin(SS)`
- Card formatted FAT32 (or FAT16 for ≤ 2 GB)
- A Wemos D1 mini micro-SD shield uses the same pins and plugs straight on
- Log files `data*.log` are written to the card root

### AOD4144 MOSFET + Heater
- N-channel logic-level MOSFET, Vgs(th) 2-3 V
- Gate driven directly from GPIO16 (3.3 V PWM), with a 10 kΩ pull-down so the heater stays off while the ESP8266 boots or resets
- 3.3 V gate drive turns it on but not fully (Rds(on) is rated at 4.5 V). For high heater currents, add a gate driver such as a TC4420 powered from 5-12 V. See `MOSFET_GATE_DRIVE_NOTES.md`.
- Heater supply: 12 V. The supply ground must be tied to the ESP8266 ground.
- Fuse the 12 V heater supply.

## Assembly Steps

1. **Power rails**: 3.3 V and GND from the ESP8266 to the breadboard rails.
2. **Multiplexer**: VCC and EN to 3.3 V, GND to ground, S0-S3 to D1-D4, COM to A0.
3. **Sensors**: voltage source to C0. Thermistor divider to C1 (3.3 V → 100 kΩ → C1 → NTC → GND).
4. **SD module**: VCC 3.3 V, GND, SCK→D5, MISO→D6, MOSI→D7, CS→D8. Insert a formatted card.
5. **Heater**: D0 → MOSFET gate, 10 kΩ gate→GND, source→GND, drain→heater(−), heater(+)→12 V, supply GND→common GND.

## Verification

With a multimeter:
- 3.3 V between mux VCC/EN and GND, and at the SD module VCC
- ~1.65 V at C1 at room temperature
- Your source voltage at C0
- 0 V at the MOSFET gate with the heater off

Expected serial output at boot (115200 baud):
```
Setting up CD74HC4067 multiplexer... OK
Multiplexer pins: S0=D1, S1=D2, S2=D3, S3=D4
Channel 0: Voltage sensor
Channel 1: Thermistor (100k NTC)
Initializing heater PWM control... OK
PWM pin: GPIO16 (D0), Frequency: 32 Hz, Range: 0-1000
PWM control via MOSFET (AOD4144) for smooth heater power regulation
A0 pin number: 17
Testing sensor channels:
Channel 0 (Voltage): 842 -> 0.8223V
Channel 1 (Thermistor): 512 -> 25.00°C
...
Initializing data file... SD card initialized.
Created new data file: data_3.log
OK
```

(ADC 512 is the divider midpoint, 1.65 V, which is exactly 25 °C for a 100k/100k divider. The PWM runs at 32 Hz with ESP8266 core 3.1.2. You should see a ~31 ms period on a scope.)

## Troubleshooting

### Board won't boot with everything attached
- D3 (GPIO0) and D4 (GPIO2) must be HIGH and D8 (GPIO15) LOW at power-up. Check that the mux and SD module aren't pulling them the wrong way.

### "SD card initialization failed"
- Check CS on D8 and the SPI lines, use a 3.3 V-compatible module, and reformat the card FAT32.
- With no card at boot, no log file is created. Insert the card and press Start Logging (it retries the SD setup), or reboot.

### Temperature shows N/A / -999
- Check the divider on C1 and the 100 kΩ resistor. The thermistor should measure ~100 kΩ at room temperature.

### Both channels read the same value
- Check the S0-S3 wiring and that EN is tied active.

### Heater never turns on / stays on
- Gate should be 0 V off and 3.3 V at 100%. Check the pull-down, source to GND, and the common ground with the 12 V supply.

## Expected Performance

- Sampling: one ADC read every 2 ms, alternating channels, so up to 250 readings/s per channel
- Voltage resolution: 10-bit (~1 mV per step on the 0-1 V scale)
- Temperature: ±1 °C after calibration
- Logging: 50-row buffer written to the SD card. A new 50 MB file about every 1.5 h at the full rate.
