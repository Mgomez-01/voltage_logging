#!/usr/bin/env python3
"""
Thermistor calibration for the PID Temp Sensor (C1) and AUX Temp Sensor (C2).

Give it the raw ADC counts each sensor read at one or more known temperatures
(ice bath, boiling water, ...). It converts the counts to NTC resistance with
the same circuit model as the firmware, fits each thermistor's Beta-equation
constants (R25 and B), and prints the values to put in src/sensor_manager.cpp.

The circuit constants (rail voltage, series resistors, A0 load, full scale,
calibration gain) are read from src/sensor_manager.cpp, so the script always
matches the firmware.

Usage
-----
  # Average the raw counts in a log file (or a pasted serial capture):
  python3 tools/calibrate_thermistors.py --average data_4.log
  # Only use the settled part of a test (find it with --timeline first):
  python3 tools/calibrate_thermistors.py --average data_4.log --timeline
  python3 tools/calibrate_thermistors.py --average data_4.log --from 120 --to 300

  # Fit from known points: TEMP_C:PID_ADC:AUX_ADC (use '-' to skip a sensor)
  python3 tools/calibrate_thermistors.py --point 0:658.2:1001.5 --point 95.7:77.7:293.1

With 1 point:   fits B only (R25 stays at the datasheet value). Rough.
With 2 points:  fits R25 and B exactly.             (ice + boiling recommended)
With 3+ points: least-squares fit of R25 and B, plus the residual at each point.

Boiling point depends on air pressure: about 100 C at sea level, ~95.7 C at
1300 m (Salt Lake City), ~94.7 C at 1609 m (Denver). Use your elevation or local station pressure (see --help).
"""
import argparse
import math
import os
import re
import statistics
import sys

T25_K = 298.15
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SENSOR_CPP = os.path.join(REPO, "src", "sensor_manager.cpp")

CONSTANTS = [
    "THERMISTOR_SUPPLY_VOLTAGE", "ADC_FULL_SCALE_VOLTAGE", "ADC_INPUT_RESISTANCE",
    "ADC_CALIBRATION_GAIN", "THERMISTOR_NOMINAL", "B_COEFFICIENT", "SERIES_RESISTOR",
    "AUX_THERMISTOR_NOMINAL", "AUX_B_COEFFICIENT", "AUX_SERIES_RESISTOR",
]


def read_firmware_constants(path):
    text = open(path).read()
    values = {}
    for name in CONSTANTS:
        m = re.search(r"const\s+float\s+" + name + r"\s*=\s*([0-9.eE+-]+)\s*;", text)
        if not m:
            sys.exit(f"Could not find {name} in {path}")
        values[name] = float(m.group(1))
    return values


def adc_to_resistance(adc, series_r, c):
    """Same math as ntcAdcToCelsius() in the firmware, up to the NTC resistance."""
    vs, rl = c["THERMISTOR_SUPPLY_VOLTAGE"], c["ADC_INPUT_RESISTANCE"]
    if adc <= 1 or adc >= 1023:
        raise ValueError(f"ADC {adc} is shorted/saturated; can't use it")
    v = adc / 1024.0 * c["ADC_FULL_SCALE_VOLTAGE"] * c["ADC_CALIBRATION_GAIN"]
    parallel = series_r * v / (vs - v)
    if parallel >= rl:
        raise ValueError(f"ADC {adc} means an open sensor")
    return parallel * rl / (rl - parallel)


def beta_temp_c(r, r25, b):
    return 1.0 / (math.log(r / r25) / b + 1.0 / T25_K) - 273.15


def resistance_at(t_c, r25, b):
    return r25 * math.exp(b * (1.0 / (t_c + 273.15) - 1.0 / T25_K))


def fit(points, r25_nominal):
    """points: [(temp_c, resistance)]. ln R = ln R25 + B (1/T - 1/T25)."""
    xs = [1.0 / (t + 273.15) - 1.0 / T25_K for t, _ in points]
    ys = [math.log(r) for _, r in points]
    if len(points) == 1:
        b = (ys[0] - math.log(r25_nominal)) / xs[0]
        return r25_nominal, b
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        sys.exit("Calibration points need different temperatures")
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return math.exp(my - b * mx), b


def average_log(path, t_from=None, t_to=None, timeline=False):
    """Average thermistor_adc / aux_adc from a CSV log, or the 'PID Temp Sensor (Ch1): N'
    lines of a serial capture. t_from/t_to (seconds since boot) limit a CSV log to the
    settled part of a test; timeline prints 10 s averages to find it. Averaged rows
    (samples > 1) hold mean counts and are weighted by their sample count."""
    pid, aux = [], []
    pid_w, aux_w = [], []
    bins = {}
    header = None
    for line in open(path, errors="replace"):
        line = line.strip()
        if line.startswith("timestamp,"):
            header = line.split(",")
            continue
        if header and line and line[0].isdigit():
            cols = line.split(",")
            if len(cols) == len(header):
                row = dict(zip(header, cols))
                ts = int(row["timestamp"]) / 1000.0
                if timeline and "thermistor_adc" in row and "aux_adc" in row:
                    bins.setdefault(int(ts // 10), []).append((float(row["thermistor_adc"]), float(row["aux_adc"])))
                if (t_from is not None and ts < t_from) or (t_to is not None and ts > t_to):
                    continue
                w = int(row.get("samples", "1") or 1)
                if "thermistor_adc" in row:
                    pid.append(float(row["thermistor_adc"])); pid_w.append(w)
                if "aux_adc" in row:
                    aux.append(float(row["aux_adc"])); aux_w.append(w)
            continue
        m = re.search(r"PID Temp Sensor \(Ch1\): (\d+)", line)
        if m:
            pid.append(int(m.group(1))); pid_w.append(1)
        m = re.search(r"AUX Temp Sensor \(Ch2\): (\d+)", line)
        if m:
            aux.append(int(m.group(1))); aux_w.append(1)
    if timeline and bins:
        print("10 s averages (pick a flat stretch, then rerun with --from/--to):")
        print(f"  {'seconds':>11} {'rows':>5} {'PID mean':>9} {'min-max':>9} {'AUX mean':>9} {'min-max':>9}")
        for k in sorted(bins):
            p = [x[0] for x in bins[k]]; a = [x[1] for x in bins[k]]
            print(f"  {k*10:5d}-{k*10+10:<5d} {len(p):5d} {statistics.mean(p):9.2f} {min(p):4g}-{max(p):<4g} "
                  f"{statistics.mean(a):9.2f} {min(a):4g}-{max(a):<4g}")
        print()
    if t_from is not None or t_to is not None:
        print(f"Window: {t_from if t_from is not None else 'start'} s to {t_to if t_to is not None else 'end'} s")
    def wmean(vals, wts):
        return sum(v * w for v, w in zip(vals, wts)) / sum(wts)
    for name, vals, wts in (("PID (thermistor_adc)", pid, pid_w), ("AUX (aux_adc)", aux, aux_w)):
        if vals:
            sd = statistics.pstdev(vals)
            print(f"{name:21s} rows={len(vals):6d}  cycles={sum(wts):7d}  mean={wmean(vals, wts):8.2f}  "
                  f"sd={sd:5.2f}  min={min(vals):g}  max={max(vals):g}")
        else:
            print(f"{name:21s} no readings found")
    if pid or aux:
        p = f"{wmean(pid, pid_w):.2f}" if pid else "-"
        a = f"{wmean(aux, aux_w):.2f}" if aux else "-"
        print(f"\nFor a known temperature T, use:  --point T:{p}:{a}")


def boiling_point_c(pressure_kpa):
    p_mmhg = pressure_kpa * 7.50062
    return 1730.63 / (8.07131 - math.log10(p_mmhg)) - 233.426


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--point", action="append", default=[],
                    help="TEMP_C:PID_ADC:AUX_ADC (use '-' to skip a sensor). Repeat for more points.")
    ap.add_argument("--average", metavar="FILE", help="Average the raw counts in a log file or serial capture")
    ap.add_argument("--timeline", action="store_true",
                    help="With --average on a CSV log: print 10 s averages to find the settled stretch")
    ap.add_argument("--from", dest="t_from", type=float, metavar="S",
                    help="With --average on a CSV log: ignore rows before S seconds since boot")
    ap.add_argument("--to", dest="t_to", type=float, metavar="S",
                    help="With --average on a CSV log: ignore rows after S seconds since boot")
    ap.add_argument("--elevation", type=float, metavar="M",
                    help="Print the standard-atmosphere boiling point at this elevation (m)")
    ap.add_argument("--pressure", type=float, metavar="KPA",
                    help="Print the boiling point at this station pressure (kPa, not sea-level corrected)")
    ap.add_argument("--firmware", default=SENSOR_CPP, help="Path to sensor_manager.cpp")
    args = ap.parse_args()

    if args.elevation is not None:
        p = 101.325 * (1 - 2.25577e-5 * args.elevation) ** 5.25588
        print(f"Elevation {args.elevation:.0f} m: ~{p:.1f} kPa, water boils at ~{boiling_point_c(p):.2f} C "
              f"(weather moves this by about +/-0.4 C)")
    if args.pressure is not None:
        print(f"Station pressure {args.pressure:.1f} kPa: water boils at {boiling_point_c(args.pressure):.2f} C")
    if args.average:
        average_log(args.average, args.t_from, args.t_to, args.timeline)
    if not args.point:
        if not (args.average or args.elevation is not None or args.pressure is not None):
            ap.print_help()
        return

    c = read_firmware_constants(args.firmware)
    print(f"Circuit (from {os.path.relpath(args.firmware, REPO)}): VS={c['THERMISTOR_SUPPLY_VOLTAGE']} V, "
          f"full scale={c['ADC_FULL_SCALE_VOLTAGE']} V, A0 load={c['ADC_INPUT_RESISTANCE']:.0f} ohm, "
          f"gain={c['ADC_CALIBRATION_GAIN']}")

    sensors = {
        "PID": dict(idx=1, rs=c["SERIES_RESISTOR"], r25=c["THERMISTOR_NOMINAL"], b=c["B_COEFFICIENT"],
                    names=("THERMISTOR_NOMINAL", "B_COEFFICIENT")),
        "AUX": dict(idx=2, rs=c["AUX_SERIES_RESISTOR"], r25=c["AUX_THERMISTOR_NOMINAL"], b=c["AUX_B_COEFFICIENT"],
                    names=("AUX_THERMISTOR_NOMINAL", "AUX_B_COEFFICIENT")),
    }
    parsed = []
    for p in args.point:
        parts = p.split(":")
        if len(parts) != 3:
            sys.exit(f"Bad --point '{p}', expected TEMP_C:PID_ADC:AUX_ADC")
        parsed.append((float(parts[0]), parts[1], parts[2]))

    for name, s in sensors.items():
        pts = [(t, float(v[s["idx"] - 1])) for t, *v in parsed if v[s["idx"] - 1] != "-"]
        print(f"\n=== {name} Temp Sensor (series {s['rs']:.0f} ohm, current R25={s['r25']:.0f}, B={s['b']:.0f}) ===")
        if not pts:
            print("  no points")
            continue
        rpts = [(t, adc_to_resistance(a, s["rs"], c)) for t, a in pts]
        r25, b = fit(rpts, s["r25"])
        if len(pts) == 1:
            print("  1 point: fitting B only, R25 kept at the datasheet value. Add a second "
                  "temperature (e.g. boiling water) for a proper fit.")
        print(f"  {'known C':>8} {'ADC':>8} {'NTC ohm':>10} {'now reads':>10} {'fit reads':>10}")
        for (t, a), (_, r) in zip(pts, rpts):
            print(f"  {t:8.2f} {a:8.1f} {r:10.0f} {beta_temp_c(r, s['r25'], s['b']):10.2f} "
                  f"{beta_temp_c(r, r25, b):10.2f}")
        print(f"  Fit: R25 = {r25:.0f} ohm, B = {b:.0f} K")
        print("  Effect at other temperatures (if the fit is right, today's firmware reads):")
        for t in (25, 45, 60, 65, 70):
            r_true = resistance_at(t, r25, b)
            print(f"    true {t:3d} C: current constants read {beta_temp_c(r_true, s['r25'], s['b']):6.2f} C")
        print(f"  src/sensor_manager.cpp:")
        print(f"    const float {s['names'][0]} = {r25:.1f};")
        print(f"    const float {s['names'][1]} = {b:.1f};")


if __name__ == "__main__":
    main()
