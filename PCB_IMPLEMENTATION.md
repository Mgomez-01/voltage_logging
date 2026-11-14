# Fabrication outputs: pcb_voltage_logging

Generated 2026-11-13 from the saved board with KiCad 10.0.8.
DRC was clean (0 errors, 0 warnings) against `pcb_voltage_logging.kicad_dru`,
the combined JLCPCB + PCBWay 2-layer rules (see `../fab_rules/`).

## Zip these for the fab (JLCPCB or PCBWay)

| File | Layer |
|---|---|
| `pcb_voltage_logging-F_Cu.gtl` | Top copper |
| `pcb_voltage_logging-B_Cu.gbl` | Bottom copper |
| `pcb_voltage_logging-F_Mask.gts` | Top solder mask |
| `pcb_voltage_logging-B_Mask.gbs` | Bottom solder mask |
| `pcb_voltage_logging-F_Silkscreen.gto` | Top silkscreen |
| `pcb_voltage_logging-B_Silkscreen.gbo` | Bottom silkscreen |
| `pcb_voltage_logging-Edge_Cuts.gm1` | Board outline |
| `pcb_voltage_logging-PTH.drl` | Plated holes (Excellon, mm) |
| `pcb_voltage_logging-NPTH.drl` | Non-plated holes (the 2 × 3.2 mm mounting holes) |
| `pcb_voltage_logging-job.gbrjob` | Gerber job file (optional, fabs ignore it or use it) |

Leave out of the zip:

* `*-drl_map.gbr`: drill maps, for checking only.
* `*_Paste.gtp/.gbp`: only needed to order a stencil. B.Paste covers J6.
* `drc_report.txt`, `README.md`, the BOM, and `old_*/` (stale exports from
  before the board was finished).

## Plot settings used

* Protel extensions
* X2 off, netlist attributes off
* Silkscreen has the soldermask subtracted
* Values not plotted
* 4.6 coordinate precision, mm
* Drill files: Excellon, mm, decimal, absolute origin, oval holes in
  alternate mode, PTH/NPTH in separate files

These settings match JLCPCB's KiCad guide. PCBWay accepts the same set.

## Order options

* 2 layers, FR-4, 1.6 mm, 1 oz copper.
* Any mask colour. HASL lead-free is fine for hand soldering. Pick ENIG if
  the microSD socket and SOIC pads should be flatter.
* Min track and via are well inside both fabs' standard tiers:
  * vias: 0.4 mm drill
  * smallest plated hole: 0.4 mm
  * the design rules allow 0.1 mm track/space
* Remove the order number from the silkscreen, or let them place it. The fabs
  offer a paid option for no number.

## Hand assembly

`pcb_voltage_logging_BOM.csv` is grouped by value and footprint (refs, value,
footprint, qty). All passives are 0805.

* **J6** (Hirose DM3AT) is on the **bottom** side.
* **U1** is the NodeMCU v2 on 0.9″ socket rows. Fit female headers.
* **MH1/MH2** are holes only. They are listed in the BOM but there is nothing
  to buy.
