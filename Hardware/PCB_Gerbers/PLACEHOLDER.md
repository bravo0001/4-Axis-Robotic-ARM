# Hardware — PCB Gerber Files

Place your PCB fabrication files in this directory.

## Required Gerber Layers

| Layer | Description | Typical Extension |
|---|---|---|
| Top Copper | F.Cu | `.gtl` / `F_Cu.gbr` |
| Bottom Copper | B.Cu | `.gbl` / `B_Cu.gbr` |
| Top Silkscreen | F.Silks | `.gto` / `F_Silkscreen.gbr` |
| Bottom Silkscreen | B.Silks | `.gbo` / `B_Silkscreen.gbr` |
| Top Solder Mask | F.Mask | `.gts` / `F_Mask.gbr` |
| Bottom Solder Mask | B.Mask | `.gbs` / `B_Mask.gbr` |
| Board Outline | Edge.Cuts | `.gko` / `Edge_Cuts.gbr` |
| Drill File | PTH / NPTH | `.drl` / `.xln` |

## Recommended Fabrication Services

- [JLCPCB](https://jlcpcb.com)
- [PCBWay](https://www.pcbway.com)
- [OSH Park](https://oshpark.com)

## Contents (add your files below)

- `Receiver_Motherboard/` — Gerber set for the custom ESP32 Receiver PCB
- `Transmitter_Controller/` — Gerber set for the Transmitter PCB (if custom)

## PCB Specifications (target)

| Parameter | Value |
|---|---|
| Layers | 2 |
| Board thickness | 1.6 mm |
| Copper weight | 1 oz |
| Min trace / space | 0.15 mm / 0.15 mm |
| Surface finish | HASL or ENIG |
