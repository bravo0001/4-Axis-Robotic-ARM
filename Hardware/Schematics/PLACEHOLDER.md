# Hardware — Schematics

Place your schematic files in this directory.

## Supported Formats

| Tool | File Extensions |
|---|---|
| KiCad | `.kicad_sch`, `.kicad_pro` |
| Eagle / Fusion 360 | `.sch`, `.brd` |
| EasyEDA | `.json` |
| PDF Export | `.pdf` |

## Contents (add your files below)

- `Receiver_Motherboard.kicad_sch` — Custom ESP32 PCB schematic
- `Transmitter_Controller.kicad_sch` — Transmitter ESP32 + IMU schematic

## Key Net / Signal Reference

| Signal | Description |
|---|---|
| `POT_AXIS1` | Potentiometer wiper → GPIO 34 |
| `POT_AXIS2` | Potentiometer wiper → GPIO 35 |
| `POT_AXIS3` | Potentiometer wiper → GPIO 36 |
| `POT_AXIS4` | Potentiometer wiper → GPIO 39 |
| `M1_FWD` / `M1_REV` | Relay H-Bridge control for Motor 1 |
| `M2_FWD` / `M2_REV` | Relay H-Bridge control for Motor 2 |
| `M3_FWD` / `M3_REV` | Relay H-Bridge control for Motor 3 |
| `M4_FWD` / `M4_REV` | Relay H-Bridge control for Motor 4 |
| `12V_MOTOR` | 12V supply rail for DC motors |
| `3V3` | 3.3V rail for ESP32 & logic |
| `GND` | Common ground |
