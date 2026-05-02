# 🦾 Wireless 4-DOF Robotic Arm

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

A wireless 4-Degrees-of-Freedom (4-DOF) robotic arm featuring a custom ESP32-based motherboard, **ESP-NOW peer-to-peer communication** for ultra-low-latency control, **12V DC motors** driven by relay H-Bridges, and **potentiometer-based absolute closed-loop feedback** processed through a **Kalman Filter** for smooth, noise-free motion. This project demonstrates advanced PCB design, digital signal processing, and industrial-grade mechatronics.

---

## 📐 System Architecture

```
┌─────────────────────────────────┐        ESP-NOW (P2P)        ┌─────────────────────────────────┐
│         TRANSMITTER             │ ──────────────────────────► │         RECEIVER / MOTHERBOARD  │
│                                 │       ~1ms latency          │                                 │
│  ESP32 DevKit                   │                             │  ESP32 Custom PCB               │
│  2x MPU-6050(IMU Sensor)        │                             │  4× 12V DC Motors               │
│  Kalman Filter (on-board)       │                             │  8× SPDT Relay H-Bridge         │
│  Reads 4-axis orientation       │                             │  4× 10kΩ Potentiometers (ADC1)  |
│                                 |                             |  12v Booster module             |
│                                 │                             │  Closed-Loop State Machine      │
└─────────────────────────────────┘                             └─────────────────────────────────┘
```

---

## ✨ Key Features

| Feature | Detail |
|---|---|
| **Communication** | ESP-NOW (Peer-to-Peer, no router required) |
| **Latency** | ~1 ms wireless transmission |
| **Degrees of Freedom** | 4-DOF (Base, Shoulder, Elbow, Wrist) |
| **Feedback** | Closed-loop absolute encoding via 10kΩ potentiometers |
| **ADC Pins (Feedback)** | GPIO 34, 35, 36, 39 (ADC1 — safe with Wi-Fi/ESP-NOW) |
| **Signal Processing** | Kalman Filter for IMU noise reduction |
| **Motor Drive** | 12V DC motors via Relay H-Bridge circuits |
| **Firmware Architecture** | Non-blocking state machine on Receiver |

---

## 📁 Repository Structure

```
4-Axis-Robotic-ARM/
├── Firmware/
│   ├── Transmitter/
│   │   └── Transmitter.ino        # ESP32 transmitter: IMU read + Kalman + ESP-NOW send
│   └── Receiver/
│       └── Receiver.ino           # ESP32 receiver: ESP-NOW recv + state machine + motor control
├── Hardware/
│   ├── Schematics/
│   │   └── PLACEHOLDER.md         # Add KiCad / Eagle schematic files here
│   └── PCB_Gerbers/
│       └── PLACEHOLDER.md         # Add Gerber files for PCB fabrication here
├── .gitignore                     # Arduino / PlatformIO ignores
├── LICENSE                        # MIT License
└── README.md
```

---

## ⚙️ Technical Specifications

### Communication
- **Protocol:** ESP-NOW (IEEE 802.11 based, Layer 2)
- **Topology:** Point-to-Point (Transmitter → Receiver)
- **Payload:** Custom `struct` carrying 4-axis angle data (floats)
- **Range:** Up to ~200 m line-of-sight
- **Latency:** ~1 ms

### Transmitter (ESP32 + IMU)
- Reads raw accelerometer & gyroscope data from IMU (e.g., MPU-6050 via I²C)
- Applies **Kalman Filter** to fuse accel + gyro and eliminate sensor drift/noise
- Packs filtered 4-axis angles into a data packet and transmits via ESP-NOW

### Receiver / Motherboard (Custom ESP32 PCB)
- Receives angle packets via `OnDataRecv` ESP-NOW callback (interrupt-driven)
- Runs a **non-blocking state machine** for each axis to drive motors to target positions
- Reads current joint positions from 10kΩ potentiometers on **ADC1 pins** (GPIO 34, 35, 36, 39)
  - ADC1 is used because ADC2 is unavailable when Wi-Fi / ESP-NOW is active
- Drives **12V DC motors** via Relay H-Bridge circuits (direction + enable control)

### Feedback / Encoding
- **Type:** Absolute analog encoding (no homing required on power-up)
- **Sensor:** 10kΩ potentiometer per axis
- **ADC Resolution:** 12-bit (0–4095 mapped to 0°–270°)
- **Pins:** GPIO 34 (Axis 1), GPIO 35 (Axis 2), GPIO 36 (Axis 3), GPIO 39 (Axis 4)

### Kalman Filter (Digital Signal Processing)
- Single-variable discrete Kalman Filter implemented in C++
- **State:** Joint angle (degrees)
- **Process noise (Q):** Tunable — accounts for motor vibration & mechanical play
- **Measurement noise (R):** Tunable — accounts for ADC quantisation & potentiometer noise
- Runs on both Transmitter (IMU fusion) and Receiver (feedback smoothing)

---

## 🚀 Getting Started

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/software) ≥ 2.x **or** [PlatformIO](https://platformio.org/)
- ESP32 Arduino core (`espressif/arduino-esp32`)
- Library: `MPU6050` by Electronic Cats (or equivalent IMU library)

### Hardware Required
| Component | Qty |
|---|---|
| ESP32 DevKit v1 | 2 |
| MPU-6050 IMU Module | 1 |
| 12V DC Gear Motor | 4 |
| SPDT Relay H-Bridge (e.g., BTS7960) | 8 |
| 10kΩ Rotary Potentiometer | 4 |
| 12V DC Power Supply | 1 |
| 3.3V / 5V Logic Power | 1 |

### Flashing the Firmware

1. **Clone the repository:**
   ```bash
   git clone https://github.com/bravo0001/4-Axis-Robotic-ARM.git
   ```

2. **Flash the Transmitter:**
   - Open `Firmware/Transmitter/Transmitter.ino` in Arduino IDE
   - Select **Board:** `ESP32 Dev Module`
   - Update `receiverMacAddress[]` with your Receiver ESP32's MAC address
   - Upload

3. **Flash the Receiver:**
   - Open `Firmware/Receiver/Receiver.ino` in Arduino IDE
   - Select **Board:** `ESP32 Dev Module`
   - Upload

### Finding the Receiver MAC Address
Upload and run the following sketch on the Receiver ESP32 to print its MAC:
```cpp
#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
}
void loop() {}
```

---

## 📡 ESP-NOW Data Packet

```cpp
typedef struct {
  float axis1_angle;   // Base        (degrees)
  float axis2_angle;   // Shoulder    (degrees)
  float axis3_angle;   // Elbow       (degrees)
  float axis4_angle;   // Wrist       (degrees)
} RoboArmPacket;
```

---

## 🗂 Hardware Files

Schematic and PCB Gerber file placeholders are located in `/Hardware/`. Add your design files there:

- `/Hardware/Schematics/` — KiCad `.kicad_sch`, Eagle `.sch`, or exported PDF schematics
- `/Hardware/PCB_Gerbers/` — Gerber files (`.gbr`), drill files (`.drl`) for PCB fabrication

---

## 📜 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

## 🙌 Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

