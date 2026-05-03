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
├── Calibration/
│   ├── All_Axis_Calibration.ino   # Multi-axis potentiometer calibration (all 4 axes simultaneously)
│   ├── Axis_degree_calibration.ino # Single-axis calibration for debugging individual axes
│   └── README.md                  # Calibration documentation
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
- Reads raw accelerometer & gyroscope data from IMU ( MPU-6050 via I²C)
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

## � Calibration

The `/Calibration/` folder contains two Arduino sketches for calibrating and testing the potentiometer feedback system:

### `All_Axis_Calibration.ino`
- Reads all four potentiometer axes simultaneously (GPIO 34, 35, 36, 39)
- Displays raw ADC values, voltage readings, and calculated angles in real-time
- Useful for verifying all axes are functioning correctly
- **Update interval:** 250 ms

### `Axis_degree_calibration.ino`
- Single-axis debugging sketch targeting GPIO 34
- Ideal for troubleshooting individual potentiometer connections
- Displays detailed readings: raw value, voltage, and angle
- **Update interval:** 100 ms

**Usage:**
1. Upload the desired calibration sketch to the Receiver ESP32
2. Open Serial Monitor at **115200 baud**
3. Rotate each potentiometer and verify the readings are linear and within expected ranges (0°–270°)
4. Adjust potentiometer range constants (270.0) in the code if your sensors have different specifications

### Running All_Axis_Calibration.ino
Upload and run this sketch on the Receiver ESP32 to simultaneously test all four potentiometers:
```cpp
// Array of pins you want to read
const int potPins[] = {34, 35, 36, 39};
const int numPins = 4;

void setup() {
  Serial.begin(115200);
  
  // ESP32 ADC setup
  analogReadResolution(12); // 0-4095 range
  analogSetAttenuation(ADC_11db); // Standard for 0-3.3V range

  Serial.println("Multi-Pin Potentiometer Initialization...");
}

void loop() {
  for (int i = 0; i < numPins; i++) {
    int rawValue = analogRead(potPins[i]);
    
    // Calculate voltage and degrees
    float voltage = rawValue * (3.3 / 4095.0);
    float degrees = (rawValue / 4095.0) * 270.0;

    // Print label for each pin
    Serial.print("PIN "); 
    Serial.print(potPins[i]);
    Serial.print(": [Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Volts: ");
    Serial.print(voltage, 2);
    Serial.print("V | Ang: ");
    Serial.print(degrees, 1);
    Serial.print("°]   ");
  }
  
  Serial.println();
  delay(250); // Delay for readability
}
```

**Expected Serial Output:**
```
Multi-Pin Potentiometer Initialization...
PIN 34: [Raw: 2048 | Volts: 1.65V | Ang: 135.0°]   PIN 35: [Raw: 1024 | Volts: 0.83V | Ang: 67.5°]   PIN 36: [Raw: 3072 | Volts: 2.48V | Ang: 202.5°]   PIN 39: [Raw: 512 | Volts: 0.41V | Ang: 33.8°]
```

### Running Axis_degree_calibration.ino
Upload and run this sketch for debugging a single potentiometer:
```cpp
const int potPin = 34; // Analog pin for potentiometer

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // 0-4095 range
}

void loop() {
  int rawValue = analogRead(potPin);
  
  float voltage = rawValue * (3.3 / 4095.0);
  float degrees = (rawValue / 4095.0) * 270.0;

  Serial.print("Raw: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage);
  Serial.print("V | Angle: ");
  Serial.print(degrees);
  Serial.println("°");

  delay(100);
}
```

**Troubleshooting Calibration:**
- **Erratic readings:** Check potentiometer wires for loose connections; verify ADC pin is correct
- **Non-linear values:** Potentiometer may be damaged; try a different unit
- **Out of range (>270°):** Adjust the 270.0 multiplier based on your potentiometer's actual rotation range
- **Stuck at min/max:** Check for mechanical binding or wiring issues

---

## �🚀 Getting Started

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

