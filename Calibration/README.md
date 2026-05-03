# Calibration Scripts

This folder contains Arduino sketches for calibrating the robotic arm's potentiometer axes.

## Files

### `All_Axis_Calibration.ino`
Simultaneously reads and displays data from all four potentiometer pins (ESP32 pins 34, 35, 36, 39).

**Features:**
- Reads all 4 axis potentiometers at once
- Displays raw ADC values, voltage readings, and calculated angles
- Uses 12-bit ADC resolution (0-4095 range)
- 3.3V input range with attenuation set to ADC_11db
- Updates every 250ms

**Output Format:**
```
PIN [pin#]: [Raw: [value] | Volts: [voltage]V | Ang: [degrees]°]
```

### `Axis_degree_calibration.ino`
Single axis calibration sketch for debugging individual potentiometer axis (pin 34).

**Features:**
- Tests one potentiometer at a time
- Displays raw ADC value, voltage, and calculated angle
- Assumes 270-degree potentiometer range
- Updates every 100ms

**Usage:**
1. Upload the appropriate sketch to your ESP32
2. Open the Serial Monitor (115200 baud)
3. Rotate potentiometer and observe readings
4. Adjust calibration values as needed in the code

**Note:** Both sketches assume a 270° potentiometer range. Modify the range value (270.0) if your potentiometers have different specifications.
