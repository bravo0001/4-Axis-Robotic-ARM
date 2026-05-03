const int potPin = 34; // Analog pin for potentiometer

void setup() {
  Serial.begin(115200);
  // Set ADC resolution to 12-bit (0-4095)
  analogReadResolution(12); 
}

void loop() {
  int rawValue = analogRead(potPin);
  
  // Calculate voltage to ensure wiring is correct (should be 0 to 3.3V)
  float voltage = rawValue * (3.3 / 4095.0);
  
  // Calculate degrees based on a standard 270-degree potentiometer
  // If your pot has a different range, change 270.0
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
