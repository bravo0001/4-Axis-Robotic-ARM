//this code runs all 4 axis together in synk with in there egree of rotation limt in loop to test everything is work perfectly fine and to check the calibration of all axis together in one go.


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
    Serial.print(voltage, 2); // 2 decimal places
    Serial.print("V | Ang: ");
    Serial.print(degrees, 1); // 1 decimal place
    Serial.print("°]   ");
  }
  
  // Print a new line after all pins are read
  Serial.println();

  delay(250); // Delay for readability
}