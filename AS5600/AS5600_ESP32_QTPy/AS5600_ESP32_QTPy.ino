/*
 * AS5600 Magnetic Position Sensor Reader
 * Target: ESP32 QT Py (Adafruit ESP32-PICO-D4)
 * 
 * Reads angle position from AS5600 sensor via I2C (STEMMA QT connector)
 * and outputs to Serial Monitor.
 * 
 * Hardware Connections:
 * - AS5600 connected via STEMMA QT cable to ESP32 QT Py I2C port
 * - Uses Wire1 (STEMMA QT connector)
 * 
 * Required Library:
 * - Adafruit AS5600 (install via Library Manager)
 */

#include <Wire.h>
#include <Adafruit_AS5600.h>

Adafruit_AS5600 as5600;  // Create sensor object

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial port to connect (needed for native USB)
  }
  delay(1000);
  
  Serial.println("AS5600 Magnetic Position Sensor - ESP32 QT Py");
  Serial.println("==============================================");
  
  // Initialize I2C on Wire1 (STEMMA QT connector)
  Wire1.begin();
  
  // Initialize AS5600 sensor with Wire1
  if (!as5600.begin(0x36, &Wire1)) {
    Serial.println("✗ ERROR: AS5600 sensor not found!");
    Serial.println("  Check:");
    Serial.println("  - STEMMA QT cable is connected");
    Serial.println("  - AS5600 sensor has power");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ AS5600 sensor detected!");
  Serial.println("\nStarting readings...");
  Serial.println("Format: Angle (degrees) | Raw Value");
  Serial.println("---------------------------------------");
}

void loop() {
  // Read angle value
  uint16_t rawAngle = as5600.getRawAngle();
  float angleDegrees = (rawAngle * 360.0) / 4096.0;
  
  // Print to Serial Monitor
  Serial.print("Angle:");
  Serial.println(angleDegrees, 2);
  
  delay(100); // Read every 100ms
}
