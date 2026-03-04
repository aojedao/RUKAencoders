/*
 * AS5600 Magnetic Position Sensor with TCA9548A Multiplexer
 * Target: ESP32 QT Py (Adafruit ESP32-PICO-D4)
 * 
 * Reads angle positions from 3 AS5600 sensors via TCA9548A I2C multiplexer
 * and outputs to Serial Plotter in standard signal:value format.
 * 
 * Hardware Connections:
 * - TCA9548A multiplexer connected via STEMMA QT to ESP32 QT Py
 * - AS5600 sensors connected to multiplexer channels 0, 1, and 2
 * - Uses Wire1 (STEMMA QT connector)
 * 
 * Required Libraries:
 * - Adafruit AS5600 (install via Library Manager)
 */

#include <Wire.h>
#include <Adafruit_AS5600.h>

#define TCAADDR 0x70  // The I2C address of the TCA9548A multiplexer
#define NUMBER_OF_SENSORS 3
#define AS5600_ADDR 0x36

// Single sensor object that we'll use for all channels
Adafruit_AS5600 as5600;

// Track which sensors initialized successfully
bool sensorIsInitialized[NUMBER_OF_SENSORS];

// Function to select a specific channel on the TCA9548A multiplexer
void tcaSelect(uint8_t i) {
  if (i > 7) return; // The TCA9548A has 8 channels (0-7)

  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();
  delay(5); // Small delay for mux switching
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial port to connect (needed for native USB)
  }
  delay(1000);
  
  Serial.println("AS5600 Multi-Sensor with TCA9548A Mux - ESP32 QT Py");
  Serial.println("====================================================");
  
  // Initialize I2C on Wire1 (STEMMA QT connector)
  Wire1.begin();
  Wire1.setClock(400000); // Set I2C clock speed to 400kHz
  delay(10);
  
  // Initialize each AS5600 sensor on its respective mux channel
  for (uint8_t sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {
    tcaSelect(sensor); // Select mux channel
    delay(50); // Ensure bus is stable
    
    Serial.print("Initializing AS5600 Sensor ");
    Serial.print(sensor);
    Serial.print(" on Mux Channel ");
    Serial.print(sensor);
    Serial.print("... ");
    
    // Initialize AS5600 sensor with Wire1 at address 0x36
    if (!as5600.begin(AS5600_ADDR, &Wire1)) {
      Serial.println("FAILED!");
      Serial.println("  Check:");
      Serial.println("  - Sensor is connected to correct mux channel");
      Serial.println("  - AS5600 sensor has power");
      sensorIsInitialized[sensor] = false;
    } else {
      Serial.println("SUCCESS!");
      sensorIsInitialized[sensor] = true;
    }
    delay(10);
  }
  
  Serial.println("====================================================");
  Serial.println("All sensors initialized. Starting readings...");
  Serial.println("Format: Sensor0:angle Sensor1:angle Sensor2:angle");
  Serial.println("====================================================");
}

void loop() {
  // Read and print each sensor immediately in the same loop
  for (uint8_t sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {
    tcaSelect(sensor); // Select mux channel for this sensor
    
    float angle = 0.0;
    if (sensorIsInitialized[sensor]) {
      // Read angle value directly
      uint16_t rawAngle = as5600.getRawAngle();
      angle = (rawAngle * 360.0) / 4096.0;
    }
    
    // Print immediately
    Serial.print("Sensor");
    Serial.print(sensor);
    Serial.print(":");
    Serial.print(angle, 2);
    
    // Add tab separator between sensors (except after last sensor)
    if (sensor < NUMBER_OF_SENSORS - 1) {
      Serial.print("\t");
    }
  }
  
  Serial.println(); // Newline at end of reading cycle
  delay(100); // Read every 100ms
}
