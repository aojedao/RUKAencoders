/*
  Single MLX90393 Sensor Test
  
  This sketch tests a single MLX90393 sensor connected through the TCA9548A
  I2C multiplexer. Use this to verify individual sensor functionality and
  diagnose potential sensor issues.

  Hardware Connections:
  - Attach the Qwiic Mux Shield to your ESP32 QT Py.
  - Connect a single sensor board to one of the mux ports (default: port 0).
  - Open the Serial Monitor at 115200 baud to see the raw output.
  - Open the Serial Plotter to see the data visualization.

  Configuration:
  - Set MUX_CHANNEL to the port where your sensor is connected (0-7)
  - Set SENSOR_ADDRESS to your sensor's I2C address (typically 0x0C or 0x0F)
*/

#include <Wire.h>
#include <MLX90393.h>

// ============================================================================
// CONFIGURATION - MODIFY THESE VALUES FOR YOUR SETUP
// ============================================================================

#define TCAADDR 0x70          // I2C address of the TCA9548A multiplexer
#define MUX_CHANNEL 0         // Which mux port is the sensor connected to (0-7)
#define SENSOR_ADDRESS 0x0C   // I2C address of the MLX90393 sensor (0x0C or 0x0F)

// ============================================================================
// END CONFIGURATION
// ============================================================================

MLX90393 mlx;
MLX90393::txyz data;
bool sensorInitialized = false;

// Function to select a specific channel on the TCA9548A multiplexer
void tcaSelect(uint8_t channel) {
  if (channel > 7) return; // The TCA9548A has 8 channels (0-7)

  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial port to connect (needed for native USB)
  }
  delay(1000);
  
  Serial.println("Single MLX90393 Sensor Test");
  Serial.println("===========================");
  Serial.print("Mux Channel: ");
  Serial.println(MUX_CHANNEL);
  Serial.print("Sensor Address: 0x");
  Serial.println(SENSOR_ADDRESS, HEX);
  Serial.println();
  
  // Initialize I2C
  Wire1.begin();
  Wire1.setClock(400000); // Set I2C clock speed to 400kHz
  delay(10);

  // Select the mux channel
  tcaSelect(MUX_CHANNEL);
  delay(50);

  // Initialize the sensor
  Serial.print("Initializing sensor... ");
  byte status = mlx.begin(SENSOR_ADDRESS, -1, Wire1);
  
  if (status == 0) {
    Serial.println("SUCCESS!");
    mlx.startBurst(0xF); // Start burst mode (read X, Y, Z, T)
    sensorInitialized = true;
    Serial.println("Sensor is ready. Starting readings...");
  } else {
    Serial.print("FAILED! Status code: ");
    Serial.println(status);
    Serial.println();
    Serial.println("Troubleshooting:");
    Serial.println("  - Check sensor is connected to the correct mux channel");
    Serial.println("  - Verify sensor I2C address (try 0x0C or 0x0F)");
    Serial.println("  - Check power connections");
    Serial.println("  - Verify I2C pull-up resistors are present");
    sensorInitialized = false;
  }
  
  Serial.println();
  Serial.println("===========================");
  Serial.println();
}

void loop() {
  // Select the mux channel (in case it was changed)
  tcaSelect(MUX_CHANNEL);
  delay(10);

  if (sensorInitialized) {
    // Read sensor data
    mlx.readBurstData(data);

    // Print in the same format as MuxMultiSensor for Arduino Plotter compatibility
    Serial.print("Sensor0X:");
    Serial.print(data.x);
    Serial.print("\t");
    Serial.print("Sensor0Y:");
    Serial.print(data.y);
    Serial.print("\t");
    Serial.print("Sensor0Z:");
    Serial.print(data.z);
    Serial.println();
  } else {
    // Print zeros if sensor is not initialized
    Serial.print("Sensor0X:");
    Serial.print(0);
    Serial.print("\t");
    Serial.print("Sensor0Y:");
    Serial.print(0);
    Serial.print("\t");
    Serial.print("Sensor0Z:");
    Serial.print(0);
    Serial.println();
  }

  delay(1); // Short delay between readings
}
