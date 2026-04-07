/*
  Qwiic Mux Shield - PCA9548A/TCA9548A Multi-Board MLX90393 Reader
  By: Gemini (code revision)
  Original code by: Nathan Seidle
  SparkFun Electronics

  This code reads from multiple sensor boards connected to a TCA9548A I2C mux.
  Each board can have either 5 sensors (5X) or 1 sensor (1X).
  1X boards can use either 0x0C or 0x0F as their sensor address.

  Hardware Connections:
  - Attach the Qwiic Mux Shield to your microcontroller.
  - Plug sensor boards into the mux ports (0-4).
  - Open the Serial Monitor at 115200 baud to see the raw output.
  - Open the Serial Plotter to see the data visualization.
*/

#include <Wire.h>
#include <MLX90393.h> 

#define TCAADDR 0x70 // The I2C address of the TCA9548A multiplexer

// ============================================================================
// BOARD CONFIGURATION - MODIFY THIS SECTION FOR YOUR SETUP
// ============================================================================

#define NUMBER_OF_BOARDS 5

// Board types: true = 5X board (5 sensors), false = 1X board (1 sensor)
bool boardIs5X[NUMBER_OF_BOARDS] = {
  true,   // Board 0: 5X
  true,   // Board 1: 5X
  true,   // Board 2: 5X
  true,   // Board 3: 5X
  true    // Board 4: 5X
};

// For 1X boards only: sensor address (0x0C or 0x0F)
// This is ignored for 5X boards
uint8_t board1XAddress[NUMBER_OF_BOARDS] = {
  0x0C,   // Board 0 (ignored if 5X)
  0x0C,   // Board 1 (ignored if 5X)
  0x0C,   // Board 2 (ignored if 5X)
  0x0C,   // Board 3 (ignored if 5X)
  0x0C    // Board 4 (ignored if 5X)
};

// ============================================================================
// END CONFIGURATION
// ============================================================================

// Maximum sensors per board (for array sizing)
#define MAX_SENSORS_PER_BOARD 5

// The I2C addresses for the 5 sensors on 5X boards
uint8_t sensor5XAddresses[5] = {0x0C, 0x13, 0x12, 0x10, 0x11};

// Create a 2D array for the MLX90393 sensor objects and their data
MLX90393 mlx[NUMBER_OF_BOARDS][MAX_SENSORS_PER_BOARD];
MLX90393::txyz data[NUMBER_OF_BOARDS][MAX_SENSORS_PER_BOARD];

// A 2D array to track which sensors have been successfully initialized
bool sensorIsInitialized[NUMBER_OF_BOARDS][MAX_SENSORS_PER_BOARD];

// Y-axis offsets for visualization on the Arduino Plotter
int yOffsets[NUMBER_OF_BOARDS] = {0};

// Function to select a specific channel on the TCA9548A multiplexer
void tcaSelect(uint8_t i) {
  if (i > 7) return; // The TCA9548A has 8 channels (0-7)

  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}

// Get the number of sensors on a specific board
byte getSensorCount(byte board) {
  return boardIs5X[board] ? 5 : 1;
}

// Get the sensor address for a specific board and sensor index
uint8_t getSensorAddress(byte board, byte sensor) {
  if (boardIs5X[board]) {
    return sensor5XAddresses[sensor];
  } else {
    return board1XAddress[board];
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Multi-Board MLX90393 Reader");
  Serial.println("============================================");
  Wire1.begin(); // Start the I2C bus used by the mux
  Wire1.setClock(400000); // Set the I2C clock speed
  delay(10);

  // Initialize all sensor slots to false
  for (byte board = 0; board < NUMBER_OF_BOARDS; board++) {
    for (byte sensor = 0; sensor < MAX_SENSORS_PER_BOARD; sensor++) {
      sensorIsInitialized[board][sensor] = false;
    }
  }

  // Initialize the sensors on all boards
  for (byte board = 0; board < NUMBER_OF_BOARDS; board++) {
    tcaSelect(board); // Enable the multiplexer channel for this board
    delay(50); // Add a longer delay to ensure the bus is stable
  
    Serial.print("--- Initializing Board ");
    Serial.print(board);
    Serial.print(" (");
    Serial.print(boardIs5X[board] ? "5X" : "1X");
    Serial.println(") ---");
  
    byte sensorCount = getSensorCount(board);
    
    for (byte sensor = 0; sensor < sensorCount; sensor++) {
      uint8_t address = getSensorAddress(board, sensor);
      
      Serial.print("  Sensor ");
      Serial.print(sensor);
      Serial.print(" at address 0x");
      Serial.print(address, HEX);
      Serial.print("... ");

      byte status = mlx[board][sensor].begin(address, -1, Wire1);
    
      if (status == 0) {
        Serial.println("OK");
        mlx[board][sensor].startBurst(0xF); // Start burst mode
        sensorIsInitialized[board][sensor] = true;
      } else {
        Serial.print("FAILED (status: ");
        Serial.print(status);
        Serial.println(")");
        sensorIsInitialized[board][sensor] = false;
      }
      delay(10); // Short delay between sensor initializations
    }
  }
  Serial.println("============================================");
  Serial.println("Initialization complete. Starting readings...");
  Serial.println();
}

void loop() {
  int sensorNumber = 0; // Global sensor counter starting from 0
  
  for (byte board = 0; board < NUMBER_OF_BOARDS; board++) {
    tcaSelect(board); // Tell mux to connect to this board's port
    delay(10); // Ensure the bus connection is stable

    byte sensorCount = getSensorCount(board);
    
    for (byte sensor = 0; sensor < sensorCount; sensor++) {
      // Only attempt to read if the sensor was initialized successfully
      if (sensorIsInitialized[board][sensor]) {
        mlx[board][sensor].readBurstData(data[board][sensor]);

        // Print X, Y, Z data with sensor number starting from 0
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("X:");
        Serial.print(data[board][sensor].x + (board * 8000));
        Serial.print("\t");
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("Y:");
        Serial.print(data[board][sensor].y + (board * 8000));
        Serial.print("\t");
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("Z:");
        Serial.print(data[board][sensor].z + (board * 8000));
        Serial.print("\t");
      } else {
        // If the sensor failed to initialize, print 0 for all axes
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("X:");
        Serial.print(0);
        Serial.print("\t");
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("Y:");
        Serial.print(0);
        Serial.print("\t");
        Serial.print("Sensor");
        Serial.print(sensorNumber);
        Serial.print("Z:");
        Serial.print(0);
        Serial.print("\t");
      }
      
      sensorNumber++; // Increment global sensor counter
    }
  }
  
  Serial.println(); // Print a newline at the end of each full reading cycle
  delay(1); // Wait a short period before the next reading cycle
}
