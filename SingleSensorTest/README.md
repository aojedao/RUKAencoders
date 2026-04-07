# Single Sensor Test

This sketch is designed to test a single MLX90393 sensor connected through the TCA9548A I2C multiplexer. Use it to diagnose individual sensor issues.

## Hardware Setup

1. Connect your ESP32 QT Py to the TCA9548A multiplexer via the STEMMA QT connector
2. Connect a single sensor board to one of the mux ports (0-7)
3. Upload this sketch to your ESP32 QT Py

## Configuration

Before uploading, modify these values in `SingleSensorTest.ino`:

```cpp
#define MUX_CHANNEL 0         // Which mux port (0-7)
#define SENSOR_ADDRESS 0x0C   // Sensor I2C address (0x0C or 0x0F)
```

## Usage

1. Open the Arduino Serial Monitor at 115200 baud
2. The sketch will attempt to initialize the sensor and report success/failure
3. If successful, it will continuously read and display X, Y, Z values
4. Open the Serial Plotter to visualize the data

## Output Format

The output matches the MuxMultiSensor format:
```
Sensor0X:<value>    Sensor0Y:<value>    Sensor0Z:<value>
```

This format is compatible with the Arduino Serial Plotter.

## Troubleshooting

If initialization fails:
- Verify the sensor is connected to the correct mux channel
- Try both 0x0C and 0x0F as the sensor address
- Check power connections
- Verify I2C pull-up resistors are present on the mux board
