# MuxMultiSensor Configuration Guide

## Overview

The MuxMultiSensor sketch now supports mixed board configurations:
- **5X boards**: 5 sensors per board (addresses: 0x0C, 0x13, 0x12, 0x10, 0x11)
- **1X boards**: 1 sensor per board (address: 0x0C or 0x0F)

## Configuration Section

Edit the configuration section in `MuxMultiSensor.ino` (lines 23-45):

```cpp
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
```

## Example Configurations

### Example 1: All 5X Boards (Default)
```cpp
bool boardIs5X[5] = {true, true, true, true, true};
// board1XAddress is ignored
```
**Total sensors**: 25 (numbered 0-24)

### Example 2: Mixed Configuration
```cpp
bool boardIs5X[5] = {true, true, false, false, true};
uint8_t board1XAddress[5] = {0x0C, 0x0C, 0x0C, 0x0F, 0x0C};
```
- Board 0: 5X (sensors 0-4)
- Board 1: 5X (sensors 5-9)
- Board 2: 1X at 0x0C (sensor 10)
- Board 3: 1X at 0x0F (sensor 11)
- Board 4: 5X (sensors 12-16)

**Total sensors**: 17 (numbered 0-16)

### Example 3: All 1X Boards
```cpp
bool boardIs5X[5] = {false, false, false, false, false};
uint8_t board1XAddress[5] = {0x0C, 0x0C, 0x0F, 0x0F, 0x0C};
```
**Total sensors**: 5 (numbered 0-4)

## Output Format

Sensors are numbered sequentially starting from 0:

```
Sensor0X:<value>  Sensor0Y:<value>  Sensor0Z:<value>  Sensor1X:<value> ...
```

Failed or missing sensors output 0 for all axes:
```
Sensor5X:0  Sensor5Y:0  Sensor5Z:0
```

## Key Changes from Original

1. **Board configuration**: Specify 5X or 1X for each board
2. **1X address selection**: Choose 0x0C or 0x0F for 1X boards
3. **No wasted reads**: Only reads sensors that exist on each board
4. **Sequential numbering**: Sensors numbered 0, 1, 2... (not Board0Sensor0)
5. **Zero output**: Failed sensors print 0 instead of random values

## Initialization Messages

During startup, you'll see:
```
--- Initializing Board 0 (5X) ---
  Sensor 0 at address 0x0C... OK
  Sensor 1 at address 0x13... OK
  ...
--- Initializing Board 2 (1X) ---
  Sensor 0 at address 0x0F... OK
```

This helps verify your configuration is correct.
