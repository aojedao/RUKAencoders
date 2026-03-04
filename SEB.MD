# Project Context

## Target Hardware
- **Primary MCU/Processor**: ESP32-PICO-D4
- **Secondary Processors**: N/A
- **Dev Board / Custom PCB**: Adafruit ESP32 QT Py
- **Board Revision**: N/A

## Clock Configuration
- **HSE/HSI**: N/A (ESP32 uses internal oscillators)
- **PLL Config**: Default ESP32 configuration
- **System Clock Frequency**: 240 MHz (ESP32 default)

## Pin Mapping / GPIO Assignments
- **I2C (Wire1)**: STEMMA QT connector (SDA/SCL pins for QT Py)

## Memory Layout
- **Flash Size**: 4 MB (ESP32-PICO-D4 typical)
- **RAM Size**: 520 KB SRAM (ESP32 internal)
- **Linker Script Regions**: Default ESP32 Arduino framework

## Peripherals & Interfaces
- **I2C (Wire1)**: STEMMA QT connector at 400 kHz

## External Devices & Sensors
- **MLX90393**: 3-axis magnetometer (multiple units, addresses 0x0C, 0x10, 0x11, 0x12, 0x13)
- **AS5600**: Magnetic position sensor (address 0x36)
- **TCA9548A/PCA9548A**: I2C multiplexer (address 0x70) - Qwiic Mux Shield

## Communication Protocols
- **Protocol (e.g. CAN, UART, SPI, I2C)**: I2C, UART (Serial)
- **Baud Rate / Mode**: I2C at 400 kHz, UART at 115200 baud

## Software Stack
- **Build System**: Arduino IDE / PlatformIO
- **HAL / Framework**: Arduino Core for ESP32
- **RTOS**: FreeRTOS (built into ESP32 Arduino core)
- **Toolchain**: Xtensa GCC
- **Compiler Flags**: Default Arduino ESP32 flags
- **External Libraries**: Wire.h, MLX90393, Adafruit_AS5600

## Development Environment
- **Host OS**: Windows (based on project path)
- **IDE**: Arduino IDE or compatible
- **Debug Interface (SWD/JTAG)**: USB Serial (native USB on ESP32)
- **Debug Probe (ST-Link/J-Link/OpenOCD)**: USB Serial Monitor

## Power Management
- **Sleep Modes**: Not configured
- **Power Domains**: N/A
- **Battery / Power Source**: USB-C power (5V)

## Boot Configuration
- **Bootloader**: ESP32 ROM bootloader + Arduino bootloader
- **OTA Mechanism**: Not implemented

## Interrupt Configuration
- **NVIC Priority Groups**: Default ESP32 FreeRTOS configuration
- **Key ISRs**: I2C hardware interrupts (handled by Arduino Wire library)

## Compliance & Standards
- N/A

## Supporting Devices
- **Qwiic/STEMMA QT ecosystem**: Multiplexer shield with 5 sensor boards, each containing 5 MLX90393 sensors

## Project Notes
- Project contains multiple sensor test configurations for magnetic position sensing
- Main application uses TCA9548A I2C multiplexer to read from 5 boards with 5 sensors each (25 total MLX90393 sensors)
- AS5600 test code demonstrates single magnetic encoder reading
- All I2C communication uses Wire1 interface (STEMMA QT connector on ESP32 QT Py)
- Sensors operate in burst mode for continuous readings
- Serial output at 115200 baud for debugging and data visualization
