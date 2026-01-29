# Vortex RGB Driver - STM32G431RBT6

## Overview

Vortex-RGB is a high-performance RGB LED/laser driver firmware for the STM32G431RBT6 microcontroller. This project provides precise control over three independent RGB channels with advanced features including current monitoring, temperature sensing, and multiple communication interfaces.

## Hardware Specifications

- **MCU**: STM32G431RBT6 (ARM Cortex-M4, 170MHz, 128KB Flash, 32KB RAM)
- **Development Environment**: STM32CubeIDE 1.16.1
- **Firmware Package**: STM32Cube FW_G4 V1.6.1
- **Date**: January 22, 2026

## Key Features

### RGB Channel Control
- **3 Independent RGB Channels** (Red, Green, Blue)
- **Adjustable Parameters per Channel**:
  - Threshold control via 8-bit digital potentiometer (AD5160)
  - Maximum current control via 16-bit DAC (DAC8551)
  - Smolder/cutoff adjustment via internal DAC
  - Current monitoring with shunt resistors (20x amplifier)
  - Maximum current limits: R=1150 mA, G=2200 mA, B=3600 mA
  - Shunt values: R=200 mΩ, G=100 mΩ, B=50 mΩ

### Peripherals and Interfaces
- **ADC**: Dual ADC (ADC1, ADC2) with DMA for current and voltage monitoring
- **DAC**: Internal DACs (DAC1, DAC3) for precise analog control
- **Comparators**: COMP1, COMP2, COMP4 for smolder control
- **Communication**:
  - USB Device (Full Speed) - CDC class for configuration
  - USART1 - Serial communication
  - FDCAN1 - CAN FD bus support
  - I2C2 - For EEPROM and sensors
  - SPI1 - For external DACs and digital potentiometers
- **Sensors**:
  - LSM6DSOX - 6-axis IMU (accelerometer + gyroscope)
  - Temperature monitoring for module and radiator
- **Storage**: External EEPROM (4KB, I2C) for configuration persistence
- **Safety**: Independent Watchdog Timer (IWDG)

### Signal Connections

#### ADC Inputs
- **ADC1**: MSV_ADC, R_ADC, G_ADC, B_ADC, B_SHT (differential), Temperature, Vbat
- **ADC2**: BSV_ADC, G_SHT (differential), GSV_ADC, RSV_ADC, R_SHT (differential)

#### GPIO Controls
- **LED Indicators**: RED (PC10), GREEN (PA15)
- **Power Enable**: R_PEN (PC8), G_PEN (PB10), B_PEN (PA5)
- **SPI Chip Selects**:
  - Threshold: R_TH_CS (PC6), G_TH_CS (PB12), B_TH_CS (PC14)
  - Divider: R_DIV_CS (PC7), G_DIV_CS (PB13), B_DIV_CS (PC15)
  - Gyroscope: GYRO_CS (PD2)
- **Interrupts**: LICHTSC_C (PC13) - Light sensor/control, GYRO_INT (PC12) - Gyroscope interrupt

## Project Structure

```
Vortex-RGB/
├── Core/
│   ├── AD5160/          # 8-bit digital potentiometer driver
│   ├── DAC8551/         # 16-bit external DAC driver
│   ├── Eeprom/          # EEPROM driver for configuration storage
│   ├── FDCAN/           # CAN FD communication
│   ├── Inc/             # Header files
│   ├── lsm6dsox_app/    # LSM6DSOX IMU sensor application
│   ├── RGB/             # RGB channel control module
│   ├── Src/             # Main source files
│   ├── Startup/         # MCU startup code
│   └── System/          # System initialization and task management
├── Drivers/             # STM32 HAL drivers
├── Middlewares/         # USB Device middleware
├── USB_Device/          # USB CDC configuration
├── X-CUBE-MEMS1/        # MEMS sensor software pack
├── Debug/               # Build artifacts (not tracked)
└── STM32G431RBTX_FLASH.ld  # Linker script
```

## Building the Project

### Prerequisites
- STM32CubeIDE 1.16.1 or later
- STM32CubeMX 6.16.0 (for configuration changes)

### Build Instructions

1. **Open Project in STM32CubeIDE**:
   ```bash
   File → Open Projects from File System
   Navigate to the project directory
   ```

2. **Build the Project**:
   ```bash
   Project → Build All (Ctrl+B)
   ```
   Or use the hammer icon in the toolbar.

3. **Flash to Hardware**:
   ```bash
   Run → Debug (F11)
   ```
   Or use the debug icon to flash and debug.

### Build Configurations
- **Debug**: Full debug symbols, no optimization
- **Release**: Optimized for size/speed (if configured)

## Configuration

### EEPROM Storage
Each RGB channel stores its configuration in EEPROM:
- Threshold value (8-bit)
- Divider value (16-bit)
- Smolder value (16-bit)
- Check digit: 0xAA55

Configuration is automatically loaded during initialization and can be saved via USB/UART commands.

### Communication Protocol
The system supports configuration via USB CDC and UART:
- Buffer sizes: RX=32 bytes, TX=32 bytes
- CRC-16 checksum for data integrity
- FDCAN support for bus communication

## Pin Configuration

See `Vortex RGB driver STM32G431RBT6.txt` for complete pin mapping details.

### Debug Interface
- **SWD**: PA13 (SWDIO), PA14 (SWCLK)

### Oscillator
- **HSE**: 8MHz crystal (PF0, PF1)
- **System Clock**: 170MHz (via PLL)

## Safety Features

- **Watchdog Timer**: Independent watchdog prevents system hang
- **Temperature Monitoring**: Protection against overheating
- **Current Monitoring**: Real-time current measurement for each channel
- **Comparator-based Cutoff**: Hardware-assisted smolder/cutoff control

## Usage

After power-on:
1. System initializes all peripherals
2. EEPROM configuration is loaded for each RGB channel
3. ADC calibration is performed
4. RGB drivers are initialized with stored parameters
5. Main task loop starts:
   - ADC measurements (DMA)
   - Watchdog refresh
   - LED status indication (RED LED toggles at 100ms)
   - System task processing

## Development Notes

- Code generation: Copy only necessary library files
- Peripheral initialization: Single files (no separate .c/.h per peripheral)
- Backup: Disabled (files not backed up during regeneration)
- Power optimization: Free pins not set to analog

## License

Copyright (c) 2025 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.

## Author

- Optik (2025)

## References

- [STM32G431RB Product Page](https://www.st.com/en/microcontrollers-microprocessors/stm32g431rb.html)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
