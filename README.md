# MIPS - Modular Intelligent Power System

Firmware for the MIPS controller, developed by GAA Custom Electronics, LLC. MIPS is a modular instrument controller platform designed for mass spectrometry and ion processing applications. It runs on an Arduino Due (Atmel SAM3X8E, 84MHz ARM Cortex-M3) and is built with PlatformIO.

## Overview

MIPS provides centralized control over a collection of plug-in hardware modules through a round-robin multitasking architecture. Modules are auto-detected at startup via TWI EEPROM scanning. Each module registers itself, adds its menu entries to the system UI, and runs as an independent thread.

The system communicates with a host computer via USB (native) or serial port, with automatic detection of the active port. Ethernet and WiFi interfaces are also supported. All commands are processed through a table-driven serial command processor.

Configuration is saved to SD card (`default.cfg`) and restored automatically on startup.

## Supported Modules

| Module | Description |
|--------|-------------|
| **ARB** | Arbitrary waveform generator, up to 6 modules, with compression mode |
| **RFdriver** | RF driver for high-Q ion guides and ion traps |
| **RFamp** | RF amplifier control |
| **DCbias** | Multi-channel DC bias supply |
| **DCbiasCtrl** | DC bias control and sequencing |
| **DCbiasList** | DC bias list/table mode |
| **DCBcurrent** | DC bias current monitoring |
| **DCBswitch** | DC bias switching |
| **DAC** | SPI/CPLD-based DAC interface |
| **ADCdrv** | ADC driver |
| **FAIMS** | Field Asymmetric Ion Mobility Spectrometry driver |
| **FAIMSfb** | FAIMS feedback control |
| **HOFAIMS** | High-order FAIMS |
| **Twave** | Travelling wave ion guide driver (rev 1–5) |
| **Compressor** | Ion compression control |
| **ARBcompressor** | ARB-based compression |
| **ESI** | Electrospray ionization control |
| **Filament** | Filament/ion source control |
| **HVPS** | High voltage power supply |
| **HVPSinterface** | HVPS interface layer |
| **DIO** | Digital I/O |
| **Analog** | Analog I/O |
| **ClockGenerator** | CY22393 clock generator control |
| **FPGA** | FPGA interface |
| **FS7140** | FS7140 frequency synthesizer |
| **SC16IS740** | SC16IS740 UART bridge |
| **DMSDMSMB** | DMS/MS motherboard support |
| **ethernet** | Ethernet interface |
| **WiFi** | WiFi interface |
| **Table** | Pulse sequence table execution |
| **Log** | System logging |

## Hardware

- **Controller:** Arduino Due (Atmel SAM3X8E, 84MHz ARM Cortex-M3)
- **Display:** ILI9340 TFT with Adafruit GFX
- **Storage:** SD card (configuration and data)
- **Communication:** USB (native), UART serial, TWI (I2C), SPI, Ethernet, WiFi
- **Module bus:** TWI with board-select addressing (up to 8 modules per system)

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

To upload:

```bash
pio run --target upload
```

All dependencies are included in the `lib/` directory. No external library paths required.

## Project Structure

```
├── src/          # Firmware source files (.cpp)
├── include/      # Project header files (.h)
├── lib/          # Local library dependencies
├── test/         # Unit tests
└── platformio.ini
```

## Author

Gordon Anderson  
GAA Custom Electronics, LLC  
gaa@owt.com
