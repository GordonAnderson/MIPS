# MIPS Firmware Architecture

**Platform:** Arduino Due (Atmel SAM3X8E, 84 MHz ARM Cortex-M3)  
**Build system:** PlatformIO (framework = arduino)  
**Version:** see `src/MIPS.cpp` `Version[]` — currently 1.262

---

## Table of Contents

1. [Overview](#overview)
2. [Directory Structure](#directory-structure)
3. [Boot Sequence](#boot-sequence)
4. [Threading Model](#threading-model)
5. [Module Discovery](#module-discovery)
6. [Serial Command Routing](#serial-command-routing)
7. [Communication Interfaces](#communication-interfaces)
8. [TWI (I2C) Bus Architecture](#twi-i2c-bus-architecture)
9. [SPI Bus Architecture](#spi-bus-architecture)
10. [UI System — Menu and Dialog](#ui-system--menu-and-dialog)
11. [Configuration Persistence](#configuration-persistence)
12. [Hardware Abstraction Layer](#hardware-abstraction-layer)
13. [Feature Variant System](#feature-variant-system)
14. [Timer Assignments](#timer-assignments)
15. [Module Reference](#module-reference)

---

## Overview

MIPS (Modular Instrument Platform System) is firmware for a custom laboratory instrument controller. It manages multiple plug-in hardware modules (DC bias supplies, RF drivers, ion guides, filament controllers, etc.) over a shared TWI bus. Each module stores its identity and calibration data in an on-board serial EEPROM; MIPS discovers and initialises modules at boot time.

The firmware is structured as a cooperative, round-robin multitasking system using the ArduinoThread library. Each hardware module runs as an independent thread. Serial commands from the host computer (USB, Ethernet, or WiFi) are dispatched through a central table-driven command processor.

---

## Directory Structure

```
MIPS/
├── src/            C++ source files (one per module + core files)
├── include/        Header files
├── lib/            Bundled third-party libraries
│   ├── ArduinoThread/
│   ├── SerialBuffer/
│   └── ...
├── variants/       Arduino Due board variant overrides (PWM freq, etc.)
├── doc/            Architecture and reference documentation
├── platformio.ini  Build configuration
└── print_variant.py  Pre-build script — prints active variant on every build
```

---

## Boot Sequence

`setup()` in `src/MIPS.cpp` executes once at power-on:

1. **USB reinitialise** — work around a hardware errata where the USB PLL locks up on noise.
2. **Serial init** — `SerialInit()` configures the ring-buffer-based serial command processor.
3. **SPI init** — clock divider set to 21 (≈4 MHz); raised to higher speed after SD card init.
4. **Display init** — ILI9340 TFT, rotation 1 (landscape).
5. **SD card init** — mounts FAT filesystem; sets `SDcardPresent` flag.
6. **Load config** — `LOADparms("default.cfg")` restores the last saved system configuration from the SD card.
7. **IO init** — `Init_IOpins()`, `ClearDOshiftRegs()`, `PulseLDAC`.
8. **Encoder init** — rotary encoder with push button; interrupt-driven.
9. **TWI init** — `TWI_RESET()` then `Wire.begin()` at `WireDefaultSpeed`.
10. **Splash screen** — `Signon()`.
11. **Hardware scan** — `ScanHardware()` walks all TWI module addresses and calls each module's `_init()` function when recognised.
12. **DIO init** — digital I/O module always initialised regardless of scan.
13. **Thread setup** — `MIPSsystemThread` (10 ms) and `LEDThread` (500 ms) added to `ThreadController`.
14. **Startup macro** — if `MIPSconfigData.StartupMacro` is set, it is played now.
15. **Ethernet init** — `Ethernet_init()`.
16. **Watchdog enable** — hardware watchdog started; each thread must call `WDT_Restart(WDT)`.

---

## Threading Model

MIPS uses the [ArduinoThread](https://github.com/ivanseidel/ArduinoThread) cooperative round-robin scheduler. A single `ThreadController` named `control` owns all threads. `loop()` simply calls `control.run()`, which calls each thread's `onRun` callback when its interval has elapsed.

All threads run at **10 ms** intervals unless noted otherwise. There is no preemption; a thread that blocks will stall all others.

| Thread name        | Source file          | Interval | Purpose |
|--------------------|----------------------|----------|---------|
| System             | MIPS.cpp             | 10 ms    | Power monitoring, interlock, serial watchdog |
| LED                | MIPS.cpp             | 500 ms   | Push-button RGB LED state machine |
| DCbias             | DCbias.cpp           | 10 ms    | DC bias output update and readback |
| DCbiasCtrl         | DCbiasCtrl.cpp       | 10 ms    | DC bias control loop |
| DCBswitch          | DCBswitch.cpp        | 10 ms    | DC bias switching |
| DCBiasCur          | DCBcurrent.cpp       | 10 ms    | DC bias current monitoring |
| RFdriver           | RFdriver.cpp         | 10 ms    | RF driver frequency/amplitude control |
| RFamp              | RFamp.cpp            | 10 ms    | RF amplifier control |
| Twave              | Twave.cpp            | 10 ms    | Travelling wave ion guide |
| FAIMS              | FAIMS.cpp            | 10 ms    | Field asymmetric IMS |
| FAIMSfb            | FAIMSfb.cpp          | 10 ms    | FAIMS feedback control |
| HOFAIMS            | HOFAIMS.cpp          | 10 ms    | High-order FAIMS |
| HVPS               | HVPS.cpp             | 10 ms    | High voltage power supply |
| HVPSinterface      | HVPSinterface.cpp    | 10 ms    | HVPS external interface |
| ESI                | ESI.cpp              | 10 ms    | Electrospray ionisation |
| Filament           | Filament.cpp         | 10 ms    | Filament/ion source control |
| ARB                | ARB.cpp              | 10 ms    | Arbitrary waveform board |
| Analog             | Analog.cpp           | 10 ms    | Analog I/O module |
| DAC                | DAC.cpp              | 10 ms    | External DAC module |
| DIO                | DIO.cpp              | 10 ms    | Digital I/O and trigger |
| FPGA               | FPGA.cpp             | 10 ms    | FPGA interface |
| DMSDMSMB           | DMSDMSMB.cpp         | 10 ms    | DMS/DMS motherboard |
| WiFi               | WiFi.cpp             | 100 ms   | WiFi status polling |
| DisplayDismiss     | Menu.cpp             | —        | Auto-dismiss timed display messages |

---

## Module Discovery

On boot, `ScanHardware()` iterates over the TWI module EEPROM addresses defined in `Variants.cpp`:

```
ModuleAddresses[] = { 0x50, 0x60, 0x62, 0x52, 0x54, 0x56 }
```

For each address the system attempts to read 100 bytes from the EEPROM. The data layout is:

| Offset | Size | Field |
|--------|------|-------|
| 0      | 2    | Size (int16) |
| 2      | 20   | Module name string |
| 22     | 1    | Hardware revision |

The `Name` string is matched against known module names (e.g. `"DCbias"`, `"RFdriver"`, `"Twave"`) and the corresponding `_init()` function is called. Each init function:
- Reads and validates its full data structure from EEPROM
- Adds a menu entry to the main menu
- Registers its thread with `control`

The board select line (`BRDSEL`, pin 47) allows two physical boards to share the same address space (board A / board B), doubling the number of addressable modules. Boards A and B are selected with `ENA_BRD_A` / `ENA_BRD_B` macros.

Valid module EEPROM addresses (per board):

| Hex address | Slot |
|-------------|------|
| 0x50        | 1 |
| 0x52        | 2 |
| 0x54        | 3 |
| 0x56        | 4 |
| 0x60        | 5 (extended) |
| 0x62        | 6 (extended) |

---

## Serial Command Routing

MIPS supports simultaneous serial command input from multiple sources. The active source is tracked by the global `serial` pointer (type `Stream*`).

**Input sources:**
- `SerialUSB` — native USB (default at startup)
- `Serial1` — hardware UART (used by Ethernet adapter or WiFi module)
- `enet_sb` (SerialBuffer) — TWI-bridged Ethernet interface
- WiFi via `Serial1` or `Serial` depending on `wifidata.SerialPort`

**Character path:**

```
ISR / polling            Ring buffer         Command processor
──────────────────────   ─────────────────   ──────────────────────
ReadAllSerial()     →    PutCh(ch)      →    ProcessSerial()
ProcessEthernet()                            TokeniseCommand()
                                             Table lookup → handler
```

`ReadAllSerial()` is called from `MIPSsystemLoop` every 10 ms. It drains `SerialUSB` and `Serial` (native + programming port) into the ring buffer.

`ProcessEthernet()` similarly drains `Serial1` or the TWI Ethernet buffer, setting `serial` to the appropriate stream before calling `PutCh()`.

The command table lives in `src/Serial.cpp`. Each entry is a `Commands` struct:

```cpp
{ "CMDNAME", CMDtype, numArgs, (char *)handlerFunctionPtr }
```

The command processor tokenises the input line on commas, matches the verb, casts the function pointer to the appropriate signature based on `CMDtype`, and calls it. Module-specific commands for DCbias, RFdriver, ESI, ARB, DAC, Filament, Twave, and Table all currently live in this central table (a refactoring TODO moves them to their respective modules).

---

## Communication Interfaces

### Native USB (`SerialUSB`)
Default host interface. 480 Mbps USB 2.0 via the SAM3X8E's native USB peripheral. Identified as "MIPS, native USB" in device descriptors.

### Serial1 (UART)
Used for either:
- **Ethernet adapter** (USR-TCP232-T V2) — direct RS-232 to Ethernet bridge at 115200 baud
- **WiFi module** (Adafruit ESP8266 Hazzah) — serial at 115200 baud running MIPSnet firmware
- **General serial I/O** — if `MIPSconfigData.Ser1ena` is set

### TWI-bridged Ethernet (`EnetUseTWI`)
An Adafruit Trinket M0 sits between MIPS Wire1 (TWI address `0x42`) and the Ethernet adapter's RS-232. Used to work around a hardware design issue on some MIPS revisions. Enabled by `MIPSconfigData.EnetUseTWI`.

### WiFi
The ESP8266 Hazzah runs the MIPSnet firmware and acts as a WiFi-to-serial bridge. MIPS communicates with it over Serial1 (or Serial) using a text command protocol (`HOST,`, `SSID,`, `PSWD,`, `CONNECT`, `STATUS`, `IP`, `REPEAT`, etc.). Settings are stored in `wifi.cfg` on the SD card.

---

## TWI (I2C) Bus Architecture

MIPS uses two TWI buses:

**Wire (primary bus)** — module communication
- Scans EEPROM addresses at startup for module discovery
- All ADC/DAC chip reads and writes on module boards
- TWI addresses 0x50–0x62 for module EEPROMs
- Various chip addresses (AD7998, AD5625, AD5593, MCP2300, etc.) per module board

**Wire1 (secondary bus)** — auxiliary devices
- TWI-bridged Ethernet interface: `0x42` (`TWI_ENET_ADD`)
- ARB on-module Arduino: `0x40` (`TWI_ARB_ADD`)
- Level detection module (address configured at runtime, `LevelDetAdd`)
- Level detector commands: `TWI_LEVDET_*` constants in `Hardware.h`

**TWI helper layer** (`TWIext.cpp`):

The `TWIext` module provides:
- `AcquireTWI()` / `ReleaseTWI()` — bus mutex (the `TWIbusy` flag prevents re-entrancy from ISR context)
- `TWIqueue()` — defers TWI operations that arrive during a bus-busy period (queue depth 10)
- `TWIset*()` / `TWIread*()` — typed helpers for bool, byte, word, 16/24/32-bit int, and float
- `TWIreadBlock()` — bulk read helper
- Software bit-bang fallback: `TWI_START`, `TWI_STOP`, `TWI_WRITE`, `TWI_READ` macros using pins 20/21

---

## SPI Bus Architecture

The hardware SPI bus (pins 74/75/76) is shared among:

| Device | CS pin | Notes |
|--------|--------|-------|
| ILI9340 TFT display | pin 4 | 320×240 colour, SPI_CS divider 21 |
| microSD card | pin 10 (`_sdcs`) | FAT filesystem via SD library |
| General SPI (`SPI_CS`) | pin 52 | DAC/ADC chips on module boards (AD5668, etc.) |

The `LDAC` signal (pin 11) latches DAC outputs simultaneously across all SPI DAC channels. DMA-accelerated SPI transfers are supported (`spiDMAinit`, `spiDmaTX`) using the SAM3X DMAC peripheral.

---

## UI System — Menu and Dialog

**Display:** 320×240 ILI9340 TFT, landscape orientation. Controlled via the `Adafruit_ILI9340` library.

**Input:** Rotary encoder with RGB push button (interrupt-driven via `DIhandler`).

**Two-level UI architecture:**

```
MainMenu  (Menu)
  └─ MenuEntry  ──→  DialogBox  (module settings)
                         └─ DialogBoxEntry[]  (individual fields)
```

`Menu` (`src/Menu.cpp`) renders a scrollable list of `MenuEntry` items. Selecting an entry either calls a function or enters a `DialogBox`.

`DialogBox` (`src/Dialog.cpp`) renders a form with typed fields. `DialogBoxEntry` types include:
- `D_INT`, `D_FLOAT` — numeric entry with range checking
- `D_STRING` — text entry
- `D_LIST` — selection from a comma-delimited string list
- `D_FUNCTION` — button that calls a function
- `D_MENU` / `D_PAGE` — navigate to another menu or dialog page

`ActiveDialog` (global `DialogBox*`) tracks what is currently displayed. A value of `(DialogBox*)(-1)` is used as a sentinel during startup to suppress module displays.

The `p()` function in `Menu.cpp` is a `printf`-style helper that writes to the TFT display at the current cursor position.

---

## Configuration Persistence

**On-module EEPROM** — each plug-in module has a serial EEPROM (I2C, typically AT24C32 or similar). The first 100 bytes store the module's identity (size, name, rev). The full module data structure is stored starting at address 0. MIPS reads and writes module configs via `ReadEEPROM()` / `WriteEEPROM()`.

**SD card** — system-level configuration and module states are saved to `default.cfg` (binary dump of each module's data structure). Loaded automatically on startup. Format: a sequence of named records, each beginning with the module name and containing its data structure verbatim.

**DueFlashStorage** — a small non-volatile scratch area in the SAM3X8E's flash (`NonVolStorage[1000]`), used for data that must survive without an SD card.

**WiFi settings** — saved separately to `wifi.cfg` on the SD card.

**SAVE / LOAD commands** — the host commands `SAVE` and `RESTORE` call `SAVEparms("default.cfg")` and `LOADparms("default.cfg")` respectively.

---

## Hardware Abstraction Layer

`src/Hardware.cpp` and `include/Hardware.h` provide:

**ADC drivers:**
- `AD7998(addr, chan)` — 8-channel, 12-bit I2C ADC (Analog Devices)
- `AD7994(addr, chan)` — 4-channel, 12-bit I2C ADC
- `AD5593readADC(addr, chan)` — mixed-signal I/O expander ADC mode
- `ADS7828(wire, addr, chan)` — 8-channel I2C ADC (Texas Instruments)

**DAC drivers:**
- `AD5625(addr, chan, val)` — 4-channel, 12-bit I2C DAC
- `AD5668(spiAdr, chan, val)` — 8-channel, 16-bit SPI DAC (with DMA support)
- `AD5593writeDAC(addr, chan, val)` — mixed-signal I/O expander DAC mode
- `AD5629write(addr, val)` — 8-channel I2C DAC
- `MCP4725(addr, cmd, val)` — single-channel I2C DAC

**Digital I/O:**
- `MCP2300(addr, reg, data)` — I2C GPIO expander
- `DigitalOut(MSB, LSB)` — shift register output (2 × 8-bit)
- `DigitalIn()` — reads 8 digital inputs

**Calibration framework:**
- `ADCchan` / `DACchan` structs hold `m` (slope) and `b` (offset) calibration coefficients
- `Counts2Value()` / `Value2Counts()` — apply linear calibration
- `ChannelCalibrate()` — interactive TFT-guided calibration
- `ChannelCalibrateSerial()` — serial-command-driven calibration
- `PWLcalibration` / `PWLlookup()` — piecewise-linear calibration for non-linear channels

**Board selection:**
- `SelectBoard(board)` / `SetAddress(addr)` drive the address lines (ADDR0–ADDR2, pins 25–27) and board select (BRDSEL, pin 47) to address the correct module slot

**DMA:**
- `spiDMAinit()`, `spiDmaTX()`, `spiDmaWait()` — SAM3X DMAC channel 0/1 for high-speed SPI DAC output

---

## Feature Variant System

`include/Variants.h` contains compile-time feature flags. Setting a flag `true` includes the corresponding module code. Each enabled flag appends a letter to the firmware version string.

| Flag | Letter | Module |
|------|--------|--------|
| `FAIMSFBcode` | `b` | FAIMS feedback controller |
| `FAIMScode` | `f` | FAIMS (field asymmetric IMS) |
| `HOFAIMcode` | `h` | High-order FAIMS |
| `HVPScode` | `v` | High voltage power supply |
| `DMSDMSMB` | `d` | DMS/DMS motherboard |
| `DCBanalog` | `a` | DC bias analog board |
| `DCBcurrent` | `c` | DC bias current monitoring |
| `DCBswitchCode` | `s` | DC bias switching |
| `HVPSinterface` | `i` | HVPS external interface |

The pre-build script `print_variant.py` reads `Variants.h` at build time and prints the active variant to the build log on every build (not just when `MIPS.cpp` is recompiled).

---

## Timer Assignments

The SAM3X8E has 9 independent timers. MIPS assigns them as follows:

| Define | Timer | Module | Purpose |
|--------|-------|--------|---------|
| `TMR_TrigOut` | 3 | DIO | Frequency source on trigger output |
| `TMR_TwaveClk` | 2 | Twave | Twave clock in compressor mode |
| `TMR_TwaveCmp` | 4 | Twave | Multi-pass compressor table |
| `TMR_Table` | 8 | Table | Pulse sequence table execution (hardware requirement) |
| `TMR_Profiles` | Timer5 | DCbias | Profile toggling |
| `TMR_DCbiasPulse` | 5 | DCbias | Single DC bias channel pulse |
| `TMR_servos` | 4 | HOFAIMS | Servo drive (shared with TwaveCmp) |
| `TMR_ARBclock` | 6 | ARB | Common ARB clock; also used by FAIMSFB scan clock |
| `TMR_DelayedTrigger` | 0 | DIO | Delayed trigger generation |
| `TMR_ADCclock` | 1 | Hardware | ADC digitizer |
| `TMR_RampClock` | 2 | Table | Voltage ramps in table mode |

> **Note:** Timer 6 output is the same pin as RF driver channel 1 power control (board address 0 / jumper A). Do not use ARB or FAIMSFB scan clock simultaneously with an RF driver at address A.

---

## Module Reference

Brief description of each source module:

| Module | Source file | Description |
|--------|-------------|-------------|
| MIPS core | MIPS.cpp | Boot, main loop, system commands, module scan |
| Serial | Serial.cpp | Ring buffer input, command table, command dispatch |
| Hardware | Hardware.cpp | ADC/DAC drivers, calibration, board select, DMA |
| Menu | Menu.cpp | TFT menu rendering, encoder input, display helpers |
| Dialog | Dialog.cpp | TFT dialog box rendering, field editing |
| DIO | DIO.cpp | Digital I/O, trigger output, input change detection |
| DCbias | DCbias.cpp | Multi-channel DC voltage supply control |
| DCbiasList | DCbiasList.cpp | DC bias channel list management |
| DCbiasCtrl | DCbiasCtrl.cpp | DC bias feedback/control loops |
| DCBcurrent | DCBcurrent.cpp | DC bias board current monitoring |
| DCBswitch | DCBswitch.cpp | DC bias high-voltage switching |
| RFdriver | RFdriver.cpp | RF drive (frequency, amplitude, phase) |
| RFamp | RFamp.cpp | RF amplifier module |
| Twave | Twave.cpp | Travelling wave ion guide control |
| Table | Table.cpp | Pulse sequence table execution engine |
| FAIMS | FAIMS.cpp | Field asymmetric ion mobility spectrometry |
| FAIMSfb | FAIMSfb.cpp | FAIMS closed-loop feedback (optional) |
| HOFAIMS | HOFAIMS.cpp | High-order FAIMS with servo control |
| ESI | ESI.cpp | Electrospray ionisation source |
| Filament | Filament.cpp | Filament/ion source emission control |
| ARB | ARB.cpp | Arbitrary waveform board + compressor |
| ARBcompressor | ARBcompressor.cpp | ARB ion compressor timing |
| Compressor | Compressor.cpp | General compressor/trap timing |
| HVPS | HVPS.cpp | High voltage power supply (optional) |
| HVPSinterface | HVPSinterface.cpp | External HVPS interface (optional) |
| DMSDMSMB | DMSDMSMB.cpp | DMS motherboard interface (optional) |
| Analog | Analog.cpp | Generic analog I/O module |
| DAC | DAC.cpp | External DAC module |
| FPGA | FPGA.cpp | FPGA configuration and communication |
| Ethernet | ethernet.cpp | USR-TCP232 Ethernet adapter driver |
| WiFi | WiFi.cpp | ESP8266 WiFi module driver |
| TWIext | TWIext.cpp | TWI bus helpers, queuing, typed read/write |
| FILEIO | FILEIO.cpp | SD card file I/O, BMP image loader, config save/load |
| Log | Log.cpp | System event logging |
| Encoder | Encoder.cpp | Rotary encoder interrupt handling |
| SC16IS740 | SC16IS740.cpp | SPI/I2C UART bridge chip driver |
| ClockGenerator | ClockGenerator.cpp | Programmable clock generator (FS7140) |
| ADCdrv | ADCdrv.cpp | High-speed ADC digitizer |
| Variants | Variants.cpp | Default data structures for all module types |
