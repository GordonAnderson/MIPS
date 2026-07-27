# Third-Party Licenses — MIPS Firmware

The MIPS firmware links a number of third-party libraries, each under its own
license. This file lists them and their licenses, as required for attribution.
The MIPS firmware itself is licensed under the MIT License — see `LICENSE`.

Last reviewed: <DATE>

| Library | License | SPDX | Source of license |
|---|---|---|---|
| Adafruit_ADS1X15 | BSD | `BSD-3-Clause` | license.txt |
| Adafruit_BMP085_U | BSD | `BSD-3-Clause` | header comment |
| Adafruit_BusIO | MIT | `MIT` | LICENSE |
| Adafruit_GFX_Library | BSD | `BSD-3-Clause` | license.txt |
| Adafruit_ILI9340 | MIT | `MIT` | header comment |
| Adafruit_Sensor | Apache 2.0 | `Apache-2.0` | header comment |
| Adafruit_Servo | BSD | `BSD-3-Clause` | header comment |
| SdFat (Adafruit fork) | MIT | `MIT` | LICENSE — replaces the GPL `SD` wrapper |
| Seeed_Arduino_RTC | MIT | `MIT` | LICENSE |
| softRTC | MIT | `MIT` | LICENSE |
| DueFlashStorage | MIT | `MIT` | licence (MIT).txt |
| MIPStimer | Public domain | — | based on DueTimer (public domain); MIPS additions by Gordon Anderson |
| ArduinoThread | MIT | `MIT` | Ivan Seidel — upstream MIT (see note) |
| DIhandler | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |
| SerialBuffer | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |
| WireServer | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |

## Notes

**SdFat replaces the GPL `SD` library.** MIPS previously linked the standalone
`SD` library (GPLv3, a wrapper over sdfatlib). It has been replaced with the
Adafruit `SdFat` fork (MIT), which wraps the same underlying sdfatlib under a
permissive license. This keeps the combined MIPS firmware MIT-licensable.

**ArduinoThread** carried no license text in the bundled copy. The upstream
project (Ivan Seidel) is MIT-licensed; the upstream LICENSE has been added to
the local copy. Confirm the local copy matches the upstream MIT project before
relying on this.

**MIPStimer** derives from DueTimer, which was released into the public domain.
The MIPS-specific additions are by Gordon Anderson and are contributed under the
same terms as the MIPS firmware (MIT).

**In-house libraries** (`DIhandler`, `SerialBuffer`, `WireServer`) are authored
by GAA Custom Electronics, LLC and are licensed under MIT as part of this
project. An MIT header has been added to each.

## Compatibility summary

All libraries above are permissive (MIT, BSD, Apache 2.0) or public domain, and
are compatible with distributing the combined MIPS firmware under the MIT
License — with one item to close: confirm the ArduinoThread local copy matches
upstream MIT.
