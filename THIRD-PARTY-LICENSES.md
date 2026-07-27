# Third-Party Licenses — MIPS Firmware

The MIPS firmware links a number of third-party libraries, each under its own
license. This file lists them and their licenses, as required for attribution.
The MIPS firmware itself is licensed under the MIT License — see `LICENSE`.

Last reviewed: <DATE>

> **Status: the SD library swap is NOT yet applied.** `lib/SD/` currently
> contains the original SparkFun/Greiman `SD` library (GPLv3), wrapper and
> bundled sdfatlib both. Until it is replaced with a permissive library, the
> combined MIPS firmware **cannot be distributed under the MIT License.** See
> the SD note below.

| Library | License | SPDX | Source of license |
|---|---|---|---|
| Adafruit_ADS1X15 | BSD | `BSD-3-Clause` | license.txt |
| Adafruit_BMP085_U | BSD | `BSD-3-Clause` | header comment |
| Adafruit_BusIO | MIT | `MIT` | LICENSE |
| Adafruit_GFX_Library | BSD | `BSD-3-Clause` | license.txt |
| Adafruit_ILI9340 | MIT | `MIT` | header comment |
| Adafruit_Sensor | Apache 2.0 | `Apache-2.0` | header comment |
| Adafruit_Servo | BSD | `BSD-3-Clause` | header comment |
| SD (SparkFun / Greiman, incl. bundled sdfatlib) | **GPLv3** | `GPL-3.0-only` | header comments; blocks MIT distribution — see note |
| Seeed_Arduino_RTC | MIT | `MIT` | LICENSE |
| softRTC | MIT | `MIT` | LICENSE |
| DueFlashStorage | MIT | `MIT` | licence (MIT).txt |
| MIPStimer | Public domain | — | based on DueTimer (public domain); MIPS additions by Gordon Anderson |
| ArduinoThread | MIT | `MIT` | Ivan Seidel — upstream MIT (see note) |
| DIhandler | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |
| SerialBuffer | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |
| WireServer | MIT | `MIT` | GAA Custom Electronics, LLC — in-house |

## Notes

**SD is GPLv3 and currently blocks MIT distribution — swap pending.** The
`lib/SD/` library is the original SparkFun/Greiman `SD` stack: a thin SparkFun
convenience wrapper (`SD.h`/`SD.cpp`, GPLv3) over a verbatim copy of William
Greiman's sdfatlib (`utility/SdFile.cpp`, `SdVolume.cpp`, `Sd2Card.cpp`, each
GPLv3). Because GPLv3 is copyleft, distributing the combined MIPS firmware while
this library is linked would require the whole firmware to be GPLv3 — which is
incompatible with the intended MIT license.

Planned resolution: replace `lib/SD/` with the Adafruit `SdFat` fork (MIT), a
modern descendant of the same Greiman sdfatlib, and update the four files that
use SD (`MIPS.ino`, `Analog.ino`, `Filament.ino`, `Serial.cpp`) to SdFat's
SD-compatibility API. Care is needed with file-open modes: Arduino `FILE_WRITE`
opens in append/at-end mode, so it must map to the equivalent SdFat flags rather
than a plain overwrite. This row and the status banner above must be updated
only after the swap compiles and the SD read/write/append paths are verified on
hardware, and `lib/SD/` is removed.

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

Every library above is permissive (MIT, BSD, Apache 2.0) or public domain and
compatible with MIT distribution **except one**: the GPLv3 `SD` library, which
must be swapped for a permissive equivalent before MIPS can be released under
MIT. Two items remain to close:

1. **Replace the GPLv3 `SD` library** with the Adafruit `SdFat` fork (MIT), test
   the SD paths on hardware, and remove `lib/SD/`. *(blocking)*
2. **Confirm the ArduinoThread local copy matches upstream MIT.** *(minor)*

Until item 1 is done, MIPS is GPLv3 by combination and cannot be distributed
under the MIT License.
