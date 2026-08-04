# Phase 0 — Bug fixes and prerequisite refactors

**Patch:** `phase0.patch` (apply with `git apply phase0.patch` from the repo root)
**Branch used:** `phase0` off current `main`
**Build status:** 41/41 objects compile, links to a valid image
**Size:** text 308,148 (baseline 307,916, **+232 bytes**), bss 25,168 (unchanged)

---

## What is in this commit

| ID | Fix | Files |
|---|---|---|
| — | Case-sensitive include fixes (**11**, not 4) | 9 files |
| B1 | `RFA_init` no longer arms the four DAC CS enables persistently | RFamp.cpp |
| B2 | `setRFArev3` HV− calibration written to `[3]` not `[2]` | RFamp.cpp |
| B3 | Stray `serial->println(cnts)` removed from `setRFADAC` | RFamp.cpp |
| B4 | `setRFADAC` AUX1/AUX2 routed to AD5625 instead of the CPLD CS path | RFamp.cpp |
| B5 | 18-bit DAC count clamped to 0..262143 inside `Set_18bitDAC` | RFamp.cpp |
| B6 | `Set_18bitDAC` wrapped in `AtomicBlock`; SPI mode set every call | RFamp.cpp |
| B7 | `SelectBoard` added to the rev 3 pole path (via T1-1) | RFamp.cpp |
| B8 | Missing `else` in the low-range `RFVNpp` filter | RFamp.cpp |
| B9 | `calRFApoleSupplies` restores `PWR_ON` and forces a pole resync | RFamp.cpp |
| B10 | `RestoreRFAsettings` clamps `Size` read from EEPROM | RFamp.cpp |
| B11 | Explicit CPLD initialisation write at the end of `RFA_init` | RFamp.cpp |
| B13 | Dead `.m`/`.b` assignments removed from `RFampCalibrate` | RFamp.cpp |
| B14 | `GRFAVPPB` registered alongside the legacy `GFRAVPPB` | RFamp.cpp |
| B15 | `SerialMute` honoured in four getters | RFamp.cpp |
| T0-1 | `RFAgainComp()` extracted from three identical copies | RFamp.cpp |
| T0-3 | `RFAdac18BITMAX` and `RFAadcLR_BASE` named constants | RFamp.h, RFamp.cpp |
| T0-4 | (same as B13) | RFamp.cpp |
| T1-1 | `RFAupdatePoles()` unifies the rev 3 pole write | RFamp.cpp |
| — | `ADCfindSum` returns `false` on timeout, not `-1` | ADCdrv.cpp |
| — | `ADCacquireWait` timeout expression corrected + watchdog petting | ADCdrv.cpp |

**Deliberately excluded:** T0-2 and T1-2 (local-pointer and command-preamble collapses) — pure
diff noise that would bury the substantive changes; T1-2 is its own commit per the plan.
**B12** (`quadAutoTune` double `NumDown++`) left unfixed pending your bench decision.

---

## The case-sensitivity problem was bigger than reported

I originally found 4 mismatched includes. With the compatibility shim removed, there were
**11 across 9 files**:

| File | Was | Now |
|---|---|---|
| src/RFamp.cpp | `"RFAMP.h"` | `"RFamp.h"` |
| src/Analog.cpp | `"wire.h"`, `"dialog.h"` | `"Wire.h"`, `"Dialog.h"` |
| src/Filament.cpp | `"wire.h"`, `"dialog.h"` | `"Wire.h"`, `"Dialog.h"` |
| src/MIPS.cpp | `"utility/Sd2card.h"` | `"utility/Sd2Card.h"` |
| src/Serial.cpp | `"hardware.h"`, `"arb.h"` | `"Hardware.h"`, `"ARB.h"` |
| include/Variants.h | `"errors.h"` | `"Errors.h"` |
| include/DCbiasCtrl.h | `"DCBias.h"` | `"DCbias.h"` |
| include/DMSDMSMB.h | `"adcdrv.h"` | `"ADCdrv.h"` |
| include/SC16IS740.h | `<WIRE.h>` | `<Wire.h>` |

The tree now compiles on a case-sensitive filesystem with no shims. This is a prerequisite
for any Linux CI.

---

## Verification notes for the bench

Ordered by how likely each is to reveal a problem.

1. **Pole voltages, rev 3 module.** T1-1 is the largest behavioural change. Check P1 and P2
   across: enable/disable, resolving DC on/off, and a range of PoleBias and ResolvingDC
   settings including one that would previously have exceeded the DAC range (sum near
   ±600 V) — that should now clamp rather than power the DAC down.
2. **`RFAACQ` host scan.** Should produce an identical spectrum to before. This is the
   acceptance test for T1-1 and T0-1 together.
3. **Range switching.** B8 changes the low-range readback filter. Watch RF− Vp-p settle after
   a range change and after an auto-tune.
4. **Auto-tune.** Exercises B8 via the −1 sentinel path.
5. **Front panel and dialogs.** T0-3 touched every `ADCchansLR` index; a wrong index would
   show as a badly scaled low-range reading.
6. **Cold start.** B11 adds a CPLD write at the end of `RFA_init`. Confirm the module comes up
   correctly on a power cycle with `NormalStartup` both true and false.
7. **`calRFApoleSupplies`.** B9 now restores `PWR_ON`. Confirm the supply state after a
   calibration run matches what it was before.

---

## Things I want to flag

**One error I introduced and caught.** The T0-1 extraction initially left three dangling
`else gc = 1.0;` clauses because the original blocks had an `else` I had not seen in my
earlier reading. The compiler caught it immediately. Mentioned because it is a fair sample of
the error rate you should expect, and of why the build loop matters.

**`ADCacquireWait` now uses `int64_t` for the intermediate.** `ADCnumsamples * 20000` overflows
a 32-bit int above 107,374 samples, and the module comment advertises 100K point vectors. The
old expression could not overflow because the operands were the other way round, so this is a
new hazard introduced by the fix rather than a pre-existing one.

**`Set_18bitDAC` invalid-channel behaviour changed.** It now clears the enable and returns
without clocking data if `DACchannel` is not one of the four CS bits. Previously it would clock
three bytes with no CS asserted. With the B4 guard this should be unreachable, but it is a
behaviour change worth knowing about.

**`AtomicBlock` nesting.** `Set_18bitDAC` now holds one, and it calls `SendFRAcpldCommand`
which holds its own. `Atomic_RestoreState` saves and restores prior state, so nesting is safe —
but this means interrupts are now disabled for the whole DAC transaction rather than just the
CPLD writes. At 7 MHz SPI that is roughly 10–15 µs. If anything in MIPS is sensitive to that,
it would show up as timing jitter elsewhere.

**Not verified by me:** everything hardware-dependent. Also the two carried-over scope items —
the plain-vs-extended SPI API question and the CPLD clock edge — are untouched by this patch
and still want confirmation.

---

## Build reproduction

```
arm-none-eabi-g++ 13.2 (Ubuntu), Arduino SAM core from github.com/arduino/ArduinoCore-sam
41 app objects + 28 core/variant + 35 library objects -> flash.ld -> mips.elf
```

This is **not** a PlatformIO build. Flags are reconstructed, the three `extra_scripts` do not
run, and the compiler is newer than the `atmelsam` platform's. Please confirm a clean
PlatformIO build before treating this phase as done.
