# MIPS filament module firmware — handoff

State of `src/Filament.cpp` after the October 2026 review and the first set of fixes. Written
October 17, 2026. Companion documents: [`MIPS_FILAMENT_TESTPLAN.md`](MIPS_FILAMENT_TESTPLAN.md) (lab
roadmap for the fixes below) and [`MIPS_QUAD_FIRMWARE_HANDOFF.md`](MIPS_QUAD_FIRMWARE_HANDOFF.md) (same
conventions, QUAD and TWI work).

**Repo:** https://github.com/GordonAnderson/MIPS, branch `master`.
**Fix commit:** `5dc8f71`. **Firmware version string is still `1.268`**, so builds with and without the
fixes look the same; identify a build with `git log -1`.

**Nothing in section 3 has run on hardware.** It builds clean under PlatformIO and was reviewed by reading.

---

## 1. What the module is

`src/Filament.cpp` (about 1620 lines) drives the filament control board: a current-regulated heater supply
used for ion source filaments. Header: [`include/Filament.h`](include/Filament.h). Defaults:
`FILAMENT_Rev_1` and `FILAMENT_Rev_4` in [`src/Variants.cpp`](src/Variants.cpp).

### Hardware revisions

| Rev | Hardware | Channels per board | Notes |
|---|---|---|---|
| 1 | Original module, AD5625 DAC + AD7998 ADC | 2 independent | Up to 2 boards, so 4 channels |
| 2 | Current reversible | 1 live, 2 structures | Channels 1 and 2 tied together in reverse; only one driver on at a time; `Idir` selects the live structure; direction changes only when disabled |
| 3 | Rev 2 plus on-module emission current | As Rev 2 | PCB rev 4.0 |
| 4 | Single board e-msion, AD5593 analog and digital IO | 1 | Enable is an AD5593 digital output; no direction control |

All boards in a system must be the same revision (`BoardFromSelectedFilamentChannel` reads the revision
from whichever board is present).

### Data

One `FilamentData` per board in `FDarray[2]`, 256 bytes, saved to the module EEPROM at offset 0
(address 0x52). The macro `FD` is `FDarray[SelectedFilamentBoard]`. Offsets (little endian, from the
build): `Size` 0, `Name` 2, `Rev` 22, `NumChannels` 23, `FCD[2]` 24 (88 bytes each), `ADCadr` 200,
`DACadr` 201, `EEPROMadr` 202, `FCyl[2]` 204, `iSense` 236, `Idir` 240, `Ecurrent` 244.
**A new field must be appended at the end**; see the restore logic in section 4.

`FCD` and `FCY` are the **dialog copies** of the selected channel. Editing the dialog changes them, and
`Filament_loop` copies them back into `FD` when `ActiveDialog->Changed` is set. Serial commands change `FD`
directly and must also update `FCD`/`FCY` when the addressed channel is the selected one, or the next dialog
edit writes the stale value back. This pattern caused several of the bugs below.

### Index conventions (a frequent source of mistakes)

| Quantity | Base | Meaning |
|---|---|---|
| Serial channel numbers | 1 | `SFLI,1,...` |
| `SelectedFilamentChan`, `Fchannel - 1` | 0 | Selected channel |
| Board | 0 | 0 = A, 1 = B |
| Channel index into `FCD[]`, `FCyl[]`, `Fcurrents[b][c]` | 0 | From `Channel2Index()` |

`Channel2Index(ch)`: Rev 1 returns `ch & 1`. Rev 2+ returns 0 when `Idir` is true (FWD) and 1 when false (REV),
so the live structure moves when the direction changes. `BoardFromSelectedFilamentChannel()` and
`Channel2Index()` return -1 on error and most callers do not check it.

### Run loop

`Filament_loop` runs every 100 mS as a thread:
1. Copy dialog edits back to `FD` if the dialog changed.
2. `FilamentCyclying()`, the point 1 / point 2 cycling state machine.
3. Serial watchdog (`SFLSWD`): if enabled and nothing has talked for 10 s, all channels are disabled.
4. Bias current from DCbias channel 8 if `iSense` is non-zero.
5. For each board and channel: apply the power enable, ramp the current setpoint and the filament voltage,
   write the DACs, read the ADC and filter the readbacks.
6. `FilamentControl()` every 11th pass: maximum power limit and, in `IVctrl` mode, supply voltage tracking.

The ramp assumes this loop runs 10 times per second: `StepSize = RampRate / 10`.
**Turn-off is also ramp limited**: disabling ramps the current setpoint to zero, and only then does the enable
pin go off. At the default 0.1 A/s that is up to 50 s from 5 A.

### Serial commands (`src/Serial.cpp`, "Filament" block)

`GFLENA`/`SFLENA`, `GFLI`/`SFLI`, `GFLAI`, `GFLSV`/`SFLSV`, `GFLASV`, `GFLV`, `GFLPWR`, `GFLRT`/`SFLRT`,
`GFLDIR`/`SFLDIR` (Rev 2/3 only), cycling `GFLP1`/`SFLP1`, `GFLP2`/`SFLP2`, `GFLCY`/`SFLCY`,
`GFLENAR`/`SFLENAR`, `RFLPARMS` (periodic reporting), `GFLSRES`/`SFLSRES` (sense resistor, board 0),
`GFLECUR` (bias or emission current), `SFLSWD` (serial watchdog). Channels are 1 based.
`SFLENAR`'s help text says ON/OFF but the code takes `YES`/`NO` and `GFLENAR` returns `NO` or the cycles
remaining (open item).

---

## 2. Review findings

Twelve problems were found by reading the whole file (October 17, 2026). Nothing was reproduced on a bench.
Status of each:

| # | Finding | Status |
|---|---|---|
| 1 | Ramp rate 0 froze the setpoint; a running filament could not be turned off | **Fixed** |
| 2 | `FilamentShutdown` wrote AD5625 commands to a Rev 4, ignored absent boards | **Fixed** (see severity note) |
| 3 | Enable output driven ON at startup until the first loop pass | **Fixed** |
| 4 | NULL dereference in the bias current block with `iSense` set and no DCbias module | Open |
| 5 | `FmodeChange` wrote both channels' mode on every revision | **Fixed** |
| 6 | `SetFilamentStatus` off by one | **Fixed** |
| 7 | Rev 2+ shared parameters not kept in sync | **Fixed** |
| 8 | `RestorFilamentSettings` ignored the stored structure size | **Fixed** |
| 9 | Emission current display mixes all channels and boards | Open |
| 10 | Reporting threads use fixed channel indices | Open |
| 11 | `CalibrateFilament` unchecked pointer, bypassed ramp, saves transient state | Open |
| 12 | Smaller items, listed in section 5 | Open |

**Severity note on finding 2.** `FilamentShutdown` runs from the power-off handler in `MIPS.cpp`, which is
entered when `ReadVin() < 10 V` with the controller alive on USB. The modules lose their supply at the same
time, so the practical effect is a defined off state while the module supply decays, and not sending
AD5625 traffic to an AD5593. It is less critical than it first looked.

---

## 3. What was changed (commit `5dc8f71`)

All in `src/Filament.cpp`.

1. **`MinRampRate` = 0.01 A/s.** The control loop uses `max(RampRate, MinRampRate)` in both ramp directions.
   The page 2 "Ramp rate" dialog entry has this as its minimum and `SFLRT` NAKs anything below it. A stored
   rate of 0 in an old EEPROM image is tolerated by the floor.
2. **`FilamentShutdown`.** Loops over boards that are present. Rev 4: `AD5593write(ADCadr, 9, 1 << Fpwr)` for
   the enable and `AD5593writeDAC(...)` zero for the supply voltage and current channels. Other revs: the
   existing `digitalWrite(Fpwr, HIGH)` and AD5625 zero writes, now for both channels of each present board.
3. **Startup.** Rev 1–3: `digitalWrite(Fpwr, HIGH)` before `pinMode(OUTPUT)`. Rev 4: `FilamentAD5593init`
   writes register 9 = 1 before the digital output is configured.
4. **`FmodeChange`.** `Rev >= 2` copies the mode to both structures; Rev 1 writes only the selected channel.
5. **`SetFilamentStatus`.** `(c - 1) == SelectedFilamentChan`.
6. **`FilamentSyncChannels(b, from)`** copies `CurrentSetpoint`, `FilamentVoltage`, `RampRate`, `Mode`,
   `MaxPower` to the other structure. Called from `SetFilamentCurrent`, `SetFilamentSupplyVoltage`, and on a
   direction change from both `CurrentDirChange` (dialog) and `SetCurrentDirection` (serial), with `from` set
   to the index that was live **before** the direction changed.
7. **`RestorFilamentSettings`.** Reads `sizeof(FilamentData)` from the EEPROM, terminates `Name`, requires
   the name to match, requires stored `Size >= offsetof(FilamentData, FCyl)` (204), `Rev` 1 to 4 and
   `NumChannels` 1 to 2, copies `min(Size, sizeof)` bytes over a copy of the **current** `FD`, then sets
   `Size = sizeof` and keeps `EEPROMadr`.

---

## 4. Design decisions and why

**The rate floor, not "0 means unlimited".** A rate of 0 could mean "no ramp limit", but that would let a
disable or enable slam the supply. A small floor keeps behaviour continuous and cannot freeze. 0.01 A/s is
the dialog step size. The side effect is that the lowest rate is slow to turn off; the watchdog path has the
same ramp (open item, section 5).

**Preload the output before making it an output.** On the Due, `pinMode(OUTPUT)` drives whatever the output
data latch holds, and after reset that is 0, which is the active (on) state of an active-low enable. Writing
HIGH first sets the latch while the pin is still an input. The AD5593 has the same issue with its
digital output register (register 9, which powers up as 0).

**`FilamentSyncChannels` keeps all Rev 2+ shared fields equal instead of patching the serial setters one at
a time.** The invariant is "both structures hold the same shared parameters, `Idir` only chooses which is
live". Anything that changes a shared parameter or `Idir` must preserve it, and a helper is harder to forget
than a hand-written pair of assignments. It deliberately does **not** copy `FilamentPwr` (a direction change
requires the filament to be off) or the hardware fields.

**Restore overlays the stored image on the current structure.** Fields past the stored `Size` keep the
current values, and this is **not** the factory default when a restore is done at run time: if you restore a
short image while the live value is non-default, the live value stays. At power-up the live value is the
compiled default (`FILAMENT_Rev_1`/`_4`), which is what matters. The lower bound on `Size` is 204, through the
TWI addresses, because those fields are needed to talk to the board at all.

**Same append-only rule as the QUAD calibration record.** A field added in the middle of `FilamentData`
silently breaks every saved module. Add at the end and the restore logic handles old images. If the struct
ever needs to grow past 256 bytes, check `WriteEEPROM`'s 16 byte page behaviour (see the QUAD handoff section
4: unaligned or wrapping writes corrupt silently).

---

## 5. Open items

| Item | Notes |
|---|---|
| **Bench test of everything in section 3** | The test plan. Highest priority; none of it has run on hardware. |
| Finding 4: bias current NULL dereference | `if(DCbDarray[0] != NULL) b = 0; else b = 1;` then `DCbDarray[b]->ADCadr` with neither present crashes. Also uses board 0's `iSense` even when only a slot 1 board exists. |
| Finding 9: emission current mixing | `BiasCurrent` is filtered from every channel and board in the loop, not the selected one. Two boards of Rev 3/4 blend. |
| Finding 10: reporting threads | `Filament_1_Report`/`_2_Report` read `[b][0]`/`[b][1]`. For Rev 2+ the live index depends on `Idir`, so with REV selected they report the idle channel. Use `Channel2Index`. |
| Finding 11: `CalibrateFilament` | `DCbDarray[DCbiasCH2Brd(7)]` used unchecked; runs `Filament_loop()` 100 times back to back so the ramp is bypassed; leaves `FilamentVoltage` at 3.0 and mode `FmodeIV` then saves them. The serial calibration helpers use `SelectBoard(0)` while `FD` follows the selected board. |
| Turn-off is ramp limited | Disable and the serial watchdog timeout both ramp down at the configured rate. For a safety shutdown consider forcing power off when the watchdog fires, or ramping faster for a disable. Not changed. |
| No guard for -1 | `Channel2Index` / `BoardFromSelectedFilamentChannel` can return -1 and callers index arrays with it. No reachable path found today because `NumberOfFilamentChannels` limits the channel, but a second `Filament_init` for the same board would double it. |
| IV mode constants | The dead band is centred on 0.8 V (`abs(Fvoltages - 0.8) > 0.1`) but the correction aims at 0.75 V. Probably harmless; confirm the intended set point. |
| Power limit and cycling | The maximum power limit lowers `CurrentSetpoint` by 0.1 A, so cycling that waits for `CurrentSetpoints == FCpoint2` can stall forever (float equality). |
| `GFLENAR` with infinite cycles | `FCnumCyl` 0 means infinite but the status prints a negative number. |
| `SFLENAR` help text | Says ON/OFF; the code accepts `YES`/`NO`. Fix the comment in the command table (it generates `MIPScommands.txt`). |
| Version not bumped | Bump before shipping so builds are distinguishable. |
| Struct versioning | There is no version field, only `Size`. Adding one would make a future change safer than inferring from size. |

---

## 6. Working conventions that worked

- **Build:** `~/.platformio/penv/bin/pio run` (the `pio` command is not on PATH). A rebuild takes a few
  seconds. `MIPScommands.txt` is regenerated after every build; commit it with the change.
- **macOS has no `timeout`.** Do not wrap a build in it; the command fails without running.
- **Scripted edits need guards.** A first attempt at the fixes corrupted the file: a `str.index()` for the
  end of a block matched the forward declaration near the top, the slice came out empty, and
  `str.replace('', new)` inserted text between every character. The file grew to 2.4 million lines and the
  build ran for about 10 minutes. Anchor on the definition including its opening brace, `assert` that the
  slice is non-empty and a plausible size, and check `wc -l` before building. `git checkout` recovers.
- **Check the build for an unexpectedly long run.** A build of a single changed file takes seconds.
- **Don't trust a symptom to point at the first suspect.** Read the call path (here the power-off handler)
  before deciding how serious a finding is; finding 2 turned out to be less critical than it looked.
- **Test plan and handoff go in the repo** with the code, and the code header or this document records what
  has and has not been run on hardware.
