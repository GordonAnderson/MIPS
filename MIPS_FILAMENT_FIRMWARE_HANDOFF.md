# MIPS filament module firmware — handoff

State of `src/Filament.cpp` after the October 2026 review and the first set of fixes. Written
October 17, 2026. Companion documents: [`MIPS_FILAMENT_TESTPLAN.md`](MIPS_FILAMENT_TESTPLAN.md) (lab
roadmap for the fixes below) and [`MIPS_QUAD_FIRMWARE_HANDOFF.md`](MIPS_QUAD_FIRMWARE_HANDOFF.md) (same
conventions, QUAD and TWI work).

**Repo:** https://github.com/GordonAnderson/MIPS, branch `master`.
**Fix commits:** `5dc8f71` (first set, section 3) and the commit titled "Filament: guard bias current, fix
reporting channel, emission display and CalibrateFilament" (second set, section 3a), then the commit
titled "Filament: fix Rev 1 channel selection overrun, add index guards and finish the cleanups" (third
set, section 3b), then the commit titled "Move the filament host commands into the filament module"
(section 3c). **Firmware version string is still `1.268`**, so builds with and without the
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
| 4 | NULL dereference in the bias current block with `iSense` set and no DCbias module | **Fixed** (second set) |
| 5 | `FmodeChange` wrote both channels' mode on every revision | **Fixed** |
| 6 | `SetFilamentStatus` off by one | **Fixed** |
| 7 | Rev 2+ shared parameters not kept in sync | **Fixed** |
| 8 | `RestorFilamentSettings` ignored the stored structure size | **Fixed** |
| 9 | Emission current display mixes all channels and boards | **Fixed** (second set) |
| 10 | Reporting threads use fixed channel indices | **Fixed** (second set) |
| 11 | `CalibrateFilament` unchecked pointer, bypassed ramp, saves transient state | **Fixed** (second set), one leftover in section 5 |
| 12 | Smaller items (`SFLENAR` help text, `GFLENAR` for infinite cycles, cycling stall after a power trim, IV constants, unguarded -1 indexes) | **Fixed** (third set) |
| 13 | Found while doing the index guards: `SelectFilamentChannel` indexed `FCD[Fchannel - 1]` for Rev 1, past the 2 element array for channels 3 and 4 (the second board), and the next dialog change wrote that data back over the channel structure | **Fixed** (third set) |

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

### 3a. Second set

All in `src/Filament.cpp`. Build tested only.

1. **Bias current block (finding 4).** The DC bias board is chosen as `DCbDarray[0]`, else `DCbDarray[1]`, else
   none, and the whole read-and-filter step is skipped when there is none. Board 0's `iSense` is still the
   one used, to match the `GFLSRES`/`SFLSRES` commands, which point at `FDarray[0].iSense`.
2. **Reporting threads (finding 10).** The four copies are now wrappers around `FilamentReport(chan)`, which
   takes the board from `BoardFromSelectedFilamentChannel(chan)` and the index from `Channel2Index(chan)`
   and prints nothing if either is -1. Rev 1 output is identical to before.
3. **Emission current display (finding 9).** New `EmissionI[2]`, one filtered value per board, updated once per
   board per pass (`c == 0`). After the channel loops, `BiasCurrent` is set from the selected board's value
   for Rev 3 and 4. The DC bias sense path for other revisions is untouched. The filter is now applied once
   per pass instead of once per channel, so a two channel board responds about half as fast as before.
4. **`CalibrateFilament` (finding 11).** Checks for a DC bias module before turning anything on and uses the
   board it found; the 100-pass waits go through `FilamentCalPasses()`, which calls `Filament_loop()` every
   100 mS with a watchdog restart; the current setpoint and voltage are remembered and
   restored (then `SelectFilamentChannel()` refreshes the dialog copy and mode text) before
   `SaveFilamentSettings()`; the filament is left off. The operating mode is set to `FmodeIV` at the end, not
   restored: IV is the production mode for this module and the intended end point of the procedure (confirmed
   by Gordon, October 17, 2026).

### 3b. Third set

All in `src/Filament.cpp` except the help text in `src/Serial.cpp` (which regenerates `MIPScommands.txt`).
Build tested only.

1. **Rev 1 channel selection overrun (finding 13).** `SelectFilamentChannel` now resolves the board and the
   channel index with `BoardFromSelectedFilamentChannel(Fchannel - 1)` and `Channel2Index(Fchannel - 1)` for
   every revision, and leaves the selection unchanged if either is -1. Before, Rev 1 used `Fchannel - 1`
   directly. On a two-board Rev 1 system, selecting channel 3 or 4 loaded bytes from past the end of
   `FCD[2]`, and the next `Changed` dialog pass wrote them into the channel's pin and DAC/ADC mapping.
   Single-board systems were not affected because the channel entry's maximum is 2.
2. **Index guards.** `SelIdx()` returns `Channel2Index(SelectedFilamentChan)` clamped to 0 when it is -1, and
   replaces the unguarded uses in the loop, cycling, control, calibration and restore code.
   `IsFilamentChannelValid` (used by every serial command) now also requires that the board and the channel
   index both resolve. `Filament_loop` keeps the previous `SelectedFilamentBoard` if the lookup fails, so the
   control loop still runs for the boards that are present.
3. **Cycling stall.** The cycle advances when the ramp reaches the commanded setpoint
   (`CurrentSetpoints == FCD[c].CurrentSetpoint`) instead of when it equals point 1 or 2. The maximum power
   limit can lower the commanded setpoint by 0.1 A, after which the ramp stops below the point and the old
   float equality never became true. A side effect: if the user changes the setpoint during a cycle, the
   cycle advances when the ramp reaches the new value.
4. **`GFLENAR`** prints `INF` when cycling is enabled with a cycle count of 0 (forever) instead of a negative
   number.
5. **Help text.** `GFLENAR`/`SFLENAR` comments in the command table and the file header now say `NO`/`YES`/`INF`.
6. **IV mode constants** are named (`IVbandCenter` 0.8, `IVbandHalf` 0.1, `IVtarget` 0.75) and explained. The
   numbers are unchanged. The target sits inside the band near its lower edge, so this was intentional.

A mistake worth remembering from this set: a global replace of `Channel2Index(SelectedFilamentChan)` with
`SelIdx()` also rewrote the body of `SelIdx()` itself, making it call itself. It compiled cleanly and would
have overflowed the stack on first use. A read of the whole diff caught it. After any scripted replace, read
the diff, especially the definitions of the thing being substituted.

### 3c. Host commands moved into the module

`FilamentCmdArray` in `Filament.cpp` holds the 28 filament commands (same entries and comments that were in
the main table in `Serial.cpp`, between `// Start of command block` and `// End of table marker`, so
`MIPScommands.txt` is still generated from them). `FilamentCmdList` is registered with
`AddToCommandList()` in the `NumberOfFilamentChannels == 0` block of `Filament_init`, like RFamp.

- The table is `const` and is in flash (`nm`: `FilamentCmdArray` at `0x000c....`). Every entry is a function or
  the address of a global. `SFLSRES`/`GFLSRES` still use `&FDarray[0].iSense` and `SFLSWD` uses
  `&FLserialWD`, which is why the pointer-to-function conversion in the earlier plan was not needed: those
  are addresses of static data, and this change was made while the data is still static.
- **No RAM saved.** The main `CmdArray` is already in flash, so moving 28 entries out of it frees no RAM. The
  gains are organization, 28 fewer entries scanned on every command lookup, and no filament commands on a
  system without the module.
- **Host-visible change:** with no filament board the commands are unknown (`GERR` = 1, `ERR_BADCMD`) instead
  of a bad-argument NAK (2). Same as RFamp.
- The command set is unchanged: 962 commands before and after, compared from `MIPScommands.txt`.
- **Dynamic allocation of the data is on hold.** Decided October 17, 2026: the gain (about 1.6 KB, and about
  0.7 KB with one board) is small next to the 16 KB freed by moving the command tables and default images out
  of RAM, and the refactor touches the safety-relevant control loop. If it is revisited, do it after the
  lab verification of sections 3, 3a and 3b so any fault is attributable.

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
| `iSense` belongs to board 0 | The bias current block and `GFLSRES`/`SFLSRES` use `FDarray[0].iSense`. With the only filament board in slot 1 the sense resistor cannot be set. |
| `CalibrateFilament` leaves DC bias channel 8 at 0 | The emission current step sets `DCbDarray[b]->DCCD[7].VoltageSetpoint = 0` and writes the DAC, then never restores the user's bias setpoint. Not changed because the DCbias module may not rewrite the DAC from the variable unless it changes. |
| Serial calibration helpers | `CalFilSupplyV`, `CalFilV`, `CalFilI` use `SelectBoard(0)` while the `FD` macro follows the selected board. Fine for a single Rev 4 board in slot 0. |
| Turn-off is ramp limited (decided, no change) | Disable and the serial watchdog timeout both ramp the current to zero at the configured rate before the enable drops: current / rate, e.g. 5 A at the default 0.1 A/s is 50 s, plus the 10 s watchdog wait. **Gordon decided on October 17, 2026 to keep this** (option A) because the gentle ramp protects the filament. The alternatives considered were an immediate off on watchdog timeout, or a faster fixed ramp for the watchdog (about 1 A/s). Revisit only if a use case needs a faster safety shutdown. Power loss already forces an immediate off (`FilamentShutdown`). |
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
