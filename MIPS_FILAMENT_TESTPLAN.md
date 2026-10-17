# MIPS filament module — lab test plan

Bench plan for the filament fixes in two sets of commits. Written October 17, 2026.

- **First set, Parts 1–8:** `5dc8f71` ("Filament: fix ramp-rate freeze, shutdown, startup glitch, Rev 2 sync
  and EEPROM restore").
- **Emission regulation, Part 12:** the commit titled "Filament: add emission current regulation (Rev 3 and 4)".
  This is new capability, not a fix. Part 12.6 covers the ADC and enable inputs added by the commit titled
  "Filament emission: add ADC setpoint inputs and a digital enable input".
- **Fourth set, Part 11:** the commit titled "Move the filament host commands into the filament module".
- **Third set, Part 10:** the commit titled "Filament: fix Rev 1 channel selection overrun, add index guards
  and finish the cleanups".
- **Second set, Part 9:** the commit titled "Filament: guard bias current, fix reporting channel, emission
  display and CalibrateFilament" (find it with `git log --oneline -- src/Filament.cpp`). Background and the reasons
behind each change are in [`MIPS_FILAMENT_FIRMWARE_HANDOFF.md`](MIPS_FILAMENT_FIRMWARE_HANDOFF.md).

**Status:** builds under PlatformIO, **not yet run on hardware.** Everything below is unverified.

**Code:** [`src/Filament.cpp`](src/Filament.cpp), [`include/Filament.h`](include/Filament.h).

---

## 0. Roadmap

Run in this order. Parts 1–3 are the safety items and should pass before anything else is trusted.

| Order | Part | What | Hardware needed | If it fails |
|---|---|---|---|---|
| 1 | 0 | Prerequisites, backup of the module EEPROM | Any filament module | Stop |
| 2 | 1 | Ramp rate floor and turn-off | Dummy load or filament | Revert `MinRampRate` use in `Filament_loop` |
| 3 | 2 | Startup enable glitch | Scope on the enable line | Report, see handoff section 4 |
| 4 | 3 | Shutdown on power loss | Bench supply you can ramp down | Report, do not ship |
| 5 | 4 | Mode change (Rev 1 vs Rev 2+) | Rev 1 board, Rev 2+ board | Revert `FmodeChange` |
| 6 | 5 | Cycling status command | Any | Low risk |
| 7 | 6 | Rev 2+ parameter sync | Rev 2 or 3 board | Revert `FilamentSyncChannels` calls |
| 8 | 7 | EEPROM restore size handling | Any, uses `WEEPROM` | Revert `RestorFilamentSettings` |
| 9 | 9 | Second set: bias guard, reporting, emission display, `CALFIL` | Rev 1, 2+, 4; DCbias module for 9.1 and 9.4 | Revert the item, they are independent |
| 10 | 10 | Third set: Rev 1 channel selection, cycling, `GFLENAR`, IV mode | Rev 1 two board system for 10.1 | Revert the item |
| 11 | 11 | Filament commands registered by the module | Any system, with and without a filament board | Revert the commit |
| 12 | 12 | Emission current regulation, and its ADC and enable inputs | Rev 3 module, vacuum tube first; a signal source on a MIPS ADC input for 12.6 | `SFLEREG,<ch>,OFF`, `SFLIADC,<ch>,-1,1,0`, `SFLEADC,<ch>,-1,1,0`, `SFLDIO,<ch>,NA` |
| 13 | 8 | Regression of normal operation (run last) | Any | Find which part above broke it |

Revision matrix. Only some tests apply to each module revision; mark the ones you have.

| Rev | Module | Applies to |
|---|---|---|
| 1 | Two channel module, one or two boards | 1, 2, 3, 4 (Rev 1 branch), 5, 7, 8 |
| 2 | Current reversible, one channel per board | 1, 2, 3, 4 (Rev 2+ branch), 5, 6, 7, 8 |
| 3 | Rev 2 plus on-module emission current | As Rev 2 |
| 4 | Single board e-msion (AD5593) | 1, 2, 3 (Rev 4 branch), 7, 8. Direction control does not apply |

---

## Part 0 — Prerequisites

- [ ] **Firmware:** build from `master` at `5dc8f71` or later (`~/.platformio/penv/bin/pio run`). The version
      string is still 1.268, so record `git log -1 --oneline` in the notes.
- [ ] **Save the module EEPROM image first.** Part 7 deliberately writes the EEPROM. Use `SAVEMOD` (save
      module EEPROM to SD) and keep the file, or at least run the dialog `Save settings` once on a known
      good state so it can be restored.
- [ ] **A safe load.** A dummy resistive load or a spare filament. Know its limit before setting any
      current. The tests below use 0.5 to 2 A, scale to your load.
- [ ] **Instruments:** DMM on the filament output, scope on the enable line (Rev 1–3: the `Fpwr` pins, 14 for
      channel 1 and 48 for channel 2 by default, active LOW; Rev 4: the AD5593 digital output, CH0, active
      LOW), a current meter or clamp.
- [ ] **Know what is installed:** `GFLRT,1` answers, and the filament dialog shows the expected number of
      channels (Rev 1 shows 2 per board, Rev 2+ shows 1 per board).
- [ ] **Command reference:** channels are 1 based on the serial commands. `SFLENAR` takes `YES` or `NO`
      (the help text says ON/OFF, that is wrong, see the handoff).

---

## Part 1 — Ramp rate floor (`MinRampRate` = 0.01 A/s)

The bug: a ramp rate of 0 made the ramp step 0, so the current never changed and the enable pin never
switched off because it only drops after the ramped-down setpoint reaches zero.

- [ ] **1.1 Serial limits.** `SFLRT,1,0` NAK. `SFLRT,1,0.005` NAK. `SFLRT,1,0.01` ACK. `SFLRT,1,1` ACK.
      `SFLRT,1,1.5` NAK. `GFLRT,1` returns the value just set.
- [ ] **1.2 Dialog limit.** On filament page 2 the "Ramp rate" entry cannot be turned down below 0.01.
- [ ] **1.3 Normal ramp.** `SFLRT,1,0.2`, `SFLI,1,1.0`, `SFLENA,1,ON`. `GFLAI,1` rises about 0.2 A every
      second and reaches 1.0 A in about 5 s. `SFLENA,1,OFF`: current falls at the same rate and **the
      enable line goes HIGH (off) when it reaches zero**, a few seconds later.
- [ ] **1.4 Slowest legal rate.** `SFLRT,1,0.01`, `SFLI,1,0.1`, enable. Reaches 0.1 A in about 10 s.
      Disable: turns off after about 10 s. It must not hang at any current.
- [ ] **1.5 A stored 0 is tolerated** (optional, needs an old module image that holds a ramp rate of 0, or
      write the 4 bytes of RampRate with `WEEPROM` and Restore). The current must move at 0.01 A/s, and
      disable must reach power-off.
- [ ] **1.6 Serial watchdog path.** `SFLSWD,TRUE`, enable the filament at 0.5 A, stop all serial traffic. After
      about 10 s the "No comms, Timedout!" message appears, the setpoint ramps down and the enable line
      goes off. Record how long the whole turn-off took at the rate in use; it is the ramp time, not
      instant. (If that is too slow for your application see handoff section 5.)

---

## Part 2 — Startup enable glitch

The bug: the enable output powered up driven LOW (on) until the first pass of the filament thread.

- [ ] **2.1 Scope the enable line through a power-up** (trigger on the falling edge, 100 mS/div or slower).
      It must stay HIGH (off) from the moment MIPS starts driving it. Repeat for each channel.
- [ ] **2.2 Rev 4:** scope the AD5593 digital output CH0 the same way.
- [ ] **2.3 DMM on the filament output across a power cycle.** No voltage or current at any point before
      the first deliberate enable. Do it at least 10 times, with and without `NormalStartup`.
- [ ] **2.4 Reset path.** `Software_Reset` or the watchdog reset behaves the same as a power cycle.

---

## Part 3 — Shutdown on power loss (`FilamentShutdown`)

The bug: it wrote AD5625 commands to Rev 4 (which uses an AD5593), ignored which boards exist, and only
drove board 0's enable pins.

`FilamentShutdown` runs from the power-off handler in [MIPS.cpp](src/MIPS.cpp) (`ReadVin() < 10.0`). That
handler is for **main power lost while the controller stays alive on USB** ("MIPS power is off! USB powering
controller!"). The modules lose their supply at the same time, so the purpose is a defined off state while
the module supply decays, and not sending the wrong chip protocol to a Rev 4.

You need a bench supply on the MIPS input that you can lower through 10 V **with USB connected**, otherwise the
controller dies with the supply and nothing runs. The module may already be dead by the time the writes land,
so judge pass or fail by the enable line and the output as the supply falls (scope, slow ramp), and by the
state when you re-apply power. **Run it with a safe load, not a good filament.**

- [ ] **3.1 Rev 1–3, one board.** Enable at 1 A. Lower the supply below 10 V (USB connected). The enable line
      goes HIGH and stays there, and the filament output reads zero as the supply falls.
- [ ] **3.2 Rev 1–3, two boards** (if available). Both boards' enable lines go off.
- [ ] **3.3 Rev 4.** Same test. Before this fix nothing was written to the AD5593 enable or DACs (the AD5625
      commands do not apply to it). Confirm the AD5593 digital output goes HIGH while the supply decays.
- [ ] **3.4 No filament module installed.** Lower the supply on a system with no filament board. It shows the
      power-off screen and resets when power returns, as before, with no hang (the function now does nothing
      when no board is present).
- [ ] **3.5 Recovery.** Restore the supply. The system restarts, the filament comes up **off**, and a
      normal enable works.

---

## Part 4 — Mode change

`FmodeChange` wrote both channels' mode for every revision. Now only Rev 2+ does that.

- [ ] **4.1 Rev 1, two channels.** Select channel 1, set Mode to `IVctrl`. Select channel 2: Mode is still
      `Ictrl`. Set channel 2 to `IVctrl` and channel 1 back to `Ictrl`. Each channel keeps its own value
      through channel switches and through Save and Restore.
- [ ] **4.2 Rev 2 or 3.** Set Mode to `IVctrl`. Switch the current direction (filament off). Mode is
      still `IVctrl` on the other direction. This is the 1/27/2019 behavior that must not regress.
- [ ] **4.3 The Voltage entry** is not editable while `IVctrl` is selected and editable in `Ictrl`
      (existing behavior).

---

## Part 5 — Cycling status (`SFLENAR`)

The bug: the dialog copy was updated for the wrong channel (1-based channel compared with a 0-based
selection).

- [ ] **5.1 Two channels (Rev 1).** In the filament dialog select channel 2, open "Cycle menu". Send
      `SFLENAR,2,YES`. "Enable cycling" shows YES on that page. Send `SFLENAR,2,NO`: back to NO.
- [ ] **5.2 Other channel unaffected.** Select channel 1 in the dialog. Send `SFLENAR,2,YES`. Channel 1's
      cycle page still shows NO.
- [ ] **5.3 Round trip.** `GFLENAR,2` returns `NO` when off, and the cycles remaining when on.
- [ ] **5.4 Cycling still works.** `SFLP1,1,0.2`, `SFLP2,1,0.6`, `SFLCY,1,3`, `SFLENAR,1,YES`, enable. The
      current moves between the two points three times and then returns to the saved setpoint with
      cycling reported as `NO`.

---

## Part 6 — Rev 2+ parameter sync

Rev 2 and later keep one set of parameters in both channel structures. A serial `SFLI` or `SFLSV` used to
update only part of that, so the other direction came up with stale values.

Use a Rev 2 or 3 board, filament **disabled** for the whole part.

- [ ] **6.1 Serial then FWD to REV.** `SFLDIR,1,FWD`, `SFLI,1,1.5`, `SFLSV,1,3.5`, `SFLRT,1,0.2`. `SFLDIR,1,REV`.
      `GFLI,1` = 1.5, `GFLSV,1` = 3.5, `GFLRT,1` = 0.2. The dialog agrees.
- [ ] **6.2 REV to FWD.** `SFLI,1,0.8`, `SFLSV,1,2.5`. `SFLDIR,1,FWD`. `GFLI,1` = 0.8, `GFLSV,1` = 2.5.
- [ ] **6.3 Dialog path.** Change the "Current dir" entry on page 2 with the filament off. Current, voltage,
      ramp rate and mode carry over, same as 6.1.
- [ ] **6.4 Enable after the change.** Enable on the new direction. It ramps to the values from 6.1/6.2,
      not to an older setpoint.
- [ ] **6.5 Refusals unchanged.** With the filament enabled, `SFLDIR` and the dialog change are refused.
- [ ] **6.6 Rev 1 is unaffected.** On a Rev 1 board, `SFLI` and `SFLSV` change only the addressed channel.

---

## Part 7 — EEPROM restore size handling

`RestorFilamentSettings` used to force the stored size to the current one, so an image from older firmware
brought in garbage for the newer fields. Now only the stored `Size` bytes are used and the rest keep their
defaults.

Structure facts (`FilamentData`, 256 bytes total, little endian, verified from the build):

| Field | Offset | Size |
|---|---|---|
| `Size` | 0 | 2 |
| `Name` | 2 | 20 |
| `Rev` | 22 | 1 |
| `NumChannels` | 23 | 1 |
| `FCD[2]` | 24 | 2 × 88 |
| `ADCadr` / `DACadr` / `EEPROMadr` | 200 / 201 / 202 | 1 each |
| `FCyl[2]` | 204 | 2 × 16 |
| `iSense` | 236 | 4 |
| `Idir` | 240 | 1 |
| `Ecurrent` | 244 | 12 |

The minimum stored size accepted is 204 (through the TWI addresses). `WEEPROM` takes
`board, TWI address, EEPROM address, data`, all decimal. The filament module EEPROM is at TWI address 82 (0x52).

- [ ] **7.1 Round trip.** Change several settings, `Save settings` on page 2, change them again, `Restore
      settings`. The saved values return. The dialog shows "Parameters Restored!".
- [ ] **7.2 Image from before `Idir` and `Ecurrent` existed.** On a Rev 2 board: set the live direction to FWD
      first (`SFLDIR,1,FWD`), because fields the image lacks keep their **current live** values. `Save
      settings`. Write `Size` = 240 (bytes 0 and 1 = 240, 0). Write byte 240 = 0 (a stored `Idir` of false). Restore.
      `GFLDIR` must report **FWD**, the default, not REV. (The old code copied the stored 0 and reported REV.)
- [ ] **7.3 Image from before `iSense` existed.** On any Rev: make sure the live value is 0 first
      (`SFLSRES,0`), then `Save settings`. Write `Size` = 236 (bytes 236
      to 239 = 255, 255, 255, 255). Restore. `GFLSRES` must report **0**, not -1.
- [ ] **7.4 Too small an image is rejected.** Write `Size` = 100. Restore. "Corrupted EEPROM data!" and
      nothing in the live settings changes.
- [ ] **7.5 Bad revision or channel count.** Write `Rev` (byte 22) = 9, Restore: rejected. Write
      `NumChannels` (byte 23) = 0 and then 7: rejected both times. Restore the real values afterwards.
- [ ] **7.6 Larger image than this firmware knows** (a newer build). Write `Size` = 400. Restore succeeds
      and uses the first 256 bytes, with no crash.
- [ ] **7.7 Wrong or unterminated name.** Change one `Name` byte (offset 2): "Corrupted EEPROM data!".
      Fill offsets 2 to 21 with 255 (no terminator): rejected, no crash.
- [ ] **7.8 Put it back.** After the tests, `Save settings` from a known good state so the EEPROM holds a
      normal 256 byte image, or reload the `SAVEMOD` file from Part 0.
- [ ] **7.9 Power-up restore.** After 7.8, power cycle. The saved settings load at start up
      (`NormalStartup` true).

---

## Part 8 — Regression of normal operation

Anything not touched should behave as before. Spot check on each revision you have.

- [ ] Enable, ramp, steady state, disable with `GFLAI`, `GFLV`, `GFLASV`, `GFLPWR` plausible.
- [ ] `IVctrl` mode holds the supply voltage so the drop across the current source is about 0.75 V.
- [ ] The maximum power limit still reduces the current setpoint when exceeded.
- [ ] `RFLPARMS,1,2` reports FIL1 every 2 s, `RFLPARMS,1,0` stops it.
- [ ] Emission current / bias current display (Rev 3, 4, or a DCbias sense resistor) still updates.
- [ ] The Calibration pages open and `Return` works. (Do not run the calibrations unless you intend to.)
- [ ] `FilamentChannels` returns the right channel count.
- [ ] Front panel: channel selection, enable, current entry, page changes, return to main menu.
- [ ] Nothing else on the system misbehaves after a filament run: DC bias, ARB and the QUAD module still
      respond (the filament thread selects boards on the TWI bus).

---

## Part 9 — Second set of fixes

Four independent changes. Each can be tested and reverted on its own.

### 9.1 Bias current guard (DCbias channel 8 current sense)

The bug: with a non-zero sense resistor and no DC bias module the loop dereferenced a NULL pointer.

- [ ] **9.1.1 No DC bias module.** On a system with a filament board and **no** DC bias module, `SFLSRES,10000`
      then enable the filament and leave it running for a minute. No reset, no hang. `GFLECUR` returns a
      number (0 or stale), not an error. `SFLSRES,0` afterwards.
- [ ] **9.1.2 With a DC bias module** (Rev 1 or 2 with the sense resistor wired as designed). `SFLSRES,<value>`
      and compare the Bias I reading and `GFLECUR` with what you saw before this change. Same value, same
      response when you change the DC bias channel 8 setpoint.
- [ ] **9.1.3 Power off.** With the sense resistor set, turn main power off and on. No lock-up in the
      filament loop while the DC bias module is unpowered.

### 9.2 Reporting threads (`RFLPARMS`)

The bug: `FIL1`/`FIL2` always read structure index 0/1. On a Rev 2+ board the live structure follows the
current direction, so with REV selected they reported the idle channel (zeros).

- [ ] **9.2.1 Rev 1.** `RFLPARMS,1,1` (every second). Lines look like `FIL1,<current>,<voltage>,<power>`
      and match `GFLAI,1`, `GFLV,1`, `GFLPWR,1`. Repeat for channels 2, 3, 4 on the boards present. Stop with
      `RFLPARMS,<ch>,0`.
- [ ] **9.2.2 Rev 2 or 3, FWD.** Enable at a small current. `RFLPARMS,1,1` matches `GFLAI,1` / `GFLV,1` /
      `GFLPWR,1`.
- [ ] **9.2.3 Rev 2 or 3, REV.** Disable, `SFLDIR,1,REV`, enable. The report is **non zero** and matches the
      `G...` commands. Before the fix it printed zeros in this direction.
- [ ] **9.2.4 Direction change while reporting.** With reporting running, disable, switch direction, enable.
      The reported values follow the live channel.
- [ ] **9.2.5 No filament board.** A channel number above the installed count is refused by the command
      (unchanged).

### 9.3 Emission current display (Rev 3 and Rev 4)

The bug: the one-per-board emission ADC reading was filtered into the single displayed value once per
channel for every channel on every board, so it blended boards and doubled the filter on a two channel
board.

- [ ] **9.3.1 One board.** With the filament running and a known emission current, "Bias I, uA" and `GFLECUR`
      agree with an external meter as well as before the change.
- [ ] **9.3.2 Response time.** Step the emission current. The reading settles in about 2 s per time
      constant (filter 0.05 per 100 mS pass), where a two channel board used to settle in about 1 s. This is
      expected; note the observed value.
- [ ] **9.3.3 Two boards** (if you have them). Select channel 1, then channel 2: the display shows each
      board's own emission current, not a blend.
- [ ] **9.3.4 Rev 1 / Rev 2 with a sense resistor** (the DC bias path) is unchanged. Rev 4 with `iSense` = 0
      shows only the emission reading.

### 9.4 `CALFIL` (Rev 4 calibration procedure)

Only on a Rev 4 filament module. This procedure energizes the filament supply and drives DC bias channel 8;
**use a dummy load** and read the prompts.

- [ ] **9.4.1 Refuses without a DC bias module.** On a Rev 4 system with no DC bias module, `CALFIL` prints
      "Can't proceed, no DC bias module..." **immediately**, before any output is turned on. The filament stays
      off, nothing was changed.
- [ ] **9.4.2 Wrong revision.** On a Rev 1/2/3 board `CALFIL` still prints "Can't proceed, wrong hardware rev!".
- [ ] **9.4.3 Record the operating settings first** (`GFLI,1`, `GFLSV,1`, `GFLRT,1` and the Mode on page 2).
- [ ] **9.4.4 Real time ramps.** Run `CALFIL`. The supply voltage now rises in 0.1 V steps over about 3 s
      (scope or DMM on the filament supply), where it used to jump to the value almost instantly. Each
      "100 pass" stage takes about 10 s. The system does not watchdog reset.
- [ ] **9.4.5 Complete the procedure** through the emission current calibration. "Filament calibration
      finished!" appears.
- [ ] **9.4.6 Settings are not left in the calibration state.** `GFLI,1` and `GFLSV,1` match 9.4.3, not 0 A and
      3.0 V. The Mode is **IVctrl** afterwards (the production mode, set deliberately at the end whatever it was
      before) and page 2 shows `IVctrl`. The filament is **off** afterwards.
- [ ] **9.4.7 Persisted.** Power cycle. The restored `GFLI`, `GFLSV`, mode and the new calibration (reported
      voltages/currents agree with a meter) are what was saved.
- [ ] **9.4.8 Known leftover.** DC bias channel 8 is left at 0 V by the procedure (see the handoff). Note
      whether you need to set it again afterwards.

---

## Part 10 — Third set

### 10.1 Rev 1 channel selection with two boards

The bug: on a two-board Rev 1 system, selecting channel 3 or 4 in the dialog loaded data from past the end of
the channel array and the next dialog change wrote it back over that channel. **Skip this on a one board
system; it could not happen there.** Save the module EEPROMs first (`SAVEMOD`).

- [ ] **10.1.1 Select every channel.** In the filament dialog set Channel to 1, 2, 3, 4 in turn. Each shows its
      own Current, Voltage and Max power (set distinct values first through the serial commands:
      `SFLI,1,0.5`, `SFLI,2,0.6`, `SFLI,3,0.7`, `SFLI,4,0.8`). No garbage values at any channel.
- [ ] **10.1.2 Edit then re-read.** On channel 3 change the Current in the dialog to 0.9. `GFLI,3` = 0.9 and
      `GFLI,4`, `GFLI,1`, `GFLI,2` are unchanged.
- [ ] **10.1.3 The channel mapping survives.** After 10.1.1/10.1.2, enable channel 3 at a small current into a
      safe load. It switches the correct output and `GFLAI,3`, `GFLV,3`, `GFLPWR,3` read plausible values.
      Before the fix this could corrupt channel 3's pin and DAC/ADC numbers.
- [ ] **10.1.4 Save and Restore** on both boards still returns what was saved.
- [ ] **10.1.5 One board.** On a single board system the Channel entry stops at 2 and channels 1 and 2 behave
      as before.

### 10.2 Serial channel validation

- [ ] `GFLI,0` and `GFLI,<one more than the channel count>` NAK. `SFLI,5,1` NAKs on a 4 channel system. No
      reset or odd values after any of them.
- [ ] `FilamentChannels` returns the number of channels present.

### 10.3 Cycling after a power trim

The bug: the maximum power limit lowers the setpoint by 0.1 A, after which the ramp stopped below point 2 and
the old test (ramp equals point 2 exactly) never succeeded, so the cycle stalled.

- [ ] **10.3.1 Normal cycle.** `SFLP1,1,0.2`, `SFLP2,1,0.6`, `SFLCY,1,3`, `SFLENAR,1,YES`, enable. Three cycles
      complete, the filament returns to the saved setpoint, `GFLENAR,1` = `NO`.
- [ ] **10.3.2 Power limited cycle.** Set the Max power on page 2 low enough that point 2 exceeds it. The
      limiter trims the current and the cycles **still complete** (at the trimmed value) instead of hanging at
      point 2. Note the reported current at the top of each cycle.
- [ ] **10.3.3 Changing the setpoint mid cycle** with `SFLI` makes the cycle advance when the ramp reaches the
      new value. Expected, not a fault.

### 10.4 `GFLENAR`

- [ ] Off: `GFLENAR,1` = `NO`. With `SFLCY,1,3` and cycling on: the cycles remaining, counting down.
- [ ] With `SFLCY,1,0` and cycling on: `INF` (previously a negative number).
- [ ] `HELP` or `MIPScommands.txt` shows the new text for `GFLENAR` and `SFLENAR` (YES/NO).

### 10.5 IV mode unchanged

- [ ] In `IVctrl` the voltage across the current source settles around 0.75 V and holds inside 0.7 to 0.9 V,
      the same as before. Only names changed.

---

## Part 11 — Commands registered by the module

The 28 filament host commands moved from the main table in Serial.cpp into a `const` table in Filament.cpp,
registered by the first `Filament_init`. The command set is unchanged (962 commands before and after, checked
from the generated `MIPScommands.txt`).

- [ ] **11.1 No filament module installed.** Each of `GFLI,1`, `GFLENA,1`, `SFLRT,1,0.1`, `GFLSRES`, `SFLSWD,TRUE`
      and `CALFIL` is refused as an unknown command, and `GERR` reports **1** (`ERR_BADCMD`). They used to
      report 2 (`ERR_BADARG`). Nothing else on the system changes, and the other commands you already
      tested still respond.
- [ ] **11.2 Filament module installed.** All of the commands in the filament section of `MIPScommands.txt`
      respond as before: `GFLENA`/`SFLENA`, `GFLI`/`SFLI`, `GFLAI`, `GFLSV`/`SFLSV`, `GFLASV`, `GFLV`, `GFLPWR`,
      `GFLRT`/`SFLRT`, `GFLDIR`/`SFLDIR`, `GFLP1`/`SFLP1`, `GFLP2`/`SFLP2`, `GFLCY`/`SFLCY`, `GFLENAR`/`SFLENAR`,
      `RFLPARMS`, `GFLSRES`/`SFLSRES`, `GFLECUR`, `SFLSWD`, `CALFIL`.
- [ ] **11.3 Two boards.** The commands are registered once, not twice: with two filament boards installed
      `GFLI,1` and `GFLI,3` both answer and nothing is duplicated or looped.
- [ ] **11.4 The pointer commands still work.** `SFLSRES,1000` then `GFLSRES` returns 1000;
      `SFLSWD,TRUE` then `SFLSWD,FALSE` are accepted. These point at `FDarray[0].iSense` and `FLserialWD`.
- [ ] **11.5 Power-off command set.** With main power off the reduced command set still answers (`GVER`,
      `ISPWR`, `GNAME`, `RESET`, `POWER`) and the filament commands are not available. After power returns the
      system restarts and the filament commands work again.
- [ ] **11.6 Host software.** If the host application probes for a filament module by sending a filament
      command, check how it reacts to error 1 instead of 2.

---

## Part 12 — Emission current regulation (Rev 3 and 4)

New capability: hold a user set emission current (uA) by adjusting the filament current. Design and limits are in
`src/FilamentEmission.cpp` and the handoff section 3d. The regulator only writes the commanded filament current
(`CurrentSetpoint`), so the ramp limiter, power limiter, serial watchdog and disable ramp-down all still apply.
Settings are RAM only and return to their defaults at power up.

**Read first.** The filament limit is the module **Max power** (page 2 of the filament dialog, 1 to 20 W in 1 W
steps; there is no serial command for it). The regulator holds the current at 97% of it. The controller cannot
see whether bias is applied: with no bias it raises the current to the power limit and waits, and when bias then
arrives the emission can be far above the setpoint for a time (the ramp limiter also limits how fast the
current falls). In the host simulation, with the limit 4% above the operating power, late bias gave a peak of
about 250 to 300 uA for a 200 uA setpoint; with the limit 25% above it, 1.6 mA to well over 5 mA for 10 to 30 s.
**Set Max power only a little above the power at the operating point.**

Defaults: setpoint 0, largest setpoint 1000 uA, lowest current 0 A, gain 0.005 A/s per unit of log error, rise
0.02 A/s, fall 0.1 A/s, rate hold 0.05 per second. Regulation starts OFF.

### 12.0 Host check of the control law (any computer)

- [ ] `c++ -std=c++11 -O1 -Iinclude tools/FilamentEmissionSim.cpp -o /tmp/femsim && /tmp/femsim` runs and prints
      the same table as in the handoff (cold start settles in 120 to 230 s, worst overshoot 28% only at the
      extreme plant, power hold at 5.83 W for a 6 W limit). This is the model, not your filament.

### 12.1 Commands and refusals (no filament needed)

- [ ] On a Rev 3 module, each of `SFLEIS`, `GFLEIS`, `SFLEREG`, `GFLEREG`, `GFLEST`, `SFLEIM`, `GFLEIM`, `SFLEIMN`,
      `GFLEIMN`, `SFLEGAIN`, `GFLEGAIN`, `SFLEUP`, `GFLEUP`, `SFLEDN`, `GFLEDN`, `SFLERH`, `GFLERH` answers, and a
      value read back equals the value set.
- [ ] `GFLEST,1` right after power up is `OFF,<measured>,<current>`. Defaults match the list above.
- [ ] Range checks NAK: `SFLEIS,1,2000` (above the largest setpoint 1000), `SFLEIM,1,0`, `SFLEGAIN,1,1`,
      `SFLEUP,1,0`, `SFLEREG,1,MAYBE`.
- [ ] `SFLEIM,1,100` with the setpoint at 200 lowers the setpoint to 100.
- [ ] On a Rev 1 or Rev 2 module the commands NAK (no on-module emission measurement). With no filament module
      they are unknown commands (`GERR` = 1).
- [ ] `SFLEREG,1,ON` is refused while filament cycling is enabled (`SFLENAR,1,YES`).

### 12.2 Measurement first (vacuum tube)

- [ ] Wire the tube so the filament module's emission measurement sees the emission current. Set **Max power** a
      little above the power at the intended operating point (see above).
- [ ] With regulation off, set the filament current by hand (`SFLI`) and compare the emission reading
      (`GFLECUR` or "Bias I, uA") with an external meter. Calibrate it first if it is off ("Cal Emission I").
      The regulator is only as good as this reading.
- [ ] Note the filament current at which emission first appears and the current for 200 uA. Set `SFLEIMN` just
      below the onset to shorten the start-up climb.

### 12.3 Closed loop

- [ ] **Cold start.** `SFLEIS,1,200`, `SFLEREG,1,ON`, then enable the filament. Poll `GFLEST,1` every 5 s and log
      status, emission and current. Expect `SEEK` rising to `REG` and a settle to within 5% in about 2 to 4
      minutes, with little overshoot. Record settle time, peak, and the steady ripple.
- [ ] **Setpoint steps.** `SFLEIS,1` 200 to 100 to 400 to 200. Each settles in well under a minute. Record.
- [ ] **Disturbance.** Change the bias or the plate voltage so the emission moves. The regulator brings it back.
- [ ] **Tuning, if needed.** Overshoot or oscillation: lower `SFLEGAIN` (halve it) and `SFLERH`. Too slow: raise
      them. Record the values that work for the tube.
- [ ] **Direction.** On a Rev 3 module, repeat a short run with the current direction reversed (`SFLDIR`). The live
      channel is regulated in either direction.

### 12.4 Limits

- [ ] **Power limit as the ceiling.** Set **Max power** below what the setpoint needs. Status `PLIM`, the current
      steady at about 97% of the power limit, emission below the setpoint, and **no sawtooth** of 0.1 A steps
      from the power limiter trimming (watch `GFLI` or the current on a meter). Raising Max power releases it.
- [ ] **Late bias (option A).** Start with the bias off: enable, regulate, wait until the current has climbed to the
      power limit (`PLIM`). Apply bias. Record the peak emission and the time to settle, and compare with the
      simulation figures above. Repeat with Max power closer to the operating power.
- [ ] **Largest setpoint.** `SFLEIM,1,300`: a setpoint above 300 is refused.

### 12.5 Interactions

- [ ] **Disable and enable.** `SFLENA,1,OFF`: status `WAIT`, the current ramps down at the ramp rate and the
      enable line drops. `SFLENA,1,ON`: it ramps back to the last regulated current and regulates again.
- [ ] **Serial watchdog.** `SFLSWD,TRUE`, stop serial traffic: after the timeout the filament turns off as before
      and the status goes to `WAIT`.
- [ ] **Cycling excludes regulation.** With regulation on, `SFLENAR,1,YES`: status `BLOCKED`, cycling runs.
      `SFLENAR,1,NO`: regulation resumes.
- [ ] **A manual `SFLI` while regulating** is treated as a new starting point and the regulator walks back.
- [ ] **Regulation off** (`SFLEREG,1,OFF`) leaves the current where it is.
- [ ] **Two modules.** Regulate channel 1 and channel 2 at different setpoints at the same time. They do not
      interact.
- [ ] **Power loss while regulating** shuts the filament down as before.

### 12.6 ADC setpoint inputs and the enable input

In the style of the ESI module's `SESICTRL`: `SFLEADC,chan,adc,slope,intercept` (ADC to emission setpoint, uA),
`SFLIADC,chan,adc,slope,intercept` (ADC to heater current, A), `SFLDIO,chan,Q..X|NA` (digital input that enables the
filament). `adc` is a MIPS ADC channel 0 to 9, or -1 for none. The value is `counts * slope + intercept` where
counts are the 12 bit Due ADC (0 to 4095 for 0 to 3.3 V at the pin), 10 reads averaged, updated at 10 Hz. Settings
are RAM only. Use a bench supply or a potentiometer into the MIPS ADC input, with your own divider.
**Test with a dummy load or the tube at a safe power first. A floating ADC input reads garbage.**

- [ ] **12.6.1 Commands.** Each of `SFLEADC`, `GFLEADC`, `SFLIADC`, `GFLIADC`, `SFLDIO`, `GFLDIO` answers.
      `GFLEADC,1` after `SFLEADC,1,2,0.5,10` returns `2,0.500000,10.000000,<last value>`. Defaults are `-1,1,0,0` and
      `NA`. Range checks NAK: `SFLEADC,1,10,1,0` (adc above 9), `SFLEADC,3,...` (no channel 3), `SFLDIO,1,A`.
- [ ] **12.6.2 Emission setpoint from the ADC.** `SFLEADC,1,<adc>,<slope>,<intercept>`; vary the input. `GFLEIS,1` follows
      it within a second, clamped to 0 and the largest setpoint (`GFLEIM`). With regulation on
      (`SFLEREG,1,ON`) the emission follows the changing setpoint. `SFLEADC,1,-1,1,0` removes it and the
      setpoint stays where it was. A manual `SFLEIS` is overwritten while it is defined.
- [ ] **12.6.3 Heater current from the ADC.** `SFLIADC,1,<adc>,<slope>,<intercept>`; vary the input with the filament
      enabled into a safe load. `GFLI,1` follows it, the actual current (`GFLAI`) follows at the ramp rate, and it is
      clamped to 0 to 12 A. `SFLI` and the dialog are overwritten while it is defined.
- [ ] **12.6.4 Heater ADC disables regulation.** With regulation on, define the heater ADC: `GFLEREG,1` = OFF,
      `GFLEST,1` status is `HEATADC`. `SFLEREG,1,ON` is refused (NAK). `SFLIADC,1,-1,1,0` removes it: status
      returns to `OFF`, `SFLEREG,1,ON` is accepted again, and regulation does not restart on its own.
- [ ] **12.6.5 Cycling.** With the heater ADC defined, enable filament cycling: the ADC stands aside (the current
      follows the cycle, status `BLOCKED`). Disable cycling and the ADC takes the current again.
- [ ] **12.6.6 The power limiter still works with the heater ADC.** Set **Max power** below the power the ADC value
      would produce (e.g. a 6 W limit and an input asking for more). The power settles at about the limit, not the
      value the ADC asks for: in the host simulation 5.96 W mean and 6.18 W peak for a 6 W limit, and a steady current
      with **no repeated 0.1 A sawtooth**. A rule that only held at 97% of the limit gave an 8.4 W peak in
      simulation (a limit cycle), which is why the command is also held to 0.1 A above the ramped current. Check
      this one carefully: it is the safety interaction. Removing the high input lets the current fall.
- [ ] **12.6.7 Enable input.** `SFLDIO,1,Q` (any of Q to X): high on that digital input enables the filament within
      about 100 mS, low disables it and the current ramps down, and **this overrides `SFLENA` and the dialog** while
      defined, as in ESI. `GFLDIO,1` returns the letter. `SFLDIO,1,NA` removes it and the filament stays as it was.
      Check the input's polarity setting (`DIinvert`) gives the expected sense.
- [ ] **12.6.8 ADC in use.** While the MIPS ADC digitizer is running (a host driven `RFAACQ` scan with the MIPS ADC),
      the ADC values hold their last reading instead of being read, and resume afterwards.
- [ ] **12.6.9 Two modules.** Assign different ADC channels to channel 1 and channel 2; each follows its own input.

---

## Sign-off

| Part | Rev tested | Result | Date | Notes |
|---|---|---|---|---|
| 0 Prerequisites | | | | `git log -1`: |
| 1 Ramp rate floor | | | | |
| 2 Startup glitch | | | | scope capture saved? |
| 3 Shutdown | | | | |
| 4 Mode change | | | | |
| 5 Cycling status | | | | |
| 6 Rev 2+ sync | | | | |
| 7 EEPROM restore | | | | EEPROM put back? |
| 9.1 Bias current guard | | | | |
| 9.2 Reporting threads | | | | |
| 9.3 Emission display | | | | |
| 9.4 `CALFIL` | | | | |
| 10.1 Rev 1 channel selection | | | | two boards needed |
| 10.2 Channel validation | | | | |
| 10.3 Cycling after trim | | | | |
| 10.4 `GFLENAR` | | | | |
| 10.5 IV mode | | | | |
| 11 Commands registered by module | | | | with and without a board |
| 12.0 Host check | | | | |
| 12.1 Commands | | | | |
| 12.2 Measurement | | | | emission reading agrees with meter? |
| 12.3 Closed loop | | | | gain / rate hold used: |
| 12.4 Limits and late bias | | | | peak emission on late bias: |
| 12.5 Interactions | | | | |
| 12.6 ADC and enable inputs | | | | limiter check 12.6.6: |
| 8 Regression | | | | |

Record anything unexpected in the handoff's open items table with the commit from `git log -1`.
