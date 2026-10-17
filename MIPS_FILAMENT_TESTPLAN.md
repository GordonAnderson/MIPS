# MIPS filament module — lab test plan

Bench plan for the filament fixes in two sets of commits. Written October 17, 2026.

- **First set, Parts 1–8:** `5dc8f71` ("Filament: fix ramp-rate freeze, shutdown, startup glitch, Rev 2 sync
  and EEPROM restore").
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
| 11 | 8 | Regression of normal operation (run last) | Any | Find which part above broke it |

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
| 8 Regression | | | | |

Record anything unexpected in the handoff's open items table with the commit from `git log -1`.
