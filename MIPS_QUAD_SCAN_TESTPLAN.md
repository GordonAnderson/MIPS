# MIPS QUAD scan and TWI — lab test roadmap

Ordered bench plan for everything not yet verified on hardware. Written October 16, 2026 for a
lab session on a different computer; read [`MIPS_QUAD_FIRMWARE_HANDOFF.md`](MIPS_QUAD_FIRMWARE_HANDOFF.md)
section 2a for the background and the design reasons behind each item.

**Status of the code under test:** all of it builds under PlatformIO. Parts A–E have **not run on
hardware** except where a line says "measured". Part F is the earlier external DAQ plan, kept as it
was, and is the regression check for the shared scan loop that Part D changes.

---

## 0. Roadmap

Run the parts in this order. Each part says what to do if it fails; none of them blocks Part A.

| Order | Part | What | Needs | If it fails |
|---|---|---|---|---|
| 1 | 0 | Prerequisites and baselines | Both firmware images, scope | Stop, nothing else is meaningful |
| 2 | A | `TWI1CMD` reply reading | Any Wire1 device (the electrometer) | Revert `bb69a44` |
| 3 | C | `GTWI` / `STWI` fixes | Wire and Wire1 devices | Revert `f4b8129` |
| 4 | B | `TWICMD` channel 0 | A Wire device behind the board mux | Set `TWICMD_FAST 0`, rebuild |
| 5 | F | Existing QSCAN, ADC path regression | QUAD module, scope | Shared loop broke: see F.4 |
| 6 | D | `QSCAN` with the electrometer source | Electrometer wired to LT | `SQSSRC,1,ADC` or `SQSETRG,1,TWI` |
| 7 | E | `RFAACQ` with the electrometer | Host script | `SQSSRC,1,ADC` |
| 8 | I | ADC m/z input with a digital strobe | RF amp module, a signal source on a MIPS ADC input, a pulse source on a digital input | `SRFAMZSTB,<module>,NA` and `SRFAMZADC,<module>,-1,1,0` |
| 9 | G | Ion test | Mass spectrometer | See handoff section 5 |

Part F runs before D because `QUADscanAcquire` is now shared by both sources, so an ADC path that no
longer passes F means the shared change is at fault, not the electrometer.

**Fallback switches, none of which need a code change except the first:**

| Switch | Effect |
|---|---|
| `#define TWICMD_FAST 0` in `src/Serial.cpp` | Channel 0 `TWICMD` back to the original loop |
| `SQSSRC,<module>,ADC` | `QSCAN` / `RFAACQ` back to the MIPS ADC |
| `SQSETRG,<module>,TWI` | Electrometer triggered over TWI, LT is never touched |

---

## 0.1 Part 0 — Prerequisites

- [ ] **MIPS firmware:** build from `master` HEAD (`~/.platformio/penv/bin/pio run`) and flash. The
      version string still says 1.268, so record `git log -1 --oneline` in the notes.
      *Result: NOT verified. October 17, 2026: the lab MIPS reports Version 1.268, Sept 14, 2026 and has the TWI1CMD fail-fast code, but the build commit was not recorded and it has no RF amp module, so the QUAD parts could not be run.*
- [x] **Electrometer firmware:** build `../IsolatedElectrometer` from its working tree or latest commit and
      flash. It **must** contain `TWI_READ_TRIGDATA` (0x8B). Check with the electrometer's own
      `GVER`, and with the TWI read in D.1 below.
      *Result: Verified October 17, 2026: the electrometer was flashed with `37eb120` (the `0x8B` command and the sample rate fix), `GVER` answers, and D.1 passed.*
- [x] **Wiring:** electrometer TRIGIN to MIPS **LT (pin 13)**. Electrometer on TWI1 (Wire1) at
      address 32 (0x20), the factory default.
      *Result: Verified October 17, 2026: the Wire1 scan (`TWI1SCAN`) shows one device at `0x20`; 60 of 60 LT pulses (30 of a few microseconds, 30 of 1 mS, positive from LOW) triggered the electrometer, and 3 pulses during a burst were counted in `GTRIGMIS` without starting a second burst.*
- [ ] **Instruments:** scope or logic analyzer, 3 channels if possible: LT, quad RF or the TRGOUT
      pin, and SDA1 (pin 70) or SCL1 (pin 71).
- [ ] **Baseline numbers** to compare against (host side, 200 iteration loop, so they include the
      serial round trip): `TWI1CMD,32,RADC` was 6.64 mS before, 3.9 mS after at 100 kHz and
      2.5 mS at 400 kHz. `GTWI,0120A125` was 1.36 mS at 400 kHz. Re-measure; a different
      computer or serial path shifts these.
      *Result: Gordon measured, October 16 to 17, 2026: `TWI1CMD,32,RADC` 6.64 mS before, 3.9 mS after at 100 kHz, 2.5 mS at 400 kHz; `GTWI,0120A125` 1.36 mS at 400 kHz. Not re-measured on this computer.*
- [x] **Record the Wire1 speed:** `GTWISPEED,1`. Note it prints the variable, not the hardware clock.
      *Result: October 17, 2026: `GTWISPEED,1` = 100000.*

---

## A. `TWI1CMD` reply reading (`TWIreadLine`)

Already measured faster. This is correctness, edge cases and the bus behaviour.

- [x] **A.1 Reply content.** `TWI1CMD,32,RADC` returns the same value text as before the change. Run
      several different commands (`GVER`, `GMODE`, a `S...` setter that returns only ACK).
      *Result: Verified October 17, 2026 over USB: `TWI1CMD,32,GVER`, `GMODE`, `GTRIGST`, `RADC` and the setters `SSRATE,10000`, `SNUMP,100`, `SMODE,1`, `CONFIG` all returned correct replies. There is no old build to compare against.*
- [ ] **A.2 ACK-only commands.** A setter returns promptly with its ACK, with no 250 mS wait.
      *Result: Partly: the setters returned their ACK. How promptly was not measured (a first attempt used a flawed timing helper and was discarded).*
- [ ] **A.3 Missing device.** `TWI1CMD,99,RADC` returns `?` immediately, not after 250 mS.
      *Result: Partly: `TWI1CMD,99,RADC` returned `?`, the fail-fast reply (the old code said nothing). How quickly was not measured.*
- [x] **A.4 Long reply.** A command whose reply is over 10 bytes (e.g. the version string) comes back
      complete and in order.
      *Result: Verified October 17, 2026: the electrometer `GVER` reply (`IsoElectrometer, version 1.0 Sept 9, 2025`, 42 characters) came back complete and in order.*
- [x] **A.5 Timing.** Run the 200 iteration loop (script below). Compare with the baseline.
      *Result: Measured by Gordon, October 16 to 17, 2026, with the 200 iteration loop: 6.64 mS before, 3.9 mS after at 100 kHz.*
- [ ] **A.6 400 kHz.** `STWISPEED,1,400000`, repeat A.1 and A.5. Then **check the other Wire1
      modules still read correctly** (DCbias, DCBcurrent, FAIMSfb, level detector, ESI, whichever are
      installed). 400 kHz is global to Wire1. Restore with `STWISPEED,1,100000` if anything misreads.
      *Result: Partly: Gordon measured 2.5 mS at 400 kHz. Not checked: that the other Wire1 modules still read correctly at 400 kHz.*
- [ ] **A.7 Bus view (scope on SDA1/SCL1).** For a `"\n"` reply: START, address+R, a few bytes, NACK,
      STOP, with no 32 byte read. **Record whether an extra byte follows the `\n`** and note it in the
      handoff open items.
- [ ] **A.8 Not ready.** A slave that is slow to answer returns 0 from the first poll and the loop
      retries every 200 µS until the reply arrives (within the 250 mS limit).
- [x] **A.9 Stuck bus.** After any failure above, `GTWI` reads still work.
      *Result: Verified October 17, 2026: after `TWI1CMD` to the absent address 99, `TWI1CMD` and `GTWI,01208B72` still worked.*

Timing loop (MIPS host script):

```javascript
StartT = new Date()
for(i=0;i<200;i++)
{
   mips.SendMess("MIPS","TWI1CMD,32,RADC\n")
}
StopT = new Date()
res = "Scan acquire time, ms = " + (StopT.getTime() - StartT.getTime())
```

---

## C. `GTWI` / `STWI` fixes

- [ ] **C.1 Board 0 on Wire.** `GTWI,0020A125` (Wire, board A, address 0x20) now works. Before the fix it
      returned BADARG for board 0. Use any Wire device you have. The hex string is: one byte holding the
      board in the high nibble and the channel in the low nibble (0 = Wire, 1 = Wire1), then the device
      address, then the command byte(s), then one byte holding the read count in the high nibble and the
      type in the low nibble (1 bool, 2 int, 3 char, 4 byte, 5 word, 6 float).
- [ ] **C.2 Board 1 on Wire.** `GTWI,1020...` still works.
- [x] **C.3 Wire1.** `GTWI,0120A125` unchanged.
      *Result: Verified October 17, 2026: `GTWI,01208B72` (Wire1, address 0x20) reads the electrometer.*
- [x] **C.4 Long read.** A `GTWI` string asking for more than 4 bytes (high nibble of the last byte, e.g.
      `...82`) no longer corrupts the stack: the board stays up and the next command answers.
      *Result: Verified October 17, 2026: `GTWI,01208B72` reads 7 bytes, more than the old 4 byte buffer; the MIPS stayed up and every following command answered.*
- [ ] **C.5 Bad board.** `GTWI,2020...` returns BADARG.
- [ ] **C.6 `STWI`.** Same board 0 / board 1 / bad board checks with `STWI`.

---

## B. `TWICMD` channel 0

Build is `TWICMD_FAST 1`. Needs a device on the Wire bus behind the board multiplexer that speaks the
`0x7F` text protocol (a module with text commands, e.g. an RF driver or FAIMS board).

- [ ] **B.1 Reply content**, board A and board B: `TWICMD,<brd>,<addr>,<command>` returns what it did before.
- [ ] **B.2 Missing device.** `TWICMD,0,99,X` returns `?` at once.
- [ ] **B.3 Lock release.** Send B.2 about 50 times, then run something that uses Wire through the TWI
      queue (`SAVE` of module settings, an RF amp update, any module read). If those stop working or
      the front panel stops updating, `TWIbusy` was left set: **set `TWICMD_FAST 0` and report it.**
- [ ] **B.4 Mux.** After a `TWICMD` to board B, normal operation on board A is unaffected.
- [ ] **B.5 Timing**, compare with the same command at `TWICMD_FAST 0` if you build both.
- [ ] **B.6 No interference** with interrupts: leave a scan or an ISR driven module running during B.1.

---

## D. `QSCAN` with the electrometer source

Setup of the electrometer through the MIPS (all via `TWI1CMD,32,...`, **arguments comma separated**, e.g.
`TWI1CMD,32,SMODE,1`): `SMODE,1`, then `SNUMP` and `SSRATE` for the burst you want, then `CONFIG` to apply them,
`SGAIN`, `SBIAS`, `ZERO` as needed.

**Electrometer sample rate.** Firmware built before October 17, 2026 (before commit "Fix the sample rate being one third of
SSRATE") ran at **one third** of `SSRATE`, so bursts took 3 times the nominal time; make sure the electrometer has the fixed
firmware (`GSRATEA` is then correct, and a 1000 point burst at `SSRATE,10000` takes 101 mS). With the fix, burst time is
`SNUMP / SSRATE` (plus about 1 mS) for `SSRATE` up to **100000**. Above 100000 the ADC loses events (2x the nominal burst time
at 120 to 200 kHz, 3x at 250 kHz) and `GSRATEA` does not show it, so stay at or below 100000 until the ADC clock is raised.
The default `SQSETMO` of 1000 mS covers a 5000 point burst at 10000 (500 mS). Choose a **short burst first** (e.g. `SNUMP,100`,
`SSRATE,10000`, about 11 mS) so the scan is quick. Then on the MIPS:
`SQSSRC,1,ELECT`, `QSCANSTAT,1`.

- [x] **D.1 Electrometer new command.** Confirm the electrometer firmware has 0x8B: `GTWI,01208B72` (Wire1,
      address 0x20, command 0x8B, read 7 bytes, print as int) returns without NAK. `GTWI` prints only the
      first 4 of the 7 bytes as an int, so the low 24 bits are the completed count. Trigger it
      (`TWI1CMD,32,STRIG`) and read again: the count increments. The sum bytes are checked by the scan
      itself in D.5.
      *Result: Verified October 17, 2026, 5 triggers: `TWI1CMD,32,STRIG` then `GTWI,01208B72`; the count (low 24 bits) equalled `GTRIGCNT` and sum byte 0 equalled `RSUM & 0xFF` every time (5 of 5). Sum bytes 1 to 3 are not visible through `GTWI`; the scan (D.5) checks them.*
- [ ] **D.2 Status.** `QSCANSTAT,1` shows `Acquire source ELECT`, address 32, trigger HW, timeout 1000.
      `GQSSRC`, `GQSEADD`, `GQSETRG`, `GQSETMO` agree.
- [ ] **D.3 Preflight failures** — each must return NAK **before any header byte** and `GERR` must report
      the code. The scan must not move the quad.
  - [ ] electrometer unplugged, or `SQSEADD,1,40` pointing at nothing: 133
  - [ ] electrometer in mode 0 or 2 (`SMODE 0`): 133
  - [ ] electrometer in mode 1 but not armed (right after `SMODE`, before it re-arms, if reproducible): 133
  - [ ] `SPROTOG` with the L input active, then `QSCAN` with `SQSETRG HW`: 135. Then `QSCAN` with
        `SQSETRG TWI` runs. Stop the profile toggle and 135 goes away.
- [ ] **D.4 Timing on the scope** (LT, quad RF or `TRGOUT` with `SQSTRIG TRUE`, optionally SDA1):
  - [ ] one 50 µS LT pulse per point, **after** the dwell, never before the quad has settled
  - [ ] the electrometer burst falls inside the same point (the pulse starts it; nothing else does)
  - [ ] the next point's m/z change starts after the previous burst finishes, not during it
- [ ] **D.5 Hardware trigger run.** `SQSSTRT/STOP/STEP` for about 20 points, `QSCAN,1`. Header, N points of 4
      bytes each, trailer. Point values are sensible (a steady source gives similar sums).
- [ ] **D.6 Electrometer counters.** `TWI1CMD,32,GTRIGCNT` equals the point count, `GTRIGMIS` is 0. A
      non zero `GTRIGMIS` means triggers arrived mid-burst (point time is shorter than the burst).
- [ ] **D.7 TWI trigger.** `SQSETRG,1,TWI`, repeat D.5/D.6. Same count and similar sums. Note the extra
      latency (about one short TWI write) on the scope.
- [ ] **D.8 Timeout.** Set `SNUMP` large (e.g. 5000 at 1000, a 5 s burst) and `SQSETMO,1,50`. `QSCAN` aborts with
      the trailer (`0xEB`), `GERR` reports 134, and the next `QSCAN` works after restoring both.
- [ ] **D.9 Abort.** ESC (0x1B) mid scan aborts with a trailer; a following scan runs normally.
- [ ] **D.10 Several scans.** `SQSNUM,1,3`: three header/trailer frames. The count keeps rising.
- [ ] **D.11 Park.** After the scan the quad is back at start m/z, and LT is LOW (scope, or the L LED is off).
- [ ] **D.12 Compare with the MIPS ADC.** Same dwell, same source, `SQSSRC ADC` vs `ELECT`: the peaks should
      line up in m/z. Heights differ (different front ends). Note the electrometer sum scale.
- [ ] **D.13 Ethernet.** With the ethernet interface connected and in use, run D.5 about 20 times. **LT is the
      ethernet config line and flashing it every 100 mS once broke the link** (RevisionHistory 1.236).
      Watch the link stays up. If it drops: use `SQSETRG,<module>,TWI` and report it.
- [ ] **D.14 Wire1 speed.** Repeat D.5 at 100 kHz and 400 kHz and note the per point overhead; the poll is
      a 9 byte transaction (about 1 mS and 0.25 mS respectively).
- [ ] **D.15 ADC path untouched.** `SQSSRC,1,ADC`, `QSCAN` behaves as in Part F.

---

## E. `RFAACQ` with the electrometer

Uses the host script that already drives `RFAACQ` for the ADC. Add `SQSSRC,<module>,ELECT` before it and
`SQSEINIT,<module>` once before the first point of each scan.

- [ ] **E.1 Without init.** `SQSSRC,1,ELECT` then `RFAACQ,1,100,0,5` **without** `SQSEINIT`: NAK, `GERR`
      reports 133.
- [ ] **E.2 First call.** After `SQSEINIT,1`, the first `RFAACQ` returns `0`.
- [ ] **E.3 Pipeline.** Each later call returns the previous point's sum. Step `mz` and check the returned
      values trail the commanded points by one, the same as the ADC path.
- [ ] **E.4 Failure marker.** Unplug the electrometer mid scan (or let it time out): the call returns `-1`.
      `SQSEINIT` after reconnecting recovers.
- [ ] **E.5 Re-init between scans.** Run two scans back to back, each with its own `SQSEINIT`. The second
      scan's first call returns 0, not the last point of the first scan.
- [ ] **E.6 Settings reset it.** Changing `SQSSRC`, `SQSEADD` or `SQSETRG` clears the initialised state; the
      next `RFAACQ` NAKs until `SQSEINIT`.
- [ ] **E.7 Overlay.** A host `RFAACQ` scan and a firmware `QSCAN` over the same range with the same cal
      table should overlay.

---

## I. ADC m/z input with a digital strobe

New capability (commit titled "RF amp: add an ADC m/z input applied by a digital strobe"): the m/z comes from an analog
voltage on a MIPS ADC input and a digital input strobe applies it. Per module, RAM only, in `src/RFAmzInput.cpp`.

`SRFAMZADC,module,adc,slope,intercept` (adc 0 to 9, -1 for none; m/z = counts * slope + intercept, 12 bit counts, 0 to
4095 for 0 to 3.3 V at the Due pin, so a full scale of 40000 is slope 9.768 and intercept 0), `SRFAMZSTB,module,Q..X|NA`
(rising edge, physical, before any `DIinvert`), and the getters `GRFAMZADC`, `GRFAMZSTB`, `GRFAMZSTAT`
(requested m/z, applied m/z, applied count, rejected count, stale count, missed count).

The strobe interrupt latches the ADC (4 reads averaged), because the value is only held for about 10 mS. A 10 mS
thread then applies it: m/z must be 10 to 40000 (otherwise ignored), the mass calibration table is applied if
enabled and the m/z must be inside the range `QSCAN` allows, and a module that is enabled is updated at once with the
same call `QSCAN` uses. Do not use it while a scan is running. Rev 1/2 modules need `SRFADCCH` set or the commands NAK.

- [ ] **I.1 Commands.** Defaults: `GRFAMZADC,1` = `-1,1.000000,0.000000`, `GRFAMZSTB,1` = `NA`, `GRFAMZSTAT,1` all zero.
      Range checks NAK: `SRFAMZADC,1,10,1,0`, `SRFAMZSTB,1,A`, module 2 on a one module system. On a Rev 1/2 module with
      no resolving DC channel they NAK with `GERR` = 132 (`ERR_QUADNODCBCHAN`).
- [ ] **I.2 Setup.** A variable voltage source on the ADC input (0 to 3.3 V at the pin, your own divider), a 10 mS pulse
      source on a digital input Q to X. `SRFAMZADC,1,<adc>,9.768,0`, `SRFAMZSTB,1,<input>`.
- [ ] **I.3 Basic apply.** Set the voltage, pulse the strobe once. `GRFAMZ,1` shows the new m/z, the RF level, pole bias and
      resolving DC change on the module, and `GRFAMZSTAT,1` shows `requested,applied` equal (table disabled) and applied
      count 1. Changing the voltage **without** a strobe does nothing.
- [ ] **I.4 The value is taken at the edge.** With a 10 mS pulse, change the voltage right after the rising edge (1 mS later).
      The applied m/z matches the voltage **at the edge**, not the new one. Repeat with the voltage changing a few mS before
      the edge to see where the latch point is. This is the point of reading the ADC in the interrupt.
- [ ] **I.5 Rate.** Strobe at 10, 20 and 50 per second with a new voltage each time. Every strobe is applied (the applied
      count equals the number sent, `rejected`, `stale` and `missed` stay 0). Note the highest rate that keeps up.
- [ ] **I.6 Rejections.** Request an m/z below 10 and above 40000: the strobe is ignored, the module m/z does not change, the
      rejected count goes up. With the calibration table enabled, an m/z far outside the table (further than `QSCAN`
      allows) is also rejected.
- [ ] **I.7 Calibration table.** Enable a table (`SQCENA`, `SQCNUM`, `SQCPNT`). A strobe commands the calibrated m/z and applies
      its resolving DC offset: `GRFAMZ,1` is the commanded value, and the result matches a `QSCAN` point at the same m/z.
      With the table disabled the commanded value equals the requested one.
- [ ] **I.8 Module disabled.** `SRFAENA,1,OFF`, strobe: the m/z is stored (`GRFAMZ,1`), nothing is written to the RF output.
      Enable the RF and the setpoint corresponds to the stored m/z.
- [ ] **I.9 Not during a scan.** Strobe repeatedly while a `QSCAN` runs: the scan is undisturbed, and **no strobe is applied
      during or after it** (the module ends parked at the scan start m/z). With a `QSCAN` that uses the MIPS ADC the
      `missed` count goes up because the digitizer owns the ADC.
- [ ] **I.10 Interactions.** `SRFAMZ` and the front panel m/z still work; the next strobe overrides them. `SRFAMZSTB,1,NA`
      stops the strobe. `SRFAMZADC,1,-1,1,0` stops the latch. Re-running either command with new values takes effect
      immediately.
- [ ] **I.11 Two modules.** Different ADC channels and strobe inputs for modules 1 and 2 act independently.
- [ ] **I.12 Other timing.** With the strobe running at 50 per second, other time critical outputs are unaffected (the
      strobe interrupt reads the ADC, a few tens of microseconds). Check a scan trigger, a compressor trigger or an ARB
      output on a scope if you use them.
- [ ] **I.13 Power cycle.** The settings are gone (RAM only). They must be sent again.

---

## G. Ion test

The standing acceptance test from the handoff (section 5, "Ion testing"). With the electrometer source it
doubles as the check that the integer sum scale and polarity are what the host expects.

---

## H. Sign-off

| Part | Result | Date | Notes |
|---|---|---|---|
| 0 Prerequisites | partial | 2026-10-17 | electrometer firmware, wiring and Wire1 speed done; MIPS build commit not recorded; no scope used |
| A `TWI1CMD` | partial | 2026-10-17 | A.1, A.4, A.5 (Gordon), A.9 done; A.2, A.3, A.6 partly; A.7, A.8 not done; extra byte after `\n` not observed (no scope) |
| C `GTWI` / `STWI` | partial | 2026-10-17 | C.3, C.4 done; C.1, C.2, C.5, C.6 need a device on the Wire channel |
| B `TWICMD` channel 0 | not done | | `TWICMD_FAST` left at: 1 |
| F Existing QSCAN regression | | | |
| D `QSCAN` electrometer | partial | 2026-10-17 | D.1 done; D.2 onwards need an RF amp module (none installed on the lab MIPS) |
| E `RFAACQ` electrometer | not done | | needs an RF amp module |
| I ADC m/z input with strobe | not done | | needs an RF amp module; latch point seen at the edge? |
| G Ion test | | | |

Record anything unexpected in the handoff's open items table, with the commit from `git log -1`.

---

# Part F — External DAQ features (earlier test plan, unchanged)

This is the earlier test plan, kept as it was. Its headings are now numbered F.1 to F.5, so where the text
refers to sections "3.2", "3.4" or "3.5", read F.3.2, F.3.4 and F.3.5.

## F.1 What changed

| Field / command | Default | Purpose |
|---|---|---|
| `StartPin` / `SQSSTPIN`, `GQSSTPIN` | `NA` (disabled) | Output pulsed once at the **start of every scan** (every repeat under `SQSNUM`, not just once per `QSCAN` command). Channel `A`..`P`, or `NA` to disable. Fixed ~200 µs pulse (`QUADscanSTARTPULSEuS` in `QUADscan.cpp`), driven through `SetOutput`/`ClearOutput` — the same SPI-based digital output bank used elsewhere in the firmware (interlock, FAIMS, compressor switch, Twave), **not** the dedicated `TRGOUT` pin. |
| `AcqEna` / `SQSADCENA`, `GQSADCENA` | `TRUE` | `TRUE` is the original behavior — the MIPS ADC acquires and streams every point. `FALSE` steps through m/z on the dwell timer only and never touches the ADC, for a scan driven entirely by an external DAQ system synced off `SQSSTPIN` and/or the existing `SQSTRIG` per-point pulse. |
| `FrameOnNoAcq` / `SQSFRAME`, `GQSFRAME` | `FRAME` | Only takes effect when `SQSADCENA` is `FALSE`. `FRAME` still streams the header/trailer per scan (no point payload) for host bookkeeping. `NONE` streams nothing at all during the scan. Ignored — framing is always sent — when `SQSADCENA` is `TRUE`. |

All three default to the pre-existing behavior, so nothing changes for any host software
that doesn't touch the new commands.

`QSCANSTAT` now reports all three settings (start pin, ADC acquire, and the frame mode —
the frame line only appears when ADC acquire is `FALSE`).

---

## F.2 Setup

- MIPS controller with a Rev 1 QUAD module installed and enabled — the same unit already
  used to validate the base QSCAN feature.
- Oscilloscope or logic analyzer with at least 2 channels: one on `TRGOUT`, one on
  whichever digital output channel is used for the start pulse (test uses `A`).
- Serial terminal to the MIPS command port.
- `QUADcalTable`/scan range set up as in the existing QSCAN validation (or defaults:
  50–500 m/z, step 1, dwell 2 ms).

---

## F.3 Test procedure

### 3.1 Build

- [ ] `pio run -e dueUSB` completes with `[SUCCESS]` and no warnings introduced by this
      change.
- [ ] Flash the firmware to the test unit.

### 3.2 Regression — existing behavior unchanged

Purpose: confirm the `QUADscanGo` refactor (splitting the point loop into
`QUADscanRunScanADC` / `QUADscanRunScanNoADC`) didn't disturb the already-validated path.

- [ ] With defaults (`SQSADCENA` `TRUE`, `SQSSTPIN` `NA`), run the same `QSCAN` sequence
      already used to validate Rev 1.
- [ ] Byte stream (header/points/trailer) matches prior behavior.
- [ ] Per-point timing on the scope matches `SQSDWELL` as before, no new jitter.
- [ ] `SQSTRIG TRUE` still pulses `TRGOUT` once per point exactly as before.
- [ ] After the scan completes, the module has moved back to `StartMZ` (`GRFAMZ`/front
      panel or equivalent) rather than sitting at `StopMZ` where the scan finished.

### 3.3 Start-of-scan pulse

- [ ] `SQSSTPIN 1,A`
- [ ] `SQSNUM 1,3`
- [ ] `QSCAN 1`
- [ ] Scope channel A: one pulse (~200 µs) at the start of each of the 3 scans, none
      mid-scan.
- [ ] Pulse occurs *before* the first point's m/z move (i.e. before `TRGOUT`/DAC activity
      for point 0 of that scan).
- [ ] `GQSSTPIN 1` reports `A`.
- [ ] `SQSSTPIN 1,NA`, rerun `QSCAN 1`: no pulse on channel A.
- [ ] `SQSSTPIN 1,Z` (out of range) is rejected (NAK / `BADARG`).
- [ ] `QSCANSTAT 1` shows the current start pin correctly in both states.

### 3.4 ADC disabled, FRAME mode

- [ ] `SQSADCENA 1,FALSE`
- [ ] `SQSFRAME 1,FRAME`
- [ ] `QSCAN 1`
- [ ] Serial capture shows a header immediately followed by a trailer for each scan, with
      **no point bytes** in between.
- [ ] `TRGOUT` (`SQSTRIG TRUE`) and the start pin (`SQSSTPIN`) still pulse correctly and at
      the right point in each scan.
- [ ] Per-point dwell timing on the scope still matches `SQSDWELL` (no ADC acquisition
      time added — total time per point should be *shorter* than the ADC-enabled case).
- [ ] `GQSADCENA 1` reports `FALSE`; `GQSFRAME 1` reports `FRAME`.
- [ ] `QSCANSTAT 1` shows `ADC acquire FALSE` and the frame line as `FRAME`.
- [ ] After the scan completes, the module has moved back to `StartMZ`, same as the
      ADC-enabled case (this step runs unconditionally, not gated on `SQSADCENA`).

### 3.5 ADC disabled, NONE mode

- [ ] `SQSFRAME 1,NONE`
- [ ] `QSCAN 1`
- [ ] Nothing is written to the serial port during the scan (beyond the initial ACK) —
      confirm with a raw serial capture, not just the terminal display.
- [ ] Scope still shows correct start-pulse and per-point (`SQSTRIG`) pulsing and timing.
- [ ] `ESC` (0x1B) sent mid-scan still aborts cleanly (watchdog/abort-poll path is
      unaffected by the ADC-disabled branch).
- [ ] `QSCANSTAT 1` shows the frame line as `NONE`.
- [ ] After the scan completes, the module has moved back to `StartMZ`, confirming the
      park-back step still runs even with nothing streamed during the scan.

### 3.6 Re-enable and confirm clean return to normal mode

- [ ] `SQSADCENA 1,TRUE`
- [ ] `QSCAN 1` streams a full spectrum again exactly as in 3.2.

---

## F.4 Known risk areas to watch

- **Shared output bank.** `SetOutput`/`ClearOutput` drive the same shadow register
  (`DOmsb`/`DOlsb`) used by other subsystems (interlock, FAIMS, compressor switch,
  Twave). Confirm the chosen `SQSSTPIN` channel isn't wired to one of those on this unit
  before testing, or the pulse will also toggle that signal.
- **Priming-point abort path.** If the first point's acquisition fails with `SQSADCENA
  TRUE`, `TRGOUT` is left high until the end-of-scan safety reset — this is unchanged
  from the original code, but worth confirming the refactor preserved it (scope `TRGOUT`
  across a deliberately provoked timeout if practical).
- **Park-back was merged in from a separate commit.** The "return to `StartMZ` after the
  scan" behavior (`a787478`, "Quad scan initial delay added") landed on `origin/master`
  from another machine while this work was in progress, and had to be manually reconciled
  with the `QUADscanGo` refactor during a rebase. It was moved out from under the old
  `ADCstop();` call and now runs unconditionally after `if(sp->AcqEna) ADCstop();`, so it
  fires for both ADC-enabled and ADC-disabled scans — see the 3.2/3.4/3.5 checks above.

---

## F.5 Sign-off (external DAQ features)

| Section | Result | Date | Notes |
|---|---|---|---|
| 3.1 Build | | | |
| 3.2 Regression | | | |
| 3.3 Start pulse | | | |
| 3.4 ADC off, FRAME | | | |
| 3.5 ADC off, NONE | | | |
| 3.6 Re-enable | | | |
