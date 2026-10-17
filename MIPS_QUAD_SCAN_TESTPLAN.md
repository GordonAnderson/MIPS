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
| 8 | G | Ion test | Mass spectrometer | See handoff section 5 |

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
- [ ] **Electrometer firmware:** build `../IsolatedElectrometer` from its working tree or latest commit and
      flash. It **must** contain `TWI_READ_TRIGDATA` (0x8B). Check with the electrometer's own
      `GVER`, and with the TWI read in D.1 below.
- [ ] **Wiring:** electrometer TRIGIN to MIPS **LT (pin 13)**. Electrometer on TWI1 (Wire1) at
      address 32 (0x20), the factory default.
- [ ] **Instruments:** scope or logic analyzer, 3 channels if possible: LT, quad RF or the TRGOUT
      pin, and SDA1 (pin 70) or SCL1 (pin 71).
- [ ] **Baseline numbers** to compare against (host side, 200 iteration loop, so they include the
      serial round trip): `TWI1CMD,32,RADC` was 6.64 mS before, 3.9 mS after at 100 kHz and
      2.5 mS at 400 kHz. `GTWI,0120A125` was 1.36 mS at 400 kHz. Re-measure; a different
      computer or serial path shifts these.
- [ ] **Record the Wire1 speed:** `GTWISPEED,1`. Note it prints the variable, not the hardware clock.

---

## A. `TWI1CMD` reply reading (`TWIreadLine`)

Already measured faster. This is correctness, edge cases and the bus behaviour.

- [ ] **A.1 Reply content.** `TWI1CMD,32,RADC` returns the same value text as before the change. Run
      several different commands (`GVER`, `GMODE`, a `S...` setter that returns only ACK).
- [ ] **A.2 ACK-only commands.** A setter returns promptly with its ACK, with no 250 mS wait.
- [ ] **A.3 Missing device.** `TWI1CMD,99,RADC` returns `?` immediately, not after 250 mS.
- [ ] **A.4 Long reply.** A command whose reply is over 10 bytes (e.g. the version string) comes back
      complete and in order.
- [ ] **A.5 Timing.** Run the 200 iteration loop (script below). Compare with the baseline.
- [ ] **A.6 400 kHz.** `STWISPEED,1,400000`, repeat A.1 and A.5. Then **check the other Wire1
      modules still read correctly** (DCbias, DCBcurrent, FAIMSfb, level detector, ESI, whichever are
      installed). 400 kHz is global to Wire1. Restore with `STWISPEED,1,100000` if anything misreads.
- [ ] **A.7 Bus view (scope on SDA1/SCL1).** For a `"\n"` reply: START, address+R, a few bytes, NACK,
      STOP, with no 32 byte read. **Record whether an extra byte follows the `\n`** and note it in the
      handoff open items.
- [ ] **A.8 Not ready.** A slave that is slow to answer returns 0 from the first poll and the loop
      retries every 200 µS until the reply arrives (within the 250 mS limit).
- [ ] **A.9 Stuck bus.** After any failure above, `GTWI` reads still work.

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
- [ ] **C.3 Wire1.** `GTWI,0120A125` unchanged.
- [ ] **C.4 Long read.** A `GTWI` string asking for more than 4 bytes (high nibble of the last byte, e.g.
      `...82`) no longer corrupts the stack: the board stays up and the next command answers.
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

Setup of the electrometer through the MIPS (all via `TWI1CMD,32,...`): `SMODE 1`, then `SNUMP` and
`SSRATE` for the burst you want, `SGAIN`, `SBIAS`, `ZERO` as needed. Choose a **short burst first**
(e.g. `SNUMP 100`, `SSRATE 10000` is about 10 mS) so the scan is quick. Then on the MIPS:
`SQSSRC,1,ELECT`, `QSCANSTAT,1`.

- [ ] **D.1 Electrometer new command.** Confirm the electrometer firmware has 0x8B: `GTWI,01208B72` (Wire1,
      address 0x20, command 0x8B, read 7 bytes, print as int) returns without NAK. `GTWI` prints only the
      first 4 of the 7 bytes as an int, so the low 24 bits are the completed count. Trigger it
      (`TWI1CMD,32,STRIG`) and read again: the count increments. The sum bytes are checked by the scan
      itself in D.5.
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
- [ ] **D.8 Timeout.** Set `SNUMP` large (e.g. 5000 at a slow rate) and `SQSETMO,1,50`. `QSCAN` aborts with
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

## G. Ion test

The standing acceptance test from the handoff (section 5, "Ion testing"). With the electrometer source it
doubles as the check that the integer sum scale and polarity are what the host expects.

---

## H. Sign-off

| Part | Result | Date | Notes |
|---|---|---|---|
| 0 Prerequisites | | | |
| A `TWI1CMD` | | | extra byte after `\n` observed? |
| C `GTWI` / `STWI` | | | |
| B `TWICMD` channel 0 | | | `TWICMD_FAST` left at: |
| F Existing QSCAN regression | | | |
| D `QSCAN` electrometer | | | ethernet link survived? |
| E `RFAACQ` electrometer | | | |
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
