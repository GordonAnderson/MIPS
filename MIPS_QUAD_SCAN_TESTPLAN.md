# MIPS QUAD scan and TWI — lab test roadmap

Ordered bench plan for everything not yet verified on hardware. Written October 16, 2026 for a
lab session on a different computer; read [`MIPS_QUAD_FIRMWARE_HANDOFF.md`](MIPS_QUAD_FIRMWARE_HANDOFF.md)
section 2a for the background and the design reasons behind each item.
The commands for each data collection mode are in [`MIPS_QUAD_DATA_COLLECTION.md`](MIPS_QUAD_DATA_COLLECTION.md).

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
      *Result: NOT verified. October 17, 2026: the lab MIPS reports Version 1.268, Sept 14, 2026 and has the TWI1CMD fail-fast code, but the build commit was not recorded. Later the same day a QUAD (RF amp module 1) was connected and Parts F, D and E were run on it by serial.*
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
- [x] **D.2 Status.** `QSCANSTAT,1` shows `Acquire source ELECT`, address 32, trigger HW, timeout 1000.
      `GQSSRC`, `GQSEADD`, `GQSETRG`, `GQSETMO` agree.
      *Result: Verified October 17, 2026 with an RF amp module (module 1) installed.*
- [ ] **D.3 Preflight failures** (partly done October 17, 2026: the three 133 cases below passed, each a NAK with no header byte, `GERR` 133 and the quad not moved; the not-armed and 135 cases were not run) — each must return NAK **before any header byte** and `GERR` must report
      the code. The scan must not move the quad.
  - [x] electrometer unplugged, or `SQSEADD,1,40` pointing at nothing: 133 (`SQSEADD,1,40`)
  - [x] electrometer in mode 0 or 2 (`SMODE 0`): 133 (both mode 0 and mode 2)
  - [ ] electrometer in mode 1 but not armed (right after `SMODE`, before it re-arms, if reproducible): 133
  - [ ] `SPROTOG` with the L input active, then `QSCAN` with `SQSETRG HW`: 135. Then `QSCAN` with
        `SQSETRG TWI` runs. Stop the profile toggle and 135 goes away.
- [ ] **D.4 Timing on the scope** *(October 17, 2026: not possible on this controller, LT is not routed to the proper pin; see D.5. The TWI-trigger timing was not scoped.)* (LT, quad RF or `TRGOUT` with `SQSTRIG TRUE`, optionally SDA1):
  - [ ] one 50 µS LT pulse per point, **after** the dwell, never before the quad has settled
  - [ ] the electrometer burst falls inside the same point (the pulse starts it; nothing else does)
  - [ ] the next point's m/z change starts after the previous burst finishes, not during it
- [ ] **D.5 Hardware trigger run.** *Result: FAILED October 17, 2026. 21 points, 5 mS dwell: header, no points, abort trailer `0xEB`, `GERR` 134. `GTRIGCNT` and `GTRIGMIS` both +0, so the electrometer saw no edge at all. The same scan with `SQSETRG,1,TWI` passes (D.7). **Cause found by Gordon with a scope, October 17, 2026: this version of the MIPS controller does not route LT (pin 13) to the proper pin, so the hardware trigger cannot work on this system.** Not a firmware fault. D.4, D.5 and D.13 need a controller with LT wired correctly; use `SQSETRG,<module>,TWI` on this one. Re-checked with the TWI trigger after the electrometer was reconnected: 2 frames of 21 points, count +42, no misses.* `SQSSTRT/STOP/STEP` for about 20 points, `QSCAN,1`. Header, N points of 4
      bytes each, trailer. Point values are sensible (a steady source gives similar sums).
- [ ] **D.6 Electrometer counters.** *Result: October 17, 2026, TWI trigger only: count +21 for 21 points, +63 for 3 scans, +301 for 301 points, `GTRIGMIS` 0 every time. Hardware trigger: see D.5.* `TWI1CMD,32,GTRIGCNT` equals the point count, `GTRIGMIS` is 0. A
      non zero `GTRIGMIS` means triggers arrived mid-burst (point time is shorter than the burst).
- [x] **D.7 TWI trigger.** `SQSETRG,1,TWI`, repeat D.5/D.6. Same count and similar sums. Note the extra
      latency (about one short TWI write) on the scope.
      *Result: Verified October 17, 2026, 21 points, count +21, `GTRIGMIS` 0, sums 343,750 to 345,008 (steady source, 100 samples at 10 kS/s). Latency not scoped.*
- [x] **D.8 Timeout.** Set `SNUMP` large (e.g. 5000 at 1000, a 5 s burst) and `SQSETMO,1,50`. `QSCAN` aborts with
      the trailer (`0xEB`), `GERR` reports 134, and the next `QSCAN` works after restoring both.
      *Result: Verified October 17, 2026 (TWI trigger): `0xEB` trailer, 0 points, 134, and the next scan ran 21 of 21 points. **Wait about 6 s after the `CONFIG` that sets the long burst:** arming into mode 1 runs a burst with no trigger (electrometer TODO), and a scan started during it fails the preflight with 133 instead.*
- [x] **D.9 Abort.** ESC (0x1B) mid scan aborts with a trailer; a following scan runs normally.
      *Result: Verified October 17, 2026 (TWI trigger): ESC after 20 of 301 points gave an `0xEB` trailer, and the next full scan returned 301 points with `0xEA`.*
- [x] **D.10 Several scans.** `SQSNUM,1,3`: three header/trailer frames. The count keeps rising.
      *Result: Verified October 17, 2026 (TWI trigger): three frames, scan numbers 0 to 2, last flag only on the third, count +63.*
- [ ] **D.11 Park.** After the scan the quad is back at start m/z, and LT is LOW (scope, or the L LED is off).
      *Result: Partly: after every scan `GRFAMZ,1` read the start m/z (200) and the RF readback returned to about 334 V. LT level not checked.*
- [ ] **D.12 Compare with the MIPS ADC.** *Result: Partly, October 17, 2026: only the electrometer source was run over the full range, because ADC channel 0 had nothing connected. 200 to 1200 m/z, step 1, dwell 5 mS, TWI trigger, 100 sample bursts at 10 kS/s, gain 1: 1001 points in 17.6 s, count +1001, no misses. Raw sums sit near 344,800 with dips of up to about 10% (peaks appear inverted: the front end is inverting). Dips at m/z 543 to 547, 1000 to 1010 and others. A host-driven scan reading `RCUR` (electrometer polarity NEG) showed the same features as positive peaks (highest 0.82 at m/z 398, 489 to 490, 516 to 518, 543 to 546; baseline 0.27), so the sign comes right with NEG and `RCUR`. The streamed sum is raw, so sign and scale are for the host. ADC versus electrometer overlay not done.* Same dwell, same source, `SQSSRC ADC` vs `ELECT`: the peaks should
      line up in m/z. Heights differ (different front ends). Note the electrometer sum scale.
- [ ] **D.13 Ethernet.** *(October 17, 2026: not possible on this controller, the hardware trigger does not run; see D.5.)* With the ethernet interface connected and in use, run D.5 about 20 times. **LT is the
      ethernet config line and flashing it every 100 mS once broke the link** (RevisionHistory 1.236).
      Watch the link stays up. If it drops: use `SQSETRG,<module>,TWI` and report it.
- [ ] **D.14 Wire1 speed.** *Result: October 17, 2026, TWI trigger, 201 points, 10 sample bursts, dwell 0: 4.98 mS per point at 100 kHz and 5.00 to 5.03 mS at 400 kHz, so the bus speed is not what sets the point time here (something else costs about 5 mS per point; not investigated). The resolving DC and pole bias read the same at 400 kHz. Restored to 100 kHz.* Repeat D.5 at 100 kHz and 400 kHz and note the per point overhead; the poll is
      a 9 byte transaction (about 1 mS and 0.25 mS respectively).
- [x] **D.16 Where the time per point goes (RESOLVED October 17, 2026, see the conclusion at the end of this item).** *Gordon's measurements: an
      `RFAACQ` scan of **800 points** (the first `GQSESTAT` counted 801) with the electrometer takes 11 s (13.8 mS per
      point) with dwell 0 and a 10 point burst; the same scan with the MIPS ADC takes 3.3 s (4.1 mS per point). Looping the
      commands from the host: `TWI1CMD,32,GTRIGCNT` 6.39 mS per call, `GTWI,01208B72` 4.44 mS per call. So the host link,
      main loop and quad update cost at most 4.1 mS per point, a Wire1 transaction is cheap, and the electrometer path
      added about 9.6 mS per point. Cause not yet found.* *(Correction, October 18, 2026: these per point figures were
      first written assuming 1000 points and were 20% low. Dividing by 800 makes them 25% higher.)* Run the scan (QSCAN or an
      `RFAACQ` scan after `SQSEINIT,1`), then `GQSESTAT,1`. It returns
      `points,trigUs,waitUs,completeUs,polls,readUs,readMaxUs,retries,fails` for the last point, in microseconds, and it
      is reset by each `QSCAN` / `SQSEINIT`. Read it as follows:
      - `completeUs` much larger than the burst (`GNUMP / GSRATEA`) plus about 1000: the electrometer is slow to
        publish a finished burst. `polls` of 1 with a large `completeUs` is the same.
      - `retries` or `fails` above 0: TWI reads are failing and being retried, and each failure costs the I2C library
        timeout, which does not scale with the bus speed. This is the leading suspect for a fixed cost.
      - `readUs` / `readMaxUs` of about 1000 or more at 100 kHz (about 350 at 400 kHz) is plain bus time. Much larger means
        clock stretching by the electrometer.
      - `trigUs` is the trigger write; it should be about 300 us at 100 kHz.
      Also send `TWI1CMD,32,GSRATEA` and `TWI1CMD,32,GNUMP` to get the burst time, and check the electrometer has the
      sample rate fix (a burst should take `SNUMP / SSRATE`, not 3 times that).
      **`GQSESUM,1`** (added after the first readings, because `GQSESTAT` is the last point only) returns averages over
      the whole scan: `points,avgCompleteUs,minCompleteUs,maxCompleteUs,avgWaitUs,avgPolls x100,maxPolls,
      pointsNeedingMoreThanOnePoll`. `avgPolls x100` of 100 means every wait found the burst already finished. In a
      `QSCAN` (no host in the loop) the minimum and average completion time are the electrometer's real latency: the
      burst plus the time to publish it plus the poll granularity. In an `RFAACQ` scan they also contain the host's
      turnaround, so use `QSCAN` for the electrometer's own timing.
      *Readings, October 17, 2026 (Gordon), `GQSESTAT,1` after a scan with a 10 point burst: at 100 kHz Wire1
      `801,208,995,15377,1,989,992,0,0`; at 400 kHz `1603,69,341,8113,1,334,992,0,0`. So: the bus speed did change
      (trigger 208 to 69 us, read 989 to 334 us), there were no retried or failed reads, and the burst had finished at
      the first poll of the last point. `completeUs` is the last point only and in an `RFAACQ` scan contains the host gap,
      so it did not locate the remaining cost. One read in each run took the 100 kHz time (`readMaxUs` 992).*
      ***Conclusion (Gordon, October 17, 2026):** an `RFAACQ` scan with the electrometer at 400 kHz on Wire1, 50 kHz sample
      rate and 10 samples takes 3.7 s for the 800 point scan (4.6 mS per point), against 3.3 s (4.1 mS per point) with the MIPS ADC, so the
      electrometer adds about 0.5 mS per point, which is what the counters predict (a 69 us trigger plus a 334 us read is
      0.4 mS; the scan times are only good to 0.1 s, which is 0.1 mS per point). The earlier 9.4 to 13.8 mS per
      point (7.5 s and 11 s) was not a fault in the scan code. Burst settings changed with `SSRATE` / `SNUMP` do not take effect until
      `CONFIG` (so reducing `SNUMP` appeared to do nothing and the bursts stayed long), and the bus had been at 100
      kHz and not 400 kHz. The 4.1 mS floor (3.3 s for 800 points) is the `RFAACQ` command round trip, the script and the plotting. A `QSCAN` with the electrometer (TWI trigger, 400 kHz, 100 kHz sample rate, 10 samples) measured about 2.7 mS per point including the host acquire and plotting (lab, October 18, 2026). To go lower
      use `QSCAN` (no host in the per point path), or send several `RFAACQ` commands before reading the replies, or plot
      every 10 to 20 points.*
- [x] **D.15 ADC path untouched.** *(Also: full range ADC scan, 200 to 1200 m/z step 1 dwell 5 mS, 1001 points in 6.0 s, `0xEA`, parked at 200.)* `SQSSRC,1,ADC`, `QSCAN` behaves as in Part F.
      *Result: Verified October 17, 2026: Part F ran on the ADC source before D, and the source was set back to ADC at the end.*

- [x] **D.16b Set-point timing (`GRFASPT`).** `RRFASPT,1`, scan, `GRFASPT,1` returns points,avgTotal,minTotal,maxTotal,avgUpdate,avgRF,avgDC in us.
      *Result: Gordon measured, October 18, 2026, non-Rev 3 module: `1002,460,458,462,25,399,35` with TWI 0 at 100 kHz; `1002,179,176,182,25,118,35` after `STWISPEED,0,400000` (the RF level DAC is a TWI AD5625). `QSCAN` with the electrometer was 2.4 to 2.7 mS per point before and 2.3 mS after, 1.13 mS for the data transfer alone, so the host plotting (about 1.2 mS) sets the pace. Not yet checked: module EEPROM reads and `TWIfails` at 400 kHz on channel 0.* *Scope on TRGOUT, October 19, 2026, both TWI at 400 kHz, 100 kHz sampling, TWI trigger, same older (non new rev) hardware: scan only 0.2 mS per point; electrometer 100 samples 2 mS, 200 samples 3 mS per point, that is burst plus about 1.0 mS. The new rev hardware (SPI RF DAC) is still to be measured.*
- [ ] **D.17 Electrometer magnitude firmware (flash first).** After `ZERO` with no beam in NEG then POS: `RADC` and `RSUM` read near 0, `RCUR` near 0. With a known 1 nA source in each direction: `RCUR` is +1 for current into the input and −1 for current out, `RADC` is about 1876 either way, and a `QSCAN` shows peaks (not dips) in the streamed sums. Check `GTWI,01208B72` and `0xA1` against `RSUM`/`RADC`.
      *Result: not run, the electrometer was not connected when this was written (October 18, 2026).*

---

## E. `RFAACQ` with the electrometer

Uses the host script that already drives `RFAACQ` for the ADC. Add `SQSSRC,<module>,ELECT` before it and
`SQSEINIT,<module>` once before the first point of each scan.

- [x] **E.1 Without init.** `SQSSRC,1,ELECT` then `RFAACQ,1,100,0,5` **without** `SQSEINIT`: NAK, `GERR`
      reports 133.
      *Result: Verified October 17, 2026: NAK, `GERR` 133.*
- [x] **E.2 First call.** After `SQSEINIT,1`, the first `RFAACQ` returns `0`.
      *Result: Verified October 17, 2026: the first call returned `0`.*
- [x] **E.3 Pipeline.** Each later call returns the previous point's sum. Step `mz` and check the returned
      values trail the commanded points by one, the same as the ADC path.
      *Result: Verified October 17, 2026 (TWI trigger): from the second call each returned a sum, 7 of 7 sums 344,451 to 345,020. The sums are steady so the one-point trail itself is not visible; the E.5 first-call-is-zero result shows the pipeline.*
- [x] **E.4 Failure marker.** Unplug the electrometer mid scan (or let it time out): the call returns `-1`.
      `SQSEINIT` after reconnecting recovers.
      *Result: Verified October 17, 2026: with the electrometer switched to mode 0 mid scan, the call that triggered it returned the previous valid sum, the next returned `-1` with `GERR` 134, and `SQSEINIT` after restoring mode 1 recovered (first call `0`, then a sum).*
- [x] **E.5 Re-init between scans.** Run two scans back to back, each with its own `SQSEINIT`. The second
      scan's first call returns 0, not the last point of the first scan.
      *Result: Verified October 17, 2026: after `SQSEINIT` the first call returned `0` and the second a sum.*
- [x] **E.6 Settings reset it.** Changing `SQSSRC`, `SQSEADD` or `SQSETRG` clears the initialised state; the
      next `RFAACQ` NAKs until `SQSEINIT`.
      *Result: Verified October 17, 2026: after `SQSEADD` and again after `SQSETRG` the next `RFAACQ` was NAKed with 133.*
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

- [ ] **I.1 Commands.** *Result: Partly, October 17, 2026: defaults match; `SRFAMZSTB,1,A` NAKs with `GERR` 2; `SRFAMZADC,1,10,1,0` and module 2 NAK but `GERR` reads 1 (ERR_BADCMD), not 2: it is a line command and the unread tokens are parsed as commands, which overwrites the error code. Cosmetic, same as ESI's `SESICTRL`. Rev 1/2 no-channel case not tried.* Defaults: `GRFAMZADC,1` = `-1,1.000000,0.000000`, `GRFAMZSTB,1` = `NA`, `GRFAMZSTAT,1` all zero.
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

*October 17, 2026, first look with a signal on the electrometer, no mass reference: see D.12. Raw sums were inverted (dips) and `RCUR` in NEG polarity was positive; this is not explained by the front end analysis (positive ions lower the raw counts, negative ions raise them), so recheck after `ZERO` with the October 18 magnitude firmware (D.17 below). The `QSCAN`/`RFAACQ` overlay and the calibrated masses are still to do.*

**RF level check to m/z 1200 (October 17, 2026, Gordon approved about 1970 V):** stepped `SRFAMZ` + `RFAQUPDATE` from 500 to 1200 in steps of 100 m/z: setpoint 822 V at 500 and 1973 V at 1200, readbacks A 786 / B 767 V at 500 and A 1990 / B 1930 V at 1200 (about 1.64 V per m/z), forward power 5.4 W at 500 and 36.9 W at 1200, nothing faulted. The readback is not the setpoint at the top: A reads 1.0 to 1.5% above it. Returns to m/z 200 and about 334 / 322 V.

---

## H. Sign-off

| Part | Result | Date | Notes |
|---|---|---|---|
| 0 Prerequisites | partial | 2026-10-17 | electrometer firmware, wiring and Wire1 speed done; MIPS build commit not recorded; scope used on LT only |
| A `TWI1CMD` | partial | 2026-10-17 | A.1, A.4, A.5 (Gordon), A.9 done; A.2, A.3, A.6 partly; A.7, A.8 not done; extra byte after `\n` not observed (no scope) |
| C `GTWI` / `STWI` | partial | 2026-10-17 | C.3, C.4 done; C.1, C.2, C.5, C.6 need a device on the Wire channel |
| B `TWICMD` channel 0 | not done | | `TWICMD_FAST` left at: 1 |
| F Existing QSCAN regression | partial | 2026-10-17 | ADC path, start pin command, ADC off FRAME and NONE, ESC abort, re-enable all pass by serial, plus the full 200 to 1200 m/z scan (1001 points, 6.0 s); scope checks (timing, `TRGOUT`, start pulse) not done |
| D `QSCAN` electrometer | partial | 2026-10-17 | TWI trigger passes (D.7 to D.10, D.15) and a full range electrometer scan was collected (D.12, partly); **hardware (LT) trigger cannot be tested: this controller does not route LT to the proper pin (D.5)**, so D.4 and D.13 are blocked; D.3 not-armed and 135 not done |
| E `RFAACQ` electrometer | partial | 2026-10-17 | E.1 to E.6 pass on the TWI trigger; E.7 overlay needs ions; not run on the HW trigger |
| I ADC m/z input with strobe | partial | 2026-10-17 | I.1 defaults and range checks only; I.2 onwards need a voltage source and a strobe pulse source |
| G Ion test | started | 2026-10-17 | electrometer spectrum 200 to 1200 m/z collected (raw sum and `RCUR`); no overlay or calibrated mass check yet |

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

- [x] With defaults (`SQSADCENA` `TRUE`, `SQSSTPIN` `NA`), run the same `QSCAN` sequence
      already used to validate Rev 1.
      *Result: October 17, 2026: 200 to 500 m/z, step 5, dwell 5: header, 61 points, `0xEA` trailer. Points were all 0 (ADC channel 0 has nothing on it).*
- [ ] Byte stream (header/points/trailer) matches prior behavior.
- [ ] Per-point timing on the scope matches `SQSDWELL` as before, no new jitter.
- [ ] `SQSTRIG TRUE` still pulses `TRGOUT` once per point exactly as before.
- [x] After the scan completes, the module has moved back to `StartMZ` (`GRFAMZ`/front
      panel or equivalent) rather than sitting at `StopMZ` where the scan finished.
      *Result: October 17, 2026: `GRFAMZ,1` = 200 and the RF readback back at about 334 V after every scan.*

### 3.3 Start-of-scan pulse

- [ ] `SQSSTPIN 1,A` *(October 17, 2026: `SQSSTPIN,1,A` accepted, `GQSSTPIN` = A, `Z` NAKed with `GERR` 2, `NA` restored, 3 scans gave 3 frames; the pulse itself was not scoped)*
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

- [ ] `SQSADCENA 1,FALSE` *(October 17, 2026, by serial: header then trailer, 0 point bytes, `GQSADCENA` FALSE, `GQSFRAME` FRAME, `QSCANSTAT` shows both; timing and pulses not scoped)*
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

- [ ] `SQSFRAME 1,NONE` *(October 17, 2026, by serial: 1 byte only, the ACK; ESC at 1 s ended a 6 s scan, a `GVER` sent right after was answered at 1.8 s; park-back seen; scope checks not done)*
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

- [x] `SQSADCENA 1,TRUE`
- [x] `QSCAN 1` streams a full spectrum again exactly as in 3.2.
      *Result: October 17, 2026: 61 points, `0xEA`.*

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
