# MIPS QUAD firmware — handoff

State of the firmware after phases 0 through 7a. Written August 2026, superseding the
earlier `MIPS_QUAD_HANDOFF.md`.

**Repo:** https://github.com/GordonAnderson/MIPS (public)
**Branches on origin:** `master`, `phase0` … `phase7`, each building on the last.
Branch from `phase7`. This document is committed on that branch; if the branch list above
does not match what `git branch -r` reports, this file is behind the repo and the repo wins.

Firmware version string is `1.263`.

---

## 1. Build harness

PlatformIO cannot be used in the sandbox: `api.registry.platformio.org` is not in the network
allowlist, so `pio run` fails at platform install. The firmware **can** be built by assembling
the toolchain from allowlisted sources. This recipe is verified working.

### Setup

```bash
apt-get install -y -q gcc-arm-none-eabi          # running as root, no sudo present
cd /home/claude
git clone --depth 1 https://github.com/arduino/ArduinoCore-sam.git
git clone https://github.com/GordonAnderson/MIPS.git MIPSv
```

The MIPS repo is self contained: `lib/` is vendored and `variants/arduino_due_x/` supplies
both the prebuilt `libsam_sam3x8e_gcc_rel.a` and the linker scripts.

### Include paths

```bash
cd /home/claude/MIPSv
INC="-Iinclude -I/home/claude/ArduinoCore-sam/cores/arduino \
 -I/home/claude/ArduinoCore-sam/system/libsam \
 -I/home/claude/ArduinoCore-sam/system/CMSIS/CMSIS/Include \
 -I/home/claude/ArduinoCore-sam/system/CMSIS/Device/ATMEL \
 -Ivariants/arduino_due_x $(for d in lib/*/; do echo -I$d -I${d}src; done | tr '\n' ' ')"
LIBINC=$(find /home/claude/ArduinoCore-sam/libraries lib -name '*.h' -exec dirname {} \; \
 | sort -u | sed 's/^/-I/' | tr '\n' ' ')
echo "$INC" > /tmp/inc.txt; echo "$LIBINC" > /tmp/libinc.txt
```

### Compile one file

```bash
cat > /tmp/cco.sh <<'SH'
#!/bin/bash
cd /home/claude/MIPSv
INC=$(cat /tmp/inc.txt); LIBINC=$(cat /tmp/libinc.txt)
src="$1"; out="$2"; shift 2
case "$src" in
 *.c) CC=arm-none-eabi-gcc; STD="-std=gnu11";;
 *)   CC=arm-none-eabi-g++; STD="-std=gnu++11 -fno-rtti -fno-exceptions";;
esac
$CC -c -mcpu=cortex-m3 -mthumb -Os -w -ffunction-sections -fdata-sections $STD \
 -DSAM3X8 -D__SAM3X8E__ -DF_CPU=84000000L -DARDUINO=10805 -DARDUINO_SAM_DUE \
 -DARDUINO_ARCH_SAM -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON \
 -DUSB_MANUFACTURER='"x"' -DUSB_PRODUCT='"y"' \
 -include include/sam3x8_ext.h $INC -Isrc $LIBINC "$@" "$src" -o "$out"
SH
chmod +x /tmp/cco.sh
```

The trailing `"$@"` allows extra flags: `-fsyntax-only` for a fast check, `-Wall -Wextra`
when hunting for problems.

### Build everything

```bash
mkdir -p /tmp/build
for f in src/*.cpp; do /tmp/cco.sh $f /tmp/build/app_$(basename $f .cpp).o || echo "FAIL $f"; done
for f in /home/claude/ArduinoCore-sam/cores/arduino/*.c \
         /home/claude/ArduinoCore-sam/cores/arduino/*.cpp \
         /home/claude/ArduinoCore-sam/cores/arduino/USB/*.cpp \
         /home/claude/ArduinoCore-sam/cores/arduino/avr/dtostrf.c \
         variants/arduino_due_x/variant.cpp; do
  /tmp/cco.sh "$f" /tmp/build/core_$(basename $f | tr '.' '_').o
done
arm-none-eabi-gcc -c -mcpu=cortex-m3 -mthumb -x assembler-with-cpp \
  /home/claude/ArduinoCore-sam/cores/arduino/wiring_pulse_asm.S \
  -o /tmp/build/core_asm_pulse.o
for f in $(find lib /home/claude/ArduinoCore-sam/libraries -name '*.cpp' -o -name '*.c' \
           | grep -v examples); do
  /tmp/cco.sh "$f" /tmp/build/lib_$(echo $f | md5sum | cut -c1-8).o
done
```

### Link

```bash
echo "void *__dso_handle;" > /tmp/dso.c
arm-none-eabi-gcc -c -mcpu=cortex-m3 -mthumb /tmp/dso.c -o /tmp/build/zz_dso.o

LD=variants/arduino_due_x/linker_scripts/gcc/flash.ld
CRTI=/usr/lib/gcc/arm-none-eabi/13.2.1/thumb/v7-m/nofp/crti.o
CRTN=/usr/lib/gcc/arm-none-eabi/13.2.1/thumb/v7-m/nofp/crtn.o
arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Os -nostartfiles \
  -Wl,--allow-multiple-definition -T $LD --specs=nosys.specs \
  -o /tmp/build/mips.elf $CRTI /tmp/build/*.o $CRTN \
  -Lvariants/arduino_due_x -lsam_sam3x8e_gcc_rel -lm -lgcc
arm-none-eabi-size /tmp/build/mips.elf
```

**Do NOT use `--gc-sections`.** With an entry point that differs from PlatformIO's it prunes
most of the application and the reported size is meaningless. Verify symbols survived, and
remember they are C++ mangled — use `nm -C` and match `[Tt] name(`:

```bash
for sym in RFA_init SelectRange RFampCalibrate QUADscanGo quadAutoTune; do
  arm-none-eabi-nm -C /tmp/build/mips.elf | grep -qE "[Tt] $sym\(" \
    && echo "  $sym present" || echo "  $sym MISSING"
done
```

Other link requirements: `-nostartfiles` plus explicit `crti.o`/`crtn.o`, a `__dso_handle`
stub, `wiring_pulse_asm.S` for `countPulseASM`, and `avr/dtostrf.c`.

**Reference sizes**, text / bss. Due has 512 kB flash, 96 kB SRAM.

| Tree | text | bss |
|---|---|---|
| phase4 | 420,004 | 26,116 |
| phase6 | 420,436 | 26,116 |
| phase7a | 420,444 | 26,116 |

### This is not a substitute for PlatformIO

GCC 13.2 here versus the older compiler in PlatformIO's `atmelsam` platform. Flags are
reconstructed and the three `extra_scripts` do not run. It catches syntax, type and
declaration errors, which is most of what matters — but every phase must also build clean
under PlatformIO on Gordon's machine before it counts as done.

One of those `extra_scripts` matters for documentation: see section 3.

---

## 2. Where the project stands

**Complete and bench verified.** Mass calibration table (entry, interpolation, EEPROM
persistence). Scan engine with binary streaming, verified uniform 3 ms per point on the
scope, ESC abort, hardware resync on exit, repeated scans back to back. Level detector dual
range calibration and range switching (phase 6, full calibration performed on the bench and
checked out).

**Applied but not yet exercised with ions.** Phase 4 (calibration applied during the scan,
distinct error codes, `QSCANSTAT` improvements) was never bench tested. Phase 7a is flashed
and pushed, but its failure path is by definition hard to provoke deliberately.

**The remaining acceptance test needs the mass spectrometer.** A firmware `QSCAN` and a host
`RFAACQ` scan over the same range with the same calibration table should overlay. If they do
not, `RFAsetScanPoint` (phase 1) is the first place to look — it is the only code both paths
share. Note that both paths pipeline identically: the host script primes with
`RFAACQ,1,start,0,dwell` then commands `mz+step` while reading the value for `mz`, and
`QUADscanGo` primes before its point loop and sends the previous point's sum while the
current one settles. The calibration lookup is applied to the point being *set* in both.

---

## 3. Phase by phase, what shipped

| Phase | Content |
|---|---|
| 0–3 | Cal table, scan engine, binary streaming, abort, resync |
| 4 | Calibration applied during scan, distinct error codes 127–131, `QSCANSTAT` |
| 5a | Help generator markers on the QUAD command table |
| 5 (doc) | `MIPSquadOperationsR3_2.docx` — Rev 3 section, firmware scan section, Appendix C |
| 6a | Both detector ranges calibrated at a common point |
| 6b | Hysteresis on the range switch, `SRFARNGHYST` / `GRFARNGHYST` |
| 6c | Range hold while a calibration page is active |
| 6d | Resync the dialog copy after a calibration |
| 7a | Always emit a trailer for every `QSCAN` header |

### 5a is load bearing for documentation

`MIPScommands.txt` is **generated**, not hand maintained. `help_file_creator.py` runs as a
PlatformIO `extra_script` after every build and rebuilds the file by scanning each `.cpp` for
`// Start of command block` … `// End of table marker`. `QUADscan.cpp` had neither marker, so
none of the 22 `SQC*`/`SQS*`/`QSCAN` commands appeared in the help file. Editing
`MIPScommands.txt` by hand is pointless — the next build overwrites it. **Command
documentation lives in the table comments.**

Confirmed working: after Gordon's PlatformIO build of phase6, all 22 commands are in
`MIPScommands.txt` with their wrapped continuation lines intact.

The parser rules: a comment at column 0 becomes a section header, an *indented* comment
becomes a continuation line of the previous entry, and `{ "NAME"  ... }  // text` becomes an
entry. There must be no `/` between the closing quote of the name and the `//`.

---

## 4. Design decisions and why

Carried forward from the earlier handoff, plus the phase 6 and 7 decisions.

**Cal table lives outside `RFAdata`, in its own EEPROM record.** Adding it to `RFAdata` would
grow `sizeof(RFAdata)` for every module including Rev 1. A Rev 1 field unit running this
firmware would write the larger structure to its EEPROM; if that module later went into a
system running older firmware, the unguarded `memcpy` there would overflow.

**EEPROM offset 368, not 376.** 376 ended exactly at 512 and maximised `RFAdata` headroom,
but the AT24C04 has a 16 byte page and a page write starting mid page wraps within that page.
`WriteEEPROM` writes fixed 16 byte chunks from the given address, so an unaligned offset
silently scrambles the record while still returning success. 368 is the highest aligned
offset that fits. Guarded by `static_assert`.

**CRC length uses `offsetof(QUADcalRecord, CRC)`, not `sizeof - sizeof(uint16_t)`.** The
struct has two bytes of trailing padding, so the subtraction gives 134 and pulls the CRC
field's own bytes into the checksum — a guaranteed mismatch on every read back. This was the
root cause of the "saves but does not persist" bug and took three patches to find.

**Everything allocated at run time, Rev 3 only.** `QUADcalTable[2]` and `QUADscanP[2]` are
arrays of pointers following the `RFAarray`/`RFAstates` pattern. A system with no Rev 3 QUAD
module carries 16 bytes of BSS rather than ~370. NULL doubles as the "not available" check.

**Scan blocks deliberately.** `RFA_loop` cannot run during a scan, which removes any race
between the 10 Hz change detection and the scan's own DAC writes. In exchange the loop pets
the watchdog and polls for abort itself.

**Streaming is fixed width binary, raw sums.** Variable length ASCII would make per point
write time depend on the value, reintroducing the timing jitter the feature exists to remove.
Averaging N samples buys resolution beyond 12 bits; a 12 bit average would discard it.

**Scan parameters are not persisted.** The EEPROM record region has 8 bytes spare and they
need 24. Persisting them means relocating or shrinking the calibration record.

**Both detector ranges calibrated at one shared physical point (6a).** Previously the low
range was calibrated at 20 V and 0.75 × `RangeThreshold` but used up to `RangeThreshold`,
and the high range was calibrated at 0.25 × `FullScale` and 0.75 × `FullScale` but used down
to `RangeThreshold`. **Both fits were extrapolating at the crossover** and nothing
constrained them to meet. Now both use `RangeThreshold` as a calibration point, and each fit
is *anchored* on it — the low range intercept from its P2, the high range intercept from its
P1 — so both reproduce that level exactly rather than approximately.

The residual floor is one high range ADC count at the threshold: with `FullScale` 2000 that
is 0.5 V, 0.25 % of a 200 V threshold. No calibration scheme beats that.

How much 6a improves things depends on `FullScale`, because that sets how far
0.25 × `FullScale` sat from `RangeThreshold`. Modelled at threshold 200: `FullScale` 2000
goes from about 5 % of reading to 0.1 %; `FullScale` 1000 was already only 0.3 % because
0.25 × 1000 = 250 was nearly at the threshold.

Guards were added on both spans so a `RangeThreshold` above 0.75 × `FullScale`, or below
20 V, cannot write an infinite slope into the calibration silently.

**Hysteresis on the range switch (6b).** `SelectRange` was a bare comparison with no memory
of the current range, so a setpoint near the threshold switched on every 10 Hz pass. Each
switch changes both the readback calibration and the setpoint DAC calibration, so the chatter
appeared in the reported level and, closed loop, in the output. Now switches up at
`RangeThreshold` and down at `RFArangeHyst` × `RangeThreshold`, default 0.90, reading the
current range from the CPLD image via the existing `isRangeHigh()`. Values outside (0,1) fall
back to 0.90 — at exactly 1.0 there is no dead band at all, which is the bug it exists to
prevent. The dead band must exceed the setpoint jitter to be worth anything: at 0.999 the
band is 0.2 V and 0.5 V of jitter still chatters.

**Range hold during calibration (6c).** `LowRangeCalPrep` / `HighRangeCalPrep` selected the
range when their page opened but nothing kept it there. `RFdrvCalPrep` — the prep callback on
*every* calibration entry including both "Point" entries — zeroes `SetPoint`, so in closed
loop `RFA_loop` called `SelectRange(brd, 0)` on the next pass and put the hardware back in
low range. The readback calibration never saw this because it is done in **open loop**, where
`SelectRange` returns early on `Mode` (`Mode` is true for open loop) and the open loop branch
pins the range high. The setpoint DAC calibration has to be done **closed loop**, where
`SelectRange` does run, so the high range setpoint DAC was being calibrated with the range
line low and `setRFADAC` writing through `DACchansLR` the whole time.

`SelectRange` now returns early for the board named by `RFArangeHoldBrd`, set by the two prep
functions and cleared by `RFampCalibrate`. `RFA_loop` also clears it whenever neither
calibration page is on screen — page 1 has no Abort, its only exit is "Next", and no
navigation path should be able to leave auto ranging switched off for the session.

**Dialog copy resync (6d).** `RFampCalibrate` wrote the new coefficients into `*RFAD` but left
`rfad` holding the pre-calibration values, and `RFA_loop` begins with
`if(ActiveDialog->Changed) *RFAD = rfad;`. `rfad` is a full `RFAdata`, so that assignment
carries `ADCchans`, `ADCchansLR`, `DACchans` and `DACchansLR` with it: the next edit of any
entry while the RF dialog was up restored the old calibration over the new one with no
indication. A silent revert. Fixed with `rfad = *RFAD` at the end of `RFampCalibrate`.

**Every `QSCAN` header gets a trailer (7a).** The priming acquisition before the point loop
sat in the outer scan loop, so its failure path broke straight past `QUADscanSendTrailer`. An
ADC timeout on the first point of a scan sent a header, no points and no trailer — the host
never got the promised byte count and never saw a terminator, so neither normal exit
condition fired. `ADCstop()` still ran, so the instrument recovered cleanly and the next
command worked, which made it look like a host fault. The failure now sets `aborted` and
guards the point loop instead of breaking.

Structural invariant worth preserving: header and trailer are at the same brace depth in the
scan loop body, and the two remaining `break`s are inside the inner point loop.

**Setpoint DAC calibration depends on the readback calibration.** `GetAverageV()` and
`GetAverageVLR()` measure through `ADCchans` / `ADCchansLR`. So the readback calibration must
be done **before** the setpoint DAC calibration, or the drive side inherits the old readback
error. This is why the setpoint DAC needed no formula change in 6a: its fits use operator
chosen setpoints and measured voltages, with no hardcoded fractions. Making the two setpoint
ranges agree is a *procedure* change — one low range and one high range point taken at the
same physical level — and has **not been done**.

---

## 5. Open items

| Item | Notes |
|---|---|
| **Ion testing** | The acceptance test. `QSCAN` and host `RFAACQ` over the same range with the same cal table should overlay. Highest priority; everything else is secondary. |
| Phase 4 never bench tested | Cal applied during scan, error codes, `QSCANSTAT`. Folded into the ion testing session. |
| Setpoint DAC ranges still meet by luck | 6a fixed the readbacks. The drive side needs one shared physical level between the two range calibrations. Procedure change, no code. |
| Crossover step never measured | The "after" number for 6a. Closed loop, auto range on, step the setpoint through 200 V and read `GRFAVPPA`/`GRFAVPPB` against a meter; sweep down through 180 too, since 6b moved the down switch. |
| `*live = dialogCopy` audit | The 6d pattern appears in ARB, DAC, DCbias, DMSDMSMB, ESI, FAIMSfb, Filament, HVPSinterface. Whether each is vulnerable depends on whether its calibration routine resyncs the dialog copy. **Not checked.** Worth a session only if the revert actually reproduced on the bench — that was never confirmed. |
| `WriteEEPROM` silently corrupts unaligned writes | Guarded by `static_assert` in QUADscan only. A few lines in `Hardware.cpp` (`int chunk = 16 - (address % 16);` for the first chunk) would protect every caller. Touches shared code, wants its own commit. |
| `ADCacquireWait` margin is a fixed +1 ms | 100 % margin at 200 samples, ~4 % at 5000. A proportional margin would be better. |
| `ADCnumsamples` not persisted | Reverts to 5000 (25 ms/point) on power cycle. Visible in `QSCANSTAT`, documented in the manual, still a trap. |
| `SRFAREV3` not persisted | Sets `Rev` in RAM only; needs Save settings. Already caused one confusing debug session. |
| `SADCSAMPS` has no range check | Bare `CMDint`. The practical ceiling is `new uint16_t[ADCnumsamples]` in 96 kB. |
| `RangeThreshold` has no host command | Front panel "Range thres" only, 0–500. |
| B11 cold start | Explicit CPLD write at end of `RFA_init`; untested with `NormalStartup` both true and false. |
| T1-2 command preamble refactor | ~30 handlers share four lines of boilerplate. Never done. |
| Auto tune coarse grid vs high Q | Mitigated by `SRFATSTEP` / `SRFATDLY` / range narrowing, not solved. A resonance narrower than the coarse step can fall between grid points. |
| Lock the range for a whole scan | Raised as optional in the dual range work: `QUADscanGo` knows the range before it starts and could hold one range for the whole spectrum. 6b and 6c may have made this unnecessary — measure first. |
| Terminal instability after binary streams | Seen twice. The most likely explanation is the host feeding the ACK and binary payload into the line oriented command parser; see the Qt handoff. Watch whether it survives the Qt work. |
| Manual: calibration ordering | `MIPSquadOperationsR3_2.docx` Appendix B does not state that readbacks must be calibrated before setpoints. Worth adding. Appendix B otherwise survived 6a untouched because it tells the operator to match the value shown on the front panel rather than naming target voltages. |

---

## 6. Working conventions that worked

- **One phase per conversation.** Context fills up; a session that has been through several
  debug cycles has less room to read code carefully. The quality difference is real.
- **Incremental patches, uniquely named.** `phase6a`, `phase6b`, `phase6c` — never reuse a
  filename. Note that patches generated in sequence may depend on each other: `phase6c` would
  not apply without `phase6b` because both edit `SelectRange`. State dependencies explicitly
  or ship a combined patch.
- **Verify patches apply before shipping.** Reconstruct the target state, `git apply --check`,
  apply, and diff the result against the intended tree.
- **Test arithmetic off target.** The interpolation, the CRC, the range calibration fits and
  the hysteresis state machine were all extracted into small host programs and run before
  shipping. Two real defects came out of it: the hysteresis guard accepting exactly 1.0, and
  a wrong overflow figure. See the Qt handoff for the reader tests.
- **Prove a change is comment-only when it claims to be.** Strip comments from both revisions
  and diff — cheaper and more convincing than reading the patch.
- **Watch for silent-success failure modes.** `WriteEEPROM` returning 0 on a bad write,
  `ADCfindSum` returning `-1` as a bool, and the 6d calibration revert all hid real failures.
  Read back and verify.
- **When a symptom appears right after a patch, check whether that patch could physically
  cause it.** The range line problem surfaced immediately after 6a but 6a only changed two
  `sprintf` strings and the fit arithmetic. It was pre-existing, and probably part of the
  original drive side discontinuity.
