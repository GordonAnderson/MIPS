# MIPS QUAD firmware scan — handoff

Context for continuing this work in a new conversation. Written August 2026.

**Repo:** https://github.com/GordonAnderson/MIPS (public)
**Branches:** `phase0` → `phase1` → `phase2` → `phase3` → `phase4`, each building on the last.
`phase4` may not be pushed yet; check.

---

## 1. Build harness — the important part

PlatformIO cannot be used directly in the sandbox: `api.registry.platformio.org` is not in the
network allowlist, so `pio run` fails at platform install. The firmware **can** still be built
by assembling the toolchain from allowlisted sources. This took some trial and error; the
working recipe is below.

### Setup

```bash
# ARM cross compiler from the Ubuntu archives (allowlisted)
apt-get install -y -q gcc-arm-none-eabi          # running as root, no sudo present

# Arduino SAM core, for headers and the core sources (github is allowlisted)
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
src="$1"; out="$2"
case "$src" in
 *.c) CC=arm-none-eabi-gcc; STD="-std=gnu11";;
 *)   CC=arm-none-eabi-g++; STD="-std=gnu++11 -fno-rtti -fno-exceptions";;
esac
$CC -c -mcpu=cortex-m3 -mthumb -Os -w -ffunction-sections -fdata-sections $STD \
 -DSAM3X8 -D__SAM3X8E__ -DF_CPU=84000000L -DARDUINO=10805 -DARDUINO_SAM_DUE \
 -DARDUINO_ARCH_SAM -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON \
 -DUSB_MANUFACTURER='"x"' -DUSB_PRODUCT='"y"' \
 -include include/sam3x8_ext.h $INC -Isrc $LIBINC "$src" -o "$out"
SH
chmod +x /tmp/cco.sh
```

Use `-fsyntax-only` instead of `-c ... -o` for a fast check; use `-Wall -Wextra` instead of
`-w` when hunting for problems.

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

### Link — the part with the gotchas

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

**Do NOT use `--gc-sections`.** With an entry point that differs from PlatformIO's, it prunes
most of the application — `RFA_init`, `RFAacquire`, `quadAutoTune` all vanish and the reported
size is meaningless. This produced several rounds of wrong size numbers before it was caught,
by noticing that adding a whole new module left the image byte identical.

Other link requirements: `-nostartfiles` plus explicit `crti.o`/`crtn.o` (else `_init`/`_fini`
are undefined), a `__dso_handle` stub, `wiring_pulse_asm.S` for `countPulseASM`, and
`avr/dtostrf.c`.

**Reference sizes** (phase4): text 420,004, bss 26,116. Due has 512 kB flash, 96 kB SRAM.

### This is not a substitute for PlatformIO

GCC 13.2 here versus the older compiler in PlatformIO's `atmelsam` platform. Flags are
reconstructed; the three `extra_scripts` do not run. It catches syntax, type and declaration
errors, which is most of what matters — but every phase must also build clean under
PlatformIO on Gordon's machine before it counts as done.

---

## 2. Where the project stands

Complete and bench verified: mass calibration table (entry, interpolation, EEPROM
persistence), scan engine with binary streaming, **verified uniform 3 ms per point timing on
the scope**, ESC abort, hardware resync on exit, repeated scans back to back.

Phase 4 (calibration applied during scan, distinct error codes, `QSCANSTAT` improvements) was
applied but not yet bench tested.

**The remaining acceptance test needs the mass spectrometer:** a firmware scan and a host
`RFAACQ` scan over the same range with the same calibration table should overlay. If they do
not, `RFAsetScanPoint` (Phase 1) is the first place to look — it is the only code both paths
share.

Phase 5 is documentation: manual updates for the `SQS*`/`SQC*` commands, scan procedure,
Rev 3 restriction, blocking behaviour, abort character, extrapolation caveat.

---

## 3. Design decisions and why

Recorded because the reasoning is not all obvious from the code.

**Cal table lives outside `RFAdata`, in its own EEPROM record.** Adding it to `RFAdata` would
grow `sizeof(RFAdata)` for every module including Rev 1. A Rev 1 field unit running this
firmware would write the larger structure to its EEPROM; if that module later went into a
system running older firmware, the unguarded `memcpy` there would overflow. B10 fixes that
going forward but deployed firmware cannot be retrofitted.

**EEPROM offset 368, not 376.** 376 ended exactly at 512 and maximised `RFAdata` headroom, but
the AT24C04 has a **16 byte page** and a page write starting mid page wraps within that page.
`WriteEEPROM` writes fixed 16 byte chunks from the given address, so an unaligned offset
silently scrambles the record while still returning success. 368 is the highest aligned offset
that fits. Guarded by `static_assert`.

**CRC length uses `offsetof(QUADcalRecord, CRC)`, not `sizeof - sizeof(uint16_t)`.** The struct
has two bytes of trailing padding, so the subtraction gives 134 and pulls the CRC field's own
bytes into the checksum — a guaranteed mismatch on every read back. This was the root cause of
the "saves but does not persist" bug and took three patches to find.

**Everything allocated at run time, Rev 3 only.** `QUADcalTable[2]` and `QUADscanP[2]` are
arrays of pointers following the `RFAarray`/`RFAstates` pattern. A system with no Rev 3 QUAD
module carries 16 bytes of BSS rather than ~370. NULL doubles as the "not available" check.

**Scan blocks deliberately.** `RFA_loop` cannot run during a scan, which removes any race
between the 10 Hz change detection and the scan's own DAC writes. In exchange the loop pets
the watchdog and polls for abort itself.

**Streaming is fixed width binary, raw sums.** Variable length ASCII would make per point write
time depend on the value, reintroducing the timing jitter the feature exists to remove.
Averaging N samples buys resolution beyond 12 bits; a 12 bit average would discard it.

**Scan parameters are not persisted.** The EEPROM record region has 8 bytes spare and they need
24. Persisting them means relocating or shrinking the calibration record.

---

## 4. Open items

| Item | Notes |
|---|---|
| `WriteEEPROM` silently corrupts unaligned writes | Guarded by `static_assert` in QUADscan only. A few lines in `Hardware.cpp` (`int chunk = 16 - (address % 16);` for the first chunk) would protect every caller. Touches shared code, wants its own commit. |
| `ADCacquireWait` margin is a fixed +1 ms | 100% margin at 200 samples, ~4% at 5000. A proportional margin would be better. |
| `ADCnumsamples` not persisted | Reverts to 5000 (25 ms/point) on power cycle. Visible in `QSCANSTAT` but will catch someone. |
| `SRFAREV3` not persisted | Sets `Rev` in RAM only; needs a Save settings. Already caused one confusing debug session. |
| B11 cold start | Explicit CPLD write added at end of `RFA_init`; untested with `NormalStartup` both true and false. |
| `RFAACQ` voltage comparison | Phase 1's acceptance test. Meter RF Vp-p and pole voltages at several m/z, compare to pre-Phase-1. Bench only, no ions needed. |
| T1-2 command preamble refactor | ~30 handlers share four lines of boilerplate. Never done. |
| Auto tune coarse grid vs high Q | Mitigated by `SRFATSTEP`/`SRFATDLY`/range narrowing, not solved. A resonance narrower than the coarse step can still fall between grid points. |
| Terminal instability after binary streams | Seen twice. Possibly the terminal software; worth watching whether it correlates with `QSCAN`. |

---

## 5. Working conventions that worked

- **One phase per conversation.** Context fills up; a session that has been through several
  debug cycles has less room to read code carefully. The quality difference is real.
- **Incremental patches, uniquely named.** `phase2a`, `phase2b`, `phase2c` — never reuse a
  filename. Reusing `phase2a` for a second version made it impossible to tell which had been
  downloaded.
- **Verify patches apply before shipping.** Reconstruct the target state, `git apply --check`,
  apply, diff against the intended result.
- **Test arithmetic off target.** The interpolation and CRC were both extracted into small
  host programs and run before shipping. The CRC test would have caught the padding bug
  immediately had it been written when the code was.
- **Watch for silent-success failure modes.** `WriteEEPROM` returning 0 on a bad write, and
  `ADCfindSum` returning `-1` as a bool, both hid real failures. Read back and verify.
