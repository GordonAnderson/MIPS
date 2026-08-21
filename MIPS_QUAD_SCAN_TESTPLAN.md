# QUAD scan — external DAQ features, bench test plan

Bench verification plan for the two additions to `QUADscanGo` on top of the already
validated QSCAN feature (Rev 1 confirmed working): a start-of-scan output pulse on a
user selected digital output, and an option to disable MIPS ADC acquisition entirely so
an external DAQ system can capture the spectrum instead.

**Code:** [`include/QUADscan.h`](include/QUADscan.h), [`src/QUADscan.cpp`](src/QUADscan.cpp).
**Status:** builds clean under PlatformIO (`pio run -e dueUSB`). Not yet bench tested —
that is what this document is for.

---

## 1. What changed

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

## 2. Setup

- MIPS controller with a Rev 1 QUAD module installed and enabled — the same unit already
  used to validate the base QSCAN feature.
- Oscilloscope or logic analyzer with at least 2 channels: one on `TRGOUT`, one on
  whichever digital output channel is used for the start pulse (test uses `A`).
- Serial terminal to the MIPS command port.
- `QUADcalTable`/scan range set up as in the existing QSCAN validation (or defaults:
  50–500 m/z, step 1, dwell 2 ms).

---

## 3. Test procedure

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

### 3.5 ADC disabled, NONE mode

- [ ] `SQSFRAME 1,NONE`
- [ ] `QSCAN 1`
- [ ] Nothing is written to the serial port during the scan (beyond the initial ACK) —
      confirm with a raw serial capture, not just the terminal display.
- [ ] Scope still shows correct start-pulse and per-point (`SQSTRIG`) pulsing and timing.
- [ ] `ESC` (0x1B) sent mid-scan still aborts cleanly (watchdog/abort-poll path is
      unaffected by the ADC-disabled branch).
- [ ] `QSCANSTAT 1` shows the frame line as `NONE`.

### 3.6 Re-enable and confirm clean return to normal mode

- [ ] `SQSADCENA 1,TRUE`
- [ ] `QSCAN 1` streams a full spectrum again exactly as in 3.2.

---

## 4. Known risk areas to watch

- **Shared output bank.** `SetOutput`/`ClearOutput` drive the same shadow register
  (`DOmsb`/`DOlsb`) used by other subsystems (interlock, FAIMS, compressor switch,
  Twave). Confirm the chosen `SQSSTPIN` channel isn't wired to one of those on this unit
  before testing, or the pulse will also toggle that signal.
- **Priming-point abort path.** If the first point's acquisition fails with `SQSADCENA
  TRUE`, `TRGOUT` is left high until the end-of-scan safety reset — this is unchanged
  from the original code, but worth confirming the refactor preserved it (scope `TRGOUT`
  across a deliberately provoked timeout if practical).

---

## 5. Sign-off

| Section | Result | Date | Notes |
|---|---|---|---|
| 3.1 Build | | | |
| 3.2 Regression | | | |
| 3.3 Start pulse | | | |
| 3.4 ADC off, FRAME | | | |
| 3.5 ADC off, NONE | | | |
| 3.6 Re-enable | | | |
