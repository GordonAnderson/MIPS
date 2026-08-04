# Phase 1 — Shared scan primitive + auto-tune speed-up

**Patch:** `phase1.patch` — apply on top of `phase0`
**Build:** 41/41 objects, links clean. Text 308,148 → 308,188 (**+40 bytes**), bss unchanged.

Two independent pieces of work in one patch, because the tune changes are needed on the
bench now. They touch different functions and can be reverted separately.

---

## Part A — `RFAsetScanPoint` extraction (the actual Phase 1)

`RFAacquire`'s hardware-setting body is now `RFAsetScanPoint(brd, mz, delta)`, declared in
`RFamp.h`. `RFAacquire` keeps argument parsing, the previous-point sum readout, the dwell,
and the ADC trigger. Symbol sizes confirm a clean split: `RFAacquire` was 0x118 (280 bytes),
now 0xC4 (196) plus `RFAsetScanPoint` at 0x68 (104); the 20-byte delta is the call overhead.

**Ordering question from the plan: resolved.** `QUADupdate(brd)` recalculates `ResolvingDC`
from the m/z value, so the subsequent `+=` applies delta to the freshly computed value. It
reads as accumulation but is effectively "computed value plus delta" every call. Preserved
exactly and now commented.

**One deliberate behaviour change.** `sum` is initialised to 0. `ADCfindSum` leaves `*sum`
untouched on all three failure paths and `RFAacquire` ignores the return value, so those
paths printed uninitialised stack memory. The "no vector recorded yet" path is taken on the
first call of every scan. Success-path output is bit-identical; only failure paths change,
from undefined to 0.

*Open design question for Phase 3:* a failed point returning 0 is indistinguishable from a
legitimately zero point mid-spectrum. The streaming format may want a real error signal.

---

## Part B — Auto-tune

### The problem

The coarse sweep samples on a grid of `quadATmaxStep` spacing starting from
`(quadATmaxF + quadATminF)/2` = 950 kHz. With the default 20 kHz step, the points nearest a
703 kHz resonance are **710 and 690 kHz** — neither on the peak:

| Q | Bandwidth at 703 kHz | Response at ±7 kHz |
|---|---|---|
| 50 | 14 kHz | near peak |
| 100 | 7 kHz | ~half power |
| 200 | 3.5 kHz | essentially invisible |
| 400 | 1.8 kHz | gone |

That also explains the run-to-run variation. At 710 kHz the reading is marginal: when it
happens to exceed the other coarse points, `FreqMax` = 710 and the 2 kHz fine phase walks
downhill into 703. When it doesn't, `FreqMax` latches onto the highest noise point, usually
near the 950 kHz start — giving 910.

### The key realisation

`qdly = 20` (2 s per step) was **not** an RF settling requirement. `#define Filter 0.05` at
the 10 Hz loop rate gives a first-order time constant of about 2 seconds, and the delay
existed to wait that filter out. But `quadAutoTune` already presets `RFVPpp`/`RFVNpp` to −1,
which makes the readback code take the next sample **unfiltered**. Actual tank settling is
Q/(πf) — microseconds.

So the per-step delay can drop by roughly an order of magnitude, and that budget buys
proportionally finer frequency steps, which is exactly what a high-Q tank needs.

### Changes

| Change | Detail |
|---|---|
| `quadATstepDelay`, default **5** | Per-step delay in 100 ms units, was hard-coded 20. 4× faster at the same resolution. |
| `SRFATDLY` / `GRFATDLY` | Host commands for the above |
| `SRFATSTEP` / `GRFATSTEP` | Host commands for `quadATmaxStep`, which previously had none |
| Completion report | Now prints the final peak metric alongside the frequency |
| Validity warnings | Warns if the tune stopped at a range limit, or converged within one coarse step of the start frequency |
| **B12 fixed** | The unguarded second `NumDown++` in the DWN case removed |

**Delays deliberately left long:** the 2 s after `quadTuneRequest` (covers RF amplifier
startup from `Enabled = true`) and the 4 s at the drive-level change to `quadAThighP`. Those
cover real settling, not the filter.

**B12 detail.** The removed line had no `(TuneStep < quadATmaxStep)` guard, unlike the line
above it. During the coarse sweep it was harmless — the guarded line's `else NumDown = 0`
fires every pass, so `NumDown` oscillated 0→1. During the **fine** sweeps both lines fired,
so `NumDown` climbed by 2 per pass and the downward search gave up after 3 declining steps
instead of `QUAD_AT_MAX_DWN` (5), while the upward search still used 5. That asymmetry biased
results high in frequency. This is the one hunk to revert first if fine-tune behaviour looks
wrong.

### Tune time budget

Coarse pass ≈ `(range / step) × 2 × quadATstepDelay × 100 ms`:

| Range | Step | Delay | Coarse pass |
|---|---|---|---|
| 1100 kHz | 20 kHz | 20 (old default) | 220 s |
| 1100 kHz | 20 kHz | 5 (new default) | 55 s |
| 1100 kHz | 5 kHz | 5 | 220 s |
| 100 kHz | 20 kHz | 5 | 5 s |
| 100 kHz | 2 kHz | 5 | 50 s |
| 100 kHz | 2 kHz | 3 | 30 s |

Full-range fine-grained sweeping stays expensive. **Narrowing the range is still the biggest
lever.**

### Suggested bench sequence

1. Default settings, confirm the tune still completes and is ~4× faster.
2. `SRFATDLY` down to 3, then 2. Watch whether the found frequency stays consistent — that
   tells you the real RF settling time, which is the number that matters.
3. Narrow: `SRFATMINF,650000` / `SRFATMAXF,750000`, then `SRFATSTEP,2000`. This should find
   703 kHz reliably and in well under a minute.
4. Widen back out and reduce `SRFATSTEP` until the full-range tune is reliable, to find the
   step your tank Q actually requires.

---

## Verification

**Part A:** `RFAACQ` host scan overlay against a previous spectrum. Also covers the Phase 0
items T0-1 (`RFAgainComp`) and T1-1 (`RFAupdatePoles`).

**Part B:** as above. Note the tune now reports a peak metric — in the final phase this is the
SWR-derived value (`100 − SWR`) rather than an RF level, since the last sweep runs in SWR
mode when `quadATuseSWR` is true.

Still open from Phase 0: B11 (cold start).
