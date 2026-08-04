# Phase 1a — Auto-tune start-state fix

**Patch:** `phase1a.patch` — apply on top of your pushed `phase1`
**Build:** 41/41 objects, links clean. Text 308,188 → 308,244 (+56 bytes), bss unchanged.
No trailing whitespace this time.

---

## Your diagnosis was right, mine was incomplete

Failing at **948 kHz** with a start frequency of 950 kHz is the signature you identified: if
the module is already tuned and driving a high output, the level reading at the start
frequency is still decaying when the first measurement is taken. `Max` gets set from that
stale reading, `FreqMax` pegs at the start frequency, and nothing measured later beats it.
Reducing the drive before tuning removing the failure confirms it directly.

That also explains why the coarse-grid theory only ever half-fit. Grid straddling is real —
710 and 690 kHz genuinely bracket 703 kHz — but it would fail consistently, not intermittently.
The start-state dependency explains the run-to-run variation. Both are probably contributing.

---

## A bug in my code: the warning never fired

The suspect-result check was inside `if(report && !SerialMute)`, so it only printed when the
tune was started with the reporting variant (`setQuadATR`). 948 kHz would have tripped the
start-frequency test, but the check never ran.

That was wrong on principle as well as in effect — a failure indication should not be optional.
It is now outside the `report` gate, suppressed only by `SerialMute`, and it also raises
`DisplayMessage("Tune suspect!", 2000)` on the front panel so a failure is visible without a
serial connection.

---

## Changes

| Change | Default | Purpose |
|---|---|---|
| `quadATstartDelay` | **40** (4 s) | Delay before the sweep begins, and after the drive level change. Replaces the hard-coded 20 and 40. |
| `quadATfineDelay` | **10** (1 s) | Per-step delay for the fine passes (`TuneStep <= 500`), separate from the coarse `quadATstepDelay` |
| First reading discarded | — | One throwaway measurement at the start frequency, then another settling delay, before `Max` tracking begins |
| Suspect warning ungated | — | No longer requires `report`; adds a front-panel message |

New host commands: `SRFATSTARTDLY` / `GRFATSTARTDLY`, `SRFATFINEDLY` / `GRFATFINEDLY`.

**Why both a longer delay and a discarded reading.** The delay is the real fix but its
sufficiency depends on how high the output was and how fast the detector decays — unknown
without measurement. Discarding the first reading costs one extra settling period and removes
the specific failure mode regardless of whether the delay was long enough. The discard does
not lose coverage: the sweep runs down from the start frequency and then back up through it,
so the start frequency is measured again on the upward pass.

The discard applies at both points where a sweep begins from a changed drive state — the
initial request, and the transition to `quadAThighP`.

---

## Delay summary after this patch

| Point | Parameter | Default | Covers |
|---|---|---|---|
| Sweep start, drive change | `quadATstartDelay` | 4.0 s | RF level decay from a previous state |
| Coarse per step | `quadATstepDelay` | 0.5 s | Frequency step settling |
| Fine per step | `quadATfineDelay` | 1.0 s | Resolving small level differences |

---

## Bench sequence

1. **Reproduce the original failure first.** Tune, confirm success, then immediately tune again
   without reducing drive. Pre-patch this failed at ~948 kHz. It should now either succeed or
   print the suspect warning and show it on the display.
2. If it still fails, raise `SRFATSTARTDLY` — 60, then 80 — until it is reliable. That number
   is the RF level decay time, which is worth knowing.
3. Once reliable, try lowering `SRFATDLY` (coarse) again to recover tune time. The start delay
   is paid once; the per-step delay is paid hundreds of times.
4. Then the narrowed-range test: `SRFATMINF,650000`, `SRFATMAXF,750000`, `SRFATSTEP,2000`.

Worth watching for: if the suspect warning fires on a tune you believe is *correct*, the
heuristic is too aggressive and needs adjusting. A legitimate resonance within one coarse step
of 950 kHz would trip it falsely.
