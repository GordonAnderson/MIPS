# MIPS QUAD — data collection reference

How to set up and run each way of collecting data from the RF amp / QUAD module, with the exact serial commands, the
binary stream format, the timing to expect, and the traps found in the lab. Written October 17, 2026 for operating the
QUAD and for writing host software. Companion documents: [`MIPS_QUAD_SCAN_TESTPLAN.md`](MIPS_QUAD_SCAN_TESTPLAN.md)
(what has been tested and the results), [`MIPS_QUAD_FIRMWARE_HANDOFF.md`](MIPS_QUAD_FIRMWARE_HANDOFF.md) (design and
open items), and `MIPScommands.txt` (the generated list of every command).

**Status of this document.** Everything here comes from the firmware source. Items marked **(lab)** were also run on
hardware on October 17, 2026; items marked **(not run)** have not been.

---

## 1. Conventions

- **Arguments are comma separated**, on every MIPS command and on every electrometer command: `SQSSTRT,1,200`,
  `TWI1CMD,32,SMODE,1`. A space does not work (the electrometer answers NAK). **(lab)**
- **Module** is 1 or 2, the RF amp module number.
- A command answers with an ACK byte (0x06) plus any value, or a NAK (0x15, then `?`). After a NAK, `GERR` returns the
  error code (section 10).
- **The scan settings (`SQS...`, `SRFAMZ...`, `SQSE...`) are RAM only.** They return to their defaults at power up, so
  send them again every time. The calibration table is separate and can be saved (section 9).
- Electrometer settings (`SNUMP`, `SSRATE`, mode...) are also RAM only unless you run `TWI1CMD,32,SAVE`.

---

## 2. Which mode to use

| You want | Use | Section |
|---|---|---|
| A spectrum, the MIPS acquires, fastest | `QSCAN` with the MIPS ADC | 4 |
| A spectrum from the electrometer, fastest | `QSCAN` with the electrometer | 5 |
| The MIPS only steps m/z, your own DAQ records | `QSCAN` with acquisition off | 6 |
| Your script chooses every m/z, MIPS acquires | `RFAACQ` with the ADC or electrometer | 7 |
| m/z set by an analog voltage, applied by a pulse | ADC m/z input with a strobe | 8 |
| Mass-calibrated m/z | the calibration table | 9 |

---

## 3. Common to every `QSCAN`

```
SRFAENA,1,ON                          the module must be enabled and main power on
SQSSTRT,1,200                         start m/z
SQSSTOP,1,1200                        stop m/z
SQSSTEP,1,1                           step (use a negative step for a descending scan)
SQSDWELL,1,5                          settle time per point, ms (default 2)
SQSNUM,1,1                            number of scans (default 1)
QSCANSTAT,1                           review everything, the point count and the estimated time
QSCAN,1                               run it
```

- At most **2000 points** per scan. The start/stop/step must give at least one point.
- `QSCAN` blocks: nothing else runs until it finishes. **Send ESC (0x1B) to abort.** **(lab)**
- When it finishes the quad is **parked back at the start m/z**. **(lab)**
- Defaults: start 50, stop 500, step 1, dwell 2, 1 scan, source ADC, acquisition on.
- Rev 1 and 2 modules need the resolving DC channel set first (`SRFADCCH`); Rev 3 does not.
- Settle time: the dwell is the time the quad gets to settle at each new m/z before acquiring. The previous point's
  data is streamed out during that settle.

### Stream format

`QSCAN` replies with an ACK byte (0x06), then for each scan:

| Part | Bytes | Content |
|---|---|---|
| Header | 11 | `55 55 55 55 AA`, points (3 bytes, little-endian), scan number (2 bytes, little-endian), last-scan flag (`FF` on the final scan, `00` otherwise) |
| Points | 4 each | one **signed 32-bit little-endian sum** per point, in order |
| Trailer | 5 | `AE AE AE AE`, then `EA` (completed) or `EB` (aborted) |

Every header gets a trailer, including when the scan fails or is aborted. The value is the **raw sum** of the samples,
not an average (divide by the number of samples in the burst). **(lab)**

A reference parser (Python, `pyserial`):

```python
import struct

def read_exact(ser, n):
    buf = b""
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk: raise TimeoutError("scan stream stopped")
        buf += chunk
    return buf

def read_scan(ser):
    """Read one frame after QSCAN has been sent and the ACK consumed. Returns (scan_number, last, points, aborted)."""
    # find the header: 55 55 55 55 AA
    sync = b""
    while sync != b"\x55\x55\x55\x55\xAA":
        sync = (sync + read_exact(ser, 1))[-5:]
    hdr = read_exact(ser, 6)
    npts = hdr[0] | (hdr[1] << 8) | (hdr[2] << 16)
    scan = hdr[3] | (hdr[4] << 8)
    last = hdr[5] == 0xFF
    points = [struct.unpack("<i", read_exact(ser, 4))[0] for _ in range(npts)] if npts else []
    # a no-acquisition scan (section 6) has a header and a trailer with no points between them
    trailer = read_exact(ser, 5)
    assert trailer[:4] == b"\xAE\xAE\xAE\xAE", "bad trailer"
    return scan, last, points, trailer[4] == 0xEB
```

If a scan is **aborted or fails** the trailer arrives with fewer points than the header promised, so a parser that
trusts the header count must also watch for the trailer. For a robust reader, look for the `AE AE AE AE` pattern as well
as counting points.

---

## 4. Mode A: `QSCAN` with the MIPS ADC

```
SQSSRC,1,ADC                          the default
SADCCHAN,0                            which MIPS ADC input (default 0)
SADCSAMPS,200                         samples summed per point (default 5000)
SADCRATE,10000                        sample rate, Hz (default 200000)
```

- Acquire time per point = `SADCSAMPS / SADCRATE`. The defaults give 25 ms per point, which is slow; choose them to
  suit the signal.
- **`QSCAN` sets up the digitizer itself.** You do not send `ADCRBINIT`, and it releases the ADC when done.
- The streamed value is the raw sum of `SADCSAMPS` samples.
- Measured: 1001 points, dwell 5 ms, **6.0 s** (about 6 ms per point). **(lab)**
- The ADC input must have a signal on it; an unconnected channel gives zeros.

---

## 5. Mode B: `QSCAN` with the electrometer

The MIPS triggers an IsolatedElectrometer on its **TWI1 (Wire1)** bus, waits for its burst, and streams the burst sum.

### 5.1 Set up the electrometer (once, through the MIPS)

```
TWI1CMD,32,SSRATE,10000               burst sample rate, Hz
TWI1CMD,32,SNUMP,100                  samples per burst (10 to 5000)
TWI1CMD,32,SMODE,1                    triggered burst mode
TWI1CMD,32,CONFIG                     apply (see the warning below)
TWI1CMD,32,GTRIGST                    must reply ARMED
TWI1CMD,32,SAVE                       optional: keep these settings across a power cycle
```

> **`CONFIG` is required whenever you change `SSRATE`, `SNUMP` (or `SANUMS`) while the electrometer is already in mode 1.**
> Those commands only store a value. The timer and the burst buffer are set up by `SMODE` and `CONFIG`. If you set the
> values first and then send `SMODE,1`, `CONFIG` is not needed. If you change them afterwards without `CONFIG`, the
> electrometer keeps running the old burst. `GNUMP` and `GSRATE` read back the stored setting, so they show the new
> value even when it has not been applied. This hid a real result for a long time. **(lab)**

- **Arming runs one burst with no trigger.** After `CONFIG`, wait for that burst to finish (the burst time plus a
  moment) and check `GTRIGST` says `ARMED` before you start a scan, otherwise the preflight fails with error 133.
- **Burst time = `SNUMP / SSRATE`** (plus about 1 ms to publish it). This is accurate for firmware from commit
  `37eb120` on; earlier electrometer firmware ran at one third of `SSRATE`. **(lab)**
- **Keep `SSRATE` at or below 100000.** The electrometer's ADC sustains 100 kS/s; above that it drops trigger events, the
  burst takes 2 to 3 times as long, and `GSRATEA` does not show it. **(lab)**
- Settings are lost at an electrometer power cycle unless you `SAVE`.

### 5.2 Set up and run the scan on the MIPS

```
SQSSRC,1,ELECT                        acquisition source: the electrometer
SQSEADD,1,32                          its address, decimal (default 32 = 0x20)
SQSETRG,1,TWI                         how to trigger it: TWI (software) or HW (LT pin), see below
SQSETMO,1,1000                        max ms to wait for one point: MUST exceed the burst time (default 1000)
QSCAN,1                               the range and dwell as in section 3
```

- **Trigger method.** `SQSETRG,1,HW` (the **default**) pulses LT, pin 13, for 50 microseconds and needs LT wired to the
  electrometer's TRIGIN **and routed to the right pin on the controller**; on some controller revisions it is not, and
  the scan then times out with error 134 having seen no edge. **`SQSETRG,1,TWI` works on every controller** and sends the
  trigger over the I²C bus. Use TWI unless you have confirmed the hardware trigger on your unit. **(lab: TWI passes;
  HW failed on a controller where LT is not routed. HW passed in a bench test of the wiring with LT driven by `DWRITE`.)**
  Because the default is HW, send `SQSETRG,1,TWI` after every MIPS power up.
- The scan checks the electrometer before it starts. If the electrometer does not answer, is not in mode 1, or is not
  armed, `QSCAN` is refused with a NAK before any data, `GERR` is **133**. **(lab)**
- If a point does not finish within `SQSETMO` the scan stops with an aborted trailer (`EB`) and `GERR` **134**. **(lab)**
- The streamed value is the **burst sum** (the sum of `SNUMP` 12-bit samples; up to 4095 × 5000 = 20,475,000, which needs
  more than 24 bits).
- `SQSEADD`, `SQSETRG` and `SQSSRC` can be changed between scans.

### 5.3 Turning the sums into current

The electrometer's counts go **down** as the current goes up (so peaks appear as dips in the raw sums):

```
counts = sum * 16 / SNUMP                      16-bit left-justified average
nA     = (zeroCounts - counts) / (Calfactor * gain)        (x 1000 for the pA range)
```

`zeroCounts` is set by `TWI1CMD,32,ZERO` with no beam (the factory value is 45000), `Calfactor` is counts per nA at unit
gain (factory 1977), and `gain` is `SGAIN`. Polarity is `SPOL`, range `SRNG`. For a single reading, `TWI1CMD,32,RCUR`
returns the calibrated current of the last burst.

### 5.4 Time per point

About **dwell + burst + 0.4 ms of TWI (at 400 kHz) + about 1 ms of overhead**. Raise the bus speed with
`STWISPEED,1,400000` (default 100000), which cuts the TWI part of each point from about 1.2 ms to about 0.4 ms; check that
your other Wire1 modules still read correctly at 400 kHz. Measured: dwell 5 ms, 100 samples at 10 kS/s, TWI trigger (bus
speed not recorded): **17.6 ms per point**. **(lab)**

---

## 6. Mode C: external DAQ (the MIPS only steps m/z)

The MIPS steps through the m/z range on the dwell timer and your DAQ records, synchronised to a pulse on a digital output.

```
SQSADCENA,1,FALSE                     the MIPS does not acquire
SQSFRAME,1,FRAME                      FRAME: still send the header and trailer per scan, no points. NONE: send nothing
SQSSTPIN,1,A                          200 us pulse on digital output A..P at the start of every scan (NA = off)
SQSTRIG,1,TRUE                        TRGOUT driven for the duration of each point
QSCAN,1                               the range and dwell as in section 3
```

- With `FRAME` the stream is a header and a trailer per scan with no points between them. With `NONE` the MIPS sends only
  the ACK. ESC still aborts either way, and a command sent right afterwards is answered. **(lab, by serial)**
- The start pulse comes before the first point of **each** scan (so `SQSNUM` scans give that many pulses). It goes out
  through the digital output bank, so choose a channel that is not wired to something else (interlock, FAIMS...).
- `TRGOUT` is inverted on the controller, so check its polarity on a scope. **(not run: pulses not scoped)**
- The time per point is just the dwell plus a little overhead.
- Set `SQSADCENA,1,TRUE` to go back to the normal modes.
- **Scan only, record nothing** (no ADC, no electrometer, no data back): `SQSADCENA,1,FALSE` with `SQSFRAME,1,NONE`, and leave
  `SQSSTPIN` at `NA` and `SQSTRIG` at `FALSE` if no external DAQ needs the pulses. The module just steps m/z on the dwell
  timer, then parks at the start m/z. See the recipe in section 13.

---

## 7. Mode D: host-stepped `RFAACQ`

Your script chooses every m/z and calls `RFAACQ,module,mz,delta,dwell` once per point.

- `mz` is 10 to 100000, `delta` is the resolving DC offset in volts (-10 to 10), `dwell` is the settle time in ms (0 to
  1000).
- **Each call returns the previous point's sum**, sets the quad to the new m/z, waits the dwell, and starts the next
  acquisition. The first call of a scan returns 0. Make one extra call at the end to read the last point.
- It does **not** use the calibration table: you supply the m/z and the delta you want.

### With the MIPS ADC

```
SADCCHAN,0   SADCSAMPS,200   SADCRATE,10000
SADCVECTS,1002                        MUST be at least points + 2, or the ADC shuts itself down mid scan
ADCRBINIT                             claim the digitizer
RFAACQ,1,200,0,0                      first call: returns 0, sets m/z 200
RFAACQ,1,201,0,0                      returns the sum for 200, sets 201 ...
ADCABORT                              when finished: release the ADC
```

### With the electrometer

```
(set up the electrometer as in section 5.1, then on the MIPS:)
SQSSRC,1,ELECT   SQSETRG,1,TWI   SQSETMO,1,1000
SQSEINIT,1                            send once before each scan: checks the electrometer, clears the previous point
RFAACQ,1,200,0,0                      first call: returns 0
RFAACQ,1,201,0,0                      returns the previous sum
```

- A failed point returns **-1** (a sum is never negative) with `GERR` 134; `SQSEINIT` again recovers. **(lab)**
- `RFAACQ` is refused with `GERR` 133 until `SQSEINIT` has been sent, and again after you change `SQSSRC`, `SQSEADD` or
  `SQSETRG`. **(lab)**
- Do **not** run it at the same time as `QSCAN` or an m/z strobe.

### Speed

Measured, an 800 point scan, dwell 0: **3.3 s with the ADC (4.1 ms per point) and 3.7 s with the electrometer (4.6 ms per
point)** (400 kHz bus, 50 kHz sample rate, 10 samples). **(lab)** The electrometer adds only about 0.5 ms per point (the counters
show a 69 microsecond trigger and a 334 microsecond read, 0.4 ms). The floor of about 4.1 ms is the command round trip, the
script and the plotting. Divide a scan time by the **actual** number of points; an earlier version of this page used 1000 and
was 20% low. To go faster:
- use `QSCAN` (no host in the per point path). **(lab)** Measured with the electrometer, TWI trigger, 400 kHz bus,
  100 kHz sample rate, 10 samples: about **2.7 ms per point** (about 2.2 s for 800 points), including the host's acquire and
  plotting time, so the scan engine alone is faster than that,
- or send several `RFAACQ` lines before reading the replies,
- or redraw the plot every 10 to 20 points.

To see how much of the per point time is the set-point change, send `RRFASPT,1` (reset), run a scan, then `GRFASPT,1`. It returns
`points,avgTotalUs,minTotalUs,maxTotalUs,avgUpdateUs,avgRFUs,avgDCUs`: the firmware time `RFAsetScanPoint` takes (the m/z
calculation, the RF level DAC, the resolving DC), in every scan mode. It does not include the physical settling of the RF amplifier
and DC supplies after the DACs are written, which needs a scope.

**(lab, October 18, 2026)** Measured on a module that is not Rev 3, where the RF level DAC is an AD5625 on TWI channel 0 and the
resolving DC is the DC bias board (AD5668, SPI):

| Part of `RFAsetScanPoint` | TWI 0 at 100 kHz | TWI 0 at 400 kHz |
|---|---|---|
| m/z calculation | 25 us | 25 us |
| RF level DAC (AD5625, TWI) | 399 us | 118 us |
| Resolving DC (AD5668, SPI) | 35 us | 35 us |
| **Total** | **460 us** | **179 us** |

Set with `STWISPEED,0,400000` (every device on channel 0 must support 400 kHz; check the module EEPROMs read back and `TWIfails`).
The Rev 3 hardware writes both through SPI, so its RF write should be faster still.

`QSCAN` with the electrometer (TWI trigger, 400 kHz, 100 kHz sample rate, 10 samples), 800 points: **2.3 ms per point at the
host, 1.13 ms per point for the data transfer alone**, so about 1.2 ms per point is the host's plotting. Speeding up the set-point
write moved the total only 0.1 ms, because the host's receive and plot set the pace, not the MIPS. Look at the host side next.

---

## 8. Mode E: m/z from an analog voltage, applied by a strobe

An external system sets a voltage on a MIPS ADC input and pulses a digital input; the MIPS reads the voltage at the edge
and applies it as the new m/z. **(not run on hardware)**

```
SRFAMZADC,1,3,9.768,0                 m/z = ADC counts * slope + intercept. ADC channel 3 here, 0 to 9, -1 for none
SRFAMZSTB,1,R                         strobe digital input Q..X, rising edge. NA to turn it off
GRFAMZSTAT,1                          requested m/z, applied m/z, applied, rejected, stale and missed counts
```

- The ADC is 12-bit, 0 to 4095 counts for 0 to 3.3 V at the pin, so `slope = 40000 / 4095 = 9.768` and `intercept = 0`
  gives 0 to 40000 m/z for 0 to 3.3 V. Divide a higher input voltage down yourself.
- The voltage is **latched in the strobe interrupt** (4 reads averaged), so it must be valid when the rising edge
  arrives and stable for a few microseconds. A 10 ms thread then applies it.
- The m/z must be 10 to 40000, otherwise the strobe is ignored and counted as rejected. If the calibration table is
  enabled it is applied, and an m/z too far outside the table is also rejected.
- If the module is enabled the new m/z reaches the hardware at once; if it is disabled the value is stored.
- A strobe is skipped (counted as missed) while the MIPS ADC digitizer owns the ADC. A strobe that is more than 500 ms old
  when the thread gets to it is discarded.
- **Do not use it during a scan.** `QSCAN` ignores strobes while it runs and clears any that arrived, and a host
  `RFAACQ` scan would fight it.
- The edge is the physical rising edge at the pin, before any digital-input inversion setting.

---

## 9. The mass calibration table

Corrects the commanded m/z and the resolving DC offset for `QSCAN` and the strobe input (not `RFAACQ`). Each row says: to
put the peak at the **true** m/z, command this **measured** m/z with this resolving DC **delta**.

```
SQCNUM,1,3                            number of rows, 0 to 10 (0 clears)
SQCPNT,1,1,100.0,100.4,0.0            row index (1 based), true m/z, m/z to command, delta in volts
SQCPNT,1,2,500.0,500.9,0.1
SQCPNT,1,3,1000.0,1001.8,0.2         "true m/z" must be strictly ascending with the index
SQCENA,1,TRUE                         apply the table
RQCAL,1                               show the whole table
SAVEQCAL,1                            keep it in the module EEPROM (RESTOREQCAL,1 reloads it)
QSCANSTAT,1                           shows what the table does at the scan endpoints
```

- Between rows the values are interpolated linearly. Outside the table they are extrapolated from the end rows, but a scan
  that starts or stops more than **2 times the table span** beyond the table is refused with `GERR` 131.
- The table is not part of the normal `SAVE`; use `SAVEQCAL`.
- With the table disabled (the default) the commanded m/z equals the requested m/z.

---

## 10. Error codes (`GERR` after a NAK)

| Code | Meaning |
|---|---|
| 1 | unknown command (also: a module or option is not installed, for example the filament commands with no filament module) |
| 2 | bad argument |
| 124–126 | the MIPS ADC is in use / already set up / not set up |
| 127 | `QSCAN`: RF module not enabled |
| 128 | `QSCAN`: main power is off |
| 129 | `QSCAN`: start/stop/step do not give a valid scan |
| 130 | `QSCAN`: more than 2000 points |
| 131 | `QSCAN`: range too far outside the calibration table |
| 132 | Rev 1/2 module has no resolving DC channel set (`SRFADCCH`) |
| 133 | electrometer not answering, not in mode 1, or not armed (also `RFAACQ` before `SQSEINIT`) |
| 134 | electrometer did not finish a point inside `SQSETMO` (or a point failed mid scan) |
| 135 | LT is in use by the DC bias profile input, so the hardware trigger cannot use it |

Line-type commands (`SQCPNT`, `SRFAMZADC`, `SQSEADD`...) can report error 1 instead of 2 for a bad value, because the
unread arguments are parsed as commands and overwrite the error code. The NAK itself is correct.

---

## 11. Checking what is going on

| Command | Gives you |
|---|---|
| `QSCANSTAT,1` | all the scan settings, the global ADC settings, the point count, the estimated scan time, and what the calibration table does at the ends of the range |
| `GQSESTAT,1` | electrometer timing for the **last point**: `points,trigUs,waitUs,completeUs,polls,readUs,readMaxUs,retries,fails` (microseconds; reset at each `QSCAN` / `SQSEINIT`) |
| `GQSESUM,1` | the same over the **whole scan**: `points,avgCompleteUs,minCompleteUs,maxCompleteUs,avgWaitUs,avgPolls x100,maxPolls,points needing more than one poll` |
| `GTWISPEED,1` | the Wire1 bus speed setting (it prints the variable, not the measured clock) |
| `TWI1SCAN` | which addresses answer on Wire1; the electrometer should show at `20` |
| `TWI1CMD,32,GTRIGST` / `GTRIGCNT` / `GTRIGMIS` | electrometer trigger state, completed bursts, and triggers that arrived mid burst |
| `GTWI,01208B72` | the electrometer's new 7-byte status read, printed as an integer: the low 24 bits are the completed count, the top byte is the first byte of the sum |

`GQSESTAT` and `GQSESUM` need the MIPS firmware from commit `61aeab8` / `95b8a4b` or later.

---

## 12. Traps found in the lab

1. **`CONFIG` after changing the burst settings** (section 5.1). The commonest mistake.
2. **Spaces instead of commas.** The electrometer NAKs `SMODE 1`.
3. **The default trigger is HW.** Send `SQSETRG,1,TWI` after every power up unless the LT wiring is confirmed.
4. **`SQSETMO` shorter than the burst** stops the scan with error 134. The burst is `SNUMP / SSRATE`.
5. **Bus speed.** The Wire1 default is 100 kHz; `STWISPEED,1,400000` makes the electrometer polling about 3 times
   faster. A reading of about 990 microseconds per 7-byte read in `GQSESTAT` means the bus is at 100 kHz.
6. **Waiting after `CONFIG`.** The arming burst makes a scan started during it fail with 133.
7. **ADC limit.** Above 100000 Hz requested the electrometer loses events. Keep `SSRATE` at or below that.
8. **The raw sums dip for the electrometer** (more current is fewer counts). Convert with section 5.3.
9. **`GNUMP` / `GSRATE` show the stored value**, not what is running.
10. **Everything is RAM only.** Resend the scan setup after every power up, or `SAVE` the electrometer.
11. **Do not mix** a strobe, `QSCAN` and `RFAACQ`: they all set the m/z.
12. **An unconnected ADC input** reads zeros (MIPS ADC mode) or garbage (the strobe input): connect the signal first.

---

## 13. Copy and paste recipes

**Electrometer spectrum, 200 to 1200 m/z, step 1, fast and safe:**
```
STWISPEED,1,400000
TWI1CMD,32,SSRATE,50000
TWI1CMD,32,SNUMP,100                  2 ms burst
TWI1CMD,32,SMODE,1
TWI1CMD,32,CONFIG                     then wait a moment, check TWI1CMD,32,GTRIGST = ARMED
SQSSRC,1,ELECT
SQSETRG,1,TWI
SQSETMO,1,1000
SQSSTRT,1,200  SQSSTOP,1,1200  SQSSTEP,1,1  SQSDWELL,1,2  SQSNUM,1,1
QSCANSTAT,1
QSCAN,1
```
An estimate, not a measurement: 2 ms dwell + 2 ms burst + 0.4 ms TWI + about 1 ms of overhead, so roughly 5 to 6 ms
per point, 5 to 6 s for 1001 points.

**ADC spectrum, same range:**
```
SQSSRC,1,ADC
SADCCHAN,0  SADCSAMPS,50  SADCRATE,50000     1 ms acquisition
SQSSTRT,1,200  SQSSTOP,1,1200  SQSSTEP,1,1  SQSDWELL,1,2  SQSNUM,1,1
QSCAN,1
```

**External DAQ, three scans, start pulse on output A:**
```
SQSADCENA,1,FALSE  SQSFRAME,1,FRAME  SQSSTPIN,1,A  SQSTRIG,1,TRUE
SQSSTRT,1,200  SQSSTOP,1,1200  SQSSTEP,1,1  SQSDWELL,1,5  SQSNUM,1,3
QSCAN,1
```

**Scan only, no data recorded anywhere (the MIPS just steps m/z; `NONE` sends nothing but the ACK):**
```
SRFAENA,1,ON
SQSADCENA,1,FALSE  SQSFRAME,1,NONE  SQSSTPIN,1,NA  SQSTRIG,1,FALSE
SQSSTRT,1,200  SQSSTOP,1,1200  SQSSTEP,1,1  SQSDWELL,1,5  SQSNUM,1,1
QSCAN,1
```
ESC aborts. Set `SQSSTPIN` and `SQSTRIG,1,TRUE` instead if an external DAQ should be synchronised (the start pulse uses the
digital output bank, so pick a free channel; the `TRGOUT` polarity is not scoped yet).

**Back to normal after any of the above:**
```
SQSADCENA,1,TRUE  SQSSTPIN,1,NA  SQSTRIG,1,FALSE  SQSSRC,1,ADC
```
