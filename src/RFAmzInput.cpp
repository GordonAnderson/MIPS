//
// RFAmzInput
//
// Set the RF amp (QUAD) m/z from an analog voltage. The voltage is on a MIPS ADC input, and a digital input is
// the strobe: when the strobe goes high the ADC value is latched and applied as the new m/z. In the style of the
// ESI and filament ADC controls, per RF amp module (1 or 2), RAM only.
//
//   SRFAMZADC,module,adc,slope,intercept   m/z = counts * slope + intercept. adc is the MIPS ADC channel 0 to 9, -1 none
//   SRFAMZSTB,module,Q..X|NA                the strobe digital input, rising edge, NA for none
//   GRFAMZADC, GRFAMZSTB, GRFAMZSTAT        read back the settings and the counts
//
// HOW IT WORKS
//   The strobe interrupt (DIhandler, rising edge) reads the ADC itself, 4 reads averaged, and records the time. It
//   cannot wait for the 10 mS thread: the value must be taken at the edge because the source holds it for only
//   about 10 mS, and a pass of the scheduler can take longer than that. A 10 mS thread then applies it:
//     - the m/z must be 10 to 40000 (the front panel limits), otherwise the strobe is ignored
//     - the mass calibration table is applied if it is enabled (QUADcalApply, as QSCAN does) and the m/z must not be
//       further outside the table than QSCAN allows (QUADcalRangeOK); with the table disabled it is a pass-through
//     - if the module is enabled it is applied with RFAsetScanPoint, the same primitive QSCAN and RFAACQ use, so the
//       hardware updates at once; if the module is disabled the m/z is stored and the normal update flag set
//   A strobe more than 500 mS old when the thread gets to it is discarded, and QSCAN clears any pending strobe when
//   it ends, so a strobe that arrived while a scan blocked the threads is never applied late.
//
// DO NOT USE WITH A SCAN IN PROGRESS. QSCAN blocks the threads (a strobe is ignored), and a host driven RFAACQ scan
// sets the m/z itself; both would fight a strobe. A strobe is also skipped, and counted as missed, while the MIPS
// ADC digitizer owns the ADC. The strobe input is the physical rising edge at the pin, before any DIinvert setting.
//
// Rev 1 and 2 modules need their resolving DC channel set (SRFADCCH), as QSCAN does, otherwise the commands NAK.
//
// STATUS AND TEST PLAN
//   Status: written, builds. NOT yet run on hardware. Test plan Part I in MIPS_QUAD_SCAN_TESTPLAN.md.
//
#include <Arduino.h>
#include <Thread.h>
#include <ThreadController.h>
#include <DIhandler.h>
#include "Serial.h"
#include "Errors.h"
#include "Hardware.h"
#include "RFamp.h"
#include "QUADscan.h"
#include "ADCdrv.h"
#include "RFAmzInput.h"

extern ThreadController control;
extern bool             QUADupdateFlag;

#define FMZ_ISR_SAMPLES  4          // ADC reads averaged in the strobe interrupt
#define FMZ_THREAD_MS    10         // The apply thread
#define FMZ_STALE_MS     500        // A strobe older than this when the thread runs is discarded
#define FMZ_MZ_MIN       10.0f      // Front panel limits for the m/z
#define FMZ_MZ_MAX       40000.0f

typedef struct
{
  int               adc;            // MIPS ADC channel 0 to 9, -1 for none
  float             m;
  float             b;
  char              strobe;         // Digital input Q to X, 0 if none
  DIhandler         *di;            // Interrupt handler, created when a strobe is first assigned
  volatile bool     pending;        // A strobe has latched a value that has not been applied yet
  volatile float    counts;         // The latched ADC counts
  volatile uint32_t ms;             // millis() at the strobe
  volatile uint32_t missed;         // Strobes skipped because the ADC was in use
  float             requested;      // Last m/z asked for, applied or not
  float             applied;        // Last m/z commanded
  uint32_t          nApplied;
  uint32_t          nRejected;      // Out of range, out of the calibration range, or no resolving DC channel
  uint32_t          nStale;
} FMZdata;

static FMZdata *FMZarray[2] = {NULL,NULL};
static Thread  *FMZthread   = NULL;

// The strobe interrupt: latch the ADC now
static void FMZstrobe(int b)
{
  FMZdata *z = FMZarray[b];
  long     sum = 0;

  if((z == NULL) || (z->adc < 0)) return;
  if(!AcquireADC()) { z->missed++; return; }          // The digitizer owns the ADC
  for(int j = 0; j < FMZ_ISR_SAMPLES; j++) sum += analogRead(z->adc);
  ReleaseADC();
  z->counts  = (float)sum / FMZ_ISR_SAMPLES;
  z->ms      = millis();
  z->pending = true;
}
static void FMZisr0(void) { FMZstrobe(0); }
static void FMZisr1(void) { FMZstrobe(1); }

// Applies latched strobes
static void FMZloop(void)
{
  for(int b = 0; b < 2; b++)
  {
    FMZdata *z = FMZarray[b];
    float    counts,req,cmd,delta;
    uint32_t ms;

    if((z == NULL) || (!z->pending)) continue;
    // Take the latched value and clear the flag together, so a new strobe is not lost or half read
    noInterrupts();
    counts     = z->counts;
    ms         = z->ms;
    z->pending = false;
    interrupts();
    if(RFAarray[b] == NULL) continue;
    if((millis() - ms) > FMZ_STALE_MS) { z->nStale++; continue; }
    req = counts * z->m + z->b;
    z->requested = req;
    if((req < FMZ_MZ_MIN) || (req > FMZ_MZ_MAX) || (!QUADcalRangeOK(b, req, req)) || (!RFAresolvingDCReady(b)))
    {
      z->nRejected++;
      continue;
    }
    // The calibration table maps the requested (true) m/z to the m/z to command and the resolving DC offset. With
    // the table disabled or empty this is a pass-through: cmd = req, delta = 0.
    QUADcalApply(b, req, &cmd, &delta);
    if(RFAarray[b]->Enabled) RFAsetScanPoint(b, cmd, delta);
    else
    {
      // Module off: store the m/z and let the normal update path recalculate. Nothing is written to the hardware.
      RFAarray[b]->mz = cmd;
      QUADupdateFlag = true;
    }
    z->applied = cmd;
    z->nApplied++;
  }
}

void RFAmzInputClear(void)
{
  for(int b = 0; b < 2; b++) if(FMZarray[b] != NULL) FMZarray[b]->pending = false;
}

// ---------------------------------------------------------------------------------------------
// Host commands. Module is 1 or 2, as for the other RF amp commands.
// ---------------------------------------------------------------------------------------------

// Returns the board's settings, or NULL after sending the NAK
static FMZdata *FMZget(int module, int *b)
{
  if((*b = RFAmodule2board(module)) == -1) return NULL;           // It has sent the NAK
  if(FMZarray[*b] == NULL) { SetErrorCode(ERR_BADARG); SendNAK; return NULL; }
  return FMZarray[*b];
}

// SRFAMZADC,module,adc,slope,intercept. A line command like ESI's SESICTRL.
void FMZsetADC(void)
{
  int     module,adc,b;
  float   slope,intercept;
  FMZdata *z;

  while(true)
  {
    if(!valueFromCommandLine(&module, 1, 2)) break;
    if(!valueFromCommandLine(&adc, -1, 9)) break;
    if(!valueFromCommandLine(&slope, -100000, 100000)) break;
    if(!valueFromCommandLine(&intercept, -100000, 100000)) break;
    if((z = FMZget(module, &b)) == NULL) return;
    // Rev 1 and 2 route resolving DC through a DC bias channel that must be configured, as QSCAN requires
    if(!RFAresolvingDCReady(b)) { SetErrorCode(ERR_QUADNODCBCHAN); SendNAK; return; }
    z->adc     = adc;
    z->m       = slope;
    z->b       = intercept;
    z->pending = false;
    SendACK;
    return;
  }
  BADARG;
}

void FMZgetADC(int module)
{
  int      b;
  FMZdata *z = FMZget(module, &b);

  if(z == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  serial->print(z->adc);
  serial->print(",");
  serial->print(z->m, 6);
  serial->print(",");
  serial->println(z->b, 6);
}

// SRFAMZSTB,module,Q..X|NA
void FMZsetStrobe(char *Module, char *value)
{
  int      b;
  String   t = Module;
  String   v = value;
  FMZdata *z = FMZget(t.toInt(), &b);

  if(z == NULL) return;
  if(v == "NA")
  {
    if(z->di != NULL) z->di->detach();
    z->strobe  = 0;
    z->pending = false;
    SendACK;
    return;
  }
  if((value[0] < 'Q') || (value[0] > 'X') || (value[1] != 0)) BADARG;
  if(!RFAresolvingDCReady(b)) { SetErrorCode(ERR_QUADNODCBCHAN); SendNAK; return; }
  if(z->di == NULL) z->di = new DIhandler;
  if(z->di == NULL) BADARG;
  z->di->detach();                                                 // Moving from another input, if it had one
  if(!z->di->attached(value[0], RISING, (b == 0) ? FMZisr0 : FMZisr1)) BADARG;
  z->strobe  = value[0];
  z->pending = false;
  SendACK;
}

void FMZgetStrobe(int module)
{
  int      b;
  FMZdata *z = FMZget(module, &b);

  if(z == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  if(z->strobe == 0) serial->println("NA");
  else serial->println(z->strobe);
}

// Reports requested m/z, applied m/z, strobes applied, rejected, stale, missed (ADC busy)
void FMZgetStat(int module)
{
  int      b;
  FMZdata *z = FMZget(module, &b);

  if(z == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  serial->print(z->requested, 3);
  serial->print(",");
  serial->print(z->applied, 3);
  serial->print(",");
  serial->print(z->nApplied);
  serial->print(",");
  serial->print(z->nRejected);
  serial->print(",");
  serial->print(z->nStale);
  serial->print(",");
  serial->println(z->missed);
}

// Command table. const, so it is in flash: every entry is a function. Registered by RFAmzInput_init.
const Commands FMZCmdArray[] = {
// Start of command block
// RF amp / QUAD m/z set from an ADC input and applied by a digital input strobe. Module is 1 or 2.
// Do not use while a scan (QSCAN or RFAACQ) is in progress. See RFAmzInput.cpp.
  {"SRFAMZADC",  CMDfunctionLine, 0, (char *)FMZsetADC},      // Set the ADC input that sets the m/z, module,adc,slope,intercept. adc 0 to 9, -1 for none. m/z = counts * slope + intercept
  {"GRFAMZADC",  CMDfunction,     1, (char *)FMZgetADC},      // Get adc,slope,intercept for the m/z ADC input
  {"SRFAMZSTB",  CMDfunctionStr,  2, (char *)FMZsetStrobe},   // Set the digital input that strobes the ADC m/z into the module, module,Q to X or NA. Rising edge
  {"GRFAMZSTB",  CMDfunction,     1, (char *)FMZgetStrobe},   // Get the m/z strobe digital input, Q to X or NA
  {"GRFAMZSTAT", CMDfunction,     1, (char *)FMZgetStat},     // Get requested m/z,applied m/z,applied count,rejected count,stale count,missed count
// End of table marker
  {0},
};

CommandList FMZCmdList = { (Commands *)FMZCmdArray, NULL };

// Called from RFA_init for each board
void RFAmzInput_init(int8_t Board)
{
  if((Board < 0) || (Board > 1)) return;
  if(FMZarray[Board] == NULL)
  {
    FMZarray[Board] = new FMZdata;
    if(FMZarray[Board] == NULL) return;
    FMZdata *z = FMZarray[Board];
    z->adc       = -1;
    z->m         = 1;
    z->b         = 0;
    z->strobe    = 0;
    z->di        = NULL;
    z->pending   = false;
    z->counts    = 0;
    z->ms        = 0;
    z->missed    = 0;
    z->requested = 0;
    z->applied   = 0;
    z->nApplied  = 0;
    z->nRejected = 0;
    z->nStale    = 0;
  }
  // The commands and the thread are set up once, for the first board
  if(FMZthread == NULL)
  {
    AddToCommandList(&FMZCmdList);
    FMZthread = new Thread();
    FMZthread->setName("RFAmz");
    FMZthread->onRun(FMZloop);
    FMZthread->setInterval(FMZ_THREAD_MS);
    control.add(FMZthread);
  }
}
