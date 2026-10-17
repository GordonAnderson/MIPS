//
// FilamentEmission
//
// Closed loop emission current regulation for the filament module. The user sets an emission current in uA
// and the filament current is adjusted to hold it. This is a client of the filament module, not part of it:
//
//   - It reads the module's existing globals (FDarray, FilamentBoards, EmissionI, Fpowers) and writes only the
//     commanded filament current, FDarray[b].FCD[].CurrentSetpoint, the same value the dialog, SFLI, the
//     filament cycling and the maximum power limiter already adjust. The ramp limiter, DAC output, power
//     limiter, serial watchdog and disable ramp-down downstream of it are untouched.
//   - FilamentData, and so the module EEPROM image, is unchanged. The settings are RAM only.
//   - Its host commands are in its own table, registered by FilamentEmission_init, which is called from the
//     end of Filament_init. Its own 100 mS thread runs the loop. Nothing in the Filament loops was changed.
//
// The regulation law and its tuning are in FilamentEmissionCore.h and are tested on the host in
// tools/FilamentEmissionSim.cpp.
//
// Supported on Rev 3 and Rev 4 modules, which have the on-module emission current measurement, one per
// board, so two modules regulate independently. Serial channel numbers are the filament module channels:
// with one channel per module, channel 1 is the first module and channel 2 the second. The regulated
// channel of a module is its live channel, the one the current direction selects.
//
// The filament limit is the module's Max power setting (page 2 of the filament dialog). The controller does
// not raise the current while the filament power is at 97% of it, and the power limiter's trimming does the
// rest. There is no separate maximum current setting. If the emission setpoint cannot be reached inside the
// limit, the controller holds there with status PLIM and says nothing else. The controller cannot tell "no
// bias applied" from "not hot enough", because the bias can come from another system, so with no bias it
// raises the current up to the power limit and waits. When bias then arrives the emission can be well above
// the setpoint for a time, because the ramp limiter also limits how fast the current can come back down.
// Setting Max power only a little above the power at the operating point keeps that excursion small.
//
// ADC INPUTS AND ENABLE INPUT (added October 2026, in the style of the ESI module's SESICTRL)
//   Each filament channel can take its emission setpoint, or its heater (filament) current, from a MIPS ADC input,
//   and its enable from a digital input. All are RAM only and per module:
//     SFLEADC,chan,adc,slope,intercept   emission setpoint (uA) = counts * slope + intercept, 0 to the largest setpoint
//     SFLIADC,chan,adc,slope,intercept   heater current (A)     = counts * slope + intercept, 0 to 12 A
//     SFLDIO,chan,Q..X|NA                digital input that enables the filament (high = enabled)
//   adc is the MIPS ADC channel 0 to 9, or -1 for none. Counts are the 12 bit Due ADC, 10 reads averaged, 10 Hz.
//   If the heater current ADC is defined it owns the filament current and emission regulation is disabled
//   (SFLEREG,ON is refused and the status is HEATADC). It stands aside while filament cycling is enabled. So that
//   the Max power limiter still works it never commands more than 0.1 A ahead of the ramped current and never
//   raises the current while the filament power is at 97% of Max power (a rewrite of CurrentSetpoint every
//   pass would otherwise undo the limiter's trimming, see the loop comment and tools/FilamentEmissionSim.cpp). The enable input is
//   authoritative, as in ESI: a digital input that is low turns the filament off within 100 mS, whatever SFLENA
//   or the dialog says. A floating or disconnected ADC input reads garbage; Max power is then the protection.
//
// STATUS AND TEST PLAN
//   Status: written, host simulated, builds. NOT yet run on hardware. Test plan Part 12 in
//   MIPS_FILAMENT_TESTPLAN.md. Bench with a vacuum tube first.
//
#include <Arduino.h>
#include <Thread.h>
#include <ThreadController.h>
#include <stdlib.h>
#include "Serial.h"
#include "Errors.h"
#include "Filament.h"
#include "Variants.h"
#include "DIO.h"
#include "ADCdrv.h"
#include "FilamentEmission.h"
#include "FilamentEmissionCore.h"

// Things this file reads from Filament.cpp
extern bool            FilamentBoards[2];
extern float           Fpowers[2][2];
extern float           EmissionI[2];
extern float           CurrentSetpoints[2][2];     // The ramped (actual) current setpoint, [board][channel]
extern int             Channel2Index(int channel);
extern int8_t          BoardFromSelectedFilamentChannel(int8_t SC);
extern ThreadController control;
extern int             NumberOfFilamentChannels;

#define FEM_MAXCUR       12.0f      // Largest filament current, A. Matches MaxFilCur in Filament.cpp
#define FEM_PERIOD_MS    500        // Regulation control period
#define FEM_THREAD_MS    100        // Thread period: ADC inputs and the enable input run at 10 Hz
#define FEM_ADC_SAMPLES  10         // ADC reads averaged
#define FEM_AHEAD        0.1f       // The heater ADC never commands more than this many amps ahead of the ramped current
#define FEM_MAX_DT       2.0f       // A longer gap than this (s) is clamped, the thread was held up

// An ADC input that sets a value: counts * m + b. adc is the MIPS ADC channel, or -1 if not defined
typedef struct
{
  int          adc;
  float        m;
  float        b;
  float        value;               // Last value computed, in the units of the setpoint it sets
} FEMadc;

typedef struct
{
  FEMparams    P;                   // Setpoint and tuning handed to the core
  FEMadc       EmADC;               // ADC input that sets the emission setpoint, uA
  FEMadc       HtADC;               // ADC input that sets the heater current, A. Defined means regulation is disabled
  char         EnableIn;            // Digital input Q to X that enables the filament, 0 if none
  float        Emax;                // Largest emission setpoint accepted, uA
  bool         On;                  // Regulation enabled
  int          Status;              // FEMstatusCodes
  float        Measured;            // Last emission measurement used, uA
  FEMcoreState Core;
  uint32_t     LastMs;              // millis() of the last control step, 0 if none
} FEMdata;

static FEMdata *FEMarray[2] = {NULL,NULL};   // Allocated for each Rev 3 or 4 board at init
static Thread  *FEMthread   = NULL;

static const char *FEMstatusNames[] = {"OFF","WAIT","BLOCKED","SEEK","REG","PLIM","IMAX","HEATADC"};

// The index of a module's live channel in FCD[] and FCyl[], same rule as Channel2Index for Rev 2 and later
static int FEMlive(int b)
{
  return FDarray[b].Idir ? 0 : 1;
}

// Rev 2 and later keep both channel structures identical, so the setpoint is written to both
static void FEMsetCurrent(int b, float v)
{
  FDarray[b].FCD[0].CurrentSetpoint = v;
  FDarray[b].FCD[1].CurrentSetpoint = v;
}

// Average of FEM_ADC_SAMPLES reads of a MIPS ADC channel, in counts. Returns false, and the caller holds its last
// value, if the ADC is in use by the digitizer.
static bool FEMreadADC(int adc, float *counts)
{
  long sum = 0;

  if(!AcquireADC()) return false;
  for(int j = 0; j < FEM_ADC_SAMPLES; j++) sum += analogRead(adc);
  ReleaseADC();
  *counts = (float)sum / FEM_ADC_SAMPLES;
  return true;
}

// Runs every FEM_THREAD_MS (10 Hz). Order for each module: the enable input, the ADC inputs, then regulation, which
// itself only steps every FEM_PERIOD_MS.
static void FEMloop(void)
{
  uint32_t now = millis();

  for(int b = 0; b < 2; b++)
  {
    FEMdata *s = FEMarray[b];
    int      status = FEMst_OFF;
    float    counts,v;

    if((s == NULL) || (!FilamentBoards[b])) continue;
    int              idx     = FEMlive(b);
    FilamentChannel *ch      = &FDarray[b].FCD[idx];
    bool             cycling = FDarray[b].FCyl[idx].FCenable;
    // 1. Enable input. Authoritative, as in ESI: high enables the filament, low disables it.
    if(s->EnableIn != 0) ch->FilamentPwr = (ReadDIO(s->EnableIn) == 1);
    // 2. ADC input that sets the emission setpoint
    if((s->EmADC.adc >= 0) && FEMreadADC(s->EmADC.adc, &counts))
    {
      v = counts * s->EmADC.m + s->EmADC.b;
      if(v < 0) v = 0;
      if(v > s->Emax) v = s->Emax;
      s->EmADC.value = v;
      s->P.Iset      = v;
    }
    // 3. ADC input that sets the heater current. Stands aside while filament cycling is enabled. Two rules keep the
    //    Max power limiter working, because rewriting CurrentSetpoint every pass would otherwise undo its trimming
    //    (FilamentControl trims 0.1 A when the measured power is over the limit):
    //      - the command is never more than FEM_AHEAD above the ramped current, so it follows the ramp and the
    //        limiter sees the real power. Without this the command jumps to the ADC value while the ramped
    //        current is still low, the measured power lags, and the limiter and the ADC fight in a limit cycle
    //        (host simulation: 8.4 W peak on a 6 W limit, against 6.2 W with this rule),
    //      - it never raises the current while the power is at 97% of Max power.
    if(s->HtADC.adc >= 0)
    {
      s->On = false;                                                      // Regulation is disabled
      if((!cycling) && FEMreadADC(s->HtADC.adc, &counts))
      {
        v = counts * s->HtADC.m + s->HtADC.b;
        if(v < 0) v = 0;
        if(v > FEM_MAXCUR) v = FEM_MAXCUR;
        s->HtADC.value = v;
        float cur = ch->CurrentSetpoint;
        float act = CurrentSetpoints[b][idx];
        if(v > act + FEM_AHEAD) v = act + FEM_AHEAD;
        if((v > cur) && (ch->MaxPower > 0) && (Fpowers[b][idx] >= FEM_POWERHOLD * ch->MaxPower)) v = cur;
        if(v != cur) FEMsetCurrent(b, v);
      }
    }
    // 4. Regulation
    if(s->HtADC.adc >= 0) status = cycling ? FEMst_BLOCKED : FEMst_HEATADC;
    else if(!s->On) status = FEMst_OFF;
    else if(cycling) status = FEMst_BLOCKED;                              // Cycling owns the current
    else if(!ch->FilamentPwr) status = FEMst_WAIT;                        // Filament off, leave the setpoint alone
    else if((s->LastMs != 0) && ((now - s->LastMs) < FEM_PERIOD_MS)) status = s->Status;   // Not time for a step yet
    else
    {
      float dt = (s->LastMs == 0) ? (FEM_PERIOD_MS / 1000.0f) : ((now - s->LastMs) / 1000.0f);
      if(dt > FEM_MAX_DT) dt = FEM_MAX_DT;
      s->LastMs       = now;
      s->Measured     = EmissionI[b];
      s->P.PowerLimit = ch->MaxPower;
      float cur = ch->CurrentSetpoint;
      float nxt = FEMcoreStep(&s->P, &s->Core, cur, s->Measured, Fpowers[b][idx], dt, &status);
      if(nxt != cur) FEMsetCurrent(b, nxt);
    }
    if(status == FEMst_OFF || status == FEMst_WAIT || status == FEMst_BLOCKED || status == FEMst_HEATADC)
    {
      // Not regulating: start the next time from a clean rate, and from a fresh time base
      FEMcoreReset(&s->Core);
      s->LastMs = 0;
    }
    s->Status = status;
  }
}

// ---------------------------------------------------------------------------------------------
// Host commands. Channel numbers are 1 based, as in the other filament commands.
// ---------------------------------------------------------------------------------------------

// Validates the channel and returns the module's settings, or NULL after sending the NAK
static FEMdata *FEMget(int channel, int *b)
{
  if(!IsFilamentChannelValid(channel, true)) return NULL;
  *b = BoardFromSelectedFilamentChannel(channel - 1);
  if((*b < 0) || (FEMarray[*b] == NULL))
  {
    // Rev 1 and 2 modules have no on-module emission measurement
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return NULL;
  }
  return FEMarray[*b];
}

// Parses a float argument within [lo, hi]. Returns false if it is outside.
static bool FEMfloat(char *text, float lo, float hi, float *v)
{
  String t = text;
  float  f = t.toFloat();

  if((f < lo) || (f > hi)) return false;
  *v = f;
  return true;
}

static void FEMgetFloat(int channel, int which)
{
  int      b;
  FEMdata *s = FEMget(channel, &b);
  float    v;

  if(s == NULL) return;
  switch(which)
  {
    case 0: v = s->P.Iset;     break;
    case 1: v = s->Emax;       break;
    case 2: v = s->P.Imin;     break;
    case 3: v = s->P.Ki;       break;
    case 4: v = s->P.SlewUp;   break;
    case 5: v = s->P.SlewDn;   break;
    default: v = s->P.RateHold; break;
  }
  SendACKonly;
  if(SerialMute) return;
  serial->println(v, 4);
}

void FEMgetIset(int channel)    { FEMgetFloat(channel, 0); }
void FEMgetEmax(int channel)    { FEMgetFloat(channel, 1); }
void FEMgetImin(int channel)    { FEMgetFloat(channel, 2); }
void FEMgetKi(int channel)      { FEMgetFloat(channel, 3); }
void FEMgetSlewUp(int channel)  { FEMgetFloat(channel, 4); }
void FEMgetSlewDn(int channel)  { FEMgetFloat(channel, 5); }
void FEMgetRateHold(int channel){ FEMgetFloat(channel, 6); }

static void FEMsetFloat(char *chan, char *value, int which)
{
  int      b;
  float    v,lo,hi;
  float    *dst;
  String   t = chan;
  FEMdata *s = FEMget(t.toInt(), &b);

  if(s == NULL) return;
  // The variable being set and the range it accepts
  switch(which)
  {
    case 0:  dst = &s->P.Iset;     lo = 0;       hi = s->Emax;   break;
    case 1:  dst = &s->Emax;       lo = 1;       hi = 10000;     break;
    case 2:  dst = &s->P.Imin;     lo = 0;       hi = FEM_MAXCUR; break;
    case 3:  dst = &s->P.Ki;       lo = 0.0005f; hi = 0.1f;      break;
    case 4:  dst = &s->P.SlewUp;   lo = 0.001f;  hi = 1;         break;
    case 5:  dst = &s->P.SlewDn;   lo = 0.001f;  hi = 1;         break;
    default: dst = &s->P.RateHold; lo = 0.005f;  hi = 1;         break;
  }
  if(!FEMfloat(value, lo, hi, &v)) BADARG;
  *dst = v;
  // Lowering the largest accepted setpoint also lowers a setpoint that is above it
  if((which == 1) && (s->P.Iset > s->Emax)) s->P.Iset = s->Emax;
  SendACK;
}

void FEMsetIset(char *chan, char *value)     { FEMsetFloat(chan, value, 0); }
void FEMsetEmax(char *chan, char *value)     { FEMsetFloat(chan, value, 1); }
void FEMsetImin(char *chan, char *value)     { FEMsetFloat(chan, value, 2); }
void FEMsetKi(char *chan, char *value)       { FEMsetFloat(chan, value, 3); }
void FEMsetSlewUp(char *chan, char *value)   { FEMsetFloat(chan, value, 4); }
void FEMsetSlewDn(char *chan, char *value)   { FEMsetFloat(chan, value, 5); }
void FEMsetRateHold(char *chan, char *value) { FEMsetFloat(chan, value, 6); }

void FEMsetOn(char *chan, char *state)
{
  int      b;
  String   t = chan;
  String   st = state;
  FEMdata *s = FEMget(t.toInt(), &b);

  if(s == NULL) return;
  if((st != "ON") && (st != "OFF")) BADARG;
  if(st == "ON")
  {
    // Cycling and regulation both drive the current setpoint, only one at a time
    if(FDarray[b].FCyl[FEMlive(b)].FCenable) BADARG;
    if(s->HtADC.adc >= 0) BADARG;               // A heater current ADC input owns the current
    FEMcoreReset(&s->Core);
    s->LastMs = 0;
    s->On = true;
  }
  else
  {
    s->On = false;
    s->Status = FEMst_OFF;
  }
  SendACK;
}

void FEMgetOn(int channel)
{
  int      b;
  FEMdata *s = FEMget(channel, &b);

  if(s == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(s->On ? "ON" : "OFF");
}

// Reports the state, the emission measurement used, and the filament current setpoint, comma separated
void FEMgetStatus(int channel)
{
  int      b;
  FEMdata *s = FEMget(channel, &b);

  if(s == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  serial->print(FEMstatusNames[s->Status]);
  serial->print(",");
  serial->print(EmissionI[b], 2);
  serial->print(",");
  serial->println(FDarray[b].FCD[FEMlive(b)].CurrentSetpoint, 3);
}

// Sets or removes an ADC input: SFLEADC / SFLIADC,channel,adc,slope,intercept. A line command like ESI's
// SESICTRL: the arguments are read from the input line. adc is the MIPS ADC channel 0 to 9, or -1 for none.
static void FEMsetADC(bool heater)
{
  int      chan,adc,b;
  float    slope,intercept;
  FEMadc   *a;

  while(true)
  {
    if(!valueFromCommandLine(&chan, 1, NumberOfFilamentChannels)) break;
    if(!valueFromCommandLine(&adc, -1, 9)) break;
    if(!valueFromCommandLine(&slope, -100000, 100000)) break;
    if(!valueFromCommandLine(&intercept, -100000, 100000)) break;
    if(!IsFilamentChannelValid(chan, false)) break;
    b = BoardFromSelectedFilamentChannel(chan - 1);
    if((b < 0) || (FEMarray[b] == NULL)) break;
    a = heater ? &FEMarray[b]->HtADC : &FEMarray[b]->EmADC;
    a->adc   = adc;
    a->m     = slope;
    a->b     = intercept;
    a->value = 0;
    if(heater && (adc >= 0))
    {
      // The heater current ADC owns the filament current: regulation is disabled
      FEMarray[b]->On = false;
      FEMcoreReset(&FEMarray[b]->Core);
      FEMarray[b]->LastMs = 0;
    }
    SendACK;
    return;
  }
  BADARG;
}
void FEMsetEmADC(void) { FEMsetADC(false); }
void FEMsetHtADC(void) { FEMsetADC(true); }

// Reports adc,slope,intercept,last value
static void FEMgetADC(int channel, bool heater)
{
  int      b;
  FEMdata *s = FEMget(channel, &b);
  FEMadc  *a;

  if(s == NULL) return;
  a = heater ? &s->HtADC : &s->EmADC;
  SendACKonly;
  if(SerialMute) return;
  serial->print(a->adc);
  serial->print(",");
  serial->print(a->m, 6);
  serial->print(",");
  serial->print(a->b, 6);
  serial->print(",");
  serial->println(a->value, 3);
}
void FEMgetEmADC(int channel) { FEMgetADC(channel, false); }
void FEMgetHtADC(int channel) { FEMgetADC(channel, true); }

// The digital input, Q to X, that enables the filament, or NA for none
void FEMsetEnableIn(char *chan, char *value)
{
  int      b;
  String   t = chan;
  String   v = value;
  FEMdata *s = FEMget(t.toInt(), &b);

  if(s == NULL) return;
  if(v == "NA") s->EnableIn = 0;
  else if((value[0] >= 'Q') && (value[0] <= 'X') && (value[1] == 0)) s->EnableIn = value[0];
  else BADARG;
  SendACK;
}

void FEMgetEnableIn(int channel)
{
  int      b;
  FEMdata *s = FEMget(channel, &b);

  if(s == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  if(s->EnableIn == 0) serial->println("NA");
  else serial->println(s->EnableIn);
}

// Command table. const, so it is in flash: every entry is a function. Registered by FilamentEmission_init.
const Commands FEMCmdArray[] = {
// Start of command block
// Filament emission current regulation, Rev 3 and 4 modules. Channel is the filament channel, 1 based.
// Regulation raises or lowers the filament current to hold the emission current. The filament limit is the
// module Max power setting. See FilamentEmission.cpp.
  {"SFLEIS",   CMDfunctionStr, 2, (char *)FEMsetIset},      // Set the emission current setpoint, channel,uA. 0 to the emission max
  {"GFLEIS",   CMDfunction,    1, (char *)FEMgetIset},      // Get the emission current setpoint, uA
  {"SFLEREG",  CMDfunctionStr, 2, (char *)FEMsetOn},        // Enable emission regulation, channel,ON|OFF. Refused while filament cycling is on or a heater current ADC input is defined
  {"GFLEREG",  CMDfunction,    1, (char *)FEMgetOn},        // Get the emission regulation state, ON or OFF
  {"GFLEST",   CMDfunction,    1, (char *)FEMgetStatus},    // Get status,measured uA,filament current A. Status is OFF, WAIT, BLOCKED, SEEK, REG, PLIM, IMAX or HEATADC
  {"SFLEIM",   CMDfunctionStr, 2, (char *)FEMsetEmax},      // Set the largest emission setpoint accepted, channel,uA. 1 to 10000, default 1000
  {"GFLEIM",   CMDfunction,    1, (char *)FEMgetEmax},      // Get the largest emission setpoint accepted, uA
  {"SFLEIMN",  CMDfunctionStr, 2, (char *)FEMsetImin},      // Set the lowest filament current regulation will command, channel,A. Default 0
  {"GFLEIMN",  CMDfunction,    1, (char *)FEMgetImin},      // Get the lowest filament current regulation will command, A
  {"SFLEGAIN", CMDfunctionStr, 2, (char *)FEMsetKi},        // Set the regulation gain, channel,A/s per unit of log error. 0.0005 to 0.1, default 0.005
  {"GFLEGAIN", CMDfunction,    1, (char *)FEMgetKi},        // Get the regulation gain
  {"SFLEUP",   CMDfunctionStr, 2, (char *)FEMsetSlewUp},    // Set the largest filament current increase rate, channel,A/s. 0.001 to 1, default 0.02
  {"GFLEUP",   CMDfunction,    1, (char *)FEMgetSlewUp},    // Get the largest filament current increase rate
  {"SFLEDN",   CMDfunctionStr, 2, (char *)FEMsetSlewDn},    // Set the largest filament current decrease rate, channel,A/s. 0.001 to 1, default 0.1
  {"GFLEDN",   CMDfunction,    1, (char *)FEMgetSlewDn},    // Get the largest filament current decrease rate
  {"SFLERH",   CMDfunctionStr, 2, (char *)FEMsetRateHold},  // Set the emission rate hold, channel,1/s of ln(emission). 0.005 to 1, default 0.05
  {"GFLERH",   CMDfunction,    1, (char *)FEMgetRateHold},  // Get the emission rate hold
  {"SFLEADC",  CMDfunctionLine, 0, (char *)FEMsetEmADC},    // Set the ADC input that sets the emission setpoint, channel,adc,slope,intercept. adc 0 to 9, -1 for none. uA = counts * slope + intercept
  {"GFLEADC",  CMDfunction,    1, (char *)FEMgetEmADC},     // Get adc,slope,intercept,last value for the emission setpoint ADC input
  {"SFLIADC",  CMDfunctionLine, 0, (char *)FEMsetHtADC},    // Set the ADC input that sets the heater current, channel,adc,slope,intercept. adc 0 to 9, -1 for none. A = counts * slope + intercept. Disables emission regulation while defined
  {"GFLIADC",  CMDfunction,    1, (char *)FEMgetHtADC},     // Get adc,slope,intercept,last value for the heater current ADC input
  {"SFLDIO",   CMDfunctionStr, 2, (char *)FEMsetEnableIn},  // Set the digital input that enables the filament, channel,Q to X or NA. High enables, low disables
  {"GFLDIO",   CMDfunction,    1, (char *)FEMgetEnableIn},  // Get the digital input that enables the filament, Q to X or NA
// End of table marker
  {0},
};

CommandList FEMCmdList = { (Commands *)FEMCmdArray, NULL };

// Called at the end of Filament_init for each board. Rev is known by then (restored from the module EEPROM).
void FilamentEmission_init(int8_t Board)
{
  if((Board < 0) || (Board > 1)) return;
  if((FDarray[Board].Rev != 3) && (FDarray[Board].Rev != 4)) return;      // No on-module emission measurement
  if(FEMarray[Board] == NULL)
  {
    FEMarray[Board] = new FEMdata;
    if(FEMarray[Board] == NULL) return;
    FEMdata *s = FEMarray[Board];
    s->P.Iset       = 0;
    s->P.Imin       = 0;
    s->P.Imax       = FEM_MAXCUR;
    s->P.Ki         = FEM_DEF_KI;
    s->P.SlewUp     = FEM_DEF_SLEWUP;
    s->P.SlewDn     = FEM_DEF_SLEWDN;
    s->P.RateHold   = FEM_DEF_RATEHOLD;
    s->P.PowerLimit = 0;
    s->Emax         = FEM_DEF_EMAX;
    s->EmADC.adc    = -1;
    s->EmADC.m      = 1;
    s->EmADC.b      = 0;
    s->EmADC.value  = 0;
    s->HtADC.adc    = -1;
    s->HtADC.m      = 1;
    s->HtADC.b      = 0;
    s->HtADC.value  = 0;
    s->EnableIn     = 0;
    s->On           = false;
    s->Status       = FEMst_OFF;
    s->Measured     = 0;
    s->LastMs       = 0;
    FEMcoreReset(&s->Core);
  }
  // The commands and the thread are set up once, for the first board
  if(FEMthread == NULL)
  {
    AddToCommandList(&FEMCmdList);
    FEMthread = new Thread();
    FEMthread->setName("FilEmis");
    FEMthread->onRun(FEMloop);
    FEMthread->setInterval(FEM_THREAD_MS);
    control.add(FEMthread);
  }
}
