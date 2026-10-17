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
//     end of Filament_init. Its own 500 mS thread runs the loop. Nothing in the Filament loops was changed.
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
#include "FilamentEmission.h"
#include "FilamentEmissionCore.h"

// Things this file reads from Filament.cpp
extern bool            FilamentBoards[2];
extern float           Fpowers[2][2];
extern float           EmissionI[2];
extern int             Channel2Index(int channel);
extern int8_t          BoardFromSelectedFilamentChannel(int8_t SC);
extern ThreadController control;

#define FEM_MAXCUR       12.0f      // Largest filament current, A. Matches MaxFilCur in Filament.cpp
#define FEM_PERIOD_MS    500        // Control period
#define FEM_MAX_DT       2.0f       // A longer gap than this (s) is clamped, the thread was held up

typedef struct
{
  FEMparams    P;                   // Setpoint and tuning handed to the core
  float        Emax;                // Largest emission setpoint accepted, uA
  bool         On;                  // Regulation enabled
  int          Status;              // FEMstatusCodes
  float        Measured;            // Last emission measurement used, uA
  FEMcoreState Core;
  uint32_t     LastMs;              // millis() of the last control step, 0 if none
} FEMdata;

static FEMdata *FEMarray[2] = {NULL,NULL};   // Allocated for each Rev 3 or 4 board at init
static Thread  *FEMthread   = NULL;

static const char *FEMstatusNames[] = {"OFF","WAIT","BLOCKED","SEEK","REG","PLIM","IMAX"};

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

static void FEMloop(void)
{
  for(int b = 0; b < 2; b++)
  {
    FEMdata *s = FEMarray[b];
    int      status = FEMst_OFF;

    if((s == NULL) || (!FilamentBoards[b])) continue;
    int idx = FEMlive(b);
    if(!s->On) status = FEMst_OFF;
    else if(FDarray[b].FCyl[idx].FCenable) status = FEMst_BLOCKED;       // Cycling owns the current
    else if(!FDarray[b].FCD[idx].FilamentPwr) status = FEMst_WAIT;       // Filament off, leave the setpoint alone
    else
    {
      uint32_t now = millis();
      float    dt  = (s->LastMs == 0) ? (FEM_PERIOD_MS / 1000.0f) : ((now - s->LastMs) / 1000.0f);
      if(dt > FEM_MAX_DT) dt = FEM_MAX_DT;
      s->LastMs   = now;
      s->Measured = EmissionI[b];
      s->P.PowerLimit = FDarray[b].FCD[idx].MaxPower;
      float cur = FDarray[b].FCD[idx].CurrentSetpoint;
      float nxt = FEMcoreStep(&s->P, &s->Core, cur, s->Measured, Fpowers[b][idx], dt, &status);
      if(nxt != cur) FEMsetCurrent(b, nxt);
    }
    if(status == FEMst_OFF || status == FEMst_WAIT || status == FEMst_BLOCKED)
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

// Command table. const, so it is in flash: every entry is a function. Registered by FilamentEmission_init.
const Commands FEMCmdArray[] = {
// Start of command block
// Filament emission current regulation, Rev 3 and 4 modules. Channel is the filament channel, 1 based.
// Regulation raises or lowers the filament current to hold the emission current. The filament limit is the
// module Max power setting. See FilamentEmission.cpp.
  {"SFLEIS",   CMDfunctionStr, 2, (char *)FEMsetIset},      // Set the emission current setpoint, channel,uA. 0 to the emission max
  {"GFLEIS",   CMDfunction,    1, (char *)FEMgetIset},      // Get the emission current setpoint, uA
  {"SFLEREG",  CMDfunctionStr, 2, (char *)FEMsetOn},        // Enable emission regulation, channel,ON|OFF. Refused while filament cycling is on
  {"GFLEREG",  CMDfunction,    1, (char *)FEMgetOn},        // Get the emission regulation state, ON or OFF
  {"GFLEST",   CMDfunction,    1, (char *)FEMgetStatus},    // Get status,measured uA,filament current A. Status is OFF, WAIT, BLOCKED, SEEK, REG, PLIM or IMAX
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
    FEMthread->setInterval(FEM_PERIOD_MS);
    control.add(FEMthread);
  }
}
