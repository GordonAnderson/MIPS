#ifndef FILAMENTEMISSIONCORE_H_
#define FILAMENTEMISSIONCORE_H_
//
// FilamentEmissionCore
//
// The emission current regulation law, with no Arduino or MIPS dependencies so it can be compiled and tested
// on the host against a plant model (tools/FilamentEmissionSim.cpp). The firmware wrapper that reads the
// measurements, owns the settings and writes the filament current setpoint is FilamentEmission.cpp.
//
// The controller is incremental: it is given the present commanded filament current and returns the next
// one. It keeps no integrator of its own, the commanded current is the state. A change made by anything
// else (a user edit, the power limiter, cycling) is therefore simply a new starting point, and there is
// nothing to wind up.
//
// Law, once per control period dt:
//    err  = ln(Iset / max(measured, Floor))          log error, so the loop gain is about the same at any
//                                                    emission level (emission rises exponentially with
//                                                    filament current)
//    step = Ki * err * dt                            amps
//    step is limited to +SlewUp*dt / -SlewDn*dt
//    rate hold:  no further step in a direction while the measured emission is already moving that way faster
//                than RateHold (change of ln(measured) per second). The measurement filter and the thermal lag
//                of the filament make the emission follow the current late; without this the current keeps
//                climbing after the target is reached and the emission overshoots.
//    power hold: no increase while the filament power is at or above PowerHoldFrac of the power limit. The
//                filament limit is the module's Max power setting, which FilamentControl enforces by trimming
//                the current; holding just below it keeps the two from fighting.
//    result is limited to [Imin, Imax]
//
// Simulated on the host (October 2026, tools/FilamentEmissionSim.cpp) with emission sensitivity of 5 to 40 per
// amp and filament thermal lag of 1 to 6 seconds: a cold start settles to +/-5% in 2 to 4 minutes, overshoot is
// 2.4% or less except for the steepest and slowest plants (10% at 20 per amp with a 6 s lag, 28% at 40 per amp
// with a 6 s lag), and a 200 to 400 uA setpoint step settles in 17 to 40 s. The plant values are guesses, tune the
// gain, slews and rate hold on the real filament.
//
#include <math.h>

#define FEM_FLOOR_UA      1.0f     // Measured emission is not allowed below this in the log, uA
#define FEM_REG_BAND      0.05f    // Within +/- 5% of the setpoint counts as regulating

// Defaults, from the simulation
#define FEM_DEF_KI        0.005f   // A/s per unit of log error
#define FEM_DEF_SLEWUP    0.02f    // A/s
#define FEM_DEF_SLEWDN    0.1f     // A/s, matches the default current ramp rate
#define FEM_DEF_RATEHOLD  0.05f    // 1/s of ln(emission)
#define FEM_DEF_EMAX      1000.0f  // uA, largest emission setpoint accepted
#define FEM_POWERHOLD     0.97f    // Fraction of the power limit at which increases stop
// Above FEM_POWERBACK of the limit the current is eased down at FEM_BACKOFF so the power limiter's 0.1 A trim
// (FilamentControl, every 1.1 s) does not act. The power reading lags the current by about 1 s (the supply
// tracking), so with only the hold the trim and the next raise made a sawtooth: bench, November 4, 2026, 9 W
// limit, heater ADC at 2.5 A, the command swung 2.14 to 2.44 A and the power 8.4 to 9.8 W.
#define FEM_POWERBACK     0.99f    // Fraction of the power limit above which the current is lowered
#define FEM_BACKOFF       0.05f    // A/s, rate the current is lowered at above FEM_POWERBACK
#define FEM_POWERSLOW     0.90f    // Heater ADC: above this fraction of the limit increases are slowed
#define FEM_SLOWUP        0.02f    // A/s, heater ADC increase rate above FEM_POWERSLOW

enum FEMstatusCodes
{
  FEMst_OFF = 0,       // Regulation not enabled
  FEMst_WAIT,          // Enabled, but the filament is off or the setpoint is 0, nothing is changed
  FEMst_BLOCKED,       // Filament current cycling is enabled, regulation stands aside
  FEMst_SEEK,          // Emission is outside the band, current is being moved
  FEMst_REG,           // Emission is within the band of the setpoint
  FEMst_PLIM,          // Wanted to increase but the filament power limit holds it
  FEMst_IMAX,          // Wanted to increase but the current is at its maximum
  FEMst_HEATADC        // A heater current ADC input owns the filament current, regulation is disabled
};

typedef struct
{
  float Iset;          // Emission setpoint, uA
  float Imin;          // Lowest filament current to command, A
  float Imax;          // Highest filament current to command, A
  float Ki;
  float SlewUp;
  float SlewDn;
  float RateHold;
  float PowerLimit;    // Filament power limit, W. 0 disables the power hold
} FEMparams;

typedef struct
{
  float prevLn;        // ln of the previous measurement, for the rate
  bool  havePrev;
} FEMcoreState;

// Call when regulation (re)starts or the filament was off, so the first rate is not computed against stale data
static inline void FEMcoreReset(FEMcoreState *s)
{
  s->havePrev = false;
  s->prevLn   = 0;
}

// One control step. cur = commanded filament current now (A), meas = measured emission (uA), power = measured
// filament power (W), dt = time since the last step (s). Returns the new commanded current and the status.
static inline float FEMcoreStep(const FEMparams *p, FEMcoreState *s, float cur, float meas, float power,
                                float dt, int *status)
{
  float y,ln,rate,err,step,lim;

  *status = FEMst_SEEK;
  if(dt <= 0) return cur;
  if(p->Iset <= 0) { *status = FEMst_WAIT; return cur; }
  if(!(meas == meas)) { *status = FEMst_WAIT; return cur; }          // NaN
  y    = (meas > FEM_FLOOR_UA) ? meas : FEM_FLOOR_UA;
  ln   = logf(y);
  rate = s->havePrev ? (ln - s->prevLn) / dt : 0.0f;
  s->prevLn   = ln;
  s->havePrev = true;
  err  = logf(p->Iset / y);
  step = p->Ki * err * dt;
  lim  = ((step > 0) ? p->SlewUp : p->SlewDn) * dt;
  if(step >  lim) step =  lim;
  if(step < -lim) step = -lim;
  if((step > 0) && (rate >  p->RateHold)) step = 0;
  if((step < 0) && (rate < -p->RateHold)) step = 0;
  if(fabsf(meas - p->Iset) <= FEM_REG_BAND * p->Iset) *status = FEMst_REG;
  if((step > 0) && (p->PowerLimit > 0) && (power >= FEM_POWERHOLD * p->PowerLimit))
  {
    step = 0;
    *status = FEMst_PLIM;
  }
  if((p->PowerLimit > 0) && (power > FEM_POWERBACK * p->PowerLimit))
  {
    step = -FEM_BACKOFF * dt;          // Ease off before the power limiter trims 0.1 A
    *status = FEMst_PLIM;
  }
  cur += step;
  if(cur < p->Imin) cur = p->Imin;
  if(cur >= p->Imax)
  {
    cur = p->Imax;
    if(err > 0) *status = FEMst_IMAX;
  }
  return cur;
}

#endif
