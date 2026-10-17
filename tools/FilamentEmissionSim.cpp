//
// Host test for the filament emission regulation core (include/FilamentEmissionCore.h).
//
//   c++ -std=c++11 -O1 -Iinclude tools/FilamentEmissionSim.cpp -o /tmp/femsim && /tmp/femsim
//
// Plant model (a guess, not a measured filament):
//   filament power        P = Rfil * I^2
//   hot current           first order lag of the ramped current, time constant tau
//   emission              Ie = 200 uA * exp(k * (Ihot - 2.0 A)), clipped at 5000 uA, zero until bias is on
//   measurement           0.05 filter every 100 ms (as Filament_loop) of Ie with 1% noise
//   ramp limiter          0.1 A/s on the current actually applied (CurrentSetpoints)
//   power limiter         when P > limit, the commanded current is trimmed by 0.1 A every 1.1 s (FilamentControl)
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <initializer_list>
#include "FilamentEmissionCore.h"

struct Run { double peak, settle, lastI, rip; int status; double maxP; double above1000; };

static double gauss(){ double u=(rand()+1.0)/(RAND_MAX+2.0), v=(rand()+1.0)/(RAND_MAX+2.0); return sqrt(-2*log(u))*cos(6.283185307*v); }

static Run sim(double k, double tau, double Rfil, double plimit, double bias_on, double iset, double imax,
               double tmax, double step_t = -1, double iset2 = 0)
{
  FEMparams p = {(float)iset, 0.0f, (float)imax, FEM_DEF_KI, FEM_DEF_SLEWUP, FEM_DEF_SLEWDN, FEM_DEF_RATEHOLD, (float)plimit};
  FEMcoreState cs; FEMcoreReset(&cs);
  double dt = 0.1, cmd = 0.5, cur = 0.5, hot = 0.5, meas = 0, nextc = 0, nextlim = 1.1, iset_now = iset;
  Run r = {0,0,0,0,0,0,0};
  double lastbad = 0, tail_min = 1e9, tail_max = 0, target = iset;
  for(int i = 0; i < (int)(tmax/dt); i++)
  {
    double t = i*dt;
    if(step_t >= 0 && t >= step_t) { iset_now = iset2; p.Iset = (float)iset2; target = iset2; }
    double d = cmd - cur, m = 0.1*dt; cur += (d > m) ? m : (d < -m) ? -m : d;
    hot += (cur - hot)*dt/tau;
    double P = Rfil*cur*cur;
    double e = (t < bias_on) ? 0.0 : 200.0*exp(k*(hot - 2.0)); if(e > 5000) e = 5000;
    meas += 0.05*(e*(1 + 0.01*gauss()) - meas);
    if(P > r.maxP) r.maxP = P;
    if(t >= nextlim) { nextlim += 1.1; if(plimit > 0 && P > plimit) cmd -= 0.1; }   // FilamentControl power limiter
    if(t >= nextc)
    {
      nextc += 0.5;
      cmd = FEMcoreStep(&p, &cs, (float)cmd, (float)meas, (float)P, 0.5f, &r.status);
    }
    if(t >= bias_on) { if(e > r.peak) r.peak = e; if(e > 1000) r.above1000 += dt; }
    if(t >= bias_on && fabs(e - target)/target > 0.05) lastbad = t - bias_on;
    if(t > tmax - 100) { if(e < tail_min) tail_min = e; if(e > tail_max) tail_max = e; }
  }
  r.settle = lastbad; r.lastI = cmd; r.rip = (tail_max - tail_min)/target*100;
  return r;
}

int main()
{
  srand(3);
  printf("1) Cold start to 200 uA, bias present, no power limit in the way (Imax 12 A)\n");
  printf("      k(/A) tau(s) | settle(s) overshoot%% ripple%%\n");
  double ws = 0, wo = -99, wr = 0;
  for(double k : {5.0, 10.0, 20.0, 40.0}) for(double tau : {1.0, 3.0, 6.0})
  {
    Run r = sim(k, tau, 1.2, 1000, 0, 200, 12, 900);
    double over = (r.peak - 200)/200*100;
    printf("      %5.0f %5.0f  | %7.0f  %8.1f  %6.2f\n", k, tau, r.settle, over, r.rip);
    if(r.settle > ws) ws = r.settle; if(over > wo) wo = over; if(r.rip > wr) wr = r.rip;
  }
  printf("   worst case: settle %.0f s  overshoot %.1f %%  ripple %.2f %%\n", ws, wo, wr);

  printf("2) Setpoint step 200 -> 400 uA at t=300 s (time measured from the step)\n");
  for(double k : {10.0, 20.0})
  {
    Run r = sim(k, 3, 1.2, 1000, 0, 200, 12, 800, 300, 400);
    printf("   k=%.0f: within +/-5%% of 400 uA %.0f s after the step, peak %.0f uA, final current %.3f A\n", k, r.settle - 300, r.peak, r.lastI);
  }

  printf("3) Power limit as the ceiling (Rfil 1.2 ohm, k=10, tau=3)\n");
  { Run r = sim(10, 3, 1.2, 6.0, 0, 200, 12, 900);
    printf("   200 uA needs 4.8 W, limit 6.0 W: settles %.0f s, ripple %.2f %%, max power %.2f W\n", r.settle, r.rip, r.maxP); }
  { Run r = sim(10, 3, 1.2, 6.0, 0, 3000, 12, 1200);
    printf("   3000 uA wanted, which needs more than the limit allows: current held at %.3f A, power %.2f W (limit 6.00),\n"
           "   emission %.0f uA, status %d (5 = PLIM)\n", r.lastI, r.maxP, 200.0*exp(10*(r.lastI - 2.0)), r.status); }

  printf("4) Option A, late bias: bias OFF until t=400 s, the current climbs to the power-limit ceiling meanwhile\n");
  printf("   (emission clipped at 5000 uA in this model)\n");
  printf("   limit   k  tau | ceiling A  peak uA  s above 1 mA  settles after bias (s)\n");
  for(double lim : {6.0, 5.0, 4.4})
    for(double k : {10.0, 20.0}) for(double tau : {1.0, 6.0})
    {
      Run r = sim(k, tau, 1.2, lim, 400, 200, 12, 1300);
      printf("   %4.1f W %3.0f %3.0f | %7.3f  %8.0f  %10.0f  %12.0f\n", lim, k, tau, sqrt(lim/1.2), r.peak, r.above1000, r.settle);
    }
  return 0;
}
