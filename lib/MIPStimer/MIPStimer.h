/*
  MIPStimer.h - Timer class for the Arduino DUE designed to support the
  MIPS system timer functions used to enable the time table functions.

  This class is bassed on the DueTimer referenced below:
  https://github.com/ivanseidel/DueTimer

  Created by Ivan Seidel Gomes, March, 2013.
  Modified by Philipp Klaus, June 2013.
  Released into the public domain.

  MIPStimer developed by Gordon Anderson
*/

#ifdef __arm__

#ifndef MIPStimer_h
#define MIPStimer_h

#include "Arduino.h"

#include <inttypes.h>

class MIPStimer
{
protected:

	// Represents the timer id (index for the array of Timer structs)
	int timer;

	// Stores the object timer clock frequency
	static double ClockFrequency[9];
	// Stores the object timer frequency
	// (allows to access current timer period and frequency):
	static double _frequency[9];
	// Timer channel used flag
	static bool Used[9];
	// Picks the best clock to lower the error
	static uint8_t bestClock(double frequency, uint32_t& retRC, uint8_t ClockDivisor);
	

public:
	struct Timer
	{
		Tc *tc;
		uint32_t channel;
		IRQn_Type irq;
	};

	static uint32_t SR[9];
	static MIPStimer getAvailable();

	// Store timer configuration (static, as it's fix for every object)
	static const Timer Timers[9];

	// Needs to be public, because the handlers are outside class:
	static void (*callbacks[9])();
	static void (*callbacksRA[9])();
	static void (*callbacksRB[9])();
	static int TIOApins[9];
	static int TIOBpins[9];

	MIPStimer (int _timer);
	MIPStimer setPriority(uint8_t pri);
	MIPStimer attachInterrupt(void (*isr)());
	MIPStimer attachInterruptRA(void (*isr)());
	MIPStimer attachInterruptRB(void (*isr)());
	MIPStimer detachInterrupt();
	MIPStimer begin();
	MIPStimer stop();
	MIPStimer stopOnRC();
	MIPStimer nostopOnRC();
	MIPStimer enableTrigger();
	MIPStimer softwareTrigger();
	MIPStimer setClock(uint32_t clock);
	MIPStimer setTrigger(uint32_t trigger);
    MIPStimer setTriggerQ(uint32_t trigger);
    MIPStimer setTIOAeffect(uint32_t count, uint32_t effect);
    MIPStimer setTIOAeffectNOIO(uint32_t count, uint32_t effect);
    MIPStimer setRA(uint32_t count);
    MIPStimer incRA(uint32_t count);
    MIPStimer setTIOBeffect(uint32_t count, uint32_t effect);
    MIPStimer setRB(uint32_t count);
    MIPStimer incRB(uint32_t count);
    MIPStimer setRC(uint32_t count);
    MIPStimer halt(bool state);

	MIPStimer start(long microseconds = -1, uint8_t ClockDivisor = 0, bool NoInterrupts = false);
	MIPStimer setFrequency(double frequency, uint8_t ClockDivisor = 0);
	MIPStimer setPeriod(long microseconds, uint8_t ClockDivisor = 0);

    bool     InUse();
	bool     checkStatusBit(uint32_t bitMask);
    uint32_t getStatus();
	uint32_t getCounter();
	uint32_t getRAcounter();
	double   getFrequency();
	long     getPeriod();
    double   getClockFrequency();
};

extern void TimerTrapISR();
// Just to call Timer.getAvailable instead of Timer::getAvailable() :
extern MIPStimer Timer;

extern MIPStimer Timer0;
extern MIPStimer Timer1;
extern MIPStimer Timer2;
extern MIPStimer Timer3;
extern MIPStimer Timer4;
extern MIPStimer Timer5;
extern MIPStimer Timer6;
extern MIPStimer Timer7;
extern MIPStimer Timer8;
#endif

#else
	#error Oops! Trying to include MIPStimer on another device?
#endif
