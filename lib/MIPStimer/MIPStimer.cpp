/*
  MIPStimer.cpp - Implementation of Timers defined in MIPStimer.h
  
  Based on DueTimer:
  https://github.com/ivanseidel/DueTimer

  Created by Ivan Seidel Gomes, March, 2013.
  Modified by Philipp Klaus, June 2013.
  Thanks to stimmer (from Arduino forum), for coding the "timer soul" (Register stuff)
  Released into the public domain.
  
  MIPStimer developed by Gordon Anderson
*/

#include "MIPStimer.h"

const MIPStimer::Timer MIPStimer::Timers[9] = {
	{TC0,0,TC0_IRQn},
	{TC0,1,TC1_IRQn},
	{TC0,2,TC2_IRQn},
	{TC1,0,TC3_IRQn},
	{TC1,1,TC4_IRQn},
	{TC1,2,TC5_IRQn},
	{TC2,0,TC6_IRQn},
	{TC2,1,TC7_IRQn},
	{TC2,2,TC8_IRQn},
};

void (*MIPStimer::callbacks[9])() = {};
void (*MIPStimer::callbacksRA[9])() = {};
void (*MIPStimer::callbacksRB[9])() = {};
bool MIPStimer::Used[9]={false,false,false,false,false,false,false,false,false};
double MIPStimer::ClockFrequency[9]={0,0,0,0,0,0,0,0,0};
double MIPStimer::_frequency[9] = {-1,-1,-1,-1,-1,-1,-1,-1,-1};
// Pin numbers for output pins for each timer, -1 indicates not avaliable for use
int MIPStimer::TIOApins[9] = {2,-1,-1,-1,-1,-1,5,3,11};
int MIPStimer::TIOBpins[9] = {13,-1,-1,-1,-1,-1,4,10,12};
// Used by get status function
uint32_t MIPStimer::SR[9] = {0,0,0,0,0,0,0,0,0};

/*
	Initializing all timers, so you can use them like this: Timer0.start();
*/
MIPStimer Timer(0);

MIPStimer Timer0(0);
MIPStimer Timer1(1);
MIPStimer Timer2(2);
MIPStimer Timer3(3);
MIPStimer Timer4(4);
MIPStimer Timer5(5);
MIPStimer Timer6(6);
MIPStimer Timer7(7);
MIPStimer Timer8(8);

MIPStimer::MIPStimer(int _timer)
{
	// The constructor of the class MIPStimer 
	timer = _timer;
	Used[timer] = true;
}

bool MIPStimer::InUse()
{
   return(Used[timer]);
}

MIPStimer MIPStimer::getAvailable()
{
	// Return the first timer with no callback set
	for(int i = 0; i < 9; i++){
		if(!callbacks[i])
			return MIPStimer(i);
	}
	// Default, return Timer0;
	return MIPStimer(0);
}

MIPStimer MIPStimer::setPriority(uint8_t pri)
{
   NVIC_SetPriority(Timers[timer].irq, pri);
   return *this;
}

MIPStimer MIPStimer::attachInterrupt(void (*isr)())
{
	// Links the function passed as argument to the timer of the object

	callbacks[timer] = isr;
	return *this;
}

MIPStimer MIPStimer::attachInterruptRA(void (*isr)())
{
	// Links the function passed as argument to the timer of the object

	callbacksRA[timer] = isr;
	return *this;
}

MIPStimer MIPStimer::attachInterruptRB(void (*isr)())
{
	// Links the function passed as argument to the timer of the object

	callbacksRB[timer] = isr;
	return *this;
}

MIPStimer MIPStimer::detachInterrupt()
{
	// Links the function passed as argument to the timer of the object

	stop(); // Stop the currently running timer

	callbacks[timer] = TimerTrapISR;
	callbacksRA[timer] = TimerTrapISR;
	callbacksRB[timer] = TimerTrapISR;
	return *this;
}

// This function sets up the timer. The default settings are:
//  - clock = internal MCK/128
//  - software trigger
// The timer is not started by this function, it is setup only
MIPStimer MIPStimer::begin()
{
	// Get current timer configuration
	Timer t = Timers[timer];

 	// Tell the Power Management Controller to disable 
	// the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);

	// Enable clock for the timer
	pmc_enable_periph_clk((uint32_t)t.irq);

	// Set the default clock frequency
	ClockFrequency[timer] = (double)VARIANT_MCK / 128.0;
	// Set up the Timer in waveform mode which creates a PWM
	// in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.
	TC_Configure(t.tc, t.channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	// Enable the RC Compare Interrupt...
	t.tc->TC_CHANNEL[t.channel].TC_IER=TC_IER_CPCS;
	// ... and disable all others.
	t.tc->TC_CHANNEL[t.channel].TC_IDR=~TC_IER_CPCS;

	NVIC_ClearPendingIRQ(Timers[timer].irq);
	NVIC_EnableIRQ(Timers[timer].irq);

	return *this;
}

MIPStimer MIPStimer::stop()
{
	// Stop the timer

	NVIC_DisableIRQ(Timers[timer].irq);
	TC_Stop(Timers[timer].tc, Timers[timer].channel);
	return *this;
}

MIPStimer MIPStimer::stopOnRC()
{
	Timer t = Timers[timer];

    t.tc->TC_CHANNEL[t.channel].TC_CMR |= TC_CMR_CPCSTOP;
	return *this;
}

MIPStimer MIPStimer::nostopOnRC()
{
	Timer t = Timers[timer];

    t.tc->TC_CHANNEL[t.channel].TC_CMR &= ~TC_CMR_CPCSTOP;
	return *this;
}

MIPStimer MIPStimer::enableTrigger()
{
	// Get current timer configuration
	Timer t = Timers[timer];
	
	t.tc->TC_CHANNEL[t.channel].TC_CCR = TC_CCR_CLKEN;
    t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR) | TC_CMR_ENETRG;
    
    //t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR) | 0xD00;
}

MIPStimer MIPStimer::halt(bool state)
{
	// Get current timer configuration
	Timer t = Timers[timer];
	
	if(state) t.tc->TC_CHANNEL[t.channel].TC_CCR = 0x02;
	else 
	{
	   t.tc->TC_CHANNEL[t.channel].TC_CCR = 0x01;
	}
}

MIPStimer MIPStimer::softwareTrigger()
{
	TC_Start(Timers[timer].tc, Timers[timer].channel);
}


// clock options are:
//  TC_CMR_TCCLKS_TIMER_CLOCK1    MCK/2
//  TC_CMR_TCCLKS_TIMER_CLOCK2    MCK/8
//  TC_CMR_TCCLKS_TIMER_CLOCK3    MCK/32
//  TC_CMR_TCCLKS_TIMER_CLOCK4    MCK/128
//  TC_CMR_TCCLKS_XC0 External clock on TCLK2, D30
MIPStimer MIPStimer::setClock(uint32_t clock)
{
	// Get current timer configuration
	Timer t = Timers[timer];

    t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_TCCLKS_Msk)) | clock;
	return *this;
}

// trigger options are:
//  TC_CMR_EEVTEDG_NONE			Software trigger
//  TC_CMR_EEVTEDG_RISING		TIOB rising edge
//  TC_CMR_EEVTEDG_FALLING		TIOB falling edge
//  TC_CMR_EEVTEDG_EDGE			TIOB both edges
MIPStimer MIPStimer::setTrigger(uint32_t trigger)
{
	// Get current timer configuration
	Timer t = Timers[timer];

    t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_EEVTEDG_Msk | TC_CMR_EEVT_Msk)) | trigger | TC_CMR_EEVT_TIOB;
//    t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
//					     & ~(TC_CMR_EEVTEDG_Msk | TC_CMR_EEVT_Msk)) | trigger | TC_CMR_EEVT_XC2;
	return *this;
}

// trigger options are:
//  TC_CMR_EEVTEDG_NONE			Software trigger
//  TC_CMR_EEVTEDG_RISING		TIOB rising edge
//  TC_CMR_EEVTEDG_FALLING		TIOB falling edge
//  TC_CMR_EEVTEDG_EDGE			TIOB both edges
MIPStimer MIPStimer::setTriggerQ(uint32_t trigger)
{
	// Get current timer configuration
	Timer t = Timers[timer];

    t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_EEVTEDG_Msk | TC_CMR_EEVT_Msk)) | trigger | TC_CMR_EEVT_XC2;
	return *this;
}

MIPStimer MIPStimer::setTIOAeffectNOIO(uint32_t count, uint32_t effect)
{
	// Get current timer configuration
	Timer t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RA = count;
		

	t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_ACPA_Msk | TC_CMR_ACPC_Msk | TC_CMR_AEEVT_Msk | TC_CMR_ASWTRG_Msk)) 
					     | effect;
					     
	// Enable the RA Compare Interrupt if effect != 0...
	if(effect !=0)
	{
	   t.tc->TC_CHANNEL[t.channel].TC_IER |= TC_IER_CPAS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR &= ~TC_IER_CPAS;
    }
    else
    {
	   t.tc->TC_CHANNEL[t.channel].TC_IER &= ~TC_IER_CPAS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR |= TC_IER_CPAS;
    }
	return *this;
}

MIPStimer MIPStimer::setTIOAeffect(uint32_t count, uint32_t effect)
{
	// Get current timer configuration
	Timer t = Timers[timer];

   if(TIOApins[timer] != -1) PIO_Configure(g_APinDescription[TIOApins[timer]].pPort,
        				   g_APinDescription[TIOApins[timer]].ulPinType,
        				   g_APinDescription[TIOApins[timer]].ulPin,
        				   g_APinDescription[TIOApins[timer]].ulPinConfiguration);

	t.tc->TC_CHANNEL[t.channel].TC_RA = count;
		

	t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_ACPA_Msk | TC_CMR_ACPC_Msk | TC_CMR_AEEVT_Msk | TC_CMR_ASWTRG_Msk)) 
					     | effect;
					     
	// Enable the RA Compare Interrupt if effect != 0...
	if(effect !=0)
	{
	   t.tc->TC_CHANNEL[t.channel].TC_IER |= TC_IER_CPAS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR &= ~TC_IER_CPAS;
    }
    else
    {
	   t.tc->TC_CHANNEL[t.channel].TC_IER &= ~TC_IER_CPAS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR |= TC_IER_CPAS;
    }
	return *this;
}

MIPStimer MIPStimer::setRA(uint32_t count)
{
    static Timer t;
    
	// Get current timer configuration
	t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RA = count;
	return *this;
}

MIPStimer MIPStimer::incRA(uint32_t count)
{
    static Timer t;
    
	// Get current timer configuration
	t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RA += count;
	return *this;
}

MIPStimer MIPStimer::setTIOBeffect(uint32_t count, uint32_t effect)
{
	// Get current timer configuration
	Timer t = Timers[timer];

//   if(TIOBpins[timer] != -1) PIO_Configure(g_APinDescription[TIOBpins[timer]].pPort,
//        				   g_APinDescription[TIOBpins[timer]].ulPinType,
//        				   g_APinDescription[TIOBpins[timer]].ulPin,
//        				   g_APinDescription[TIOBpins[timer]].ulPinConfiguration);
        				   
	t.tc->TC_CHANNEL[t.channel].TC_RB = count;
	
	t.tc->TC_CHANNEL[t.channel].TC_CMR = (t.tc->TC_CHANNEL[t.channel].TC_CMR 
					     & ~(TC_CMR_BCPB_Msk | TC_CMR_BCPC_Msk | TC_CMR_BEEVT_Msk | TC_CMR_BSWTRG_Msk)) 
					     | effect;

	// Make sure EEVT in CMR is not 0, TIOB will not happen if it is 0
	if((t.tc->TC_CHANNEL[t.channel].TC_CMR & TC_CMR_EEVT_Msk) == 0) 
		t.tc->TC_CHANNEL[t.channel].TC_CMR |= TC_CMR_EEVT_XC0;
	
	// Enable the RB Compare Interrupt if effect != 0...
	if(effect !=0)
	{
	   t.tc->TC_CHANNEL[t.channel].TC_IER |= TC_IER_CPBS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR &= ~TC_IER_CPBS;
    }
    else
    {
	   t.tc->TC_CHANNEL[t.channel].TC_IER &= ~TC_IER_CPBS;
	   t.tc->TC_CHANNEL[t.channel].TC_IDR |= TC_IER_CPBS;
    }
	return *this;
}

MIPStimer MIPStimer::setRB(uint32_t count)
{
    static Timer t;
    
	// Get current timer configuration
	t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RB = count;
	return *this;
}

MIPStimer MIPStimer::incRB(uint32_t count)
{
    static Timer t;
    
	// Get current timer configuration
	t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RB += count;
	return *this;
}

MIPStimer MIPStimer::setRC(uint32_t count)
{
    static Timer t;
    
	// Get current timer configuration
	t = Timers[timer];

	t.tc->TC_CHANNEL[t.channel].TC_RC = count;
	return *this;
}

MIPStimer MIPStimer::start(long microseconds, uint8_t ClockDivisor, bool NoInterrupts)
{
	// Start the timer
	// If a period is set, then sets the period and start the timer

	if(microseconds > 0)
		setPeriod(microseconds,ClockDivisor);
	
	if(_frequency[timer] <= 0)
		setFrequency(1,ClockDivisor);

	if(!NoInterrupts)
	{
		NVIC_ClearPendingIRQ(Timers[timer].irq);
		NVIC_EnableIRQ(Timers[timer].irq);
	}		
	TC_Start(Timers[timer].tc, Timers[timer].channel);
	return *this;
}

uint8_t MIPStimer::bestClock(double frequency, uint32_t& retRC, uint8_t ClockDivisor)
{
	/*
		Pick the best Clock, thanks to Ogle Basil Hall!

		Timer		Definition
		TIMER_CLOCK1	MCK /  2
		TIMER_CLOCK2	MCK /  8
		TIMER_CLOCK3	MCK / 32
		TIMER_CLOCK4	MCK /128
	*/
	struct {
		uint8_t flag;
		uint8_t divisor;
	} clockConfig[] = {
		{ TC_CMR_TCCLKS_TIMER_CLOCK1,   2 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK2,   8 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK3,  32 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
	};
	float ticks;
	float error;
	int clkId = 3;
	int bestClock = 3;
	float bestError = 1.0;
	do
	{
	        if(ClockDivisor == clockConfig[clkId].flag)
	        {
	           bestClock = clkId;
	           break;
	        }
		ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[clkId].divisor;
		error = abs(ticks - round(ticks));
		if (abs(error) < bestError)
		{
			bestClock = clkId;
			bestError = error;
		}
	} while (clkId-- > 0);
	ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[bestClock].divisor;
	retRC = (uint32_t) round(ticks);
	return clockConfig[bestClock].flag;
}

MIPStimer MIPStimer::setFrequency(double frequency, uint8_t ClockDivisor)
{
	// Set the timer frequency (in Hz)

	// Prevent negative frequencies
	if(frequency <= 0) { frequency = 1; }

	// Remember the frequency
	_frequency[timer] = frequency;

	// Get current timer configuration
	Timer t = Timers[timer];

	uint32_t rc = 0;
	uint8_t clock;

	// Tell the Power Management Controller to disable 
	// the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);

	// Enable clock for the timer
	pmc_enable_periph_clk((uint32_t)t.irq);

	// Find the best clock for the wanted frequency
	clock = bestClock(frequency, rc, ClockDivisor);
	
	// Set the clock frequency variable
	if(clock == TC_CMR_TCCLKS_TIMER_CLOCK1) ClockFrequency[timer] = (double)VARIANT_MCK / 2.0;
	if(clock == TC_CMR_TCCLKS_TIMER_CLOCK2) ClockFrequency[timer] = (double)VARIANT_MCK / 8.0;
	if(clock == TC_CMR_TCCLKS_TIMER_CLOCK3) ClockFrequency[timer] = (double)VARIANT_MCK / 32.0;
	if(clock == TC_CMR_TCCLKS_TIMER_CLOCK4) ClockFrequency[timer] = (double)VARIANT_MCK / 128.0;


	// Set up the Timer in waveform mode which creates a PWM
	// in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.
	TC_Configure(t.tc, t.channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);
	// Reset counter and fire interrupt when RC value is matched:
	TC_SetRC(t.tc, t.channel, rc);
	// Enable the RC Compare Interrupt...
	t.tc->TC_CHANNEL[t.channel].TC_IER=TC_IER_CPCS;
	// ... and disable all others.
	t.tc->TC_CHANNEL[t.channel].TC_IDR=~TC_IER_CPCS;

	return *this;
}

MIPStimer MIPStimer::setPeriod(long microseconds, uint8_t ClockDivisor){
	// Set the period of the timer (in microseconds)

	// Convert period in microseconds to frequency in Hz
	double frequency = 1000000.0 / microseconds;	
	setFrequency(frequency,ClockDivisor);
	return *this;
}

double MIPStimer::getFrequency()
{
	// Get current time frequency
	return _frequency[timer];
}

long MIPStimer::getPeriod()
{
	// Get current time period
	return 1.0/getFrequency()*1000000;
}

double MIPStimer::getClockFrequency()
{
	return ClockFrequency[timer];
}

bool MIPStimer::checkStatusBit(uint32_t bitMask)
{
    uint32_t  i;
    
	// Get current timer configuration
	Timer t = Timers[timer];
	SR[timer] &= 0xFF;
	SR[timer] |= t.tc->TC_CHANNEL[t.channel].TC_SR;
	i = SR[timer];
	SR[timer] &= ~bitMask;
	if((i & bitMask) != 0) return true;
	return false;
}

uint32_t MIPStimer::getStatus()
{
	// Get current timer configuration
	Timer t = Timers[timer];
	
	return(t.tc->TC_CHANNEL[t.channel].TC_SR);
}

uint32_t MIPStimer::getCounter()
{
	// Get current timer configuration
	Timer t = Timers[timer];
	
	return(t.tc->TC_CHANNEL[t.channel].TC_CV);
}

uint32_t MIPStimer::getRAcounter()
{
	// Get current timer configuration
	Timer t = Timers[timer];
	
	return(t.tc->TC_CHANNEL[t.channel].TC_RA);
}

/*
	Implementation of the timer callbacks defined in 
	arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/sam3x8e.h
*/
void TimerTrapISR()
{
}

void TC0_Handler()
{
	MIPStimer::SR[0] &= 0xFF;
	MIPStimer::SR[0] |=  TC0->TC_CHANNEL[0].TC_SR;
	if(MIPStimer::SR[0] & TC_SR_CPAS) MIPStimer::callbacksRA[0]();
	if(MIPStimer::SR[0] & TC_SR_CPBS) MIPStimer::callbacksRB[0]();
	if(MIPStimer::SR[0] & TC_SR_CPCS) MIPStimer::callbacks[0]();
	MIPStimer::SR[0] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC1_Handler()
{
	MIPStimer::SR[1] &= 0xFF;
	MIPStimer::SR[1] |=  TC0->TC_CHANNEL[1].TC_SR;
	if(MIPStimer::SR[1] & TC_SR_CPAS) MIPStimer::callbacksRA[1]();
	if(MIPStimer::SR[1] & TC_SR_CPBS) MIPStimer::callbacksRB[1]();
	if(MIPStimer::SR[1] & TC_SR_CPCS) MIPStimer::callbacks[1]();
	MIPStimer::SR[1] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC2_Handler()
{
	MIPStimer::SR[2] &= 0xFF;
	MIPStimer::SR[2] |=  TC0->TC_CHANNEL[2].TC_SR;
	if(MIPStimer::SR[2] & TC_SR_CPAS) MIPStimer::callbacksRA[2]();
	if(MIPStimer::SR[2] & TC_SR_CPBS) MIPStimer::callbacksRB[2]();
	if(MIPStimer::SR[2] & TC_SR_CPCS) MIPStimer::callbacks[2]();
	MIPStimer::SR[2] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC3_Handler()
{
	MIPStimer::SR[3] &= 0xFF;
	MIPStimer::SR[3] |=  TC1->TC_CHANNEL[0].TC_SR;
	if(MIPStimer::SR[3] & TC_SR_CPAS) MIPStimer::callbacksRA[3]();
	if(MIPStimer::SR[3] & TC_SR_CPBS) MIPStimer::callbacksRB[3]();
	if(MIPStimer::SR[3] & TC_SR_CPCS) MIPStimer::callbacks[3]();
	MIPStimer::SR[3] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC4_Handler()
{
	MIPStimer::SR[4] &= 0xFF;
	MIPStimer::SR[4] |=  TC1->TC_CHANNEL[1].TC_SR;
	if(MIPStimer::SR[4] & TC_SR_CPAS) MIPStimer::callbacksRA[4]();
	if(MIPStimer::SR[4] & TC_SR_CPBS) MIPStimer::callbacksRB[4]();
	if(MIPStimer::SR[4] & TC_SR_CPCS) MIPStimer::callbacks[4]();
	MIPStimer::SR[4] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC5_Handler()
{
	MIPStimer::SR[5] &= 0xFF;
	MIPStimer::SR[5] |=  TC1->TC_CHANNEL[2].TC_SR;
	if(MIPStimer::SR[5] & TC_SR_CPAS) MIPStimer::callbacksRA[5]();
	if(MIPStimer::SR[5] & TC_SR_CPBS) MIPStimer::callbacksRB[5]();
	if(MIPStimer::SR[5] & TC_SR_CPCS) MIPStimer::callbacks[5]();
	MIPStimer::SR[5] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC6_Handler()
{
	MIPStimer::SR[6] &= 0xFF;
	MIPStimer::SR[6] |=  TC2->TC_CHANNEL[0].TC_SR;
	if(MIPStimer::SR[6] & TC_SR_CPAS) MIPStimer::callbacksRA[6]();
	if(MIPStimer::SR[6] & TC_SR_CPBS) MIPStimer::callbacksRB[6]();
	if(MIPStimer::SR[6] & TC_SR_CPCS) MIPStimer::callbacks[6]();
	MIPStimer::SR[6] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void TC7_Handler()
{
	MIPStimer::SR[7] &= 0xFF;
	MIPStimer::SR[7] |= TC2->TC_CHANNEL[1].TC_SR;
	if(MIPStimer::SR[7] & TC_SR_CPAS) MIPStimer::callbacksRA[7]();
	if(MIPStimer::SR[7] & TC_SR_CPBS) MIPStimer::callbacksRB[7]();
	if(MIPStimer::SR[7] & TC_SR_CPCS) MIPStimer::callbacks[7]();
	MIPStimer::SR[7] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
void __attribute__((weak)) TC8_Handler() 
{
	MIPStimer::SR[8] &= 0xFF;
	MIPStimer::SR[8] |= TC2->TC_CHANNEL[2].TC_SR;
	if(MIPStimer::SR[8] & TC_SR_CPAS) { MIPStimer::callbacksRA[8](); MIPStimer::SR[8] &= ~TC_SR_CPAS;}
	if(MIPStimer::SR[8] & TC_SR_CPBS) { MIPStimer::callbacksRB[8]();  MIPStimer::SR[8] &= ~TC_SR_CPBS;}
	if(MIPStimer::SR[8] & TC_SR_CPCS) { MIPStimer::callbacks[8]();  MIPStimer::SR[8] &= ~TC_SR_CPCS;}
//	MIPStimer::SR[8] &= ~(TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS);
}
