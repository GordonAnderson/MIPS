#include "Thread.h"

Thread::Thread(void (*callback)(void), long _interval){
	enabled = true;
	onRun(callback);
	_cached_next_run = 0;
	last_run = 0;

	ThreadID = (int)this;
	#ifdef USE_THREAD_NAMES
		ThreadName = "Thread ";
		ThreadName = ThreadName + ThreadID;
	#endif

	setInterval(_interval);
};

void Thread::setName(char *name)
{
   Name = name;
}

const char *Thread::getName(void)
{
   return(Name.c_str());
}

void Thread::runned(long time)
{
	// If less than 0, than get current ticks
	if(time < 0) time = millis();

	// Saves last_run
	last_run = time;

	// Cache next run
	_cached_next_run = last_run + interval;
}

void Thread::setInterval(long _interval){
	// Filter intervals less than 0
	interval = (_interval < 0? 0: _interval);

	// Cache the next run based on the last_run
	_cached_next_run = last_run + interval;
}

void Thread::setNextRunTime(long _nextTime){
	// Set this tasks next run time
	_cached_next_run = _nextTime;
}

bool Thread::shouldRun(long time)
{
	// If less than 0, then get current ticks
	if(time < 0) time = millis();

    // Added by GAA, 3/1/18. In case millisec timer is reset
    // Force it to run if time is less that the last run time.
    // Un commented this line on 3/31/25
    if(time < last_run) _cached_next_run = time;
    
	// Exceeded the time limit, AND is enabled? Then should run...
	return ((time >= _cached_next_run) && enabled);
}

void Thread::onRun(void (*callback)(void)){
	_onRun = callback;
}

int Thread::getID(void)
{
   return(ThreadID);
}

long Thread::getInterval(void)
{
   return(interval);
}

unsigned long Thread::runTimeMs(void)
{
   return runTime;
}

void Thread::run(){
	runned();
	startTime = millis();
	if(_onRun != NULL)
		_onRun();
	runTime = millis() - startTime;

	// Update last_run and _cached_next_run
//	runned();
}