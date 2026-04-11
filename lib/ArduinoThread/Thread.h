/*
 	Thread.h - An runnable object

	Thread is responsable for holding the "action" for something,
	also, it responds if it "should" or "should not" run, based on
	the current time;

	For instructions, go to https://github.com/ivanseidel/ArduinoThread

	Created by Ivan Seidel Gomes, March, 2013.
	Released into the public domain.
	
	Functionality added by Gordon Anderson, July 2015
	1.) Added function to name a thread and retrive a threads name
	2.) Added get ID
	3.) Added thread run time calculation and ability to read the run time
	4.) Fixed the next run point in time to not include the processes run time,
	    this may have been a mistake?
*/

#ifndef Thread_h
#define Thread_h

#include <Arduino.h>
#include <inttypes.h>

/*
	Uncomment this line to enable ThreadName Strings.

	It might be usefull if you are loging thread with Serial,
	or displaying a list of threads...
*/
// #define USE_THREAD_NAMES	1

class Thread{
protected:
	// Desired interval between runs
	long interval;

	// Last runned time in Ms
	long last_run;

	// Scheduled run in Ms (MUST BE CACHED)	
	long _cached_next_run;
	
	// Variables used to determine the tasks run time in millsecs
	unsigned long startTime;
	unsigned long runTime;
	String Name;

	/*
		IMPORTANT! Run after all calls to run()
		Updates last_run and cache next run.
		NOTE: This MUST be called if extending
		this class and implementing run() method
	*/
	void runned(long time=-1);	

	// Callback for run() if not implemented
	void (*_onRun)(void);		

public:

	// If the current Thread is enabled or not
	bool enabled;

	// ID of the Thread (initialized from memory adr.)
	int ThreadID;

	#ifdef USE_THREAD_NAMES
		// Thread Name (used for better UI).
		String ThreadName;			
	#endif

	Thread(void (*callback)(void) = NULL, long _interval = 0);

    // Returns the run time in millisec for the last task call
    unsigned long runTimeMs(void);

    void setName(char *);
    const char *getName(void);
    
    int getID(void);
    long getInterval(void);
    
	// Set the desired interval for calls, and update _cached_next_run
	virtual void setInterval(long _interval);

	// Set the the next run time for task
	virtual void setNextRunTime(long _nextTime);

	// Return if the Thread should be runned or not
	virtual bool shouldRun(long time = -1);

	// Callback set
	void onRun(void (*callback)(void));

	// Runs Thread
	virtual void run();
};

#endif
