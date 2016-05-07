/*
  DueTimer.h - DueTimer header file, definition of methods and attributes...
  For instructions, go to https://github.com/ivanseidel/DueTimer

  Created by Ivan Seidel Gomes, March, 2013.
  Modified by Philipp Klaus, June 2013.
  Released into the public domain.
*/

#ifdef __arm__

#ifndef DueTimer_h
#define DueTimer_h

#include "Arduino.h"

#include <inttypes.h>

/*
	This fixes compatibility for Arduono Servo Library.
	Uncomment to make it compatible.

	Note that:
		+ Timers: 0,2,3,4,5 WILL NOT WORK, and will
				  neither be accessible by Timer0,...
*/
// #define USING_SERVO_LIB	true

#ifdef USING_SERVO_LIB
	#warning "HEY! You have set flag USING_SERVO_LIB. Timer0, 2,3,4 and 5 are not available"
#endif


#define NUM_TIMERS  9
class ICallBackTimer
{
protected:
	friend void TC0_Handler(void);
	friend void TC1_Handler(void);
	friend void TC2_Handler(void);
	friend void TC3_Handler(void);
	friend void TC4_Handler(void);
	friend void TC5_Handler(void);
	friend void TC6_Handler(void);
	friend void TC7_Handler(void);
	friend void TC8_Handler(void);
	
	virtual void OnTimerCallBack(void * parameter) {};
};

class DueTimer
{
protected:
	// Represents the timer id (index for the array of Timer structs)
	const unsigned short timer;
	void * parameter;
	// Stores the object timer frequency
	// (allows to access current timer period and frequency):
	static double _frequency[NUM_TIMERS];

	// Picks the best clock to lower the error
	static uint8_t bestClock(double frequency, uint32_t& retRC);

	// Make Interrupt handlers friends, so they can use callbacks
	friend void TC0_Handler(void);
	friend void TC1_Handler(void);
	friend void TC2_Handler(void);
	friend void TC3_Handler(void);
	friend void TC4_Handler(void);
	friend void TC5_Handler(void);
	friend void TC6_Handler(void);
	friend void TC7_Handler(void);
	friend void TC8_Handler(void);

	static void (*callbacks[NUM_TIMERS])();
	static ICallBackTimer *icallbacks[NUM_TIMERS];
	
	struct Timer
	{
		Tc *tc;
		uint32_t channel;
		IRQn_Type irq;
	};

	// Store timer configuration (static, as it's fixed for every object)
	static const Timer Timers[NUM_TIMERS];
	//static const DueTimer* DueTimers[NUM_TIMERS];
	static const DueTimer *DueTimers[NUM_TIMERS];
public:
	DueTimer(unsigned short _timer);
	static DueTimer& getAvailable(void);
	static int getAvailableCount(void);
	static DueTimer& getTimer(unsigned short _timer);
	
	DueTimer& attachInterrupt(void (*isr)());
	DueTimer& attachInterrupt(ICallBackTimer* callbacktimer,void *_parameter = NULL);
	DueTimer& detachInterrupt(void);
	DueTimer& start(long microseconds = -1,bool once = false);
	DueTimer& stop(void);
	DueTimer& setFrequency(double frequency,bool once = false);
	DueTimer& setPeriod(unsigned long microseconds,bool once = false);

	double getFrequency(void) const;
	long getPeriod(void) const;
};

// Just to call Timer.getAvailable instead of Timer::getAvailable() :
extern DueTimer Timer;

extern DueTimer Timer1;
// Fix for compatibility with Servo library
#ifndef USING_SERVO_LIB
	extern DueTimer Timer0;
	extern DueTimer Timer2;
	extern DueTimer Timer3;
	extern DueTimer Timer4;
	extern DueTimer Timer5;
#endif
extern DueTimer Timer6;
extern DueTimer Timer7;
extern DueTimer Timer8;

#endif

#else
	#error Oops! Trying to include DueTimer on another device?
#endif
