#ifndef DIhandler_h
#define DIhandler_h

#include <Arduino.h>
#include <inttypes.h>

#define MaxDIhandlers  20

class DIhandler
{
//protected:
public:
	static  int DIpin[8];		      // This array maps digital inputs Q - X to the pin numbers
	static  IRQn_Type irqs[8];
	static  Pio *pio[8];
	static  uint32_t pin[8];
	static  void (*DI_ISR[8])(void);  // Interrupt handelers
	static  int NumHandlers;
	static  DIhandler *handlers[MaxDIhandlers];
	static  int NumInterrupts;
	
	char   di;				 // Digital input channel, Q - X
	int    mode;		 	 // Mode used for this call back, Pos, Neg, or Change
	void  (*userISR)(void);  // function called when a valid Interrupt happens
//public:
  DIhandler(void);
  ~DIhandler(void);
	bool attached(char DI, int mode,void (*isr)(void));	// Attaches an ISR, true if it was able to do
	bool isAttached(void);
	void detach(void);      // Detaches if posible, if the DI is chained it may not be posible to release
  void setPriority(uint8_t pri);
  void setPriority(char DI, uint8_t pri);
  uint32_t getPriority(void);
	bool activeLevel(void);	// Returns true if the active level is true
	bool state(void);	      // Returns the inputs state, true if high
	bool test(int mode);	  // Returns true if input state matches the level
};

#endif


