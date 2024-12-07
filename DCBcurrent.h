#ifndef DCBcurrent_h
#define DCBcurrent_h

#if DCBcurrent

#define DCBCurAdd    0x48    // DCbias current monitor base address
#define FILTER        0.1

typedef struct
{
  uint8_t   rev;              // Revision of driver
  uint16_t  size;             // Size of this struct
  bool      TestEna;          // Enables limit testing
  float     TrigLevel;        // Trigger level in mA
	ADCchan	  curCH[8];		      // ADC current input channels parameters
} DCBcurData;

typedef struct
{
  float   ch[8];
} Currents;

bool dcbcurEnable(int CH);

#endif
#endif


