#ifndef DCbiasCtrl_h
#define DCbiasCtrl_h

#if DCBanalog

#include "DCBias.h"

#define TCA9534add    0x21    // DIO TWI base address

#define EEPROMOFFSET  352
#define ADC_CS        2
#define FILTER        0.1

typedef struct
{
  uint8_t   rev;              // Revision of driver
  uint16_t  size;             // Size of this struct
	uint8_t	  chanMode;		      // bit set for Analog input mode
	int		    offsets[8];		    // DAC offsets for analog input mode
	ADCchan	  adcCH[8];		      // ADC input calibration parameters
} DCBiasCtrlData;

typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "ADC"
  int8_t  Rev;               // Holds the board revision number
} ADCmodule;

typedef struct
{
  float   ch[8];
} ADCctrlVs;

// prototypes
void ADC_init(int8_t Board, int8_t addr);
bool dcbctrlEnable(int CH);

#endif
#endif
