#ifndef ANALOG_H_
#define ANALOG_H_

#include "Hardware.h"

typedef struct
{
  char    Name[20];
  int     Chan;                          // This is the controlling channel number if needed
  float   ADCvalue;                      // In volts, 0 to 5.0 is the range
  float   Min;
  float   Max;
  ADCchan DCmon;
  void    (*ValueUpdate)(int, float);    // Pointer to function called with value update
} AnalogChannel;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "Analog"  
  int8_t        Rev;                    // Holds the board revision number
  bool          Enabled;
  int           NumChannels;
  int8_t        ADC1add;                // TWI address of first ADC device, channel 0 through 3
  int8_t        ADC2add;                // TWI address of second ADC device, channel 4 through 7
  AnalogChannel ac[8];
} Analog;

// Prototypes
void AnalogChanCal(void);
void SaveAnalogSettings(void);
void RestoreAnalogSettings(void);
void Analog_init(void);
void RestoreAnalogSettings(bool);
void RestoreAnalogSettings(void);

#endif




