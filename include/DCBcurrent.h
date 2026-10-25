#ifndef DCBcurrent_h
#define DCBcurrent_h

#if DCBcurrent

#define DCBCurAdd    0x48    // DCbias current monitor base address
#define FILTER        0.1

// I2C interface used to talk to the current monitor modules (ADS7828 ADC) on the Wire1 pins
// (Arduino pins 70 SDA, 71 SCL, PA17/PA18):
//   false = the hardware Wire1 (TWI0) interface
//   true  = bit banged software I2C (soft_i2c.cpp) on the same pins. The current monitors are the
//           only devices on these pins, so the soft driver owns them.
// Use the pin numbers, not the Arduino SDA1/SCL1 names: Hardware.h redefines SDA1/SCL1 as 19/18,
// which overrides the variant's 70/71 in any file that includes it.
#define DCBcurUseSoftI2C       true
#define DCBcurSoftSDA          70   // Soft I2C SDA Arduino pin number, 70 = PA17
#define DCBcurSoftSCL          71   // Soft I2C SCL Arduino pin number, 71 = PA18
#define DCBcurSoftI2CdelayUS   5      // Soft I2C half clock period in uS, 5 = about 100 kHz or less
#define DCBcurSoftI2Cstretch   10000  // Soft I2C clock stretch timeout in uS

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
void curScan(void);

#endif
#endif


