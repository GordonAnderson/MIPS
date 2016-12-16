//
#ifndef ESI_H_
#define ESI_H_

extern float MaxESIvoltage;

typedef struct
{
  float  VoltageSetpoint;      // DC bias channel setpoint voltage
  float  MaxVoltage;           // Full scale voltage for this channel
  float  MaxCurrent;           // Output current trip point
  // Hardware specific definitions
  DACchan  DCctrl;             // DC bias DAC to control output voltage
  ADCchan  DCVmon;             // DC bias ADC channel used for voltage monitor
  ADCchan  DCImon;             // DC bias ADC channel used for current monitor
} ESIChannellData;

// One struct for each ESI board
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "ESI"
  int8_t  Rev;               // Holds the board revision number
  ESIChannellData ESIchan[2];
  // TWI device addresses
  uint8_t ADCadr;            // 4 channel ADC used for readback
  uint8_t DACadr;            // 4 channel DAC, channel A used for offset
  uint8_t EEPROMadr;
} ESIdata;

// Prototypes
void ESInumberOfChannels(void);
void SetESIchannel(char *Chan, char *Value);
void GetESIchannel(int chan);
void GetESIchannelV(int chan);
void GetESIchannelI(int chan);
void GetESIchannelMax(int chan);

#endif






