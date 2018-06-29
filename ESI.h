//
#ifndef ESI_H_
#define ESI_H_

extern float MaxESIvoltage;
extern bool  ESIcurrentTest;

typedef struct
{
  bool   Enable;               // If true the output is enabled, if false its set to 0
  float  VoltageSetpoint;      // DC bias channel setpoint voltage
  float  MaxVoltage;           // Full scale voltage for this channel
  float  VoltageLimit;         // User programmable voltage limit
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
  // The following variables are used for the rev 3 ESI module using
  // EMCO modules and relay switching
  bool    Enable;           // Enable output
  float   VoltageSetpoint;  // Setpoint voltage
} ESIdata;

// Prototypes
void ESInumberOfChannels(void);
void SetESIchannel(char *Chan, char *Value);
void GetESIchannel(int chan);
void GetESIchannelV(int chan);
void GetESIchannelI(int chan);
void GetESIchannelMax(int chan);
void GetESIchannelMin(int chan);
void GetESIstatus(int chan);
void SetESIchannelDisable(int chan);
void SetESIchannelEnable(int chan);
void SetESImodulePos(int module, int value);
void SetESImoduleNeg(int module, int value);

void SaveESI2EEPROM(void);

#endif







