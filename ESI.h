//
#ifndef ESI_H_
#define ESI_H_

extern float MaxESIvoltage;
extern bool  ESIcurrentTest;

enum ESIlevMode
{
  ESI_OFF  = 0,
  ESI_IFGT = 1,
  ESI_IFLT = 2,
};

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
  // Added 07/28/2020 to support using the level detector to enable and disable 
  // the ESI channel
  int     ESIstepsize;       // Allow user to program this level in volts per 10mS units
  ESIlevMode LevelMode[2];   // Defines the level detection mode
  float   LevelThreshold[2]; // Level detection system threshold levels 
  bool    EnableGatting[2];  // Set true to enable the gating mode. The relay is used for fast on/off
  bool    Gate[2];
  bool    SystemGatable;     // Flag  set to  enable the gate menu options  
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

void SetESIgateEnable(char *module, char *state);
void SetESIramp(int module, int ramp);
void GetESIramp(int module);

#endif
