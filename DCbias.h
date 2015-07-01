#ifndef DCBIAS_H_
#define DCBIAS_H_

extern float MaxDCbiasVoltage;

typedef struct
{
  float  VoltageSetpoint;      // DC bias channel setpoint voltage
  // Hardware specific definitions
  DACchan  DCctrl;             // DC bias DAC to control output voltage
  ADCchan  DCmon;              // DC bias ADC channel used for monitor
} DCbiasChannellData;

// One struct for each DC bias board
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "DCbias"
  int8_t  Rev;               // Holds the board revision number
  int8_t  NumChannels;       // Number of channels suppoted by this board
  float   MaxVoltage;        // Defines board voltage range
  float   MinVoltage;  
  bool    Offsetable;        // True if this board is offsetable
  int8_t  DACspi;            // SPI address dor control DAC
  // TWI device addresses
  uint8_t ADCadr;            // 8 channel ADC used for readback
  uint8_t DACadr;            // 4 channel DAC, channel A used for offset
  uint8_t EEPROMadr;
  DCbiasChannellData  DCCD[8];   // 8 channel maximum
  DCbiasChannellData  DCoffset;  // Offset channel data
} DCbiasData;

// Prototypes
int  DCbiasValue2Counts(int chan, float value);
float DCbiasCounts2Value(int chan, int counts);
bool isDCbiasBoard(int Board);
void DCbiasDACupdate(int chan, int counts);
void DCbiasPowerSet(char *cmd);
void DCbiasPower(void);
void DelayMonitoring(void);

#endif


