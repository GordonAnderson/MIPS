#ifndef DCBIAS_H_
#define DCBIAS_H_

extern float MaxDCbiasVoltage;
extern int   NumberOfDCChannels;
extern bool  DCbiasUpdate;
extern bool  DCbiasBoards[2];
extern bool  DCbiasTestEnable;

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
  int8_t  DACspi;            // SPI address for control DAC
  // TWI device addresses
  uint8_t ADCadr;            // 8 channel ADC used for readback
  uint8_t DACadr;            // 4 channel DAC, channel A used for offset
  uint8_t EEPROMadr;
  DCbiasChannellData  DCCD[8];   // 8 channel maximum
  DCbiasChannellData  DCoffset;  // Offset channel data
  bool    UseOneOffset;          // If true only one offset is used for both boards
  bool    OffsetReadback;        // True if the offset channel has readback. This is always on the last ADC channel
} DCbiasData;

typedef struct
{
  float Readbacks[8];       // Monitor voltage readback storage
  // Variables used to determine changes in values
  float DCbiasV[8];
  float DCbiasO;
} DCbiasState;

extern DCbiasData  *DCbDarray[4];

// Prototypes
int   DCbiasValue2Counts(int chan, float value);
float DCbiasCounts2Value(int chan, int counts);
bool  isDCbiasBoard(int Board);
void  DCbiasDACupdate(int chan, int counts);
void  DCbiasPowerSet(char *cmd);
void  DCbiasPower(void);
void  DelayMonitoring(void);

void  DCbiasRead(int, float **);
void  DCbiasRead(int);
bool  DCbiasReadMax(int chan, float *fVal);
void  DCbiasReadMax(int chan);
bool  DCbiasReadMin(int chan, float *fVal);
void  DCbiasReadMin(int chan);
void  DCbiasSet(char *Chan, char *value);
void  DCbiasSet(int Chan, float value);
void  DCbiasSetNumBoardChans(int board, int num);
void  DCbiasReportAllSetpoints(void);
void  SetAllDCbiasChannels(void);
void  DCbiasReportAllValues(void);
void  DCbiasDelta(char *Value);
void  DCbiasOffsetable(char *schan, char *state);
void  DCbiasUseOneOffset(char *state);
void  DCbiasOffsetReadback(char *state);

void SetDCbiasProfile(void);
void GetDCbiasProfile(int num);
void SetApplyDCbiasProfile(int num);
void SetDCbiasProfileFromCurrent(int num);
void StopProfileToggle(void);
void SetDCbiasProfileToggle(void);


#endif





