#ifndef DCBIAS_H_
#define DCBIAS_H_

#define MAXDCbiasMODULES   4

extern float MaxDCbiasVoltage;
extern int   NumberOfDCChannels;
extern bool  DCbiasUpdate;
extern bool  DCbiasBoards[2];
extern bool  DCbiasTestEnable;
extern bool  DCbiasUpdate;
extern bool  AutoReset;

// DCbias pulse channel variables
extern bool  DCbiasPena;
extern int   DCbiasPchan;
extern float DCbiasPvoltage;
extern int   DCbiasPdelay;
extern int   DCbiasPwidth;


typedef struct
{
  float    VoltageSetpoint;      // DC bias channel setpoint voltage
  // Hardware specific definitions
  DACchan  DCctrl;               // DC bias DAC to control output voltage
  ADCchan  DCmon;                // DC bias ADC channel used for monitor
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
  // The following variables support application of an offset voltage
  float   OffsetOffset;          // This offset is added to the offset channel
  uint8_t OffsetChanMsk;         // Offset channel mask. Set a bits to enable offset on a channel
  float   ChannelOffset;         // Channel offset applied to the channels enabled via OffsetChanMsk bits
  float   ADCgainOff;            // Gain to be applied to the ADC value when offset adjust is enabled
  float   ADCgainCh;             // Gain to be applied to the ADC value when channel offset adjust is enabled
  char    PolDIO;                // This defines a DIO signal to use for polarity control of the offset
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
void  SetDCbiasADCtwiADD(int module, int add);
void  SetDCbiasDACtwiADD(int module, int add);
void  SetDCbiasRange(int board, int range);
void  SetDCbiasLimit(int board, int range);
void  SetDCbiasExtended(int board);
void  ReportDCbiasSuppplies(int module);

void SetDCbiasProfile(void);
void GetDCbiasProfile(int num);
void SetApplyDCbiasProfile(int num);
void SetDCbiasProfileFromCurrent(int num);
void StopProfileToggle(void);
void SetDCbiasProfileToggle(void);

void SetDCbiasPena(char *state);
void SetDCbiasPtrigger(char *src);
void GetDCbiasPtrigger(void);

void SaveDCB2EEPROM(void);

void DCbiasCalParms(int chan);
void DCbiasCalsetM(char *chan, char *val);
void DCbiasCalsetB(char *chan, char *val);

void SetDCbiasOffOff(char *brd, char *fval);
void GetDCbiasOffOff(int board);
void SetDCbiasCHOff(char *brd, char *fval);
void GetDCbiasCHOff(int board);
void SetDCbiasCHMK(char *brd, char *mask);
void GetDCbiasCHMK(int board);
void SetADCoffsetAdjust(char *brd, char *gain);
void SetADCchannelAdjust(char *brd, char *gain);
void SetADCgainPol(char *brd, char *dio);
void GetADCgainPol(int board);
void SetLevelDetOffsetAdjust(char *brd, char *TWIadd);
void SetLevelDetChOffsetAdjust(char *brd, char *TWIadd);
int DCbiasChan2DAC(int chan);

bool CalDCbiasChannel(int channel);
void CalDCbiasChannels(void);
bool CalDCbiasOffset(int channel);

#endif
