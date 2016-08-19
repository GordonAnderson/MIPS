#ifndef Twave_h
#define Twave_h

extern float MaxTwaveVoltage;

extern DialogBox TwaveDialog;
extern DialogBox TwaveDialog2;
extern DialogBox TwaveCalMenu;

extern char TwaveCompressorTable[];

extern char Cmode[];
extern char CswitchState[];

enum CompressorState
{
  CS_TRIG,
  CS_NONCOMPRESS,
  CS_COMPRESS,
  CS_NORMAL,
  CS_DELAY
};

enum CompressorSwitchState
{
  CSS_OPEN_REQUEST,
  CSS_CLOSE_REQUEST,
  CSS_OPEN,
  CSS_CLOSE,
  CSS_IDLE
};

typedef struct
{
  float  VoltageSetpoint;      // Twave channel setpoint voltage
  // Hardware specific definitions
  DACchan  DCctrl;             // Twave DAC to control output voltage
  ADCchan  DCmon;              // Twave ADC channel used for monitor
} TwaveChannellData;

typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name
  int8_t  Rev;               // Holds the board revision number
  int     Velocity;
  uint8_t Sequence;
  // TWI device addresses
  uint8_t DACadr;
  uint8_t ADCadr;
  uint8_t GPIOadr;
  uint8_t CLOCKadr;
  // DAC/ADC channel data data
  TwaveChannellData  TWCD[4]; // 4 total channels, Pulse, Resting, Guard 1, Guard 2
  // The following paraneters were added for rev 2.0 and above
  bool    Direction;          // Twave direction, forward or reverse, true = forward
  // The following parameters were add for rev 3 and above
  char   TWdirDI;             // External input for direction control
  int8_t TWdirLevel;          // External input direction control level
  char   TWsyncDI;            // External input for sync control
  int8_t TWsyncLevel;         // External input sync control level
  //
  bool    UseCommonClock;     // Flag set to true to use a common clock, this will cause both modules is set each others value
  // The following variable support the Twave compressor mode of operation
  bool   CompressorEnabled;   // True if the compressor mode has been enabled
  uint8_t Corder;              // Compressor order, 1 to 20
  int8_t NumPasses;           // Total number of passes through device
  int8_t CNth;                // Compress every Nth pass
  float  Tdelay;              // Delay from trigger to start of compressor or pass, in millisec
  float  Tcompress;           // Time in compress mode, in millisec
  float  Tnormal;             // Time in normal mode, in millisec
  float  TnoC;                // Time for non compressed pass, in millisec
  char   Ctrig;               // External input for compressor start trigger
  int8_t CtrigLevel;          // External input triggerl level
  char   Cswitch;             // Digitial output to control output switch to relead ions to mass spec
  int8_t CswitchLevel;        // Digital output level control
  // Added Feb 20, 2016
  char   TWgateDI;            // External gate for Twave output
  int8_t TWgateLevel;         // External gate for Twave outlut level
} TwaveData;

// Multi-pass compressor stack structure used for loops
typedef struct
{
  bool Inited;
  int StartOfLoop;
  int Count;
} CompressorStack;

extern TwaveData TD;
extern TwaveData TDarray[2];

// Prototypes
void SetPulseVoltage(void);
void SetVelocity(void);
void SetSequence(void);
void SetRestingVoltage(void);
void SetGuard1(void);
void SetGuard2(void);
void Twave_init(int8_t Board, uint8_t addr);
void Twave_loop(void);

// Compressor prototypes
void ConfigureTrig(void);
void ConfigureSwitch(void);
void SetSwitch(void);

// Host command prototypes
void TWAVEnumberOfChannels(void);
void sendTWAVEsequence(int channel);
void sendTWAVEfrequency(int channel);
void setTWAVEfrequency(int channe, int freq);
void setTWAVEsequence(char *chan, char *value);
void sendTWAVEpulseVoltage(int channel);
void setTWAVEpulseVoltage(char *chan, char *voltage);
void sendTWAVEguard1Voltage(int channel);
void setTWAVEguard1Voltage(char *chan, char *voltage);
void sendTWAVEguard2Voltage(int channel);
void setTWAVEguard2Voltage(char *chan, char *voltage);
void getTWAVEdir(int channe);
void setTWAVEdir(char *chan, char *dirstr);

// Compressor host command prototypes
void SetTWCmode(char *mode);
void GetTWCorder(void);
bool RangeTest(DialogBoxEntry *des, char *EntryName, float fval);
void SetTWCorder(int ival);
void SetTWCtriggerDelay(char *str);
void SetTWCcompressTime(char *str);
void SetTWCnormalTime(char *str);
void SetTWCnoncompressTime(char *str);
void TWCtrigger(void);
void SetTWCswitch(char *mode);

#endif




