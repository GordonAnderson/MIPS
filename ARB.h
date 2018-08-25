#ifndef ARB_H_
#define ARB_H_

// Digital IO pins used by the ARB system
#define ARBsync 9
#define ARBmode 48                    // Signals ARB module to enter compress mode

#define CompressBoard 1               // Defines the board address used for the compressor 

#define ARBnormal   digitalWrite(ARBmode,LOW)
#define ARBcompress digitalWrite(ARBmode,HIGH)

// TWI commands and constants
//#define TWI_ARB_ADD         0x32    // This is the TWI address for the ARB module's Arduino
#define TWI_ARB_ADD         0x40      // This is the TWI address for the ARB module's Arduino for channels 1 and 2
                                      // use 0x42 for channels 3 and 4. (40 = 64, 42 = 66)

#define TWI_SET_FREQ        0x01      // Set frequency, Hz, 16 bit unsigned word
#define TWI_SET_WAVEFORM    0x02      // Set waveform type, 8 bit unsigned type
#define TWI_SET_REF         0x03      // Set reference defines p-p voltage, 16 bit unsigned, 0 to 4095
#define TWI_SET_OFFSET      0x04      // Set offset, 16 bit unsigned, 0 to 4095
#define TWI_SET_DIR         0x05      // Set waveform direction, 8 bit unsigned. 0 = forward, 1 = reverse
#define TWI_SET_VECTOR      0x06      // Set the waveform vector format:
                                      // 8 bit offset into buffer, 8 bit number of bytes (29 max)
                                      // 8 bit unsigned values. 0 to 255
#define TWI_SET_ENABLE      0x07      // Enables or diables the ARB outputs. 0 = diaable, 1 = enables
#define TWI_SET_RANGE       0x08      // Sets the output range, 0 to 50, float
#define TWI_SET_OFFSETV     0x09      // Sets the output offset, -50 to 50, float
#define TWI_SET_AUX         0x0A      // Sets the aux output, -50 to 50, float
#define TWI_SET_BUFFER_LEN  0x0B      // Define the buffer length in samples, word
#define TWI_SET_NUM_BUFFER  0x0C      // Number of times to play buffer on each trigger, byte
#define TWI_SET_SET_BUFFER  0x0D      // Set all values in buffer, float
#define TWI_SET_SET_CHANNEL 0x0E      // Set a channel number in buffer, byte, float
#define TWI_SET_SET_CHN_RNG 0x0F      // Set a channel value over range in buffer, byte, word, work, float
#define TWI_SET_MODE        0x10      // Set ARB mode, 0 = TWAVEmode, 1 = ARBmode
#define TWI_SET_DAC         0x11      // Set DAC channel, 8 bit DAC number, float DAC level, -100 to 100
#define TWI_SET_ARB_INDEX   0x12      // Set ARB buffer index, 16 bit word that points to DAC location in buffer
#define TWI_SET_ARB_VECTOR  0x13      // Set ARB vector in buffer, format:
                                      // 8 bit channel number, 8 bit number of bytes (29 max)
                                      // 8 bit unsiged values. 0 to 255. The index will automatically advance to allow long buffer fills.
#define TWI_SET_SYNC_ENA    0x14      // Set external sync enable (true or false) 
#define TWI_SET_COMP_ENA    0x15      // Set compression enable (true or false) 
#define TWI_SET_COMP_ORDER  0x16      // Set compression order, byte value, 0 to 255 
#define TWI_SET_COMP_EXT    0x17      // This flag will enable the hardware line to define normal/compression modes
                                      // true will enable, when the hardware line is high then we are in compress mode.
#define TWI_SET_EXT_CLOCK   0x18      // This command enables external (off board) clock. accepts true or false, true = external clock
#define TWI_SET_BRD_BIAS    0x19      // This command sets the board bias voltage, 1 byte board number, 1 float with value
#define TWI_SET_PWR         0x20      // This command turns on and off power supply. accepts true or false, true = on
#define TWI_SET_TST_ENABLE  0x21      // This command enables voltage testing and shutdown. accepts true or false, true = enable
#define TWI_SET_CRAMP       0x22      // This command sets the cpression order ramp variable, 16 bit int, -200 to 200
#define TWI_SET_CRAMPORDER  0x23      // This command sets the cpression order ramp step size, 16 bit int, 1 to 200

// The following commands are used for the pulse sequence generator. The update
// commands send the data that is queued and loaded when the load updates command is sent to the ARB
#define TWI_UPDATE_AUX      0x24      // Updates the aux output, -50 to 50, float
#define TWI_UPDATE_BRD_BIAS 0x25      // This command sets the board bias voltage, 1 byte board number, 1 float with value
#define TWI_LOAD_UPDATES    0x26      // Updated values are loaded into the hardware via this command

#define TWI_SERIAL             0x27      // This command enables the TWI port to process serial commands
#define TWI_SET_COMP_ORDER_EX  0x28      // Set compression order, word value, 0 to 65535 

#define TWI_READ_REQ_FREQ   0x81      // Returns requested frequency
#define TWI_READ_ACT_FREQ   0x82      // Returns actual frequency
#define TWI_READ_STATUS     0x83      // Returns status byte

// Waveform types, used in TWI command
#define TWI_WAVEFORM_SIN    0x01
#define TWI_WAVEFORM_RAMP   0x02
#define TWI_WAVEFORM_TRI    0x03
#define TWI_WAVEFORM_PULSE  0x04
#define TWI_WAVEFORM_ARB    0x05

// Constants
#define ppp   32  // Number of points per waveform period
#define NP    32  // Number of periods in buffer
#define CHANS 8   // Total number of DAC channels
#define MAXBUFFER 8000

enum WaveFormTypes
{
  ARB_SIN,
  ARB_RAMP,
  ARB_TRIANGLE,
  ARB_PULSE,
  ARB_ARB
};

typedef struct
{
  char  Mode[7];
  int   Freq;
  float Amplitude;
  float Offset;
  float OffsetA;
  float OffsetB;
  float Aux;
  int   BufferLength;
  int   NumBuffers;
  int   Direction;
  int   WFT;
  int   Enable;
  bool  ARBwaveformUpdate;
} ARBstate;

// One struct for each ARB board
typedef struct
{
  int16_t       Size;              // This data structures size in bytes
  char          Name[20];          // Holds the board name, "ARB"
  int8_t        Rev;               // Holds the board revision number
  bool          Enable;            // Turns the ARB on and off
  int8_t        NumChannels;       // Number of channels suppoted by this board
  bool          Direction;         // Twave direction
  float         MaxVoltage;        // Defines board voltage range
  float         MinVoltage;  
  bool          Offsetable;        // True if this board is offsetable
  int           Frequency;         // Requested frequency
  float         Voltage;           // Output p-p voltage
  float         Offset;            // Offset voltage of 0 point voltage
  uint8_t       PPP;               // Number of data points per period
  WaveFormTypes wft;               // Waveform type
  int8_t        WaveForm[32];      // Arbitrary waveform storage
  // TWI device addresses
  uint8_t       ARBadr;            // Arduino Due ARB controller address
  uint8_t       EEPROMadr;
  // Additional parameters added for rev 2 upgade to ARB module
  char          Mode[6];           // TWAVE or ARB modes
  float         Aux;               // Aux output voltage
  char          ARBsyncIn;         // Sync input in TWAVE mode or trigger in AARB mode
  int8_t        ARBsyncLevel;      // Sync level, 0,CHANGE,RISING, or FALLING
  // ARB mode parameters
  int           BufferLength;      // Buffer length
  int           NumBuffers;        // Number of times to play the buffer on each trigger

  char          ARBdirDI;          // Sync input in TWAVE mode or trigger in AARB mode
  int8_t        ARBdirLevel;       // Sync level, 0,CHANGE,RISING, or FALLING
  // Compressor variables
  bool          UseCommonClock;     // Flag set to true to use a common clock, this will cause both modules to set each others value
  bool          CompressorEnabled;  // True if the compressor mode has been enabled
  uint8_t       CorderNU;           // Compressor order, 1 to 20, no longer used, extended to 16 bit unsigned
  int8_t        NumPasses;          // Total number of passes through device
  float         Tdelay;             // Delay from trigger to start of compressor or pass, in millisec
  float         Tcompress;          // Time in compress mode, in millisec
  float         Tnormal;            // Time in normal mode, in millisec
  float         TnoC;               // Time for non compressed pass, in millisec
  char          Ctrig;              // External input for compressor start trigger
  int8_t        CtrigLevel;         // External input triggerl level
  char          Cswitch;            // Digitial output to control output switch to relead ions to mass spec
  int8_t        CswitchLevel;       // Digital output level control
  // Flags
  bool          ARBcommonOffset;    // If true only one board provides offset for both channels
  // Dual output board offset, supported only on rev 3 arb system
  bool          DualOutputs;        // True for dual output boards installed for one ARB
  float         OffsetA;            // Output set A offset
  float         OffsetB;            // Output set B offset
  int           Cramp;              // Order ramping
  int           CrampOrder;         // Order ramping step size
  // Extended compresion order to unsigned 16 bits
  int           Corder;
} ARBdata;

extern ARBdata  *ARBarray[4];
extern DialogBoxEntry ARBCompressorEntries2[];
extern DialogBox ARBCompressorDialog;
extern MIPStimer *ARBclock;

//Prototypes
String GetWaveformString(WaveFormTypes wft);
WaveFormTypes GetWaveformType(String WaveformString);
void SetEnable(void);
void SelectARBmodule(bool);
void SetFrequency(void);
void SetAmplitude(void);
void SetOffset(void);
void SetBoardBias(int board, int add, float Voltage, uint8_t cmd = TWI_SET_BRD_BIAS);
void SetWaveform(void);
void SetDirection(void);
void SetARBwaveform(void);

void UpdateAux(int8_t brd, float val, bool FlushQueued);
void UpdateOffsetA(int8_t brd, float val, bool FlushQueued);
void UpdateOffsetB(int8_t brd, float val, bool FlushQueued);
void ProcessQueuedARB(void);
void QueueARBupdate(bool Release = false);
void ProcessARB(void);

void ReportARBchannels(void);
void SetARBbufferLength(int module, int len);
void GetARBbufferLength(int module);
void SetARBbufferNum(int module, int num);
void GetARBbufferNum(int module);
void SetARBMode(char *module, char *mode);
void GetARBMode(int module);
void SetWFfreq(int module, int freq);
void GetWFfreq(int module);
void SetWFenable(int module);
void SetWFdisable(int module);
void SetWFrange(char *module, char *srange);
void GetWFrange(int module);
void SetWFoffsetV(char *module, char *srange);
void GetWFoffsetV(int module);
void SetWFaux(char *module, char *srange);
void GetWFaux(int module);
void SetARBdirection(char *module, char *dir);
void GetARBdirection(int module);
void SetARBwaveform(void);
void GetARBwaveform(int module);
void SetARBchns(char *module, char *sval);
void SetARBchannel(void);
void SetARBchanRange(void);
void SetARBwfType(char *sMod, char *Swft);
void GetARBwfType(int module);
void SetARBUseCommonClock(char *module, char *flag);
void SetARBcommonOffset(char *flag);
void SetARBtwiADD(int module, int add);
void SetARBDualBoard(char *module, char *sval);
void SetARBoffsetBoardA(char *module, char *val);
void GetARBoffsetBoardA(int module);
void SetARBoffsetBoardB(char *module, char *val);
void GetARBoffsetBoardB(int module);
void ARBmoduleSync(void);

// Compressor proto types
void SetARBCompressorEnabled(char *flag);
void SetARBCmode(char *mode);
void GetARBCorder(void);
void SetARBCorder(int ival);
void GetARBCtriggerDelay(void);
void SetARBCtriggerDelay(char *str);
void GetARBCcompressTime(void);
void SetARBCcompressTime(char *str);
void GetARBCnormalTime(void);
void SetARBCnormalTime(char *str);
void GetARBCnoncompressTime(void);
void SetARBCnoncompressTime(char *str);
void ARBCtrigger(void);
void SetARBCswitch(char *mode);

void SaveARB2EEPROM(void);

#endif







