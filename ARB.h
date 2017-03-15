#ifndef ARB_H_
#define ARB_H_

// Digital IO pins used by the ARB system
#define ARBsync 9
#define ARBmode 48                    // Signals ARB module to enter compress mode

#define CompressBoard 1               // Defines the board address used for the compressor 

#define ARBnormal   digitalWrite(ARBmode,LOW)
#define ARBcompress digitalWrite(ARBmode,HIGH)

// TWI commands and constants
#define TWI_ARB_ADD         0x32

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

#define TWI_READ_REQ_FREQ   0x81      // Returns requested frequency
#define TWI_READ_ACT_FREQ   0x82      // Returns actual frequency

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
  bool          UseCommonClock;     // Flag set to true to use a common clock, this will cause both modules is set each others value
  bool          CompressorEnabled;  // True if the compressor mode has been enabled
  uint8_t       Corder;             // Compressor order, 1 to 20
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
} ARBdata;

extern ARBdata  ARBarray[2];
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
void SetWaveform(void);
void SetDirection(void);
void SetARBwaveform(void);

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

// Compressor proto types
void SetARBCmode(char *mode);
void GetARBCorder(void);
void SetARBCorder(int ival);
void SetARBCtriggerDelay(char *str);
void SetARBCcompressTime(char *str);
void SetARBCnormalTime(char *str);
void SetARBCnoncompressTime(char *str);
void ARBCtrigger(void);
void SetARBCswitch(char *mode);

#endif





