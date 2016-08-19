#ifndef ARB_H_
#define ARB_H_

#define ARBsync 9

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
  uint8_t       WaveForm[32];      // Arbitrary waveform storage
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
} ARBdata;

extern ARBdata  ARBarray[2];

//Prototypes
String GetWaveformString(WaveFormTypes wft);
WaveFormTypes GetWaveformType(String WaveformString);
void SetEnable(void);
void SelectARBmodule(void);
void SetFrequency(void);
void SetAmplitude(void);
void SetOffset(void);
void SetWaveform(void);
void SetDirection(void);
void SetARBwaveform(void);

void ReportARBchannels(void);
void SetARBbufferLength(int len);
void SetARBbufferNum(int num);
void SetARBMode(char *mode);
void GetARBMode(void);
void SetWFfreq(int freq);
void GetWFfreq(void);
void SetWFenable(void);
void SetWFdisable(void);
void SetWFrange(char *srange);
void SetWFoffsetV(char *srange);
void SetWFaux(char *srange);
void SetARBchns(char *sval);
void SetARBchannel(char * sch, char *sval);
void SetARBchanRange(void);

#endif



