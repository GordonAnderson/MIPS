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
#define TWI_SET_VECTOR      0x06      // Set the waveform vector, 32 8 bit unsigned values. 0 to 255
#define TWI_SET_ENABLE      0x07      // Enables or diables the ARB outputs. 0 = diaable, 1 = enables

#define TWI_READ_REQ_FREQ   0x81      // Returns requested frequency
#define TWI_READ_ACT_FREQ   0x82      // Returns actual frequency

// Waveform types, used in TWI command
#define TWI_WAVEFORM_SIN    0x01
#define TWI_WAVEFORM_RAMP   0x02
#define TWI_WAVEFORM_TRI    0x03
#define TWI_WAVEFORM_PULSE  0x04
#define TWI_WAVEFORM_ARB    0x05

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
} ARBdata;

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

#endif

