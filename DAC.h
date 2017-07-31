#ifndef DAC_H_
#define DAC_H_
// The DAC module uses a CPLD to provides the interface to the DAC allowing the DAC digitial
// signals to be shut off for better noise performance. The CPLD has a spi interface with a
// single 8 bit control word. 

// CPLD control byte bit definitions
#define DACcpldERROR        0
#define DACcpldSTATUS       1
#define DACcpldSELECTED     2
#define DACcpldDACenable    3

// CPLD control bits
#define DACstrobe  49

// DAC modules has 8 channels, 7 are -10 to 10 volts single ended and one is 0 to 10 volts differential.
// The system allows programming the channels engineering units and range for a specific application
typedef struct
{
  // Programable units and range
  float  Value;                // Engineering units value, converted to voltage using min and max
  float  Min;                  // This min and max value are used to calculate the output voltage
  float  Max;
  // labels
  char   Name[6];              // Channel name
  char   Units[6];             // Channel units
  float  VoltageSetpoint;      // DAC channel setpoint voltage
  // Hardware specific definitions
  DACchan  DCctrl;             // DAC channel address and counts to voltage cal parameters
} DACchannellData;

// One struct for each DC bias board
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "DCbias"
  int8_t  Rev;               // Holds the board revision number
  int8_t  NumChannels;       // Number of channels suppoted by this board
  int8_t  CPLDspi;           // SPI address for CPLD controller
  int8_t  DACspi;            // SPI address for control DAC 
  // TWI device addresses
  uint8_t EEPROMadr;
  DACchannellData  DACCD[8]; // 8 channels
} DACdata;

// This structure saves the current state of the DAC outputs.
// This is used to determine the channels that require updates,
// only the required updates are performed.
typedef struct
{
  float  Value[8];
} DACstate;

extern int   NumberOfDACchannels;

// Prototypes
void ReportDACchannels(void);
void SetDACValue(char *name, char *value);
void GetDACValue(char *name);
void SetDACMax(char *name, char *value);
void GetDACMax(char *name);
void SetDACMin(char *name, char *value);
void GetDACMin(char *name);
void SetDACUnits(char *name, char *value);
void GetDACUnits(char *name);
void SetDACName(int module, int channel, char *Name);
void GetDACName(int module, int channel);

#endif

