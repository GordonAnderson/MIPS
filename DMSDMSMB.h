#ifndef DMSDMSMB_h
#define DMSDMSMB_h
#include "hardware.h"

#if DMSDMSMB
#define SIGNATURE  0xAA55A5A5

#define MIPSscanAdv         5        // MIPS IO pin used to trigger scan advance on DMSDMS system
                                     // Connects to PB13 on both CPS on the DMSDMS motherboard 
                                     // This is pin 42 on the motherboard

#define FRAGPORT            Serial3  // Serial port used to comminicate with the fragmentor module
#define FRAGBAUD            19200    // Baudrate for frag serial interface

// TWI commands and constants
#define TWI_SET_ENABLE      0x01      // Set system enable, true = enabled. bool
#define TWI_SET_FREQ        0x02      // Set frequency, int 32
#define TWI_SET_DUTY        0x03      // Set duty cycle, unsigned 8 bit, percentage
#define TWI_SET_MODE        0x04      // Set Vrf mode, true = closed loop, bool
#define TWI_SET_DRIVE       0x05      // Drive level, float, percentage
#define TWI_SET_VRF         0x06      // Set the Vrf voltage setpoint, float
#define TWI_SET_MAXDRV      0x07      // Set the maximum drive level, float
#define TWI_SET_MAXPWR      0x08      // Set the maximum power, float

#define TWI_SET_CV          0x09      // Set CV voltage, float
#define TWI_SET_BIAS        0x0A      // Set BIAS voltage, float

#define TWI_SET_CV_START    0x0B      // Set CV scan start voltage, float
#define TWI_SET_CV_END      0x0C      // Set CV scan end voltage, float
#define TWI_SET_VRF_START   0x0D      // Set Vrf scan start voltage, float
#define TWI_SET_VRF_END     0x0E      // Set Vrf scan end voltage, float
#define TWI_SET_DURATION    0x0F      // Set step duration in mS, int
#define TWI_SET_STEPS       0x10      // Set number of steps, int
#define TWI_SET_EXTSTEP     0x11      // Set to true to exable external step advance, bool
#define TWI_SET_STPPIN      0x12      // Defines the external pin used to advance step, byte
#define TWI_SET_STEPSTR     0x13      // Starts step scan, no args
#define TWI_SET_STEPSTP     0x14      // Stops step scan, no args

#define TWI_SET_FLUSH       0x15      // Flushs the output buffer
#define TWI_SET_CAL         0x16      // Generates the Drive level to Vrf calibration tables needed for scanning Vrf
#define TWI_SET_SCNRPT      0x17      // Scan report flag, false to stop report, true by default

#define TWI_SET_VRF_NOW     0x18      // Set the Vrf voltage setpoint and then adjust the drive to achive the setpoint
#define TWI_SET_VRF_TABLE   0x29      // Set the Vrf voltage using the calibration table, float

#define TWI_SERIAL          0x27      // This command enables the TWI port to process serial commands

#define TWI_SET_ELEC_POSOFF 0x40      // Set electrometer positive offset voltage, float
#define TWI_SET_ELEC_NEGOFF 0x41      // Set electrometer negative offset voltage, float
#define TWI_SET_ELEC_POSZ   0x42      // Set electrometer positive zero voltage, float
#define TWI_SET_ELEC_NEGZ   0x43      // Set electrometer negative zero voltage, float
#define TWI_SET_ELEC_ZERO   0x44      // Perform electrometer zero adjust
#define TWI_SET_ELEC_M4     0x45      // True to use M4 ADC acquire loop for current readings

#define TWI_READ_READBACKS  0x81      // Returns the readback structure
#define TWI_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_READ_DRIVE      0x83      // Returns the current drive setting, float
#define TWI_READ_VRF        0x84      // Returns the current Vrf setting, float
#define TWI_READ_CV         0x85      // Returns the current CV setting, float

#define TWI_READ_ELEC_POS   0x86      // Returns the electrometer positive current, float
#define TWI_READ_ELEC_NEG   0x87      // Returns the electrometer negative current, float
#define TWI_READ_ELEC_POSZ  0x88      // Returns the electrometer positive zero voltage, float
#define TWI_READ_ELEC_NEGZ  0x89      // Returns the electrometer negative zero voltage, float

typedef struct
{
  float        CV;          // Actual CV voltage
  float        Bias;        // Actual bias voltage
} CVBIASreadBacks;

typedef struct
{
  bool          update;
  bool          Enable[2];
  float         CV[2];
  float         Bias[2];
  float         PosZero;
  float         NegZero;
  float         PosOffset;
  float         NegOffset;
  float         CVstart[2];
  float         CVend[2];
  int           Steps;
  int           StepDuration;
} CVBIASstate;

typedef struct
{
  bool          Enable;
  float         CV;
  float         Bias;
  // AD5592 ADC channels
  ADCchan       DCBAMon;                // DC bias output A voltage monitor
  ADCchan       DCBBMon;                // DC bias output B voltage monitor
  // AD5592 DAC channels
  DACchan       DCBACtrl;               // DC bias output A voltage control
  DACchan       DCBBCtrl;               // DC bias output B voltage control
  // Scanning parameters
  float         CVstart;
  float         CVend;
} CVBIASChanParams;

typedef struct
{
  bool          M4ena;             // True to enable M4's ADC to read and process the current inputs
  ADCchan       PosCtrl;           // ADC channel 0, Positive input
  ADCchan       NegCtrl;           // ADC channel 1, Negative input
  DACchan       PosZeroCtrl;       // DAC channel 2, positive zero control
  DACchan       NegZeroCtrl;       // DAC channel 3, Negative zero control
  DACchan       PosOffsetCtrl;     // DAC channel 4, positive offset control
  DACchan       NegOffsetCtrl;     // DAC channel 5, positive offset control
  float         PosZero;
  float         NegZero;
  float         PosOffset;
  float         NegOffset;
} Electrometer;

// The DMS channels are numbered 0 and 1 and the CV bias outputs are A and B
typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name
  int8_t        Rev;                    // Holds the board revision number
  int8_t        TWIadd;                 // Base TWI address, not used!
  bool          Enable;
  float         DCref;
  DACchan       DCrefCtrl;              // DC reference voltage, channel 3
  CVBIASChanParams    channel[2];       // Bias channels, 0 and 1 for the two DMS channels
  Electrometer  electrometer;           // Electrometer control parameters
  // Scanning parameters
  float         Duration;               // Scan time in seconds
  int           Loops;
  // Step based scanning
  int           Steps;
  int           StepDuration;           // in mSec
  bool          EnableExtStep;          // Enable the use of external step advance input
  int           ExtAdvInput;            // Define the pin to be used to advance scanning,
  // External scan trigger options, these parameters are used by MIPS.
  char          ScanTrigger;            // Trigger input channel
  int8_t        TriggerLevel;           // Trigger level, 0,CHANGE,RISING, or FALLING 
  unsigned int Signature;               // Must be 0xAA55A5A5 for valid data
} CVBIASdata;

typedef struct
{
  float        V;          // DC voltage into driver monitor, channel 6
  float        I;          // DC current into driver monitor, channel 7
  float        Vrf;        // Vrf actual voltage, M0 ADC channel A0
} WAVEFORMSreadBacks;

typedef struct
{
  bool          update;
  bool          Enable[2];            // Turns the FAIMS drive on or off
  float         Vrf[2];               // Setpoint voltage
  int           Freq[2];              // FAIMS frequency
  int8_t        Duty[2];              // FAIMS duty cycle
  float         Drive[2];             // Drive level, in percentage
  bool          Mode[2];              // True if in closed loop control
  float         MaxPower[2];
  float         MaxDrive[2];
  float         VRFstart[2];
  float         VRFend[2];
  int           Steps;
  int           StepDuration;
} WAVEFORMSstate;

typedef struct
{
  bool          Enable;
  int           Freq;                   // FAIMS frequency
  int           Duty;                   // FAIMS duty cycle
  float         Drive;                  // Drive level, in percentage
  float         Vrf;                    // Setpoint voltage
  bool          Mode;                   // True if in closed loop control
  float         MaxPower;
  float         MaxDrive;
  float         loopGain;
  // AD5592 ADC channels
  ADCchan       DCVmon;                 // DC voltage into driver monitor
  ADCchan       DCImon;                 // DC current into driver monitor
  // Vrf ADC channel on M4 processor
  ADCchan       VRFMon;
  // Scanning parameters
  float         VRFstart;
  float         VRFend;
  // Lookup table for drive level to Vrf calibration
  float         LUVrf[21];
} WAVEFORMSChanParams;

// The DMS channels are numbered 0 and 1 and the CV bias outputs are A and B
typedef struct
{
  int16_t       Size;               // This data structures size in bytes
  char          Name[20];           // Holds the board name
  int8_t        Rev;                // Holds the board revision number
  int8_t        TWIadd;             // Base TWI address, not used!
  bool          Enable;
  WAVEFORMSChanParams    channel[2];
  // Scanning parameters
  float         Duration;           // Scan time in seconds
  int           Loops;
  // Step based scanning
  int           Steps;
  int           StepDuration;       // in mSec
  bool          EnableExtStep;      // Enable the use of external step advance input
  int           ExtAdvInput;        // Define the pin to be used to advance scanning,
  // External scan trigger options, these parameters are used by MIPS.
  char          ScanTrigger;        // Trigger input channel
  int8_t        TriggerLevel;       // Trigger level, 0,CHANGE,RISING, or FALLING 
  unsigned int Signature;           // Must be 0xAA55A5A5 for valid data
} WAVEFORMSdata;

extern CVBIASdata      cvbiasdata;
extern WAVEFORMSdata   waveformsdata;

bool RestoreDMSDMSsettings(int brd, CVBIASdata  *cvd,bool NoDisplay);
bool RestoreDMSDMSsettings(int brd, WAVEFORMSdata  *wfd,bool NoDisplay);
void RestoreDMSDMSsettings(void);

void CVBIAS_init(int brd, int add);
void WAVEFORMS_init(int brd, int add);

// The following parameters support the high voltage module used for an
// ion source

// AD5592 hardware parameters
#define   HVM_TWI   0x11
// AD5592 channel use
#define   CHhvpENA  0
#define   CHhvpCtrl 1
#define   CHhvpMon  2
#define   CHhvnENA  3
#define   CHhvnCtrl 4
#define   CHhvnMon  5

// HV module data structure
typedef struct
{
  int16_t       Size;               // This data structures size in bytes
  char          Name[20];           // Holds the module name
  int8_t        Rev;                // Holds the board revision number
  int8_t        TWIadd;             // Base TWI address, not used!
  bool          Enable;
  bool          Inited;
  // HV module parameters
  float         voltage;            // Setpoint voltage
  float         MaxVoltage;
  // ADC and DAC calibration structures
  DACchan       VposCtrl;
  ADCchan       VposMon;
  DACchan       VnegCtrl;
  ADCchan       VnegMon;
} HVmodule;

#endif
#endif