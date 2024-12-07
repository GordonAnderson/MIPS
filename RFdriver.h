#ifndef RFDRIVER_H_
#define RFDRIVER_H_

extern float MaxRFVoltage;
extern int   NumberOfRFChannels;

extern int   RFarcCH;
extern float RFarcDrv;
extern float RFarcV;

extern bool  RFgatingOff;
extern bool  TuneAbort;

// ADC control structure. This allows an ADC channel to control a RF channel's
// drive or setpoint depending on the RF driver channels mode. ADC input is assumed
// 0 to 10V.
// 0 to 10V = 0 to 100%
// 0 to 10V = 0 to 1000V 
// 
typedef struct
{
  bool  enable;           // Enable the channel
  int   RFchan;           // RF channel 1 thru 4
  float gain;             // ADC counts to voltage conversion factor
} ADCcontrolRF;

enum RFdriverMode
{
  RF_MANUAL,
  RF_AUTO,
};

typedef struct  // 58 bytes
{
  int    Freq;                 // RF driver frequency in Hz
  float  DriveLevel;           // RF driver level in percentage
  float  Setpoint;             // RF level in volts p-p for automatic control mode
  RFdriverMode  RFmode;        // Defines the RF driver mode of operation
  float  MaxDrive;             // Software limit to drive level
  float  MaxPower;             // Software power limit for RF head
  // Hardware specific definitions
  int8_t   PWMchan;             // Define the PWM output channel
  ADCchan  RFpADCchan;          // RF positive phase readback ADC channel
  ADCchan  RFnADCchan;          // RF negative phase readback ADC channel
  ADCchan  DriveVADCchan;       // RF drive voltage monitor ADC channel
  ADCchan  DriveIADCchan;       // RF drive current monitor ADC channel
} RFchannelData;

// One struct for each RF driver board
typedef struct  // 147 bytes
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name
  int8_t  Rev;               // Holds the board revision number
  int8_t  NumChannels;       // Number of channels suppoted by this board
  // TWI device addresses
  uint8_t ADCadr;
  uint8_t CLOCKadr;
  uint8_t EEPROMadr;
  RFchannelData  RFCD[2];
  // These parameters really belong in RFchnnelData but there were added after release so placing at end of struct gives backwards compatability
  char   RFgateDI[2];       // Gate input, 0 if not used otherwise its the input P-X
  int8_t RFgateTrig[2];     // Gate level, 0,CHANGE,RISING, or FALLING
  #if RFdriver2
  int    Signature;         // Must be 0xAA55A5A5 for valid data
  #endif
  float  PowerLimit;
  // Piece wise linear calibration data structures
  PWLcalibration PWLcal[2][2];   // Piece wise linear data structures, first index is RF channel, second
                                 // is phase, 0 for RF+ and 1 of RF-
} RFdriverData;

#if RFdriver2 == true
extern RFdriverData  *RFDDarray[];
#define   rfDDarray  (*RFDDarray)
#else
extern RFdriverData  RFDDarray[];
extern float         gatedDrive[4];
#define   rfDDarray  RFDDarray
void setRFgatedDrv(char *Chan, char *Val);
void getRFgatedDrv(int Chan);
#endif

// Prototypes
void RF_A1_ISR(void);
void RF_A2_ISR(void);
void RF_B1_ISR(void);
void RF_B2_ISR(void);

void RFdriver_init(int8_t Board, int8_t addr);
void RFfreq(int channel, int freq);
void RFdrive(int channel, float Drive);
void RFdrive(char *Chan, char *Val);
void RFvoltage(int channel, float Voltage);
void RFvoltage(char *Chan, char *Val);
void RFfreqReport(int channel);
void RFvoltageReportP(int channel);
void RFvoltageReportN(int channel);
void RFdriveReport(int channel);
void RFvoltageReport(int channel);
void RFheadPower(int channel);
void RFreportAll(void);
void RFautoTune(int channel);
void RFautoRetune(int channel);
void RFcalParms(void);
void RFcalP(char *, char *);
void RFcalN(char *, char *);
void GetRFpwrLimit(int channel);
void SetRFpwrLimit(int channel, int Power);
void genPWLcalTable(char *channel, char *phase);
float PWLlookup(int ch, int ph, int adcval);
int8_t BoardFromSelectedChannel(int8_t SC);
void RFgateDisable(char *cmd);
void SetRFautoTuneAbort(void);

void RFmodeReport(int);
void RFmodeSet(char *, char *);
void SaveRF2EEPROM(void);

void setupADCRFcontrol(char *chan, char *gain);
void setADCRFenable(char *chan, char *value);
void getADCRFenable(char *chan);

void UpdateRFdrive(int chan, float value);
void ProcessRFdrive(void);

#if RFdriver2
// RFdriver2 specific definitions

// TWI commands and constants
#define TWI_RF_SET_CHAN           0x01      // Set the active channel that all commands will reference, byte, channel 1 or 2
#define TWI_RF_SET_FREQ           0x02      // Set frequency, int 32
#define TWI_RF_SET_MODE           0x04      // Set Vrf mode, true = closed loop, bool
#define TWI_RF_SET_DRIVE          0x05      // Drive level, float, percentage
#define TWI_RF_SET_VRF            0x06      // Set the Vrf voltage setpoint, float
#define TWI_RF_SET_MAXDRV         0x07      // Set the maximum drive level, float
#define TWI_RF_SET_MAXPWR         0x08      // Set the maximum power, float
#define TWI_RF_SET_ATUNE          0x09      // Start the auto tune process for selected channel
#define TWI_RF_SET_RTUNE          0x0A      // Start the auto re-tune process for selected channel
#define TWI_RF_SET_CALP           0x0B      // Sets the calibration parameters for VRFP, m and b. two floats
#define TWI_RF_SET_CALN           0x0C      // Sets the calibration parameters for VRFN, m and b. two floats
#define TWI_RF_CALP               0x0D      // The command will adjust the positive channel gain to set the Vpp to the setpoint.
                                            // If the setpoint is negative then the calibration is set to default, float
#define TWI_RF_CALN               0x0E      // The command will adjust the negative channel gain to set the Vpp to the setpoint.
                                            // If the setpoint is negative then the calibration is set to default, float
#define TWI_RF_SET_GATENA         0x0F      // Enable the gate input of the DIO channel passed, when the DIO line is high the RF output 
                                            // is gated off, int
#define TWI_RF_SET_GATEDIS        0x10      // Disable the gate interrupt
#define TWI_RF_SET_GATE           0x11      // Gates the RF off if true, on if false. 
#define TWI_RF_SET_PWL            0x12      // Sets a table entry in the piecewise linear look up table.
                                            // channel (byte), phase (byte), index (byte), voltage (word) 
#define TWI_RF_SET_PWL_N          0x13      // Sets the number of piecewise linear look up table entries.
                                            // channel (byte), phase (byte), entries (byte) 

#define TWI_RF_SERIAL             0x27      // This command enables the TWI port to process serial commands

#define TWI_RF_READ_READBACKS     0x81      // Returns the readback structure
#define TWI_RF_READ_AVALIBLE      0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_RF_READ_DRIVE         0x83      // Returns the current drive setting, float
#define TWI_RF_READ_FREQ          0x84      // Returns the current frequency setting, int
#define TWI_RF_READ_TUNE          0x85      // Returns the auto tune flag, bool
#define TWI_RF_READ_CALP          0x86      // Returns the calibration parameters for VRFP, m and b. two floats
#define TWI_RF_READ_CALN          0x87      // Returns the calibration parameters for VRFN, m and b. two floats
#define TWI_RF_READ_PWL           0x88      // Returns the value and ADCvalue (words) for the requested PWL table entry
                                            // channel (byte), phase (byte), entry number (byte)
#define TWI_RF_READ_PWL_N         0x89      // Returns the the number of PWL table entries
                                            // channel (byte), phase (byte)

// Current status structure, used to determine if any values have changed. One
// for each channel in module.
typedef struct
{
  bool           update;
  int            Freq;                // RF driver frequency in Hz
  float          DriveLevel;          // RF driver level in percentage
  float          Setpoint;            // RF level in volts p-p for automatic control mode
  RFdriverMode   RFmode;              // Defines the RF driver mode of operation
  float          MaxDrive;            // Software limit to drive level
  float          MaxPower;            // Software power limit for RF head
} RFDRVstate;

// One struct for each channel, 2 channels in this module
typedef struct
{
  float        RFP;        // RF+ output voltage monitor
  float        RFN;        // RF- output voltage monitor
  float        V;          // Driver supply voltage monitor
  float        I;          // Driver supply current monitor
  float        PWR;        // Driver power level
} RFreadBacks;

#endif

#endif
