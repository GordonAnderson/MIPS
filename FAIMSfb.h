#ifndef FAIMSFB_h
#define FAIMSFB_h
#include "Hardware.h"

#define SIGNATURE  0xAA55A5A5
#define DRVPWMFREQ 50000

#define MIPSscanAdv         48

#define ESC   27
#define ENQ   5

// TWI commands and constants
#define TWI_FB_SET_ENABLE      0x01      // Set system enable, true = enabled. bool
#define TWI_FB_SET_FREQ        0x02      // Set frequency, int 32
#define TWI_FB_SET_DUTY        0x03      // Set duty cycle, unsigned 8 bit, percentage
#define TWI_FB_SET_MODE        0x04      // Set Vrf mode, true = closed loop, bool
#define TWI_FB_SET_DRIVE       0x05      // Drive level, float, percentage
#define TWI_FB_SET_VRF         0x06      // Set the Vrf voltage setpoint, float
#define TWI_FB_SET_MAXDRV      0x07      // Set the maximum drive level, float
#define TWI_FB_SET_MAXPWR      0x08      // Set the maximum power, float

#define TWI_FB_SET_CV          0x09      // Set CV voltage, float
#define TWI_FB_SET_BIAS        0x0A      // Set BIAS voltage, float

#define TWI_FB_SET_CV_START    0x0B      // Set CV scan start voltage, float
#define TWI_FB_SET_CV_END      0x0C      // Set CV scan end voltage, float
#define TWI_FB_SET_VRF_START   0x0D      // Set Vrf scan start voltage, float
#define TWI_FB_SET_VRF_END     0x0E      // Set Vrf scan end voltage, float
#define TWI_FB_SET_DURATION    0x0F      // Set step duration in mS, int
#define TWI_FB_SET_STEPS       0x10      // Set number of steps, int
#define TWI_FB_SET_EXTSTEP     0x11      // Set to true to exable external step advance, bool
#define TWI_FB_SET_STPPIN      0x12      // Defines the external pin used to advance step, byte
#define TWI_FB_SET_STEPSTR     0x13      // Starts step scan, no args
#define TWI_FB_SET_STEPSTP     0x14      // Stops step scan, no args

#define TWI_FB_SET_FLUSH       0x15      // Flushs the output buffer
#define TWI_FB_SET_CAL         0x16      // Generates the Drive level to Vrf calibration tables needed for scanning Vrf
#define TWI_FB_SET_SCNRPT      0x17      // Scan report flag, false to stop report, true by default

#define TWI_FB_SET_VRF_NOW     0x18      // Set the Vrf voltage setpoint and then adjust the drive to achive the setpoint

#define TWI_FB_SERIAL          0x27      // This command enables the TWI port to process serial commands
#define TWI_FB_SET_VRF_TABLE   0x29      // Set the Vrf voltage using the calibration table, float

#define TWI_FB_READ_READBACKS  0x81      // Returns the readback structure
#define TWI_FB_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_FB_READ_DRIVE      0x83      // Returns the current drive setting, float
#define TWI_FB_READ_VRF        0x84      // Returns the current Vrf setting, float
#define TWI_FB_READ_CV         0x85      // Returns the current CV setting, float

typedef struct // 28 bytes
{
  float CVrb;
  float VRFrb;
  float PWR;
  float PosCurrent;
  float NegCurrent;
  float PosOffset;
  float NegOffset;
} FAIMSFBvars;

typedef struct // 20 bytes
{
  float        CV;         // Actual CV voltage
  float        BIAS;       // Actual bias voltage
  float        V;          // DC voltage into driver monitor, channel 6
  float        I;          // DC current into driver monitor, channel 7
  float        Vrf;        // Vrf actual voltage, M0 ADC channel A0
} ReadBacks;

typedef struct // 20 bytes
{
  int       Point;
  uint32_t  TimeStamp;
  float     Vrf;
  float     CV;
} ScanPoint;

typedef struct // 76 bytes
{
  bool          update;
  bool          Enable;            // Turns the FAIMS drive on or off
  int           Freq;              // FAIMS frequency
  int8_t        Duty;              // FAIMS duty cycle
  float         Drive;             // Drive level, in percentage
  float         Vrf;               // Setpoint voltage
  bool          Mode;              // True if in closed loop control
  float         CV;
  float         Bias;
  float         DCref;
  float         MaxPower;
  float         MaxDrive;
  float         MaxOnTime;         // Defines the number of hours before the system automatically shuts down 
  float         ArcSens;
  float         VRFstart;
  float         VRFend;
  float         CVstart;
  float         CVend;
  int           Steps;
  int           StepDuration;      // in mSec
  float         PosOffset;
  float         NegOffset;
} FAIMSFBstate;

// This must match the structure in the FAIMSFB module's CPU
typedef struct  // 260 byte
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "FAIMSFB"
  int8_t        Rev;                    // Holds the board revision number
  bool          Enable;                 // Turns the FAIMS drive on or off
  int8_t        TWIadd;                 // Base TWI address, not used!
  int           Freq;                   // FAIMS frequency
  int           Duty;                   // FAIMS duty cycle
  float         Drive;                  // Drive level, in percentage
  float         Vrf;                    // Setpoint voltage
  bool          Mode;                   // True if in closed loop control
  float         CV;
  float         Bias;
  float         MaxPower;
  float         MaxDrive;
  float         MaxOnTime;              // Defines the number of hours before the system automatically shuts down 
  // Arc detector sensitivity
  float         ArcSens;
  // AD5592 ADC channels
  ADCchan       DCB1Mon;                // DC bias channel 1 voltage monitor, channel 4
  ADCchan       DCB2Mon;                // DC bias channel 2 voltage monitor, channel 5
  ADCchan       DCVmon;                 // DC voltage into driver monitor, channel 6
  ADCchan       DCImon;                 // DC current into driver monitor, channel 7
  // AD5592 DAC channels
  DACchan       DCB1Ctrl;               // DC bias channel 1 voltage control, channel 0
  DACchan       DCB2Ctrl;               // DC bias channel 2 voltage control, channel 1
  DACchan       DCrefCtrl;              // DC reference voltage, channel 3
  // Vrf ADC channel on M0 processor
  ADCchan       VRFMon;
  // Scanning parameters
  float         VRFstart;
  float         VRFend;
  float         CVstart;
  float         CVend;
  float         DCref;
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
  // Lookup table for drive level to Vrf calibration
  float         LUVrf[21];
  // Electrometer parameters, only used on module 1. This data is used by the MIPS controller. The
  // FAIMSfb module does not use these parameters.
  bool          ElectrometerEnable;
  int8_t        ElectAdd;               // TWI address for the electrometer AD5593, used Wire1, 0x11
  ADCchan       ElectPosCtrl;           // ADC channel 0, Positive input
  ADCchan       ElectNegCtrl;           // ADC channel 1, Negative input
  DACchan       ElectPosZeroCtrl;       // DAC channel 2, positive zero control
  DACchan       ElectNegZeroCtrl;       // DAC channel 3, Negative zero control
  DACchan       ElectPosOffsetCtrl;     // DAC channel 4, positive offset control
  DACchan       ElectNegOffsetCtrl;     // DAC channel 5, positive offset control
  float         ElectPosZero;
  float         ElectNegZero;
  float         ElectPosOffset;
  float         ElectNegOffset;
  //
  int           Signature;              // Must be 0xAA55A5A5 for valid data
  // The following variabels were added June 3, 2021. These support the new
  // electrometer with the on board M4 processor
  bool          ElectM4ena;             // The flag is true if the M4 processor is enabled for reading
  int8_t        ElectM4Add;             // TWI address for the electrometer M4 processor, used Wire1, 0x20
  ADCchan       ElectPosCtrlM4;         // ADC channel 0, Positive input
  ADCchan       ElectNegCtrlM4;         // ADC channel 1, Negative input
} FAIMSFBdata;

extern ReadBacks rb;
extern float     Power;
extern int       NumSamples;

// Prototypes
   
// Scan function prototypes

// Host command prototypes
int FAIMSfbModule2Brd(int module, bool response = false);
int FAIMSfbModule2Brd(char *module, bool response = false);
void SetFAIMSfbEnable(char *module, char *ena);
void ReturnFAIMSfbEnable(int module);
void SetFAIMSfbMode(char *module, char *mode);
void ReturnFAIMSfbMode(int module);
void SetFAIMSfbFreq(int module, int freq);
void ReturnFAIMSfbFreq(int module);
void SetFAIMSfbDuty(int module, int duty);
void ReturnFAIMSfbDuty(int module);
void SetFAIMSfbDrive(char *module, char *drive);
void ReturnFAIMSfbDrive(int module);
void ReturnFAIMSfbDriveV(int module);
void ReturnFAIMSfbDriveI(int module);
void SetFAIMSfbVrf(char *module, char *Vrf);
void ReturnFAIMSfbVrf(int module);
void ReturnFAIMSfbVrfV(int module);
void ReturnFAIMSfbPWR(int module);
void SetFAIMSfbMaxDrive(char *module, char *MaxDrv);
void ReturnFAIMSfbMaxDrive(int module);
void SetFAIMSfbMaxPower(char *module, char *MaxPwr);
void ReturnFAIMSfbMaxPower(int module);
void SetFAIMSfbCV(char *module, char *CV);
void ReturnFAIMSfbCV(int module);
void ReturnFAIMSfbCVrb(int module);
void SetFAIMSfbBIAS(char *module, char *BIAS);
void ReturnFAIMSfbBIAS(int module);
void ReturnFAIMSfbBIASrb(int module);
void SetFAIMSfbCVstart(char *module, char *CVstart);
void ReturnFAIMSfbCVstart(int module);
void SetFAIMSfbCVend(char *module, char *CVend);
void ReturnFAIMSfbCVend(int module);
void SetFAIMSfbVRFstart(char *module, char *VRFstart);
void ReturnFAIMSfbVRFstart (int module);
void SetFAIMSfbVRFend(char *module, char *VRFend);
void ReturnFAIMSfbVRFend(int module);
void SetFAIMSfbStepDuration(int module, int StpDur);
void ReturnFAIMSfbStepDuration(int module);
void SetFAIMSfbSteps(int module, int Steps);
void ReturnFAIMSfbSteps(int module);
void InitFAIMSfbScan(int module);
void StopFAIMSfbScan(int module);
void SetFAIMSfbVrfNow(char *module, char *Vrf);
void SetFAIMSfbVrfTable(char *module, char *Vrf);
void GenerateVrfTable(int module);
void FAIMSfbNumberOfChannels(void);

void SetEMRTenable(char *ena);
void ReturnEMRTenable(void);
void SetEMRTM4enable(char *ena);
void ReturnEMRTM4enable(void);
void ReturnEMRTpos(void);
void ReturnEMRTneg(void);
void SetEMRTposOff(char *val);
void ReturnEMRTposOff(void);
void SetEMRTnegOff(char *val);
void ReturnEMRTnegOff(void);
void SetEMRTposZero(char *val);
void ReturnEMRTposZero(void);
void SetEMRTnegZero(char *val);
void ReturnEMRTnegZero(void);
void SetEMRTzero(void);

#endif
