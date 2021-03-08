/*

FAIMS user adjustable parameters

Frequency
Corse phase
Fine phase
Enable Drv1
Enable Drv2
Enable Drv3
Drv1 level
Drv2 level
Drv3 level
Primary capacitance
Harmonic capacitance

FAIMS monitored values

Primary RF
RF pos
RF neg
Drv1 voltage
Drv1 current
Drv2 voltage
Drv2 current
Drv3 voltage
Drv3 current

*/

#ifndef FAIMS_h
#define FAIMS_h

extern float MaxFAIMSVoltage;
extern bool  DiableArcDetect;

extern float TotalPower;
extern float KVoutP;
extern float KVoutN;

extern float  DCoffsetRB;
extern float  DCbiasRB;
extern float  DCcvRB;

extern bool   FAIMSscan;
extern bool   FAIMSstepScan;

extern bool   Lock;
extern float  LockSetpoint;
extern int    Loops;


// Drive data structure
typedef struct
{
  int8_t   PWMchan;             // Define the PWM output channel
  float    Drv;                 // Drive level in percentage
  bool     Enable;
  float    MaxPower;
  ADCchan  Vmon;
  ADCchan  Imon;
} Drive;

// FAIMS data structure. This data is saved in EEPROM
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "FAIMS"
  int8_t  Rev;               // Holds the board revision number
  int     Freq;              // Primary drive frequency
  float   Drv;               // Overall drive level
  bool    Enable;            // Overall enable
  int     PhaseC;            // Coarse phase control
  int     PhaseF;            // Fine phase control
  float   Pcap;              // Primary capacitor position
  float   Hcap;              // Harmonic capacitor position
  float   MaxPower;
  float   MaxDrive;
  float   MaxOnTime;         // Defines the number of hours before the system automatically shuts down 
  DCbiasChannellData  DCoffset,DCbias,DCcv;
  float   CVstart;
  float   CVend;
  float   Duration;          // Scan time in seconds
  // Coil driver data structures
  Drive  Drv1,Drv2,Drv3;
  // RF voltage monitors
  ADCchan  RFpri, RFharP, RFharN;
  // TWI device addresses
  uint8_t EEPROMadr;
  uint8_t ADC1adr;
  uint8_t ADC2adr;
  uint8_t DACadr;
  uint8_t CLOCKadr;
  uint8_t GPIOadr;
  uint8_t DELAYadr;
  uint8_t PWMadr;
  // Press and temp correction parameters
  bool   Compensation;         // Flag that turns on and off the compensation for pressure and temp
  float  PressureCoeff;        // Pressure coefficent
  float  TempCoeff;            // Temp coefficent
  float  PressTempLimit;       // Pressure & Temp adjustment limit in % of drive
  // Loop scan value
  int    Loops;
  // Arc detector sensitivity
  float  ArcSens;
  // Step based scanning
  int    Steps;
  int    StepDuration;        // in mSec
  // External scan trigger options
  char          ScanTrigger;  // Trigger input channel
  int8_t        TriggerLevel; // Trigger level, 0,CHANGE,RISING, or FALLING
} FAIMSdata;

extern FAIMSdata  faims;

// FAIMS Auto tune parameters

// Auto tune data types
enum FAIMSdir
{
  FAIMSup,
  FAIMSdown,
  FAIMSdone
};

enum FAIMSautoTuneStates
{
  FAIMSidle,
  FAIMSexit,
  FAIMSabort,
  FAIMSenable,
  FAIMSfindFreq,
  FAIMSfindFreqFine,
  FAIMSfindPcap,
  FAIMSfindHcap,
  FAIMSsetPhase,
  FAIMStunePhase,
  FAIMSsetHarmonic
};

extern char TuneState[];

// Prototypes
void SaveFAIMSSettings(void);
void RestoreFAIMSSettings(void);
void CalibrateDCbias(void);
void CalibrateDCcv(void);
void CalibrateDCoffset(void);
bool FAIMSstartAutoTune(void);

// Serial command prototypes
void FAIMSnumberOfChannels(void);
void FAIMSsetDrive(char *drv);
void FAIMSsetCV(char *volts);
void FAIMSsetBIAS(char *volts);
void FAIMSsetOffset(char *volts);
void FAIMSsetRFharPcal(char *m, char *b);
void FAIMSsetRFharNcal(char *m, char *b);

void SaveFAIMS2EEPROM(void);

void FAIMSsetCVstart(char *volts);
void FAIMSsetCVend(char *volts);
void FAIMSsetDuration(char *secs);
void FAIMSsetLoops(char *count);
void FAIMSsetStepTime(char *msec);
void FAIMSsetSteps(char *count);

void FAIMSsetLock(char *state);
void FAIMSsetLockSP(char *KV);

void FAIMSrequestAutoTune(void);
void FAIMSautoTuneAbort(void);

#endif
