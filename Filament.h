#ifndef FILAMENT_H_
#define FILAMENT_H_

extern bool FLserialWD;


enum FilamentModes
{
  FmodeI,                   // Current control mode
  FmodeIV                   // Current control with voltage adjustment to
                            // reduce power disapation in MOSFET
};

enum FilamentCyclingStates
{
  FC_START,               // Cycling started
  FC_WAIT_PT1,            // Waiting to reach point 1
  FC_WAIT_PT2,            // Waiting to reach point 2
  FC_STOP                 // Cyclying stopped
};

typedef struct
{
  float    CurrentSetpoint;    // Current setpoint
  float    FilamentVoltage;    // Filament voltage setpoint
  bool     FilamentPwr;        // Power supply on/off state, true = on
  float    RampRate;           // Current rampe rate in amps per second.
  int      Mode;               // Filament controller mode of operation
  float    MaxPower;           // Maximum filament power limit
  // Hardware specific definitions
  int      Fpwr;               // Filament power on/off pin number
  DACchan  DCfsuply;           // Filament supply voltage control
  DACchan  Fcurrent;           // Filament current
  ADCchan  DCfsuplyMon;        // Filament supply voltage monitor
  ADCchan  Fvoltage;           // Filament voltage
  ADCchan  FcurrentMon;        // Filament current monitor
} FilamentChannel;

typedef struct
{
  bool    FCenable;            // Enable the cycling mode
  float   FCpoint1;            // Current point 1
  float   FCpoint2;            // Current point 2
  int     FCnumCyl;            // Number of cycles
} FilamentCycling;

// One struct for each Filament board
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "Filamanet"
  int8_t  Rev;               // Holds the board revision number
  int8_t  NumChannels;       // Number of channels suppoted by this board, normally 2
  FilamentChannel FCD[2];    // Two filament channels per board
  // TWI address data
  uint8_t ADCadr;            // 8 channel ADC used for readback
  uint8_t DACadr;            // 4 channel DAC, channel used for output control
  uint8_t EEPROMadr;
  FilamentCycling FCyl[2];   // Data structures supporting the cycling functionnt
  int     iSense;            // Bias current sense resistor value. If non-zero then channel 8 of DCbias board is used for 
                             // current monitoring.
  // Added the current direction flag for the firmware rev 2
  bool    Idir;              // Current direction
  ADCchan Ecurrent;          // Emission current adc channel, requires rev 4 module
} FilamentData;

extern FilamentData  FDarray[2];

// Function prototypes
void FilamentChannels(void);
bool IsFilamentChannelValid(int channel, bool Response);
void GetFilamentEnable(int channel);
void SetFilamentEnable(char *Chan, char *State);
void GetFilamentCurrent(int channel);
void GetFilamentActualCurrent(int channel);
void SetFilamentCurrent(char *Chan, char *Current);
void GetFilamentSupplyVoltage(int channel);
void SetFilamentSupplyVoltage(char *Chan, char *Voltage);
void GetFilamentActualSupplyVoltage(int channel);
void GetFilamentVoltage(int channel);
void GetFilamentPower(int channel);
void GetCurrentRampRate(int channel);
void SetCurrentRampRate(char *chan, char *RampRate);
void ResetFilamentSerialWD(void);

void GetFilamentCycleCurrent1(int channel);
void SetFilamentCycleCurrent1(char *chan, char *current);
void GetFilamentCycleCurrent2(int channel);
void SetFilamentCycleCurrent2(char *chan, char *current);
void GetFilamentCycleCount(int channel);
void SetFilamentCycleCount(char *chan, char *count);
void GetFilamentStatus(int channel);
void SetFilamentStatus(char *chan, char *Status);
void GetCerrentDirection(int channel);
void SetCurrentDirection(char *chan, char *dir);
void FilamentShutdown(void);

void SetFilamentReporting(int channel, int period);
void ReportBiasCurrent(void);

void SaveFIL2EEPROM(void);

#endif






