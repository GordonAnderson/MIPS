#ifndef FILAMENT_H_
#define FILAMENT_H_

enum FilamentModes
{
  FmodeI,                   // Current control mode
  FmodeIV                   // Current control with voltage adjustment to
                            // reduce power disapation in MOSFET
};

typedef struct
{
  float    CurrentSetpoint;    // Current setpoint
  float    FilamentVoltage;    // Filament voltage setpoint
  bool     FilammentPwr;       // Power supply on/off state, true = on
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
} FilamentData;

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

#endif

