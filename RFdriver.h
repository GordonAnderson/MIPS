#ifndef RFDRIVER_H_
#define RFDRIVER_H_

extern float MaxRFVoltage;

enum RFdriverMode
{
  RF_MANUAL,
  RF_AUTO,
};

typedef struct
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
typedef struct
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
} RFdriverData;

#endif



