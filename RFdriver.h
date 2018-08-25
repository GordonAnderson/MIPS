#ifndef RFDRIVER_H_
#define RFDRIVER_H_

extern float MaxRFVoltage;
extern int  NumberOfRFChannels;

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
  // These parameters really below in RFchnnelData but there were added after release so placing at end of struct give backwards compatability
  char   RFgateDI[2];       // Gate input, 0 if not used otherwise its the input P-X
  int8_t RFgateTrig[2];     // Gate level, 0,CHANGE,RISING, or FALLING
} RFdriverData;

// Prototypes
void RF_A1_ISR(void);
void RF_A2_ISR(void);
void RF_B1_ISR(void);
void RF_B2_ISR(void);

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

void RFmodeReport(int);
void RFmodeSet(char *, char *);
void SaveRF2EEPROM(void);

#endif









