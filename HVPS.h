#ifndef HVPS_h
#define HVPS_h

typedef struct
{
  bool          Enable;
  int           Voltage;
} HVPSchState;

typedef struct
{
  bool          Update;
  HVPSchState   HVPSCH[2];
} HVPSstate;

// Channel data structure
typedef struct
{
  bool          Enable;
  int           Voltage;
  DACchan       DCPena ;               // DAC to control output voltage enable, positive supply
  DACchan       DCPctrl;               // DAC to control output voltage, positive supply
  ADCchan       DCPmon;                // ADC channel used for monitor, positive supply
  DACchan       DCNena ;               // DAC to control output voltage enable, negative supply
  DACchan       DCNctrl;               // DAC to control output voltage, negative supply
  ADCchan       DCNmon;                // ADC channel used for monitor, negative supply
  // Limits and config data
  int           PosSupplyV;            // Positive power supply voltage
  int           NegSupplyV;            // Negative power supply voltage
  int           PosLimit;              // Max positive voltage limit
  int           NegLimit;              // Max negative voltage limit         
} HVPSchannel;

// High Voltage Power Supply control data structure
typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "HVPS"
  int8_t        Rev;                    // Holds the board revision number
  int8_t        TWIadd;                 // Modules base address set by jumpers
  int8_t        TWIdac;                 // Control DAC TWWI address
  int8_t        TWIadc;                 // 4 channel ADC for readbacks
  int8_t        NumChannels;            // Number of active channels, 1 or 2
  HVPSchannel   HVPSCH[2];              // Channel control structure
} HVPSdata;

extern HVPSdata HVPS_Rev_1;
extern HVPSdata HVPS_Rev_2;
extern float    MaxHVvoltage;

// Prototypes
void HVPSNumberOfChannels(void);

#endif
