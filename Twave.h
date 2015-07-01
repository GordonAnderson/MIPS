#ifndef Twave_h
#define Twave_h

extern float MaxTwaveVoltage;

extern DialogBox TwaveDialog;
extern DialogBox TwaveDialog2;
extern DialogBox TwaveCalMenu;

typedef struct
{
  float  VoltageSetpoint;      // Twave channel setpoint voltage
  // Hardware specific definitions
  DACchan  DCctrl;             // Twave DAC to control output voltage
  ADCchan  DCmon;              // Twave ADC channel used for monitor
} TwaveChannellData;

typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name
  int8_t  Rev;               // Holds the board revision number
  int     Velocity;
  uint8_t Sequence;
  // TWI device addresses
  uint8_t DACadr;
  uint8_t ADCadr;
  uint8_t GPIOadr;
  uint8_t CLOCKadr;
  // DAC/ADC channel data data
  TwaveChannellData  TWCD[4]; // 4 total channels, Pulse, Resting, Guard 1, Guard 2
  // The following paraneters were added for rev 2.0 and above
  bool    Direction;          // Twave direction, forward or reverse, true = forward
} TwaveData;

extern TwaveData TD;

// Prototypes
void SetPulseVoltage(void);
void SetVelocity(void);
void SetSequence(void);
void SetRestingVoltage(void);
void SetGuard1(void);
void SetGuard2(void);
void Twave_init(void);
void Twave_loop(void);

// Serial command prototypes
void TWAVEnumberOfChannels(void);
void sendTWAVEsequence(void);
void setTWAVEfrequency(int freq);
void setTWAVEsequence(char *value);
void setTWAVEpulseVoltage(char *voltage);
void setTWAVEguard1Voltage(char *voltage);
void setTWAVEguard2Voltage(char *voltage);
void getTWAVEdir(void);
void setTWAVEdir(char *dirstr);

#endif


