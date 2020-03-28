#ifndef HOFAIMS_H_
#define HOFAIMS_H_

// IO pins used my the HOFAIMS modules
#define   Servo1  16
#define   Servo2  2

#define   ServoMaxTimerCount  32000
#define   ServoTimePerCount   (656250.0/1000000.0)

// High order FAIMS data structure. This structure is saved in EEPROM on the module
// and contains all the calibration and control parameters.
typedef struct
{
  int16_t       Size;              // This data structures size in bytes
  char          Name[20];          // Holds the board module, "HOFAINS"
  int8_t        Rev;               // Holds the board revision number
  bool          Enable;            // Turns the HIFAIMS module on and off
  float         Drive;             // Drive level for the coil in percent
  float         Cap;               // Tuning capacitor position 0 to 100 percent
  int           Delay;             // Delay line value, 0 to 255
  float         MaxPower;
  float         MaxDrive;  
  // ADC channels
  ADCchan  Vmon, Imon, RFmon;
  // TWI device addresses
  uint8_t       EEPROMadr;         // On board memory address
  uint8_t       ADCadr;            // 4 channel ADC used for monitoring
  // SPI addresses
  uint8_t       DELAYspi;          // SPI address of the delay line
} HOFAIMSdata;

// Prototypes
void HOFAIMS_init(int8_t Board, int8_t addr);
void HOFAIMS_loop(void);

#endif
