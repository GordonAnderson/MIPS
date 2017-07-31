#ifndef RFAMP_H_
#define RFAMP_H_
//
// The RFamp moudule is desiged to control the 40 watt linear RF amplifier. This modules gererates the
// reference oscillator and monitors signals generated by the RF amplifier. These signals monitor RF
// and DC voltages and currents as well as the temp of the drive transistors.
//
// The RFamp modules used a XLINK CPLD to interface to the DDS chip as well as control logic on the 
// module. This module also contains the PID loop feedback to support closed loop RF level control.
//

// CPLD control byte bit definitions
#define RFAcpldERROR          0       // Error LED, active high
#define RFAcpldSTATUS         1       // Status LED, active high
#define RFAcpldCLOSED         2       // Clossed LED, active high. on when in clossed loop mode
#define RFAcpldBLNK           3       // Used to apply a hold on RF level detectors
#define RFAcpldFSELECT        4       // DDS control line
#define RFAcpldPSEL0          5       // DDS control line 
#define RFAcpldPSEL1          6       // DDS control line
#define RFAcpldDRVSP_SELECT   7       // Selects open or closed loop
#define RFAcpldINVERT         8       // Inverts the loop control

// CPLD control bits
#define RFAstrobe  49

// ADC input channels
#define RFAadcDCV             0
#define RFAadcDCI             1
#define RFAadcTEMP            2
#define RFAadcRFV             3
#define RFAadcRFI             4
#define RFAadcSWR             5
#define RFAadcRFLA            6
#define RFAadcRFLB            7

// DAC output channels
#define RFAdacDRIVE           0
#define RFAdacSETPOINT        1
#define RFAdacAUX1            2
#define RFAdacAUX2            3

// One struct for each DC bias board
typedef struct
{
  int16_t Size;              // This data structures size in bytes
  char    Name[20];          // Holds the board name, "DCbias"
  int8_t  Rev;               // Holds the board revision number
  bool    Enabled;           // True if enabled
  int     Freq;
  float   Drive;
  float   SetPoint;
  bool    Mode;              // True = open loop mode
  bool    Invert;
  // SPI device addresses
  int8_t  CPLDspi;           // SPI address for CPLD controller
  int8_t  DDSspi;            // SPI address for DDS ref osc generator
  // TWI device addresses
  uint8_t EEPROMadr;
  uint8_t DACadr;
  uint8_t ADCadr;
  DACchan DACchans[4];       // 4 channels of DAC
  ADCchan ADCchans[8];       // 8 channels of ADC
} RFAdata;                    

// This structure saves the current state of the RFamp module
// and the calculated values used to display various RFamp 
// parameters.
typedef struct
{
  // Current state values
  bool    Enabled;
  int     Freq;
  float   Drive;
  float   SetPoint;
  bool    Mode;
  bool    Invert;
  // Misc
  uint16_t CPLDimage;
  // Measured and calculated monitor values
  float   DCV;
  float   DCI;
  float   DCpower;
  float   Temp;
  float   RFV;
  float   RFI;
  float   RFpowerFWD;
  float   RFpowerREF;
  float   ph;
  float   SWR;
  float   RFVPpp;             // Monitored RF+ Vp-p
  float   RFVNpp;             // Monitored RF- Vp-p
} RFAstate;

extern int   NumberOfRFAchannels;

// Prototypes
void ReportRFAchannels(void);

#endif

