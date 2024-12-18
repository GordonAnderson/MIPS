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
#define RFAcpldRANGE          3       // Used to gain range of the RF level detectors
#define RFAcpldFSELECT        4       // DDS control line
#define RFAcpldPSEL0          5       // DDS control line 
#define RFAcpldPSEL1          6       // DDS control line
#define RFAcpldDRVSP_SELECT   7       // Selects open or closed loop
#define RFAcpldINVERT         8       // Inverts the loop control

// CPLD control bits
#define RFAstrobe  49                 // Pin 14 on EXT1 connector
                                      // Pin 11 on EXT1 connectes to CPLD pin 30,
                                      // called CPLD_CLEA but not used

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

// One struct for each RF amp board
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
  // Configuration parameters
  float   FullScale;         // Full scale voltage Vp-p
  float   Ro;                // Quad radius parameter, in mm
  float   mz;                // Target m/z value
  float   ResolvingDC;       // Resolving DC voltage
  float   PoleBias;          // Pole bias
  float   Res;               // Resolution in AMU
  // Low range parameters
  float   RangeThreshold;    // Threshold to apply low range cal
  ADCchan ADCchansLR[2];
  DACchan DACchansLR;      
  float   K;                 // Factor used to calculate DC from Quad Vp-p,  
  // Resolving DC bias channel number
  int     DCBchan;
  // Auto range enable
  bool    AutoRangeEna;
  // Gain compensation parameters
  bool    EnableComp;
  int     CalFrequency;
  float   FreqComp;
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
  float   ResolvingDC;
  float   PoleBias;
  bool    Mode;
  bool    Invert;
  bool    ResolvingDCenable;
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

extern      RFAdata  *RFAarray[2];
extern int  NumberOfRFAchannels;
extern bool ResolvingDCenable; 

// Prototypes
void RFA_init(int8_t Board, int8_t addr);
void RFAsetRange(char *Module, char *value);
void RFAgetRange(int Module);
void ReportRFAchannels(void);
void RFAsetPoleBias(char *Module, char *value);
void RFAgetPoleBias(int Module);
void RFAsetResolvingDC(char *Module, char *value);
void RFAgetResolvingDC(int Module);
void RFAsetRo(char *Module, char *value);
void RFAgetRo(int Module);
void RFAsetMZ(char *Module, char *value);
void RFAgetMZ(int Module);
void RFAsetRes(char *Module, char *value);
void RFAgetRes(int Module);
void RFAupdateQUAD(int Module);
void RFAsetGain(char *Module, char *value);
void RFAreturnGain(int Module);
void RFampNumber(void);

void SetRangeHigh(int brd, bool Send = true);
void SetRangeLow(int brd, bool Send = true);

void RFAsetFreq(int module, int freq);
void RFAgetFreq(int module);
void RFAgetPWR(int module);
void RFAsetMode(char *module, char *state);
void RFAgetMode(int module);
void RFAsetDrive(char *module, char *drive);
void RFAgetDrive(int module);
void RFAsetLevel(char *module, char *level);
void RFAgetLevel(int module);
void RFAgetVPPA(int module);
void RFAgetVPPB(int module);
void RFAreport(int module);
void RFAsetENA(char *Module, char *value);
void RFAgetENA(int Module);
void RFAsetK(char *Module, char *value);
void RFAgetK(int Module);
void RFAsetDCBchan(int Module, int value);
void RFAgetDCBchan(int Module);
void RFsetDrvZero(char *Module, char *value);

int RFAmodule2board(int Module);

void RFAsetAutoRange(char *Module, char *value);
void RFAgetAutoRange(int module);

void RFAsetFreqComp(char *Module, char *value);
void RFAgetFreqComp(int Module);
void RFAsetCalFreq(int Module, int Freq);
void RFAgetCalFreq(int Module);
void RFAsetCompG(char *Module, char *Comp);
void RFAgetCompG(int Module);

void quadAutoTune(int brd, bool report = true);
void setQuadAT(int module);
void setQuadATR(int module);

#endif
