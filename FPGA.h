#ifndef FPGA_h
#define FPGA_h

#define FPGAbaseADD 0     // SPI base address on FGPA, must be first byte send
#define FPGAspiADD  3     // SPI address line selection bits, 0 thru 7

typedef struct
{
  uint8_t   baseAdd;
  uint8_t   boardAdd;
  uint16_t  cfg;
  uint8_t   ADCmask;
  uint8_t   DACmask;
  uint16_t  m[8];
  uint16_t  b[8];
  uint16_t  adc[8];
  uint16_t  dac[8];
} FPGAdata;

// One struct for each ARB board
typedef struct
{
  int16_t       Size;              // This data structures size in bytes
  char          Name[20];          // Holds the board name, "FPGA"
  int8_t        Rev;               // Holds the board revision number
  int8_t        TWIadd;            // TWI address of the eeprom on this module
  ADCchan       adc[8];            // ADC input calibration parameters
  FPGAdata      fpga;
  int           DCBfirst;          // DC bias module first channel, 1, or 9.
} FPGAmodule;

// Starting address offset to registers in FPGA
#define STARTcfg    0
#define STARTadcMSK 2
#define STARTdacMSK 3
#define STARTm      4
#define STARTb      20
#define STARTadc    36
#define STARTdac    52

enum FPGA_array {
  FPGA_M    = STARTm,
  FPGA_B    = STARTb,
  FPGA_ADC  = STARTadc,
  FPGA_DAC  = STARTdac
};

// Note; ADC values are converted to DAC values using the fopllowing equasion:
// dac = (adc * m)/1024 + b

extern FPGAdata  *fpga;

// Configuration word bits locations
#define BUSCTRL     0                   // FPGA control module bus if 1
#define RUNCAL      1                   // Runs the calculation state machine if 1
#define RUNADC      2                   // Enable ADC updates if 1
#define LDACbit     8                   
#define BRDSELbit   9
#define ADDR0bit    10
#define ADDR1bit    11
#define ADDR2bit    12
   
#define setBUSCTRL(var,val) {var &= ~(0x01 << BUSCTRL); var |= ((val & 0x01) << BUSCTRL);}
#define setRUNCAL(var,val)  {var &= ~(0x01 << RUNCAL); var |= ((val & 0x01) << RUNCAL);}
#define setRUNADC(var,val)  {var &= ~(0x01 << RUNADC); var |= ((val & 0x01) << RUNADC);}
#define setLDAC(var,val)    {var &= ~(0x01 << LDACbit); var |= ((val & 0x01) << LDACbit);}
#define setBUSSEL(var,val)  {var &= ~(0x01 << BRDSELbit); var |= ((val & 0x01) << BRDSELbit);}
#define setADDR(var,val)    {var &= ~(0x07 << ADDR0bit); var |= ((val & 0x07) << ADDR0bit);}

#define writeFPGAcfg  writeFPGA(fpga, STARTcfg, 2, (uint8_t *)&fpga->cfg);
#define readFPGAcfg   readFPGA(fpga, STARTcfg, 2, (uint8_t *)&fpga->cfg);

#define writeFPGAadcMSK  writeFPGA(fpga, STARTadcMSK, 1, (uint8_t *)&fpga->ADCmask);
#define readFPGAadcMSK   readFPGA(fpga, STARTadcMSK, 1, (uint8_t *)&fpga->ADCmask);

#define writeFPGAdacMSK  writeFPGA(fpga, STARTdacMSK, 1, (uint8_t *)&fpga->DACmask);
#define readFPGAdacMSK   readFPGA(fpga, STARTdacMSK, 1, (uint8_t *)&fpga->DACmask);

#define writem  writeFPGA(fpga, STARTm, 16, (uint8_t *)&fpga->m);
#define readm   readFPGA(fpga, STARTm, 16, (uint8_t *)&fpga->m);

#define writeb  writeFPGA(fpga, STARTb, 16, (uint8_t *)&fpga->b);
#define readb   readFPGA(fpga, STARTb, 16, (uint8_t *)&fpga->b);

#define writeADC  writeFPGA(fpga, STARTadc, 16, (uint8_t *)&fpga->adc);
#define readADC   readFPGA(fpga, STARTadc, 16, (uint8_t *)&fpga->adc);

#define writeDAC  writeFPGA(fpga, STARTdac, 16, (uint8_t *)&fpga->dac);
#define readDAC   readFPGA(fpga, STARTdac, 16, (uint8_t *)&fpga->dac);

// Prototypes
void FPGAinit(void);
void readFPGAdacDisplay(void);

void writeFPGA(FPGAdata *fd, int offset, int numBytes, uint8_t *ptr);
void readFPGA(FPGAdata *fd, int offset, int numBytes, uint8_t *ptr);

#endif