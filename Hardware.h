//
// Hardware definitions for the MIPS controller
//
#ifndef Hardware_h
#define Hardware_h

#include <DIhandler.h>

extern int PulseWidth;
extern int PulseFreq;
extern int BurstCount;

// Delayed trigger capability variables
extern int  DtrigDelay;
extern int  DtrigPeriod;
extern int  DtrigNumber;
extern int  DtrigCurrentNum;
extern bool DtrigEnable;
extern bool TWIbusy;


typedef struct
{
  int8_t  Chan;              // ADC channel number 0 through max channels for chip
  float   m;                 // Calibration parameters to convert channel to engineering units
  float   b;                 // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;              // DAC channel number 0 through max channels for chip
  float   m;                 // Calibration parameters to convert engineering to DAC counts
  float   b;                 // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Data structure used by the calibration function.
typedef struct
{
  float    Min;
  float    Max;
  int8_t   DACaddr;           // DAC device address, SPI if < 8;
  int8_t   ADCaddr;           // ADC device address
  DACchan  *DACout;           // DAC output channel to calibrate
  ADCchan  *ADCreadback;      // ADC readback to calibrate, NULL if not used
  int(*ADCpointer)(int8_t, int8_t, int8_t);
} ChannelCal;

// The following are used for the Push Button LED control.
// A task ran every helf second is used to control the
// LEDs and supports on, flash and off.

enum PBledStates
{
  ON,
  OFF,
  FLASH,
  NOTHING
};

// This structure supports DIO special operations. Reporting of input
// changes and mirroring of inputs to outputs. An array of 8 of these
// structures supports each digital input.
typedef struct
{
  int    DI;           // Digital input pin number
  int    LastLevel;    // Inputs last level
  bool   Changed;      // True if state change detected
  bool   Report;       // True if reporting is selected
  int    ReportState;  // RISING, FALLING, or CHANGE report flag
  bool   Mirror;       // True if this input is mirrored to an output
  char   DO;           // Digital output, i.e. A
  DIhandler *DIh;      // Interrupt handler
} DIOops;

extern DIOops  dioops[8];
extern int PBled;
extern PBledStates  PBledMode;

#define PB_ON(a)  PBled=a; PBledMode = ON
#define PB_OFF(a)  PBled=a; PBledMode = OFF
#define PB_FLASH(a)  PBled=a; PBledMode = FLASH

// These are the pins used for the DUE hardware SPI
#define _sclk 76
#define _miso 74
#define _mosi 75
#define _cs 4
#define _rst 33
#define _dc 34

// microSD chip select pin
#define _sdcs 10

// Rotary encoder hardware pins and LED pins
#define PB_RED       31
#define PB_GREEN     24
#define PB_BLUE      23
#define PB_PHASE_A   29
#define PB_PHASE_B   22
#define PB           32

#define PB_GREEN_ON   digitalWrite(PB_GREEN,LOW)
#define PB_GREEN_OFF  digitalWrite(PB_GREEN,HIGH)
#define PB_RED_ON     digitalWrite(PB_RED,LOW)
#define PB_RED_OFF    digitalWrite(PB_RED,HIGH)
#define PB_BLUE_ON    digitalWrite(PB_BLUE,LOW)
#define PB_BLUE_OFF   digitalWrite(PB_BLUE,HIGH)

// Red LED on the PC board
//#define RED_LED      19
//#define RED_LED_ON    digitalWrite(RED_LED,LOW)
//#define RED_LED_OFF   digitalWrite(RED_LED,HIGH)

// Misc signals
#define BACKLIGHT     3     // TFT display backlight control, pwm to control level
#define LDAC          11
#define TRIGGER       12
#define LDACctrl      A11
#define ADDR0         25
#define ADDR1         26
#define ADDR2         27
#define SPI_CS        52
#define SCL           45    // Shift register clear
#define DOenable      44    // Digital output buffer enable, active low
#define DOMSBlatch    DAC0  // Digital output MSB latch
#define BRDSEL        47
#define TWI_SCL       21
#define TWI_SDA       20
#define PWR_ON        15    // Enables power supply on high voltage DC bias boards,
                            // low = on
#define TRGOUT        A10   // Trigger output, this signal is inverted on controller
#define AUXTRGOUT     A9    // Aux Trigger output, this signal is inverted on controller
#define RFON          9     // Used by FAIMS to turn on RF on LED
#define POWER         A2    // Power enable

#define  ENA_BRD_A    digitalWrite(BRDSEL,HIGH)
#define  ENA_BRD_B    digitalWrite(BRDSEL,LOW)

// Digitial input pins, Q through X
#define  DI0          30
#define  DI1          12
#define  DI2          50
#define  DI3          51
#define  DI4          53
#define  DI5          28
#define  DI6          46
#define  DI7          17

// Macros
#define LDAClow        digitalWrite(LDAC,LOW)
#define LDAChigh       digitalWrite(LDAC,HIGH)
#define LDACctrlLow    digitalWrite(LDACctrl,LOW)
#define LDACctrlHigh   digitalWrite(LDACctrl,HIGH)
#define LDACrelease    {pinMode(LDACctrl,INPUT);}
#define LDACcapture    {pinMode(LDACctrl,OUTPUT); digitalWrite(LDACctrl,HIGH);}
#define PulseLDAC      {LDAChigh; LDAClow;}
#define RFON_OFF       digitalWrite(RFON,LOW);
#define RFON_ON        digitalWrite(RFON,HIGH);
#define SetTRGOUT      digitalWrite(TRGOUT, LOW);
#define ResetTRGOUT    digitalWrite(TRGOUT, HIGH);

// DMA definitions used for DAC spi DMA to speed up the DAC output process
/** Use SAM3X DMAC if nonzero */
#define USE_SAM3X_DMAC 1
/** Use extra Bus Matrix arbitration fix if nonzero */
#define USE_SAM3X_BUS_MATRIX_FIX 0
/** Time in ms for DMA receive timeout */
#define SAM3X_DMA_TIMEOUT 100
/** chip select register number */
#define SPI_CHIP_SEL 3
/** DMAC receive channel */
#define SPI_DMAC_RX_CH  1
/** DMAC transmit channel */
#define SPI_DMAC_TX_CH  0
/** DMAC Channel HW Interface Number for SPI TX. */
#define SPI_TX_IDX  1
/** DMAC Channel HW Interface Number for SPI RX. */
#define SPI_RX_IDX  2

void spiDMAinit(void);
void spiDmaTX(uint32_t* src, uint16_t count,void (*isr)() = NULL);
void spiDmaWait(void);

// Level detection module constants. This module can be installed in MIPS using teh aux connector
// on the MIPS controller. There are a number of places where this module is used in the MIPS firmware.
// The level detection module accepts an analog input and has a number of configurable modes. 
// Examples:
//        DCbias module to use offset or output voltage values, allows detection of changed
//        input value and adjustment of outputs.
//        Timing generation, changes can be used with the level detectors internal look up table
//        to select a table number and then trigger the timing generator
//        ARB delay adjust, changes can be used with the level detectors internal look up table
//        to send a delay value to the selected ARB module. This delay is used in the edge detection
//        operation.
// TWI commands and constants
#define TWI_LEVDET_SET_ADCMODE     0x01      // Set ADC mode, 0 thru 5, byte
#define TWI_LEVDET_SET_LL          0x02      // Set LowerLimit, in volts, float
#define TWI_LEVDET_SET_UL          0x03      // Set UpperLimit, in volts, float
#define TWI_LEVDET_SET_THRES       0x04      // Set ADC threshold for detection filter, ADC counts, byte
#define TWI_LEVDET_SET_ZP          0x05      // Set the zero point in volts, 0 to 3, float
#define TWI_LEVDET_SET_TMODE       0x06      // Set the Table mode, 0= off, 1 = use raw ADC counts, 2 = use calibrated ADC value

#define TWI_LEVDET_SERIAL          0x27      // This command enables the TWI port to process serial commands
#define TWI_LEVDET_CMD             0x7F      // This command sends a ascii string to the serial command processor
                                              // and returns the response through the TWI interface

#define TWI_LEVDET_READ_ADCRAW     0x81      // Returns ADC raw counts, word
#define TWI_LEVDET_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_LEVDET_READ_ADC        0x83      // Returns ADC voltage, in volts, float
#define TWI_LEVDET_LOOKUP          0x84      // Use current ADC value to lookup response in the lookup table

// Hardware interface signals
#define LEVCHANGE                  13        // MIPS input pin used to detect change signal from the level module
#define SCL1                       18        // Pin used for wire1 SCL signal
#define SDA1                       19        // Pin used for wire1 SDA signal

extern  int  LevelDetAdd;                    // Level detection module TWI address, used Wire1 interface

// Prototypes
void  GenerateBurst(int num);
void  QueueBurst(int num);
void  ProcessBurst(void);
void  DefineDeviceAddress(char *board, char *addr);
void  ReportAD7998(int chan);
void  ReportAD7994(int chan);
void  Init_IOpins(void);
void  Reset_IOpins(void);
void  Software_Reset(void);
void  RebootStatus(void);
int   ReadInput(char inputCH);
void  SetOutput(char chan, int8_t active);
void  ClearOutput(char chan, int8_t active);
void  DigitalOut(int8_t MSB, int8_t LSB);
void  ClearDOshiftRegs(void);
uint8_t DigitalIn(void);
int  ReadEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count);
int  WriteEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count);
int  AD7998(int8_t adr, uint16_t *vals);
int  AD7998(int8_t adr, int8_t chan, int8_t num);
int  AD7994(int8_t adr, int8_t chan);
int  AD7994(int8_t adr, uint16_t *vals);
int  AD7994(int8_t adr, int8_t chan, int8_t num);
int  AD5625(int8_t adr, uint8_t chan, uint16_t val);
int  AD5625(int8_t adr, uint8_t chan, uint16_t val, int8_t Cmd);
int  AD5625_EnableRef(int8_t adr);
void AD5668(int8_t spiAdr, int8_t DACchan, uint16_t vali);
void AD5668(int8_t spiAdr, int8_t DACchan, uint16_t vali, int8_t Cmd);
int  MCP2300(int8_t adr, uint8_t bits);
int  MCP2300(int8_t adr, uint8_t reg, uint8_t bits);
int  MCP2300(int8_t adr, uint8_t reg, uint8_t *data);
void SetAddress(int8_t addr);
void SelectBoard(int8_t Board);
int  SelectedBoard(void);
void ChannelCalibrate(ChannelCal *CC, char *Name);
void ChannelCalibrate(ChannelCal *CC, char *Name, int ZeroPoint, int MidPoint);
bool ChannelCalibrateSerial(ChannelCal *CC, char *Message);
bool ChannelCalibrateSerial(ChannelCal *CC, char *Message, float ZeroPoint, float MidPoint);
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *AC);
int Value2Counts(float Value, DACchan *DC);
int Value2Counts(float Value, ADCchan *AC);
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val);
int AD5593readADC(int8_t addr, int8_t chan);
int AD5593readADC(int8_t addr, int8_t chan, int8_t num);
int AD5593writeDAC(int8_t addr, int8_t chan, int val);
int AD5629write(uint8_t addr, uint32_t val);

void SetDelayTrigInput(char *input, char *level);
void SetDelayTrigEnable(char *sena);
void SetDelayTrigModule(char *module);

void CPUtemp(void); 

extern int AuxTrigMax;
extern int TrigMax;

extern bool     Tracing;
extern uint8_t  TPpointer;
extern uint8_t  *TracePoints;
extern uint32_t *TracePointTimes;

#define TRACE(a) {if(Tracing) TraceCapture(a);}

void TraceCapture(uint8_t tp);
void TraceReport(void);
void TraceEnable(void);

void LevelDetChangeReport(void);
void SetLevelDetChangeReport(char *TWIadd);

#endif
