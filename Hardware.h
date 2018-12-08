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
float ReadVin(void);
void  SetOutput(char chan, int8_t active);
void  ClearOutput(char chan, int8_t active);
void  DigitalOut(int8_t MSB, int8_t LSB);
uint8_t DigitalIn(void);
int  ReadEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count);
int  WriteEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count);
void TWI_RESET(void);
void TWI_START(void);
void TWI_STOP(void);
bool TWI_WRITE(int8_t val);
int8_t TWI_READ(bool Reply);
int  AD7998(int8_t adr, uint16_t *vals);
int  AD7994(int8_t adr, uint16_t *vals);
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
int SelectedBoard(void);

bool AcquireTWI(void);
void ReleaseTWI(void);
void TWIqueue(void (*TWIfunction)());

void TriggerOut(char *cmd);
void TriggerOut(int microSec, bool WithLimits = false);
void AuxOut(char *cmd);
void QueueTriggerOut(int microSec);
void ProcessTriggerOut(void);
void ADCread(int chan);
bool QueueTpulseFunction(void (*Tfunction)(), bool add);
void PlayTpulseFunctions(void);

void DIOopsReport(void);
void DIOreport(char *port, char *mode);
void DIOmirror(char *in, char *out);

void SetDelayTrigInput(char *input, char *level);
void SetDelayTrigEnable(char *sena);
void SetDelayTrigModule(char *module);

bool bmpDraw(char *filename, uint8_t x, uint8_t y);
void bmpReport(char *filename);

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

void TWIreset(void);

// SD file io commands
void ListFiles(void);
void DeleteFile(char *FileName);
void GetFile(char *FileName);
void PutFile(char * FileName,char *Fsize);
void EEPROMtoSD(void);
void SDtoEEPROM(void);
void SaveAlltoSD(void);
void LoadAllfromSD(void);
void EEPROMtoSerial(char *brd, char *add);
void SerialtoEEPROM(char *brd, char *add);

//void ShutterEnable(char *state);

#endif
