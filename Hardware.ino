//
// Hardware
//
//  This file contains low level hardware IO drivers. This includes analog and digitial IO devices.
//  The general calibration functions are in this file and these functions are used for all the 
//  calibration function. These functions all calibration of the outputs as well as the readbacks.
//
//
// Gordon Anderson
//
#include "Hardware.h"
#include "Variants.h"

int AuxTrigMax = 0;
int TrigMax    = 0;

int    PBled;
PBledStates  PBledMode = OFF;

extern Menu MainMenu;

int BoardSelect = 0;
int DeviceAddr = 0;

// This is used for the clock generation on the trigger output line
int PulseWidth = 5;
int PulseFreq  = 200;
int BurstCount = 100;
int CurrentCount;
bool BurstQueued = false;

DIOops  dioops[8] = {DI0,0,false,false,0,false,0,NULL,
                     DI1,0,false,false,0,false,0,NULL,
                     DI2,0,false,false,0,false,0,NULL,
                     DI3,0,false,false,0,false,0,NULL,
                     DI4,0,false,false,0,false,0,NULL,
                     DI5,0,false,false,0,false,0,NULL,
                     DI6,0,false,false,0,false,0,NULL,
                     DI7,0,false,false,0,false,0,NULL
                     };

MIPStimer FreqBurst(TMR_TrigOut);

void FreqBurstISR()
{
  static int lfreq = -1;

  if(lfreq == -1) lfreq = PulseFreq;
  else
  {
    if(lfreq != PulseFreq)
    {
      // Here is frequency has changed so update the timer.
      lfreq = PulseFreq;
      FreqBurst.setFrequency((double)PulseFreq);
      FreqBurst.start(-1, 0, false);
    }
  }
  TriggerOut(PulseWidth,true);
  if(BurstCount < 0) return;
  CurrentCount++;
  if(CurrentCount >= BurstCount) FreqBurst.stop();
}

// This function will generate a burst of clock pulses defined by PulseWidth and PulseFreq
void GenerateBurst(int num)
{
  SendACK;
  if(num == 0)
  {
    FreqBurst.stop();
    return;
  }
  BurstCount = num;
  // Start the real time interrupt
  CurrentCount=0;
  FreqBurst.attachInterrupt(FreqBurstISR);
  FreqBurst.setFrequency((double)PulseFreq);
  FreqBurst.start(-1, 0, false);
}

void QueueBurst(int num)
{
  BurstCount = num;
  BurstQueued = true;
}

void ProcessBurst(void)
{
  if(!BurstQueued) return;
  BurstQueued = false;
  CurrentCount=0;
  FreqBurst.attachInterrupt(FreqBurstISR);
  FreqBurst.setFrequency((double)PulseFreq);
  FreqBurst.start(-1, 0, false);
}

// This function is called by the serial command processor and saves the TWI device address and board address.
// These parameters are used for the low level device interface commands.
// There is no error checking of ack/nak logic in this function.
// board address is 0 or 1 and addr is a integer value
void DefineDeviceAddress(char *board, char *addr)
{
  String res;

  res = board;
  BoardSelect = res.toInt();
  res = addr;
  DeviceAddr = res.toInt();
}

// This function reports the average value read from the AD7988 ADC and reported through the USB interface.
void ReportAD7998(int chan)
{
  int brd;
  
  // Save current board address
  brd = digitalRead(BRDSEL);
  // Select board
  SelectBoard(BoardSelect);
  // Read channel
  serial->println((int)AD7998(DeviceAddr, chan, 100));
  // Restore board address
  digitalWrite(BRDSEL,brd);
}

// This function reports the average value read from the AD7984 ADC and reported through the USB interface.
void ReportAD7994(int chan)
{
  int brd;
  
  // Save current board address
  brd = digitalRead(BRDSEL);
  // Select board
  SelectBoard(BoardSelect);
  // Read channel
  serial->println((int)AD7994(DeviceAddr, chan, 100));
  // Restore board address
  digitalWrite(BRDSEL,brd);
}

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}
float Counts2Value(int Counts, ADCchan *AC)
{
  return (Counts - AC->b) / AC->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *AC)
{
  int counts;

  counts = (Value * AC->m) + AC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// calibration dalogbox data structures and variables
int ZeroCalValue;
int MidCalValue;
int DACZeroCounts;
int DACMidCounts;
int ADCZeroCounts;
int ADCMidCounts;

DialogBoxEntry CalibrationDialogEntries[] = {
  {" Set output to", 0, 4, D_INT, 0, 65535, 1, 18, false, "%d", &ZeroCalValue, NULL, NULL},
  {" Set output to", 0, 5, D_INT, 0, 65535, 1, 18, false, "%d", &MidCalValue, NULL, NULL},
  {" Exit", 0, 7, D_DIALOG, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"", 0, 2, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Abort", 0, 8, D_DIALOG, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"       Calibrating", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {NULL},
};

DialogBox CalibrationDialog = {
  {"Calibration Menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, CalibrationDialogEntries
};

// This function allows calibration of a DAC output channel along with its ADC readback channel.
//
// If this function blocks it will shut down all normal processing including the dialog box processing.
void ChannelCalibrate(ChannelCal *CC, char *Name)
{
  int ZeroPoint;
  int MidPoint;
  
  ZeroPoint = CC->Min + (CC->Max - CC->Min) / 2;
  MidPoint = ZeroPoint + (CC->Max - ZeroPoint) / 2;
  ChannelCalibrate(CC, Name, ZeroPoint, MidPoint);
}

void ChannelCalibrate(ChannelCal *CC, char *Name, int ZeroPoint, int MidPoint)
{
  ZeroCalValue = ZeroPoint;
  MidCalValue = MidPoint;
  // Setup the calibration dialog structure
  CalibrationDialogEntries[3].Name = Name;
  if (CC->DACout != NULL)
  {
    DACZeroCounts = Value2Counts((float)ZeroCalValue, CC->DACout);
    DACMidCounts = Value2Counts((float)MidCalValue, CC->DACout);
  }
  CalibrationDialogEntries[2].Value = ActiveDialog;
  CalibrationDialogEntries[4].Value = ActiveDialog;
  // Display the dialog
  DialogBoxDisplay(&CalibrationDialog);
  // Do not return, this function blocks and processes the encoder events in the following loop
  while (1)
  {
    ProcessSerial();
    WDT_Restart(WDT);
    LDAClow;
    if (CalibrationDialog.Selected == 0)
    {
      if (CC->DACout != NULL)
      {
        if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACZeroCounts);
        else if ((CC->DACaddr & 0xFE) == 0x10) { AD5593writeDAC(CC->DACaddr, CC->DACout->Chan, DACZeroCounts); delay(100);}
        else { AD5625(CC->DACaddr, CC->DACout->Chan, DACZeroCounts); delay(100);}
      }
      if (CC->ADCreadback != NULL) ADCZeroCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
    }
    if (CalibrationDialog.Selected == 1)
    {
      if (CC->DACout != NULL)
      {
        if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACMidCounts);
        else if ((CC->DACaddr & 0xFE) == 0x10) { AD5593writeDAC(CC->DACaddr, CC->DACout->Chan, DACMidCounts); delay(100);}
        else { AD5625(CC->DACaddr, CC->DACout->Chan, DACMidCounts); delay(100);}
      }
      if (CC->ADCreadback != NULL) ADCMidCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
    }
    // Process any encoder event
    if (ButtonRotated)
    {
      ButtonRotated = false;
      if ((CalibrationDialog.State == M_ENTRYSELECTED) && ((CalibrationDialog.Selected == 0) || (CalibrationDialog.Selected == 1)))
      {
        if (CalibrationDialog.Selected == 0)
        {
          DACZeroCounts += encValue;
          encValue = 0;
          if (DACZeroCounts < 0) DACZeroCounts = 0;
          if (DACZeroCounts > 65535) DACZeroCounts = 65535;
          if (CC->DACout != NULL)
          {
            if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACZeroCounts);
            else if ((CC->DACaddr & 0xFE) == 0x10) { AD5593writeDAC(CC->DACaddr, CC->DACout->Chan, DACZeroCounts); delay(100);}
            else { AD5625(CC->DACaddr, CC->DACout->Chan, DACZeroCounts); delay(100);}
          }
          if (CC->ADCreadback != NULL) ADCZeroCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
        }
        if (CalibrationDialog.Selected == 1)
        {
          DACMidCounts += encValue;
          encValue = 0;
          if (DACMidCounts < 0) DACMidCounts = 0;
          if (DACMidCounts > 65535) DACMidCounts = 65535;
          if (CC->DACout != NULL)
          {
            if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACMidCounts);
            else if ((CC->DACaddr & 0xFE) == 0x10) { AD5593writeDAC(CC->DACaddr, CC->DACout->Chan, DACMidCounts); delay(100);}
            else { AD5625(CC->DACaddr, CC->DACout->Chan, DACMidCounts); delay(100);}
          }
          if (CC->ADCreadback != NULL) ADCMidCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
        }
      }
      else
      {
        if (ActiveMenu != NULL) MenuProcessChange(ActiveMenu, encValue);
        else if (ActiveDialog != NULL) DialogBoxProcessChange(ActiveDialog, encValue);
        encValue = 0;
      }
    }
    if (ButtonPressed)
    {
      delay(10);
      ButtonPressed = false;
      encValue = 0;
      if (ActiveMenu != NULL) MenuButtonPress(ActiveMenu);
      else if (ActiveDialog != NULL) DialogButtonPress(ActiveDialog);
    }
    if (ActiveDialog != &CalibrationDialog) break; // This exit happens when user selects exit or abort options.
  }
  if (CC->DACout == NULL) if (CC->ADCreadback != NULL) ADCMidCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
  if (CalibrationDialog.Selected == 4) return;  // This is a result of abort option
  // If exit is selected then calculate the calibration factors.
  if (CC->DACout != NULL)
  {
    CC->DACout->m = ((float)DACZeroCounts - (float)DACMidCounts) / ((float)ZeroCalValue - (float)MidCalValue);
    CC->DACout->b = (float)DACZeroCounts - CC->DACout->m * (float)ZeroCalValue;
  }
  if (CC->ADCreadback != NULL)
  {
    CC->ADCreadback->m = ((float)ADCZeroCounts - (float)ADCMidCounts) / ((float)ZeroCalValue - (float)MidCalValue);
    CC->ADCreadback->b = (float)ADCZeroCounts - CC->ADCreadback->m * (float)ZeroCalValue;
  }
}

// This function sets the board select bit based on the board value
void SelectBoard(int8_t Board)
{
  static Pio *pio = g_APinDescription[BRDSEL].pPort;
  static uint32_t pin =g_APinDescription[BRDSEL].ulPin;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if ((Board & 1) == 1) pio->PIO_CODR = pin;    // Set pin low;
  else pio->PIO_SODR = pin;                     // Set pin high;
}

// Returns the current board selection
int SelectedBoard(void)
{
  static Pio *pio = g_APinDescription[BRDSEL].pPort;
  static uint32_t pin =g_APinDescription[BRDSEL].ulPin;

  if((pio->PIO_ODSR & pin) == 0) return(1);
  return(0);
}

// This function sets all the IO lines as needed by MIPS.
void Init_IOpins(void)
{
  // Setup the hardware pin directions. Note setting pinMode to output
  // will drive the output pin high.

  // Rotary encoder LEDS
  pinMode(PB_RED, OUTPUT);
  pinMode(PB_GREEN, OUTPUT);
  pinMode(PB_BLUE, OUTPUT);
  // Misc control lines
  pinMode(ADDR0, OUTPUT);
  pinMode(ADDR1, OUTPUT);
  pinMode(ADDR2, OUTPUT);
  pinMode(LDAC, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(BRDSEL, OUTPUT);
  pinMode(SPI_CS, OUTPUT);
  pinMode(PWR_ON, OUTPUT);
  pinMode(LDACctrl, OUTPUT);
  pinMode(TRGOUT, OUTPUT);
  digitalWrite(TRGOUT, HIGH);
  pinMode(AUXTRGOUT, OUTPUT);
  digitalWrite(AUXTRGOUT, HIGH);
  digitalWrite(LDACctrl, HIGH);
  pinMode(RFON, OUTPUT);
  pinMode(DOMSBlatch,OUTPUT);
  pinMode(DOenable,OUTPUT);
  RFON_OFF;
  ENA_BRD_A;
}

// This function resets all the IO lines to inputs, this is called
// when its detected that there is no power applied on Vin pin
void Reset_IOpins(void)
{
  // Rotary encoder LEDS
  pinMode(PB_RED, INPUT);
  pinMode(PB_GREEN, INPUT);
  pinMode(PB_BLUE, INPUT);
  // On PCB red LED
//  pinMode(RED_LED, INPUT);
  // Misc control lines
  pinMode(DAC0,INPUT);
//  pinMode(ADDR0, INPUT);
//  pinMode(ADDR1, INPUT);
//  pinMode(ADDR2, INPUT);
  pinMode(LDAC, INPUT);
  pinMode(SCL, INPUT);
  pinMode(BRDSEL, INPUT);
  pinMode(SPI_CS, INPUT);
  // Set the PWM outputs inactive, this is 5,6,7,8
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(14, HIGH);
  digitalWrite(48, HIGH);

  digitalWrite(ADDR0, LOW);
  digitalWrite(ADDR1, LOW);
  digitalWrite(ADDR2, LOW);

  // Make sure LDAC is low
  pinMode(LDACctrl,OUTPUT);
  pinMode(LDAC, OUTPUT);
  digitalWrite(LDAC, LOW);
  digitalWrite(LDACctrl, HIGH);
  
}

void Software_Reset(void)
{
  //============================================================================================
  //   führt ein Reset des Arduino DUE aus...
  //
  //   Parameter: keine
  //   Rueckgabe: keine
  //============================================================================================
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
  while (true);
}

// Reports the last reboot reason and the number of millisec the system has been up and running, this
// time counter is reset every 50 days.
void RebootStatus(void)
{
   SendACKonly;
   uint32_t i = REG_RSTC_SR;  // Reads the boot flag
   i >>= 8;
   i &= 7;   
   switch (i)
   {
       case 0:
          serial->print("First power-up Reset, ");
          break;
       case 1:
          serial->print("Return from Backup mode, ");
          break;
       case 2:
          serial->print("Watchdog fault occurred, ");
          break;
       case 3:
          serial->print("Processor reset required by the software, ");
          break;
       case 4:
          serial->print("NRST pin detected low, ");
          break;
       default:
          serial->print("Unknown!, ");
          break;
   }
   serial->println(millis());
   serial->print("TWI failure / reset count: ");
   serial->println(TWIfails);
   TraceReport();
}

// This function reads analog input on A8 and returns the value. A8 is connected to Vin through
// a voltage divider, 10K in series with 1K. This is used to determine in MIPS power is applied
// or if the USB is powering up the DUE.
// This function returns the Vin voltage as a float. The ADC input is left in its default 10 bit mode.
// A8 input is D62
//
// Updated on 2/4/2015 to read return average of 100 readings.
float ReadVin(void)
{
  int ADCvalue;
  int i;

  ADCvalue = 0;
  for (i = 0; i < 100; i++) ADCvalue += analogRead(62);
  ADCvalue = ADCvalue / 100;
  return ((((float)ADCvalue * 3.3) / 4096.0) * 11.0);
}

// This function will set the three address lines used for the SPI device selection. It is assumed
// the bit directions have already been set.
void SetAddress(int8_t addr)
{
  static Pio *pio = g_APinDescription[ADDR0].pPort;

  pio->PIO_CODR = 7;           // Set all bits low
  pio->PIO_SODR = addr & 7;    // Set bit high
}

// Clears the digitial output shift registers.
void ClearDOshiftRegs(void)
{
  pinMode(SCL,OUTPUT);
  digitalWrite(SCL, LOW);
  digitalWrite(SCL, HIGH);
  // Pulse the latch lines
  pinMode(DOMSBlatch,OUTPUT);
  digitalWrite(DOMSBlatch,LOW);
  digitalWrite(DOMSBlatch,HIGH);
  PulseLDAC;
  // This is used on REV 3.0 controller to disable output during boot
  pinMode(DOenable,OUTPUT);
  digitalWrite(DOenable,LOW);
  // This is DI6 used to drive the enable after its cut free from the buffer.
  // This is a hack for old controlers and needs be be removed at some point.
  //pinMode(46,OUTPUT);     // Removed 12/7/2018
  //digitalWrite(46,LOW);
}

// This function sets the selected output where chan is the output, 'A' through 'P'.
// active is HIGH or LOW, HIGH indicates active high output.
void SetOutput(char chan, int8_t active)
{
  int bn;

  // Make sure inputs are valid
  if((chan < 'A') || (chan > 'P')) return;
  if((active != HIGH) && (active != LOW)) return;
  // calculate bit number
  bn = chan - 'A';
  if(bn < 8)
  {
    if(active == HIGH) MIPSconfigData.DOlsb |= (1 << bn);
    else MIPSconfigData.DOlsb &= ~(1 << bn);
  }
  else
  {
    if(active == HIGH) MIPSconfigData.DOmsb |= (1 << (bn & 7));
    else MIPSconfigData.DOmsb &= ~(1 << (bn & 7));    
  }
  DigitalOut(MIPSconfigData.DOmsb,MIPSconfigData.DOlsb);
  PulseLDAC;
}

// This function clears the selected output where chan is the output, 'A' through 'P'.
// active is HIGH or LOW, HIGH indicates active high output.
void ClearOutput(char chan, int8_t active)
{
  if(active == HIGH) SetOutput(chan, LOW);
  else SetOutput(chan, HIGH);
}

// This function sends 16 bits to the digital IO using the SPI. Its assumes the SPI interface has been
// started.
// JP1 position 2 jumper needs to be installed on the MIPS controller hardware.
void DigitalOut(int8_t MSB, int8_t LSB)
{
  static Pio *pio = g_APinDescription[DOMSBlatch].pPort;
  static uint32_t pin =g_APinDescription[DOMSBlatch].ulPin;

  AtomicBlock< Atomic_RestoreState > a_Block;
  // Set the address to 6
  SetAddress(6);
  // Set mode
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  // Set the data
  SPI.transfer(SPI_CS, MSB, SPI_CONTINUE);
  SPI.transfer(SPI_CS, LSB);
  delayMicroseconds(2);              // Test the need for this delay
  // For rev 3.0 controller output using address 7 and stobe with output A12.
  SetAddress(7);
  SPI.transfer(SPI_CS, MSB);
  pio->PIO_CODR = pin;               // Set pin low;
  pio->PIO_SODR = pin;               // Set pin high;
  delayMicroseconds(2);              // Test the need for this delay
  SetAddress(0);
}

// This function reads the 8 digitial input lines and packs the data into one unsigned
// 8 bit byte. This value is returned.
uint8_t DigitalIn(void)
{
  uint8_t  val;

  val = 0;
  if (digitalRead(DI0) == HIGH) val |= 1;
  if (digitalRead(DI1) == HIGH) val |= 2;
  if (digitalRead(DI2) == HIGH) val |= 4;
  if (digitalRead(DI3) == HIGH) val |= 8;
  if (digitalRead(DI4) == HIGH) val |= 16;
  if (digitalRead(DI5) == HIGH) val |= 32;
  if (digitalRead(DI6) == HIGH) val |= 64;
  if (digitalRead(DI7) == HIGH) val |= 128;
  return (val);
}

int ReadInput(char inputCH)
{
  if(inputCH == 'Q') return(digitalRead(DI0));
  if(inputCH == 'R') return(digitalRead(DI1));
  if(inputCH == 'S') return(digitalRead(DI2));
  if(inputCH == 'T') return(digitalRead(DI3));
  if(inputCH == 'U') return(digitalRead(DI4));
  if(inputCH == 'V') return(digitalRead(DI5));
  if(inputCH == 'W') return(digitalRead(DI6));
  if(inputCH == 'X') return(digitalRead(DI7));
  return(LOW);
}

// =====================================================================================

// TWI device support low level routines

// =====================================================================================

// The following routines support reading and writting from an Atmel AT24CS04 512 x 8
// EEPROM. This device is located on each module in the MIPS system and holds module
// specific information

// src points to buffer used to hold the data.
// dard is the TWI device address.
// address is the start address in the EEPROM.
// count is the total number of bytes to read.
//
// Returns 0 if no errors are detected.
//
// There seems to be an arduino limit of 32 on the requestFrom function
// so the data is walked out 32 bytes at a time.
int ReadEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count)
{
  byte  *bval;
  int   iStat, i = 0, num;

  AcquireTWI();
  num = count;
  bval = (byte *)src;
//  delay(10);
  while (1)
  {
    Wire.beginTransmission(dadr | ((address >> 8) & 1));
    Wire.write(address & 0xFF);
    iStat = Wire.endTransmission(true);
    if (iStat != 0) 
    {
      ReleaseTWI();
      return (iStat);
    }
    if (num > 32) Wire.requestFrom(dadr | ((address >> 8) & 1), 32);
    else Wire.requestFrom(dadr | ((address >> 8) & 1), num);
    while (Wire.available())
    {
      *(bval++) = Wire.read();
      i++;
      if (i > count) 
      {
        ReleaseTWI();
        return (-1);
      }
    }
    if (num <= 32) break;
    num -= 32;
    address += 32;
  }
  if (i != count) 
  {
    ReleaseTWI();
    return (-1);
  }
  return (0);
}

// src points to buffer used to hold the data.
// dard is the TWI device address.
// address is the start address in the EEPROM.
// count is the total number of bytes to write.
//
// Returns 0 if no errors are detected.
int WriteEEPROM(void *src, uint8_t dadr, uint16_t address, uint16_t count)
{
  byte  *bval;
  int   iStat, i = 0, num;

  AcquireTWI();
  num = count;
  bval = (byte *)src;
  delay(10);
  while (1)
  {
    Wire.beginTransmission(dadr | ((address >> 8) & 1));
    Wire.write(address & 0xFF);
    // Write bytes, 16 maximum
    for (int j = 0; j < 16; j++)
    {
      if (j >= num) break;
      Wire.write(*(bval++));
      i++;
    }
    iStat = Wire.endTransmission(true);
    if (iStat != 0) 
    {
      ReleaseTWI();
      return (iStat);
    }
    // wait for it to finish writting 16 bytes or timeout
    for (int k = 0; k < 21; k++)
    {
      Wire.beginTransmission(dadr);
      Wire.write(0);
      if (Wire.endTransmission(true) == 0) break;
      if (k == 20)
      {
        ReleaseTWI();
        return (-1); // Timeout!
      }
    }
    // Setup for the next loop
    if (num <= 16) break;
    num -= 16;
    address += 16;
  }
  if (i != count) 
  {
    ReleaseTWI();
    return (-1);
  }
  ReleaseTWI();
  return (0);
}

// The following code sopports the TWI interface by implementing the protocol in
// software, i.e. bit banging. This provides maximum flexibility but is only
// used when the Wire function will not work.
#define TWI_SCL_OUT            pinMode(TWI_SCL,OUTPUT)
#define TWI_SDA_OUT            pinMode(TWI_SDA,OUTPUT)
#define TWI_SDA_IN             pinMode(TWI_SDA,INPUT)
#define TWI_SCL_HI             digitalWrite(TWI_SCL,HIGH)
#define TWI_SCL_LOW            digitalWrite(TWI_SCL,LOW)
#define TWI_SDA_HI             digitalWrite(TWI_SDA,HIGH)
#define TWI_SDA_LOW            digitalWrite(TWI_SDA,LOW)
#define TWI_SDA_data           digitalRead(TWI_SDA)

// TWI bus reset function.
// The bus recovery procedure is as follows
//  1.) Master tries to assert logic 1 on SDA line
//  2.) Master still sees a logig 0 then generate a clock pulse on SLC (1,0,1 transistion)
//  3.) Master examines SDA. If SDA = 0 goto step 2 if SDA = 1 goto step 4
//  4.) Generate a STOP condition
void TWI_RESET(void)
{
  // Do resect procedure for both board addresses
  int brd = SelectedBoard();
  for(int b=0; b<2;b++)
  {
    SelectBoard(b);
    for(int i=0;i<2;i++)
    {
      TWI_START();
      TWI_STOP();
      TWI_SDA_IN;
      TWI_SCL_OUT;
      for(int i = 0; i < 10; i++)
      {
        TWI_SCL_HI;
        TWI_SCL_LOW;
        delayMicroseconds(5);
        TWI_SCL_HI;
        delayMicroseconds(5);
        if(TWI_SDA_data == HIGH) break;
      }
      TWI_STOP();
      TWI_STOP();
    }
  }
  SelectBoard(brd);
  return;
 
  TWI_START();
  TWI_STOP();
  // Generate a bunch of clocks
  for (int i = 0; i < 1000; i++)
  {
    TWI_SCL_LOW;
    TWI_SCL_HI;
  }
  TWI_STOP();
  TWI_STOP();
}

// Issue a start condition
void TWI_START(void)
{
  TWI_SCL_OUT;
  TWI_SDA_OUT;
  TWI_SCL_HI;
  TWI_SDA_HI;
  TWI_SDA_LOW;
  TWI_SCL_LOW;
}

// Issue a stop condition
void TWI_STOP(void)
{
  TWI_SDA_OUT;
  TWI_SDA_LOW;
  TWI_SDA_LOW;
  TWI_SCL_HI;
  TWI_SDA_HI;
}

// Write a byte to the TWI bus, this function returns true if acked and false if naked
bool TWI_WRITE(int8_t val)
{
  int8_t Response;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  TWI_SDA_OUT;
  for (int i = 0; i < 8; i++)
  {
    if ((val & 0x80) != 0) TWI_SDA_HI;
    else TWI_SDA_LOW;
    val = val << 1;
    TWI_SCL_HI;
    TWI_SCL_LOW;
  }
  // Now read the ACK or NAK from the device
  TWI_SDA_IN;
  TWI_SCL_HI;
  Response = TWI_SDA_data;
  TWI_SCL_LOW;
  if (Response == HIGH) return (false);
  return (true);
}

// Reads and returns a byte from the TWI interface, if reply is true than ACK is sent to device,
// if reply is false the NAK is sent.
int8_t TWI_READ(bool Reply)
{
  int8_t val, r;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  val = 0;
  TWI_SDA_IN;
  for (int i = 0; i < 8; i++)
  {
    val = val << 1;
    TWI_SCL_HI;
    if (TWI_SDA_data == HIGH) val |= 1;
    TWI_SCL_LOW;
  }
  // Now write the ACK or NAK to the device
  TWI_SDA_OUT;
  if (Reply == HIGH) TWI_SDA_HI;
  else TWI_SDA_LOW;
  TWI_SCL_HI;
  TWI_SCL_LOW;
  TWI_SDA_IN;
  return (val);
}

// Called when a TWI error is detected. This function will reset the TWI interface and reinit the
// wire driver
void TWIerror(void)
{
  TWI_RESET();
  Wire.begin();
}

// The following routines support the Analog Devices DAC and ADC used to monitor and
// control voltages in the MIPS system.

// The AD7998 is 8 channel ADC. A 2.5 volt reference is used.
//
// This function outputs a value to the selected channel.
// adr = TWI address of device
// vals = pointer to a memory block where results are saved.
//        the values are 0 to 65535 reguardless of ADC resolution
//
// Return the status of the TWI transaction, 0 if no errors or -1.
//
// Note: ARM byte order is LSB then MSB!
//
// The Arduino Wire driver will not work to drive this device. I used "bit banging" TWI
// routines defined above. This device requires no stop condition between the conversion
// write and the read of data.
int AD7998(int8_t adr, uint16_t *vals)
{
  int   iStat, i,v;
  byte  *bvals;

  for (i = 0; i < 8; i++)
  {
    AD7998(adr, i);
    v = AD7998(adr, i);
    if(v == -1) return(-1);
    vals[i] = v;
  }
  return (0);

  AcquireTWI();
  bvals = (byte *)vals;
  while (1)
  {
    TWI_START();
    if ((iStat = TWI_WRITE(adr << 1)) == false) break;
    if ((iStat = TWI_WRITE(0x02)) == false) break;
    if ((iStat = TWI_WRITE(0x0F)) == false) break;
    if ((iStat = TWI_WRITE(0xF8)) == false) break;
    if ((iStat = TWI_WRITE(0x70)) == false) break;
    TWI_START();
    if ((iStat = TWI_WRITE((adr << 1) + 1)) == false) break;
    for (i = 0; i < 8; i++)
    {
      bvals[i * 2 + 1] = TWI_READ(LOW);
      if (i == 7) bvals[i * 2] = TWI_READ(HIGH);
      else bvals[i * 2] = TWI_READ(LOW);
    }
    TWI_STOP();
    for (i = 0; i < 8; i++) vals[i] &= 0xFFF;
    for (i = 0; i < 8; i++) vals[i] <<= 4;
    iStat = 0;
    break;
  }
  Wire.begin();  // Release control of clock and data lines
  ReleaseTWI();
  return (iStat);
}

// 4 channel ADC
int AD7994(int8_t adr, uint16_t *vals)
{
  int   iStat, i, v;
  byte  *bvals;

  for (i = 0; i < 4; i++)
  {
    AD7994(adr, i);
    v = AD7994(adr, i);
    if(v == -1) return(-1);
    vals[i] = v;

  }
  return (0);
}

int AD7994_b(int8_t adr, int8_t chan)
{
  unsigned int val;
  
  AcquireTWI();
  chan++;
  if (chan == 3) chan = 4;
  else if (chan == 4) chan = 8;
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(adr);
  if(Wire.endTransmission(false) !=0)
  {
    TWIerror();
    TWIfails++;
    ReleaseTWI();
    return(-1);
  }
  #ifdef IDE == 1.6.5
  Wire.requestFrom(adr, 2, (chan) << 4, 1);
  #else
  Wire.requestFrom(adr, 2, (chan) << 4, 1,0);
  #endif
  while (Wire.available() < 2);
  val = (Wire.read() << 8) & 0xFF00;
  val |= (Wire.read()) & 0xFF;
  val &= 0xFFF;
  val <<= 4;
  if(Wire.endTransmission() !=0)
  {
    TWIerror();
    TWIfails++;
    ReleaseTWI();
    return(-1);
  }
  ReleaseTWI();
  return(val);
}

int AD7994(int8_t adr, int8_t chan)
{
  int   iStat, i;
  unsigned int val;

  if(MIPSconfigData.TWIhardware) return(AD7994_b(adr,chan));
  AcquireTWI();
  while (1)
  {
    chan++;
    if (chan == 3) chan = 4;
    else if (chan == 4) chan = 8;
    TWI_START();
    if ((iStat = TWI_WRITE(adr << 1)) == false) break;
    if ((iStat = TWI_WRITE((chan) << 4)) == false) break;
    TWI_START();
    if ((iStat = TWI_WRITE((adr << 1) + 1)) == false) break;
    val = (TWI_READ(LOW) << 8) & 0xFF00;
    val |= (TWI_READ(HIGH)) & 0xFF;
    TWI_STOP();
    val &= 0xFFF;
    val <<= 4;
    iStat = 0;
    break;
  }
  Wire.begin();  // Release control of clock and data lines
  ReleaseTWI();
  if(iStat != 0) return(-1); 
  return (val);
}

// This function reads one channel from the AD7998 ADC
int AD7998_b (int8_t adr, int8_t chan)
{
  unsigned int val;
  
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(adr);
  if(Wire.endTransmission(false) !=0)
  {
    TWIerror();
    TWIfails++;
    ReleaseTWI();
    return(-1);
  }
  #ifdef IDE == 1.6.5
  Wire.requestFrom(adr, 2, 0x80 | chan << 4, 1);
  #else
  Wire.requestFrom(adr, 2, 0x80 | chan << 4, 1, 0);
  #endif
  while (Wire.available() < 2);
  val = (Wire.read() << 8) & 0xFF00;
  val |= (Wire.read()) & 0xFF;
  val &= 0xFFF;
  val <<= 4;
  if(Wire.endTransmission() !=0)
  {
    TWIerror();
    TWIfails++;
    ReleaseTWI();
    return(-1);
  }
  ReleaseTWI();
  return(val);
}

// This function reads one channel from the AD7998 ADC
int AD7998(int8_t adr, int8_t chan)
{
  int   iStat, i;
  unsigned int val;

  if(MIPSconfigData.TWIhardware) return(AD7998_b(adr,chan));
  AcquireTWI();
  while (1)
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    TWI_START();
    if ((iStat = TWI_WRITE(adr << 1)) == false) break;
    if ((iStat = TWI_WRITE(0x80 | chan << 4)) == false) break;
    TWI_START();
    if ((iStat = TWI_WRITE((adr << 1) + 1)) == false) break;
    val = (TWI_READ(LOW) << 8) & 0xFF00;
    val |= (TWI_READ(HIGH)) & 0xFF;
    TWI_STOP();
    val &= 0xFFF;
    val <<= 4;
    iStat = 0;
    break;
  }
  Wire.begin();  // Release control of clock and data lines
  ReleaseTWI();
  return (val);
}

// This function read one ADC channel a user selected number of times and returns
// the average value.
int AD7998(int8_t adr, int8_t chan, int8_t num)
{
  int i, val = 0;

  for (i = 0; i < num; i++) val += AD7998(adr, chan);
  return (val / num);
}

int AD7994(int8_t adr, int8_t chan, int8_t num)
{
  int i, val = 0;

  for (i = 0; i < num; i++) val += AD7994(adr, chan);
  return (val / num);
}

// AD5593 IO routines. This is a analog and digitial IO chip with
// a TWI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5593
// Return 0 if no error else an error code is returned
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val)
{
  int iStat;
  
  AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write(pb);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  ReleaseTWI();
  return (iStat);
}

// Read from AD5593R
// returns -1 on any error
int AD5593readWord(uint8_t addr, uint8_t pb)
{
  int iStat;
  
  AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write(pb);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  if(iStat != 0)
  {
    ReleaseTWI();
    return (-1);
  }
  // Now read the data word
  int i = 0,j = 0;
  Wire.requestFrom(addr, (uint8_t)2);
  while(Wire.available())
  {
     if(i==0) j = Wire.read() << 8;
     if(i==1) j |= Wire.read();
     i++;
  }
  ReleaseTWI();
  return(j);
}

int AD5593readADC(int8_t addr, int8_t chan)
{
   int iStat;
   
   // Select the ADC channel number
   if((iStat = AD5593write(addr, 0x02, (1 << chan))) != 0) return(-1);
   // Read the data and make sure the address is correct then left 
   // justify in 16 bits
   int i = AD5593readWord(addr, 0x40);
   if(((i >> 12) & 0x7) != chan) return(-1);
   i <<= 4;
   return(i & 0xFFFF);
}


int AD5593readADC(int8_t addr, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5593readADC(addr, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

int AD5593writeDAC(int8_t addr, int8_t chan, int val)
{
   uint16_t  d;
   // convert 16 bit DAC value into the DAC data data reg format
   d = (val>>4) | (chan << 12) | 0x8000;
   return(AD5593write(addr, 0x10 | chan, d));
}

// End of AD5593 routines

// AD5625 is a 4 channel DAC.
//
// This function outputs a value to the selected channel.
// adr = TWI address of device
// chan = channel number, 0,1,2, or 3
// val = binary value to output to the DAC
//
// Return the status of the TWI transaction, 0 if no errors.
int AD5625(int8_t adr, uint8_t chan, uint16_t val)
{
  AD5625(adr, chan, val, 0);
}

int AD5625(int8_t adr, uint8_t chan, uint16_t val,int8_t Cmd)
{
  int iStat;

  AcquireTWI();
//  interrupts();
  Wire.beginTransmission(adr);
  Wire.write((Cmd << 3) | chan);
  //    if(chan <= 3) val <<= 4;
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  {
//    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
    if(iStat != 0)
    {
       TWIerror();
       TWIfails++;
    }
  }
  ReleaseTWI();
  return (iStat);
}

// This function enables the internal voltage reference in the
// AD5625
int AD5625_EnableRef(int8_t adr)
{
  int iStat;

  AcquireTWI();
  Wire.beginTransmission(adr);
  Wire.write(0x38);
  Wire.write(0);
  Wire.write(1);
  iStat = Wire.endTransmission();
  ReleaseTWI();
  return (iStat);
}

// This function writes the value to the selected channel on the selected SPI
// address to the 8 channel DAC.
// Feb 11, 2016. Working to speed up this transfer
//  - about 2uS between bytes
//  - 1 byte sent is about 800nS
// Buffering ans using the black transfer cut byte transfer to a little over 1 uS
// About 25uS between channel updates
// This whole routine takes 25uS!
// SetAddress takes 10uS!
// After all updates for performance this routine takes about 5uS
// March 13, 2016: Added command parameter.
// Cmd = 0 for update on LDAC
// Cmd = 3 to update when written reguardless of LDAC
void AD5668(int8_t spiAdr, int8_t DACchan, uint16_t vali)
{
  AD5668(spiAdr, DACchan, vali, 0);
}

// Select only one more of operation!
//#define useSPIclass
#define useSPIinline
//#define useSPIDMA
void AD5668(int8_t spiAdr, int8_t DACchan, uint16_t vali, int8_t Cmd)
{
  uint8_t     buf[4];
  uint16_t    val;
  static bool inited = false;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if (!inited)
  {
    inited = true;
    SPI.setDataMode(SPI_CS, SPI_MODE1);
    // Set the address
    SetAddress(spiAdr);
    // Set the data
    SPI.transfer(SPI_CS, 0x06, SPI_CONTINUE);
    SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
    SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
    SPI.transfer(SPI_CS, 0x00);
    SetAddress(0);
    // Enable the DMA controller for SPI transfers
    spiDMAinit();
  }
  val = vali;
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  // Set the address
  SetAddress(spiAdr);
// Option 1, send with SPI object
#ifdef useSPIclass
  // Fill buffer with data
  buf[0] = Cmd;
  buf[1] = (DACchan << 4) | (val >> 12);
  buf[2] = (val >> 4);
  buf[3] = (val << 4);
  // Send the data
  SPI.transfer(SPI_CS, buf, 4);
#endif
// Option 2, send with inline minimul code
#ifdef useSPIinline
  Spi* pSpi = SPI0;
  uint32_t b;
  static uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(SPI_CS);
  pSpi->SPI_TDR = (uint32_t)Cmd | SPI_PCS(ch);
  while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
  b = pSpi->SPI_RDR;
  pSpi->SPI_TDR = (uint32_t)(((DACchan << 4) | (val >> 12)) & 0xFF) | SPI_PCS(ch);
  while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
  b = pSpi->SPI_RDR;
  pSpi->SPI_TDR = (uint32_t)((val >> 4) & 0xFF) | SPI_PCS(ch);
  while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
  b = pSpi->SPI_RDR;
  pSpi->SPI_TDR = (uint32_t)((val << 4) & 0xFF) | SPI_PCS(ch) | SPI_TDR_LASTXFER;
  while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
  b = pSpi->SPI_RDR;
#endif
// Option 3, Send with DMA, will help on big buffers
#ifdef useSPIDMA
  static uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(SPI_CS);
  uint32_t dbuf[4];
  dbuf[0] = (uint32_t)Cmd | SPI_PCS(ch);
  dbuf[1] = (uint32_t)(((DACchan << 4) | (val >> 12)) & 0xFF) | SPI_PCS(ch);
  dbuf[2] = (uint32_t)((val >> 4) & 0xFF) | SPI_PCS(ch);
  dbuf[3] = (uint32_t)((val << 4) & 0xFF) | SPI_PCS(ch) | SPI_TDR_LASTXFER;
  spiDmaTX(&dbuf[0], 4);
  spiDmaWait();
#endif
// Finished so clear the address to 0
  SetAddress(0);
}

// The following routine supports the MCP2300 GPIO TWI device.
// This is used for the sequence generator in Twave module and
// all bits are set to output.
int MCP2300(int8_t adr, uint8_t bits)
{
  int iStat;

  AcquireTWI();
  Wire.beginTransmission(adr);
  Wire.write(0);    // IO direction register
  Wire.write(0);    // Set all bits to output
  if ((iStat = Wire.endTransmission()) != 0 ) 
  {
    ReleaseTWI();
    return (iStat);
  }
  Wire.beginTransmission(adr);
  Wire.write(0x0A);  // Read the output latch command
  Wire.write(bits);
  iStat = Wire.endTransmission();
  ReleaseTWI();
  return (iStat);
}

// This function writes a byte to the selected register in the
// MCP2300 GPIO TWI device.
int MCP2300(int8_t adr, uint8_t reg, uint8_t bits)
{
  int iStat;

  AcquireTWI();
  Wire.beginTransmission(adr);
  Wire.write(reg);     // Register
  Wire.write(bits);    // Set bits
  iStat = Wire.endTransmission();
  ReleaseTWI();
  return (iStat);
}

// This function reads a byte from the selected register in the
// MCP2300 GPIO TWI device.
int MCP2300(int8_t adr, uint8_t reg, uint8_t *data)
{
  int iStat;

  AcquireTWI();
  Wire.beginTransmission(adr);
  Wire.write(reg);     // Register
  if ((iStat = Wire.endTransmission()) != 0 )
  {
    ReleaseTWI();
    return (iStat);
  }
  Wire.requestFrom(adr, 1);
  if (Wire.available()) *data = Wire.read();
  ReleaseTWI();
  return (0);
}

//
// Routines used to allow TWI use in ISR. The TWI queue functions allow you to
// queue function with parameters.
//
#define MaxQueued 10

enum TWIcbType
{
  Empty,
  VoidVoid,
  VoidIntIntFloat,
  VoidIntIntBool,
  VoidIntIntWord
};

union TWIfunctions
{
  void  (*funcVoidVoid)(void);
  void  (*funcIntIntFloat)(int, int, float);
  void  (*funcIntIntBool)(int, int, bool);
  void  (*funcIntIntWord)(int, int, uint16_t);
};

typedef struct 
{
  TWIcbType             Type;
  union   TWIfunctions  pointers;
  int                   Int1,Int2;
  float                 Float1;
  bool                  Bool1;
  uint16_t              Word1;
} TWIqueueEntry;

TWIqueueEntry TWIq[MaxQueued] = {{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL}};

bool TWIbusy=false;
// This functoin acquires the TWI interface.
// Returns false if it was busy.
bool AcquireTWI(void)
{
 // AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy) return(false);
  TWIbusy = true;
  return(true);
}

// This routine releases the TWI interface and if there are any queued functions they are called
// and then the queued pointer it set to NULL. This allows an ISR to use this function
// and queue up its io if the TWI interface is busy.
void ReleaseTWI(void)
{
  static bool busy = false;

  if(busy) return;
  busy = true;
  //AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy)
  {
    for(int i=0;i<MaxQueued;i++)
    {
      if(TWIq[i].pointers.funcVoidVoid != NULL) 
      {
        if(TWIq[i].Type == VoidVoid) TWIq[i].pointers.funcVoidVoid();
        else if(TWIq[i].Type == VoidIntIntFloat) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Float1);
        else if(TWIq[i].Type == VoidIntIntBool) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Bool1);
        else if(TWIq[i].Type == VoidIntIntWord) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Word1);
        TWIq[i].pointers.funcVoidVoid = NULL;
        TWIq[i].Type = Empty;
      }
    }
    TWIbusy=false;
  }
  busy=false;
}

// This queues up a function to call when the current TWI operation finishes.
void TWIqueue(void (*TWIfunction)(void))
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcVoidVoid == NULL)
  {
     TWIq[i].pointers.funcVoidVoid = TWIfunction;
     TWIq[i].Type = VoidVoid;
     break;
  }
}

void TWIqueue(void (*TWIfunction)(int,int,float),int arg1,int arg2,float arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntFloat == NULL)
  {
     TWIq[i].pointers.funcIntIntFloat = TWIfunction;
     TWIq[i].Type   = VoidIntIntFloat;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Float1 = arg3;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,bool),int arg1,int arg2,bool arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntBool == NULL)
  {
     TWIq[i].pointers.funcIntIntBool = TWIfunction;
     TWIq[i].Type   = VoidIntIntBool;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Bool1  = arg3;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,uint16_t),int arg1,int arg2,uint16_t arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntWord == NULL)
  {
     TWIq[i].pointers.funcIntIntWord = TWIfunction;
     TWIq[i].Type   = VoidIntIntWord;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Word1  = arg3;
     break;
  }  
}

// Trigger output functions. These functions support pulsing the output in micro second units as
// well as queueing up the function for later execution, this support using the trigger output
// in a signal table.

// This function is valid for rev 2.0 and above controllers. This command
// will generate a output trigger pulse. The following commands are
// supported:
//  HIGH, sets the output high
//  LOW, sets the output low
//  PULSE, pulses the output from its current state for 1 milli sec
//  If an integer value is passed then we pulse in micro seconds
void TriggerOut(char *cmd)
{
  String Cmd;
  int uS;
  
  Cmd = cmd;
  uS = Cmd.toInt();
  if(uS >0)
  {
     SendACK;
     TriggerOut(uS);
     return;
  }
  if ((strcmp(cmd, "HIGH") == 0) || (strcmp(cmd, "LOW") == 0) || (strcmp(cmd, "PULSE") == 0) || (strcmp(cmd, "FollowS") == 0))
  {
    SendACK;
    if (strcmp(cmd, "FollowS") == 0) TriggerFollowS();
    if (strcmp(cmd, "HIGH") == 0) digitalWrite(TRGOUT, LOW);
    if (strcmp(cmd, "LOW") == 0) digitalWrite(TRGOUT, HIGH);
    if (strcmp(cmd, "PULSE") == 0)
    {
      if (digitalRead(TRGOUT) == HIGH)
      {
        digitalWrite(TRGOUT, LOW);
        delay(1);
        digitalWrite(TRGOUT, HIGH);
      }
      else
      {
        digitalWrite(TRGOUT, HIGH);
        delay(1);
        digitalWrite(TRGOUT, LOW);
      }
    }
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void AuxOut(char *cmd)
{
  String Cmd;
  
  Cmd = cmd;
  if ((strcmp(cmd, "HIGH") == 0) || (strcmp(cmd, "LOW") == 0) || (strcmp(cmd, "PULSE") == 0))
  {
    SendACK;
    if (strcmp(cmd, "HIGH") == 0) digitalWrite(AUXTRGOUT, LOW);
    if (strcmp(cmd, "LOW") == 0) digitalWrite(AUXTRGOUT, HIGH);
    if (strcmp(cmd, "PULSE") == 0)
    {
      if (digitalRead(AUXTRGOUT) == HIGH)
      {
        digitalWrite(AUXTRGOUT, LOW);
        delay(1);
        digitalWrite(AUXTRGOUT, HIGH);
      }
      else
      {
        digitalWrite(AUXTRGOUT, HIGH);
        delay(1);
        digitalWrite(AUXTRGOUT, LOW);
      }
    }
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

void FollowSisr(void)
{
   static Pio *pioS = g_APinDescription[DI2].pPort;
   static uint32_t pinS = g_APinDescription[DI2].ulPin;
   static Pio *pioTrig = g_APinDescription[TRGOUT].pPort;
   static uint32_t pinTrig = g_APinDescription[TRGOUT].ulPin;

   if(pioS->PIO_PDSR & pinS) pioTrig->PIO_CODR = pinTrig;
   else pioTrig->PIO_SODR = pinTrig;
}

void TriggerFollowS(void)
{
   static Pio *pioS = g_APinDescription[DI2].pPort;
   static uint32_t pinS = g_APinDescription[DI2].ulPin;
   
   attachInterrupt(digitalPinToInterrupt(DI2), FollowSisr, CHANGE);
   pioS->PIO_SCIFSR = pinS;
   pioS->PIO_IFER = pinS;  // Enable the fast deglitch
}

// This function will pulse the trigger output 
inline void TriggerOut(int microSec, bool WithLimits)
{
  static Pio *pio = g_APinDescription[TRGOUT].pPort;
  static uint32_t pin = g_APinDescription[TRGOUT].ulPin;
  static uint32_t dwReg;
  static int16_t skip = 0;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if((TrigMax > 0) && WithLimits)
  {
    if(++skip > ((PulseFreq-1) / TrigMax)) skip = 0;
    if(skip != 0) 
    {
      PlayTpulseFunctions();
      return;
    }
  }
  dwReg = pio->PIO_ODSR;       // Get output state
  if((dwReg & pin) == 0)
  {
    pio->PIO_SODR = pin;         // Set output high
    delayMicroseconds(microSec);
    pio->PIO_CODR = pin;         // Set output low
  }
  else
  {
    pio->PIO_CODR = pin;         // Set output low
    delayMicroseconds(microSec);
    pio->PIO_SODR = pin;         // Set output high
  }
  PlayTpulseFunctions();
}

#define NUMQTF 5
void  (*QuededTrigFunctionList[NUMQTF])() = {NULL,NULL,NULL,NULL,NULL};

// This function adds or removes a function from a list of up to five
// function pointers. These functions are called after the Trigger
// pulse is generated. 
// if add = true the function is added, if false its removed
bool QueueTpulseFunction(void (*Tfunction)(), bool add)
{
  int i;
  
  if(add)
  {
    // First test if its already in the list
    for(i=0;i<NUMQTF;i++) if(QuededTrigFunctionList[i] == Tfunction) return true;
    // Add the function to the list
    for(i=0;i<NUMQTF;i++)
    {
      if(QuededTrigFunctionList[i] == NULL) 
      {
        QuededTrigFunctionList[i] = Tfunction;
        return true;
      }
    }
    return false;
  }
  else
  {
    for(i=0;i<NUMQTF;i++)
    {
      if(QuededTrigFunctionList[i] == Tfunction) 
      {
        QuededTrigFunctionList[i] = NULL;
        return true;
      }
    }
    return false;  
  }
  return true;
}

// This function will call all the queued trigger functions
void PlayTpulseFunctions(void)
{
  for(int i=0;i<NUMQTF;i++)
  {
    if(QuededTrigFunctionList[i] != NULL) QuededTrigFunctionList[i](); 
  }
}

void AuxTrigger(void)
{
  static Pio *pio = g_APinDescription[AUXTRGOUT].pPort;
  static uint32_t pin = g_APinDescription[AUXTRGOUT].ulPin;
  static int16_t skip = 0;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if(AuxTrigMax > 0)
  {
    if(++skip > ((PulseFreq-1) / AuxTrigMax)) skip = 0;
    if(skip != 0) return;
  }
  pio->PIO_CODR = pin;         // Set output high
  delayMicroseconds(PulseWidth);
  pio->PIO_SODR = pin;         // Set output low  
}

bool TriggerOutQueued = false;
int  TriggerOutmicroSec = 0;

// Call to queue the trigger out function for later used
void QueueTriggerOut(int microSec)
{
  TriggerOutQueued = true;
  TriggerOutmicroSec = microSec;
}

// If queued call it and clear the flag
void ProcessTriggerOut(void)
{
  if(TriggerOutQueued)
  {
    TriggerOutQueued = false;
    TriggerOut(TriggerOutmicroSec);
  }
}

// ADC output function to support the ADC command. This function takes an integer values and returns the 
// Counts for the channel selected.
// Valid channels numbers are 0 through 3.

void ADCread(int chan)
{
   int i;
  
   if((chan >= 0) && (chan <=3))
   {
     SendACKonly;
     i = analogRead(chan);
     if(!SerialMute) serial->println(i);
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;
}

// The following routines support DIO operations:
//  Input state changes
//  Input to output mirroring
// One common interrupt supports all 8 input channels. The attach interrupt is always done 
// with the change mode, the ISR will read the port to determine the actual change.
//
// Would also like to add a trigger command where an input edge triggers an output
// action. Like TRIG,Q,P,A,L this would cause output A to go low on a positive edge of input Q

// This function is called from the main pooling loop and sends the
// change messages when enabled.
// Updated 9/2/2017 to first build a string and then send the full string, needed on the MALID system
// to stop missing parts of the message.
void DIOopsReport(void)
{
  int  i;
  char chan;
  char sbuf[50];

  for(i=0;i<8;i++)
  {
    chan = 'Q' + i;
    if((dioops[i].Report) && (dioops[i].Changed))
    {
      uint32_t t = millis();
      if(dioops[i].ReportState == CHANGE) sprintf(sbuf,"DIC,%c,CHANGED,%u\n",chan,t);
      if(dioops[i].ReportState == RISING) sprintf(sbuf,"DIC,%c,RISING,%u\n",chan,t);
      if(dioops[i].ReportState == FALLING) sprintf(sbuf,"DIC,%c,FALLING,%u\n",chan,t);
      serial->print(sbuf);
      dioops[i].Changed = false;
    }
  }
}

// This interrupt service routine looks through the dioops structure and
// processes any changes. Reporting is done in the main polling loop.
// The attach interrupt is always done as change to make sure we see every transistion.
void DIOopsISR(void)
{
  int  i,j;
  bool DIOset=false;

  TRACE(9);
  for(i=0;i<8;i++)
  {
//    if((dioops[i].Report) || (dioops[i].Mirror))
    {
      j = digitalRead(dioops[i].DI);
      if(j != dioops[i].LastLevel)
      {
        dioops[i].LastLevel = j;
        if(!dioops[i].Mirror)
        {
          if(dioops[i].ReportState == CHANGE) dioops[i].Changed = true;
          if((dioops[i].ReportState == RISING) && (j == HIGH)) dioops[i].Changed = true;
          if((dioops[i].ReportState == FALLING) && (j == LOW)) dioops[i].Changed = true;
        }
        if(dioops[i].Mirror)
        {
          if(j == LOW) SDIO_Set_Image(dioops[i].DO, '0');
          else SDIO_Set_Image(dioops[i].DO, '1');
          DIOset = true;  // flag to cause update
        }
      }
    }
  }
  if(DIOset)
  {
    DOrefresh;
    PulseLDAC;
    UpdateDigitialOutputArray();
  }
}

// This function is a host command processing function. This function will set up an input port for
// change monitoring and will send a command to the host when the change is detected.
// port is Q,R,S,T,U,V,W, or X
// mode is RISING,FALLING,CHANGE, or OFF
// OFF will remove and monitoring function.
void DIOreportMonitor(char *port, char *mode, bool Report)
{
  int i;

  // Convert port to index
  i = port[0] - 'Q';
  if((i<0) || (i>7)) 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if ((strcmp(mode, "RISING") == 0) || (strcmp(mode, "FALLING") == 0) || (strcmp(mode, "CHANGE") == 0))
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].ReportState = CHANGE;
    if (strcmp(mode, "RISING") == 0) dioops[i].ReportState = RISING;
    if (strcmp(mode, "FALLING") == 0) dioops[i].ReportState = FALLING;
    dioops[i].Report = Report;
    dioops[i].Changed = false;
    if(!dioops[i].Mirror) dioops[i].DIh->attached(port[0],CHANGE,DIOopsISR);
    SendACK;
    return;
  }
  if (strcmp(mode, "OFF") == 0)
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].ReportState = 0;
    dioops[i].Report = false;
    dioops[i].Changed = false;
    if(!dioops[i].Mirror) dioops[i].DIh->detach();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void DIOreport(char *port, char *mode)
{
  DIOreportMonitor(port,mode,true);
}

void DIOmonitor(char *port, char *mode)
{
  DIOreportMonitor(port,mode,false);  
}

void DIOchangeReport(char *port)
{
  int  i;

  i = port[0] - 'Q';
  if((i < 0) || (i > 7))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(SerialMute) return;
  if(dioops[i].Changed)
  {
    if((dioops[i].ReportState == CHANGE)||(dioops[i].ReportState == RISING)||(dioops[i].ReportState == FALLING)) serial->println("TRUE");
    else serial->println("FALSE");
    return;
  }
  serial->println("FALSE");
}

// This function will mirror an input (Q through X) to an output (A through P or OFF)
void DIOmirror(char *in, char *out)
{
  int i;

  // Convert in to index
  i = in[0] - 'Q';
  if((i<0) || (i>7)) 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If out is OFF then disable the mirror for this channel
  if (strcmp(out, "OFF") == 0)
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].Mirror = false;
    if(!dioops[i].Report) dioops[i].DIh->detach();
    SendACK;
    return;
  }
  if((out[0] >= 'A') && (out[0] <= 'P'))
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].Mirror = true;
    dioops[i].DO = out[0];
    if(!dioops[i].Report) dioops[i].DIh->attached(in[0],CHANGE,DIOopsISR);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// 
// The following functions support delayed triggering and trigger repeat functions. These
// functions allow you to define an input trigger and then a module that this trigger will
// call. The trigger can be programmed with a delay as well as a trigger repeat period and
// number of repeats. The interface to this these functions are through the host computer
// interface.
//
// Serial commands:
//  SDTRIGINP,input,active    Defines the trigger input Q - X and active level POS or NEG.
//  SDTRIGDLY,time            Defines the trigger delay in microseconds.
//  SDTRIGPRD,period          Defines the trigger repeat time in microseconds.
//  SDTRIGRPT,num             Defines the number of trigger repeats, 0 = forever
//  SDTRIGMOD,module          Define the module that this trigger affects
//  SDTRIGENA,state           True enables the trigger false disables
//
DIhandler *DIdelayedTrigger = NULL;
MIPStimer *DelayTriggerTMR = NULL;
void  (*DelayedTrigFunction)() = NULL;
char DtrigInput[3] = "NA";
char DtrigLevel[5] = "NA";
int  DtrigDelay    = 0;
int  DtrigPeriod   = 0;
int  DtrigNumber   = 1;
int  DtrigCurrentNum;
bool DtrigEnable   = false;

// This ISR is called when the timer reaches its terminal count, or max count
void DelayedTriggerTMR_ISR(void)
{
  if(DelayedTrigFunction != NULL) DelayedTrigFunction();
  DtrigCurrentNum++;
  if(DtrigNumber == 0) return;
  if(DtrigCurrentNum >= DtrigNumber) DelayTriggerTMR->stop();
  DelayTriggerTMR->setRC((DtrigPeriod * 105)/10);
}

// This function is called as a result of the hardware trigger and calls the trigger function and processes any 
// timer needs
void DelayedTriggerISR(void)
{
  if(!DtrigEnable) return;
  if(DtrigDelay == 0)
  {
    // If here there is no trigger delay so call the function
    if(DelayedTrigFunction != NULL) DelayedTrigFunction();
    if(DtrigNumber == 1) DelayTriggerTMR->stop();
    else
    {
      DelayTriggerTMR->setRC((DtrigPeriod * 105)/10);
      DelayTriggerTMR->begin();
      DelayTriggerTMR->setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);   // 10.5 MHz clock
      DelayTriggerTMR->enableTrigger();
      DelayTriggerTMR->softwareTrigger();
    }
    DtrigCurrentNum = 1;    
    return;
  }
  else
  {
    // If here setup the timer with the trigger delay
    DelayTriggerTMR->setRC((DtrigDelay * 105)/10);
    DelayTriggerTMR->begin();
    DelayTriggerTMR->setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);   // 10.5 MHz clock
    DelayTriggerTMR->enableTrigger();
    DelayTriggerTMR->softwareTrigger();
  }
  DtrigCurrentNum=0;
}

// This function sets the delay trigger input and level. Any current trigger seetings and 
// actions are stopped.
// Valid trigger inputs are Q thru X, and t to be triggered by the Trigger output this
// is used for the MALDI2 system
void SetDelayTrigInput(char *input, char *level)
{
  int di,dil;
  
  // Validate input values
  if(input[0] != 't')
  {
     di = FindInList(DIlist, input);
     dil = FindInList(DILlist, level);
     if((di == -1) || (dil == -1))
     {
       SetErrorCode(ERR_BADARG);
       SendNAK;  
       return;
     }
  }
  // Allocate handler and timer if null
  if(DelayTriggerTMR == NULL)
  {
    DelayTriggerTMR = new MIPStimer(TMR_DelayedTrigger);
    DelayTriggerTMR->begin();
    DelayTriggerTMR->attachInterrupt(DelayedTriggerTMR_ISR);
    DelayTriggerTMR->setTrigger(TC_CMR_EEVTEDG_NONE);
    DelayTriggerTMR->setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);   // 10.5 MHz clock
  }
  DelayTriggerTMR->stop();
  DtrigEnable = false;
  if(input[0] == 't')
  {
    // Queue trigger function to start the delay
    QueueTpulseFunction(DelayedTriggerISR,true);
  }  
  else
  {
     QueueTpulseFunction(DelayedTriggerISR,false);
     if(DIdelayedTrigger == NULL) DIdelayedTrigger = new DIhandler;
     if((DelayTriggerTMR == NULL) || (DIdelayedTrigger == NULL))
     {
       SetErrorCode(ERR_INTERNAL);
       SendNAK;  
       return;
     }
     // Setup trigger
     DIdelayedTrigger->detach();
     DIdelayedTrigger->attached(input[0], dil-2, DelayedTriggerISR);
  }
  SendACK;
  return;
}

// This function processes delay trigger enable and disable commands.
void SetDelayTrigEnable(char *sena)
{
  String Sena;

  Sena = sena;
  // If false disable the timer and set the enable flag to false
  if(Sena == "FALSE")
  {
    DtrigEnable = false;
    DelayTriggerTMR->stop();
    SendACK;
    return;
  }
  else if(Sena == "TRUE")
  {
    // Here with enable request
    DtrigEnable = true;
    SendACK;
    return;    
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

// This function sets the module to trigger with the delay trigger function.
void SetDelayTrigModule(char *module)
{
  String Module;

  Module = module;
  if(Module == "ARB") DelayedTrigFunction = ARBsyncISR;
  else if(Module == "SWEEP") DelayedTrigFunction = ARBTWAVEsweepISR;
  else if(Module == "ADC") DelayedTrigFunction = ADCtrigger;
  else if(Module == "AUXTRIG") DelayedTrigFunction = AuxTrigger;
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;  
    return;    
  }
  SendACK;
}

//
// End of delay trigger functions
//

//
// BMP image loading functions
//

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

// MIPS display is 320 x 240 pixels in size.
// This function requires the bmp file to be 24 bit
// depth and use no compression.

#define BUFFPIXEL 20

// Returns false if any error is detected
bool bmpDraw(char *filename, uint8_t x, uint8_t y) 
{
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0;

  if((x >= tft.width()) || (y >= tft.height())) return(false);

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL)
  {
    return(false);
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) 
  { // BMP signature
    read32(bmpFile);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    // Read DIB header
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) 
    { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) 
      { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;
        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) 
        {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) 
        { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) 
          { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          for (col=0; col<w; col++) 
          { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) 
            { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }
            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }
  bmpFile.close();
  if(!goodBmp) return(false);
  return(true);
}

// This function reports details of the bmp immage file as well
// defining the MIPS display requirements.

void bmpReport(char *filename) 
{
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  boolean  goodBmp = false;       // Set to true on valid header parse


  serial->println();
  serial->println("MIPS display details:");
  serial->println("  display size: 320 x 240");
  serial->println("  color graphics");
  serial->println("  bmp file must have 24 bit depth");
  serial->println("  and use no compression");

  
  serial->println();
  serial->print("Image file '");
  serial->print(filename);
  serial->println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL)
  {
    serial->print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) 
  { // BMP signature
    serial->print("File size: "); serial->println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    serial->print("Image Offset: "); serial->println(bmpImageoffset, DEC);
    // Read DIB header
    serial->print("Header size: "); serial->println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) 
    { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      serial->print("Bit Depth: "); serial->println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) 
      { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        serial->print("Image size: ");
        serial->print(bmpWidth);
        serial->print('x');
        serial->println(bmpHeight);
        serial->println("BMP format is valid.");
      } // end goodBmp
    }
  }
  bmpFile.close();
  if(!goodBmp) serial->println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) 
{
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) 
{
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

//
// End of BMP functions
//

// DMA spi transfer functions
/** Disable DMA Controller. */
static void dmac_disable() {
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
/** Enable DMA Controller. */
static void dmac_enable() {
  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
/** Disable DMA Channel. */
static void dmac_channel_disable(uint32_t ul_num) {
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}
/** Enable DMA Channel. */
static void dmac_channel_enable(uint32_t ul_num) {
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}
/** Poll for transfer complete. */
static bool dmac_channel_transfer_done(uint32_t ul_num) {
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
}

//
// SPI DMA functions, used for DAC output on DCbias modules
//

void spiDMAinit(void)
{
    pmc_enable_periph_clk(ID_DMAC);
    dmac_disable();
    DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
    dmac_enable();  
}

void (*DMAisr)() = NULL;

// DMA ISR is used to reenable the next buffer. This interrupt fires when
// a buffer completes if defined a user ISR is called.
void DMAC_Handler(void)
{
   if(DMAisr != NULL) DMAisr();
}

// The function sends a buffer to the SPI interface. The transfers are 32 bit
// and the SPI defines the transfer width. The buffer contains a 32 bit word for
// each transfer with the LSB being the data and the MSB are the chip select and 
// flags.
// Example use:
//  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(SPI_CS);
//  uint32_t dbuf[4];
//  dbuf[0] = (uint32_t)buf[0] | SPI_PCS(ch);
//  dbuf[1] = (uint32_t)buf[1] | SPI_PCS(ch);
//  dbuf[2] = (uint32_t)buf[2] | SPI_PCS(ch);
//  dbuf[3] = (uint32_t)buf[3] | SPI_PCS(ch) | SPI_TDR_LASTXFER;
//  spiDmaTX(&dbuf[0], 4);
//  spiWait();
//
void spiDmaTX(uint32_t* src, uint16_t count,void (*isr)()) 
{
  dmac_channel_disable(SPI_DMAC_TX_CH);
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_SADDR = (uint32_t)src;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DSCR =  0;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLB =  DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |
    DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CFG = DMAC_CFG_DST_PER(SPI_TX_IDX) |
      DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ALAP_CFG;

  // If isr is != NULL then enable interrupts and call the isr routine
  // when transfer completes
  DMAisr = isr;
  if(isr != NULL)
  {
      int i = DMAC->DMAC_EBCISR;

      DMAC->DMAC_EBCIER |= 1 << (SPI_DMAC_TX_CH);

      NVIC_ClearPendingIRQ(DMAC_IRQn);
      NVIC_EnableIRQ(DMAC_IRQn);
  }

  dmac_channel_enable(SPI_DMAC_TX_CH);
}

void spiDmaWait(void)
{
   Spi* pSpi = SPI0;

   while (!dmac_channel_transfer_done(SPI_DMAC_TX_CH)) {}
   while ((pSpi->SPI_SR & SPI_SR_TXEMPTY) == 0) {}
   uint8_t b = pSpi->SPI_RDR;
}

// The following function support the trace mode used to capature the the program's state and display the last 16
// points captured. The TracePoints and TracePointTimes buffers are located in memory not used by the application, 
// thai has to be done to make sure this memory is never touched by the system and thus preserved during reset and
// watchdog timer reset. This requires editing the flash.ld linker file that is located in the linker scripts dir of
// the arduino development enviornment. Edit the following line:
//   ram (rwx)   : ORIGIN = 0x20070000, LENGTH = 0x00017F80 /* sram, 96K , was 18000, GAA updated*/


bool     Tracing = false;
uint8_t  TPpointer = 0;
// These addresses are above the stack and not touched by the system
uint8_t  *TracePoints = (uint8_t *)0x20087F80;
uint32_t *TracePointTimes = (uint32_t *)0x20087FA0;

// This function saves a trace point in the trace FIFO
void TraceCapture(uint8_t tp)
{
   TracePoints[TPpointer] = tp;
   TracePointTimes[TPpointer] = millis();
   TPpointer++;
   TPpointer &= 15;
}

// This function reports the trace data
void TraceReport(void)
{
  uint8_t  i,strt;

  strt = 0;
  for(i=0;i<16;i++) if(TracePointTimes[i] >= TracePointTimes[strt]) strt = i;
  strt++;
  serial->println("Trace report.");
  for(i=0;i<16;i++)
  {
    serial->print(TracePoints[strt & 15]);
    serial->print(" @ ");
    serial->println(TracePointTimes[strt & 15]);
    strt++;
  }
}

// This function is called by the command processor to enable the trace function and clear the buffers
void TraceEnable(void)
{
  uint8_t i;
  
  for(i=0;i<16;i++)
  {
    TracePoints[i] = 0;
    TracePointTimes[i] = 0;
  }
  TPpointer = 0;
  Tracing = true;
  SendACK;
}

// The following function support reading and writting the EEPROM module memory to
// the SD card or to the MIPS host app using the USB interface.

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Compute CRC one byte at a time
void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// This function reads the EEPROM contents from the selected module and writes
// the data to a file on the SD card. If the file is present it will be overwritten.
// Returns 0 is operation completes with no errors.
int SaveEEPROMtoSD(char *FileName, uint8_t board, uint8_t EEPROMadd)
{
  uint8_t buf[512];
  File file;
    
  // Read the EEPROM
  SelectBoard(board);
  if(ReadEEPROM(buf, EEPROMadd, 0, 512) != 0) return ERR_EEPROMREAD;
  // Write the data to SD card
  // Test SD present flag, exit and NAK if no card or it failed to init
  if(!SDcardPresent) return ERR_NOSDCARD;
  AtomicBlock< Atomic_RestoreState > a_Block;
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(FileName);
  // Open file and write config structure to disk
  if(!(file=SD.open(FileName,FILE_WRITE))) return ERR_CANTCREATEFILE;
  file.write((byte *)buf,512);
  file.close();
  return(0);
}

// This function writes the EEPROM contents on the selected module from the
// the selected file on the SD card. 
// Returns 0 is operation completes with no errors.
int LoadEEPROMfromSD(char *FileName, uint8_t board, uint8_t EEPROMadd)
{
  uint8_t buf[512];
  int     i,fVal;
  File    file;
    
  // Read file from SD card
  if(!SDcardPresent) return ERR_NOSDCARD;
  {
     AtomicBlock< Atomic_RestoreState > a_Block;
     SD.begin(_sdcs);
     // Open the file
     if(!(file=SD.open(FileName,FILE_READ))) return ERR_CANTOPENFILE;
     // read the data
     for(i=0;i<512;i++)
     {
       if((fVal = file.read()) == -1) break;
       buf[i] = fVal;
     }
     file.close();
  }
  if(i != 512) return ERR_READINGSD;
  // Write data to EEPROM
  SelectBoard(board);  
  if(WriteEEPROM(buf, EEPROMadd, 0, 512) != 0) return ERR_EEPROMWRITE;
  return 0;
}

// This function saves the EEPROM data to the SD card.
// This function is called by the serial command processor with the parameters
// in the ring buffer. This routine expects the following:
// FileName
// Module address, A or B
// EEPROM TWI address, hex
void EEPROMtoSD(void)
{
   char   *Token,Module;
   String sToken,FileName;
   int    add,err,brd;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     FileName = Token;
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     Module = toupper(Token[0]);
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sscanf(Token,"%x",&add);
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the Module range
     if((Module != 'A') && (Module != 'B')) break;
     // Now we can call the function!
     brd = 0;
     if(Module == 'B') brd = 1; 
     if((err = SaveEEPROMtoSD((char *)FileName.c_str(), brd, add)) != 0)
     {
       SetErrorCode(err);
       SendNAK;
       return;
     }
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

// This function restores the EEPROM data from the SD card.
// This function is called by the serial command processor with the parameters
// in the ring buffer. This routine expects the following:
// FileName
// Module address, A or B
// EEPROM TWI address, hex
void SDtoEEPROM(void)
{
   char   *Token,Module;
   String sToken,FileName;
   int    add,err,brd;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     FileName = Token;
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     Module = toupper(Token[0]);
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sscanf(Token,"%x",&add);
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the Module range
     if((Module != 'A') && (Module != 'B')) break;
     // Now we can call the function!
     brd = 0;
     if(Module == 'B') brd = 1; 
     if((err = LoadEEPROMfromSD((char *)FileName.c_str(), brd, add)) != 0)
     {
       SetErrorCode(err);
       SendNAK;
       return;
     }
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

// Lists all files found on the SD card
void ListFiles(void)
{
  File root, entry;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    SD.begin(_sdcs);
    root = SD.open("/", FILE_READ);
    root.rewindDirectory();
  }
  SendACKonly;
  while (true)
  {
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      entry = root.openNextFile();
    }
    if (!entry) break;
    serial->print(entry.name());
    serial->print(", ");
    serial->println(entry.size());
  }
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      root.close();  
    }
}

File root;
int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

void rm(File dir, String tempPath) {
  while(true) {
    WDT_Restart(WDT);
    File entry =  dir.openNextFile();
    String localPath;

    serial->println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if( SD.rmdir( folderBuf ) )
        {
          serial->print("Deleted folder ");
          serial->println(folderBuf);
          FolderDeleteCount++;
        } 
        else
        {
          serial->print("Unable to delete folder ");
          serial->println(folderBuf);
          FailCount++;
        }
      } 
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if( SD.remove( charBuf ) )
        {
          serial->print("Deleted ");
          serial->println(localPath);
          DeletedCount++;
        } 
        else
        {
          serial->print("Failed to delete ");
          serial->println(localPath);
          FailCount++;
        }

      }
    } 
    else {
      // break out of recursion
      break;
    }
  }
}

void DeleteFile(char *FileName)
{
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  if(strcmp(FileName,"*") == 0)
  {
    // Delete all files on SD card

    root = SD.open("/");
    delay(2000);
    WDT_Restart(WDT);

    rm(root, rootpath);
    SendACK;
    return;
  }
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Remove the existing default.cfg file
  SD.remove(FileName); 
  SendACK; 
}

// This function will send the selected file to the USB port. The file is assumed to
// be binary and its contents are converted to hex and send. after the ACK is sent.
// After the ACK, the files size is sent as an ascii string with a EOL termination, then
// the data block is sent as ascii hex followed by a EOL, and finally the 8 bit CRC is
// sent as a byte followed by a EOL.
void GetFile(char *FileName)
{
  byte     b,crc=0;
  int      i,fVal,fsize;
  File     file;
  char     sbuf[3];
  uint32_t start;
    
  // Open the file and read its size
  // Read file from SD card
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.begin(_sdcs);
    // Open the file
    if(!(file=SD.open(FileName,FILE_READ)))
    {
      SetErrorCode(ERR_CANTOPENFILE);
      SendNAK;
      return;
    }
    SendACK;
    serial->println(fsize = file.size());
  }
  // read the data
  for(i=0; i<fsize; i++)
  {
    {
      AtomicBlock< Atomic_RestoreState >   a_Block;
      fVal = file.read();
    }
    if(fVal == -1) break;
    b = fVal;
    ComputeCRCbyte(&crc,b);
    sprintf(sbuf,"%02x",b);
    serial->print(sbuf);
    if((i>0) && ((i%1024) == 0))
    {
      // Halt and wait for "Next" request. Timeout after
      // 10 sec with no ation and exit
      start = millis();
      while(RB.Commands == 0)
      {
         if(millis() > start + 10000)
         {
            AtomicBlock< Atomic_RestoreState >   a_Block;
            serial->println("\nFile sending to host timedout!");
            file.close();
            return;
         }
         ReadAllSerial();
      }
      GetToken(false);
      GetToken(false);
    }
  }
  serial->println("");
  serial->println(crc);
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
  }
  serial->print(FileName);
  serial->println(" file send to host!");
}

// The function will receive a file from the USB connected host. The file must be sent
// in hex and use the following format:
// First the file name and file size, in bytes (decimal) are sent. If the file can
// be created an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct the file will be saved.
// If the file is already present it will be overwitten.
void PutFile(char * FileName,char *Fsize)
{
  File   file;
  String sToken;
  int    numBytes,val,tcrc;
  char   c,buf[3],*Token;
  byte   b,crc=0;
  uint32_t start;
  
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.begin(_sdcs);
    // Remove the existing default.cfg file
    SD.remove(FileName);
    // Open file and write config structure to disk
    if(!(file=SD.open(FileName,FILE_WRITE)))
    {
      SetErrorCode(ERR_CANTCREATEFILE);
      SendNAK;
      return;
    }
  }
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    b = val;
    {
      AtomicBlock< Atomic_RestoreState >   a_Block;
      file.write(b);
    }
    ComputeCRCbyte(&crc,b);
    WDT_Restart(WDT);
    if((i>0) && (numBytes > 512) && (((i+1)%512)==0)) serial->println("Next");
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->print(FileName);
       serial->println(" file received from host and saved.");
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.remove(FileName);
  }
  return;
TimeoutExit:
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
    SD.remove(FileName);
  }
  serial->println("\nFile receive from host timedout!");
  return;
}

// This function will save all the module's EEPROM data to files on the SD card.
// All modules found are saved with the following naming convention:
// Name_board_add where
// Name = first three chars for name found in EEPROM
// board = A or B
// add = hex address of EEPROM
void SaveAlltoSD(void)
{
  char    filename[10];
  uint8_t addr;
  char    signature[20];
  int     err;

  // Loop through all the addresses looking for signatures
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 20) == 0)
    {
      // Build file name
      for(int i=0; i<3; i++) filename[i] = signature[2+i];
      filename[3] = '_';
      filename[4] = 'A';
      filename[5] = '_';
      sprintf(&filename[6],"%02x",addr);
      // Save to SD
      serial->print("Saving: ");
      serial->println(filename);
      err = SaveEEPROMtoSD(filename, 0, addr);
      if(err != 0)
      {
        SetErrorCode(err);
        SendNAK;
        return;   
      }
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 20) == 0)
    {
      // Build file name
      for(int i=0; i<3; i++) filename[i] = signature[2+i];
      filename[3] = '_';
      filename[4] = 'B';
      filename[5] = '_';
      sprintf(&filename[6],"%02x",addr);
      // Save to SD
      serial->print("Saving: ");
      serial->println(filename);
      err = SaveEEPROMtoSD(filename, 1, addr);    
      if(err != 0)
      {
        SetErrorCode(err);
        SendNAK;
        return;   
      }
    }
  }
  SendACK;
}

// This function will search the SD card for files with the following format:
// Name_board_add where
// Name = first three chars for name found in EEPROM
// board = A or B
// add = hex address of EEPROM
// If found and the size is 512 then the files will be loaded into module EEPROM.
void LoadAllfromSD(void)
{
  File root, entry;
  int  board, addr;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  root = SD.open("/", FILE_READ);
  SendACKonly;
  root.rewindDirectory();
  while (true)
  {
    if (!(entry = root.openNextFile())) break;
    // Test the name to see if its a valid filename
    if((strlen(entry.name()) == 8) && (entry.name()[3] == '_') && (entry.name()[5] == '_') && (entry.size() == 512))
    {
      if((toupper(entry.name()[4]) == 'A') || (toupper(entry.name()[4]) == 'B'))
      {
        board = 0;
        if(toupper(entry.name()[4]) == 'B') board = 1;
        sscanf(&entry.name()[6],"%x",&addr);
        serial->print("Writing: ");
        serial->println(entry.name());
        //entry.close();
        int err = LoadEEPROMfromSD(entry.name(), board, addr);    
        if(err != 0)
        {
          serial->print("Error writting EEPROM: ");
          serial->println(err);
        }
      }
    }
  }
  root.close();  
  SendACK;
}

// This function will send the selected EEPROM contents to the host using the
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
// brd is A or B
// add is 0x50, 0x52, 0x54, or 0x56
void EEPROMtoSerial(char *brd, char *add)
{
  char sbuf[3];
  int  addr,board;
  byte buf[512];

  sscanf(add,"%x",&addr);
  // Check the inputs and exit if error
  if(((toupper(brd[0]) != 'A') && (toupper(brd[0] != 'B')) || (addr < 0x50) || (addr > 0x56) || ((addr & 1) !=0)))
  {
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;       
  }
  board = 0;
  if(toupper(brd[0]) == 'B') board = 1;
  // Read the EEPROM data
  SelectBoard(board);
  if(ReadEEPROM(buf, addr, 0, 512) != 0) 
  {
    SetErrorCode(ERR_EEPROMREAD);
    SendNAK;
    return;
  }
  SendACK;
  // Send the filesize
  serial->println(512);
  // Send the data as hex
  for(int i=0; i<512; i++)
  {
    sprintf(sbuf,"%02x",buf[i]);
    serial->print(sbuf);
  }
  serial->println("");
  // Send the CRC then exit
  serial->println(ComputeCRC(buf,512));
}

// This function will receive the selected EEPROM contents from the host using the
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
// brd is A or B
// add is 0x50, 0x52, 0x54, or 0x56
void SerialtoEEPROM(char *brd, char *add)
{
  char sbuf[3],*Token,c;
  int  addr,board,numBytes,val,crc;
  byte buf[512];
  uint32_t start;

  sscanf(add,"%x",&addr);
  // Check the inputs and exit if error
  if(((toupper(brd[0]) != 'A') && (toupper(brd[0] != 'B')) || (addr < 0x50) || (addr > 0x56) || ((addr & 1) !=0)))
  {
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;       
  }
  board = 0;
  if(toupper(brd[0]) == 'B') board = 1;
  SendACK;
  start = millis();
  // Receive the number of bytes
  while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
  sscanf(Token,"%d",&numBytes); 
  GetToken(true); // Get the \n and toss
  // Read the data block
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sbuf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sbuf[1] = c;
    sbuf[2] = 0;
    sscanf(sbuf,"%x",&val);
    buf[i & 511] = val;
  }
  start = millis();
  // Now we should see an EOL, \n
  while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
  if(c == '\n')
  {
    // Get CRC and test
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sscanf(Token,"%d",&crc);
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    if((Token[0] == '\n') && (crc == ComputeCRC(buf,512)))
    {
      // Write the EEPROM buffer
      SelectBoard(board);  
      if(WriteEEPROM(buf, addr, 0, 512) == 0)
      {
        serial->println("EPROM data written!");
        return;
      }
    }
  }
  serial->println("Unable to write to EEPROM!");
  return;
TimeoutS2E:
  serial->println("\nEEPROM data receive from host timedout!");
  return;
}

void CPUtemp(void) 
{
  /* Enable ADC channel 15 and turn on temperature sensor */
  ADC->ADC_CHER = 1 << 15;
  ADC->ADC_ACR |= ADC_ACR_TSON;
  /* Start conversion. */
  ADC->ADC_CR = ADC_CR_START;
  /* Wait for end of the conversion. */
  while (ADC->ADC_ISR & ADC_ISR_EOC15 == ADC_ISR_EOC15);
  delay(100); // Keep this delay      
  /* Read the value. */ 
  int mV = ADC->ADC_LCDR;
  /* Start conversion. */
  ADC->ADC_CR = ADC_CR_START;
  /* Wait for end of the conversion. */
  while (ADC->ADC_ISR & ADC_ISR_EOC15 == ADC_ISR_EOC15);
  delay(100); // Keep this delay      
  /* Read the value. */ 
  mV = ADC->ADC_LCDR;
  /* Disable channel 15. */
  ADC->ADC_CHDR = 1 << 15; 

  float treal = (( (3300 * mV)/4096 ) - 800) * 0.37736 + 25.5;
  SendACKonly;
  if(!SerialMute) serial->println(treal);
}

void TWIreset(void)
{
  TWI_RESET();
  Wire.begin();
  Wire.setClock(100000);
  SendACK;
}
