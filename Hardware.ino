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
// March 26, 2019
// This function needs to be rewitten. Basicalliy it needs to
// be reduced to:
//   Wire.requestFrom(adr, 2, 0x80 | chan << 4, 1, 0);
//   val = (Wire.read() << 8) & 0xFF00;
//   val |= (Wire.read()) & 0xFF;
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
//  while(Wire.available())
//  {
//     if(i==0) j = Wire.read() << 8;
//     if(i==1) j |= Wire.read();
//     i++;
//  }
  j = Wire.read() << 8;
  j |= Wire.read();
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

// AD5593 IO routines for Wire1 interface. This is a analog and digitial IO chip with
// a TWI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5593
// Return 0 if no error else an error code is returned
int AD5593writeWire1(uint8_t addr, uint8_t pb, uint16_t val)
{
  int iStat;
  
  Wire1.beginTransmission(addr);
  Wire1.write(pb);
  Wire1.write((val >> 8) & 0xFF);
  Wire1.write(val & 0xFF);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire1.endTransmission();
  }
  return (iStat);
}

// Read from AD5593R
// returns -1 on any error
int AD5593readWordWire1(uint8_t addr, uint8_t pb)
{
  int iStat;

  Wire1.beginTransmission(addr);
  Wire1.write(pb);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire1.endTransmission();
  }
  if(iStat != 0) return (-1);
  // Now read the data word
  int i = 0,j = 0;
  Wire1.requestFrom(addr, (uint8_t)2);
//  while(Wire1.available())
//  {
//     if(i==0) j = Wire1.read() << 8;
//     else if(i==1) j |= Wire1.read();
//     if(++i>=1) break;
//  }
  j = Wire1.read() << 8;
  j |= Wire1.read();
  return(j);
}

int AD5593readADCWire1(int8_t addr, int8_t chan)
{
   int iStat;
   
   // Select the ADC channel number
   if((iStat = AD5593writeWire1(addr, 0x02, (1 << chan))) != 0) return(-1);
   // Read the data and make sure the address is correct then left 
   // justify in 16 bits
   int i = AD5593readWordWire1(addr, 0x40);
   if(((i >> 12) & 0x7) != chan) return(-1);
   i <<= 4;
   return(i & 0xFFFF);
}

int AD5593readADCWire1(int8_t addr, int8_t chan, int8_t num)
{
  int i,j,k,val=0;
  int iStat;

  // Select the ADC channel number for repeat read
  if((iStat = AD5593writeWire1(addr, 0x02, 0x200 | (1 << chan))) != 0) return(-1);
  // Read the data and make sure the address is correct then left 
  // justify in 16 bits
  Wire1.beginTransmission(addr);
  Wire1.write(0x40);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire1.endTransmission();
  }
  if(iStat != 0) return (-1);
  // Now read the data word
  k = 0;
  Wire1.requestFrom(addr, (uint8_t)2 * num);
  for(i=0;i<num;i++)
  {
    j  = Wire1.read() << 8;
    j |= Wire1.read();
    if(((j >> 12) & 0x7) != chan) return(-1);
    k += (j << 4) & 0xFFFF;
  }
  return(k/num);
}

int AD5593writeDACWire1(int8_t addr, int8_t chan, int val)
{
   uint16_t  d;
   // convert 16 bit DAC value into the DAC data data reg format
   d = (val>>4) | (chan << 12) | 0x8000;
   return(AD5593writeWire1(addr, 0x10 | chan, d));
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

// Select only one mode of operation!
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
