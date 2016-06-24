//
// Hardware
//
//  This file contains low level hardware IO drivers. This includes analog and digitial IO devices.
//  The general calibration functions are in this file and these functions are used for all the 
//  calibration function. These functions all calibration of the outputs as well as the readbacks.
//
// To do list:
//  1.) Add general input interrupt processing. This capability needs the following features:
//      a.) Develop a enable function that will:
//          - accept a digitial input channel A - P
//          - define trigger type, high,low,pos,neg,edge
//          - define the call back function when event happens
//          - return true if it is setup or false if there is a setup error.
//      b.) Add a release function to remove a call back and if its the last call back detach the interrupt
//      c.) This capability allows multiple call backs for one input interrupt the only requirement is the 
//          trigger type has to match for all the call backs
//      d.) It would be cool to develop this as a class!
//
// Gordon Anderson
//
#include "Hardware.h"

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

MIPStimer FreqBurst(3);

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
  TriggerOut(PulseWidth);
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
  {" Set output to", 0, 4, D_INT, 0, 65535, 1, 20, false, "%d", &ZeroCalValue, NULL, NULL},
  {" Set output to", 0, 5, D_INT, 0, 65535, 1, 20, false, "%d", &MidCalValue, NULL, NULL},
  {" Exit", 0, 7, D_DIALOG, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"", 0, 2, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Abort", 0, 8, D_DIALOG, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"       Calibrating", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {NULL},
};

DialogBox CalibrationDialog = {
  {"Calibration Menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0, CalibrationDialogEntries
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
    WDT_Restart(WDT);
    LDAClow;
    if (CalibrationDialog.Selected == 0)
    {
      if (CC->DACout != NULL)
      {
        if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACZeroCounts);
        else { AD5625(CC->DACaddr, CC->DACout->Chan, DACZeroCounts); delay(100);}
      }
      if (CC->ADCreadback != NULL) ADCZeroCounts = CC->ADCpointer(CC->ADCaddr, CC->ADCreadback->Chan, 100);
    }
    if (CalibrationDialog.Selected == 1)
    {
      if (CC->DACout != NULL)
      {
        if (CC->DACaddr < 8) AD5668(CC->DACaddr, CC->DACout->Chan, DACMidCounts);
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
            if (CC->DACaddr < 8) CC->ADCpointer(CC->DACaddr, CC->DACout->Chan, DACZeroCounts);
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
            if (CC->DACaddr < 8) CC->ADCpointer(CC->DACaddr, CC->DACout->Chan, DACMidCounts);
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
  static Pio *pio = NULL;
  static uint32_t pin;

  if(pio==NULL)  
  {
    pio = g_APinDescription[BRDSEL].pPort;
    pin = g_APinDescription[BRDSEL].ulPin;
  }
  AtomicBlock< Atomic_RestoreState > a_Block;
  if (Board == 1) pio->PIO_CODR = pin;    // Set pin low;
  else pio->PIO_SODR = pin;               // Set pin high;
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
  // On PCB red LED
//  pinMode(RED_LED, OUTPUT);
  // Misc control lines
  pinMode(ADDR0, OUTPUT);
  pinMode(ADDR1, OUTPUT);
  pinMode(ADDR2, OUTPUT);
  pinMode(LDAC, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(BRDSEL, OUTPUT);
  pinMode(SPI_CS, OUTPUT);
  pinMode(PWR_ON, OUTPUT);
  pinMode(A11, OUTPUT);
  pinMode(TRGOUT, OUTPUT);
  digitalWrite(A11, HIGH);
  pinMode(RFON, OUTPUT);
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
  pinMode(ADDR0, INPUT);
  pinMode(ADDR1, INPUT);
  pinMode(ADDR2, INPUT);
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
}

void Software_Reset()
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
  static Pio *pio = NULL;
  /*
  if ((addr & 4) != 0) digitalWrite(ADDR2, HIGH);
  else digitalWrite(ADDR2, LOW);
  if ((addr & 2) != 0) digitalWrite(ADDR1, HIGH);
  else digitalWrite(ADDR1, LOW);
  if ((addr & 1) != 0) digitalWrite(ADDR0, HIGH);
  else digitalWrite(ADDR0, LOW);
  */
  // The above commented out code takes 10uS to execute! The code below
  // is much faster (3uS) and assumes ADDR0-ADDR2 are D.0-D.2
  if(pio==NULL)  pio = g_APinDescription[ADDR0].pPort;
  pio->PIO_CODR = 7;    // Set all bits low
  pio->PIO_SODR = addr & 7;    // Set bit high

}

// Clears the digitial output shift registers.
void ClearDOshiftRegs(void)
{
  pinMode(SCL,OUTPUT);
  digitalWrite(SCL, LOW);
  digitalWrite(SCL, HIGH);
  // Pulse the latch lines
  pinMode(DAC0,OUTPUT);
  digitalWrite(DAC0,LOW);
  digitalWrite(DAC0,HIGH);
  PulseLDAC;
  // This is used on REV 3.0 controller to disable output during boot
  pinMode(44,OUTPUT);
  digitalWrite(44,LOW);
  // This is DI6 used to drive the enable after its cut free from the buffer.
  pinMode(46,OUTPUT);
  digitalWrite(46,LOW);
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
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Set the address to 6
  SetAddress(6);
  // Set mode
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  // Set the data
  SPI.transfer(SPI_CS, MSB, SPI_CONTINUE);
  SPI.transfer(SPI_CS, LSB);
  delayMicroseconds(2);
  // For rev 3.0 controller output using address 7 and stobe with output A12.
  SetAddress(7);
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  SPI.transfer(SPI_CS, MSB);
  pinMode(DAC0,OUTPUT);
  digitalWrite(DAC0,LOW);
  digitalWrite(DAC0,HIGH);
  delayMicroseconds(2);
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

  num = count;
  bval = (byte *)src;
  delay(10);
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
  /*
  // Set SDA as input, clock as output
  TWI_SDA_IN;
  TWI_SCL_OUT;
  for(int i = 0; i < 10; i++)
  {
    if(TWI_SDA_data == HIGH) break;
    TWI_SCL_HI;
    TWI_SCL_LOW;
    TWI_SCL_HI;
  }
  TWI_STOP();
  return;
  */
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

// The following routines support the Analog Devices DAC and ADC used to monitor and
// control voltages in the MIPS system.

// The AD7998 is 8 channel ADC. A 2.5 volt reference is used.
//
// This function outputs a value to the selected channel.
// adr = TWI address of device
// vals = pointer to a memory block where results are saved.
//        the values are 0 to 65535 reguardless of ADC resolution
//
// Return the status of the TWI transaction, 0 if no errors.
//
// Note: ARM byte order is LSB then MSB!
//
// The Arduino Wire driver will not work to drive this device. I used "bit banging" TWI
// routines defined above. This device requires no stop condition between the conversion
// write and the read of data.
int AD7998(int8_t adr, uint16_t *vals)
{
  int   iStat, i;
  byte  *bvals;

  for (i = 0; i < 8; i++)
  {
    AD7998(adr, i);
    vals[i] = AD7998(adr, i);
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
  int   iStat, i;
  byte  *bvals;

  for (i = 0; i < 4; i++)
  {
    AD7994(adr, i);
    vals[i] = AD7994(adr, i);
  }
  return (0);
}

int AD7994(int8_t adr, int8_t chan)
{
  int   iStat, i;
  unsigned int val;

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
  return (val);
}

// This function reads one channel from the AD7998 ADC
int AD7998_b(int8_t adr, int8_t chan)
{
  unsigned int val;
  
  AcquireTWI();
  Wire.beginTransmission(adr);
  Wire.endTransmission(false);

  Wire.requestFrom(adr, 2, 0x80 | chan << 4, 1);

  while (Wire.available() < 2);

  val = (Wire.read() << 8) & 0xFF00;
  val |= (Wire.read()) & 0xFF;
  val &= 0xFFF;
  val <<= 4;
  Wire.endTransmission();
  ReleaseTWI();
  return(val);
}

// This function reads one channel from the AD7998 ADC
int AD7998(int8_t adr, int8_t chan)
{
  int   iStat, i;
  unsigned int val;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  AcquireTWI();
  while (1)
  {
//    AtomicBlock< Atomic_RestoreState > a_Block;
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
  Wire.beginTransmission(adr);
  Wire.write((Cmd << 3) | chan);
  //    if(chan <= 3) val <<= 4;
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
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
  }
  val = vali;
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  // Set the address
  SetAddress(spiAdr);
  // Fill buffer with data
  buf[0] = Cmd;
  buf[1] = (DACchan << 4) | (val >> 12);
  buf[2] = (val >> 4);
  buf[3] = (val << 4);
  // Send the data
  SPI.transfer(SPI_CS, buf, 4);
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
// Routines used to allow TWI use in ISR. 
//
#define MaxQueued 5

bool TWIbusy=false;
void  (*TWIqueued[MaxQueued])(void) = {NULL,NULL,NULL,NULL,NULL};
// This functoin acquires the TWI interface.
// Returns false if it was busy.
bool AcquireTWI(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy) return(false);
  TWIbusy = true;
  return(true);
}

// This routine releases the TWI interface and if there are any queued functions they are called
// and then the queued pointer it set to NULL. This allows an ISR to use this function
// and queue up its io if the TWI interface is busy.
void ReleaseTWI(void)
{
  static void (*function)();
  int i;
  
  AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy)
  {
    for(i=0;i<MaxQueued;i++)
    {
      if(TWIqueued[i] != NULL) 
      {
        function = TWIqueued[i];
        TWIqueued[i] = NULL;
        function();
        TWIqueued[i] = NULL;
      }
    }
    TWIbusy=false;
  }
}

// This queues up a function to call when the currect TWI operation finishes.
void TWIqueue(void (*TWIfunction)())
{
  int i;

  for(i=0;i<MaxQueued;i++)
  {
     if(TWIqueued[i] == NULL) 
     {
      TWIqueued[i] = TWIfunction;
      break;
     }
  }
}

// Trigger output functions. These functions support pulsing the output in micro second units as
// well as queueing up the function for later execution, this support using the trigger outlut
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
  
//  if (MIPSconfigData.Rev < 2)
//  {
//    SetErrorCode(ERR_NOTSUPPORTINREV);
//    SendNAK;
//    return;
//  }
  Cmd = cmd;
  uS = Cmd.toInt();
  if(uS >0)
  {
     SendACK;
     TriggerOut(uS);
     return;
  }
  if ((strcmp(cmd, "HIGH") == 0) || (strcmp(cmd, "LOW") == 0) || (strcmp(cmd, "PULSE") == 0))
  {
    SendACK;
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

// This function will pulse the trigger output 
void TriggerOut(int microSec)
{
  static Pio *pio = pio = g_APinDescription[TRGOUT].pPort;
  static uint32_t pin = pin = g_APinDescription[TRGOUT].ulPin;
  static uint32_t dwReg;

  AtomicBlock< Atomic_RestoreState > a_Block;
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
void DIOopsReport(void)
{
  int  i;
  char chan;
  
  for(i=0;i<8;i++)
  {
    chan = 'Q' + i;
    if((dioops[i].Report) && (dioops[i].Changed))
    {
      serial->print("DIC,");
      serial->print(chan);
      if(dioops[i].ReportState == CHANGE) serial->print(",CHANGED,");
      if(dioops[i].ReportState == RISING) serial->print(",RISING,");
      if(dioops[i].ReportState == FALLING) serial->print(",FALLING,");
      serial->println(millis());
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

  for(i=0;i<8;i++)
  {
    if((dioops[i].Report) || (dioops[i].Mirror))
    {
      j = digitalRead(dioops[i].DI);
      if(j != dioops[i].LastLevel)
      {
        dioops[i].LastLevel = j;
        if(dioops[i].Report)
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
void DIOreport(char *port, char *mode)
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
    dioops[i].Report = true;
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

// Laser Shutter control functions for Mike Belov's system
/*
void ShutterOpen(void)
{
  // Shutter to connected to output G, this interrupt will set G high
  // Set the image bits
  SDIO_Set_Image('G', '1');
  // Send to the hardware
  DOrefresh;
  // Toggle LDAC
  PulseLDAC;
  UpdateDigitialOutputArray();
  ShutterOpened = true;
}

void ShutterClose(void)
{
  // Shutter to connected to output G, this interrupt will set G low
  // Set the image bits
  SDIO_Set_Image('G', '0');
  // Send to the hardware
  DOrefresh;
  // Toggle LDAC
  PulseLDAC;
  UpdateDigitialOutputArray();
  ShutterClosed = true;
}

// This function is called to enable (TRUE) or disable (FALSE) the shutter
// If TRUE then the Q and R inputs are setup for positive edge trigger. 
// If false the interrupts are removed.
void ShutterEnable(char *state)
{
  if ((strcmp(state, "TRUE") !=0) && (strcmp(state, "FALSE") != 0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if (strcmp(state, "TRUE") == 0)
  {
    // Set up Q and R inputs for positive edge trigger
    attachInterrupt(DI0, ShutterOpen, RISING);
    attachInterrupt(DI1, ShutterClose, RISING);
  }
  else
  {
    // Remove the interrupts from Q and R
    detachInterrupt(DI0);
    detachInterrupt(DI1);
  }
}

*/
