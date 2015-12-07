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
//  ZeroCalValue = CC->Min + (CC->Max - CC->Min) / 2;
//  MidCalValue = ZeroCalValue + (CC->Max - ZeroCalValue) / 2;
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
  if (Board == 1) ENA_BRD_B;
  else ENA_BRD_A;
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
  pinMode(RED_LED, OUTPUT);
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
  pinMode(RED_LED, INPUT);
  // Misc control lines
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
  return ((((float)ADCvalue * 3.3) / 1024.0) * 11.0);
}

// This function will set the three address lines used for the SPI device selection. It is assumed
// the bit directions have already been set.
void SetAddress(int8_t addr)
{
  if ((addr & 4) != 0) digitalWrite(ADDR2, HIGH);
  else digitalWrite(ADDR2, LOW);
  if ((addr & 2) != 0) digitalWrite(ADDR1, HIGH);
  else digitalWrite(ADDR1, LOW);
  if ((addr & 1) != 0) digitalWrite(ADDR0, HIGH);
  else digitalWrite(ADDR0, LOW);
}

// Clears the digitial output shift registers.
void ClearDOshiftRegs(void)
{
  digitalWrite(SCL, LOW);
  digitalWrite(SCL, HIGH);
}

// This function sends 16 bits to the digital IO using the SPI. Its assumes the SPI interface has been
// started.
// JP1 position 2 jumper needs to be installed on the MIPS controller hardware.
void DigitalOut(int8_t MSB, int8_t LSB)
{
  // Set the address to 6
  SetAddress(6);
  // Set mode
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  // Set the data
  SPI.transfer(SPI_CS, MSB, SPI_CONTINUE);
  SPI.transfer(SPI_CS, LSB);
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
    if (iStat != 0) return (iStat);
    if (num > 32) Wire.requestFrom(dadr | ((address >> 8) & 1), 32);
    else Wire.requestFrom(dadr | ((address >> 8) & 1), num);
    while (Wire.available())
    {
      *(bval++) = Wire.read();
      i++;
      if (i > count) return (-1);
    }
    if (num <= 32) break;
    num -= 32;
    address += 32;
  }
  if (i != count) return (-1);
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
    if (iStat != 0) return (iStat);
    // wait for it to finish writting 16 bytes or timeout
    for (int k = 0; k < 21; k++)
    {
      Wire.beginTransmission(dadr);
      Wire.write(0);
      if (Wire.endTransmission(true) == 0) break;
      if (k == 20) return (-1); // Timeout!
    }
    // Setup for the next loop
    if (num <= 16) break;
    num -= 16;
    address += 16;
  }
  if (i != count) return (-1);
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
void TWI_RESET(void)
{
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

// Write a byte to the TWI bus, this function returns true is acked and false f naked
bool TWI_WRITE(int8_t val)
{
  int8_t Response;

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
  return (val);
}

// This function reads one channel from the AD7998 ADC
int AD7998(int8_t adr, int8_t chan)
{
  int   iStat, i;
  unsigned int val;

  while (1)
  {
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
  Wire.beginTransmission(adr);
  Wire.write(chan);
  //    if(chan <= 3) val <<= 4;
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  return (Wire.endTransmission());
}

// This function enables the internal voltage reference in the
// AD5625
int AD5625_EnableRef(int8_t adr)
{
  Wire.beginTransmission(adr);
  Wire.write(0x38);
  Wire.write(0);
  Wire.write(1);
  return (Wire.endTransmission());
}

// This function writes the value to the selected channel on the selected SPI
// address to the 8 channel DAC.
void AD5668(int8_t spiAdr, int8_t DACchan, uint16_t vali)
{
  uint16_t val;
  static bool inited = false;

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
  // Set the data
  SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
  SPI.transfer(SPI_CS, (DACchan << 4) | (val >> 12), SPI_CONTINUE);
  SPI.transfer(SPI_CS, (val >> 4), SPI_CONTINUE);
  SPI.transfer(SPI_CS, (val << 4));
  SetAddress(0);
}

// The following routine supports the MCP2300 GPIO TWI device.
// This is used for the sequence generator in Twave module and
// all bits are set to output.
int MCP2300(int8_t adr, uint8_t bits)
{
  int iStat;

  Wire.beginTransmission(adr);
  Wire.write(0);    // IO direction register
  Wire.write(0);    // Set all bits to output
  if ((iStat = Wire.endTransmission()) != 0 ) return (iStat);

  Wire.beginTransmission(adr);
  Wire.write(0x0A);  // Read the output latch command
  Wire.write(bits);
  return (Wire.endTransmission());
}

// This function writes a byte to the selected register in the
// MCP2300 GPIO TWI device.
int MCP2300(int8_t adr, uint8_t reg, uint8_t bits)
{
  Wire.beginTransmission(adr);
  Wire.write(reg);     // Register
  Wire.write(bits);    // Set bits
  return (Wire.endTransmission());
}

// This function reads a byte from the selected register in the
// MCP2300 GPIO TWI device.
int MCP2300(int8_t adr, uint8_t reg, uint8_t *data)
{
  int iStat;

  Wire.beginTransmission(adr);
  Wire.write(reg);     // Register
  if ((iStat = Wire.endTransmission()) != 0 ) return (iStat);

  Wire.requestFrom(adr, 1);
  if (Wire.available()) *data = Wire.read();
  return (0);
}


