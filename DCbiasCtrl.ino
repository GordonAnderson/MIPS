#include "DCbiasCtrl.h"
#include "Arduino.h"
#include <Wire.h>
#include "Hardware.h"

#if DCBanalog

//
// This file adds support for the DCbias analog control and DCbias amplifier modules.
// These two modules provide DCbias functionality under MIPS control and allow user selected
// channels to be controlled by analog inputs. The DCbias driver is used along with code
// in this file to allow the expanded options. The parameters used for this capability
// are saved in the DCbais modules EEPROM after the DCbias structure. The EEPROM is located
// on the control module. The control module also has a 8 channel 14 bit ADC to allow 
// monitoring the analog inputs.
//
// The compiler option needs to be true to enable this code. When enabled the DCbias_init 
// function will call this files init function to install this driver. The presents of the
// TCA9534 DIO and the expected TWI address signals the presents of the proper hardware.
//
// Uses the DCbias DAC SPI address and TWI address for the AD5593.
// The ADC also uses the DAC SPI address but a different CS
//
// Calibration procudure, assumes +/-2.5V analog in for +/-250V output, before
// starting you must calibrate the offset channel if used, you must use the 
// CDCBOFF command
//  - Turn off readback testing, SDCBTEST,FALSE
//  - Set channel to ADC mode, SDCBCMD,1,ADC
//  - Apply a 1KHz sin signal, 1Vp-p
//  - Adjust the channel gain pot for 100Vp-p
//  - Remove the sin input
//  - Use the offset connect to adjust DAC to set outpit to 0, SDCBCOF,1,value
//  - Repeat this procedure for all channels
//  - Perform DCbias module calibration using host commands, CDCBCH
//  - Save all updates, SAVEM,DCB, and DCBCSAVE,1
// Next calibrate the ADC readbacks, you will need a power supply to set input voltages
// and a meter to read the actual voltage
//  - Use CALDCBCCH,1 command
//
// The offset cound in ADC mode can be calculated by dividing the measured offset by
// 0.00776. This may take a couple interation to zero the offset.
//
// To do list
//  - Add the serial mute logic to all the commands
//  - Add a offset adjust where you end the voltage, do it twice to make sure its 0
//  - Document the code and the logic
//
// Gordon Anderson
// Aug,16,2024
//


DCBiasCtrlData *dcbiasctrl[4]   = {NULL,NULL,NULL,NULL};
ADCctrlVs      *adcctrlvs[4]    = {NULL,NULL,NULL,NULL};
Thread         DCBiasCtrlThread = Thread();

DCBiasCtrlData dcbcTemplate
{
  1,sizeof(DCBiasCtrlData),0,
  {0,0,0,0,0,0,0,0},
  {
    {0,3277,0},
    {1,3277,0},
    {2,3277,0},
    {3,3277,0},
    {4,3277,0},
    {5,3277,0},
    {6,3277,0},
    {7,3277,0},
  }
};

const Commands DCBiasCtrlCmdArray[] = {
  {"DCBCLOAD",  CMDfunction, 1, (char *)dcbcRestore},             // Load saved parameters from EEPROM, ch = any channel on module
  {"DCBCSAVE",  CMDfunction, 1, (char *)dcbcSave},                // Save parameters to the EEPORM, ch = any channel on module
  {"DCBCFORMAT",CMDfunction, 1, (char *)dcbcFormat},              // Save default parameters to the EEPORM, ch = any channel on module
  {"SDCBCMD",  CMDfunctionStr, 2, (char *)dcbcSetCHmode},         // Sets a channels mode, ch, mode = ADC|MIPS
  {"GDCBCMD",  CMDfunction, 1, (char *)dcbcGetCHmode},            // Returns a channels mode, ch
  {"SDCBCOF",  CMDfunction, 2, (char *)dcbcSetCHoff},             // Sets a channels offset when in ADC mode, ch, offset in adc counts
  {"GDCBCOF",  CMDfunction, 1, (char *)dcbcGetCHoff},             // Returns a channels offset, ch
  {"GDCBCADC", CMDfunction, 1, (char *)dcbcGetADC},               // Returns a ADC value for the channel selected
  {"CALDCBCCH", CMDfunction, 1, (char *)dcbcCalibratetADC},       // Calibrates an ADC value for select channel
  {0}
};

CommandList DCBiasCtrlList = {(Commands *)DCBiasCtrlCmdArray, NULL };

  // TCA9534, I2C IO port, setup all bits to output
  // Base address 0x20, jumpers set to 0
  // address,cmd,data
  // 0x20,3,0x00
  // 0x20,1,0xFF
// TCA9534 registers
#define   TCA9534in         0
#define   TCA9534out        1
#define   TCA9534invert     2
#define   TCA9534config     3         // Set bit to 0 for output

int TCA9534write(uint8_t add, uint8_t reg, uint8_t val)
{
  AcquireTWI();
  Wire.beginTransmission(add);
  Wire.write(reg);    
  Wire.write(val);   
  ReleaseTWI();
  return(Wire.endTransmission());
}

// TLC3578IDW IO routines

// Scan all 8 channels and perform calibration as well as filtering
// Each channel calibrated to +/- 10 volts
void TLC3578IDWscan(int brd)
{
  TLC3578IDW(DCbDarray[brd]->DACspi, 0x0AA00);
  // Initial read and setup for first channel
  TLC3578IDW(DCbDarray[brd]->DACspi, 0x00A00 | (dcbiasctrl[brd]->adcCH[0].Chan << 12) & 0x7000);
  for(int i=0;i<8;i++)
  {
    float fval = Counts2Value(TLC3578IDW(DCbDarray[brd]->DACspi, 0x00A00 | (dcbiasctrl[brd]->adcCH[(i+1)&7].Chan << 12) & 0x7000), &dcbiasctrl[brd]->adcCH[i]);
    adcctrlvs[brd]->ch[i] = fval * FILTER + (1-FILTER) *  adcctrlvs[brd]->ch[i];
  }
}

int TLC3578IDW(int brd, int ch, int num)
{
  TLC3578IDW(DCbDarray[brd]->DACspi, 0x00A00 | (dcbiasctrl[brd]->adcCH[ch].Chan << 12) & 0x7000);
  int sum=0;
  for(int i=0;i<num;i++) sum += TLC3578IDW(DCbDarray[brd]->DACspi, 0x00A00 | (dcbiasctrl[brd]->adcCH[ch].Chan << 12) & 0x7000);
  return sum/num;
}

// Perform one 16 bit transaction
// Returns left justified 14bit result.
// +/- 10 volt range
int TLC3578IDW(uint8_t spiADD, int cmd)
{
  int res = 0;

  pinMode(ADC_CS,OUTPUT);
  SetAddress(spiADD);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);
    digitalWrite(ADC_CS,LOW);
    res = SPI.transfer16((uint16_t)(cmd & 0xFFFF));
    digitalWrite(ADC_CS,HIGH); 
    SPI.setDataMode(SPI_MODE2);
  }
  SetAddress(0);
  return res & 0xFFFC; 
}

void DCbiasCtrl_loop(void)
{
  for(int brd=0;brd<4;brd++)
  {
    if(adcctrlvs[brd] != NULL)
    {
      SelectBoard(brd);
      TLC3578IDWscan(brd);
    }
  }
}

int checkBoard(int CH)
{
  int brd=DCbiasCH2Brd(CH-1);
  if(dcbiasctrl[brd] == NULL) { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
  if(brd == -1) { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
  return brd;
}

int checkBoard(char *chan)
{
  String token = chan;
  return checkBoard(token.toInt()-1);
}

// This command will enable the DCbiasCtrl system for the board that contains
// the channel number passed. The TCA9534write is sent the config word to test
// if this module is installed. If an error is returned its assumed the module
// is not present in the system.
bool dcbctrlEnable(int CH)
{
  int brd;

  // Get board address for the channel passed
  if((brd=DCbiasCH2Brd(CH-1)) == -1) return false;
  // Test if hardware is installed
  SelectBoard(brd);
  if(TCA9534write(TCA9534add + (brd / 2),TCA9534config,0) != 0) 
  {
    delay(100);
    if(TCA9534write(TCA9534add + (brd / 2),TCA9534config,0) != 0) return false;
  } 
  // Allocate the data structure
  if(dcbiasctrl[brd] == NULL)
  {
    // Allocate space for array
    dcbiasctrl[brd] = new DCBiasCtrlData;
    if(dcbiasctrl[brd] == NULL) return false;
    adcctrlvs[brd] = new ADCctrlVs;
    if(adcctrlvs[brd] == NULL) return false;
    for(int i = 0;i<8;i++) adcctrlvs[brd]->ch[i] = 0;
  }
  // If this is the first defined insert commands into the command processor
  int num = 0;
  for(int i = 0;i<4;i++) if(dcbiasctrl[i] != NULL) num++;
  if(num == 1) 
  {
    // Add conmmands 
    AddToCommandList(&DCBiasCtrlList);
    // Start a thread to read the ADC channels
    // Configure Threads
    DCBiasCtrlThread.setName("DCbiasCtrl");
    DCBiasCtrlThread.onRun(DCbiasCtrl_loop);
    DCBiasCtrlThread.setInterval(100);
    // Add threads to the controller
    control.add(&DCBiasCtrlThread);
  }
  // Init the hardware, set all channels to MIPS control mode and set 
  *dcbiasctrl[brd] = dcbcTemplate;
  SelectBoard(brd);
  TCA9534write(TCA9534add + (brd / 2),TCA9534out,dcbiasctrl[brd]->chanMode);
  // DAC to internal reference
  AD5668_EnableRef(DCbDarray[brd]->DACspi);
  // Load the settings 
  dcbcLoad(brd);
  return true;
}

void dcbcFormat(int CH)
{
  int brd;
  
  if((brd=checkBoard(CH)) == -1) return;
  SelectBoard(brd);
  if(WriteEEPROM(&dcbcTemplate, DCbDarray[brd]->EEPROMadr, EEPROMOFFSET, sizeof(DCBiasCtrlData)) == 0) {SendACK;}
  else SendNAK;
}

// Save the DCbiasCtrl structure to EEPROM, this is located in the DCBias module's EEPROM, saved
// after the DCbias setings
void dcbcSave(int CH)
{
  int brd;
  
  if((brd=checkBoard(CH)) == -1) return;
  SelectBoard(brd);
  if(WriteEEPROM(dcbiasctrl[brd], DCbDarray[brd]->EEPROMadr, EEPROMOFFSET, sizeof(DCBiasCtrlData)) == 0) {SendACK;}
  else SendNAK;
}

bool dcbcLoad(int brd)
{
  DCBiasCtrlData dcb;

  SelectBoard(brd);
  if(ReadEEPROM(&dcb, DCbDarray[brd]->EEPROMadr, EEPROMOFFSET, sizeof(DCBiasCtrlData)) == 0)
  {
    // Check for valid revisions and size
    if(dcb.rev != 1) return false;
    if(dcb.size != sizeof(DCBiasCtrlData)) return false;
    *dcbiasctrl[brd] = dcb;
    // Set the channel modes
    TCA9534write(TCA9534add + (brd / 2),TCA9534out,dcbiasctrl[brd]->chanMode);
    // Set the DAC offsets
    for(int i=0;i<8;i++)
    {
      if((dcbiasctrl[brd]->chanMode & (1 << (i))) != 0)
      {
        AD5668(DCbDarray[brd]->DACspi,DCbDarray[brd]->DCCD[i].DCctrl.Chan,dcbiasctrl[brd]->offsets[i]);
        // Mask readback testing for this channel
        DCBtstMask[brd] |= 1 << (i & 7);
        // Block the DCBias driver from updating the DACs
        DCBchngMask[brd] |= 1 << (i & 7);
      }
      else
      {
        // Un-mask readback testing for this channel
        DCBtstMask[brd] &= ~(1 << (i & 7));
        // Un block the DCBias driver from updating the DACs
        DCBchngMask[brd] &= ~(1 << (i & 7));
      }
    }
    DCbiasUpdate = true;
    return true;
  }
  return false;
}

void dcbcRestore(int CH)
{
  DCBiasCtrlData dcb;
  int brd;

  if((brd=checkBoard(CH)) == -1) return;
  if(dcbcLoad(brd)) 
  {
    SendACK;
    return;
  }
  BADARG;
}

void dcbcSetCHmode(char *chan, char *mode)
{
  int brd,ch;
  String token = chan;

  // convert channel number to board address
  if((brd=checkBoard(ch = token.toInt())) == -1) return;
  ch--;
  token = mode;
  if(token=="ADC")  dcbiasctrl[brd]->chanMode  |=   1 << (ch & 7);
  else if(token=="MIPS") dcbiasctrl[brd]->chanMode  &= ~(1 << (ch & 7));
  else BADARG;
  SelectBoard(brd);
  TCA9534write(TCA9534add + (brd / 2),TCA9534out,dcbiasctrl[brd]->chanMode);
  if(token=="ADC")  
  {
    // Set this channels offset
    AD5668(DCbDarray[brd]->DACspi,DCbDarray[brd]->DCCD[ch & 7].DCctrl.Chan,dcbiasctrl[brd]->offsets[ch & 7]);
    // Mask this channel's readback testing
    DCBtstMask[brd] |= 1 << (ch & 7);
    // Block the DCBias driver from updating the DACs
    DCBchngMask[brd] |= 1 << (ch & 7);
  }
  if(token=="MIPS")
  {
    DCbiasUpdate = true;
    // Un-mask this channels readback testing
    DCBtstMask[brd] &= ~(1 << (ch & 7));
    // Un block the DCBias driver from updating the DACs
    DCBchngMask[brd] &= ~(1 << (ch & 7));
  }
  SendACK;
}

void dcbcGetCHmode(int ch)
{
  int brd;

  // convert channel number to board address
  if((brd=checkBoard(ch)) == -1) return;
  ch--;
  SendACKonly;
  if((dcbiasctrl[brd]->chanMode  & (1 << (ch & 7))) != 0) serial->println("ADC");
  else serial->println("MIPS");
}

void dcbcSetCHoff(int ch, int offset)
{
  int brd;

  // convert channel number to board address
  if((brd=checkBoard(ch)) == -1) return;
  ch--;
  dcbiasctrl[brd]->offsets[ch & 7] = offset;
  if((dcbiasctrl[brd]->chanMode & (1 << (ch & 7))) != 0)
  {
    SelectBoard(brd);
    AD5668(DCbDarray[brd]->DACspi,DCbDarray[brd]->DCCD[ch & 7].DCctrl.Chan,dcbiasctrl[brd]->offsets[ch & 7]);
  }
  SendACK;
}

void dcbcGetCHoff(int ch)
{
  int brd;

  // convert channel number to board address
  if((brd=checkBoard(ch)) == -1) return;
  ch--;
  SendACKonly;
  serial->println(dcbiasctrl[brd]->offsets[ch & 7]);
}

void dcbcGetADC(int ch)
{
  int brd;

  // convert channel number to board address
  if((brd=checkBoard(ch)) == -1) return;
  ch--;
  SendACKonly;
  serial->println(adcctrlvs[brd]->ch[ch & 7]);
}

void dcbcCalibratetADC(int ch)
{
  int brd;


  // convert channel number to board address
  if((brd=checkBoard(ch)) == -1) return;
  ch--;
  SelectBoard(brd);
  // Apply voltage 1 and read the ADC channels raw data
  float val1 = UserInputFloat("Apply voltage 1 and enter value : ");
  int adc1   = TLC3578IDW(brd, ch, 10);
  serial->println(adc1);
  // Apply voltage 2 and read the ADC channels raw data
  float val2 = UserInputFloat("Apply voltage 2 and enter value : ");
  int adc2   = TLC3578IDW(brd, ch, 10);
  serial->println(adc2);
  // Calculate and apply calibration parameters
  dcbiasctrl[brd]->adcCH[ch & 7].m = ((float)adc1 - (float)adc2) / (val1 - val2);
  dcbiasctrl[brd]->adcCH[ch & 7].b = (float)adc1 - dcbiasctrl[brd]->adcCH[ch & 7].m * val1;
  serial->println(dcbiasctrl[brd]->adcCH[ch & 7].m);
  serial->println(dcbiasctrl[brd]->adcCH[ch & 7].b);
}

#endif


