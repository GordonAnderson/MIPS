//
// Twave
//
// This file supports the Twave driver card, rev 1.0.
//
// This initial version of the Twave driver does not use the readback ADC. Also no
// host computer commands have been implemented.
//
// Needed to lower frequency below the 250KHz limit of the clock generator chip. Setup
// Timer channel 6 to output on pin 5, that is on connector EXT2 pin 9. Jumper EXT2 pin 9
// to pin 14 and then this signal can be found on pin 14 of IC10.
//
//
// Gordon Anderson
//
#include "Twave.h"
#include "Hardware.h"
#include "Menu.h"
#include "Dialog.h"
#include "pwm01.h"
#include "Variants.h"

extern bool NormalStartup;

// This define enables the use of timer6 to generate the sequence generator clock.
// Applies to rev 1 only
#define UseTimer

// Use the timer channel 6 for a clock in rev 1.0 to get to lower frequencies
MIPStimer TwaveClk(6);

// DAC channel assignment
#define  DAC_PulseVoltage    0
#define  DAC_RestingVoltage  1
#define  DAC_Guard1          2
#define  DAC_Guard2          3

#define   TWreset  ADDR0
#define   TWload   ADDR1

// Rev 2.0 bit definitions for PLD sequence generator
#define   TWdir       49
#define   TWclr       14
#define   TWld        48

#define   MinPulseVoltage    7

MenuEntry METwaveMonitor = {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog, NULL, NULL};
MenuEntry METwaveMonitor2 = {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog2, NULL, NULL};

// Twave system variables
int NumberOfTWAVEchannels = 0;
TwaveData TD = Twave_Rev1;
int  SelectedTwaveBoard = 0;    // Active board, 0 or 1 = A or B

extern Menu MainMenu;

//MIPS Threads
Thread TwaveThread  = Thread();

float MaxTwaveVoltage = 0;  // This value is set to the highest Twave output voltage

void SetVelocity(void)
{
  if (TD.Rev == 2)
  {
    SetRef(8000000);
    CY_Init(TD.CLOCKadr);
    SetPLL2freq(TD.CLOCKadr, TD.Velocity * 8);
  }
  else
  {
#ifdef UseTimer
    TwaveClk.setFrequency((double)(TD.Velocity * 2));
    TwaveClk.setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk.start(-1, 0, true);
#else
    SetRef(8000000);
    CY_Init(TD.CLOCKadr);
    SetPLL2freq(TD.CLOCKadr, TD.Velocity);
#endif
  }
}

void SetSequence(void)
{
  int   i;
  int   numTry = 0;

  if (TD.Rev == 2)
  {
    // Retry if the GPIO fails, this is needed!
    for (i = 0; i < 100; i++)
    {
      numTry++;
      if (MCP2300(TD.GPIOadr, ~TD.Sequence) == 0) break;
      delay(2);
    }
//    serial->println(TD.Sequence);
//    serial->println(numTry);
    digitalWrite(TWclr, LOW);
    digitalWrite(TWclr, HIGH);
    digitalWrite(TWclr, HIGH);
    digitalWrite(TWld, LOW);
    digitalWrite(TWld, LOW);
    digitalWrite(TWld, LOW);
    digitalWrite(TWld, HIGH);
    return; 
  }
  // Init the sequence generator pattern
  digitalWrite(TWreset, LOW);       // Clear the sequence generator
  digitalWrite(TWreset, HIGH);
  digitalWrite(TWreset, HIGH);
  digitalWrite(TWreset, HIGH);
  delay(2);
  // Retry if the GPIO fails, this is needed!
  for (i = 0; i < 100; i++)
  {
    if (MCP2300(TD.GPIOadr, ~TD.Sequence) == 0) break;
    delay(2);
  }
  digitalWrite(TWload, HIGH);       // Load the sequence generator
  digitalWrite(TWload, HIGH);
  delay(2);
  digitalWrite(TWload, HIGH);
  digitalWrite(TWload, LOW);
  digitalWrite(TWload, HIGH);
  digitalWrite(TWload, LOW);
  digitalWrite(TWload, HIGH);
}

// This function will set the Pulse voltage DAC output.
void SetPulseVoltage(void)
{
  int DACcounts;

  if (TD.TWCD[DAC_PulseVoltage].VoltageSetpoint < (TD.TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage)) TD.TWCD[DAC_PulseVoltage].VoltageSetpoint = TD.TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage;
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TD.TWCD[DAC_PulseVoltage].VoltageSetpoint, &TD.TWCD[DAC_PulseVoltage].DCctrl);
  AD5625(TD.DACadr, TD.TWCD[DAC_PulseVoltage].DCctrl.Chan, DACcounts);
}

// This function will set the Resting voltage DAC output.
void SetRestingVoltage(void)
{
  int DACcounts;

  if (TD.TWCD[DAC_PulseVoltage].VoltageSetpoint < (TD.TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage)) TD.TWCD[DAC_PulseVoltage].VoltageSetpoint = TD.TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage;
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TD.TWCD[DAC_RestingVoltage].VoltageSetpoint,  &TD.TWCD[DAC_RestingVoltage].DCctrl);
  AD5625(TD.DACadr, TD.TWCD[DAC_RestingVoltage].DCctrl.Chan, DACcounts);
}

// This function will set the Guard 1 DAC output.
void SetGuard1(void)
{
  int DACcounts;

  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TD.TWCD[DAC_Guard1].VoltageSetpoint,  &TD.TWCD[DAC_Guard1].DCctrl);
  AD5625(TD.DACadr, TD.TWCD[DAC_Guard1].DCctrl.Chan, DACcounts);
}

// This function will set the Guard 2 DAC output.
void SetGuard2(void)
{
  int DACcounts;

  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TD.TWCD[DAC_Guard2].VoltageSetpoint,  &TD.TWCD[DAC_Guard2].DCctrl);
  AD5625(TD.DACadr, TD.TWCD[DAC_Guard2].DCctrl.Chan, DACcounts);
}

// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveTwaveSettings(void)
{
  if (WriteEEPROM(&TD, 0x50, 0, sizeof(TwaveData)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

// Restore the parameters from the EEPROM on the Twave board. Read the EEPROM and make sure the board name
// matches what is expected, only load if its correct.
void RestoreTwaveSettings(void)
{
  RestoreTwaveSettings(false);
}

void RestoreTwaveSettings(bool NoDisplay)
{
  TwaveData td;

  if (ReadEEPROM(&td, 0x50, 0, sizeof(TwaveData)) == 0)
  {
    if (strcmp(td.Name, TD.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      memcpy(&TD, &td, sizeof(TwaveData));
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}


DialogBoxEntry TwaveDialogEntries[] = {
#ifdef UseTimer
  {" Clock freq, Hz", 0, 1, D_INT, 1000, 2000000, 1000, 16, false, "%7d", &TD.Velocity, NULL, SetVelocity},
#else
  {" Clock freq, Hz", 0, 1, D_INT, 250000, 2000000, 1000, 16, false, "%7d", &TD.Velocity, NULL, SetVelocity},
#endif
  {" Sequence", 0, 2, D_BINARY8, 0, 255, 1, 15, false, NULL, &TD.Sequence, NULL, NULL},
  {" Pulse voltage", 0, 3, D_FLOAT, MinPulseVoltage, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[0].VoltageSetpoint, NULL, SetPulseVoltage},
  {" Resting voltage", 0, 4, D_FLOAT, 0, 100 - MinPulseVoltage, 0.1, 18, false, "%5.1f", &TD.TWCD[1].VoltageSetpoint, NULL, SetRestingVoltage},
  {" Guard 1", 0, 5, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[2].VoltageSetpoint, NULL, SetGuard1},
  {" Guard 2", 0, 6, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[3].VoltageSetpoint, NULL, SetGuard2},
  {" Save settings", 0, 8, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, SaveTwaveSettings, SaveTwaveSettings},
  {" Restore settings", 0, 9, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, RestoreTwaveSettings, RestoreTwaveSettings},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox TwaveDialog = {{"Twave parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, TwaveDialogEntries
};

// Rev 2.0 dialog options
DialogBoxEntry TwaveDialogEntries2[] = {
  {" Clock freq, Hz", 0, 1, D_INT, 32000, 400000, 1000, 16, false, "%7d", &TD.Velocity, NULL, SetVelocity},
  {" Sequence", 0, 2, D_BINARY8, 0, 255, 1, 15, false, NULL, &TD.Sequence, NULL, NULL},
  {" Pulse voltage", 0, 3, D_FLOAT, MinPulseVoltage, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[0].VoltageSetpoint, NULL, SetPulseVoltage},
  {" Wave dir", 0, 4, D_FWDREV, 0, 1, 1, 19, false, NULL, &TD.Direction, NULL, NULL},
  {" Guard 1", 0, 5, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[2].VoltageSetpoint, NULL, SetGuard1},
  {" Guard 2", 0, 6, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[3].VoltageSetpoint, NULL, SetGuard2},
  {" Calibration menu", 0, 8, D_DIALOG  , 0, 0, 0, 0, false, NULL, &TwaveCalMenu, NULL, NULL},
  {" Save settings", 0, 9, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, SaveTwaveSettings, SaveTwaveSettings},
  {" Restore settings", 0, 10, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, RestoreTwaveSettings, RestoreTwaveSettings},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox TwaveDialog2 = {{"Twave parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, TwaveDialogEntries2
};

DialogBoxEntry TwaveEntriesCalMenu[] = {
  {" Cal Pulse V"            , 0, 1, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibratePulse, NULL},
  {" Cal Guard 1"            , 0, 2, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard1, NULL},
  {" Cal Guard 2"            , 0, 3, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard2, NULL},
  {" Return to DC drive menu", 0, 10, D_DIALOG , 0, 0, 0, 0, false, NULL, &TwaveDialog2, NULL},
  {NULL},
};

DialogBox TwaveCalMenu = {
  {"Twave cal menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, TwaveEntriesCalMenu
};

void CalibratePulse(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 100;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[0].DCctrl;
  CC.ADCreadback = &TD.TWCD[0].DCmon;
  // Define this channels name
  sprintf(Name, "      Pulse V");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalibrateGuard1(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 100;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[2].DCctrl;
  CC.ADCreadback = &TD.TWCD[2].DCmon;
  // Define this channels name
  sprintf(Name, "     Guard 1");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalibrateGuard2(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 100;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[3].DCctrl;
  CC.ADCreadback = &TD.TWCD[3].DCmon;
  // Define this channels name
  sprintf(Name, "     Guard 2");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

// External trigger interrupt vectors here!
void TwaveSyncISR(void)
{
  // Re-load the sequence generator
  digitalWrite(TWload, LOW);
  digitalWrite(TWload, HIGH);
}

void Twave_init(int8_t Board)
{
  // Set active board to board being inited
  SelectedTwaveBoard = Board;
  SelectBoard(Board);
  LDAClow;
  // If normal startup load the EEPROM parameters from the RF driver card.
  if (NormalStartup)
  {
    RestoreTwaveSettings(true);
  }
  // Set reference on DAC and set all DAC channels to 0
  if (TD.Rev == 1) AD5625_EnableRef(TD.DACadr);
  // If rev 2 setup control bits
  if (TD.Rev == 2)
  { 
    pinMode(TWdir, OUTPUT);
    pinMode(TWclr, OUTPUT);
    pinMode(TWld, OUTPUT);
  }
  SetPulseVoltage();
  SetRestingVoltage();
  SetGuard1();
  SetGuard2();
  // Init the clock generator and set initial frequency
  SetVelocity();
  // Init the sequence generator pattern
  digitalWrite(TWreset, LOW);       // Clear the sequence generator
  digitalWrite(TWreset, HIGH);
  SetSequence();
  // Setup the menu
  if (TD.Rev == 1) AddMainMenuEntry(&METwaveMonitor);
  if (TD.Rev == 1) DialogBoxDisplay(&TwaveDialog);
  if (TD.Rev == 2) AddMainMenuEntry(&METwaveMonitor2);
  if (TD.Rev == 2) DialogBoxDisplay(&TwaveDialog2);
  // Configure Threads
  TwaveThread.onRun(Twave_loop);
  TwaveThread.setInterval(100);
  // Add threads to the controller
  controll.add(&TwaveThread);
  // Use trig input on pin 12 as a trigger to reset the sequence generator
  attachInterrupt(12, TwaveSyncISR, RISING);
}

// This is the main loop for the Twave function. This loop is called 10 times a sec.
void Twave_loop(void)
{
  static int CurrentVelocity = 0;
  static uint8_t CurrentSequence = 0;

  SelectBoard(SelectedTwaveBoard);
  // Update the output parameters
  SetPulseVoltage();
  SetRestingVoltage();
  SetGuard1();
  SetGuard2();
  // Only set the velocity if its changed
  if (CurrentVelocity != TD.Velocity)
  {
#ifdef UseTimer
    TwaveClk.setFrequency((double)(TD.Velocity * 2));
    TwaveClk.setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk.start(-1, 0, true);
#else
    CurrentVelocity = TD.Velocity;
    SetRef(8000000);
    SetPLL2freq(TD.CLOCKadr, TD.Velocity);
#endif
  }
  if (TD.Rev == 2) 
  {
     if(TD.Direction) digitalWrite(TWdir, HIGH);
     else digitalWrite(TWdir, LOW);
  }
  // Only update the output sequence if its changed
  if (CurrentSequence != TD.Sequence)
  {
    CurrentSequence = TD.Sequence;
    SetSequence();
  }
  // Determine the higest Twave output voltage
  MaxTwaveVoltage = 0;
  if (TD.TWCD[DAC_Guard1].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TD.TWCD[DAC_Guard1].VoltageSetpoint;
  if (TD.TWCD[DAC_Guard2].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TD.TWCD[DAC_Guard2].VoltageSetpoint;
  if (TD.TWCD[DAC_PulseVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TD.TWCD[DAC_PulseVoltage].VoltageSetpoint;
  if (TD.TWCD[DAC_RestingVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TD.TWCD[DAC_RestingVoltage].VoltageSetpoint;
}

//
// This section contains all of the serial command processing routines.
//
// Serial commands supported:
//  GTWF                        Get TWAVE frequency
//  STWF,<value>                Set TWAVE frequency  
//  GTWPV                       Get TWAVE pulse voltage
//  STWPV,<value>               Set TWAVE pulse voltage
//  GTWG1V                      Get TWAVE guard voltage, channel 1
//  STWG1V,<value>              Set TWAVE guard voltage, channel 1
//  GTWG2V                      Get TWAVE guard voltage, channel 2
//  STWG2V,<value>              Set TWAVE guard voltage, channel 2
//  GTWSEQ                      Get TWAVE sequence, 8 bit binary value returned
//  STWSEQ,<value>              Set TWAVE sequence, 8 bit binary value
// The following direction command is only valid on rev 2 or higher
//  GTWDIR                      Get TWAVE direction, FWD or REV
//  STWDIR,<value>              Set TWAVE direction, FWD or REV

// Returns the number of ESI output channels
void TWAVEnumberOfChannels(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfTWAVEchannels);
}

// Display the current sequence
void sendTWAVEsequence(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(TD.Sequence,BIN);
}

#define  TWAVEdisplayUpdate     {if(ActiveDialog == &TwaveDialog) {DisplayAllDialogEntries(&TwaveDialog); TwaveDialog.State = M_SCROLLING;}}

// Set the TWAVE frequency
void setTWAVEfrequency(int freq)
{
  DialogBoxEntry *DBE;
  
  // Test the requensed frequence to make sure it is with in range
  if(TD.Rev == 1) DBE = TwaveDialogEntries2;
  else DBE = TwaveDialogEntries2;
  if((freq >= DBE[0].Min) && (freq <= DBE[0].Max))
  {
    // Set the frequency
    TD.Velocity = freq;
    // Update the display
    TWAVEdisplayUpdate;
    // Exit
    SendACK;
    return;
  }
  // Here with invalid input
  SetErrorCode(ERR_VALUERANGE);
  SendNAK;
}

// Set the TWAVE bit sequence. The input string is a binary value with upto 8 bits max. 
// The only value values are 0 or 1.
void setTWAVEsequence(char *value)
{
  int i;
  int bval = 0;
  
  for(i=0;i<strlen(value);i++)
  {
    if((value[i] != '0') || (value[i] != '1') || (strlen(value) > 8)) 
    {
      // error exit for bad argument
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;
    }
    bval <<= 1;
    bval |= value[i] - '0';
  }
  TD.Sequence = bval;
  TWAVEdisplayUpdate;
  SendACK;
}

// Validate the voltage input command. Use the Pulse voltage to range test all
// voltage requests, they are all set to the same limits. 
// Returns a pointer to the float value or NULL on error.
float *TWAVEvoltageSetValidation(char *voltage)
{
  DialogBoxEntry *DBE;
  static float fval;
  
  sscanf(voltage,"%f",&fval);
  // Test the voltage range
  if(TD.Rev == 1) DBE = TwaveDialogEntries2;
  else DBE = TwaveDialogEntries2;
  if((fval < DBE[2].Min) || (fval > DBE[2].Max))
  {
    // here on range error so exit
    SetErrorCode(ERR_VALUERANGE);
    SendNAK;
    return NULL;
  }
  // Return pointer to the value
  return &fval;
}

// Set the TWAVE pulse voltage level
void setTWAVEpulseVoltage(char *voltage)
{
  float *fval;
  
  if((fval=TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  TD.TWCD[0].VoltageSetpoint = *fval;
  TWAVEdisplayUpdate;
  SendACK;  
}

// Set the TWAVE guard 1 voltage level
void setTWAVEguard1Voltage(char *voltage)
{
  float *fval;
  
  if((fval=TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  TD.TWCD[2].VoltageSetpoint = *fval;
  TWAVEdisplayUpdate;
  SendACK;  
}

// Set the TWAVE guard 2 voltage level
void setTWAVEguard2Voltage(char *voltage)
{
  float *fval;
  
  if((fval=TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  TD.TWCD[3].VoltageSetpoint = *fval;
  TWAVEdisplayUpdate;
  SendACK;  
}

// Get the TWAVE waveform direction, forward or reverse
void getTWAVEdir(void)
{
  if(TD.Rev <= 1)
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;
  }
  SendACKonly;
  if(TD.Direction) if(!SerialMute) serial->println("FWD");
  else if(!SerialMute) serial->println("REV");
}

// Set the TWAVE waveform direction, forward or reverse, FWD, REV
void setTWAVEdir(char *dirstr)
{
  if(TD.Rev <= 1)
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;
  }
  if((strcmp(dirstr,"FWD")==0) || (strcmp(dirstr,"REV")==0))
  {
    if(strcmp(dirstr,"FWD")==0) TD.Direction = true;
    else TD.Direction = false;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}


