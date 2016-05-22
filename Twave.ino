//
// Twave
//
// This file supports the Twave driver card, rev 1.0, 2.0, and 3.0.
// While support is still present for rev 1.0 this version has been retired.
//
// This version of the Twave driver does not use the readback ADC. 
//
// Needed to lower frequency below the 250KHz limit of the clock generator chip. Setup
// Timer channel 6 to output on pin 5, that is on connector EXT2 pin 9. Jumper EXT2 pin 9
// to pin 14 and then this signal can be found on pin 14 of IC10. This hardware mod is
// required for rev 2.0. 
//
// Support added for Twave driver module rev 3.0. December 19. 2015 This version
// uses a clock chip that will function over the required rance.
//
// Added features:
//    0.) Add support for 2 twave boards.
//    1.) Add menu option to allow sync trigger input.
//    2.) Add menu option to allow digitial input of direction control.
//    3.) Using different bits for direction control of module A and B.
//    4.) Add menu option to allow second module to use first modules clock
//        this also requires a board jumper. Add UseCommonClock flag and logic.
//    5.) Updated all the serial commands to accpect a Twave channel number.
//    6.) Add the compressor function. The is only valid on a dual Twave system.
//        - Use EXT2-14 (DUE 9, c.21) for the clock on board A, no changes needed.
//        - Use EXT2-12 (DUE 8, c.22) for the clock on board B, requires a cut and jump
//          need to route this signal through EXT2-14 pin on board B.
//        - Add compressor menu option when enabled
//            - Compression factor, 1 to 20
//            - Enable, ON/OFF
//            - External input for enable
//        - ISR to support the clock function
//        - Timer ISR used to generate the multi-pass timing.
// Add gate function and gate serial command. Then gated off all outputs will go low and no change
// when gated back on the bit pattern will reload and start. Maybe we can just set the pulse voltage 
// to zero, this would be the simple thing to do but still needs TWI interface.
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
// Applies to rev 1.0 and rev 2.0 only
#define UseTimer

// Use the timer channel 6 for a clock in rev 1.0 to get to lower frequencies.
// Also used on rev 2 due to issues with the clock chip.
// This timer is also used to generate the software clock in the compressor mode.
MIPStimer TwaveClk(6);
//MIPStimer TwaveClk(4);

// DAC channel assignment
#define  DAC_PulseVoltage    0
#define  DAC_RestingVoltage  1
#define  DAC_Guard1          2
#define  DAC_Guard2          3

#define   TWreset  ADDR0
#define   TWload   ADDR1      // Only used on rev 1.0, upgraded to TWld on rev 2.0

// Rev 2.0 bit definitions for PLD sequence generator
#define   TWdirA       49
#define   TWdirB       16
#define   TWclr        14
#define   TWld         48

#define   MinPulseVoltage    7

MenuEntry METwaveMonitor =  {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog , NULL, NULL};   // Rev 1 menu
MenuEntry METwaveMonitor2 = {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog2, NULL, NULL};   // Rev 2 and 3 menu

extern DialogBoxEntry TwaveDialogEntries2page2[];
extern DialogBox      CompressorDialog;

// Twave system variables
int       NumberOfTwaveModules  = 0;
bool      TwaveBoards[2]        = {false, false};
uint8_t   TwaveEEPROMaddr[2] = {0x50, 0x50}; // Addresses of the on module EEPROM
int       TwaveModuleNumber     = 1;         // User interface selected module number
TwaveData TDarray[2] = {Twave_Rev3, Twave_Rev3};
int       SelectedTwaveModule = 0;           // Selected Twave module index, 0 or 1 for module 1 or 2
int       TWboardAddress[2] = { -1, -1};     // Contains board A and B addresses, 0 or 1, if we have two boards its always 0,1. if its one board the could be 0,-1 or 1,-1
// To select the hardware board do the following: SelectBoard(TWboardAddress[SelectedTwaveBoard]);
TwaveData TD;

char TwaveCompressorTable[100] = "N10C3";

DIhandler *DIdirTW[2];
void (*TWdirISRs[2])(void) = {TW_1_DIR_ISR, TW_2_DIR_ISR};
DIhandler *DIsyncTW[2];
void (*TWsyncISRs[2])(void) = {TW_1_SYNC_ISR, TW_2_SYNC_ISR};
DIhandler *DIgateTW[2];
void (*TWgateISRs[2])(void) = {TW_1_GATE_ISR, TW_2_GATE_ISR};

// ISR functions

void TW_1_GateOn(void)
{
  int brd,mSdelay;
  
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = digitalRead(BRDSEL);
  ENA_BRD_A;
  MCP2300(TDarray[0].GPIOadr, ~TDarray[0].Sequence);
  // Restore the board
  digitalWrite(BRDSEL,brd);
  // Pulse the load line
  digitalWrite(TWld, LOW);
  // Hold this long enough for a clock to happen, if used on board clock this is 
  // 1/(clock freq *4). If software clock used in compressor then generate an edge
  if(TDarray[0].CompressorEnabled)
  {
    // Force a clock edge with software
    CompressorClockCycle();
    CompressorClockCycle();
  }
  else
  {
    mSdelay = 1000000 / (TDarray[0].Velocity * 4);
    if(mSdelay <= 10) mSdelay = 25;
    delayMicroseconds(mSdelay);
  }
  digitalWrite(TWld, HIGH);
}

void TW_1_GateOff(void)
{
  int brd,mSdelay;
  
//  digitalWrite(TWld, LOW);  // Hold load line actice, this will stop the clocking
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = digitalRead(BRDSEL);
  ENA_BRD_A;
  MCP2300(TDarray[0].GPIOadr, 0xFF);
  // Restore the board
  digitalWrite(BRDSEL,brd);
  digitalWrite(TWld, LOW);
  // Hold this long enough for a clock to happen, if used on board clock this is 
  // 1/(clock freq *4). If software clock used in compressor then generate an edge
  if(TDarray[0].CompressorEnabled)
  {
    // Force a clock edge with software
    CompressorClockCycle();
    CompressorClockCycle();
  }
  else
  {
    mSdelay = 1000000 / (TDarray[0].Velocity * 4);
    if(mSdelay <= 10) mSdelay = 25;
    delayMicroseconds(mSdelay);
  }
  digitalWrite(TWld, HIGH);
}

// This function will gate on or gate off the Twave signal. Its called when every a
// level change is detected on the gate input. 
void TW_1_GATE_ISR(void)
{
  bool level;

  level = DIgateTW[0]->activeLevel();  // Read input
  if(((level) && (TDarray[0].TWgateLevel == HIGH)) || ((!level) && (TDarray[0].TWgateLevel == LOW)))
  {
    // Gate Twave off, Hold load line active and set sequence to 0xFF
    if(AcquireTWI()) TW_1_GateOff();
    else TWIqueue(TW_1_GateOff);
  }
  else
  {
    // Gate Twave on, Set the sequence and pulse the load line
    if(AcquireTWI()) TW_1_GateOn();
    else TWIqueue(TW_1_GateOn);
  }
}

void TW_2_GateOn(void)
{
  int brd,mSdelay;
  
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = digitalRead(BRDSEL);
  ENA_BRD_B;
  MCP2300(TDarray[1].GPIOadr, ~TDarray[1].Sequence);
  // Restore the board
  digitalWrite(BRDSEL,brd);
  // Pulse the load line
  digitalWrite(TWld, LOW);
  // Hold this long enough for a clock to happen, if used on board clock this is 
  // 1/(clock freq *4). If software clock used in compressor then generate an edge
  if(TDarray[0].CompressorEnabled)
  {
    // Force a clock edge with software
    CompressorClockCycle();
    CompressorClockCycle();
  }
  else
  {
    mSdelay = 1000000 / (TDarray[1].Velocity * 4);
    if(mSdelay <= 10) mSdelay = 25;
    delayMicroseconds(mSdelay);
  }
  digitalWrite(TWld, HIGH);
}

void TW_2_GateOff(void)
{
  int brd,mSdelay;
  
//  digitalWrite(TWld, LOW);  // Hold load line actice, this will stop the clocking
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = digitalRead(BRDSEL);
  ENA_BRD_B;
  MCP2300(TDarray[1].GPIOadr, 0xFF);
  // Restore the board
  digitalWrite(BRDSEL,brd);
  digitalWrite(TWld, LOW);
  // Hold this long enough for a clock to happen, if used on board clock this is 
  // 1/(clock freq *4). If software clock used in compressor then generate an edge
  if(TDarray[0].CompressorEnabled)
  {
    // Force a clock edge with software
    CompressorClockCycle();
    CompressorClockCycle();
  }
  else
  {
    mSdelay = 1000000 / (TDarray[1].Velocity * 4);
    if(mSdelay <= 10) mSdelay = 25;
    delayMicroseconds(mSdelay);
  }
  digitalWrite(TWld, HIGH);
}

void TW_2_GATE_ISR(void)
{
  bool level;

  level = DIgateTW[1]->activeLevel();  // Read input
  if(((level) && (TDarray[1].TWgateLevel == HIGH)) || ((!level) && (TDarray[1].TWgateLevel == LOW)))
  {
    // Gate Twave off, Hold load line active and set sequence to 0xFF
    if(AcquireTWI()) TW_2_GateOff();
    else TWIqueue(TW_2_GateOff);
  }
  else
  {
    // Gate Twave on, Set the sequence and pulse the load line
    if(AcquireTWI()) TW_2_GateOn();
    else TWIqueue(TW_2_GateOn);
  }
}

void TW_1_DIR_ISR(void)
{
  if (DIdirTW[0]->activeLevel()) TDarray[0].Direction = true;
  else TDarray[0].Direction = false;
  if (0 == SelectedTwaveModule)
  {
    if (DIdirTW[0]->activeLevel()) TD.Direction = true;
    else TD.Direction = false;
  }
  if (TDarray[0].Direction) digitalWrite(TWdirA, HIGH);
  else digitalWrite(TWdirA, LOW);
  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}
void TW_2_DIR_ISR(void)
{
  if (DIdirTW[1]->activeLevel()) TDarray[1].Direction = true;
  else TDarray[1].Direction = false;
  if (1 == SelectedTwaveModule)
  {
    if (DIdirTW[1]->activeLevel()) TD.Direction = true;
    else TD.Direction = false;
  }
  if (TDarray[1].Direction) digitalWrite(TWdirB, HIGH);
  else digitalWrite(TWdirB, LOW);
  AtomicBlock< Atomic_RestoreState > a_Block;
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}
void TW_1_SYNC_ISR(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Re-load the sequence generator
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}
void TW_2_SYNC_ISR(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Re-load the sequence generator
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}

extern Menu MainMenu;

//MIPS Threads
Thread TwaveThread  = Thread();

float MaxTwaveVoltage = 0;  // This value is set to the highest Twave output voltage

void SetVelocity(int ModuleIndex)
{
  //  SelectBoard(TWboardAddress[SelectedTwaveBoard]);
  if (TDarray[ModuleIndex].Rev == 3)
  {
    // Rev 3 only supports the on module FS7140 clock chip.
    if(TDarray[0].CompressorEnabled)
    {
      TwaveClk.attachInterrupt(CompressorClockISR);
      TwaveClk.setFrequency((double)TDarray[0].Velocity);
      TwaveClk.start(-1, 0, false);
    }
    else FS7140setup(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity * 4);
  }
  else if (TDarray[ModuleIndex].Rev == 2)
  {
#ifdef UseTimer
    // The 8 times multiplier is because the output toggles on each clock thus devides
    // by two and the rev 2 hardware has a 4x division.
    TwaveClk.setFrequency((double)(TDarray[ModuleIndex].Velocity * 8));
    TwaveClk.setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk.start(-1, 0, true);
#else
    SetRef(8000000);
    CY_Init(TDarray[ModuleIndex].CLOCKadr);
    SetPLL2freq(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity * 8);
#endif
  }
  else // Rev 1
  {
#ifdef UseTimer
    // The 2 times multiplier is because the output toggles on each clock thus devides
    // by two.
    TwaveClk.setFrequency((double)(TDarray[ModuleIndex].Velocity * 2));
    TwaveClk.setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk.start(-1, 0, true);
#else
    SetRef(8000000);
    CY_Init(TDarray[ModuleIndex].CLOCKadr);
    SetPLL2freq(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity);
#endif
  }
}

void SetSequence(int ModuleIndex)
{
  int   i;
  int   numTry = 0;

  if ((TDarray[ModuleIndex].Rev == 2) || (TDarray[ModuleIndex].Rev == 3))
  {
    // Retry if the GPIO fails, this is needed!
    for (i = 0; i < 100; i++)
    {
      numTry++;
      if (MCP2300(TDarray[ModuleIndex].GPIOadr, ~TDarray[ModuleIndex].Sequence) == 0) break;
      delay(2);
    }
    AtomicBlock< Atomic_RestoreState > a_Block;
    digitalWrite(TWclr, LOW);
    delayMicroseconds(1000);
    digitalWrite(TWclr, HIGH);
    delayMicroseconds(1000);
    digitalWrite(TWld, LOW);
    delayMicroseconds(1000);
    CompressorClockCycle();
    CompressorClockReset();
    digitalWrite(TWld, HIGH);
    delayMicroseconds(1000);
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
    if (MCP2300(TDarray[ModuleIndex].GPIOadr, ~TDarray[ModuleIndex].Sequence) == 0) break;
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
void SetPulseVoltage(int ModuleIndex)
{
  int DACcounts;

  //  SelectBoard(TWboardAddress[SelectedTwaveBoard]);
  if (TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint < (TDarray[ModuleIndex].TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage)) TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint = TDarray[ModuleIndex].TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPulseVoltage;
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint, &TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].DCctrl);
  AD5625(TDarray[ModuleIndex].DACadr, TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].DCctrl.Chan, DACcounts);
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
void SetGuard1(int ModuleIndex)
{
  int DACcounts;

  //  SelectBoard(TWboardAddress[SelectedTwaveBoard]);
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TDarray[ModuleIndex].TWCD[DAC_Guard1].VoltageSetpoint,  &TDarray[ModuleIndex].TWCD[DAC_Guard1].DCctrl);
  AD5625(TDarray[ModuleIndex].DACadr, TDarray[ModuleIndex].TWCD[DAC_Guard1].DCctrl.Chan, DACcounts);
}

// This function will set the Guard 2 DAC output.
void SetGuard2(int ModuleIndex)
{
  int DACcounts;

  //  SelectBoard(TWboardAddress[SelectedTwaveBoard]);
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TDarray[ModuleIndex].TWCD[DAC_Guard2].VoltageSetpoint,  &TDarray[ModuleIndex].TWCD[DAC_Guard2].DCctrl);
  AD5625(TDarray[ModuleIndex].DACadr, TDarray[ModuleIndex].TWCD[DAC_Guard2].DCctrl.Chan, DACcounts);
}

// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveTwaveSettings(void)
{
  if (WriteEEPROM(&TDarray[SelectedTwaveModule], TwaveEEPROMaddr[SelectedTwaveModule], 0, sizeof(TwaveData)) == 0)
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

  if (ReadEEPROM(&td, TwaveEEPROMaddr[SelectedTwaveModule], 0, sizeof(TwaveData)) == 0)
  {
    if (strcmp(td.Name, TDarray[SelectedTwaveModule].Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      memcpy(&TDarray[SelectedTwaveModule], &td, sizeof(TwaveData));
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      SelectTwaveModule();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}


DialogBoxEntry TwaveDialogEntries[] = {
#ifdef UseTimer
  {" Clock freq, Hz", 0, 1, D_INT, 1000, 2000000, 1000, 16, false, "%7d", &TD.Velocity, NULL, NULL},
#else
  {" Clock freq, Hz", 0, 1, D_INT, 250000, 2000000, 1000, 16, false, "%7d", &TD.Velocity, NULL, NULL},
#endif
  {" Sequence", 0, 2, D_BINARY8, 0, 255, 1, 15, false, NULL, &TD.Sequence, NULL, NULL},
  {" Pulse voltage", 0, 3, D_FLOAT, MinPulseVoltage, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[0].VoltageSetpoint, NULL, NULL},
  {" Resting voltage", 0, 4, D_FLOAT, 0, 100 - MinPulseVoltage, 0.1, 18, false, "%5.1f", &TD.TWCD[1].VoltageSetpoint, NULL, NULL},
  {" Guard 1", 0, 5, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[2].VoltageSetpoint, NULL, NULL},
  {" Guard 2", 0, 6, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[3].VoltageSetpoint, NULL, NULL},
  {" Save settings", 0, 8, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, SaveTwaveSettings, SaveTwaveSettings},
  {" Restore settings", 0, 9, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, RestoreTwaveSettings, RestoreTwaveSettings},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox TwaveDialog = {{"Twave parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, TwaveDialogEntries
};

// Rev 2.0/3.0 dialog options
DialogBoxEntry TwaveDialogEntries2[] = {
  {" Twave module"       , 0, 1, D_INT, 1, 1, 1, 22, false, "%1d", &TwaveModuleNumber, NULL, SelectTwaveModule},
  {" Clock freq, Hz"     , 0, 2, D_INT, 1000, 300000, 1000, 16, false, "%7d", &TD.Velocity, NULL, NULL},
  {" Sequence"           , 0, 3, D_BINARY8, 0, 255, 1, 15, false, NULL, &TD.Sequence, NULL, NULL},
  {" Pulse voltage"      , 0, 4, D_FLOAT, MinPulseVoltage, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[0].VoltageSetpoint, NULL, NULL},
  {" Wave dir"           , 0, 5, D_FWDREV, 0, 1, 1, 19, false, NULL, &TD.Direction, NULL, NULL},
  {" Guard 1"            , 0, 6, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[2].VoltageSetpoint, NULL, NULL},
  {" Guard 2"            , 0, 7, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &TD.TWCD[3].VoltageSetpoint, NULL, NULL},
  {" Next page"          , 0, 8, D_PAGE  , 0, 0, 0, 0, false, NULL, &TwaveDialogEntries2page2, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, SaveTwaveSettings, SaveTwaveSettings},
  {" Restore settings"   , 0, 10, D_FUNCTION, 0, 0, 0.0, 0, false, NULL, NULL, RestoreTwaveSettings, RestoreTwaveSettings},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox TwaveDialog2 = {{"Twave parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, TwaveDialogEntries2
};

DialogBoxEntry TwaveDialogEntries2page2[] = {
  {" Cal Pulse V"   , 0, 1, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibratePulse, NULL},
  {" Cal Guard 1"   , 0, 2, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard1, NULL},
  {" Cal Guard 2"   , 0, 3, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard2, NULL},
  {" Sync input"    , 0, 4, D_DI     , 0, 0, 2, 21, false, DIlist, &TD.TWsyncDI, NULL, NULL},
  {" Sync level"    , 0, 5, D_DILEVEL, 0, 0, 4, 19, false, DILlist, &TD.TWsyncLevel, NULL, NULL},
  {" Dir input"     , 0, 6, D_DI     , 0, 0, 2, 21, false, NULL, &TD.TWdirDI, NULL, NULL},
  {" Dir level"     , 0, 7, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.TWdirLevel, NULL, NULL},
  {" Gate input"    , 0, 8, D_DI     , 0, 0, 2, 21, false, NULL, &TD.TWgateDI, NULL, NULL},
  {" Gate level"    , 0, 9, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.TWgateLevel, NULL, NULL},
  {" Compressor"    , 0, 10, D_OFF , 0, 0, 0, 0, false, NULL, &CompressorDialog, NULL,NULL},
  {" Return to first page", 0, 11, D_PAGE , 0, 0, 0, 0, false, NULL, &TwaveDialogEntries2, NULL, NULL},
  {NULL},
};

void CalibratePulse(void)
{
  ChannelCal CC;
  char       Name[20];

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
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
  ChannelCalibrate(&CC, Name, 20.0, 75.0);
}

void CalibrateGuard1(void)
{
  ChannelCal CC;
  char       Name[20];

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
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
  ChannelCalibrate(&CC, Name, 10.0, 75.0);
}

void CalibrateGuard2(void)
{
  ChannelCal CC;
  char       Name[20];

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
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
  ChannelCalibrate(&CC, Name, 10.0, 75.0);
}

// External trigger interrupt vectors here!
// Only used on rev 1.0
void TwaveSyncISR(void)
{
  // Re-load the sequence generator
  digitalWrite(TWload, LOW);
  digitalWrite(TWload, HIGH);
}

// Selects the active Twave module, 1 or 2
void SelectTwaveModule(void)
{
  DialogBoxEntry *de;
  
  // Select the board
  SelectedTwaveModule = TwaveModuleNumber - 1;
  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  TD = TDarray[SelectedTwaveModule];
  // Update the compressor display option
  de = GetDialogEntries(TwaveDialogEntries2page2, "Compressor");
  if(TwaveModuleNumber > 1) de->Type = D_OFF;
  else if(TDarray[0].CompressorEnabled) de->Type = D_DIALOG;
  // Redraw the dialog box
  if (ActiveDialog == &TwaveDialog2) DialogBoxDisplay(&TwaveDialog2);
}

void Twave_init(int8_t Board, uint8_t addr)
{
  DialogBox *SavedDialog;

  TwaveEEPROMaddr[Board] = addr;
  // Flag the board as present
  TwaveBoards[Board] = true;
  TWboardAddress[NumberOfTwaveModules] = Board;
  // Create digital input dir and sync objects
  DIdirTW[Board] = new DIhandler;
  DIsyncTW[Board] = new DIhandler;
  DIgateTW[Board] = new DIhandler;
  // Set active board board being inited
  SelectedTwaveModule = Board;
  SelectBoard(Board);
  LDAClow;
  // If normal startup load the EEPROM parameters from the RF driver card.
  SavedDialog = ActiveDialog;
  ActiveDialog = NULL;          // Stop any display updating
  if (NormalStartup)
  {
    RestoreTwaveSettings(true);
  }
  ActiveDialog = SavedDialog;
  // Set active board board being inited
  SelectedTwaveModule = Board;
  SelectBoard(Board);
  TD = TDarray[SelectedTwaveModule];
  // Set reference on DAC and set all DAC channels to 0
  if (TD.Rev == 1) AD5625_EnableRef(TD.DACadr);
  // If rev 2 or 3 setup control bits
  if ((TD.Rev == 2) || (TD.Rev == 3))
  {
    if (Board == 0) pinMode(TWdirA, OUTPUT);
    if (Board == 1) pinMode(TWdirB, OUTPUT);
    pinMode(TWclr, OUTPUT);
    pinMode(TWld, OUTPUT);
  }
  SetPulseVoltage(Board);
  SetRestingVoltage();
  SetGuard1(Board);
  SetGuard2(Board);
  // Init the clock generator and set initial frequency
  SetVelocity(Board);
  // Init the sequence generator pattern
  digitalWrite(TWreset, LOW);       // Clear the sequence generator
  digitalWrite(TWreset, HIGH);
  SetSequence(Board);
  TWboardAddress[NumberOfTwaveModules] = Board;
  // Init the gate state
  DIgateTW[Board]->detach();
  DIgateTW[Board]->attached(TDarray[Board].TWgateDI, CHANGE, TWgateISRs[Board]);
  if(TDarray[Board].TWgateDI != -1)
  {
    if(Board == 0) TW_1_GATE_ISR();
    else TW_2_GATE_ISR();
  }
  //
  if (NumberOfTwaveModules == 0)
  {
    // Setup the menu
    if (TD.Rev == 1) AddMainMenuEntry(&METwaveMonitor);
    if (TD.Rev == 1) DialogBoxDisplay(&TwaveDialog);
    if ((TD.Rev == 2) || (TD.Rev == 3)) AddMainMenuEntry(&METwaveMonitor2);
    if ((TD.Rev == 2) || (TD.Rev == 3))
    {
      if (ActiveDialog == NULL)
      {
        ActiveDialog = &TwaveDialog2;
        SelectTwaveModule();
      }
    }
    // Configure Threads
    TwaveThread.setName("Twave");
    TwaveThread.onRun(Twave_loop);
    TwaveThread.setInterval(100);
    // Add threads to the controller
    control.add(&TwaveThread);
    // Use trig input on pin 12 as a trigger to reset the sequence generator
    if (TD.Rev == 1) attachInterrupt(12, TwaveSyncISR, RISING);
    CompressorInit();
  }
  else
  {
    SelectedTwaveModule = 0;
    SelectBoard(0);
    TD = TDarray[SelectedTwaveModule];
  }
  NumberOfTwaveModules++;
  if (TD.Rev > 1) TwaveDialogEntries2[0].Max = NumberOfTwaveModules;
}

// This is the main loop for the Twave function. This loop is called 10 times a sec.
void Twave_loop(void)
{
  static uint8_t CurrentSequence[2] = {0, 0};
  static int     CurrentVelocity[2] = {0, 0};
  static int     CurrentDirection[2] = { -2, -2};
  static int     LastGateLevel[2] = {-1,-1};
  static float   CurrentPulseVoltage[2] = {-1,-1};
  static float   CurrentGuard1Voltage[2] = {-1,-1};
  static float   CurrentGuard2Voltage[2] = {-1,-1};
  int i;

  if ((ActiveDialog == &TwaveDialog2)||(ActiveDialog == &TwaveDialog)||(ActiveDialog == &CompressorDialog)) TDarray[SelectedTwaveModule] = TD;   // Store any changes, if dialog is selected
  CompressorLoop();
  for (i = 0; i < 2; i++)
  {
    if (TwaveBoards[i])
    {
      SelectBoard(TWboardAddress[i]);
      // Update the output parameters only if they have changed
      if(TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint != CurrentPulseVoltage[i])
      {
         CurrentPulseVoltage[i] = TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint;
         SetPulseVoltage(i);
      }
      if (TDarray[i].Rev == 1) SetRestingVoltage();
      if(TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint != CurrentGuard1Voltage[i])
      {
         CurrentGuard1Voltage[i] = TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint;
         SetGuard1(i);
      }
      if(TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint != CurrentGuard2Voltage[i])
      {
         CurrentGuard2Voltage[i] = TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint;
         SetGuard2(i);
      }
      if (TDarray[i].Rev == 2)
      {
        if (TDarray[i].Direction) digitalWrite(TWdirA, HIGH);
        else digitalWrite(TWdirA, LOW);
      }
      if (TDarray[i].Rev == 3)
      {
        if (CurrentDirection[i] != TDarray[i].Direction)
        {
          CurrentDirection[i] = TDarray[i].Direction;
          if (i == 0)
          {
            if (TDarray[i].Direction) digitalWrite(TWdirA, HIGH);
            else digitalWrite(TWdirA, LOW);
          }
          if (i == 1)
          {
            if (TDarray[i].Direction) digitalWrite(TWdirB, HIGH);
            else digitalWrite(TWdirB, LOW);
          }
            digitalWrite(TWld, LOW);
            delay(1);
            digitalWrite(TWld, HIGH);
        }
      }
      if (TDarray[i].Rev >= 2)
      {
        if ((TDarray[i].TWdirDI != DIdirTW[i]->di) || (TDarray[i].TWdirLevel != DIdirTW[i]->mode))
        {
          DIdirTW[i]->detach();
          DIdirTW[i]->attached(TDarray[i].TWdirDI, TDarray[i].TWdirLevel, TWdirISRs[i]);
        }
        if ((TDarray[i].TWsyncDI != DIsyncTW[i]->di) || (TDarray[i].TWsyncLevel != DIsyncTW[i]->mode))
        {
          DIsyncTW[i]->detach();
          DIsyncTW[i]->attached(TDarray[i].TWsyncDI, TDarray[i].TWsyncLevel, TWsyncISRs[i]);
        }
        if ((TDarray[i].TWgateDI != DIgateTW[i]->di) || (CHANGE != DIgateTW[i]->mode))
        {
          DIgateTW[i]->detach();
          DIgateTW[i]->attached(TDarray[i].TWgateDI, CHANGE, TWgateISRs[i]);
        }
        // If the level changes for the gate then update the gate state by calling the proper ISR function
          if((TDarray[i].TWgateDI != -1) && (TDarray[i].TWgateLevel != LastGateLevel[i]))
          {
            LastGateLevel[i] = TDarray[i].TWgateLevel;
            if(i == 0) TW_1_GATE_ISR();
            else TW_2_GATE_ISR();
          }
      }
      // Only update the output sequence if its changed
      if (CurrentSequence[i] != TDarray[i].Sequence)
      {
        CurrentSequence[i] = TDarray[i].Sequence;
        SetSequence(i);
      }
      // Only update the velocity if its changed
      if (CurrentVelocity[i] != TDarray[i].Velocity)
      {
        CurrentVelocity[i] = TDarray[i].Velocity;
        SetVelocity(i);
        if (TDarray[0].UseCommonClock) // Make both clocks match if use common clock flag is set
        {
          TDarray[(i + 1) & 1].Velocity = TDarray[i].Velocity;
          TD.Velocity = TDarray[i].Velocity;
        }
      }
      // Determine the higest Twave output voltage
      MaxTwaveVoltage = 0;
      if (TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_RestingVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_RestingVoltage].VoltageSetpoint;
    }
  }
  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  // Display the monitored values based on the dialog box curently being displayed
  if (ActiveDialog == &TwaveDialog2) RefreshAllDialogEntries(&TwaveDialog2);
  if (ActiveDialog == &TwaveDialog) RefreshAllDialogEntries(&TwaveDialog);
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
//
// All commands updated to accept channel number 1 or 2, dec 2015

// Returns the number of Twave modules in the MIPS system
void TWAVEnumberOfChannels(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfTwaveModules);
}

// The function returns to the index for the requested Twave module, 0 or 1.
// This function also makes sure the index is valid
int GetTwaveIndex(int channel)
{
  if ((channel < 1) || (channel > 2))
  {
    SetErrorCode(ERR_VALUERANGE);
    if (!SerialMute)
    {
      SendNAK
    }
    return -1;
  }
  return channel - 1;
}

// Display the current sequence
void sendTWAVEsequence(int channel)
{
  int i = GetTwaveIndex(channel);
  if (i == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[i].Sequence, BIN);
}

// Report the twave frequency for the selected channel
void sendTWAVEfrequency(int channel)
{
  int i;

  i = GetTwaveIndex(channel);
  if (i == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[i].Velocity);
}

// Set the TWAVE frequency
void setTWAVEfrequency(int channel, int freq)
{
  DialogBoxEntry *DBE;
  int i, saveselected;

  i = GetTwaveIndex(channel);
  if (i == -1) return;
  // Test the requensed frequence to make sure it is with in range
  if (TDarray[i].Rev == 1) DBE = &TwaveDialogEntries[0];
  else DBE = &TwaveDialogEntries2[1];
  if ((freq >= DBE->Min) && (freq <= DBE->Max))
  {
    // Set the frequency
    if (i == SelectedTwaveModule) TD.Velocity = freq;
    TDarray[i].Velocity = freq;
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
void setTWAVEsequence(char * chan, char *value)
{
  int i, channel, index;
  int bval = 0;

  sscanf(chan, " %d", &channel);
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  for (i = 0; i < strlen(value); i++)
  {
    if ((value[i] != '0') && (value[i] != '1') || (strlen(value) > 8))
    {
      // error exit for bad argument
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;
    }
    bval <<= 1;
    bval |= value[i] - '0';
  }
  if (index == SelectedTwaveModule) TD.Sequence = bval;
  TDarray[index].Sequence = bval;
  SendACK;
}

// Validate the voltage input command. Use the Pulse voltage to range test all
// voltage requests, they are all set to the same limits.
// Returns a pointer to the float value or NULL on error.
float *TWAVEvoltageSetValidation(char *voltage)
{
  DialogBoxEntry *DBE;
  static float fval;

  sscanf(voltage, "%f", &fval);
  // Test the voltage range
  if (TD.Rev == 1) DBE = &TwaveDialogEntries[2];
  else DBE = &TwaveDialogEntries2[3];
  if ((fval < DBE->Min) || (fval > DBE->Max))
  {
    // here on range error so exit
    SetErrorCode(ERR_VALUERANGE);
    SendNAK;
    return NULL;
  }
  // Return pointer to the value
  return &fval;
}

// Send the TWAVE pulse voltage level
void sendTWAVEpulseVoltage(int channel)
{
  int index;
  float *fval;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[index].TWCD[0].VoltageSetpoint);
}

// Set the TWAVE pulse voltage level
void setTWAVEpulseVoltage(char *chan, char *voltage)
{
  int channel, index;
  float *fval;

  sscanf(chan, " %d", &channel);
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  if ((fval = TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[0].VoltageSetpoint = *fval;
  TDarray[index].TWCD[0].VoltageSetpoint = *fval;
  SendACK;
}

// Send the TWAVE guard 1 voltage level
void sendTWAVEguard1Voltage(int channel)
{
  int index;
  float *fval;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[index].TWCD[2].VoltageSetpoint);
}

// Set the TWAVE guard 1 voltage level
void setTWAVEguard1Voltage(char *chan, char *voltage)
{
  int channel, index;
  float *fval;

  sscanf(chan, " %d", &channel);
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  if ((fval = TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[2].VoltageSetpoint = *fval;
  TDarray[index].TWCD[2].VoltageSetpoint = *fval;
  SendACK;
}

// Send the TWAVE guard 2 voltage level
void sendTWAVEguard2Voltage(int channel)
{
  int index;
  float *fval;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[index].TWCD[3].VoltageSetpoint);
}

// Set the TWAVE guard 2 voltage level
void setTWAVEguard2Voltage(char *chan, char *voltage)
{
  int channel, index;
  float *fval;

  sscanf(chan, " %d", &channel);
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  if ((fval = TWAVEvoltageSetValidation(voltage)) == NULL) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[3].VoltageSetpoint = *fval;
  TDarray[index].TWCD[3].VoltageSetpoint = *fval;
  SendACK;
}

// Get the TWAVE waveform direction, forward or reverse
void getTWAVEdir(int channel)
{
  int index;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  if (TDarray[index].Rev <= 1)
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;
  }
  SendACKonly;
  if (TDarray[index].Direction)
  {
    if (!SerialMute) serial->println("FWD");
  }
  else if (!SerialMute) serial->println("REV");
}

// Set the TWAVE waveform direction, forward or reverse, FWD, REV
void setTWAVEdir(char *chan, char *dirstr)
{
  int channel, index;

  sscanf(chan, " %d", &channel);
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  if (TDarray[index].Rev <= 1)
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;
  }
  if ((strcmp(dirstr, "FWD") == 0) || (strcmp(dirstr, "REV") == 0))
  {
    if (strcmp(dirstr, "FWD") == 0)
    {
      if (index == SelectedTwaveModule) TD.Direction = true;
      TDarray[index].Direction = true;
    }
    else
    {
      if (index == SelectedTwaveModule) TD.Direction = false;
      TDarray[index].Direction = false;
    }
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

//
// The following code supports the compressor mode of operation. This mode requires
// two Twave boards in one MIPS system.
// The clock is generated in software for the compressor mode
//  Primary Twave module clock is DUE pin 9, c.21
//  Secondary Twave module clock is DUE pin 8, c.22
//
// Variables in the Twave structure
//    int    Corder;            // Compressor order
//    int    NumPasses;
//    int    CNth;
//    float  Tdelay;
//    float  Tcompress;
//    float  Tnormal;
//    float  TnoC;
//    char   Ctrig;              // External input for compressor start trigger
//    int8_t CtrigLevel;         // External input triggerl level
//    char   Ctrig;              // Digitial output to control output switch to relead ions to mass spec
//    int8_t CtrigLevel;         // Digital output level control
//
// Calculate the total time for number of passes.
//  Number of compressed passes = NumPasses / CNth
//  Number of non compressed passes =  NumPasses - NumPasses / CNth
//  Total time = (NumPasses - NumPasses / CNth) * TnoC + (NumPasses / CNth) * (Tcompress + Tnormal)
//
//  CurrentPass, defines the current pass and starts at 1
//  State = NonCompress,Compress,Normal
//
//  StateChangeISR, fires every time the compressor state changes
//    if State = Compress then update state to Normal and set time for next interrupt to Tnornal
//    if State = Normal or NonCompress then advance CurrentPass
//    if CurrentPass % CNth == 0 then State = Compress
//    else State = NonCompress
//    Set next interrupt time
//
// May 10, 2016. The compressor control was changed from the compress every Nth orginal design to
// use a compression table to control the order of non compressed and compressed cycles.
// Compressor table structure
//  Xn...
// Where
//  X
//    = N for non compressed
//    = C for compressed
//    = O for order
//    = S for switch control
//  n
//    = number of cycles or options parameter
//

extern DialogBoxEntry CompressorEntries2[];

char *CmodeList = "Normal,Compress";
char Cmode[12]  = "Normal";

DialogBoxEntry CompressorEntries[] = {
  {" Mode"                , 0, 1, D_LIST   , 0,  0, 8, 15, false, CmodeList, Cmode, NULL, UpdateMode},
  {" Order"               , 0, 2, D_INT8   , 0, 127, 1, 20, false, "%3d", &TD.Corder, NULL, UpdateMode},
  {" Compression table"   , 0, 3, D_TITLE  , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" "                    , 0, 4, D_STRING , 0, 2, 0, 2, false, "%.20s", TwaveCompressorTable, NULL, NULL},
//{" Num passes"          , 0, 3, D_INT8   , 1, 50, 1, 21, false, "%2d", &TD.NumPasses, NULL, NULL},
//{" Compress Nth"        , 0, 4, D_INT8   , 1, 20, 1, 21, false, "%2d", &TD.CNth, NULL, NULL},
  {" Trig delay, mS"      , 0, 5, D_FLOAT  , 0.1, 999, 0.1, 18, false, "%5.1f", &TD.Tdelay, NULL, NULL},
  {" Compress t, mS"      , 0, 6, D_FLOAT  , 0.1, 999, 0.1, 18, false, "%5.1f", &TD.Tcompress, NULL, NULL},
  {" Normal t, mS"        , 0, 7, D_FLOAT  , 0.1, 999, 0.1, 18, false, "%5.1f", &TD.Tnormal, NULL, NULL},
  {" Non Comp t, mS"      , 0, 8, D_FLOAT  , 0.1, 999, 0.1, 18, false, "%5.1f", &TD.TnoC, NULL, NULL},
  {" Next page"           , 0, 10, D_PAGE  , 0, 0, 0, 0, false, NULL, &CompressorEntries2, NULL, NULL},
  {" Return to Twave menu", 0, 11, D_DIALOG, 0, 0, 0, 0, false, NULL, &TwaveDialog2, NULL, NULL},
  {NULL},
};

char *CswitchList    = "Open,Close";
char CswitchState[6] = "Open";

DialogBoxEntry CompressorEntries2[] = {
  {" Trig input"          , 0, 1, D_DI     , 0, 0, 2, 21, false, NULL, &TD.Ctrig, NULL, ConfigureTrig},
  {" Trig level"          , 0, 2, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.CtrigLevel, NULL, ConfigureTrig},
  {" Force trigger"       , 0, 4, D_FUNCTION,0, 0, 0, 0,  false, NULL, NULL, CompressorTriggerISR, CompressorTriggerISR},
  {" Switch output"       , 0, 6, D_DO     , 0, 0, 2, 21, false, NULL, &TD.Cswitch, NULL, SetSwitch},
  {" Switch level"        , 0, 7, D_DOLEVEL, 0, 0, 4, 19, false, NULL, &TD.CswitchLevel, NULL, SetSwitch},
  {" Switch state"        , 0, 8, D_LIST   , 0, 0, 5, 18, false, CswitchList, CswitchState, NULL, SetSwitch},
  {" First page"          , 0, 10,D_PAGE   , 0, 0, 0, 0,  false, NULL, &CompressorEntries, NULL, NULL},
  {" Return to Twave menu", 0, 11,D_DIALOG , 0, 0, 0, 0,  false, NULL, &TwaveDialog2, NULL, NULL},
  {NULL},
};

DialogBox CompressorDialog = {{"Compressor params", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, CompressorEntries
};

#define MaxOrder  127

#define CLOCK_A   9
#define CLOCK_B   8

MIPStimer CompressorTimer(4);

volatile uint32_t  ClockArray[MaxOrder * 8];
volatile int       ClockReset = 16;
volatile int       ClockIndex = 0;
volatile int       CR = 16;

volatile Pio *pio;

// Time delay parameters. All the time values in milli secs are converted into timer counts and saved in these 
// variables.
#define   C_clock       656250      // This is the clock frequenncy used for the timing control
                                    // for the compressor state machine. This is the clock frequency
                                    // for the timer. 
uint32_t  C_Td;         // Trigger delay
uint32_t  C_Tc;         // Compressed time
uint32_t  C_Tn;         // Normal time
uint32_t  C_Tnc;        // Non compressed cycle time
uint32_t  C_NextEvent;  // Next event counter value

DIhandler *CtrigInput;

CompressorState CState;
int CurrentPass;

void UpdateMode(void)
{
  if(strcmp(Cmode,"Normal")==0) ClockReset = 8;
  else ClockReset = TD.Corder * 8;
}

void ConfigureTrig(void)
{
  CtrigInput->detach();
  CtrigInput->attached(TD.Ctrig, TD.CtrigLevel, CompressorTriggerISR);  
}

// Set the switch state based on user dialog box selection
void SetSwitch(void)
{
  if(strcmp(CswitchState,"Open")==0) SetOutput(TD.Cswitch,TD.CswitchLevel);
  else ClearOutput(TD.Cswitch,TD.CswitchLevel);
}

// Called when the timer reaches the desired time point.
// This function will update the state
void CompressorTimerISR(void)
{
  char OP;
  
  // This interrupt occurs when the current state has timed out so advance to the next
  switch (CState)
  {
    case CS_COMPRESS:
      // Next state is always normal
      ClockReset = 8;
      C_NextEvent += C_Tn;
      CState = CS_NORMAL;
      break;
    case CS_NORMAL:
    case CS_NONCOMPRESS:
      CurrentPass++;
    case CS_TRIG:
      // State will be Compress, NonCompress or 0 indicating finished, defined by table value
      OP = GetNextOperationFromTable(false);
      if(OP == 'C')
      {
        CState = CS_COMPRESS;
        C_NextEvent += C_Tc;
        ClockReset = TDarray[0].Corder * 8;
      }
      else if(OP == 'N')
      {
        CState = CS_NONCOMPRESS;
        C_NextEvent += C_Tnc;
        ClockReset = 8;
      }
      break;
    default:
      break;
  }
  // Test if all passes are complete, if so stop the timer and exit
  if(OP == 0)
  {
    // Stop the timer
    CompressorTimer.stop();
    // Restore the mode
    UpdateMode();
    return;
  }
  // Update the timer
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
}

// Called when we are going to start a compression cycle
void CompressorTriggerISR(void)
{
  // Clear and setup variables
  ClockReset = 8;             // Put system in normal mode
  CurrentPass = 0;
  GetNextOperationFromTable(true);
  C_NextEvent = C_Td;
  CState = CS_TRIG;
  // Setup the timer used to generate interrupts
  CompressorTimer.begin();
  CompressorTimer.setTrigger(TC_CMR_EEVTEDG_NONE);
  CompressorTimer.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);
  CompressorTimer.attachInterruptRA(CompressorTimerISR);
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
  CompressorTimer.enableTrigger();
  CompressorTimer.softwareTrigger();
}

// This function reads the compressor table and return the next operation that will be performed.
// The returned value is:
//                        'N' for non compressed cycle
//                        'C' for compressed cycle
//                         0 at when done
// This function will process commands in the table that perform setting updates.
//     Valid commands that are processed by this function:
//                        'S' for switch control, 0 to close, 1 to open
//                        'O' order, 0 to 127 are valid values
// If the init is true that the table pointers are reset to the start and the function return 0;
char GetNextOperationFromTable(bool init)
{
  static int tblindex=0;
  static char OP;
  static int count = 0;

  if(init)
  {
    tblindex = 0;
    count = 0;
    return(0);
  }
  if(count > 0)
  {
    count--;
    return(OP);
  }
  while(1)
  {
    // Find a valid character
    while(1)
    {
      if(TwaveCompressorTable[tblindex] == 0) return(0);
      OP = TwaveCompressorTable[tblindex++];
      count = 1;  // Default to count of 1
      if(isDigit(TwaveCompressorTable[tblindex]))
      {
        // If here then get the value, it has to be an integer
        count = int(TwaveCompressorTable[tblindex++] - '0');
        while(isDigit(TwaveCompressorTable[tblindex])) count = count * 10 + int(TwaveCompressorTable[tblindex++] - '0');
      }
    }
    if((OP=='N')||(OP=='C'))
    {
      // Only return valid options, N for non compressed and C for compressed
      count--;
      return(OP);
    }
    if(OP == 'S')
    {
      if(count == 0) ClearOutput(TD.Cswitch,TD.CswitchLevel);
      if(count == 1) SetOutput(TD.Cswitch,TD.CswitchLevel);
    }
    if(OP == 'O')
    {
      if((count >= 0) && (count <= 127))
      {
         TDarray[0].Corder = count;
         TD.Corder = count;
         UpdateMode();
      }
    }
  }
}

// This interrupt service routine generates the two clocks of each Twave module
void CompressorClockISR(void)
{
  uint32_t i;
  
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Generate 4 clock pulses
  i = ClockArray[ClockIndex];
  pio->PIO_SODR = i;    // Set bit high
  pio->PIO_CODR = i;    // Set bit low
  pio->PIO_SODR = i;    // Set bit high
  pio->PIO_CODR = i;    // Set bit low
  pio->PIO_SODR = i;    // Set bit high
  pio->PIO_CODR = i;    // Set bit low
  pio->PIO_SODR = i;    // Set bit high
  pio->PIO_CODR = i;    // Set bit low
  ClockIndex++;
  if(ClockReset == 0) ClockReset = 16;
  if(ClockIndex >= CR) 
  {
    CR = ClockReset;
    if((TD.Corder == 0) && (ClockIndex == 16)) ClockIndex = 8;  // for order = 0 the system will just hold its position.
    else ClockIndex = 0;
  }
}

// This function generates one clock cycle on both the software clock pins. 
// Used in the compressor reset functions.
void CompressorClockCycle(void)
{
  if(!TDarray[0].CompressorEnabled) return;
  // Generate 1 clock cycle
  pio->PIO_SODR = ClockArray[0];    // Set bit high
  pio->PIO_CODR = ClockArray[0];    // Set bit low
}

// Reset the clocks used in the compressor
void CompressorClockReset(void)
{
  if(!TDarray[0].CompressorEnabled) return;
  ClockIndex = 0;
}

void CompressorInit(void)
{
  int            i;
  DialogBoxEntry *de;
  
  #ifdef TestTwave
    TDarray[0].CompressorEnabled=true;
  #endif
  if(!TDarray[0].CompressorEnabled) return;  // Exit if the compressor is not enabled
  TDarray[0].UseCommonClock = true;          // If we are in compressor mode then we must use a common clock
  TDarray[1].UseCommonClock = true;
  pinMode(CLOCK_A, OUTPUT);  // Clock A output pin
  pinMode(CLOCK_B, OUTPUT);  // Clock B output pin
  // Setup the pio pointer
  pio = g_APinDescription[CLOCK_A].pPort;
  // Init the clock array used to generate the clocks
  for(i=0;i < MaxOrder * 8;i++)
  {
    ClockArray[i] =  g_APinDescription[CLOCK_A].ulPin;
    if(i < 8) ClockArray[i] |= g_APinDescription[CLOCK_B].ulPin;
  }
  // Setup the timer and the ISR for the clock
  TwaveClk.attachInterrupt(CompressorClockISR);
  TwaveClk.setFrequency((double)TDarray[0].Velocity);
  TwaveClk.start(-1, 0, false);
  // Clear the switch output
  ClearOutput(TDarray[0].Cswitch,TDarray[0].CswitchLevel);
  // Enable the compressor menu selection 
  de = GetDialogEntries(TwaveDialogEntries2page2, "Compressor");
  if(de != NULL) de->Type = D_DIALOG;
  // Setup the trigger line and ISR
  CtrigInput = new DIhandler;
  CtrigInput->detach();
  CtrigInput->attached(TDarray[0].Ctrig, TDarray[0].CtrigLevel, CompressorTriggerISR);
  UpdateMode();
  SetSwitch();
}

// This is the main loop function for the compressor mode. This function
// will exit if the compressor flag is not set.
void CompressorLoop(void)
{
  if(!TDarray[0].CompressorEnabled) return;  // Exit if the compressor is not enabled
  // Calculate all the times in clock count units
  C_Td  = (TDarray[0].Tdelay / 1000.0) * C_clock;
  C_Tc  = (TDarray[0].Tcompress / 1000.0) * C_clock;
  C_Tn  = (TDarray[0].Tnormal / 1000.0) * C_clock;
  C_Tnc = (TDarray[0].TnoC / 1000.0) * C_clock;
  if (ActiveDialog == &CompressorDialog) RefreshAllDialogEntries(&CompressorDialog);
}

// End of compressor code

//
// The following code supports the Compressor host commands. 
//
//   GTWCTBL          Get Twave compressor table
//   STWCTBL          Set Twave compressor table
//   GTWCMODE         Get Twave compressor mode
//   STWCMODE         Set Twave compressor mode
//   GTWCORDER        Get Twave compressor order
//   STWCORDER        Set Twave compressor order
//   GTWCTD           Get Twave compressor trigger delay, mS
//   STWCTD           Set Twave compressor trigger delay, mS
//   GTWCTC           Get Twave compressor compress time, mS
//   STWCTC           Set Twave compressor compress time, mS
//   GTWCTN           Get Twave compressor normal time, mS
//   STWCTN           Set Twave compressor normal time, mS
//   GTWCTNC          Get Twave compressor non compress time, mS
//   STWCTNC          Set Twave compressor non compress time, mS
//   TWCTRG           Software trigger the compressor
//   GTWCSW           Get Twave compressor Switch state
//   STWCSW           Set Twave compressor Switch state

void SetTWCmode(char *mode)
{
  if ((strcmp(mode, "Normal") == 0) || (strcmp(mode, "Compress") == 0))
  {
    strcpy(Cmode,mode);
    UpdateMode();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}
void GetTWCorder(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[0].Corder);
}
bool RangeTest(DialogBoxEntry *des, char *EntryName, float fval)
{
  DialogBoxEntry *de;
  
  de = GetDialogEntries(des, EntryName);
  if((fval >= de->Min) && (fval <= de->Max)) return true;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}
void SetTWCorder(int ival)
{
  if(RangeTest(CompressorEntries,"Order",ival))
  {
    TDarray[0].Corder = ival;
    TD.Corder = ival;
    UpdateMode();
    SendACK;
  }
}
void SetTWCtriggerDelay(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(CompressorEntries,"Trig delay, mS",fval))
  {
    TDarray[0].Tdelay = fval;
    TD.Tdelay = fval;
    SendACK;
  }
}
void SetTWCcompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(CompressorEntries,"Compress t, mS",fval))
  {
    TDarray[0].Tcompress = fval;
    TD.Tcompress = fval;
    SendACK;
  }
}
void SetTWCnormalTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(CompressorEntries,"Normal t, mS",fval))
  {
    TDarray[0].Tnormal = fval;
    TD.Tnormal = fval;
    SendACK;
  }
}
void SetTWCnoncompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(CompressorEntries,"Normal t, mS",fval))
  {
    TDarray[0].TnoC = fval;
    TD.TnoC = fval;
    SendACK;
  }
}
void TWCtrigger(void)
{
   SendACKonly;
   CompressorTriggerISR();
}
void SetTWCswitch(char *mode)
{
  if ((strcmp(mode, "Open") == 0) || (strcmp(mode, "Close") == 0))
  {
    strcpy(CswitchState,mode);
    SetSwitch();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}
// End of compressor host command routines




