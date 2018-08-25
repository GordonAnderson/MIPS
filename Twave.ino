//
// Twave
//
// This file supports the Twave driver card, rev 1.0, 2.0, 3.0, 4.0 and 5.0.
// While support is still present for rev 1.0 this version has been retired.
//
// This version of the Twave driver does not use the readback ADC. 
//
// Needed to lower frequency below the 250KHz limit of the clock generator chip. Setup
// Timer channel 6 to output on pin 5, that is on connector EXT2 pin 9. Jumper EXT2 pin 9
// to pin 14 and then this signal can be found on pin 14 of IC10. This hardware mod is
// required for rev 2.0. Also required to use timer channel 6 for clock generation on 
// Rev 2.0
//
// Support added for Twave driver module rev 3.0. December 19. 2015 This version
// uses a clock chip that will function over the required range.
//
// Rev 4.0 of the Twave module adds a Coolrunner CPLD to the hardware. This allows
// all the compressor clock logic to be moved into hardware on the module. The CPLD
// has a 16 bit SPI interface used to control all the options. Hardware signals
// for SPI data, SPI clock, Strobe, Select provide the interface to the Due controller.
// On board jumpers allow on board or external clock.
//
// Firmwave revs do not always match the hardware board versions.
// Use Rev 1 firmware for Rev 1 hardware module
// Use Rev 2 firmware for Rev 2 hardware module
// Use Rev 3 firmware for Rev 3 hardware module
// Use Rev 4 firmware for Rev 4 hardware module (format to rev 3 and then use the SMREV command to set to rev 4)
// Use Rev 5 firmware for Rev 4.1 hardware module (500 volt operation)
//
// Added features:
//    0.) Added support for 2 twave boards.
//    1.) Added menu option to allow sync trigger input.
//    2.) Added menu option to allow digitial input of direction control.
//    3.) Using different bits for direction control of module A and B.
//    4.) Add menu option to allow second module to use first modules clock
//        this also requires a board jumper. Add UseCommonClock flag and logic.
//    5.) Updated all the serial commands to accpect a Twave channel number.
//    6.) Add the compressor function. The is only valid on a dual Twave system.
//        - Use EXT2-14 (DUE 9, c.21) for the clock on board A, no changes needed.
//        - Use EXT2-12 (DUE 8, c.22) for the clock on board B, requires a cut and jump
//          need to route this signal through EXT2-14 pin on board B.
//        - Add compressor menu option when enabled
//            - Compression factor, 1 to 255
//            - Enable, ON/OFF
//            - External input for enable
//        - ISR to support the clock function
//        - Timer ISR used to generate the multi-pass timing.
// Add gate function and gate serial command. Then gated off all outputs will go low and no change
// when gated back on the bit pattern will reload and start. Maybe we can just set the pulse voltage 
// to zero, this would be the simple thing to do but still needs TWI interface.
//
// Ahmed requested the frequenncy sweep function. This will allow defining the following
//  -Start freq
//  -Stop freq
//  -sweep time in seconds
//  -Start and Stop
//  - Must support two twave channels, with syncronized start
//  - Commands:
//      STWSSTRT,chan,freq    // defines start freq
//      GTWSSTRT,chan
//      STWSSTP,chan,freq     // defines stop freq
//      GTWSSTP,chan
//      STWSTM,chan,time      // defines sweep time in seconds
//      GTWSTM,chan
//      STWSGO,chan           // Starts the sweep on channel 1 or 2, or 3 for both
//      STWSHLT,chan          // Stops the sweep on channel 1 or 2, or 3 for both
//      GTWSTA,chan           // Returns selected channel scan status, running or stopped
//   Implementation details
//    - Parameters are not saved
//    - Updated 10 times per second
//
// Gordon Anderson
//
#include "Twave.h"
#include "Hardware.h"
#include "Menu.h"
#include "Dialog.h"
#include "pwm01.h"
#include "Variants.h"
#include "Compressor.h"

extern bool NormalStartup;

// This define enables the use of timer6 to generate the sequence generator clock.
// Applies to rev 1.0 and rev 2.0 only
#define UseTimer

// Use the timer channel 6 for a clock in rev 1.0 to get to lower frequencies.
// Also used on rev 2 due to issues with the clock chip.
// This timer is also used to generate the software clock in the compressor mode.
// 6,4 works
//MIPStimer TwaveClk(6);
//MIPStimer TwaveClk(TMR_TwaveClk);
MIPStimer *TwaveClk = NULL;

// DAC channel assignment
#define  DAC_PulseVoltage    0
#define  DAC_RestingVoltage  1
#define  DAC_Guard1          2
#define  DAC_Guard2          3

// ADC channel assignment
#define  ADC_PulseVoltage    0
#define  ADC_RestingVoltage  1
#define  ADC_Guard1          2
#define  ADC_Guard2          3

#define   TWreset  ADDR0
#define   TWload   ADDR1      // Only used on rev 1.0, upgraded to TWld on rev 2.0

// Rev 2.0 bit definitions for PLD sequence generator
#define   TWdirA       49
#define   TWdirB       16
#define   TWclr        14
#define   TWld         48

#define   MinPulseVoltage        7
#define   MinPulseVoltageRev4    5
#define   MinPulseVoltageRev5    20
#define   MinPulseVoltageSave    15

MenuEntry METwaveMonitor =  {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog , NULL, NULL};   // Rev 1 menu
MenuEntry METwaveMonitor2 = {" Twave module", M_DIALOG, 0, 0, 0, NULL, &TwaveDialog2, NULL, NULL};   // Rev 2 and 3 menu

extern DialogBoxEntry TwaveDialogEntries2page2[];
extern DialogBoxEntry TwaveDialogEntriesCalPage[];
extern DialogBox      CompressorDialog;

// Twave system variables
int       NumberOfTwaveModules  = 0;
bool      TwaveBoards[2]        = {false, false};
uint8_t   TwaveEEPROMaddr[2] = {0x50, 0x50}; // Addresses of the on module EEPROM
int       TwaveModuleNumber     = 1;         // User interface selected module number
TwaveData TDarray[2] = {Twave_Rev3, Twave_Rev3};
int       SelectedTwaveModule = 0;           // Selected Twave module index, 0 or 1 for module 1 or 2
int       TWboardAddress[2] = { -1, -1};     // Contains board A and B addresses, 0 or 1, if we have two boards its always 0,1. if its one board the could be 0,-1 or 1,-1
// Voltage readbacks
float     ReadbackPV[2] = {0,0};
float     ReadbackG1[2] = {0,0};
float     ReadbackG2[2] = {0,0};
int16_t   TwaveTestDelay= 200;               // Hold off for testing delay when voltages are changed, 20 sec initial delay to allow disabling
// To select the hardware board do the following: SelectBoard(TWboardAddress[SelectedTwaveBoard]);
TwaveData TD;

bool TW_1_gatedOff = false;
bool TW_2_gatedOff = false;

// CPLD SPI input register image variables.
uint16_t  TWcpld[2] = {TWMdir | TWMload | TWMmode | TWMsrena | 0x0F, TWMdir | TWMload | TWMmode | TWMsrena | 0x0F};
//uint16_t  TWcpld[2] = {TWMdir | TWMmode | TWMsrena | 0x0F, TWMdir | TWMmode | TWMsrena | 0x0F};

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
  
  if(TDarray[TWboardAddress[0]].Rev >= 4)
  {
     if((TWcpld[TWboardAddress[0]] & TWMsrena) != 0) return;
     AtomicBlock< Atomic_RestoreState > a_Block;  
     TWcpld[TWboardAddress[0]] |= TWMsrena;
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[0]);
     TWcpldLoad(TWboardAddress[0]);
     SelectBoard(b);
     return;
  }
  // If the gate voltage is > 0 then set the pulse voltage to
  // the gate voltage. If the gate voltage is 0 then stop the twave
  if(TDarray[0].GateV > 7.0)
  {
    brd=SelectedBoard();
    SelectBoard(0);
    SetPulseVoltage(0);
    SelectBoard(brd);
    if(!TW_1_gatedOff) return;
  }
  // Read and save board select then select the board and update the sequence to 0xFF
  brd=SelectedBoard();
  SelectBoard(0);
  MCP2300(TDarray[0].GPIOadr, ~TDarray[0].Sequence);
  TW_1_gatedOff = false;
  SetPulseVoltage(0);   // Just in case it was reduced
  SelectBoard(brd);
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
  
  if(TDarray[TWboardAddress[0]].Rev >= 4)
  {
     if((TWcpld[TWboardAddress[0]] & ~TWMsrena) == 0) return;
     AtomicBlock< Atomic_RestoreState > a_Block;  
     TWcpld[TWboardAddress[0]] &= ~TWMsrena;
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[0]);
     TWcpldLoad(TWboardAddress[0]);
     SelectBoard(b);
     return;
  }
  // If the gate voltage is > 0 then set the pulse voltage to
  // the default voltage. If the gate voltage is 0 then start the twave
  if(TDarray[0].GateV > 7.0)
  {
    brd = SelectedBoard();
    SelectBoard(0);
    SetPulseVoltage(0,TDarray[0].GateV);
    SelectBoard(brd);
    return;
  }
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = SelectedBoard();
  SelectBoard(0);
  MCP2300(TDarray[0].GPIOadr, 0xFF);
  TW_1_gatedOff = true;
  SelectBoard(brd);
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
  if((((level) && (TDarray[TWboardAddress[0]].TWgateLevel == HIGH)) || ((!level) && (TDarray[TWboardAddress[0]].TWgateLevel == LOW))) || 
     (((level) && (TDarray[TWboardAddress[0]].TWgateLevel == RISING)) || ((!level) && (TDarray[TWboardAddress[0]].TWgateLevel == FALLING))))
  {
    // Gate Twave off, Hold load line active and set sequence to 0xFF
    if(TDarray[TWboardAddress[0]].Rev >= 4) TW_1_GateOff();
    else
    {
       if(AcquireTWI()) TW_1_GateOff();
       else TWIqueue(TW_1_GateOff);
    }
  }
  else
  {
    // Gate Twave on, Set the sequence and pulse the load line
    if(TDarray[TWboardAddress[0]].Rev >= 4) TW_1_GateOn();
    else
    {
       if(AcquireTWI()) TW_1_GateOn();
       else TWIqueue(TW_1_GateOn);
    }
  }
}

void TW_2_GateOn(void)
{
  int brd,mSdelay;

  if(TDarray[TWboardAddress[1]].Rev >= 4)
  {
     if((TWcpld[TWboardAddress[1]] & TWMsrena) != 0) return;
     AtomicBlock< Atomic_RestoreState > a_Block;  
     TWcpld[TWboardAddress[1]] |= TWMsrena;
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[1]);
     TWcpldLoad(TWboardAddress[1]);
     SelectBoard(b);
     return;
  }
  // If the gate voltage is > 0 then set the pulse voltage to
  // the gate voltage. If the gate voltage is 0 then stop the twave
  if(TDarray[1].GateV > 7.0)
  {
    brd=SelectedBoard();
    SelectBoard(1);
    SetPulseVoltage(1);
    SelectBoard(brd);
    if(!TW_2_gatedOff) return;
  }
  // Read and save board select then select the board and update the sequence to 0xFF
  brd=SelectedBoard();
  SelectBoard(1);
  MCP2300(TDarray[1].GPIOadr, ~TDarray[1].Sequence);
  TW_2_gatedOff = false;
  SetPulseVoltage(1);
  SelectBoard(brd);
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

  if(TDarray[TWboardAddress[1]].Rev >= 4)
  {
     if((TWcpld[TWboardAddress[1]] & ~TWMsrena) == 0) return;
     AtomicBlock< Atomic_RestoreState > a_Block;  
     TWcpld[TWboardAddress[1]] &= ~TWMsrena;
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[1]);
     TWcpldLoad(TWboardAddress[1]);
     SelectBoard(b);
     return;
  }
  // If the gate voltage is > 0 then set the pulse voltage to
  // the default voltage. If the gate voltage is 0 then start the twave
  if(TDarray[1].GateV > 7.0)
  {
    brd = SelectedBoard();
    SelectBoard(1);
    SetPulseVoltage(1,TDarray[1].GateV);
    SelectBoard(brd);
    return;
  }
  // Read and save board select then select the board and update the sequence to 0xFF
  brd = SelectedBoard();
  SelectBoard(1);
  MCP2300(TDarray[1].GPIOadr, 0xFF);
  TW_2_gatedOff = true;
  SelectBoard(brd);
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
  if((((level) && (TDarray[1].TWgateLevel == HIGH)) || ((!level) && (TDarray[1].TWgateLevel == LOW))) || 
     (((level) && (TDarray[1].TWgateLevel == RISING)) || ((!level) && (TDarray[1].TWgateLevel == FALLING))))
  {
    // Gate Twave off, Hold load line active and set sequence to 0xFF
    if(TDarray[TWboardAddress[1]].Rev >= 4) TW_2_GateOff();
    else
    {
       if(AcquireTWI()) TW_2_GateOff();
       else TWIqueue(TW_2_GateOff);      
    }
  }
  else
  {
    // Gate Twave on, Set the sequence and pulse the load line
    if(TDarray[TWboardAddress[1]].Rev >= 4) TW_2_GateOn();
    else
    {
       if(AcquireTWI()) TW_2_GateOn();
       else TWIqueue(TW_2_GateOn);
    }
  }
}

void TW_1_DIR_ISR(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;  
  if((TDarray[0].TWdirLevel == HIGH) || (TDarray[0].TWdirLevel == RISING))
  {
    if (DIdirTW[0]->activeLevel()) TDarray[0].Direction = true;
    else TDarray[0].Direction = false;    
  }
  else
  {
    if (!DIdirTW[0]->activeLevel()) TDarray[0].Direction = true;
    else TDarray[0].Direction = false;        
  }
  if(TDarray[TWboardAddress[0]].Rev >= 4)
  {
     if((TDarray[0].Direction) && ((TWcpld[TWboardAddress[0]] & TWMdir) != 0)) return;
     if((!TDarray[0].Direction) && ((TWcpld[TWboardAddress[0]] & TWMdir) == 0)) return;
     if(TDarray[0].Direction) TWcpld[TWboardAddress[0]] |= TWMdir;
     else TWcpld[TWboardAddress[0]] &= ~TWMdir;
     // Update the CPLD
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[0]);
     TWcpldLoad(TWboardAddress[0]);
     SelectBoard(b);
     return;
  }
  if (TDarray[0].Direction) digitalWrite(TWdirA, HIGH);
  else digitalWrite(TWdirA, LOW);
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}
void TW_2_DIR_ISR(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;  
  if((TDarray[TWboardAddress[1]].TWdirLevel == HIGH) || (TDarray[TWboardAddress[1]].TWdirLevel == RISING))
  {
    if (DIdirTW[TWboardAddress[1]]->activeLevel()) TDarray[TWboardAddress[1]].Direction = true;
    else TDarray[TWboardAddress[1]].Direction = false;    
  }
  else
  {
    if (!DIdirTW[TWboardAddress[1]]->activeLevel()) TDarray[TWboardAddress[1]].Direction = true;
    else TDarray[TWboardAddress[1]].Direction = false;        
  }
  if(TDarray[TWboardAddress[1]].Rev >= 4)
  {
     if((TDarray[TWboardAddress[1]].Direction) && ((TWcpld[TWboardAddress[TWboardAddress[1]]] & TWMdir) != 0)) return;
     if((!TDarray[TWboardAddress[1]].Direction) && ((TWcpld[TWboardAddress[TWboardAddress[1]]] & TWMdir) == 0)) return;
     // Only update if there is a change
     if(TDarray[1].Direction) TWcpld[TWboardAddress[1]] |= TWMdir;
     else TWcpld[TWboardAddress[1]] &= ~TWMdir;
     // Update the CPLD
     int b=SelectedBoard();
     SelectBoard(TWboardAddress[1]);
     TWcpldLoad(TWboardAddress[1]);
     SelectBoard(b);
     return;
  }
  if (TDarray[TWboardAddress[1]].Direction) digitalWrite(TWdirB, HIGH);
  else digitalWrite(TWdirB, LOW);
  digitalWrite(TWld, LOW);
  delayMicroseconds(250);
  CompressorClockCycle();
  CompressorClockReset();
  digitalWrite(TWld, HIGH);
}

void TW_1_SYNC_ISR(void)
{
  uint8_t  *b;

  if(TDarray[0].Rev >= 4)
  {
     AtomicBlock< Atomic_RestoreState > a_Block;
     if((TwaveBoards[1]) && (TDarray[0].TWsyncDI == TDarray[1].TWsyncDI) && (TDarray[0].TWsyncLevel == TDarray[1].TWsyncLevel))
     {
        b = (uint8_t *)&TWcpld[1];
        b[0] = ~TDarray[1].Sequence;
        TWcpld[1] |= TWMload;
     }
     b = (uint8_t *)&TWcpld[0];
     b[0] = ~TDarray[0].Sequence;
     TWcpld[0] |= TWMload;
     TWcpldLoad(1,false);
     TWcpldLoad(0,true);
     TWcpld[0] &= ~TWMload;
     TWcpld[1] &= ~TWMload;
     // Need to delay one clock period at this point to let the load happen in the hardware
     delayMicroseconds(1000000/TDarray[0].Velocity);
     TWcpldLoad(1,false);
     TWcpldLoad(0,false);
     return;
  }
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
  uint8_t  *b;

  if(TDarray[1].Rev >= 4)
  {
     AtomicBlock< Atomic_RestoreState > a_Block;
     if((TwaveBoards[0]) && (TDarray[0].TWsyncDI == TDarray[1].TWsyncDI) && (TDarray[0].TWsyncLevel == TDarray[1].TWsyncLevel)) return;
     b = (uint8_t *)&TWcpld[1];
     b[0] = ~TDarray[1].Sequence;
     TWcpld[1] |= TWMload;
     TWcpldLoad(0,false);
     TWcpldLoad(1,true);
     TWcpld[1] &= ~TWMload;
     delayMicroseconds(1000000/TDarray[0].Velocity);
     TWcpldLoad(1,false);
     return;
  }
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
//  SelectBoard(TWboardAddress[ModuleIndex]);
  if (TDarray[ModuleIndex].Rev >= 4)
  {
    FS7140setup(TDarray[ModuleIndex].CLOCKadr,TDarray[ModuleIndex].Velocity);   
  }
  else if (TDarray[ModuleIndex].Rev == 3)
  {
    // Rev 3 supports the on module FS7140 clock chip.
    if(TDarray[0].CompressorEnabled)
    {
      TwaveClk->attachInterrupt(CompressorClockISR);
      TwaveClk->setFrequency((double)TDarray[0].Velocity);
      TwaveClk->start(-1, 0, false);
    }
    else FS7140setup(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity * 4);
  }
  else if (TDarray[ModuleIndex].Rev == 2)
  {
#ifdef UseTimer
    // The 8 times multiplier is because the output toggles on each clock thus divides
    // by two and the rev 2 hardware has a 4x division.
    TwaveClk->setFrequency((double)(TDarray[ModuleIndex].Velocity * 8));
    TwaveClk->setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk->start(-1, 0, true);
#else
    SetRef(8000000);
    CY_Init(TDarray[ModuleIndex].CLOCKadr);
    SetPLL2freq(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity * 8);
#endif
  }
  else // Rev 1
  {
#ifdef UseTimer
    // The 2 times multiplier is because the output toggles on each clock thus divides
    // by two.
    TwaveClk->setFrequency((double)(TDarray[ModuleIndex].Velocity * 2));
    TwaveClk->setTIOAeffect(1, TC_CMR_ACPA_TOGGLE);
    TwaveClk->start(-1, 0, true);
#else
    SetRef(8000000);
    CY_Init(TDarray[ModuleIndex].CLOCKadr);
    SetPLL2freq(TDarray[ModuleIndex].CLOCKadr, TDarray[ModuleIndex].Velocity);
#endif
  }
}

void SetSequence(int ModuleIndex)
{
  int     i;
  int     numTry = 0;
  uint8_t *b;

  if(TDarray[ModuleIndex].Rev >= 4)
  {
    b = (uint8_t *)&TWcpld[TWboardAddress[ModuleIndex]];
    // Load image with sequence in lower 8 bits and set the Load bit
    b[0] = ~TDarray[ModuleIndex].Sequence;
    TWcpld[TWboardAddress[ModuleIndex]] |= TWMload;
    // Update the CPLD
    TWcpldLoad(TWboardAddress[ModuleIndex]);
    // Clear the load bit in image register
    TWcpld[TWboardAddress[ModuleIndex]] &= ~TWMload;
    TWcpldLoad(TWboardAddress[ModuleIndex],false);
    return;
  }
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
    // These long delay are needed because the clr and load are synchronous and require a clock
    // edge. This delay can be calculated from the clock frequency to improve performance.
    // Clear and load may be able to happen at the same time.
    digitalWrite(TWclr, LOW);
    delayMicroseconds(1000);
    digitalWrite(TWclr, HIGH);
    delayMicroseconds(1000);
    digitalWrite(TWld, LOW);
    delayMicroseconds(1000);
    CompressorClockCycle();     // This is insuring the edge happens, should allow removing the delays.
                                // This is only true if using the software generated clock. This logic needs to
                                // include the proper tests
    CompressorClockReset();
    digitalWrite(TWld, HIGH);
    delayMicroseconds(1000);    // Do not see the need for this delay.
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
  int   DACcounts;
  float MinPV;

  if(TDarray[ModuleIndex].Rev == 5) MinPV = MinPulseVoltageRev5;
  else if(TDarray[ModuleIndex].Rev == 4) MinPV = MinPulseVoltageRev4;
  else MinPV = MinPulseVoltage;
  //  SelectBoard(TWboardAddress[SelectedTwaveBoard]);
  if (TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint < (TDarray[ModuleIndex].TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPV)) TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint = TDarray[ModuleIndex].TWCD[DAC_RestingVoltage].VoltageSetpoint + MinPV;
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].VoltageSetpoint, &TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].DCctrl);
  AD5625(TDarray[ModuleIndex].DACadr, TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].DCctrl.Chan, DACcounts);
}

void SetPulseVoltage(int ModuleIndex, float voltage)
{
  int DACcounts;
  float MinPV;

  if(TDarray[ModuleIndex].Rev == 5) MinPV = MinPulseVoltageRev5;
  else if(TDarray[ModuleIndex].Rev == 4) MinPV = MinPulseVoltageRev4;
  else MinPV = MinPulseVoltage;
  if(voltage < MinPV) voltage = MinPV;
  // Use calibration data to convert engineering units requested to DAC counts.
  DACcounts = Value2Counts(voltage, &TDarray[ModuleIndex].TWCD[DAC_PulseVoltage].DCctrl);
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
  float pv;
  
  pv = TDarray[SelectedTwaveModule].TWCD[DAC_PulseVoltage].VoltageSetpoint;
  if(pv < MinPulseVoltageSave) pv = MinPulseVoltageSave;
  if (WriteEEPROM(&TDarray[SelectedTwaveModule], TwaveEEPROMaddr[SelectedTwaveModule], 0, sizeof(TwaveData)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
  TDarray[SelectedTwaveModule].TWCD[DAC_PulseVoltage].VoltageSetpoint = pv;
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
      if(isnan(TDarray[SelectedTwaveModule].GateV)) TDarray[SelectedTwaveModule].GateV = 0.0;
      SelectTwaveModule();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
  if(TDarray[SelectedTwaveModule].TWCD[DAC_PulseVoltage].VoltageSetpoint < MinPulseVoltageSave) TDarray[SelectedTwaveModule].TWCD[DAC_PulseVoltage].VoltageSetpoint = MinPulseVoltageSave;
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
  M_SCROLLING, 0, 0,false, TwaveDialogEntries
};

// Rev 2.0/3.0/4.0 dialog options
DialogBoxEntry TwaveDialogEntries2[] = {
  {" Twave module"       , 0, 1, D_INT, 1, 1, 1, 22, false, "%1d", &TwaveModuleNumber, NULL, SelectTwaveModule},
  {" Clock freq, Hz"     , 0, 2, D_INT, 3000, 300000, 1000, 16, false, "%7d", &TD.Velocity, NULL, NULL},
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
  M_SCROLLING, 0, 0,false, TwaveDialogEntries2
};

DialogBoxEntry TwaveDialogEntries2page2[] = {
  {" Calibrate"     , 0, 1, D_PAGE , 0, 0, 0, 0, false, NULL, &TwaveDialogEntriesCalPage, NULL, NULL},
  {" Sync input"    , 0, 2, D_DI     , 0, 0, 2, 21, false, DIlist, &TD.TWsyncDI, NULL, NULL},
  {" Sync level"    , 0, 3, D_DILEVEL, 0, 0, 4, 19, false, DILlist, &TD.TWsyncLevel, NULL, NULL},
  {" Dir input"     , 0, 4, D_DI     , 0, 0, 2, 21, false, NULL, &TD.TWdirDI, NULL, NULL},
  {" Dir level"     , 0, 5, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.TWdirLevel, NULL, NULL},
  {" Gate input"    , 0, 6, D_DI     , 0, 0, 2, 21, false, NULL, &TD.TWgateDI, NULL, NULL},
  {" Gate level"    , 0, 7, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.TWgateLevel, NULL, NULL},
  {" Gate Voltage"  , 0, 8, D_FLOAT  , 0, 100, 0.1, 18, false, "%5.1f", &TD.GateV, NULL, NULL},
  {" Compressor"    , 0, 10, D_OFF , 0, 0, 0, 0, false, NULL, &CompressorDialog, NULL,NULL},
  {" Return to first page", 0, 11, D_PAGE , 0, 0, 0, 0, false, NULL, &TwaveDialogEntries2, NULL, NULL},
  {NULL},
};

DialogBoxEntry TwaveDialogEntriesCalPage[] = {
  {" Cal Pulse V"   , 0, 1, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibratePulse, NULL},
  {" Cal Guard 1"   , 0, 2, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard1, NULL},
  {" Cal Guard 2"   , 0, 3, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateGuard2, NULL},
  {" Return"        , 0, 10, D_PAGE , 0, 0, 0, 0, false, NULL, &TwaveDialogEntries2page2, NULL, NULL},
  {NULL},
};

// The function loads the TW CPLD image register to the hardware
void TWcpldLoad(int index, bool Strobe)
{
  uint8_t  *b;

  AtomicBlock< Atomic_RestoreState > a_Block;
  b = (uint8_t *)&TWcpld[index];
  // Select the board
  if(index == 0) digitalWrite(TWselect,LOW);
  else digitalWrite(TWselect,HIGH);
  // Set the SPI address
  SetAddress(4);
  // Send data
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  SPI.transfer(SPI_CS, b[1], SPI_CONTINUE);
  SPI.transfer(SPI_CS, b[0]);
  // Pulse strobe and exit
  if(Strobe)
  {
     digitalWrite(TWstrobe,HIGH);
     digitalWrite(TWstrobe,LOW);
  }
  SetAddress(0);  
  delayMicroseconds(2);
}

// This function will load the sequence into both twave channel and
// clear the clock logic. This should sync the two channels
void TWcpldSync(void)
{
  uint8_t  *b;

  AtomicBlock< Atomic_RestoreState > a_Block;
  b = (uint8_t *)&TWcpld[0];
  b[0] = ~TDarray[0].Sequence;
  b = (uint8_t *)&TWcpld[1];
  b[0] = ~TDarray[1].Sequence;
  TWcpld[0] |= TWMload;
//  TWcpld[0] |= TWMclear;
  TWcpld[1] |= TWMload;
//  TWcpld[1] |= TWMclear;
  TWcpldLoad(0,false);
  TWcpldLoad(1,true);
  TWcpld[0] &= ~TWMload;
//  TWcpld[0] &= ~TWMclear;
  TWcpld[1] &= ~TWMload;
//  TWcpld[1] &= ~TWMclear;
  TWcpldLoad(0,false);
  TWcpldLoad(1,false);
}

void CalibratePulse(void)
{
  ChannelCal CC;
  char       Name[20];
  DialogBoxEntry *de = GetDialogEntries(TwaveDialogEntries2, "Pulse voltage");

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = de->Max;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[0].DCctrl;
  CC.ADCreadback = &TD.TWCD[0].DCmon;
  // Define this channels name
  sprintf(Name, "      Pulse V");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, de->Max * 0.20, de->Max * 0.75);
  TDarray[SelectedTwaveModule] = TD;
}

void CalibrateGuard1(void)
{
  ChannelCal CC;
  char       Name[20];
  DialogBoxEntry *de = GetDialogEntries(TwaveDialogEntries2, "Guard 1");

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = de->Max;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[2].DCctrl;
  CC.ADCreadback = &TD.TWCD[2].DCmon;
  // Define this channels name
  sprintf(Name, "     Guard 1");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, de->Max * 0.10, de->Max * 0.75);
  TDarray[SelectedTwaveModule] = TD;
}

void CalibrateGuard2(void)
{
  ChannelCal CC;
  char       Name[20];
  DialogBoxEntry *de = GetDialogEntries(TwaveDialogEntries2, "Guard 2");

  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = de->Max;
  CC.DACaddr = TD.DACadr;
  CC.ADCaddr = TD.ADCadr;
  CC.DACout = &TD.TWCD[3].DCctrl;
  CC.ADCreadback = &TD.TWCD[3].DCmon;
  // Define this channels name
  sprintf(Name, "     Guard 2");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, de->Max * 0.10, de->Max * 0.75);
  TDarray[SelectedTwaveModule] = TD;
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
  DialogBox      *SavedDialog;
  DialogBoxEntry *de;

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
  if(TwaveClk == NULL)
  {
    // Setup the timer channel based on board rev
    if(TD.Rev <= 2)
    {
      #ifdef UseTimer
      TwaveClk = new MIPStimer(6);    // These REVs have to use timer 6 for clock generation
      #endif
    }
    if(TwaveClk == NULL) TwaveClk = new MIPStimer(TMR_TwaveClk);
    if(TwaveClk == NULL) return;
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
  // If rev 4.0 setup the CPLD control bits and update the minimum pulse voltage
  if(TD.Rev >= 4)
  {
    pinMode(TWstrobe, OUTPUT);
    pinMode(TWselect, OUTPUT);
    digitalWrite(TWstrobe, LOW);
    digitalWrite(TWselect, LOW);
    de = GetDialogEntries(TwaveDialogEntries2page2, "Gate Voltage");
    de->Type = D_OFF; 
    de = GetDialogEntries(TwaveDialogEntries2, "Pulse voltage"); 
    de->Min = MinPulseVoltageRev4;
    if(TD.Rev == 5) de->Min = MinPulseVoltageRev5;
    if(TD.Rev == 4)
    {
       de = GetDialogEntries(TwaveDialogEntries2, "Clock freq, Hz"); 
       if(de != NULL)
       {
          de->Min = 5000;
          de->Max = 2000000;
       }
    }
  }
  // If rev 4 of the frimware driver then set the voltage upper limits to 500V
  if(TD.Rev == 5)
  {
    de = GetDialogEntries(TwaveDialogEntries2, "Pulse voltage");
    de->Max = 500;
    de->StepSize = 1;
    de = GetDialogEntries(TwaveDialogEntries2, "Guard 1");
    de->Max = 500;
    de->StepSize = 1;
    de = GetDialogEntries(TwaveDialogEntries2, "Guard 2");
    de->Max = 500;
    de->StepSize = 1;
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
    if ((TD.Rev == 2) || (TD.Rev == 3) || (TD.Rev == 4) || (TD.Rev == 5)) AddMainMenuEntry(&METwaveMonitor2);
    if ((TD.Rev == 2) || (TD.Rev == 3) || (TD.Rev == 4) || (TD.Rev == 5))
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

// This function reads the Pulse voltage and Guard readbacks and checks for a voltage
// error. If an error is found the system shuts down and resquets a re start.
void TwaveTests(void)
{
  int i;
  uint16_t counts;
  float val,error,MaxError;
  bool  TwaveVerror;
  
  for (i = 0; i < 2; i++)
  {
    if (TwaveBoards[i])
    {
       SelectBoard(TWboardAddress[i]);
       // Read the Pulse voltage and filter
       AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_PulseVoltage].DCmon.Chan);
       counts = AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_PulseVoltage].DCmon.Chan);
       val = Counts2Value(counts, &TDarray[i].TWCD[ADC_PulseVoltage].DCmon);
       ReadbackPV[i] = Filter * val + (1-Filter) * ReadbackPV[i];
       // Read Guard 1 voltage and filter
       AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_Guard1].DCmon.Chan);
       counts = AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_Guard1].DCmon.Chan);
       val = Counts2Value(counts, &TDarray[i].TWCD[ADC_Guard1].DCmon);
       ReadbackG1[i] = Filter * val + (1-Filter) * ReadbackG1[i];
       // Read Guard 2 voltage and filter
       AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_Guard2].DCmon.Chan);
       counts = AD7998(TDarray[i].ADCadr, TDarray[i].TWCD[ADC_Guard2].DCmon.Chan);
       val = Counts2Value(counts, &TDarray[i].TWCD[ADC_Guard2].DCmon);
       ReadbackG2[i] = Filter * val + (1-Filter) * ReadbackG2[i];
       // If rev 5 module and change delay has expired do the test
       if(TDarray[i].Rev == 5)
       {
          if(TwaveTestDelay <= 0)
          {
            TwaveVerror = false;
            // Look for error that is greater than 10% of setpoint or 2 volts, whatever is larger
            error = abs(ReadbackPV[i] - TDarray[i].TWCD[ADC_PulseVoltage].VoltageSetpoint);
            MaxError = TDarray[i].TWCD[ADC_PulseVoltage].VoltageSetpoint / 10;
            if(MaxError < 5) MaxError = 5;
            if(error > MaxError) TwaveVerror = true;
            error = abs(ReadbackG1[i] - TDarray[i].TWCD[ADC_Guard1].VoltageSetpoint);
            MaxError = TDarray[i].TWCD[ADC_Guard1].VoltageSetpoint / 10;
            if(MaxError < 5) MaxError = 5;
            if(error > MaxError) TwaveVerror = true;
            error = abs(ReadbackG2[i] - TDarray[i].TWCD[ADC_Guard2].VoltageSetpoint);
            MaxError = TDarray[i].TWCD[ADC_Guard2].VoltageSetpoint / 10;
            if(MaxError < 5) MaxError = 5;
            if(error > MaxError) TwaveVerror = true;
            if((TwaveVerror) && (TDarray[i].EnableTest))
            {
              // Shutdown the system, set sequence to 0 and voltages to 0, issue error message and
              // stop.
              TDarray[i].Sequence = 0;
              SetSequence(i);
              TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint = 0;
              SetPulseVoltage(i);
              TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint = 0;
              SetGuard1(i);
              TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint = 0;
              SetGuard2(i);
              // Display error message
              DisplayMessage("Twave Voltage Error!");
              // Stop and run only the main system loop to detect power loss,
              // also turn off the DCbias supply
              digitalWrite(PWR_ON,HIGH);
              while(1) 
              {
                WDT_Restart(WDT);  // Stop!
                delay(10);
                MIPSsystemLoop();
              }
            }
          }
          else TwaveTestDelay--;
       }
    }
  }
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

  if ((ActiveDialog == &TwaveDialog2)||(ActiveDialog == &TwaveDialog)||(ActiveDialog == &CompressorDialog)) 
  {
    if(ActiveDialog->Changed)
    {
       TDarray[SelectedTwaveModule] = TD;
       ActiveDialog->Changed = false;
    }
  }
  CompressorLoop();
  ProcessSweep();
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
         TwaveTestDelay = 100;
      }
      if (TDarray[i].Rev == 1) SetRestingVoltage();
      if(TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint != CurrentGuard1Voltage[i])
      {
         CurrentGuard1Voltage[i] = TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint;
         SetGuard1(i);
         TwaveTestDelay = 100;
      }
      if(TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint != CurrentGuard2Voltage[i])
      {
         CurrentGuard2Voltage[i] = TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint;
         SetGuard2(i);
         TwaveTestDelay = 100;
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
      if (TDarray[i].Rev >= 4)
      {
        if (CurrentDirection[i] != TDarray[i].Direction)
        {
          CurrentDirection[i] = TDarray[i].Direction;
          if(TDarray[i].Direction) TWcpld[TWboardAddress[i]] |= TWMdir;
          else TWcpld[TWboardAddress[i]] &= ~TWMdir;
          // Update the CPLD
          TWcpldLoad(TWboardAddress[i]);
        }
      }
      if (TDarray[i].Rev >= 2)
      {
        if ((TDarray[i].TWdirDI != DIdirTW[i]->di) || (TDarray[i].TWdirLevel != DIdirTW[i]->mode))
        {
          DIdirTW[i]->detach();
          DIdirTW[i]->attached(TDarray[i].TWdirDI, CHANGE, TWdirISRs[i]);
//          DIdirTW[i]->attached(TDarray[i].TWdirDI, TDarray[i].TWdirLevel, TWdirISRs[i]);
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
          CurrentVelocity[(i + 1) & 1] = TDarray[i].Velocity;
          SelectBoard(TWboardAddress[(i + 1) & 1]);
          SetVelocity((i + 1) & 1);
          SelectBoard(TWboardAddress[i]);
//          TD.Velocity = TDarray[i].Velocity;
        }
      }
      // Determine the highest Twave output voltage
      MaxTwaveVoltage = 0;
      if (TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_Guard1].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_Guard2].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_PulseVoltage].VoltageSetpoint;
      if (TDarray[i].TWCD[DAC_RestingVoltage].VoltageSetpoint > MaxTwaveVoltage) MaxTwaveVoltage = TDarray[i].TWCD[DAC_RestingVoltage].VoltageSetpoint;
    }
  }
  TwaveTests();
  SelectBoard(TWboardAddress[SelectedTwaveModule]);
  // Display the monitored values based on the dialog box curently being displayed
  if (ActiveDialog == &TwaveDialog2) RefreshAllDialogEntries(&TwaveDialog2);
  if (ActiveDialog == &TwaveDialog) RefreshAllDialogEntries(&TwaveDialog);
  TD = TDarray[SelectedTwaveModule];
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

// Set the TWAVE pulse voltage level. This function is called from the serial processor.
void setTWAVEpulseVoltage(char *chan, char *voltage)
{
  int    channel, index;
  String token;
  float  fval;

  token = chan;
  channel = token.toInt();
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  token = voltage;
  fval = token.toFloat();
  if (TD.Rev == 1) if(!RangeTest(TwaveDialogEntries, "Pulse voltage", fval)) return;
  else if(!RangeTest(TwaveDialogEntries2, "Pulse voltage", fval)) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[0].VoltageSetpoint = fval;
  TDarray[index].TWCD[0].VoltageSetpoint = fval;
  SendACK;
}

// Send the TWAVE guard 1 voltage level
void sendTWAVEguard1Voltage(int channel)
{
  int index;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[index].TWCD[2].VoltageSetpoint);
}

// Set the TWAVE guard 1 voltage level
void setTWAVEguard1Voltage(char *chan, char *voltage)
{
  int    channel, index;
  String token;
  float  fval;

  token = chan;
  channel = token.toInt();
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  token = voltage;
  fval = token.toFloat();
  if (TD.Rev == 1) if(!RangeTest(TwaveDialogEntries, "Guard 1", fval)) return;
  else if(!RangeTest(TwaveDialogEntries2, "Guard 1", fval)) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[2].VoltageSetpoint = fval;
  TDarray[index].TWCD[2].VoltageSetpoint = fval;
  SendACK;
}

// Send the TWAVE guard 2 voltage level
void sendTWAVEguard2Voltage(int channel)
{
  int index;

  index = GetTwaveIndex(channel);
  if (index == -1) return;
  SendACKonly;
  if (!SerialMute) serial->println(TDarray[index].TWCD[3].VoltageSetpoint);
}

// Set the TWAVE guard 2 voltage level
void setTWAVEguard2Voltage(char *chan, char *voltage)
{
  int    channel, index;
  String token;
  float  fval;

  token = chan;
  channel = token.toInt();
  index = GetTwaveIndex(channel);
  if (index == -1) return;
  token = voltage;
  fval = token.toFloat();
  if (TD.Rev == 1) if(!RangeTest(TwaveDialogEntries, "Guard 2", fval)) return;
  else if(!RangeTest(TwaveDialogEntries2, "Guard 2", fval)) return;
  // Update the voltage value and exit
  if (index == SelectedTwaveModule) TD.TWCD[3].VoltageSetpoint = fval;
  TDarray[index].TWCD[3].VoltageSetpoint = fval;
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

// This command enables or disables the voltage readback tests performed on Twave rev 5 (500V) system only.
// Valid values are TRUE and FALSE. This command sets the flag for both channels.
void SetTWenableTest(char *flag)
{
  String  sFlag;

  sFlag = flag;
  if((sFlag != "TRUE") && (sFlag != "FALSE"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
  }
  if(sFlag == "TRUE") TDarray[0].EnableTest = TDarray[1].EnableTest = true;
  else TDarray[0].EnableTest = TDarray[1].EnableTest = false;
  SendACK;
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

DialogBoxEntry CompressorEntries[] = {
  {" Mode"                , 0, 1, D_LIST   , 0,  0, 8, 15, false, CmodeList, Cmode, NULL, UpdateMode},
  {" Order"               , 0, 2, D_UINT8  , 0, 255, 1, 20, false, "%3d", &TD.Corder, NULL, UpdateMode},
  {" Compression table"   , 0, 3, D_TITLE  , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" "                    , 0, 4, D_STRING , 0, 2, 0, 2, false, "%-20.20s", TwaveCompressorTable, NULL, NULL},
  {" Trig delay, mS"      , 0, 5, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &TD.Tdelay, NULL, NULL},
  {" Compress t, mS"      , 0, 6, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &TD.Tcompress, NULL, NULL},
  {" Normal t, mS"        , 0, 7, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &TD.Tnormal, NULL, NULL},
  {" Non Comp t, mS"      , 0, 8, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &TD.TnoC, NULL, NULL},
  {" Next page"           , 0, 10, D_PAGE  , 0, 0, 0, 0, false, NULL, &CompressorEntries2, NULL, NULL},
  {" Return to Twave menu", 0, 11, D_DIALOG, 0, 0, 0, 0, false, NULL, &TwaveDialog2, NULL, NULL},
  {NULL},
};

DialogBoxEntry CompressorEntries2[] = {
  {" Trig input"          , 0, 1, D_DI     , 0, 0, 2, 21, false, NULL, &TD.Ctrig, NULL, ConfigureTrig},
  {" Trig level"          , 0, 2, D_DILEVEL, 0, 0, 4, 19, false, NULL, &TD.CtrigLevel, NULL, ConfigureTrig},
  {" Switch output"       , 0, 3, D_DO     , 0, 0, 2, 21, false, NULL, &TD.Cswitch, NULL, SetSwitch},
  {" Switch level"        , 0, 4, D_DOLEVEL, 0, 0, 4, 19, false, NULL, &TD.CswitchLevel, NULL, SetSwitch},
  {" Switch state"        , 0, 5, D_LIST   , 0, 0, 5, 18, false, CswitchList, CswitchState, NULL, SetSwitch},
  {" Cramp"               , 0, 7, D_OFF    , -200, 200 ,   1, 19, false, "%4d", &TD.CompressRamp, NULL, NULL},
  {" Cramp order"         , 0, 8, D_OFF    , 1, 200 ,   1, 19, false, "%4d", &TD.CrampOrder, NULL, NULL},
  {" Force trigger"       , 0, 9, D_FUNCTION,0, 0, 0, 0,  false, NULL, NULL, CompressorTriggerISR, CompressorTriggerISR},
  {" First page"          , 0, 10,D_PAGE   , 0, 0, 0, 0,  false, NULL, &CompressorEntries, NULL, NULL},
  {" Return to Twave menu", 0, 11,D_DIALOG , 0, 0, 0, 0,  false, NULL, &TwaveDialog2, NULL, NULL},
  {NULL},
};

DialogBox CompressorDialog = {{"Compressor params", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0,false, CompressorEntries
};

#define MaxOrder  255

#define CLOCK_A   9
#define CLOCK_B   8

volatile uint32_t  ClockArray[MaxOrder * 8];
volatile int       ClockReset = 16;
volatile int       ClockIndex = 0;
volatile int       CR = 16;
volatile int       CrampCounter = 0;
volatile bool      C_ClockEnable = true;

volatile Pio *pio;

void UpdateMode(void)
{
  uint8_t *b;
  
  if(TDarray[0].Rev >= 4)
  {
    // Rev 4.0 uses a CPLD for clock and compression logic
    uint8_t *b = (uint8_t *)&TWcpld[1];    // board address 1 is always the compressor channel
    b[0] = TDarray[0].Corder;
    if(strcmp(Cmode,"Normal")==0) TWcpld[1] |= TWMmode;
    else TWcpld[1] &= ~TWMmode;
    TWcpld[1] |= TWMorder;
    if(TDarray[0].Corder == 0)
    {
      TWcpld[1] |= TWMhold;
      b[0] = 2;
    }
    else TWcpld[1] &= ~TWMhold;
    TWcpldLoad(1,true);
    TWcpld[1] &= ~TWMorder;
    TWcpldLoad(1,false);
    return;
  }
  if(strcmp(Cmode,"Normal")==0) ClockReset = 8;
  else ClockReset = TD.Corder * 8;
  CrampCounter = 0;
}

void ConfigureTrig(void)
{
  CtrigInput->detach();
  CtrigInput->setPriority(TD.Ctrig, 8);
  CtrigInput->attached(TD.Ctrig, TD.CtrigLevel, CompressorTriggerISR);  
}

// Set the switch state based on user dialog box selection
void SetSwitch(void)
{
  if(strcmp(CswitchState,"Open")==0) SetOutput(TD.Cswitch,TD.CswitchLevel);
  else ClearOutput(TD.Cswitch,TD.CswitchLevel);
}

// Called when the timer reaches the desired time point set in the B
// compare register. This is used for gate or switch control
void SwitchTimerISR(void)
{
  switch (CSState)
  {
    case CSS_OPEN_REQUEST:
      // Open switch and set timer to close
      SetOutput(TD.Cswitch,TD.CswitchLevel);
      CSState = CSS_CLOSE;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
     break;
    case CSS_CLOSE_REQUEST:
      // Close switch and set timer to close
      ClearOutput(TD.Cswitch,TD.CswitchLevel);
      CSState = CSS_OPEN;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
      break;
    case CSS_OPEN:
      SetOutput(TD.Cswitch,TD.CswitchLevel);
      break;
    case CSS_CLOSE:
      ClearOutput(TD.Cswitch,TD.CswitchLevel);  
      break;
    default:
      break;
  }
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
      // If C_NormAmpMode is 1 or 2 then set the amplitude to TW1 level.
      if((C_NormAmpMode == 1) || (C_NormAmpMode == 2))
      {
         if(AcquireTWI()) TWCsetV2toV1();
         else TWIqueue(TWCsetV2toV1);
      }
      if(C_NormAmpMode == 2)
      {
         if(AcquireTWI()) TWCsetV();
         else TWIqueue(TWCsetV);          
      }
      // Next state is always normal
      ClockReset = 8;
      C_NextEvent += C_Tn;
      CrampCounter = 0;
      CState = CS_NORMAL;
      if(TDarray[0].Rev >= 4)
      {
         TWcpld[1] |= TWMmode;
         TWcpldLoad(1,true);
      }
      OP = 'X';
      break;
    case CS_NORMAL:
    case CS_NONCOMPRESS:
      CurrentPass++;
    case CS_TRIG:
    case CS_DELAY:
      // State will be Compress, NonCompress or 0 indicating finished, defined by table value
      OP = GetNextOperationFromTable(false);
      if(OP == 'C')
      {
        CState = CS_COMPRESS;
        C_NextEvent += C_Tc;
        CrampCounter = 0;
        ClockReset = TDarray[0].Corder * 8;
        if(TDarray[0].Rev >= 4)
        {
          TWcpld[1] &= ~TWMmode;
          TWcpldLoad(1,true);
        }
        // If C_NormAmpMode is 1 or 2 then set the amplitude to TW2 level.
        if((C_NormAmpMode == 1) || (C_NormAmpMode == 2))
        {
           if(AcquireTWI()) TWCsetv();
           else TWIqueue(TWCsetv);
        }
        if(C_NormAmpMode == 2)
        {
           if(AcquireTWI()) TWCsetV1toV2();
           else TWIqueue(TWCsetV1toV2);          
        }
      }
      else if(OP == 'N')
      {
        CState = CS_NONCOMPRESS;
        C_NextEvent += C_Tnc;
        ClockReset = 8;
        CrampCounter = 0;
        if(TDarray[0].Rev >= 4)
        {
           TWcpld[1] |= TWMmode;
           TWcpldLoad(1,true);
        }
      }
      else if(OP == 'D')
      {
        CState = CS_DELAY;  // Delay, hold current mode during delay
        C_NextEvent += C_Delay;
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
    if(TDarray[0].Rev >= 4) TWcpldLoad(0,false);
    UpdateMode();
    return;
  }
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
}

// Called when we are going to start a compression cycle.
// The A compare register is used for table timing.
// The B compare register is used for the switch or gate timing
void CompressorTriggerISR(void)
{
  uint32_t   start;
  
  AtomicBlock< Atomic_RestoreState > a_Block;  // uncommented, 8/22/17
  // Clear and setup variables
  ClockReset = 8;             // Put system in normal mode
  CrampCounter = 0;
  CurrentPass = 0;
  if(TDarray[0].Rev >= 4)
  {
    TWcpld[1] |= TWMmode;
  }
  GetNextOperationFromTable(true);
  C_NextEvent = C_Td;
  CState = CS_TRIG;
  // Setup the timer used to generate interrupts
  CompressorTimer.begin();
  CompressorTimer.setPriority(0);
  CompressorTimer.attachInterruptRA(CompressorTimerISR);
  CompressorTimer.attachInterruptRB(SwitchTimerISR);
  CompressorTimer.setTrigger(TC_CMR_EEVTEDG_NONE);
  CompressorTimer.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
  CompressorTimer.enableTrigger();
  CompressorTimer.softwareTrigger();
  CompressorTimer.setPriority(0);
} 

// Set twave channel 1 voltage to the value defined in the channel 1 data structure
void TWCsetV(void)
{
//   Wire.setClock(400000);
   int b=SelectedBoard();
   SelectBoard(0);
   SetPulseVoltage(0,TDarray[0].TWCD[DAC_PulseVoltage].VoltageSetpoint); 
   SelectBoard(b); 
//   Wire.setClock(100000);
}

// Set twave channel 2 voltage to the value defined in the channel 2 data structure
void TWCsetv(void)
{
//   Wire.setClock(400000);
   int b=SelectedBoard();
   SelectBoard(1);
   SetPulseVoltage(1,TDarray[1].TWCD[DAC_PulseVoltage].VoltageSetpoint); 
   SelectBoard(b); 
//   Wire.setClock(100000);
}

// Set the twave channel 2 voltage to the value defined in the channel 1 data structure
void TWCsetV2toV1(void)
{
//   Wire.setClock(400000);
   int b=SelectedBoard();
   SelectBoard(1);
   SetPulseVoltage(1,TDarray[0].TWCD[DAC_PulseVoltage].VoltageSetpoint); 
   SelectBoard(b); 
//   Wire.setClock(100000);
}

// Set the twave channel 1 voltage to the value defined in the channel 2 data structure
void TWCsetV1toV2(void)
{
//   Wire.setClock(400000);
   int b=SelectedBoard();
   SelectBoard(0);
   SetPulseVoltage(0,TDarray[1].TWCD[DAC_PulseVoltage].VoltageSetpoint); 
   SelectBoard(b); 
//   Wire.setClock(100000);
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
//
// June 19, 2016. Add the following commands:
//    'V' for TW1 voltage in units of volts, has to be whole numbers
//    'v' for TW2 voltage in units of volts, has to be whole numbers
//    'c' for compress time in milliseconds
//    'n' for normal time in a compress cycle
//    't' for non compress cycle time
// July 2, 2016. Added the 'F' for frequency command
//
// Need to add ability to stop all clocking for a defined time.
// Added repeat capability in the compressor table. Use this syntax ....[.....]9.... to define
// a loop. Logic in code goes like this:
//    1.) If we find a [ then push on to stack
//    2.) When we find a ] if there is an entry on the stack init it with the count and dec the count.
//        if its already inited then dec the count. If count is 0 then move on and clear entry from stack
//    3.) Allow a depth of five.
//
// Added the following commands:
//    'D' Delay in milliseconds
//    's' Stop the clock
//    'r' Restart the clock
//
// Added the following switch commands July 30, 2016
//    'o' open gate time in mS
//    'g' time to open gate relative to start of table in mS
//    'G' time to close gate relative to start of table in mS
//
// Added the following command, august 8, 2016
//    'M' set the mode of the compressor channel, channel 2
//        if 0 its normal mode and TW2 amplitude control sets the amplitude in compress and normal.
//        if 1 then the TW channel 1 amplitude is used in the normal mode on the TW2 compressor.
//    
char GetNextOperationFromTable(bool init)
{
  int         index,b;
  float       fval;
  static int  tblindex=0;
  static char OP;
  static int  count = 0;

  if(init)
  {
    CompressorStackInit();
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
        // If here then get the value, number can be a float but has to start with a number, 0.1 is ok, .1 is not ok
        count = int(TwaveCompressorTable[tblindex++] - '0');
        while(isDigit(TwaveCompressorTable[tblindex])) count = count * 10 + int(TwaveCompressorTable[tblindex++] - '0');
        // If its a decimal point then its a float so process
        fval = 0;
        if(TwaveCompressorTable[tblindex] == '.')
        {
          tblindex++;
          for(float d = 10; isDigit(TwaveCompressorTable[tblindex]); d *= 10) fval += (float)(TwaveCompressorTable[tblindex++] - '0') / d;
        }
        fval += count;
      }
      break;
    }
    if((OP=='N')||(OP=='C'))
    {
      // Only return valid options, N for non compressed and C for compressed
      count--;
      return(OP);
    }
    if(OP == 'D') //Delay
    {
      C_Delay = (fval / 1000.0) * C_clock;
      count = 0;
      return(OP);
    }
    if(OP == 'g') //Time to open gate
    {
      C_GateOpenTime = (fval / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_OPEN_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    if(OP == 'G') //Time to close gate
    {
      C_GateOpenTime = (fval / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_CLOSE_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    if(OP == 'S')
    {
      if(count == 0) ClearOutput(TD.Cswitch,TD.CswitchLevel);
      if(count == 1) SetOutput(TD.Cswitch,TD.CswitchLevel);
    }
    if(OP == 'O')
    {
      if((count >= 0) && (count <= 255))
      {
         TDarray[0].Corder = count;
         TD.Corder = count;
         UpdateMode();
      }
    }
    if(OP == 'V')
    {
      index = GetTwaveIndex(1);
      if (index != -1)
      {
        if((count > 7) && (count <= 100))
        {
           if (index == SelectedTwaveModule) TD.TWCD[0].VoltageSetpoint = fval;
           TDarray[index].TWCD[0].VoltageSetpoint = fval; 
           if(AcquireTWI()) TWCsetV();
           else TWIqueue(TWCsetV);
        }   
      }
    }
    if(OP == 'v')
    {
      index = GetTwaveIndex(2);
      if (index != -1)
      {
        if((count > 7) && (count <= 100))
        {
           if (index == SelectedTwaveModule) TD.TWCD[0].VoltageSetpoint = fval;
           TDarray[index].TWCD[0].VoltageSetpoint = fval;
           if(AcquireTWI()) TWCsetv();
           else TWIqueue(TWCsetv);
        }   
      }      
    }
    if(OP == 'F')
    {
      index = GetTwaveIndex(1);
      if (index != -1)
      {
        if((count > 1000) && (count <= 300000))
        {
           if (index == SelectedTwaveModule) TD.Velocity = count;
           TDarray[index].Velocity = count;
           SetVelocity(0);
        }   
      }      
    }
    if(OP == 'c')
    {
      TD.Tcompress = TDarray[0].Tcompress = fval;
      C_Tc  = (TDarray[0].Tcompress / 1000.0) * C_clock;
    }
    if(OP == 'n')
    {
      TD.Tnormal = TDarray[0].Tnormal = fval;
      C_Tn  = (TDarray[0].Tnormal / 1000.0) * C_clock;
    }
    if(OP == 't')
    {
      TD.TnoC = TDarray[0].TnoC = fval;
      C_Tnc = (TDarray[0].TnoC / 1000.0) * C_clock;
    }
    if(OP == 's')  // Stop the clock
    {
      C_ClockEnable = false;
    }
    if(OP == 'r')  // Restart the clock
    {
      C_ClockEnable = true;    
    }
    if(OP == 'o')  // Sets the switch open time
    {
      C_SwitchTime = (fval / 1000.0) * C_clock;
    }
    if(OP == 'M')  // Set compressor normal amplitude mode
    {
      C_NormAmpMode = count;
    }
    if(OP == 'K')  
    {
       TDarray[0].CompressRamp = count;
    }
    if(OP == 'k')  
    {
       TDarray[0].CrampOrder = count;
    }
    if(OP == 'Q') 
    {
      TDarray[0].Sequence = count;
      count = 0;
      SetSequence(0);
    }
    if(OP == 'q') 
    {
      TDarray[1].Sequence = count;
      count = 0;
      SetSequence(1);
    }
    if(OP == '[')
    {
       CompressorLoopStart(tblindex);
    }    
    if(OP == ']')
    {
      int i = CompressorProcessLoop(count);
      if(i != -1) tblindex = i;
    }    
  }
}

// This interrupt service routine generates the two clocks, one for each Twave channel (1 and 2).
// The hardware have a divide by 4 so each clock require 4 transistions.
void CompressorClockISR(void)
{
  uint32_t i;

  if(!C_ClockEnable) return;
  if(CState == CS_DELAY) return;
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
    else 
    {
      // This is the end of a compression cycle of the clock. Resetting the ClockIndex
      // Starts the next cycle. Compression order ramping logic is applied here.
      // ClockReset is order * 8 (8 clock cycles per one twave cycle), its = 8 for normal non-compress.
      // ClockReset = 8 indicates normal mode.
      ClockIndex = 0;
      // Apply compression ramp logic here. Adjust the ClockReset counter.
      if((TDarray[0].CompressRamp != 0) && (ClockReset != 8))
      {
         CrampCounter++;
         if(CrampCounter >= abs(TDarray[0].CompressRamp))
         {
           CrampCounter = 0;
           if(TDarray[0].CompressRamp < 0) ClockReset -= 8 * TDarray[0].CrampOrder;
           else ClockReset += 8 * TDarray[0].CrampOrder;
           if(ClockReset < 16) ClockReset = 16;
           if(ClockReset > 8 * 255) ClockReset = 8 * 255;
           CR = ClockReset;
         } 
      }
      else CrampCounter = 0;
    }
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
  // Setup the pio pointer
  pio = g_APinDescription[CLOCK_A].pPort;
  // Init the clock array used to generate the clocks
  // Rev 3 used a software clock, rev 4 is done in hardware
  if(TDarray[0].Rev == 3)
  {
    pinMode(CLOCK_A, OUTPUT);  // Clock A output pin
    pinMode(CLOCK_B, OUTPUT);  // Clock B output pin
    for(i=0;i < MaxOrder * 8;i++)
    {
      ClockArray[i] =  g_APinDescription[CLOCK_A].ulPin;
      if(i < 8) ClockArray[i] |= g_APinDescription[CLOCK_B].ulPin;
    }
    // Setup the timer and the ISR for the clock
    TwaveClk->attachInterrupt(CompressorClockISR);
    TwaveClk->setFrequency((double)TDarray[0].Velocity);
    TwaveClk->start(-1, 0, false);
    //NVIC_SetPriority(TC7_IRQn, 0);
  }
  // Clear the switch output
  ClearOutput(TDarray[0].Cswitch,TDarray[0].CswitchLevel);
  // Enable the compressor menu selection 
  de = GetDialogEntries(TwaveDialogEntries2page2, "Compressor");
  if(de != NULL) de->Type = D_DIALOG;
  // If rev 2 or 3 enable Cramp selection
  if((TDarray[0].Rev == 2) || (TDarray[0].Rev == 3))
  {
     de = GetDialogEntries(CompressorEntries2, "Cramp");
     if(de != NULL) 
     {
        de[0].Type = D_INT;
        de[1].Type = D_INT;
     }
  }  
  // Setup the trigger line and ISR
  CtrigInput = new DIhandler;
  UpdateMode();
  SetSwitch();
}

// This is the main loop function for the compressor mode. This function
// will exit if the compressor flag is not set.
void CompressorLoop(void)
{
  static int init = -1;

  if(init == -1)
  {
    CtrigInput->detach();
    CtrigInput->setPriority(TDarray[0].Ctrig, 0);
    CtrigInput->attached(TDarray[0].Ctrig, TDarray[0].CtrigLevel, CompressorTriggerISR);
    init = 0;
  }
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
   SendACK;
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

// This function saves the TWAVE module data to EEPROM. All detected TWAVE modules are saved.
void SaveTWAVE2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<2; b++)
  {
    if(TwaveBoards[b])
    {
      SelectBoard(b);
      if (WriteEEPROM(&TDarray[b], TwaveEEPROMaddr[b], 0, sizeof(TwaveData)) != 0) berr = true;
    }
  }
  SelectBoard(brd);
  if(berr)
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;
}






