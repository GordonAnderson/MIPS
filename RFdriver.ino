#if RFdriver2 == false
#include "Variants.h"
//
// RFdriver
//
// This file supports the RF driver board on the MIPS system. Two RF driver boards can be installed
// in one MIPS system. Each RF driver can drive 2 high Q heads so a single MIPS system can drive
// 4 high Q heads.
//
// Each drive channel has independent RF frequency and drive level controls. This driver monitors
// the RF hrad voltage and current and calcualtes total power. Limits can be programmed by the user
// tyo limit power and also limit drive level.
//
// Auto tune and re-tune features are present to help setup and adjustments as system warms up.
//
// The RF heads are driven with a square wave and this RF drive controls the supply voltage to the driving
// FET(s) in the RF head. This adjusts the maximum RF voltage. The RF driver supports two modes of operation.
// open loop and voltage controlled output level.
//
// The RF driver rev level sets the behavior of the RF voltage level readbacks:
//  Rev 0 and Rev 1 = Linear readout using the data structures cal mx+b parameters.
//  Rev 2 = Used for the eiceman project and this rev will only display the RF+ output. this
//          readout is corrected for the acvite level sensors. This rev also changes the control loop
//          to respond faster.
//  Rev 3 = Convert to engineering units for the Linear tech level sensors, 5th order correction.
//  Rev 4 = Used for the RFcoilDriver board. This board has only one channel and used a AD7994 ADC.
//          Used for the eiceman project. Suports only one board.
//  Rev 5 = Used piecewise linear lookup table calibration
//  Rev 6 = Same as rev 4 except displays both RF+ and RF- channels. This is used for the fragmentor
//          with phase and anti-phase outputs.

// PLL is not stable below 250,000Hz
#define MinFreq    400000
#define MaxFreq    5100000

extern bool NormalStartup;

#define  PWMFS  4095          // Full scale PWM output value
// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.05         // Strong filter coefficent

//MIPS Threads
Thread RFdriverThread  = Thread();

#define RFDD RFDDarray[SelectedRFBoard]

RFdriverData  RFDDarray[2] = {RFDD_A_Rev_1, RFDD_B_Rev_1};

#ifdef TestMode
int  NumberOfRFChannels = 2; // Defines the number of RF channels supported. Set during intaliztion.
// valid values are 0, 2, or 4
bool RFdriverBoards[2] = {true, false}; // Defines the boards that are present in the system
#else
int  NumberOfRFChannels = 0;
bool RFdriverBoards[2] = {false, false};
#endif
int  Channel = 1;            // User selected channel
int  SelectedRFChan = 0;     // Active channel
int  SelectedRFBoard = 0;    // Active board, 0 or 1 = A or B
float MaxRFVoltage = 0;      // This value is set to the highest RF output voltage
RFchannelData RFCD;          // Holds the selected channel's data
ADCcontrolRF *adcRFcontrol[2] = {NULL,NULL};

// These arrays hold the monitor values, array indexes are board,channel. These are scanned
// in the loop and filtered. These values are both displayed and also used for control
// and limit testing.
float RFpVpps[2][2];
float RFnVpps[2][2];
float Powers[2][2];
// These are the actively displayed values
float RFpVpp;
float RFnVpp;
float Power;

// Auto tune parameters
bool TuneRequest   = false;
bool RetuneRequest = false;
bool Tuning        = false;
bool TuneReport    = false;
bool TuneAbort     = false;
int  TuneRFChan;
int  TuneRFBoard;
// Tune states
#define TUNE_SCAN_DOWN 1
#define TUNE_SCAN_UP 2

#define MaxNumDown 5
#define MAXSTEP    100000

// Arc detection parameters
int   RFarcCH = 0;
float RFarcDrv = 0;
float RFarcV = 0;

bool  RFgatingOff = false;
DIhandler *DIh[2][2];
void (*GateTriggerISRs[2][2])(void) = {RF_A1_ISR, RF_A2_ISR, RF_B1_ISR, RF_B2_ISR};
float gatedDrive[4] = {0,0,0,0};

DialogBoxEntry RFdriverDialogEntriesPage1[] = {
  {" RF channel", 0, 1, D_INT, 1, 2, 1, 21, false, "%2d", &Channel, NULL, SelectChannel},
  {" Freq, Hz"  , 0, 2, D_INT, MinFreq, MaxFreq, 1000, 16, false, "%7d", &RFCD.Freq, NULL, NULL},
  {" Drive %"   , 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &RFCD.DriveLevel, NULL, NULL},
  {" Vout Vpp"  , 0, 3, D_OFF, 0, 5000, 1, 18, false, "%5.0f", &RFCD.Setpoint, NULL, NULL},
  {" RF+ Vpp"   , 0, 5, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFpVpp, NULL, NULL},
  {" RF- Vpp"   , 0, 6, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFnVpp, NULL, NULL},
  {" Power, W"  , 0, 7, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.1f", &Power, NULL, NULL},
  {" Next page" , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextRFPage, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char *ModeList = "MANUAL,AUTO";
char RFmode[8] = "MANUAL";
char RFgateDI[5] = "NA";
char RFgateTrig[5] = "NA";

DialogBoxEntry RFdriverDialogEntriesPage2[] = {
  {" Drive limit, %", 0, 1, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &RFCD.MaxDrive, NULL, NULL},
  {" Power limit, W", 0, 2, D_FLOAT, 0, 50, 1, 18, false, "%5.1f", &RFCD.MaxPower, NULL, NULL},
  {" Mode"          , 0, 3, D_LIST, 0, 0, 7, 16, false, ModeList, RFmode, NULL, RFmodeChange},
  {" Gate input"    , 0, 4, D_LIST, 0, 0, 5, 18, false, DIlist, RFgateDI, NULL, RFgateChange},
  {" Gate level"    , 0, 5, D_LIST, 0, 0, 5, 18, false, DILlist, RFgateTrig, NULL, RFgateChange},
  {" Auto tune"     , 0, 6, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFat, NULL},
  {" Auto retune"   , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFart, NULL},
  {" Save settings" , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFdriverSettings, NULL},
  {" Restore settings", 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFdriverSettings, NULL},
  {" First page"         , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetFirstRFPage, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

// change orgin to 3,7
DialogBox RFdriverDialog = {
  {"RF driver parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, RFdriverDialogEntriesPage1
};

MenuEntry MERFdriverMonitor = {" RF driver module", M_DIALOG, 0, 0, 0, NULL, &RFdriverDialog, NULL, NULL};

// Called from the UI to enable auto tune, only works in manual mode
void RFat(void)
{
  if(RFCD.RFmode != RF_MANUAL) return;
  TuneRFChan = SelectedRFChan;
  TuneRequest = true;  
}

// Called from the UI to enable auto retune, only works in manual mode
void RFart(void)
{
  if(RFCD.RFmode != RF_MANUAL) return;
  TuneRFChan = SelectedRFChan;
  RetuneRequest = true;  
}

void RFmodeChange(void)
{
  String mode;

  mode = RFmode;
  if (mode == "MANUAL")
  {
    RFCD.RFmode = RF_MANUAL;
    RFdriverDialogEntriesPage1[2].Type = D_FLOAT;
    RFdriverDialogEntriesPage1[3].Type = D_OFF;
  }
  else if (mode == "AUTO")
  {
    // If RFmode just changed to auto then set the voltage setpoint to the current value
    if(RFCD.RFmode != RF_AUTO)
    {
        RFCD.Setpoint = (RFpVpps[SelectedRFBoard][SelectedRFChan & 1] + RFnVpps[SelectedRFBoard][SelectedRFChan & 1]) / 2;
    }
    RFCD.RFmode = RF_AUTO;
    RFdriverDialogEntriesPage1[2].Type = D_OFF;
    RFdriverDialogEntriesPage1[3].Type = D_FLOAT;
  }
  if(ActiveDialog == &RFdriverDialog) ActiveDialog->Changed = true;
}

void RFgateChange(void)
{
  String trig;

  RFDD.RFgateDI[SelectedRFChan & 1] = RFgateDI[0];
  if (RFDD.RFgateDI[SelectedRFChan & 1] == 'N') RFDD.RFgateDI[SelectedRFChan & 1] = 0;
  trig = RFgateTrig;
  if (trig == "NA") RFDD.RFgateTrig[SelectedRFChan & 1] = -1;
  else if (trig == "BOTH") RFDD.RFgateTrig[SelectedRFChan & 1] = CHANGE;
  else if (trig == "HIGH") RFDD.RFgateTrig[SelectedRFChan & 1] = HIGH;
  else if (trig == "LOW") RFDD.RFgateTrig[SelectedRFChan & 1] = LOW;
  else if (trig == "POS") RFDD.RFgateTrig[SelectedRFChan & 1] = RISING;
  else if (trig == "NEG") RFDD.RFgateTrig[SelectedRFChan & 1] = FALLING;
}

void SetNextRFPage(void)
{
  RFdriverDialog.Entry = RFdriverDialogEntriesPage2;
  RFdriverDialog.Selected = 0;
  RFdriverDialog.State = M_SCROLLING;
  DialogBoxDisplay(&RFdriverDialog);
}

void SetFirstRFPage(void)
{
  RFdriverDialog.Entry = RFdriverDialogEntriesPage1;
  RFdriverDialog.Selected = 0;
  RFdriverDialog.State = M_SCROLLING;
  DialogBoxDisplay(&RFdriverDialog);
}

// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveRFdriverSettings(void)
{
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  RFDD.Size = sizeof(RFdriverData);
  if (WriteEEPROM(&RFDD, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

// Restore the parameters from the EEPROM on the RFdriver board. Read the EEPROM and make sure the board name
// matches what is expected, only load if its correct.
void RestoreRFdriverSettings(void)
{
  RestoreRFdriverSettings(false);
}

void RestoreRFdriverSettings(bool NoDisplay)
{
  RFdriverData rfdd;

  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  if (ReadEEPROM(&rfdd, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    if (strcmp(rfdd.Name, RFDD.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      //if(rfdd.Size != sizeof(RFdriverData)) rfdd.Size = sizeof(RFdriverData);
      rfdd.EEPROMadr = RFDD.EEPROMadr;
      memcpy(&RFDD, &rfdd, rfdd.Size);
      RFDD.Size = sizeof(RFdriverData);
      RFCD = RFDD.RFCD[SelectedRFChan & 1];
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      SelectChannel();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected
int8_t BoardFromSelectedChannel(int8_t SC)
{
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if (SC <= 1)
  {
    if (RFdriverBoards[0]) return (0);
    if (RFdriverBoards[1]) return (1);
    return (-1);
  }
  //  if selected channel is 2 or 3 then use board 1 and make sure board 0 is present
  if ((SC == 2) || (SC == 3))
  {
    if ((RFdriverBoards[0]) && (RFdriverBoards[1])) return (1);
  }
  return (-1);
}

// Called after the user selects a channel
void SelectChannel(void)
{
  SelectedRFChan = Channel - 1;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  RFCD = RFDD.RFCD[(Channel - 1) & 1];
  SelectBoard(SelectedRFBoard);
//  Powers[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = RFnVpps[SelectedRFBoard][SelectedRFChan] = 0;
  RFgateDI[0] = RFDD.RFgateDI[SelectedRFChan & 1];
  RFgateDI[1] = 0;
  if (RFgateDI[0] == 0) strcpy(RFgateDI, "NA");
  if (RFDD.RFgateTrig[SelectedRFChan & 1] == -1) strcpy(RFgateTrig, "NA");
  else if (RFDD.RFgateTrig[SelectedRFChan & 1] == CHANGE) strcpy(RFgateTrig, "BOTH");
  else if (RFDD.RFgateTrig[SelectedRFChan & 1] == HIGH) strcpy(RFgateTrig, "HIGH");
  else if (RFDD.RFgateTrig[SelectedRFChan & 1] == LOW) strcpy(RFgateTrig, "LOW");
  else if (RFDD.RFgateTrig[SelectedRFChan & 1] == RISING) strcpy(RFgateTrig, "POS");
  else if (RFDD.RFgateTrig[SelectedRFChan & 1] == FALLING) strcpy(RFgateTrig, "NEG");
  if (RFCD.RFmode == RF_MANUAL) strcpy(RFmode, "MANUAL");
  else strcpy(RFmode, "AUTO");
  RFmodeChange();
  RFgateChange();
  // Set the power limit
  RFdriverDialogEntriesPage2[1].Max = RFDD.PowerLimit;
  // If rev 2 or 4 then do not display the RF- channel
  if((RFDD.Rev == 2) || (RFDD.Rev == 4))
  {
    RFdriverDialogEntriesPage1[5].Type = D_OFF;
  }
  else
  {
    RFdriverDialogEntriesPage1[5].Type = D_FLOAT;    
  }
  // Update the display
  if (ActiveDialog == &RFdriverDialog) DialogBoxDisplay(&RFdriverDialog);
}

// The following two functions support setting the RF drive or level fron the pulse
// sequence generator. UpdateRFdrive saves the requesed setting and ProcessRFdrive 
// applies the requested value.
float RFqueuedValues[4] = {-1,-1,-1,-1};
void UpdateRFdrive(int chan, float value)
{
  if((chan >=0) && (chan <= 3)) RFqueuedValues[chan] = value;
}
void ProcessRFdrive(void)
{
  for(int chan=0; chan<4; chan++)
  {
     if(RFqueuedValues[chan] < 0) continue;
     int b = BoardFromSelectedChannel(chan);
     if(RFDDarray[b].RFCD[chan & 1].RFmode == RF_AUTO) RFDDarray[b].RFCD[chan & 1].Setpoint = RFqueuedValues[chan];
     {
        if(RFqueuedValues[chan] > RFDDarray[b].RFCD[chan & 1].MaxDrive) RFqueuedValues[chan] = RFDDarray[b].RFCD[chan & 1].MaxDrive;
        RFDDarray[b].RFCD[chan & 1].DriveLevel = RFqueuedValues[chan];
        analogWrite(RFDDarray[b].RFCD[chan & 1].PWMchan, (RFDDarray[b].RFCD[chan & 1].DriveLevel * PWMFS) / 100);
     }
     RFqueuedValues[chan] = -1;
  }
}


// This function is called at powerup to initiaize the RF driver.
// The board parameter (0 or 1) defines the board select parameter where this card was found.
// Up to two boards are supported. Board A, or 0, is always called first.
// If only one board is installed it can be board 0 or 1.
void RFdriver_init(int8_t Board, int8_t addr)
{
  DialogBox *sd;
  
  // Flag the board as present
  RFdriverBoards[Board] = true;
  // Create the digital input gate objects
  DIh[Board][0] = new DIhandler;
  DIh[Board][1] = new DIhandler;
  // Set active board to board being inited
  SelectedRFBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  sd = ActiveDialog;
  ActiveDialog = NULL;
  if(NumberOfRFChannels > 0)
  {
    Channel = 3;
    SelectedRFChan = Channel - 1;
  }
  RFDDarray[Board].EEPROMadr = addr;
  if (NormalStartup)
  {
    RestoreRFdriverSettings(true);
  }
  // Init the clock generator and set the frequencies
  SetRef(20000000);
  CY_Init(RFDD.CLOCKadr,Board);
  SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[0].Freq,Board);
  SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[1].Freq,Board);
  // Setup the PWM outputs and set levels
  analogWriteResolution(12);
  SelectedRFBoard = Board;
  pinMode(RFDD.RFCD[0].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[0].PWMchan, 0);
  pinMode(RFDD.RFCD[1].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[1].PWMchan, 0);
  // Define the initial selected channel as 0 and setup
  Channel = 1;
  SelectChannel();
  ActiveDialog = sd;
  // Setup the menu if this is the very first call to this init function
  if (NumberOfRFChannels == 0)
  {
    AddMainMenuEntry(&MERFdriverMonitor);
    if (ActiveDialog == NULL) DialogBoxDisplay(&RFdriverDialog);
    // Configure Threads
    RFdriverThread.setName("RFdriver");
    RFdriverThread.onRun(RFdriver_loop);
    RFdriverThread.setInterval(100);
    // Add threads to the controller
    control.add(&RFdriverThread);
  }
  if((RFDDarray[Board].Rev == 4) || (RFDDarray[Board].Rev == 6))
  {
    RFDD.RFCD[0].RFpADCchan.Chan = 2;
    RFDD.RFCD[0].RFnADCchan.Chan = 3;
    NumberOfRFChannels++;
  }
  else NumberOfRFChannels += 2;  // Always add two channels for each board
  // Set the maximum number of channels in the selection menu
  RFdriverDialogEntriesPage1[0].Max = NumberOfRFChannels;
}

// This function checks the power limits and performs control functions
void RFcontrol(void)
{
  int board, chan;
  float es,g;

  // Check the power and reduce the drive level if power is over its limit
  for (board = 0; board < 2; board++)
  {
    for (chan = 0; chan < 2; chan++)
    {
      if (Powers[board][chan] > RFDDarray[board].RFCD[chan].MaxPower) RFDDarray[board].RFCD[chan].DriveLevel -= 0.1;
      if (RFDDarray[board].RFCD[chan].DriveLevel < 0) RFDDarray[board].RFCD[chan].DriveLevel = 0;
      // Only process if  this channel is not gated off
      if(!DIh[board][chan]->test(RFDDarray[board].RFgateTrig[chan])) break;
      if ((SelectedRFBoard == board) && ((SelectedRFChan & 1) == chan))
      {
        RFCD.DriveLevel = RFDDarray[board].RFCD[chan].DriveLevel;
      }
      // If we are in auto mode (closed loop control) then adjust drive as needed to maintain Vpp
      if (RFDDarray[board].RFCD[chan].RFmode == RF_AUTO)
      {
        if(((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2) > RFDDarray[board].RFCD[chan].Setpoint) es = ((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2) / RFDDarray[board].RFCD[chan].Setpoint;
        if(((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2) < RFDDarray[board].RFCD[chan].Setpoint) es = RFDDarray[board].RFCD[chan].Setpoint / ((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2);
        es = abs(RFDDarray[board].RFCD[chan].Setpoint - ((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2));
        g = es * 0.05;
        if (((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2) > RFDDarray[board].RFCD[chan].Setpoint) RFDDarray[board].RFCD[chan].DriveLevel -= 0.08 * g;
        else RFDDarray[board].RFCD[chan].DriveLevel += 0.08 * g;
        RFDDarray[board].RFCD[chan].DriveLevel = (float)((int)(RFDDarray[board].RFCD[chan].DriveLevel * 100)) / 100.0;        
        if (RFDDarray[board].RFCD[chan].DriveLevel < 0) RFDDarray[board].RFCD[chan].DriveLevel = 0;
        if (RFDDarray[board].RFCD[chan].DriveLevel > RFDDarray[board].RFCD[chan].MaxDrive) RFDDarray[board].RFCD[chan].DriveLevel = RFDDarray[board].RFCD[chan].MaxDrive;
        if ((SelectedRFBoard == board) && ((SelectedRFChan & 1) == chan))
        {
          RFCD.DriveLevel = RFDDarray[board].RFCD[chan].DriveLevel;
        }
      }
    }
  }
}

// Auto tune algorithm, procedure is as follows:
// 1.) Set power tp 10%
// 2.) Set frequency to 1MHz
// 3.) Go down in frequency in 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 4.) Go up in frequency from 1MHzin 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 5.) Use the peak found in steps 3 and 4 and sweep in 10K steps using the same procedure in 3 and 4
// 6.) Use the peak found in step 5 and sweep in 1K steps using the same procedure in 3 and 4
// 7.) Done!
//
// Called from the main processing loop, this function does not block, uses a state machine to perform thge logic
// States
//  TUNE_SCAN_DOWN
//  TUNE_SCAN_UP

void RFdriver_tune(void)
{
   static int    TuneStep = 100000, TuneState;
   static float  Max, Current, Last;
   static int    FreqMax, Freq;
   static int    NumDown,Nth;

   if(TuneRequest)
   {
     TuneAbort = false;
     ButtonPressed = false;
     TuneRFBoard = BoardFromSelectedChannel(TuneRFChan); 
     // Set freq to 1MHz
     RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq = 1000000;
     // Set drive to 10%
     RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].DriveLevel = 10;
     Tuning = true;
     TuneStep = MAXSTEP;
     Freq = 1000000;
     Last = Max = 0;
     NumDown = -MaxNumDown;
     TuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     DisplayMessage("Tune in process");
     return;
   }
   if(RetuneRequest)
   {
     TuneAbort = false;
     ButtonPressed = false;
     TuneRFBoard = BoardFromSelectedChannel(TuneRFChan); 
     // Set freq to current
     Freq = RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq;
     Tuning = true;
     TuneStep = 1000;
     Last = Max = 0;
     NumDown = 0;
     RetuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     DisplayMessage("Retune in process");
     return;
   }
   if(!Tuning) return;
   // Here if the system is tuning
   if((TuneAbort) || (ButtonPressed))
   {
      TuneAbort = false;
      ButtonPressed = false;
      Tuning = false;
      DismissMessage();
      if(TuneReport && !SerialMute)
      {
        serial->print("Auto tune aborted, channel = ");
        serial->println(TuneRFChan + 1);
      }
      TuneReport = false;
      return;    
   }
   if(--Nth > 0) return;
   Nth = 20;
   Current = RFpVpps[TuneRFBoard][TuneRFChan & 1] + RFnVpps[TuneRFBoard][TuneRFChan & 1];
   TuneRFBoard = BoardFromSelectedChannel(TuneRFChan);
   switch (TuneState)
   {
     case TUNE_SCAN_DOWN:
//        if(Current > Max)
//        {
//          Max = Current;
//          FreqMax = RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq;
//        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq -= TuneStep;
        if((NumDown >= MaxNumDown) || (RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq < MinFreq))
        {
          TuneState = TUNE_SCAN_UP;
          RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq = Freq;
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if(Current > Max)  // Move this code here to force tune system to not use the limit value
        {
          Max = Current;
          FreqMax = RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq;
        }
        if(Current <= (Last + 1)) NumDown++;
        break;
     case TUNE_SCAN_UP:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq;
        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if((Current == 0) && (Last == 0)) NumDown--;
        RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq += TuneStep;
        if((NumDown >= MaxNumDown) || (RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq > MaxFreq))
        {
          // Here we have found the peak for this step size, this
          // process repeats until step size is 1KHz
          Freq = FreqMax;
          if(Freq < MinFreq) Freq = MinFreq;
          if(Freq > MaxFreq) Freq = MaxFreq;
          RFDDarray[TuneRFBoard].RFCD[TuneRFChan & 1].Freq = Freq;
          if(TuneStep == 1000)
          {
            // If here we are done!
            Tuning = false;
            DismissMessage();
            if(TuneReport && !SerialMute)
            {
              serial->print("Auto tune complete, channel = ");
              serial->print(TuneRFChan + 1);
              serial->print(", frequency = ");
              serial->println(Freq);
            }
            TuneReport = false;
            return;
          }
          else 
          {
            if(TuneStep == MAXSTEP) TuneStep = 10000;
            else TuneStep /= 10;
          }
          TuneState = TUNE_SCAN_DOWN;
          NumDown = 0;
        }
        break;
     default:
        break;
   }
   Last = Current;
}

float RFdriverCounts2Volts(int Rev, int ADCcounts, ADCchan *adcchan)
{
   float Pv;
   int   brd;
   
   if(Rev == 3)
   {
      // Convert to engineering units for the Linear tech level sensors, 5th order correction.
      // y = -62.27195 + 2.06452*x - 0.025363*x^2 + 0.000161919*x^3 - 3.873611e-7*x^4 + 3.255975e-10*x^5
      Pv = Counts2Value(ADCcounts, adcchan);
      Pv = 3.255975e-10 * pow(Pv,5) - 3.873611e-7 * pow(Pv,4) + 0.000161919 * pow(Pv,3) - 0.025363 * pow(Pv,2) + 2.06452 * Pv - 62.27195;
   }
   else if((Rev == 2) || (Rev == 4) || (Rev == 6))
   {
     // This rev supports linear operation for high voltage, up to 4000Vp-p, this is equations is for the RF level detector circuit.
     // Rev 2 also displays only the RF+ output. Used on the Eiceman project
     Pv = Counts2Value(ADCcounts, adcchan);
     // This code attempts to correct for nonlinear performance near 0
     float Zc = (float)Counts2Value(0, adcchan) / 0.8;
     if(Pv <= Zc) Pv = Pv - (Zc - Pv) * 4.0;   
   }
   else if(Rev <= 1)
   {
     // This is a linear calibration using the data structure parameters
     Pv = Counts2Value(ADCcounts, adcchan);
   }  
   else if(Rev == 5)
   {
     // Find the RF channel number and phase, then return the lookup value
     for(int i=0; i<NumberOfRFChannels; i++)
     {
       brd = BoardFromSelectedChannel(i);
       if(&RFDDarray[brd].RFCD[i & 1].RFpADCchan == adcchan) return PWLlookup(i+1,0,ADCcounts);
       if(&RFDDarray[brd].RFCD[i & 1].RFnADCchan == adcchan) return PWLlookup(i+1,1,ADCcounts);
     }
   }
   return Pv; 
}

// This function is called by the main loop every 100 millisec
void RFdriver_loop(void)
{
  int i,j,iStat;
  uint16_t ADCvals[8];
  float V, I, Pv, Nv;
  static int LastFreq[2][2] = { -1, -1, -1, -1};
  static  int disIndex = 0;

  MaxRFVoltage = 0;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  if(ActiveDialog == &RFdriverDialog)
  {
    if(ActiveDialog->Changed) 
    {
      // This stores any changes back to the selected channels data structure
      RFDD.RFCD[SelectedRFChan & 1] = RFCD;    
      // Set the Drive level limit in the UI menu
      RFdriverDialogEntriesPage1[2].Max = RFCD.MaxDrive;
      ActiveDialog->Changed = false;
    }
  }
  RFdriver_tune();
  // Process ADC control settings
  for(i=0;i<2;i++)
  {
    if(adcRFcontrol[i] != NULL)
    {
      if(adcRFcontrol[i]->enable)
      {
        int b = BoardFromSelectedChannel(i);
        if(RFDDarray[b].RFCD[i].RFmode == RF_MANUAL)
        {
          RFDDarray[b].RFCD[i].DriveLevel = 10.0 * adcRFcontrol[i]->gain * analogRead(i);
          if(RFDDarray[b].RFCD[i].DriveLevel > RFDDarray[b].RFCD[i].MaxDrive) RFDDarray[b].RFCD[i].DriveLevel = RFDDarray[b].RFCD[i].MaxDrive;
        }
        else
        {
          RFDDarray[b].RFCD[i].Setpoint = 100.0 * adcRFcontrol[i]->gain * analogRead(i);
        }
      }
    }
  }
  // Update the clock generator and set the frequencies for
  // all boards that are present in system. Only update if the freq
  // has actually changed.
  // This logic lowers the drive level to 0 then waits a couple milli seconds
  // before changing the frequency and then resets the drive level. This prevents the noise from
  // causing the PLL to lock up.

  for (i = 0; i < NumberOfRFChannels; i++)
  {
    SelectedRFBoard = BoardFromSelectedChannel(i);
    SelectBoard(SelectedRFBoard);
    if (LastFreq[SelectedRFBoard][i & 1] != RFDD.RFCD[i & 1].Freq)
    {
      // Only apply the new frequency if we are not gated off or bypassed
      if ((DIh[SelectedRFBoard][i & 1]->test(RFDDarray[SelectedRFBoard].RFgateTrig[i & 1])) || RFgatingOff)
      {
        // Lower drive level to 0 then delay
        analogWrite(RFDD.RFCD[i & 1].PWMchan, 0);
        //delay(2);
        if((i & 1) == 0)
        {
          //AtomicBlock< Atomic_RestoreState > a_Block;
          AcquireTWI();
          for (j = 0; j < 5; j++) if (SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[i & 1].Freq, SelectedRFBoard) == 0) break;      
          ReleaseTWI();  
        }
        else
        {
          //AtomicBlock< Atomic_RestoreState > a_Block;
          AcquireTWI();
          for (j = 0; j < 5; j++) if (SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[i & 1].Freq, SelectedRFBoard) == 0) break;   
          ReleaseTWI();               
        }
        LastFreq[SelectedRFBoard][i & 1] = RFDD.RFCD[i & 1].Freq;
        // Reset drive level
        analogWrite(RFDD.RFCD[i & 1].PWMchan, (RFDD.RFCD[i & 1].DriveLevel * PWMFS) / 100);
      }
    }    
  }

  // Update the PWM outputs and set levels
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    SelectedRFBoard = BoardFromSelectedChannel(i);
    if ((DIh[SelectedRFBoard][i & 1]->test(RFDDarray[SelectedRFBoard].RFgateTrig[i & 1])) || RFgatingOff)
    {
      analogWrite(RFDD.RFCD[i & 1].PWMchan, (RFDD.RFCD[i & 1].DriveLevel * PWMFS) / 100);
    }
    else analogWrite(RFDD.RFCD[i & 1].PWMchan, (gatedDrive[i] * PWMFS) / 100);
  }
  // Check and update the RF gate controls
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    SelectedRFBoard = BoardFromSelectedChannel(i);
    if ((RFDD.RFgateDI[i & 1] != DIh[SelectedRFBoard][i & 1]->di) || (RFDD.RFgateTrig[i & 1] != DIh[SelectedRFBoard][i & 1]->mode))
    {
      DIh[SelectedRFBoard][i & 1]->detach();
      DIh[SelectedRFBoard][i & 1]->attached(RFDD.RFgateDI[i & 1], CHANGE, GateTriggerISRs[SelectedRFBoard][i & 1]);
      // Call the ISR to process the change
      GateTriggerISRs[SelectedRFBoard][i & 1]();
    }
  }
  // Read all the voltage monitors and caculate all the RF head power, 7mS
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    // Select board and read parameters
    SelectedRFBoard = BoardFromSelectedChannel(i);
    SelectBoard(SelectedRFBoard);
    // Read the ADC monitor values for the selected channels.
    ValueChange = false;
    //delay(1);
    if ((i == 0) || (i == 2))   // Only read the board's ADC when first indexed
    {
      if((RFDD.Rev == 4) || (RFDD.Rev == 6)) iStat = AD7994(RFDD.ADCadr, ADCvals);
      else iStat = AD7998(RFDD.ADCadr, ADCvals);
      //serial->println(ADCvals[7]);
      if((iStat != 0) || (ValueChange))
      {
        i++;
        continue;
      }
    }
    if(iStat != 0) continue;
    if ((DIh[SelectedRFBoard][i & 1]->test(RFDDarray[SelectedRFBoard].RFgateTrig[i & 1])) || RFgatingOff)
    {
      if(RFDD.Rev == 3)
      {
         // Convert to engineering units for the Linear tech level sensors, 5th order correction.
         // y = -62.27195 + 2.06452*x - 0.025363*x^2 + 0.000161919*x^3 - 3.873611e-7*x^4 + 3.255975e-10*x^5
         Pv = Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFpADCchan.Chan], &RFDD.RFCD[i & 1].RFpADCchan);
         Pv = 3.255975e-10 * pow(Pv,5) - 3.873611e-7 * pow(Pv,4) + 0.000161919 * pow(Pv,3) - 0.025363 * pow(Pv,2) + 2.06452 * Pv - 62.27195;
         Nv = Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFnADCchan.Chan], &RFDD.RFCD[i & 1].RFnADCchan);
         Nv = 3.255975e-10 * pow(Nv,5) - 3.873611e-7 * pow(Nv,4) + 0.000161919 * pow(Nv,3) - 0.025363 * pow(Nv,2) + 2.06452 * Nv - 62.27195;
      }
      else if((RFDD.Rev == 2) || (RFDD.Rev == 4) || (RFDD.Rev == 6))
      {
        // This rev supports linear operation for high voltage, up to 4000Vp-p, this equation is for the RF level detector circuit.
        // Rev 2 also displays only the RF+ output. Used on the Eiceman project
        Pv = (float)Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFpADCchan.Chan], &RFDD.RFCD[i & 1].RFpADCchan);
        Nv = (float)Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFnADCchan.Chan], &RFDD.RFCD[i & 1].RFnADCchan);
        // This code attempts to correct for nonlinear performance near 0
        float Zc = (float)Counts2Value(0, &RFDD.RFCD[i & 1].RFpADCchan) / 0.8;
        if(Pv <= Zc) Pv = Pv - (Zc - Pv) * 4.0;   
        Zc = (float)Counts2Value(0, &RFDD.RFCD[i & 1].RFnADCchan) / 0.8;
        if(Nv <= Zc) Nv = Nv - (Zc - Nv) * 4.0;   
        if(RFDD.Rev != 6) Nv = Pv;  // Make them match for the control loop
      }
      else if(RFDD.Rev <= 1)
      {
        // This is a linear calibration using the data structure parameters
        Pv = Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFpADCchan.Chan], &RFDD.RFCD[i & 1].RFpADCchan);
        Nv = Counts2Value(ADCvals[RFDD.RFCD[i & 1].RFnADCchan.Chan], &RFDD.RFCD[i & 1].RFnADCchan);
      }
      else if(RFDD.Rev == 5)
      {
        Pv = PWLlookup(i+1,0,ADCvals[RFDD.RFCD[i & 1].RFpADCchan.Chan]);
        Nv = PWLlookup(i+1,1,ADCvals[RFDD.RFCD[i & 1].RFnADCchan.Chan]);
      }
      // Filter with 1st order difference equation
      RFpVpps[SelectedRFBoard][i & 1] = Filter * Pv + (1 - Filter) * RFpVpps[SelectedRFBoard][i & 1];
      RFnVpps[SelectedRFBoard][i & 1] = Filter * Nv + (1 - Filter) * RFnVpps[SelectedRFBoard][i & 1];
      // Arc detection logic. If enabled the average filtered voltage is compared to the unfiltered
      // value. If a sudden change is detected then an error is flagged and the drive reduced to zero.
      // This only applies to the selected RF channel. 
      if(RFarcCH == i+1)
      {
         Pv = (Pv + Nv) / 2.0;   // Average unfiltered
         Nv = (RFpVpps[SelectedRFBoard][i & 1] + RFnVpps[SelectedRFBoard][i & 1]) / 2.0;  // Average filtered
         if((Nv > 500) && (Pv < 0.75*Nv) && (RFDDarray[SelectedRFBoard].RFCD[i & 1].DriveLevel > 0))
         {
           DisplayMessageButtonDismiss(" Arc detected! ");
           LogMessage("RF arc detected.");
           RFarcDrv = RFDDarray[SelectedRFBoard].RFCD[i & 1].DriveLevel;
           RFarcV   = Nv;
           RFDDarray[SelectedRFBoard].RFCD[i & 1].DriveLevel = 0;
         }
      }
      // Limit test
      if (RFpVpps[SelectedRFBoard][i & 1] < 0) RFpVpps[SelectedRFBoard][i & 1] = 0;
      if (RFnVpps[SelectedRFBoard][i & 1] < 0) RFnVpps[SelectedRFBoard][i & 1] = 0;
      //if (abs(RFpVpps[SelectedRFBoard][i & 1]) > MaxRFVoltage) MaxRFVoltage = abs(RFpVpps[SelectedRFBoard][i & 1]);
      //if (abs(RFnVpps[SelectedRFBoard][i & 1]) > MaxRFVoltage) MaxRFVoltage = abs(RFnVpps[SelectedRFBoard][i & 1]);
      // Calculate RF head power
      V = Counts2Value(ADCvals[RFDD.RFCD[i & 1].DriveVADCchan.Chan], &RFDD.RFCD[i & 1].DriveVADCchan);
      I = Counts2Value(ADCvals[RFDD.RFCD[i & 1].DriveIADCchan.Chan], &RFDD.RFCD[i & 1].DriveIADCchan);
      // Filter with 1st order difference equation
      Powers[SelectedRFBoard][i & 1] = Filter * (V * I) + (1 - Filter) * Powers[SelectedRFBoard][i & 1];
      if (Powers[SelectedRFBoard][i & 1] < 0) Powers[SelectedRFBoard][i & 1] = 0;
    }
    if (abs(RFpVpps[SelectedRFBoard][i & 1]) > MaxRFVoltage) MaxRFVoltage = abs(RFpVpps[SelectedRFBoard][i & 1]);
    if (abs(RFnVpps[SelectedRFBoard][i & 1]) > MaxRFVoltage) MaxRFVoltage = abs(RFnVpps[SelectedRFBoard][i & 1]);
  }
  // Reselect the active channel's board
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  // Read the ADC monitor values for the selected channel.
  RFpVpp = RFpVpps[SelectedRFBoard][SelectedRFChan & 1];
  RFnVpp = RFnVpps[SelectedRFBoard][SelectedRFChan & 1];
  Power = Powers[SelectedRFBoard][SelectedRFChan & 1];
  if (ActiveDialog->Entry == RFdriverDialogEntriesPage1)
  {
    if((ActiveDialog->Selected != disIndex+1) || (ActiveDialog->State == M_SCROLLING)) DisplayDialogEntry(&ActiveDialog->w, &ActiveDialog->Entry[disIndex+1], false);
    if(++disIndex >= 6) disIndex = 0;
  }
  RFcontrol();
  RFCD = RFDD.RFCD[SelectedRFChan & 1]; 
}

void EnablePLL2(int add, int brd, bool state)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(brd);
  SetPLL2enable(add,state,brd);
  if(cb != brd) SelectBoard(cb);
  ReleaseTWI();  
}

void EnablePLL3(int add, int brd, bool state)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(brd);
  SetPLL3enable(add,state,brd);
  if(cb != brd) SelectBoard(cb);
  ReleaseTWI();    
}

// Gate ISR functions
void RF_A1_ISR(void)
{
  if(RFgatingOff) return;
  if(DIh[0][0]->test(RFDDarray[0].RFgateTrig[0])) 
  {
    analogWrite(RFDDarray[0].RFCD[0].PWMchan, (RFDDarray[0].RFCD[0].DriveLevel * PWMFS) / 100);
//    if(gatedDrive[0] == 0)
    {
      if(AcquireTWI()) EnablePLL2(RFDDarray[0].CLOCKadr,0,true);
      else TWIqueue(EnablePLL2,RFDDarray[0].CLOCKadr,0,true);
    }
  }
  else 
  {
    analogWrite(RFDDarray[0].RFCD[0].PWMchan, (gatedDrive[0] * PWMFS) / 100);
    if(gatedDrive[0] == 0)
    {
      if(AcquireTWI()) EnablePLL2(RFDDarray[0].CLOCKadr,0,false);
      else TWIqueue(EnablePLL2,RFDDarray[0].CLOCKadr,0,false);
    }
  }
}
void RF_A2_ISR(void)
{
  if(RFgatingOff) return;
  if(DIh[0][1]->test(RFDDarray[0].RFgateTrig[1])) 
  {
    analogWrite(RFDDarray[0].RFCD[1].PWMchan, (RFDDarray[0].RFCD[1].DriveLevel * PWMFS) / 100);
//    if(gatedDrive[1] == 0)
    {
      if(AcquireTWI()) EnablePLL3(RFDDarray[0].CLOCKadr,0,true);
      else TWIqueue(EnablePLL3,RFDDarray[0].CLOCKadr,0,true);
    }
  }
  else 
  {
    analogWrite(RFDDarray[0].RFCD[1].PWMchan, (gatedDrive[1] * PWMFS) / 100);
    if(gatedDrive[1] == 0)
    {
      if(AcquireTWI()) EnablePLL3(RFDDarray[0].CLOCKadr,0,false);
      else TWIqueue(EnablePLL3,RFDDarray[0].CLOCKadr,0,false);
    }
  }
}
void RF_B1_ISR(void)
{
  if(RFgatingOff) return;
  if(DIh[1][0]->test(RFDDarray[1].RFgateTrig[0])) 
  {
    analogWrite(RFDDarray[1].RFCD[0].PWMchan, (RFDDarray[1].RFCD[0].DriveLevel * PWMFS) / 100);
//    if(gatedDrive[2] == 0)
    {
      if(AcquireTWI()) EnablePLL2(RFDDarray[1].CLOCKadr,1,true);
      else TWIqueue(EnablePLL2,RFDDarray[1].CLOCKadr,1,true);
    }
  }
  else 
  {
    analogWrite(RFDDarray[1].RFCD[0].PWMchan, (gatedDrive[2] * PWMFS) / 100);
    if(gatedDrive[2] == 0)
    {
      if(AcquireTWI()) EnablePLL2(RFDDarray[1].CLOCKadr,1,false);
      else TWIqueue(EnablePLL2,RFDDarray[1].CLOCKadr,1,false);
    }
  }
}
void RF_B2_ISR(void)
{
  if(RFgatingOff) return;
  if(DIh[1][1]->test(RFDDarray[1].RFgateTrig[1])) 
  {
    analogWrite(RFDDarray[1].RFCD[1].PWMchan, (RFDDarray[1].RFCD[1].DriveLevel * PWMFS) / 100);
//    if(gatedDrive[3] == 0)
    {
      if(AcquireTWI()) EnablePLL3(RFDDarray[1].CLOCKadr,1,true);
      else TWIqueue(EnablePLL3,RFDDarray[1].CLOCKadr,1,true);
    }
  }
  else 
  {
    analogWrite(RFDDarray[1].RFCD[1].PWMchan, (gatedDrive[3] * PWMFS) / 100);
    if(gatedDrive[3] == 0)
    {
      if(AcquireTWI()) EnablePLL3(RFDDarray[1].CLOCKadr,1,false);
      else TWIqueue(EnablePLL3,RFDDarray[1].CLOCKadr,1,false);
    }
  }
}

//
// This section contains all the serial command processing routines.
//

// Reports the number of RF output channels avalible.
void RFnumber(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfRFChannels);
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsChannelValid(int channel, bool Response = true)
{
  if ((channel >= 1) && (channel <= NumberOfRFChannels)) return true;
  if(!Response) return false;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Set a channels freqency
void RFfreq(int channel, int freq)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If freq value is invalid send NAK and exit
  if ((freq < MinFreq) || (freq > MaxFreq))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the frequency
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[(channel - 1) & 1].Freq = freq;
  if (channel - 1 == SelectedRFChan) RFCD.Freq = freq;
}

void RFdrive(int channel, float Drive)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  i = BoardFromSelectedChannel(channel - 1);
  // If Drive value is invalid exit
  if ((Drive < 0) || (Drive > RFDDarray[i].RFCD[(channel - 1) & 1].MaxDrive)) return;
  RFDDarray[i].RFCD[(channel - 1) & 1].DriveLevel = Drive;
  if (channel - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
}

void RFdrive(char *Chan, char *Val)
{
  int   channel, i;
  float Drive;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Drive);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  i = BoardFromSelectedChannel(channel - 1);
  // If Drive value is invalid send NAK and exit
  if ((Drive < 0) || (Drive > RFDDarray[i].RFCD[(channel - 1) & 1].MaxDrive))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  RFDDarray[i].RFCD[(channel - 1) & 1].DriveLevel = Drive;
  if (channel - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
}

void RFvoltage(int channel, float Voltage)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  // If Drive value is invalid exit
  if ((Voltage < 0) || (Voltage > 5000.0)) return;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[(channel - 1) & 1].Setpoint = Voltage;
  if (channel - 1 == SelectedRFChan) RFCD.Setpoint = Voltage;
}

void RFvoltage(char *Chan, char *Val)
{
  int   channel, i;
  float Voltage;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Voltage);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If Drive value is invalid send NAK and exit
  if ((Voltage < 0) || (Voltage > 4000.0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[(channel - 1) & 1].Setpoint = Voltage;
  if (channel - 1 == SelectedRFChan) RFCD.Setpoint = Voltage;
}

// Reports the selected channels mode, MANUAL or AUTO
void RFmodeReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // report the mode
  SendACKonly;
  if(SerialMute) return;
  int i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i].RFCD[(channel - 1) & 1].RFmode == RF_MANUAL) serial->println("MANUAL");
  else serial->println("AUTO");
}

// Sets the selected channels mode, MANUAL or AUTO
void RFmodeSet(char *chan, char *mode)
{
  int    channel;
  String sToken;

  sToken = chan;
  channel = sToken.toInt();
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  sToken = mode;
  if((sToken != "AUTO") && (sToken != "MANUAL"))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  SendACK;
  int i = BoardFromSelectedChannel(channel - 1);
  if(sToken == "MANUAL") RFDDarray[i].RFCD[(channel - 1) & 1].RFmode = RF_MANUAL;
  else 
  {
    RFDDarray[i].RFCD[(channel - 1) & 1].Setpoint = (RFpVpps[i][(channel - 1) & 1] + RFnVpps[i][(channel - 1) & 1]) / 2;
    RFDDarray[i].RFCD[(channel - 1) & 1].RFmode = RF_AUTO;
  }
  // If this channel is displayed on the MIPS UI then update
  if((channel-1) == SelectedRFChan)
  {
    RFCD.Setpoint = RFDDarray[i].RFCD[(channel - 1) & 1].Setpoint;
    RFCD.RFmode = RFDDarray[i].RFCD[(channel - 1) & 1].RFmode;
    if(RFDDarray[i].RFCD[(channel - 1) & 1].RFmode == RF_MANUAL) strcpy(RFmode, "MANUAL");
    else strcpy(RFmode, "AUTO");
    RFmodeChange();
    // If this page is being dispayed then refresh
    if(ActiveDialog == &RFdriverDialog) DialogBoxDisplay(&RFdriverDialog);
  }
}

// Report a channels frequency
void RFfreqReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Report the channels frequency
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[(channel - 1) & 1].Freq);
}

// Auto tune a selected channel
void RFautoTune(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning)
  {
    SetErrorCode(ERR_TUNEINPROCESS);
    SendNAK;
    return;
  }
  // Exit if not in manual mode for this channel
  i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i].RFCD[(channel - 1) & 1].RFmode != RF_MANUAL)
  {
    SetErrorCode(ERR_NOTINMANMODE);
    SendNAK;
    return;    
  }
  // Set the tune flag and exit
  SendACK;
  TuneRFChan = channel - 1;
  TuneRequest = true;
  TuneReport = true;   // Causes the auto tune algorithm to send report
}

// Auto retune a selected channel, this function starts and the current freq and power settings and
// peaks the output by making small adjustments
void RFautoRetune(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning)
  {
    SetErrorCode(ERR_TUNEINPROCESS);
    SendNAK;
    return;
  }
  // Exit if not in manual mode for this channel
  i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i].RFCD[(channel - 1) & 1].RFmode != RF_MANUAL)
  {
    SetErrorCode(ERR_NOTINMANMODE);
    SendNAK;
    return;    
  }
  // Set the tune flag and exit
  SendACK;
  TuneRFChan = channel - 1;
  RetuneRequest = true;
  TuneReport = true;   // Causes the auto tune algorithm to send report
}

void RFvoltageReportP(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFpVpps[i][(channel - 1) & 1]);
}

void RFvoltageReportN(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFnVpps[i][(channel - 1) & 1]);
}

void RFdriveReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[(channel - 1) & 1].DriveLevel);
}

// Reports the voltage setpoint
void RFvoltageReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[(channel - 1) & 1].Setpoint);
}

void RFheadPower(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(Powers[i][(channel - 1) & 1]);
}

void GetRFpwrLimit(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].PowerLimit);
}

void SetRFpwrLimit(int channel, int Power)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  if((Power < 1) || (Power > 100)) BADARG;
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].PowerLimit = Power;
  if(i == SelectedRFBoard) RFDD.PowerLimit = Power;
}

// Reports the frequency and Vpp for + and - channels for each RF channel in system.
void RFreportAll(void)
{
  int  i,brd;

  if (SerialMute) return;
  SendACKonly;
  for(i=1;i<=4;i++)
  {
    if (!IsChannelValid(i,false)) break;
    brd = BoardFromSelectedChannel(i - 1);
    if(i > 1) serial->print(",");
    serial->print(RFDDarray[brd].RFCD[(i - 1) & 1].Freq);
    serial->print(",");
    serial->print(RFDDarray[brd].RFCD[(i - 1) & 1].DriveLevel);
    serial->print(",");
    serial->print(RFpVpps[brd][(i - 1) & 1]);
    serial->print(",");
    serial->print(RFnVpps[brd][(i - 1) & 1]);
  }
  serial->println("");
}

// Sets the defined channels calibration parameters to the values passed. The 
// parameters are in the ring buffer when this function is called.
// channel,slope, intercept
// This function sets the pos and neg monitor calibration to the same values.
void RFcalParms(void)
{
   char   *Token;
   String sToken;
   int    ch,brd;
   float  m,b;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     m = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     b = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the channel and exit if error
     if (!IsChannelValid(ch,false)) break;
     brd = BoardFromSelectedChannel(ch - 1);
     RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.m = m;
     RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.b = b;
     RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.m = m;
     RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.b = b;  
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
  
}

// This function saves the RF driver module data to EEPROM. All detected RF modules are saved.
// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveRF2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<2; b++)
  {
    if(RFdriverBoards[b])
    {
      SelectBoard(b);
      if (WriteEEPROM(&RFDDarray[b], RFDDarray[b].EEPROMadr, 0, sizeof(RFdriverData)) != 0) berr = true;
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

// This routine processes the RFgateDisable command. If disabled the flag is set and all the 
// RF channels are enabled
void RFgateDisable(char *cmd)
{
  String sToken;
  int    i,brd;

  sToken = cmd;
  if(sToken == "TRUE")
  {
    RFgatingOff = true;
    // Enable all channels
    for (i = 0; i < NumberOfRFChannels; i++)
    {
      brd = BoardFromSelectedChannel(i);
      SelectBoard(brd);
      if((i & 1) == 0) EnablePLL2(RFDDarray[brd].CLOCKadr,brd,true);
      else EnablePLL3(RFDDarray[brd].CLOCKadr,brd,true);
    }
  }
  else if(sToken == "FALSE") RFgatingOff = false;
  else BADARG;
  SendACK;
}

// This routine will set the Auto Tune abort flag
void SetRFautoTuneAbort(void)
{
  TuneAbort = true;
  SendACK;
}

// Software calibration functions to make minor adjustment to the RF level readback detectors. 
// These routines interate to converge on the desired output. To use this feature you need
// to set the RF level and read its level then calibrate the channel by defining the desired
// readback level.
void RFcalP(char *channel, char *Vpp)
{
  String sToken;
  int ch,brd,ADCraw;
  float vpp,MVpp, m;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  brd = BoardFromSelectedChannel(ch - 1);
  if(vpp < 0.0)
  {
    RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.m = 32;
    SendACK;
    return;
  }
  // Read the ADC value for the current settings.
  ADCraw = AD7998(RFDDarray[brd].ADCadr, RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.Chan, 100);
  // Save the current gain value
  m = RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.m;
  // Adj m to find calibration match
  for(int i=0;i<100000;i++)
  {
    MVpp = RFdriverCounts2Volts(RFDDarray[brd].Rev, ADCraw, &RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan);
    RFDDarray[brd].RFCD[(ch-1) & 1].RFpADCchan.m -= 10*(vpp - MVpp)/vpp;
    if((vpp - MVpp) < 1) break;
    WDT_Restart(WDT);
  }
  SendACK;
}

void RFcalN(char *channel, char *Vpp)
{
  String sToken;
  int ch,brd,ADCraw;
  float vpp,MVpp, m;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  brd = BoardFromSelectedChannel(ch - 1);
  if(vpp < 0.0)
  {
    RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.m = 32;
    SendACK;
    return;
  }
  // Read the ADC value for the current settings.
  ADCraw = AD7998(RFDDarray[brd].ADCadr, RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.Chan, 100);
  // Save the current gain value
  m = RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.m;
  // Adj m to find calibration match
  for(int i=0;i<100000;i++)
  {
    MVpp = RFdriverCounts2Volts(RFDDarray[brd].Rev, ADCraw, &RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan);
    RFDDarray[brd].RFCD[(ch-1) & 1].RFnADCchan.m -= 10*(vpp - MVpp)/vpp;
    if((vpp - MVpp) < 1) break;
    WDT_Restart(WDT);
  }
  SendACK;
}

//
// The following functions support the piece wise linear calibration model.
//

// These functions will generate the calibration table. The user will adjust the 
// drive to set a desired voltage and then enter the value. Up to ten points
// be definded. Enter an empty string to terminate data entry. It is assumed 
// that the channel has been tuned and is properly connected.
// channel is 1 through number of channels
// phase is RF+ or RF-
uint8_t PWLch;

void RFdriveAllowADJ(void)
{
  int   brd;
  float Drive;
  
  // Call the task system to allow all system processing
  control.run();
  // Process any encoder event
  if (ButtonRotated)
  {
    ButtonRotated = false;
    Drive = ((float)encValue)/10.0;
    encValue = 0;
    // Get the RFdiver structure and adjust the drive level 
    if (!IsChannelValid(PWLch,false)) return;
    brd = BoardFromSelectedChannel(PWLch - 1);
    // If Drive value is invalid exit
    Drive += RFDDarray[brd].RFCD[(PWLch - 1) & 1].DriveLevel;
    if ((Drive < 0) || (Drive > RFDDarray[brd].RFCD[(PWLch - 1) & 1].MaxDrive)) return;
    RFDDarray[brd].RFCD[(PWLch - 1) & 1].DriveLevel = Drive;
    if (PWLch - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
  }
}

void genPWLcalTable(char *channel, char *phase)
{
  String sToken;
  char   *res;
  int    brd,ph,i;
  float  Drive;
  
  sToken = channel;
  PWLch = sToken.toInt();
  if (!IsChannelValid(PWLch,false)) return;
  brd = BoardFromSelectedChannel(PWLch - 1);
  sToken = phase;
  if((sToken != "RF+") && (sToken != "RF-")) BADARG;
  if(sToken == "RF+") ph = 0;
  else ph = 1;
  // Send the user instructions.
  serial->println("This function will generate a piecewise linear");
  serial->println("calibration table. Set the drive level to reach");
  serial->println("desired calibration points and then enter the");
  serial->println("measured value to the nearest volt. Press enter");
  serial->println("When finished. Voltage must be increasing!");
  // Loop to allow user to adjust drive and enter measured voltage
  RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].num = 0;
  i=0;
  Drive = RFDDarray[brd].RFCD[(PWLch - 1) & 1].DriveLevel;
  while(true)
  {
     serial->print("\nPoint ");
     serial->println(i+1);
     res = UserInput("Enter measured voltage: ", RFdriveAllowADJ);
     if( res == NULL) break;
     sToken = res;
     RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].Value[i] = sToken.toInt();
     // Read the ADC raw counts
     SelectBoard(BoardFromSelectedChannel(PWLch - 1));
     if(ph == 0) RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].ADCvalue[i] = AD7998(RFDDarray[brd].ADCadr, RFDDarray[brd].RFCD[(PWLch - 1) & 1].RFpADCchan.Chan, 100);
     else RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].ADCvalue[i] = AD7998(RFDDarray[brd].ADCadr, RFDDarray[brd].RFCD[(PWLch - 1) & 1].RFpADCchan.Chan, 100);
     i++;
     RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].num = i;
     if(i>=MAXPWL) break;
  }
  serial->println("");
  // Report the table
  serial->print("Number of table entries: ");
  serial->println(RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].num);
  for(i=0;i<RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].num;i++)
  {
    serial->print(RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].Value[i]);
    serial->print(",");
    serial->println(RFDDarray[brd].PWLcal[(PWLch - 1) & 1][ph].ADCvalue[i]);
  }
  // Done!
  serial->println("\nData entry complete!");
  RFDDarray[brd].RFCD[(PWLch - 1) & 1].DriveLevel = Drive;
  if (PWLch - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
}

// This function will use the selected piecewise linear table to convert the 
// adcval to output voltage.
// ch = Rf channel 1 through maximum RF channels
// ph = phase, 0 = RF+, 1 = RF-
float PWLlookup(int ch, int ph, int adcval)
{
  int            brd,i;
  PWLcalibration *pwl;
  
  if (!IsChannelValid(ch,false)) return 0;
  brd = BoardFromSelectedChannel(ch - 1);
  pwl = &RFDDarray[brd].PWLcal[(ch - 1) & 1][ph];
  return PWLlookup(pwl,adcval);
}

// The following functions support analog voltage control of the RFdriver module. 
// ADC0 controls RFdriver channel 1 and ADC1 controls RFdriver channel 1. 

// This commands sets up an ADC channel and sets the ADC input gain. The gain is
// used to convert the ADC counts to a 0 to 10V range.
// chan is 1 or 2
// gain is a float that is multiplied by the ADC count
void setupADCRFcontrol(char *chan, char *gain)
{
  String token;

  token = chan;
  int adc = token.toInt() - 1;
  token = gain;
  float adcgain = token.toFloat();
  if((adc!=0)&&(adc!=1)) BADARG;
  if(adcRFcontrol[adc] == NULL) adcRFcontrol[adc] = new ADCcontrolRF;
  adcRFcontrol[adc]->gain = adcgain;
  adcRFcontrol[adc]->RFchan = adc + 1;
  adcRFcontrol[adc]->enable = false;
  SendACK;
}
void setADCRFenable(char *chan, char *value)
{
  String token;

  token = chan;
  int adc = token.toInt() - 1;
  if((adc!=0)&&(adc!=1)) BADARG;
  if(adcRFcontrol[adc] == NULL) BADARG;
  token = value;
  if(token == "TRUE") adcRFcontrol[adc]->enable = true;
  else if(token == "FALSE") adcRFcontrol[adc]->enable = false;
  else BADARG;
  SendACK;
}
void getADCRFenable(char *chan)
{
  String token;

  token = chan;
  int adc = token.toInt() - 1;
  if((adc!=0)&&(adc!=1)) BADARG;
  if(adcRFcontrol[adc] == NULL) BADARG;
  SendACKonly;
  if(SerialMute) return;
  if(adcRFcontrol[adc]->enable) serial->println("TRUE");
  else serial->println("FALSE");
} 

void setRFgatedDrv(char *Chan, char *Val)
{
  int ch;
  String token;

  token = Chan;
  ch = token.toInt();
  if (!IsChannelValid(ch)) return;
  int brd = BoardFromSelectedChannel(ch - 1);
  token = Val;
  gatedDrive[ch - 1] = token.toFloat();
  if(gatedDrive[ch - 1] < 0) gatedDrive[ch - 1] = 0;
  if(gatedDrive[ch - 1] > RFDDarray[brd].RFCD[ch & 1].MaxDrive) gatedDrive[ch - 1] = RFDDarray[brd].RFCD[ch & 1].MaxDrive;
  SendACK;
}
void getRFgatedDrv(int Chan)
{
  if (!IsChannelValid(Chan)) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(gatedDrive[Chan - 1]);
}

#endif
