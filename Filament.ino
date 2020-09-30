//
// Filament
//
// This file contains the driver software for the Filament control board. This board has two filament channels that can be independatly set and controlled.
// The fillament controller supports the following features:
//
//  1.) The following control modes are supported
//      a.) Current control mode
//      b.) Current control mode with voltage control to minimize power disapated in current source MOSFET
//  2.) Limit testing
//      a.) Maximum power
//  3.) Serial commands support most functions
//
// New capability added January 21, 2016:
//  1.) Add serial commands for reading and setting the current ramp rate. done but not tested, 12/23/15
//       GFLRT       This command returns the rate in amps per second
//       SFLRT,rate  Allows you to set the rate in amps per second
//  2.) Add serial command to report the current, voltage, and power on a user defined interval.
//      This command will also require a stop command.
//
//  Details for the ramping capability to the filament controller to automatically ramp between between two currents at the 
//  defined current ramp rate. This capability could be used through the user interface or by using host interface command. 
//  The commands I will add are:
//
//    GFLP1           Get current point 1
//    SFLP1,current   Set current point 1
//    GFLP2           Get current point 2
//    SFLP2,current   Set current point 2
//    GFLCY           Get the number of ramp cycles
//    SFLCY,cycles    Set the number of ramp cycles where 1 cycle is ramping from Point 1 to Point 2 and back to Point 1.
//    GFLENAR         Returns the status of the cycling, OFF, or number of cycles remaining
//    SFLENAR,state   Sets the cycling to ON or OFF
//
//  Ramping logic
//    - Set current point 1
//    - When we reach current point 1 set to current point 2
//    - when we reach current point 1 count cycle and if not done return to set 1
//
//    To support the reporting function I will add a command to cause the system to report values
//
//    RFLPARMS,chan,period  This command will cause the system to report filament channel parameters at the period (in seconds) defined.
//                          If the period is set to 0 the reporting will stop. The following parameters will be reported and comma separated
//                          on one line:
//                          FIL#, where # is the selected filament channel, 1 or 2
//                          current,
//                          voltage,
//                          power
//                          Implement using a task scheduled to fire at the definded rate.
//
//  Add emision current monitoring and display. Added July 2016
//    If the emission current flag is set then use the current sense resistor value and the voltage difference between setpoint
//    and readback on channel 8 to calculate the emision current.
//    Move ramp rate to display page 2 to make room on first page for emission current.
//    Add flag for emission current monitoring and also add variable for sense resistor size.
//    Need to figure out how to fix DC bias module. Its read back is a 500 uA load at 50 volts so if I make the sense resistor
//    10K that will be a 5 volt drop at no load current. I think this will calibrate out so it may be ok, then I can just read
//    setpoint - readback and devide by sense resistor to get emission current. This is how the code is designed.
//    Use sense resistor value as the flag, if 0 no sense, other wise assume on channel 8 and display sense. Add sense resistor 
//    command to MIPS, both set and read.
//
// Add current direction, through the filament, control. This is done by tieing channel 1 and 2 together but in reverse
// direction. The firmware is upgraded (REV = 2) to only allow one driver on an a time and all parameters are set to the 
// same values. Current direction control the channel that is enabled. Current direction can only be changed when the channel
// is disabled. The followiong firmware updated are needed:
//    1.) Need to keep backwards compatability. Use Rev = 2 to flag this mode.
//    2.) In this mode only 1 channel per board
//    3.) Make sure all parameters are the same
//    4.) Allow only one channel to be enabled at a time
//    5.) Only allow polarity change when disabled
//
// Firmware rev levels and functions
//  Rev 1 = Orginal 2 channel filament module
//  Rev 2 = Current reversable single channel mode. Need to tie the two channels together.
//  Rev 3 = Supports on module emission current monitoring, added on PCB rev 4.0
//  Rev 4 = Supports the e-msion single board solution. This is a one channel filament 
//          and uses the AD5593 analog and digital IO chip. A rev 4 variant is used to set up
//          this mode. added sept 10, 2019
//
// Gordon Anderson
// July 25, 2015
// March 7,2017, revised to support firmware rev 2
// August 31, 2017, revised to support firmware rev 3
//
#include <stdarg.h>
#include "wire.h"
#include "SD.h"
#include "string.h"
#include "Serial.h"
#include "Filament.h"
#include "dialog.h"

//MIPS Threads
Thread FilamentThread  = Thread();
// A reporting thread for each channel
Thread FilamentReportingThreads[4] = {Thread(),Thread(),Thread(),Thread()};

#define MaxFilCur 12

#define FD FDarray[SelectedFilamentBoard]

FilamentData  FDarray[2] = {FILAMENT_Rev_1, FILAMENT_Rev_1};

int  NumberOfFilamentChannels = 0;
bool FilamentBoards[2] = {false, false};
bool CurrentDir;                   // Reflects the current modules current direction value
bool FLserialWD = false;           // Serial activity watchdog timer
int  FLserialTimer = 100;          // 10 sec timer
int  Fchannel = 1;                 // User selected channel
int  SelectedFilamentChan = 0;     // Active channel
int  SelectedFilamentBoard = 0;    // Active board, 0 or 1 = A or B
FilamentChannel FCD;               // Holds the selected channel's data
FilamentCycling FCY;               // Holds the selected channel's cycling data

// This array is used to enable the setpoint ramp rate. This array contains the actual setpoint
// that is moved at the defined ramp rate in amps/sec for the defined channel.
float CurrentSetpoints[2][2] = {0, 0, 0, 0};
#define VstepSize 0.1
float FilamentVoltageSetpoints[2][2] = {0, 0, 0, 0};

// These arrays hold the monitor values, array indexes are board,channel. These are scanned
// in the loop and filtered. These values are both displayed and also used for control
// and limit testing.
float FsupplyVsUF[2][2];    // Unfiltered filament supply voltage, used for control loop
float FsupplyVs[2][2];
float FvoltagesUF[2][2];    // Unfiltered filament voltage, used for control loop
float Fvoltages[2][2];
float FcurrentsUF[2][2];    // Unfiltered filament current, used for control loop
float Fcurrents[2][2];
float Fpowers[2][2];
// These are the actively displayed values
float FsupplyV;
float Fvoltage;
float Fcurrent;
float Fpower;
float BiasCurrent;

extern DialogBoxEntry FilamentEntriesPage2[];
extern DialogBoxEntry FilamentEntriesCalibration[];
extern DialogBox      FilamentCycleDialog;

DialogBoxEntry FilamentEntriesPage1[] = {
  {" Channel"            , 0, 1, D_INT  , 1, 2, 1, 21, false, "%2d", &Fchannel, NULL, SelectFilamentChannel},
  {" Enable"             , 0, 2, D_ONOFF, 0, 1,  1, 20, false, NULL, &FCD.FilamentPwr, NULL, NULL},
  {" Current"            , 0, 3, D_FLOAT, 0, MaxFilCur, 0.01, 18, false, "%5.2f", &FCD.CurrentSetpoint, NULL, NULL},
  {" Voltage"            , 0, 4, D_FLOAT, .7, 5, 0.1, 18, false, "%5.2f", &FCD.FilamentVoltage, NULL, NULL},
  {" Bias I, uA"         , 0, 5, D_FLOAT, 0, 0, 0, 18, true, "%5.0f", &BiasCurrent, NULL, NULL},
  {" Supply V"           , 0, 6, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &FsupplyV, NULL, NULL},
  {" Filament V"         , 0, 7, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &Fvoltage, NULL, NULL},
  {" Filament I"         , 0, 8, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &Fcurrent, NULL, NULL},
  {" Power"              , 0, 9, D_FLOAT, 0, 0, 0, 18, true, "%5.1f", &Fpower, NULL, NULL},
  {" Next page"          , 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesPage2, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char *FmodeList = "Ictrl,IVctrl";
char Fmode[8]   = "Ictrl";

DialogBoxEntry FilamentEntriesPage2[] = {
  {" Mode"               , 0, 1, D_LIST , 0, 0, 7, 16, false, FmodeList, Fmode, NULL, FmodeChange},
  {" Max power"          , 0, 2, D_FLOAT, 1, 20, 1, 18, false, "%5.0f", &FCD.MaxPower, NULL, NULL},
  {" Ramp rate"          , 0, 3, D_FLOAT, 0, 1, 0.01, 18, false, "%5.3f", &FCD.RampRate, NULL, NULL},
  {" Current dir"        , 0, 4, D_OFF, 0, 1, 1, 19, false, NULL, &CurrentDir, NULL, CurrentDirChange},
  {" Calibration"        , 0, 5, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesCalibration, NULL, NULL},
  {" Cycle menu"         , 0, 8, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FilamentCycleDialog, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveFilamentSettings, NULL},
  {" Restore settings"   , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorFilamentSettings, NULL},
  {" First page"         , 0,11, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesPage1, NULL, NULL},
  {NULL},
};

DialogBox FilamentDialog = {
  {"Filament control params",ILI9340_BLACK, ILI9340_WHITE,2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0,false, FilamentEntriesPage1
};

DialogBoxEntry FilamentEntriesCalibration[] = {
  {" Cal Supply V"       , 0, 1, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentSupplyV, NULL},
  {" Cal Filament V"     , 0, 2, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentV, NULL},
  {" Cal Filament I"     , 0, 3, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentI, NULL},
  {" Cal Emission I"     , 0, 4, D_OFF, 0, 0, 0, 0, false, NULL, NULL, CalEmissionI, NULL},
  {" Return"             , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesPage2, NULL, NULL},
  {NULL},
};

int CurrentCycle;

DialogBoxEntry FilamentCycleEntries[] = {
  {" Enable cycling"      , 0, 1, D_YESNO, 0, 1, 1, 20, false, NULL, &FCY.FCenable, NULL, NULL},
  {" Current 1"           , 0, 2, D_FLOAT, 0, MaxFilCur, 0.01, 18, false, "%5.2f", &FCY.FCpoint1, NULL, NULL},
  {" Current 2"           , 0, 3, D_FLOAT, 0, MaxFilCur, 0.01, 18, false, "%5.2f", &FCY.FCpoint2, NULL, NULL},
  {" Num cycles"          , 0, 4, D_INT  , 0, 1000, 1, 18, false, "%5d", &FCY.FCnumCyl, NULL, NULL},

  {" Current cycle"       , 0, 6, D_INT, 0, 0, 0, 15, true, "%8d", &CurrentCycle, NULL, NULL},

  {" Return filament menu", 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &FilamentDialog, NULL, NULL},
  {NULL},
};

DialogBox FilamentCycleDialog = {
  {"Filament cycling params",ILI9340_BLACK, ILI9340_WHITE,2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0,false, FilamentCycleEntries
};

MenuEntry MEFilamentModule = {" Filament module", M_DIALOG, 0, 0, 0, NULL, &FilamentDialog, NULL, NULL};

// Calibration functions
void CalFilamentSupplyV(void)
{
  ChannelCal CC;
  char *Name = " Filament Supply Voltage";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  if(FD.Rev == 4) CC.ADCpointer = &AD5593readADC;
  else CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 5;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.DCfsuply;
  CC.ADCreadback = &FCD.DCfsuplyMon;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  FD.FCD[Channel2Index(SelectedFilamentChan)] = FCD;
}

void CalFilamentV(void)
{
  ChannelCal CC;
  char *Name = "    Filament Voltage";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  if(FD.Rev == 4) CC.ADCpointer = &AD5593readADC;
  else CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 5;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.DCfsuply;
  CC.ADCreadback = &FCD.Fvoltage;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  FD.FCD[Channel2Index(SelectedFilamentChan)] = FCD;
}

void CalFilamentI(void)
{
  ChannelCal CC;
  char *Name = "    Filament Current";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  if(FD.Rev == 4) CC.ADCpointer = &AD5593readADC;
  else CC.ADCpointer = &AD7998;
  //  CC.Min = -2;
  //  CC.Max = 4;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.Fcurrent;
  CC.ADCreadback = &FCD.FcurrentMon;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, 5);
  FD.FCD[Channel2Index(SelectedFilamentChan)] = FCD;
}

void CalEmissionI(void)
{
  ChannelCal CC;
  char *Name = "    Emission Current";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  if(FD.Rev == 4) CC.ADCpointer = &AD5593readADC;
  else CC.ADCpointer = &AD7998;
  CC.DACaddr = 0;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = NULL;
  CC.ADCreadback = &FD.Ecurrent;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, 100);
}

void FmodeChange(void)
{
  String mode;

  mode = Fmode;
  if (mode == "Ictrl")
  {
    FCD.Mode = FmodeI;
    FilamentEntriesPage1[3].NoEdit = false;
  }
  else if (mode == "IVctrl")
  {
    FCD.Mode = FmodeIV;
    FilamentEntriesPage1[3].NoEdit = true;
  }
  //FD.FCD[Channel2Index(SelectedFilamentChan)] = FCD;
  FD.FCD[0].Mode = FD.FCD[1].Mode = FCD.Mode;             // I think this fixes the bug where the rev channel does not update the Mode 
                                                          // When changed on the fwd channel. 1/27/2019
}

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected
// Accepts channel number (SC) 0 through 3
int8_t BoardFromSelectedFilamentChannel(int8_t SC)
{
  int Rev = -1;
  
  // First find a board that is present in the system and read the rev, all boards must be the 
  // same rev.
  if(FilamentBoards[0]) Rev = FDarray[0].Rev;
  if(FilamentBoards[1]) Rev = FDarray[1].Rev;
  if(Rev == -1) return(-1);
  if(Rev >= 2)
  {
    if(SC < 1)
    {
      if (FilamentBoards[0]) return (0);
      if (FilamentBoards[1]) return (1);
      return (-1);
    }
    if (SC == 1)
    {
      if ((FilamentBoards[0]) && (FilamentBoards[1])) return (1);
    }
    return (-1);
  }
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if (SC <= 1)
  {
    if (FilamentBoards[0]) return (0);
    if (FilamentBoards[1]) return (1);
    return (-1);
  }
  //  if selected channel is 2 or 3 then use board 1 and make sure board 0 is present
  if ((SC == 2) || (SC == 3))
  {
    if ((FilamentBoards[0]) && (FilamentBoards[1])) return (1);
  }
  return (-1);
}

// This function converts a channel number, 0 to 3 for rev 1 and 0 to 1 for rev 2, into an index into the
// channel structure. In the case of rev 2 the direction flag is used to define the index.
// Returns -1 on error
int Channel2Index(int channel)
{
  int b;

  b = BoardFromSelectedFilamentChannel(channel);
  if(b == -1) return(-1);
  if(FDarray[b].Rev >= 2)
  {
    if((channel<0) || (channel>1)) return(-1);
    if(FDarray[b].Idir) return(0);
    return(1);
  }
  if((channel<0) || (channel>3)) return(-1);
  return(channel & 1);
}

// Called after the user selects a channel
void SelectFilamentChannel(void)
{
  if(FD.Rev >= 2)
  {
    SelectedFilamentChan = (Fchannel - 1);
    SelectedFilamentBoard = BoardFromSelectedFilamentChannel(SelectedFilamentChan);
    FCD = FD.FCD[Channel2Index(SelectedFilamentChan)];
    FCY = FD.FCyl[Channel2Index(SelectedFilamentChan)];    
  }
  else
  {
    SelectedFilamentChan = Fchannel - 1;
    SelectedFilamentBoard = BoardFromSelectedFilamentChannel(SelectedFilamentChan);
    FCD = FD.FCD[Fchannel - 1];
    FCY = FD.FCyl[Fchannel - 1];
  }
  if (FCD.Mode == FmodeI) strcpy(Fmode, "Ictrl");
  else strcpy(Fmode, "IVctrl");
  // Update the display
  if (ActiveDialog == &FilamentDialog) DialogBoxDisplay(&FilamentDialog);
}

// Called after user updates the current direction, only used on Rev 2 of this driver
void CurrentDirChange(void)
{
  if((digitalRead(FD.FCD[0].Fpwr) == LOW) || (digitalRead(FD.FCD[1].Fpwr) ==  LOW))
  {
    CurrentDir = FD.Idir;
    return;    
  }
  if(FCD.FilamentPwr)
  {
    CurrentDir = FD.Idir;
    return;
  }
  // Here if filament power is off
  if(CurrentDir == FD.Idir) return;
  FD.Idir = CurrentDir;
  SelectFilamentChannel();
}

// Write the current board parameters to the EEPROM on the Filament board.
void SaveFilamentSettings(void)
{
  SelectBoard(SelectedFilamentBoard);
  FD.Size = sizeof(FilamentData);
  if (WriteEEPROM(&FD, FD.EEPROMadr, 0, sizeof(FilamentData)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

void RestorFilamentSettings(bool NoDisplay)
{
  FilamentData fd;

  SelectBoard(SelectedFilamentBoard);
  if (ReadEEPROM(&fd, FD.EEPROMadr, 0, sizeof(FilamentData)) == 0)
  {
    if (strcmp(fd.Name, FD.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (fd.Size != sizeof(FilamentData)) fd.Size = sizeof(FilamentData);
      fd.EEPROMadr = FD.EEPROMadr;
      memcpy(&FD, &fd, fd.Size);
      FCD = FD.FCD[Channel2Index(SelectedFilamentChan)];
      FCY = FD.FCyl[Channel2Index(SelectedFilamentChan)];
      CurrentDir = FD.Idir;
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      SelectFilamentChannel();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

void RestorFilamentSettings(void)
{
  RestorFilamentSettings(false);
}

// Init the AD5593 (Analog and digital IO chip) for the Filament module. The following 
// setup requirements:
// CH0 = Digital out, enable
// CH1 = DAC output, voltage control
// CH2 = ADC in, voltage monitor
// CH3 = ADC in, filament voltage monitor
// CH4 = DAC out, filament current control
// CH5 = ADC in, filament current monitor
// CH6 = ADC in, emission current
// Internal 2.5V reference with 0 to 2.5V range
// No pullups
void FilamentAD5593init(int8_t addr)
{
   // General purpose configuration
   AD5593write(addr, 3, 0x0100);
   // Set reference
   AD5593write(addr, 11, 0x0200);     // Set int reference
   // Set LDAC mode
   AD5593write(addr, 7, 0x0000);
   // Set DAC outputs channels
   AD5593write(addr, 5, 0x0012);
   // Set ADC input channels
   AD5593write(addr, 4, 0x006C);
   // Set DIO outputs channels
   AD5593write(addr, 8, 0x0001);
   // Turn off all pulldowns
   AD5593write(addr, 6, 0x0000);   
}

// This function is called on power up initilization. This function
// will enable the filament driver module.
void Filament_init(int8_t Board, int8_t addr)
{
  DialogBoxEntry *de;
  
  // Flag the board as present
  FilamentBoards[Board] = true;
  // Set active board to board being inited
  SelectedFilamentBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the Filament card.
  FDarray[Board].EEPROMadr = addr;
  if (NormalStartup) RestorFilamentSettings(true);
  // Init the hardware here...
  if(FDarray[Board].Rev == 4) FilamentAD5593init(FDarray[Board].ADCadr);
  else
  {
     pinMode(FDarray[Board].FCD[0].Fpwr, OUTPUT);
     pinMode(FDarray[Board].FCD[1].Fpwr, OUTPUT);
  }
  // If current sense resisance value is 0 then turn off the display,
  // Enable the display on rev 3
  de = GetDialogEntries(FilamentEntriesPage1, " Bias I, uA");
  if(de != NULL)
  {
    if(FDarray[Board].iSense == 0) de->Type = D_OFF;
    else de->Type = D_FLOAT;
    if(FDarray[Board].Rev >= 3) de->Type = D_FLOAT;
  }
  // If Rev is 2 or greater then enable the current direction option
  if(FDarray[Board].Rev >= 2)
  {
    de = GetDialogEntries(FilamentEntriesPage2, " Current dir");
    if(de != NULL) de->Type = D_FWDREV;
    // Make sure both mode values for forward and reverse are the same and 
    // if prefer IVmode. Jan 28, 2019
    FDarray[Board].FCD[0].Mode |= FDarray[Board].FCD[1].Mode;
    FDarray[Board].FCD[1].Mode |= FDarray[Board].FCD[0].Mode;
  }
  // If this is rev 3 or Rev 4 then enable the emission current calibration option
  if((FDarray[Board].Rev == 3) || (FDarray[Board].Rev == 4))
  {
    de = GetDialogEntries(FilamentEntriesCalibration, " Cal Emission I");
    if(de != NULL) de->Type = D_FUNCTION;
  }
  if(FDarray[Board].Rev == 4)
  {
    de = GetDialogEntries(FilamentEntriesPage2, " Current dir");
    if(de != NULL) de->Type = D_OFF;    
  }
  // Define the initial selected channel as 0 and setup
  Fchannel = 1;
  SelectFilamentChannel();
  // Setup the menu if this is the very first call to this init function
  if (NumberOfFilamentChannels == 0)
  {
    AddMainMenuEntry(&MEFilamentModule);
    if (ActiveDialog == NULL) DialogBoxDisplay(&FilamentDialog);
    // Configure Threads
    FilamentThread.setName("Filament");
    FilamentThread.onRun(Filament_loop);
    FilamentThread.setInterval(100);
    // Add threads to the controller
    control.add(&FilamentThread);
  }
  if(FDarray[Board].Rev >= 2) NumberOfFilamentChannels++;
  else NumberOfFilamentChannels += 2;  // Always add two channels for each board if Rev is 1
  // Set the maximum number of channels in the selection menu
  FilamentEntriesPage1[0].Max = NumberOfFilamentChannels;
}

// This function implements the filament cyclying between to programmed setpoints, point 1 and point 2.
// The current is changed at the user defined ramp rate. The user also defines how many cycles are
// desired.
FilamentCyclingStates FC_State[2][2] = {FC_STOP,FC_STOP,FC_STOP,FC_STOP};
int  FilamentCycleCounts[2][2] = {0,0,0,0};
bool FilamentCycleCountTrigger[2][2] = {false,false,false,false};

void FilamentCyclying(void)
{
  int          b,c;                  // Board and channel number to loop through
  static float SavedSetpoint[2][2];  // Saved setpoints when in the cycling mode

  // Loop through all boards and channels
  for (b = 0; b < 2; b++)
  {
    for (c = 0; c < 2; c++)
    {
      if((Channel2Index(SelectedFilamentChan) == c) &&(SelectedFilamentBoard == b)) CurrentCycle = FilamentCycleCounts[b][c];
      // Test if cycling is enabled
      if(FDarray[b].FCyl[c].FCenable)
      {
        switch (FC_State[b][c])
        {
          case FC_START:  // Save the current setpoint and set point 1
            SavedSetpoint[b][c] = FDarray[b].FCD[c].CurrentSetpoint;
            FDarray[b].FCD[c].CurrentSetpoint = FDarray[b].FCyl[c].FCpoint1;
            FC_State[b][c] = FC_WAIT_PT1;
            FilamentCycleCounts[b][c] = 0;
            break;
          case FC_WAIT_PT1: // When current setpoint = point 1 then advance to point 2
            if(CurrentSetpoints[b][c] == FDarray[b].FCyl[c].FCpoint1)
            {
              FDarray[b].FCD[c].CurrentSetpoint = FDarray[b].FCyl[c].FCpoint2;
              FC_State[b][c] = FC_WAIT_PT2;
              if(FilamentCycleCountTrigger[b][c]) FilamentCycleCounts[b][c]++;
              FilamentCycleCountTrigger[b][c] = false;
              if((FilamentCycleCounts[b][c] >= FDarray[b].FCyl[c].FCnumCyl) && (FDarray[b].FCyl[c].FCnumCyl != 0))
              {
                FDarray[b].FCD[c].CurrentSetpoint = SavedSetpoint[b][c];
                FC_State[b][c] = FC_STOP;
                FDarray[b].FCyl[c].FCenable = false;
              }
            }
            break;
          case FC_WAIT_PT2: // When current setpoint = point 2 then advance to point 1
            if(CurrentSetpoints[b][c] == FDarray[b].FCyl[c].FCpoint2)
            {
              FDarray[b].FCD[c].CurrentSetpoint = FDarray[b].FCyl[c].FCpoint1;
              FC_State[b][c] = FC_WAIT_PT1;
              FilamentCycleCountTrigger[b][c] = true;
            }
            break;
          case FC_STOP:
            FC_State[b][c] = FC_START;
            break;
          default:
            break;
        }
      }
      else
      {
        // Here if not enabled, if we are not in STOP state then 
        // restore the current setpoint and set state to stop.
        if(FC_State[b][c] != FC_STOP)
        {
          FDarray[b].FCD[c].CurrentSetpoint = SavedSetpoint[b][c];
          FC_State[b][c] = FC_STOP;
        }
      }
    }
  }
  FCD = FD.FCD[Channel2Index(SelectedFilamentChan)];
  FCY = FD.FCyl[Channel2Index(SelectedFilamentChan)];
}

// This function performs filament control and monitoring tests. The following operations are performed:
//   1.) Maximum power test, if filament power exceeds the maximum then the current setpoint is reduced.
//   2.) If the operating mode is FmodeIV then the voltage across the current source is held at .7 volts
//       by adjusting the filament supply voltage as needed. This minimizes the power disapation in the
//       current source.
void FilamentControl(void)
{
  int        b, c;  // Board and channel number to loop through
  int static skip = 0;

  if (++skip <= 10) return;
  skip = 0;
  // Loop through all the filament channels
  for (b = 0; b < 2; b++)
  {
    if (FilamentBoards[b])
    {
      for (c = 0; c < 2; c++)
      {
        if (FDarray[b].FCD[c].FilamentPwr)
        {
          // Here if power is on
          // Test if power is over the limit
          if (((FsupplyVs[b][c] - Fvoltages[b][c]) * Fcurrents[b][c]) > FDarray[b].FCD[c].MaxPower) FDarray[b].FCD[c].CurrentSetpoint -= 0.1;
          if (FDarray[b].FCD[c].CurrentSetpoint < 0) FDarray[b].FCD[c].CurrentSetpoint = 0;
          // If mode is FmodeIV then set the supply voltage as needed
          if (FDarray[b].FCD[c].Mode == FmodeIV)
          {
            if(abs(Fvoltages[b][c] - 0.8) > 0.1)
            {
              FDarray[b].FCD[c].FilamentVoltage -= (Fvoltages[b][c] - 0.75);
              if (FDarray[b].FCD[c].FilamentVoltage > 5) FDarray[b].FCD[c].FilamentVoltage = 5;
              if (FDarray[b].FCD[c].FilamentVoltage < 0.7) FDarray[b].FCD[c].FilamentVoltage = 0.7;
            }
          }
        }
      }
    }
  }
  FCD = FD.FCD[Channel2Index(SelectedFilamentChan)];
}

void ResetFilamentSerialWD(void)
{
  FLserialTimer = 100;
}

void FilamentShutdown(void)
{
  digitalWrite(FDarray[0].FCD[0].Fpwr, HIGH);
  digitalWrite(FDarray[0].FCD[1].Fpwr, HIGH);
  SelectBoard(0);
  AD5625(FDarray[0].DACadr, FDarray[0].FCD[0].DCfsuply.Chan, 0);
  AD5625(FDarray[0].DACadr, FDarray[0].FCD[0].Fcurrent.Chan, 0);
  AD5625(FDarray[0].DACadr, FDarray[0].FCD[1].DCfsuply.Chan, 0);
  AD5625(FDarray[0].DACadr, FDarray[0].FCD[1].Fcurrent.Chan, 0);
  SelectBoard(1);
  AD5625(FDarray[1].DACadr, FDarray[1].FCD[0].DCfsuply.Chan, 0);
  AD5625(FDarray[1].DACadr, FDarray[1].FCD[0].Fcurrent.Chan, 0);
  AD5625(FDarray[1].DACadr, FDarray[1].FCD[1].DCfsuply.Chan, 0);
  AD5625(FDarray[1].DACadr, FDarray[1].FCD[1].Fcurrent.Chan, 0);
}

// The process is scheduled by the Filament_init function if the module is detected.
// This is the main Filament run loop that is called every 100 millisec
void Filament_loop(void)
{
  int        b, c;  // Board and channel number to loop through
  uint16_t   ADCvals[8];
  int        adcStatus;
  float      StepSize;
  static     float LastVoltage = -1000;

  SelectedFilamentBoard = BoardFromSelectedFilamentChannel(SelectedFilamentChan);
  SelectBoard(SelectedFilamentBoard);
  if ((ActiveDialog == &FilamentDialog) || (ActiveDialog == &FilamentCycleDialog))
  {
    if(ActiveDialog->Changed)
    {
      FD.FCD[Channel2Index(SelectedFilamentChan)]  = FCD;    // This stores any changes back to the selected channels data structure
      FD.FCyl[Channel2Index(SelectedFilamentChan)] = FCY;
      // If this is rev 2 or greater then set the channel parameters to the same values
      if(FD.Rev >= 2)
      {
        FDarray[SelectedFilamentBoard].FCD[0].CurrentSetpoint = FDarray[SelectedFilamentBoard].FCD[1].CurrentSetpoint = FCD.CurrentSetpoint;
        FDarray[SelectedFilamentBoard].FCD[0].FilamentVoltage = FDarray[SelectedFilamentBoard].FCD[1].FilamentVoltage = FCD.FilamentVoltage;
        FDarray[SelectedFilamentBoard].FCD[0].RampRate        = FDarray[SelectedFilamentBoard].FCD[1].RampRate        = FCD.RampRate;
        FDarray[SelectedFilamentBoard].FCD[0].Mode            = FDarray[SelectedFilamentBoard].FCD[1].Mode            = FCD.Mode;
        FDarray[SelectedFilamentBoard].FCD[0].MaxPower        = FDarray[SelectedFilamentBoard].FCD[1].MaxPower        = FCD.MaxPower;
      }
      ActiveDialog->Changed = false;
    }
  }
  FilamentCyclying();
  // Test the serial watch dog timer
  if(FLserialWD)
  {
    // First see if anything is enabled, if so test for timeout
    if (((FilamentBoards[0]) && ((FDarray[0].FCD[0].FilamentPwr) || (FDarray[0].FCD[1].FilamentPwr))) || ((FilamentBoards[1]) && ((FDarray[1].FCD[0].FilamentPwr) || (FDarray[1].FCD[1].FilamentPwr))))
    {
      FLserialTimer--;
      if(FLserialTimer <= 0)
      {
        // If here we timed out so diable all the active boards in system
        if (FilamentBoards[0]) FDarray[0].FCD[0].FilamentPwr = FDarray[0].FCD[1].FilamentPwr = false;
        if (FilamentBoards[1]) FDarray[1].FCD[0].FilamentPwr = FDarray[1].FCD[1].FilamentPwr = false;
        FLserialTimer = 100;
        DisplayMessageButtonDismiss("No comms, Timedout!");
      }
    }
  }
  // If current sense resistance is not 0 then calculate the bias current
  // This will not work with rev 4 and the sense resistance should be zero for rev 4
  if((FDarray[0].iSense != 0) && IsPowerON())
  {
    if(DCbDarray[0] != NULL) b = 0;
    else b = 1;
    SelectBoard(b);
    // Read and average the ADC value for the current monitor
    int SenseADC =  AD7998(DCbDarray[b]->ADCadr, 7, 50);
    float BiasV = Counts2Value(SenseADC,&DCbDarray[b]->DCCD[7].DCmon) + DCbDarray[b]->DCoffset.VoltageSetpoint;
    if(DCbDarray[b]->DCCD[7].VoltageSetpoint != LastVoltage)
    {
      LastVoltage = DCbDarray[b]->DCCD[7].VoltageSetpoint;
      BiasCurrent = ((DCbDarray[b]->DCCD[7].VoltageSetpoint - BiasV) / float(FDarray[0].iSense)) * 1000000.0;
    }
    else
    {
      float BCfilter = 0.05;
      BiasCurrent = BCfilter * (((DCbDarray[b]->DCCD[7].VoltageSetpoint - BiasV) / float(FDarray[0].iSense)) * 1000000.0) + (1-BCfilter) * BiasCurrent;
    }
  }
  // Loop through all the filament channels and output all the control parmaeters and update
  // all readback values
  for (b = 0; b < 2; b++)
  {
    if (FilamentBoards[b])
    {
      SelectBoard(b);
      if((FDarray[b].ADCadr & 0xFE) == 0x10)  // This ADC/DAC used on rev 4 only
      {
        int i;
        adcStatus = 0;
        if((i=AD5593readADC(FDarray[b].ADCadr, 2)) == -1) adcStatus=-1; ADCvals[2] = i;
        if((i=AD5593readADC(FDarray[b].ADCadr, 3)) == -1) adcStatus=-1; ADCvals[3] = i;
        if((i=AD5593readADC(FDarray[b].ADCadr, 5)) == -1) adcStatus=-1; ADCvals[5] = i;
        if((i=AD5593readADC(FDarray[b].ADCadr, 6)) == -1) adcStatus=-1; ADCvals[6] = i;
      }
      else adcStatus = AD7998(FDarray[b].ADCadr, ADCvals);  // adcStatus is zero if no errors
      for (c = 0; c < FDarray[b].NumChannels; c++)
      {
        // Process power
        if (FDarray[b].FCD[c].FilamentPwr)
        {
          if((FDarray[b].ADCadr & 0xFE) == 0x10) AD5593write(FDarray[b].ADCadr, 9, 0);
          else digitalWrite(FDarray[b].FCD[c].Fpwr, LOW); // Power on
          // Adjust the current setpoint applying the ramp rate limit, this logic assumes this loop runs 10 times a sec
          StepSize = FDarray[b].FCD[c].RampRate / 10;
          if (abs(FDarray[b].FCD[c].CurrentSetpoint - CurrentSetpoints[b][c]) < StepSize) CurrentSetpoints[b][c] = FDarray[b].FCD[c].CurrentSetpoint;
          else if (FDarray[b].FCD[c].CurrentSetpoint > CurrentSetpoints[b][c]) CurrentSetpoints[b][c] += StepSize;
          else if (FDarray[b].FCD[c].CurrentSetpoint < CurrentSetpoints[b][c]) CurrentSetpoints[b][c] -= StepSize;
          // Ramp the filament voltage to its requested value when the power is on
          if(FDarray[b].FCD[c].FilamentVoltage > FilamentVoltageSetpoints[b][c]) 
          {
            FilamentVoltageSetpoints[b][c] += VstepSize;
            if(FilamentVoltageSetpoints[b][c] > FDarray[b].FCD[c].FilamentVoltage) FilamentVoltageSetpoints[b][c] = FDarray[b].FCD[c].FilamentVoltage;
          }
          else if(FDarray[b].FCD[c].FilamentVoltage < FilamentVoltageSetpoints[b][c]) 
          {
            FilamentVoltageSetpoints[b][c] -= VstepSize;
            if(FilamentVoltageSetpoints[b][c] < 0) FilamentVoltageSetpoints[b][c] = 0;
          }
        }
        else
        {
          // Ramp current to zero and then turn off power
          StepSize = FDarray[b].FCD[c].RampRate / 10;
          CurrentSetpoints[b][c] -= StepSize;
          if(CurrentSetpoints[b][c] <= 0)
          {
             if((FDarray[b].ADCadr & 0xFE) == 0x10) AD5593write(FDarray[b].ADCadr, 9, 1 << FDarray[b].FCD[c].Fpwr);
             else digitalWrite(FDarray[b].FCD[c].Fpwr, HIGH); // Power off
             CurrentSetpoints[b][c] = 0;
             FilamentVoltageSetpoints[b][c] = 0;
          }
        }
        // Output the voltage and current control data to the DAC
        if((FDarray[b].ADCadr & 0xFE) == 0x10)
        {
           AD5593writeDAC(FDarray[b].DACadr, FDarray[b].FCD[c].DCfsuply.Chan, Value2Counts(FilamentVoltageSetpoints[b][c], &FDarray[b].FCD[c].DCfsuply));
           AD5593writeDAC(FDarray[b].DACadr, FDarray[b].FCD[c].Fcurrent.Chan, Value2Counts(CurrentSetpoints[b][c], &FDarray[b].FCD[c].Fcurrent));          
        }
        else
        {
           AD5625(FDarray[b].DACadr, FDarray[b].FCD[c].DCfsuply.Chan, Value2Counts(FilamentVoltageSetpoints[b][c], &FDarray[b].FCD[c].DCfsuply));
           AD5625(FDarray[b].DACadr, FDarray[b].FCD[c].Fcurrent.Chan, Value2Counts(CurrentSetpoints[b][c], &FDarray[b].FCD[c].Fcurrent));
        }
        // Read and filter all the readback and monitor values
        if (adcStatus == 0)
        {
          // If this is rev 3 or 4 then read the emission current per channel
          if((FDarray[b].Rev == 3) || (FDarray[b].Rev == 4))
          {
             float BCfilter = 0.05;
             float Ei = Counts2Value(ADCvals[FDarray[b].Ecurrent.Chan], &FDarray[b].Ecurrent);
             BiasCurrent = BCfilter * Ei + (1-BCfilter) * BiasCurrent;    
          }
          // Filament supply voltage
          FsupplyVsUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].DCfsuplyMon.Chan], &FDarray[b].FCD[c].DCfsuplyMon);
          FsupplyVs[b][c] = Filter * FsupplyVsUF[b][c] + (1 - Filter) * FsupplyVs[b][c];
          // Filament load side voltage
          FvoltagesUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].Fvoltage.Chan], &FDarray[b].FCD[c].Fvoltage);
          Fvoltages[b][c] = Filter * FvoltagesUF[b][c] + (1 - Filter) * Fvoltages[b][c];
          // Filament current
          FcurrentsUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].FcurrentMon.Chan], &FDarray[b].FCD[c].FcurrentMon);
          Fcurrents[b][c] = Filter * FcurrentsUF[b][c] + (1 - Filter) * Fcurrents[b][c];
          #ifdef TestFilament
          Fcurrents[b][c] = CurrentSetpoints[b][c];
          #endif
          // Calculate power dispated in filament
          Fpowers[b][c] = (FsupplyVs[b][c] - Fvoltages[b][c]) * Fcurrents[b][c];
        }
      }
    }
  }
  // Update the display variables for voltage and power
  FsupplyV = FsupplyVs[SelectedFilamentBoard][Channel2Index(SelectedFilamentChan)];
  Fvoltage = Fvoltages[SelectedFilamentBoard][Channel2Index(SelectedFilamentChan)];
  Fcurrent = Fcurrents[SelectedFilamentBoard][Channel2Index(SelectedFilamentChan)];
  Fpower = Fpowers[SelectedFilamentBoard][Channel2Index(SelectedFilamentChan)];
  // Update the display if needed....
  if (ActiveDialog->Entry == FilamentEntriesPage1) RefreshAllDialogEntries(ActiveDialog);
  if (ActiveDialog->Entry == FilamentCycleEntries) RefreshAllDialogEntries(ActiveDialog);
  FilamentControl();
}

//
// The following section contains all the serial command processing functions.
//
// GFLENA,chan        Get filament ON/OFF status
// SFLENA,chan,value  Set filament ON/OFF status
// GFLI,chan          Get filament channel current (setpoint)
// GFLAI,chan         Get filament channel actual current
// SFLI,chan,value    Set filament channel current
// GFLSV,chan         Get filament supply voltage (setpoint)
// GFLASV,chan        Get the actual supply side voltage
// SFLSV,chan,value   Set filament supply voltage
// GFLV,chan          Get filament voltage (actual)
// GFLPWR,chan        Get filament power (actual)

// Returns the number of Filament channels
void FilamentChannels(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfFilamentChannels);
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsFilamentChannelValid(int channel, bool Response = true)
{
  if ((channel >= 1) && (channel <= NumberOfFilamentChannels)) return true;
  if (!Response) return false;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Returns ON or OFF state of the given filament channel number
void GetFilamentEnable(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (SerialMute) return;
  if (FDarray[b].FCD[Channel2Index(channel-1)].FilamentPwr) serial->println("ON");
  else serial->println("OFF");
}

void SetFilamentEnable(char *Chan, char *State)
{
  String res;
  int b, c;

  res = Chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = State;
  if ((res != "ON") && (res != "OFF"))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if (res == "ON") FDarray[b].FCD[Channel2Index(c - 1)].FilamentPwr = true;
  else FDarray[b].FCD[Channel2Index(c - 1)].FilamentPwr = false;
  if ((c - 1) == SelectedFilamentChan) FCD.FilamentPwr = FDarray[b].FCD[Channel2Index(c - 1)].FilamentPwr;
  SendACK;
}

// Returns current setpoint for filament channel number
void GetFilamentCurrent(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCD[Channel2Index(channel - 1)].CurrentSetpoint);
}

// Returns actual current for filament channel number
void GetFilamentActualCurrent(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(Fcurrents[b][Channel2Index(channel - 1)]);
}

void SetFilamentCurrent(char *Chan, char *Current)
{
  String res;
  int b, c;
  float I;

  res = Chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = Current;
  I = res.toFloat();
  if ((I < 0) || (I > MaxFilCur))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCD[Channel2Index(c - 1)].CurrentSetpoint = I;
  if(FDarray[b].Rev >= 2) FDarray[b].FCD[0].CurrentSetpoint = FDarray[b].FCD[0].CurrentSetpoint = I;
  if ((c - 1) == SelectedFilamentChan) FCD.CurrentSetpoint = I;
  SendACK;
}

// Returns supply voltage setpoint for filament channel number
void GetFilamentSupplyVoltage(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCD[Channel2Index(channel - 1)].FilamentVoltage);
}

void SetFilamentSupplyVoltage(char *Chan, char *Voltage)
{
  String res;
  int b, c;
  float V;

  res = Chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = Voltage;
  V = res.toFloat();
  if ((V < .7) || (V > 5))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCD[Channel2Index(c - 1)].FilamentVoltage = V;
  if ((c - 1) == SelectedFilamentChan) FCD.FilamentVoltage = FDarray[b].FCD[Channel2Index(c - 1)].FilamentVoltage;
  if(FDarray[b].Rev >= 2) FDarray[b].FCD[0].FilamentVoltage = FDarray[b].FCD[0].FilamentVoltage = V;  
  SendACK;
}

// Returns supply side actual voltage for filament channel number
void GetFilamentActualSupplyVoltage(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FsupplyVs[b][Channel2Index(channel - 1)]);
}

// Returns load side actual voltage for filament channel number
void GetFilamentVoltage(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(Fvoltages[b][Channel2Index(channel - 1)]);
}

// Returns load side actual power in watts for filament channel number
void GetFilamentPower(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(Fpowers[b][Channel2Index(channel - 1)]);
}

// Returns the current ramp rate in amps per seconds for filament channel number selected
void GetCurrentRampRate(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCD[Channel2Index(channel - 1)].RampRate);
}

// Sets the current ramp rate in amps per seconds for filament channel number selected
void SetCurrentRampRate(char *chan, char *RampRate)
{
  int b, c;
  String res;
  float fval;

  res = chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = RampRate;
  fval = res.toFloat();
  if ((fval < 0.0) || (fval > 1))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCD[Channel2Index(c - 1)].RampRate = fval;
  if(FDarray[b].Rev >= 2) FDarray[b].FCD[0].RampRate = FDarray[b].FCD[1].RampRate = fval;
  if ((c - 1) == SelectedFilamentChan) FCD.RampRate = FDarray[b].FCD[Channel2Index(c - 1)].RampRate;
  SendACK;
}

// This command is only valid for Rev 2 of the firmware driver. This command returns
// the current direction for the seleced channel, FWD or REV.
void GetCerrentDirection(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  if((FDarray[b].Rev < 2) || (FDarray[b].Rev == 4))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  SendACKonly;
  if(SerialMute) return;
  if(FDarray[b].Idir) serial->println("FWD");
  else serial->println("REV");
}

// This command is only valid for Rev 2 of the firmware driver. This command sets
// the current direction for the seleced channel, FWD or REV. The channel must be
// disabled and the current reduced to zero or an error is returned.
void SetCurrentDirection(char *chan, char *dir)
{
  int b, channel;
  String res;
  float fval;

  res = chan;
  channel = res.toInt();
  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  res = dir;
  if((FDarray[b].Rev < 2) || (FDarray[b].Rev == 4) || ((res != "FWD") && (res != "REV")))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  // Make sure the channel is disabled and power supplies are off
  if((FDarray[b].FCD[Channel2Index(channel-1)].FilamentPwr) || (digitalRead(FDarray[b].FCD[0].Fpwr) == LOW) || (digitalRead(FDarray[b].FCD[1].Fpwr) ==  LOW))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  if(res == "FWD") CurrentDir = FDarray[b].Idir = true;
  else  CurrentDir = FDarray[b].Idir = false;
  SendACK;
  // Refresh the screen if the filament dialog is displayed. Also need to update the 
  // active channel buffer
  FCD = FD.FCD[Channel2Index(SelectedFilamentChan)];
  FCY = FD.FCyl[Channel2Index(SelectedFilamentChan)];
  if (ActiveDialog == &FilamentDialog) DialogBoxDisplay(&FilamentDialog);
}

//
// Serial command processing routines for the cycling function are listed below.
//
void GetFilamentCycleCurrent1(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCyl[Channel2Index(channel-1)].FCpoint1);  
}

void SetFilamentCycleCurrent1(char *chan, char *current)
{
  String res;
  int b, c;
  float I;

  res = chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = current;
  I = res.toFloat();
  if ((I < 0) || (I > MaxFilCur))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCyl[Channel2Index(c-1)].FCpoint1 = I;
  if ((c-1) == SelectedFilamentChan) FCY.FCpoint1 = I;
  SendACK;
}

void GetFilamentCycleCurrent2(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCyl[Channel2Index(channel-1)].FCpoint2);  
}

void SetFilamentCycleCurrent2(char *chan, char *current)
{
  String res;
  int b, c;
  float I;

  res = chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = current;
  I = res.toFloat();
  if ((I < 0) || (I > MaxFilCur))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCyl[Channel2Index(c-1)].FCpoint2 = I;
  if ((c-1) == SelectedFilamentChan) FCY.FCpoint2 = I;
  SendACK;
}

void GetFilamentCycleCount(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) serial->println(FDarray[b].FCyl[Channel2Index(channel-1)].FCnumCyl);  
}

void SetFilamentCycleCount(char *chan, char *count)
{
  String res;
  int b, c;
  int cnt;

  res = chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = count;
  cnt = res.toInt();
  if ((cnt < 0) || (cnt > 1000))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FDarray[b].FCyl[Channel2Index(c-1)].FCnumCyl = cnt;
  if ((c-1) == SelectedFilamentChan) FCY.FCnumCyl = cnt;
  SendACK;
}

void GetFilamentStatus(int channel)
{
  int b;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  SendACKonly;
  if (!SerialMute) 
  {
    if(!FDarray[b].FCyl[Channel2Index(channel-1)].FCenable) serial->println("NO");
    else serial->println(FDarray[b].FCyl[Channel2Index(channel-1)].FCnumCyl - FilamentCycleCounts[b][Channel2Index(channel-1)]);
  }
}

void SetFilamentStatus(char *chan, char *Status)
{
  String res;
  int b, c;
  int cnt;

  res = chan;
  c = res.toInt();
  if (!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c-1);
  res = Status;
  if ((!res.equals("NO")) && (!res.equals("YES")))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if(res.equals("NO")) FDarray[b].FCyl[Channel2Index(c-1)].FCenable = false;
  else FDarray[b].FCyl[Channel2Index(c-1)].FCenable = true;
  if ((c) == SelectedFilamentChan) FCY.FCenable = FDarray[b].FCyl[Channel2Index(c-1)].FCenable;
  SendACK;
}

// Set filament reporting function. This command will start a task running at the rate defined.
// If the rate is set to 0 the reporting is stopped.
void Filament_1_Report(void)
{
  int b;

  b=BoardFromSelectedFilamentChannel(0);
  serial->print("FIL1,");
  serial->print(Fcurrents[b][0]);
  serial->print(",");
  serial->print(Fvoltages[b][0]);
  serial->print(",");
  serial->println(Fpowers[b][0]);
}
void Filament_2_Report(void)
{
  int b;

  b=BoardFromSelectedFilamentChannel(1);
  serial->print("FIL2,");
  serial->print(Fcurrents[b][1]);
  serial->print(",");
  serial->print(Fvoltages[b][1]);
  serial->print(",");
  serial->println(Fpowers[b][1]);
}
void Filament_3_Report(void)
{
  int b;

  b=BoardFromSelectedFilamentChannel(2);
  serial->print("FIL3,");
  serial->print(Fcurrents[b][0]);
  serial->print(",");
  serial->print(Fvoltages[b][0]);
  serial->print(",");
  serial->println(Fpowers[b][0]);
}
void Filament_4_Report(void)
{
  int b;

  b=BoardFromSelectedFilamentChannel(3);
  serial->print("FIL4,");
  serial->print(Fcurrents[b][1]);
  serial->print(",");
  serial->print(Fvoltages[b][1]);
  serial->print(",");
  serial->println(Fpowers[b][1]);
}

void SetFilamentReporting(int channel, int period)
{
  int b,c;

  if (!IsFilamentChannelValid(channel)) return;
  b = BoardFromSelectedFilamentChannel(channel-1);
  c = channel - 1;
  if ((period < 0) || (period > 10000))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  switch (c)
  {
    case 0:
      control.remove(&FilamentReportingThreads[c]);
      if(period == 0) break;
      FilamentReportingThreads[c].setName("Fil_1_Rpt");
      FilamentReportingThreads[c].onRun(Filament_1_Report);
      FilamentReportingThreads[c].setInterval(period * 1000);
      control.add(&FilamentReportingThreads[c]);
      break;
    case 1:
      control.remove(&FilamentReportingThreads[c]);
      if(period == 0) break;
      FilamentReportingThreads[c].setName("Fil_2_Rpt");
      FilamentReportingThreads[c].onRun(Filament_2_Report);
      FilamentReportingThreads[c].setInterval(period * 1000);
      control.add(&FilamentReportingThreads[c]);
      break;
    case 2:
      control.remove(&FilamentReportingThreads[c]);
      if(period == 0) break;
      FilamentReportingThreads[c].setName("Fil_3_Rpt");
      FilamentReportingThreads[c].onRun(Filament_3_Report);
      FilamentReportingThreads[c].setInterval(period * 1000);
      control.add(&FilamentReportingThreads[c]);
      break;
    case 3:
      control.remove(&FilamentReportingThreads[c]);
      if(period == 0) break;
      FilamentReportingThreads[c].setName("Fil_4_Rpt");
      FilamentReportingThreads[c].onRun(Filament_4_Report);
      FilamentReportingThreads[c].setInterval(period * 1000);
      control.add(&FilamentReportingThreads[c]);
      break;
    default:
      break;
  }
}

// Report filament bias current
void ReportBiasCurrent(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(BiasCurrent);
}

// This function saves the Filament module data to EEPROM. All detected Filament modules are saved.
void SaveFIL2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<2; b++)
  {
    if(FilamentBoards[b])
    {
      SelectBoard(b);
      if (WriteEEPROM(&FDarray[b], FDarray[b].EEPROMadr, 0, sizeof(FilamentData)) != 0) berr = true;
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
