//
// Filament
//
// This file contains the driver software for the Filament control board. This board has two filament channels that can be independatly set and controlled.
//
// To do list:
//  1.) Add mode logic for following modes
//      a.) Current control mode, this is implemented now
//      b.) Current control mode with voltage control to minimize power disapated in current source MOSFET. done
//      c.) Other modes? constant power?
//  2.) Add limit testing
//      a.) Maximum power, done
//  3.) Add serial commands, done
//
// Gordon Anderson
// July 25, 2015
//
#include <stdarg.h>
#include "wire.h"
#include "SD.h"
#include "string.h"
#include "Serial.h"
#include "Filament.h"
#include "dialog.h"

//MIPS Threads
Thread FilmentThread  = Thread();

#define MaxFilCur 12

#define FD FDarray[SelectedFilamentBoard]

FilamentData  FDarray[2] = {FILAMENT_Rev_1, FILAMENT_Rev_1};

int  NumberOfFilamentChannels = 0;
bool FilamentBoards[2] = {false, false};
int  Fchannel = 1;                 // User selected channel
int  SelectedFilamentChan = 0;     // Active channel
int  SelectedFilamentBoard = 0;    // Active board, 0 or 1 = A or B
FilamentChannel FCD;               // Holds the selected channel's data

// This array is used to enable the setpoint ramp rate. This array contains the actual setpoint
// that is moved at the defined ramp rate in amps/sec for the defined channel.
float CurrentSetpoints[2][2] = {0,0,0,0};

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

extern DialogBoxEntry FilamentEntriesPage2[];

DialogBoxEntry FilamentEntriesPage1[] = {
  {" Channel"            , 0, 1, D_INT  , 1, 2, 1, 21, false, "%2d", &Fchannel, NULL, SelectFilamentChannel},
  {" Enable"             , 0, 2, D_ONOFF, 0, 1,  1,20, false, NULL, &FCD.FilammentPwr, NULL, NULL},
  {" Current"            , 0, 3, D_FLOAT, 0,MaxFilCur,0.01,18, false, "%5.2f", &FCD.CurrentSetpoint, NULL, NULL},
  {" Voltage"            , 0, 4, D_FLOAT, .7, 5,0.1,18, false, "%5.2f", &FCD.FilamentVoltage, NULL, NULL},
  {" Ramp rate"          , 0, 5, D_FLOAT, 0, 1,0.01,18, false, "%5.3f", &FCD.RampRate, NULL, NULL},
  {" Supply V"           , 0, 6, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &FsupplyV, NULL, NULL},
  {" Filament V"         , 0, 7, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &Fvoltage, NULL, NULL},
  {" Filament I"         , 0, 8, D_FLOAT, 0, 0, 0, 18, true, "%5.2f", &Fcurrent, NULL, NULL},
  {" Power"              , 0, 9, D_FLOAT, 0, 0, 0, 18, true, "%5.1f", &Fpower, NULL, NULL},
  {" Next page"          , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesPage2, NULL, NULL},
  {" Return to main menu", 0,11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char *FmodeList = "Ictrl,IVctrl";
char Fmode[8] = "Ictrl";

DialogBoxEntry FilamentEntriesPage2[] = {
  {" Mode"               , 0, 1, D_LIST , 0, 0, 7, 16, false, FmodeList, Fmode, NULL, FmodeChange},
  {" Max power"          , 0, 2, D_FLOAT, 1, 12, 1, 18, false, "%5.0f", &FCD.MaxPower, NULL, NULL},

  {" Cal Supply V"       , 0, 4, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentSupplyV, NULL},
  {" Cal Filament V"     , 0, 5, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentV, NULL},
  {" Cal Filament I"     , 0, 6, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalFilamentI, NULL},
  
  {" Save settings"      , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveFilamentSettings, NULL},
  {" Restore settings"   , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorFilamentSettings, NULL},
  {" First page"         , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, FilamentEntriesPage1, NULL, NULL},
  {" Return to main menu", 0,11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox FilamentDialog = {
  {
    "Filament control params",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0,0, FilamentEntriesPage1
};

MenuEntry MEFilamentModule = {" Filament module", M_DIALOG, 0, 0, 0, NULL, &FilamentDialog, NULL, NULL};

// Calibration functions
void CalFilamentSupplyV(void)
{
  ChannelCal CC;
  char *Name = " Filament Supply Voltage";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 5;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.DCfsuply;
  CC.ADCreadback = &FCD.DCfsuplyMon;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalFilamentV(void)
{
  ChannelCal CC;
  char *Name = "    Filament Voltage";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = 0;
  CC.Max = 5;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.DCfsuply;
  CC.ADCreadback = &FCD.Fvoltage;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalFilamentI(void)
{
  ChannelCal CC;
  char *Name = "    Filament Current";

  SelectBoard(SelectedFilamentBoard);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
//  CC.Min = -2;
//  CC.Max = 4;
  CC.DACaddr = FD.DACadr;
  CC.ADCaddr = FD.ADCadr;
  CC.DACout = &FCD.Fcurrent;
  CC.ADCreadback = &FCD.FcurrentMon;
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, 5);
}

void FmodeChange(void)
{
  String mode;

  mode = Fmode;
  if (mode == "Ictrl")
  {
    FCD.Mode = FmodeI;
    FilamentEntriesPage2[3].NoEdit = false;
  }
  else if (mode == "IVctrl")
  {
    FCD.Mode = FmodeIV;
    FilamentEntriesPage2[3].NoEdit = true;
  }
}

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected
int8_t BoardFromSelectedFilamentChannel(int8_t SC)
{
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

// Called after the user selects a channel
void SelectFilamentChannel(void)
{  
  // Set all the selected channel parameters
  FCD = FD.FCD[Fchannel - 1];
  SelectedFilamentChan = Fchannel - 1;
  SelectedFilamentBoard = BoardFromSelectedFilamentChannel(SelectedFilamentChan);
  
  if (FCD.Mode == FmodeI) strcpy(Fmode, "Ictrl");
  else strcpy(Fmode, "IVctrl");
  // Update the display
  if (ActiveDialog == &FilamentDialog) DialogBoxDisplay(&FilamentDialog);
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
      if(fd.Size > sizeof(FilamentData)) fd.Size = sizeof(FilamentData);
      memcpy(&FD, &fd, fd.Size);
      FCD = FD.FCD[SelectedFilamentChan];
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

// This function is called on power up initilization. This function
// will enable the filament driver module.
void Filament_init(int8_t Board)
{
  // Flag the board as present
  FilamentBoards[Board] = true;
  // Set active board to board being inited
  SelectedFilamentBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the Filament card.
  if (NormalStartup)
  {
    RestorFilamentSettings(true);
  }
  // Init the hardware here...
  pinMode(FDarray[Board].FCD[0].Fpwr,OUTPUT);
  pinMode(FDarray[Board].FCD[1].Fpwr,OUTPUT);
  // Define the initial selected channel as 0 and setup
  Fchannel = 1;
  SelectFilamentChannel();
  // Setup the menu if this is the very first call to this init function
  if (NumberOfFilamentChannels == 0)
  {
    AddMainMenuEntry(&MEFilamentModule);
    if (ActiveDialog == NULL) DialogBoxDisplay(&FilamentDialog);
    // Configure Threads
    FilmentThread.setName("Filiment");
    FilmentThread.onRun(Filament_loop);
    FilmentThread.setInterval(100);
    // Add threads to the controller
    control.add(&FilmentThread);
  }
  NumberOfFilamentChannels += 2;  // Always add two channels for each board
  // Set the maximum number of channels in the selection menu
  FilamentEntriesPage1[0].Max = NumberOfFilamentChannels;
/*  
  serial->println(FDarray[0].FCD[0].DCfsuply.m);
  serial->println(FDarray[0].FCD[0].DCfsuply.b);
  serial->println(FDarray[0].FCD[0].Fcurrent.m);
  serial->println(FDarray[0].FCD[0].Fcurrent.b);
  serial->println(FDarray[0].FCD[0].DCfsuplyMon.m);
  serial->println(FDarray[0].FCD[0].DCfsuplyMon.b);
  serial->println(FDarray[0].FCD[0].Fvoltage.m);
  serial->println(FDarray[0].FCD[0].Fvoltage.b);
  serial->println(FDarray[0].FCD[0].FcurrentMon.m);
  serial->println(FDarray[0].FCD[0].FcurrentMon.b);
*/
}

// This function performs filament control and monitoring tests. The following operations are performed:
//   1.) Maximum power test, if filament power exceeds the maximum then the current setpoint is reduced.
//   2.) If the operating mode is FmodeIV then the voltage across the current source is held at .7 volts
//       by adjusting the filament supply voltage as needed. This minimizes the power disapation in the
//       current source.
void FilamentControl(void)
{
   int        b,c;   // Board and channel number to loop through
   int static skip=0;
  
   if(++skip <= 10) return;
   skip = 0;
   // Loop through all the filament channels
   for(b=0;b<2;b++)
   {
      if(FilamentBoards[b])
      {
         for(c=0;c<2;c++)
         {
            if(FDarray[b].FCD[c].FilammentPwr)
            {
              // Here if power is on
              // Test if power is over the limit
              if(((FsupplyVs[b][c] - Fvoltages[b][c]) * Fcurrents[b][c]) > FDarray[b].FCD[c].MaxPower) FDarray[b].FCD[c].CurrentSetpoint -= 0.1;
              if(FDarray[b].FCD[c].CurrentSetpoint < 0) FDarray[b].FCD[c].CurrentSetpoint = 0;
              // If mode is FmodeIV then set the supply voltage as needed
              if(FDarray[b].FCD[c].Mode == FmodeIV)
              {
                 FDarray[b].FCD[c].FilamentVoltage -= (Fvoltages[b][c] - 0.7);
                 if(FDarray[b].FCD[c].FilamentVoltage > 5) FDarray[b].FCD[c].FilamentVoltage = 5;
                 if(FDarray[b].FCD[c].FilamentVoltage < 0.7) FDarray[b].FCD[c].FilamentVoltage = 0.7;
              }
            }
         }
      }
   }
   FCD = FD.FCD[SelectedFilamentChan];
}

// The process is scheduled by the Filament_init function if the module is detected.
// This is the main Filament run loop that is called every 100 millisec
void Filament_loop(void)
{
   int        b,c;   // Board and channel number to loop through
   uint16_t   ADCvals[8];
   int        adcStatus;
   static int disIndex = 1;
   float StepSize;

   SelectedFilamentBoard = BoardFromSelectedFilamentChannel(SelectedFilamentChan);
   SelectBoard(SelectedFilamentBoard);
   FD.FCD[SelectedFilamentChan] = FCD;    // This stores any changes back to the selected channels data structure
   // Loop through all the filament channels and output all the control parmaeters and update
   // all readback values
   for(b=0;b<2;b++)
   {
      if(FilamentBoards[b])
      {
         SelectBoard(b);
         adcStatus = AD7998(FDarray[b].ADCadr, ADCvals);
         for(c=0;c<2;c++)
         {
           // Process power 
           if(FDarray[b].FCD[c].FilammentPwr) 
           {
              digitalWrite(FDarray[b].FCD[c].Fpwr,LOW);  // Power on
              // Adjust the current setpoint applying the ramp rate limit, this logic assumes this loop runs 10 times a sec
              StepSize = FDarray[b].FCD[c].RampRate/10;
              if(abs(FDarray[b].FCD[c].CurrentSetpoint-CurrentSetpoints[b][c]) < StepSize) CurrentSetpoints[b][c] = FDarray[b].FCD[c].CurrentSetpoint;
              else if(FDarray[b].FCD[c].CurrentSetpoint > CurrentSetpoints[b][c]) CurrentSetpoints[b][c] += StepSize;
              else if(FDarray[b].FCD[c].CurrentSetpoint < CurrentSetpoints[b][c]) CurrentSetpoints[b][c] -= StepSize;
           }
           else 
           {
              digitalWrite(FDarray[b].FCD[c].Fpwr,HIGH);  // Power off
              CurrentSetpoints[b][c] = 0;
           }
           // Output the voltage and current control data to the DAC
           AD5625(FDarray[b].DACadr,FDarray[b].FCD[c].DCfsuply.Chan,Value2Counts(FDarray[b].FCD[c].FilamentVoltage,&FDarray[b].FCD[c].DCfsuply));
           AD5625(FDarray[b].DACadr,FDarray[b].FCD[c].Fcurrent.Chan,Value2Counts(CurrentSetpoints[b][c],&FDarray[b].FCD[c].Fcurrent));
           // Read and filter all the readback and monitor values
           if(adcStatus == 0)
           {
              // Filament supply voltage
              FsupplyVsUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].DCfsuplyMon.Chan], &FDarray[b].FCD[c].DCfsuplyMon);
              FsupplyVs[b][c] = Filter * FsupplyVsUF[b][c] + (1- Filter) * FsupplyVs[b][c];
              // Filament load side voltage
              FvoltagesUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].Fvoltage.Chan], &FDarray[b].FCD[c].Fvoltage);
              Fvoltages[b][c] = Filter * FvoltagesUF[b][c] + (1 - Filter) * Fvoltages[b][c];
              // Filament current
              FcurrentsUF[b][c] = Counts2Value(ADCvals[FDarray[b].FCD[c].FcurrentMon.Chan], &FDarray[b].FCD[c].FcurrentMon);
              Fcurrents[b][c] = Filter * FcurrentsUF[b][c] + (1 - Filter) * Fcurrents[b][c];
              // Calculate power dispated in filament
              Fpowers[b][c] = (FsupplyVs[b][c] - Fvoltages[b][c]) * Fcurrents[b][c];
           }
        }
      }
   }
   // Update the display variables for voltage and power
   FsupplyV = FsupplyVs[SelectedFilamentBoard][SelectedFilamentChan];
   Fvoltage = Fvoltages[SelectedFilamentBoard][SelectedFilamentChan];
   Fcurrent = Fcurrents[SelectedFilamentBoard][SelectedFilamentChan];
   Fpower = Fpowers[SelectedFilamentBoard][SelectedFilamentChan];
   // Update the display if needed....
   if (ActiveDialog->Entry == FilamentEntriesPage1)
   {
      if((ActiveDialog->Selected != disIndex) || (ActiveDialog->State == M_SCROLLING)) DisplayDialogEntry(&ActiveDialog->w, &ActiveDialog->Entry[disIndex], false);
      if(++disIndex > 8) disIndex = 1;
   }
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
  if(!SerialMute) serial->println(NumberOfFilamentChannels);
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsFilamentChannelValid(int channel, bool Response = true)
{
  if ((channel >= 1) && (channel <= NumberOfFilamentChannels)) return true;
  if(!Response) return false;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Returns ON or OFF state of the given filament channel number
void GetFilamentEnable(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(SerialMute) return;
   if(FDarray[b].FCD[channel-1].FilammentPwr) serial->println("ON");
   else serial->println("OFF");
}

void SetFilamentEnable(char *Chan, char *State)
{
  String res;
  int b,c;

  res = Chan;
  c = res.toInt();
  if(!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c);
  res = State;
  if((res != "ON") && (res != "OFF"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
  }
  if(res == "ON") FDarray[b].FCD[c-1].FilammentPwr = true;
  else FDarray[b].FCD[c-1].FilammentPwr = false;
  if ((c - 1) == SelectedFilamentChan) FCD.FilammentPwr = FDarray[b].FCD[c-1].FilammentPwr;
  SendACK;
}

// Returns current setpoint for filament channel number
void GetFilamentCurrent(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(FDarray[b].FCD[channel-1].CurrentSetpoint);
}

// Returns actual current for filament channel number
void GetFilamentActualCurrent(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(Fcurrents[b][channel-1]);
}

void SetFilamentCurrent(char *Chan, char *Current)
{
  String res;
  int b,c;
  float I;
  
  res = Chan;
  c = res.toInt();
  if(!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c);
  res = Current;
  I = res.toFloat();
  if((I < 0) || (I > MaxFilCur))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
  }
  FDarray[b].FCD[c-1].CurrentSetpoint = I;
  if ((c - 1) == SelectedFilamentChan) FCD.CurrentSetpoint = FDarray[b].FCD[c-1].CurrentSetpoint;
  SendACK;
}

// Returns supply voltage setpoint for filament channel number
void GetFilamentSupplyVoltage(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(FDarray[b].FCD[channel-1].FilamentVoltage);
}

void SetFilamentSupplyVoltage(char *Chan, char *Voltage)
{
  String res;
  int b,c;
  float V;
  
  res = Chan;
  c = res.toInt();
  if(!IsFilamentChannelValid(c)) return;
  b = BoardFromSelectedFilamentChannel(c);
  res = Voltage;
  V = res.toFloat();
  if((V < .7) || (V > 5))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
  }
  FDarray[b].FCD[c-1].FilamentVoltage = V;
  if ((c - 1) == SelectedFilamentChan) FCD.FilamentVoltage = FDarray[b].FCD[c-1].FilamentVoltage;
  SendACK;
}

// Returns supply side actual voltage for filament channel number
void GetFilamentActualSupplyVoltage(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(FsupplyVs[b][channel-1]);
}

// Returns load side actual voltage for filament channel number
void GetFilamentVoltage(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(Fvoltages[b][channel-1]);
}

// Returns load side actual power in watts for filament channel number
void GetFilamentPower(int channel)
{
   int b;
   
   if(!IsFilamentChannelValid(channel)) return;
   b = BoardFromSelectedFilamentChannel(channel);
   SendACKonly;
   if(!SerialMute) serial->println(Fpowers[b][channel-1]);
}

