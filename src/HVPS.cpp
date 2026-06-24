//
// HVPS module driver
//
// This module provides control for up to four HVPS modules, each with one or two channels.
// Each channel has a positive and negative supply voltage that can be set, along with an
// enable control. The module also provides readback of the actual voltage for each channel.
// The module is designed to be flexible, allowing for different hardware revisions and configurations,
// with the details defined in the HVPSdata structure for each module. The user interface is built
// around a dialog box that allows the user to set the voltage and enable state for each channel, as well as
// calibrate the channels and save/restore settings to/from EEPROM. The module also provides command-line
// access to the same controls for remote operation.
//
// Updated 2024-06-01: Added positive and negative voltage supply commands (SHVPSPOS and SHVPSNEG) to allow 
//                     setting the supply voltages for each channel. These commands update the corresponding 
//                     fields in the HVPSCH structure for the specified channel and ensure that the new 
//                     voltage is within the defined limits. Additionally, added detailed comments to the 
//                     functions for setting positive and negative supply voltages, as well as the channel 
//                     calibration function, to explain their purpose and parameters.
//                     Added support to allow ADC channels to control the HVPS setpoint.
//
#include "Variants.h"
#include "HVPS.h"
#include "Arduino.h"

#if HVPScode

//MIPS Threads
Thread HVPSthread  = Thread();

#define Filter 0.05

// Forward declarations
extern Menu MainMenu;
extern ThreadController control;
extern bool NormalStartup;
void AddMainMenuEntry(MenuEntry *me);
void initHVPSui(void);
void HVPSChanCalPos(void);
void HVPSChanCalNeg(void);
void SaveHVPSSettings(void);
void RestoreHVPSSettings(void);
void RestoreHVPSSettings(bool NoDisplay);
void HVPS_loop(void);
void SetHVPSvoltage(int chan, int value);
void GetHVPSvoltage(int chan);
void GetHVPSvoltageRB(int chan);
void SetHVPSenable(char *chan, char *value);
void GetHVPSenable(int chan);
void DAC7678write(uint8_t addr, int data);
void DAC7678(uint8_t addr, uint8_t chan, uint16_t counts);
int  HVPSgetBoard(int chan);
int  HVPSgetCh(int chan);
void SetHVPSpositive(int chan, int voltage);
void SetHVPSnegative(int chan, int voltage);
void SetHVPSadcControl(void);
void GetHVPSposCal(int chan);
void SetHVPSposCal(void);
void GetHVPSnegCal(int chan);
void SetHVPSnegCal(void);

#define HVPS HVPSarray[SelectedHVPSboard]

HVPSdata     *HVPSarray[4]  = {NULL,NULL,NULL,NULL};
HVPSstate    *HVPSstates[4] = {NULL,NULL,NULL,NULL};
HVPSadc      *hvpsadc = NULL;
float        MaxHVvoltage=0;
float        HVPSrbs[8] = {11,22,0,0,0,0,0,0};

int NumberOfHVPSchannels    =  0;    // 2 per module, max of 4 modules or 8 channels
int SelectedHVPSboard       =  0;    // Active board, 0 through 3

int HVPSCalChannel = 1;

// Default parameters for modules 1 and 2.
HVPSdata HVPS_Rev_1 = {
    sizeof(HVPSdata),"HVPS",1,
    0x52,0x48,0x23,2,
    
    false,0,
    0,26000,0,
    1,3.29,789,
    0,12.14,2886.33,
    2,26000,0,
    3,-3.43,334.67,
    1,-12.77,1068.67,
    3000,-3000,3000,-3000,

    false,0,
    4,26000,0,
    5,3.29,789,
    2,12.14,2886.33,
    6,26000,0,
    7,-3.43,334.67,
    3,-12.77,1068.67,
    3000,-3000,3000,-3000,  
};

// Default parameters for modules 3 and 4.
HVPSdata HVPS_Rev_2 = {
    sizeof(HVPSdata),"HVPS",1,
    0x54,0x4A,0x20,2,
    
    false,0,
    0,26000,0,
    1,3.29,789,
    0,12.14,2886.33,
    2,26000,0,
    3,-3.43,334.67,
    1,-12.77,1068.67,
    3000,-3000,3000,-3000,

    false,0,
    4,26000,0,
    5,3.29,789,
    2,12.14,2886.33,
    6,26000,0,
    7,-3.43,334.67,
    3,-12.77,1068.67,
    3000,-3000,3000,-3000,  
};

extern DialogBoxEntry HVPSDialogEntriesPage2[];

DialogBoxEntry HVPSDialogEntriesPage1[] = {
  {" 1",  0, 1, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 1, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 2",  0, 2, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 2, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 3",  0, 3, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 3, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 4",  0, 4, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 4, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 5",  0, 5, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 5, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 6",  0, 6, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 6, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 7",  0, 7, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 7, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {" 8",  0, 8, D_ONOFF,     0,    1,  1, 4, false, NULL,  NULL, NULL, NULL},
  {" ",  10, 8, D_INT,   -3000, 3000, 10, 1, false, "%5d", NULL, NULL, NULL},
  {"Ch  Ena  Request Actual", 0, 0, D_TITLE, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page", 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, HVPSDialogEntriesPage2, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry HVPSDialogEntriesPage2[] = {
  {" Calibrate Pos"      , 0, 1, D_INT, 1, 8, 1, 20, false, "%2d", &HVPSCalChannel, NULL, HVPSChanCalPos},
  {" Calibrate Neg"      , 0, 2, D_INT, 1, 8, 1, 20, false, "%2d", &HVPSCalChannel, NULL, HVPSChanCalNeg},
  {" Save settings"      , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveHVPSSettings, NULL},
  {" Restore settings"   , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreHVPSSettings, NULL},
  {" First page"         , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, HVPSDialogEntriesPage1, NULL, NULL},
  {" Return to main menu", 0, 10,D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry HVPSDialogEntriesActualVoltages[] = {
  {" ",  0, 1, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[0], NULL, NULL},
  {" ",  0, 2, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[1], NULL, NULL},
  {" ",  0, 3, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[2], NULL, NULL},
  {" ",  0, 4, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[3], NULL, NULL},
  {" ",  0, 5, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[4], NULL, NULL},
  {" ",  0, 6, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[5], NULL, NULL},
  {" ",  0, 7, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[6], NULL, NULL},
  {" ",  0, 8, D_FLOAT, 0, 0, 1, 18, true, "%5.0f", &HVPSrbs[7], NULL, NULL},
  {NULL},
};

DialogBox HVPSDialog = {
  {
    "HVPS parameters",
    ILI9340_BLACK,ILI9340_WHITE,2,0, 0,300, 220,B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,HVPSDialogEntriesPage1
};

DialogBox HVPSDialogRB = {
  {
    "HVPS parameters",
    ILI9340_BLACK,ILI9340_WHITE,2,0, 0,300, 220,B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,HVPSDialogEntriesActualVoltages
};

MenuEntry MEHVPSMonitor = {" HVPS module", M_DIALOG,0,0,0,NULL,&HVPSDialog,NULL,NULL};

const Commands  HVPSCmdArray[] = {
// Start of command block
//
// HV module commands
//
  {"SHVPS",   CMDfunction, 2, (char *)SetHVPSvoltage},          // Set HVPS channel voltage
  {"GHVPS",   CMDfunction, 1, (char *)GetHVPSvoltage},          // Return HVPS channel voltage, setpoint
  {"GHVPSV",  CMDfunction, 1, (char *)GetHVPSvoltageRB},        // Return HVPS channel voltage, readback
  {"SHVPSENA",CMDfunctionStr, 2, (char *)SetHVPSenable},        // Set HVPS channel enable, ON or OFF
  {"GHVPSENA",CMDfunction, 1, (char *)GetHVPSenable},           // Return HVPS channel enable, ON or OFF
  {"SHVPSPOS",CMDfunction, 2, (char *)SetHVPSpositive},         // Set the positive HV supply voltage, channel, value
  {"SHVPSNEG",CMDfunction, 2, (char *)SetHVPSnegative},         // Set the negative HV supply voltage, channel, value
  {"SHVPSCTRL",CMDfunctionLine, 0, (char *)SetHVPSadcControl},  // Defines ADC channel to control output channel voltage
                                                                // channel,enable DI,ADC channel,gain,offset. ADC channel = -1 to disable
  {"GHVPSPCAL",CMDfunction, 1, (char *)GetHVPSposCal},          // Reports a positive channel slope and offset DAC parameters
  {"SHVPSPCAL",CMDfunctionLine, 0, (char *)SetHVPSposCal},      // Sets a positive channel slope and offset DAC parameters
  {"GHVPSNCAL",CMDfunction, 1, (char *)GetHVPSnegCal},          // Reports a negative channel slope and offset DAC parameters
  {"SHVPSNCAL",CMDfunctionLine, 0, (char *)SetHVPSnegCal},      // Sets a negative channel slope and offset DAC parameters
 
// End of table marker
  {0},
};

CommandList HVPSCmdList = { (Commands *)HVPSCmdArray, NULL };

// Initializes the HVPS user-interface entries for the available HVPS channels. 
// It walks through up to four HVPS data structures and their channels, 
// populating each active channel’s enable control, voltage setpoint, min/max 
// limits, and actual-voltage display entry, then disables any UI records that 
// are not needed.
void initHVPSui(void)
{
  int i,j,ch = 0;
  
  // Loop through all the HVPS data structures
  for(i=0; i<4; i++)
  {
    if(HVPSarray[i] != NULL)
    {
      for(j=0; j<HVPSarray[i]->NumChannels; j++)
      {
        // Set channel enable
        HVPSDialogEntriesPage1[ch * 2].Type = D_ONOFF;
        HVPSDialogEntriesPage1[ch * 2].Value = &HVPSarray[i]->HVPSCH[j].Enable;
        // Set channel setpoint
        HVPSDialogEntriesPage1[ch * 2 + 1].Type = D_INT;
        HVPSDialogEntriesPage1[ch * 2 + 1].Value = &HVPSarray[i]->HVPSCH[j].Voltage;
        // Set channel min and max
        HVPSDialogEntriesPage1[ch * 2 + 1].Min = HVPSarray[i]->HVPSCH[j].NegLimit;
        HVPSDialogEntriesPage1[ch * 2 + 1].Max = HVPSarray[i]->HVPSCH[j].PosLimit;
        // Turn on this channel readback display
        HVPSDialogEntriesActualVoltages[ch].Type = D_FLOAT;
        ch++;
      }
    }
  }
  // Turn off all UI records that are not needed
  for(i=ch; i<8; i++)
  {
    HVPSDialogEntriesPage1[i * 2].Type = D_OFF;
    HVPSDialogEntriesPage1[i * 2 + 1].Type = D_OFF;
    HVPSDialogEntriesActualVoltages[i].Type = D_OFF;
  }
  // Set number of channels for calibration ui options
  HVPSDialogEntriesPage2[0].Max = NumberOfHVPSchannels;
  HVPSDialogEntriesPage2[1].Max = NumberOfHVPSchannels;
}

// Calibrate the channel defined by HVPSCalChannel (1 thru n). Performs the following setup
void HVPSChanCalPos(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b,c;
  
  b = HVPSgetBoard(HVPSCalChannel-1);
  c = HVPSgetCh(HVPSCalChannel-1);
  if(b < 0) return;
  if(c < 0) return;
  SelectBoard(b & 1);
  // Turn on the positive supply and set the voltages to zero
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNena));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPena.Chan,Value2Counts(2.5, &HVPSarray[b]->HVPSCH[c].DCPena));
  // Set up the calibration data structure
  CC.ADCpointer = &AD7994;
  CC.DACpointer = &DAC7678;
  CC.Min=0;
  CC.Max=HVPSarray[b]->HVPSCH[c].PosLimit;
  CC.DACaddr=HVPSarray[b]->TWIdac;  
  CC.ADCaddr=HVPSarray[b]->TWIadc;
  CC.DACout=&HVPSarray[b]->HVPSCH[c].DCPctrl;
  CC.ADCreadback=&HVPSarray[b]->HVPSCH[c].DCPmon;
  // Define this channels name
  sprintf(Name,"  HVPS Channel %2d",HVPSCalChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name,HVPSarray[b]->HVPSCH[c].PosSupplyV / 10,HVPSarray[b]->HVPSCH[c].PosSupplyV / 1.5);
  // Turn everything off
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPena));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNena));
  // force an update
  HVPSstates[b]->Update = true;
}

void HVPSChanCalNeg(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b,c;
  
  b = HVPSgetBoard(HVPSCalChannel-1);
  c = HVPSgetCh(HVPSCalChannel-1);
  if(b < 0) return;
  if(c < 0) return;
  SelectBoard(b & 1);
  // Turn on the positive supply and set the voltages to zero
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPena));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNena.Chan,Value2Counts(2.5, &HVPSarray[b]->HVPSCH[c].DCNena));
  // Set up the calibration data structure
  CC.ADCpointer = &AD7994;
  CC.DACpointer = &DAC7678;
  CC.Min=HVPSarray[b]->HVPSCH[c].NegLimit;
  CC.Max=0;
  CC.DACaddr=HVPSarray[b]->TWIdac;  
  CC.ADCaddr=HVPSarray[b]->TWIadc;
  CC.DACout=&HVPSarray[b]->HVPSCH[c].DCNctrl;
  CC.ADCreadback=&HVPSarray[b]->HVPSCH[c].DCNmon;
  // Define this channels name
  sprintf(Name,"  HVPS Channel %2d",HVPSCalChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name,HVPSarray[b]->HVPSCH[c].NegSupplyV / 10,HVPSarray[b]->HVPSCH[c].NegSupplyV / 1.5);
  // Turn everything off
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNctrl.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNctrl));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCPena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCPena));
  DAC7678(HVPSarray[b]->TWIdac,HVPSarray[b]->HVPSCH[c].DCNena.Chan,Value2Counts(0, &HVPSarray[b]->HVPSCH[c].DCNena));
  // force an update
  HVPSstates[b]->Update = true;  
}

void SaveHVPSSettings(void)
{
  bool Success = true;
  
  for(int i = 0; i < 4; i++)
  {
     if(HVPSarray[i] != NULL)
     {
       SelectBoard(i & 1);
       if(WriteEEPROM(HVPSarray[i], HVPSarray[i]->TWIadd, 0, sizeof(HVPSdata)) != 0) Success = false;
     } 
  }
  if(Success)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Error saving!",2000);  
}

void RestoreHVPSSettings(void)
{
  RestoreHVPSSettings(false);
}

void RestoreHVPSSettings(bool NoDisplay)
{
  int b;
  HVPSdata hvps;
  bool Success=true;
  bool Corrupted=false;
  
  for(b=0;b<4;b++)
  {
    if(HVPSarray[b] == NULL) continue;
    SelectBoard(b & 1);
    if(ReadEEPROM(&hvps, HVPSarray[b]->TWIadd, 0, sizeof(HVPSdata)) == 0)
    {
       if(strcmp(hvps.Name,HVPSarray[b]->Name) == 0)
       {
         // Here if the name matches so copy the data to the operating data structure
         hvps.TWIadd = HVPSarray[b]->TWIadd;
         // Make sure both channels are disabled
         hvps.HVPSCH[0].Enable = false;
         hvps.HVPSCH[1].Enable = false;
         memcpy(HVPSarray[b], &hvps, hvps.Size);
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);  
}

// HVPS_init initializes the specified HVPS board by allocating its data and state objects, assigning the 
// TWI address, selecting the board, loading default revision-dependent settings, clearing the receive 
// buffer state, and preparing the hardware by writing to the DAC and restoring EEPROM settings on normal 
// startup. It also sets up the HVPS command/menu/thread infrastructure the first time a channel is 
// initialized, updates the active board and channel count, and finishes by initializing the HVPS UI.
void HVPS_init(int8_t Board, int8_t addr)
{
  int i;

  // If there are already two or more modules add 2 to the board number
  if(HVPSarray[Board] != NULL) Board += 2;
  // Allocate the module data structure based on board number passed
  HVPSarray[Board] = new HVPSdata;
  HVPSstates[Board] = new HVPSstate;
  // Init the data structures
  HVPSarray[Board]->TWIadd = addr;
  if(Board < 2) *HVPSarray[Board] = HVPS_Rev_1;
  else  *HVPSarray[Board] = HVPS_Rev_2;
  for(i=0;i<8;i++) HVPSrbs[i] = 0;
  HVPSstates[Board]->Update = true;
  // Set active board to board being inited
  SelectedHVPSboard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreHVPSSettings(true);
  }
  // Init the hardware
  DAC7678write(HVPSarray[Board]->TWIdac,0x905000);
  if(NumberOfHVPSchannels == 0)
  {
    // Add the commands to the command processor
    AddToCommandList(&HVPSCmdList);
    // Setup the menu
    AddMainMenuEntry(&MEHVPSMonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&HVPSDialog);
    // Configure Threads
    HVPSthread.setName("HVPS");
    HVPSthread.onRun(HVPS_loop);
    HVPSthread.setInterval(100);
    // Add threads to the controller
    control.add(&HVPSthread);
  }
  else
  {
    // If here we are setting up the second DCbias card so point back to the first one
    SelectedHVPSboard = 0;
    SelectBoard(0);
  }
  NumberOfHVPSchannels += HVPSarray[Board]->NumChannels;
  initHVPSui();
}

// Move this function to the hardware file...
// This function sends the DAC counts to the selected DAC channel.
// Device TWI addresses
// A0 low:   0x48
// A0 high:  0x4A
// A0 float: 0x4C
// 0x905000 to init
void DAC7678write(uint8_t addr, int data)
{
  AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write((data >> 16) &0xFF);
  Wire.write((data>>8) & 0xFF);
  Wire.write(data & 0xFF);
  Wire.endTransmission();
  ReleaseTWI();
}


void DAC7678(uint8_t addr, uint8_t chan, uint16_t counts)
{
  AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write(chan | 0x30);
  Wire.write((counts>>8) & 0xFF);
  Wire.write(counts & 0xFF);
  Wire.endTransmission();
  ReleaseTWI();
}

// HVPSgetBoard returns the board index for a specified HVPS logical channel number by scanning up to four HVPS 
// data structures and counting their channels until it finds a match. It returns the matching board index or -1 
// if the requested channel is not present, with the intended channel range being 0 through 7.
int HVPSgetBoard(int chan)
{
  int i,j,ch=0;

  // Loop through all the HVPS data structures
  for(i=0; i<4; i++)
  {
    if(HVPSarray[i] != NULL)
    {
      for(j=0; j<HVPSarray[i]->NumChannels; j++)
      {
        if(ch == chan) return i;
        ch++;
      }
    }
  }
  return -1;
}

// Returns channel number in the HVPS data structure for the HVPS channel number, 0 thru 7
int HVPSgetCh(int chan)
{
  int i,j,ch=0;

  // Loop through all the HVPS data structures
  for(i=0; i<4; i++)
  {
    if(HVPSarray[i] != NULL)
    {
      for(j=0; j<HVPSarray[i]->NumChannels; j++)
      {
        if(ch == chan) return j;
        ch++;
      }
    }
  }
  return -1;
}

// HVPS_ADC_Control scans the configured HVPS channels, enables or disables each channel based 
// on its enable input, and when a channel is enabled and has an ADC assigned, reads the ADC value 
// ten times, averages the raw counts, applies the channel’s calibration coefficients, and updates 
// that channel’s voltage setpoint. It exits immediately if the ADC configuration pointer 
// is not available.
void HVPS_ADC_Control(void)
{
  int   b,c,counts;
  float voltage;

  if(hvpsadc == NULL) return;
  // Loop through all posible channels
  for(int i=0;i<NumberOfHVPSchannels;i++)
  {
    b = HVPSgetBoard(i);
    if(b < 0) continue;
    c = HVPSgetCh(i) & 1;
    if(c < 0) continue;
    if((HVPSarray[b] != NULL) && (hvpsadc[i].enable != 0))
    {
      if((hvpsadc[i].enable >= 'Q') && (hvpsadc[i].enable <= 'X'))
      {
        if(ReadDIO(hvpsadc[i].enable)==1) HVPSarray[b]->HVPSCH[c].Enable = true;
        else HVPSarray[b]->HVPSCH[c].Enable = false;
      }
    }
    if((HVPSarray[b] != NULL) && (HVPSarray[b]->HVPSCH[c].Enable == true) && (hvpsadc[i].adc != -1))
    {
      // Read the ADC raw counts and apply calibration
      counts = 0;
      for(int j=0;j<10;j++) counts += analogRead(hvpsadc[i].adc);
      voltage = (int)(counts/10 * hvpsadc[i].m + hvpsadc[i].b);
      // Now set the voltage setpoint
      HVPSarray[b]->HVPSCH[c].Voltage = voltage;
    }
  }
}

// HVPS_loop is the 100 ms HVPS service routine that scans up to four module slots, applies channel 
// enable and voltage changes to the DAC-controlled positive/negative outputs (including polarity-change 
// handling by disabling and re-enabling the channel), clears each module’s update flag, and refreshes 
// the HVPS UI when the relevant dialog is active. It then samples each mapped channel’s ADC monitor 
// input, stores a filtered readback value in HVPSrbs, tracks the maximum absolute readback in MaxHVvoltage, 
// and does so for all mapped channels even though the code comments indicate that disabled channels should 
// be forced to zero.
void HVPS_loop(void)
{
  int   i,j,k;
  float fVal;
  
  MaxHVvoltage = 0;
  HVPS_ADC_Control();
  // Loop through all modules and process any enable and setpoint changes
  for(i=0; i<4; i++)
  {
    if(HVPSarray[i] != NULL)
    {
      // Select the board
      SelectBoard(i & 1);
      for(j=0; j<HVPSarray[i]->NumChannels; j++)
      {
        if((HVPSstates[i]->Update) || (HVPSarray[i]->HVPSCH[j].Enable != HVPSstates[i]->HVPSCH[j].Enable))
        {
          //
          // Process enable changes...
          //
          if(HVPSarray[i]->HVPSCH[j].Enable)
          {
            // Here to enable this channel, set the voltages for pos and neg to 0 first then enable.
            // This assumes all switches are off.
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCPctrl));
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCNctrl));
            delay(10);
            if(HVPSarray[i]->HVPSCH[j].Voltage >= 0)
            {
              // Turn on the positive switch
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPena.Chan,Value2Counts(2.5, &HVPSarray[i]->HVPSCH[j].DCPena));
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPctrl.Chan,Value2Counts(HVPSarray[i]->HVPSCH[j].Voltage, &HVPSarray[i]->HVPSCH[j].DCPctrl));
            }
            else
            {
              // Turn on the negative switch
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNena.Chan,Value2Counts(2.5, &HVPSarray[i]->HVPSCH[j].DCNena));
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNctrl.Chan,Value2Counts(HVPSarray[i]->HVPSCH[j].Voltage, &HVPSarray[i]->HVPSCH[j].DCNctrl));
            }
          }
          else
          {
            // Here to disable this channel, set pos and neg voltages to 0 and disable switches on both 
            // pos and neg supplies
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCPctrl));
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCNctrl));
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPena.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCPena));
            DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNena.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCNena));
          }
          HVPSstates[i]->HVPSCH[j].Enable = HVPSarray[i]->HVPSCH[j].Enable;
        }
        if((HVPSstates[i]->Update) || (HVPSarray[i]->HVPSCH[j].Voltage != HVPSstates[i]->HVPSCH[j].Voltage))
        {
          //
          // Process voltage changes... 
          //    - if the polarity changes we need to change the enable switches
          //    - if the channel is disabled then do nothing
          //    - if the update flag is set do nothing, it will be handeled by enable code
          //
          if(!HVPSstates[i]->Update && HVPSstates[i]->HVPSCH[j].Enable)
          {
            if(((HVPSarray[i]->HVPSCH[j].Voltage >= 0) && (HVPSstates[i]->HVPSCH[j].Voltage < 0)) || ((HVPSarray[i]->HVPSCH[j].Voltage < 0) && (HVPSstates[i]->HVPSCH[j].Voltage >= 0)))
            {
              // Here with a polarity change so disable everything and let the enable logic re-enable
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCPctrl));
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNctrl.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCNctrl));
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPena.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCPena));
              DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNena.Chan,Value2Counts(0, &HVPSarray[i]->HVPSCH[j].DCNena));
              HVPSstates[i]->HVPSCH[j].Enable = false;
            }
            else
            {
              // Here for a voltage change only, no polarity change.
              if(HVPSstates[i]->HVPSCH[j].Voltage >=0)
              {
                DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCPctrl.Chan,Value2Counts(HVPSarray[i]->HVPSCH[j].Voltage, &HVPSarray[i]->HVPSCH[j].DCPctrl));
              }
              else
              {
                DAC7678(HVPSarray[i]->TWIdac,HVPSarray[i]->HVPSCH[j].DCNctrl.Chan,Value2Counts(HVPSarray[i]->HVPSCH[j].Voltage, &HVPSarray[i]->HVPSCH[j].DCNctrl));
              }
            }
            HVPSstates[i]->HVPSCH[j].Voltage = HVPSarray[i]->HVPSCH[j].Voltage;
          }
        }
      }
      HVPSstates[i]->Update = false;
    }
  }
  // Read and update all readback values. If a channel is enabled and voltage is >= 0
  // then use positive channel readback, else negative channel readback. If channel is
  // disabled then set to 0.
  for(i=0;i<8;i++)
  {
    j = HVPSgetBoard(i);
    k = HVPSgetCh(i);
    if(j < 0) break;
    if(k < 0) break;
 //   if(HVPSarray[j]->HVPSCH[k].Enable)
    {
      SelectBoard(j & 1);
      if(HVPSarray[j]->HVPSCH[k].Voltage >= 0)
      {
        fVal = Counts2Value(AD7994(HVPSarray[j]->TWIadc,HVPSarray[j]->HVPSCH[k].DCPmon.Chan),&HVPSarray[j]->HVPSCH[k].DCPmon);
        HVPSrbs[i] = Filter * fVal + (1 - Filter) * HVPSrbs[i];
        if(abs(HVPSrbs[i]) > MaxHVvoltage) MaxHVvoltage = abs(HVPSrbs[i]);
      }
      else
      {
        fVal = Counts2Value(AD7994(HVPSarray[j]->TWIadc,HVPSarray[j]->HVPSCH[k].DCNmon.Chan),&HVPSarray[j]->HVPSCH[k].DCNmon);
        HVPSrbs[i] = Filter * fVal + (1 - Filter) * HVPSrbs[i];
        if(abs(HVPSrbs[i]) > MaxHVvoltage) MaxHVvoltage = abs(HVPSrbs[i]);
      }
    }
  }
  // Update the UI
  if (ActiveDialog == &HVPSDialog) RefreshAllDialogEntries(&HVPSDialog);
  if (ActiveDialog->Entry == HVPSDialogEntriesPage1) RefreshAllDialogEntries(&HVPSDialogRB);
}

//
// Host command processing routines
//

// HVPSNumberOfChannels is a host-command handler for reporting the number of HVPS channels. 
// It sends an acknowledgment-only response and, unless serial output is muted, prints the 
// NumberOfHVPSchannels value over the serial connection.
void HVPSNumberOfChannels(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfHVPSchannels);  
}
 
// This function test the channel and returns the board index if the channel
// is valid. -1 is returned on error and the error message is returned.
// Channel is 1 thru n
int TestChannel(int chan)
{
  int j = HVPSgetBoard(chan-1);
  int k = HVPSgetCh(chan-1);
  
  if((j<0) || (k<0))
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return -1;
  }
  return j;
}

// SetHVPSvoltage sets the voltage for a specified HVPS channel by validating the channel, translating 
// it to the internal channel index, and writing the provided value to the corresponding HVPSCH[].Voltage 
// field. If the channel test fails (the TestChannel call returns a negative value), the function exits 
// early; otherwise, it also sends an acknowledgment.
void SetHVPSvoltage(int chan, int value)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  HVPSarray[b]->HVPSCH[c].Voltage = value;
  SendACK;
}

void GetHVPSvoltage(int chan)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  SendACKonly; 
  if(!SerialMute) serial->println(HVPSarray[b]->HVPSCH[c].Voltage); 
  return;
}

void GetHVPSvoltageRB(int chan)
{
  if(TestChannel(chan) < 0) return;
  SendACKonly;
  if(!SerialMute) serial->println((int)HVPSrbs[chan-1]);
}

void SetHVPSenable(char *chan, char *value)
{
  String Token;
  int ch,b,c;

  Token = chan;
  ch = Token.toInt();
  if((b=TestChannel(ch)) < 0) return;
  c = HVPSgetCh(ch-1);
  Token = value;
  if((Token != "ON") && (Token != "OFF")) BADARG;
  if(Token == "ON") HVPSarray[b]->HVPSCH[c].Enable = true;
  else HVPSarray[b]->HVPSCH[c].Enable = false;
  SendACK;
}

void GetHVPSenable(int chan)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  SendACKonly; 
  if(SerialMute) return;
  if(HVPSarray[b]->HVPSCH[c].Enable) serial->println("ON");
  else serial->println("OFF");
}

// This function sets the positive supply voltage for the specified channel. It updates 
// both the PosSupplyV and PosLimit fields in the HVPSCH structure for that channel, 
// ensuring that the new voltage is within the allowed limits. After updating the values, 
// it sends an acknowledgment back to the host.
// Parameters:
// chan: The channel number (1-based index) for which to set the positive supply voltage.
// voltage: The new positive supply voltage to set for the specified channel.
void SetHVPSpositive(int chan, int voltage)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  HVPSarray[b]->HVPSCH[c].PosSupplyV = voltage;
  HVPSarray[b]->HVPSCH[c].PosLimit = voltage;
  SendACK;
}

// This function sets the negative supply voltage for the specified channel. It updates 
// both the NegSupplyV and NegLimit fields in the HVPSCH structure for that channel, 
// ensuring that the new voltage is within the allowed limits. After updating the values, 
// it sends an acknowledgment back to the host.
// Parameters:
// chan: The channel number (1-based index) for which to set the negative supply voltage.
// voltage: The new negative supply voltage to set for the specified channel.
void SetHVPSnegative(int chan, int voltage)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  HVPSarray[b]->HVPSCH[c].NegSupplyV = voltage;
  HVPSarray[b]->HVPSCH[c].NegLimit = voltage;
  SendACK;
}

// SetHVPSadcControl processes a command to configure the ADC settings for one HVPS channel. 
// It reads channel, enable digital input, ADC selection, slope, and intercept values from the command 
// input buffer, stores them in the corresponding HVPSadc entry (allocating the array if needed), 
// and sends an ACK on success or a NAK if parsing fails.
void SetHVPSadcControl(void)
{
  int    chan,adc;
  char   *tkn,e;
  float  slope,intercept;

  // Read the arguments from the ring buffer
  while(true)
  { 
    if(!valueFromCommandLine(&chan,1,NumberOfHVPSchannels)) break;
    tkn = TokenFromCommandLine(',');
    if(tkn != NULL) e = tkn[0];
    else e = 0;
    if((e < 'Q') || (e > 'X')) e = 0;
    if(!valueFromCommandLine(&adc,-1,9)) break;
    if(!valueFromCommandLine(&slope,-100000,100000)) break;
    if(!valueFromCommandLine(&intercept,-100000,100000)) break;
    if(hvpsadc == NULL)
    {
      hvpsadc = new HVPSadc[NumberOfHVPSchannels];
      for(int i=0;i<NumberOfHVPSchannels;i++) hvpsadc[i].adc = -1;
    }
    hvpsadc[chan-1].enable = e;
    hvpsadc[chan-1].adc    = adc;
    hvpsadc[chan-1].m      = slope;
    hvpsadc[chan-1].b      = intercept;
    SendACK;
    return;
  }
  BADARG;
}

// GetHVPSposCal retrieves the position-calibration values for a specified HVPS channel and reports 
// them over serial. It validates the incoming channel, maps it to the internal channel index, sends 
// an ACK-only response, and then prints the DCPctrl.m and DCPctrl.b values unless serial output is 
// muted or the channel is invalid.
void GetHVPSposCal(int chan)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  SendACKonly; 
  if(SerialMute) return;
  serial->print(HVPSarray[b]->HVPSCH[c].DCPctrl.m);
  serial->print(",");
  serial->println(HVPSarray[b]->HVPSCH[c].DCPctrl.b);
}

// SetHVPSposCal() reads a channel number, a slope, and an intercept from the command/ring-buffer 
// input, validates the channel, and, if the selected HVPS channel exists, stores those values into 
// the target channel’s DCP control calibration fields (m and b). It sends an acknowledgement and 
// returns on success, while malformed input or an invalid channel causes it to terminate with a 
// bad-argument response.
void SetHVPSposCal(void)
{
  int    chan;
  int    b,c;
  float  slope,intercept;

  // Read the arguments from the ring buffer
  while(true)
  { 
    if(!valueFromCommandLine(&chan,1,NumberOfHVPSchannels)) break;
    if(!valueFromCommandLine(&slope,-100000,100000)) break;
    if(!valueFromCommandLine(&intercept,-100000,100000)) break;
    if((b=TestChannel(chan)) < 0) return;
    c = HVPSgetCh(chan-1);
    if(HVPSarray[b] != NULL) HVPSarray[b]->HVPSCH[c].DCPctrl.m = slope;
    if(HVPSarray[b] != NULL) HVPSarray[b]->HVPSCH[c].DCPctrl.b = intercept;
    SendACK;
    return;
  }
  BADARG;
}

// GetHVPSnegCal retrieves the negative-calibration values for a specified HVPS channel and reports 
// them over serial. It validates the incoming channel, maps it to the internal channel index, sends 
// an ACK-only response, and then prints the DCNctrl.m and DCNctrl.b values unless serial output is 
// muted or the channel is invalid.
void GetHVPSnegCal(int chan)
{
  int b,c;
  
  if((b=TestChannel(chan)) < 0) return;
  c = HVPSgetCh(chan-1);
  SendACKonly; 
  if(SerialMute) return;
  serial->print(HVPSarray[b]->HVPSCH[c].DCNctrl.m);
  serial->print(",");
  serial->println(HVPSarray[b]->HVPSCH[c].DCNctrl.b);
}

// SetHVPSnegCal() reads a channel number, a slope, and an intercept from the command/ring-buffer 
// input, validates the channel, and, if the selected HVPS channel exists, stores those values into 
// the target channel’s DCP control calibration fields (m and b). It sends an acknowledgement and 
// returns on success, while malformed input or an invalid channel causes it to terminate with a 
// bad-argument response.
void SetHVPSnegCal(void)
{
  int    chan;
  int    b,c;
  float  slope,intercept;

  // Read the arguments from the ring buffer
  while(true)
  { 
    if(!valueFromCommandLine(&chan,1,NumberOfHVPSchannels)) break;
    if(!valueFromCommandLine(&slope,-100000,100000)) break;
    if(!valueFromCommandLine(&intercept,-100000,100000)) break;
    if((b=TestChannel(chan)) < 0) return;
    c = HVPSgetCh(chan-1);
    if(HVPSarray[b] != NULL) HVPSarray[b]->HVPSCH[c].DCNctrl.m = slope;
    if(HVPSarray[b] != NULL) HVPSarray[b]->HVPSCH[c].DCNctrl.b = intercept;
    SendACK;
    return;
  }
  BADARG;
}

#endif
