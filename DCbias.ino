//
// DCbias
//
// This file supports the DC bias modules on the MIPS system. The MIPS controller
// supports up to two DCbias modules.
//
// October 15, 2016. Added the DCbias profile capability. This allow up to 10 voltage
// profiles. The following commands support the voltage profiles.
//  SDCBPRO,num,ch1,ch2,ch3...    // Set a profile, must provide all channels
//  GDCBPRO,num                   // Return a profile
//  ADCBPRO,num                   // Apply a profile
//  CDCBPRO,num                   // Set profile with current values
//  TDCBPRO,p1,p1,dwell           // Toggle between profile p1 and p2, dwell at each profile
//                                // for dwell millisec
//  TDCBSTP                       // Stops the toggling
// Additional commands
//  SDCBALL,ch1,ch2,ch3....       // Set all DC bias channels
//  SDCBOFFENA,chan,val           // Sets the offsetable flag TRUE or FALSE    
//
// Add support for up to 4 DCbias modules in one MIPS system, required the following updates:
//  - Create rev 2 DC bias board templates with new addresses
//  - Dynamically allocate data array space
//  - Add structure for state variables and allocate
//  - Board address points to module array, if only one board it can be board 0 or 1
//  - Dynamically allocate the needed profile array and update all profile code
//
// Added features
//   November 3, 2017
//      1.) Automatically remove the popup trip message when the power supply is reset
//      2.) Add a automaic trip reset feature that will work as follows
//          - If the supply trips wait 5 seconds and reset
//          - If it trips 3 times in 45 seconds then leave it off
//          - Turning on the power supply via UI or command will reset the logic
//          - Enable with a serial command for auto reset
//   February 27, 2018
//      1.) Added support for rev 4.0 50 volt bias board with the AD5593 analog io chip.
//          this chip allow readback of the offset as well as supply voltages. Will up grade all
//          hardware in the future to use this chip. The firmware automatically detects this chip
//          using it TWI address of 0x10 or 0x11.
//      2.) Added host command to return supply voltages monitored with AD5593
//   September 10, 2019
//      1.) Added the rev = 3 functionality for the e-msion filament controller board. The
//          following things are done:
//          - AD5593 set for internal ref
//          - Added rev 3 config for 60 volt board to options.
//            Output channel order is: 4,0,5,1,6,2,7,3
//
// Gordon Anderson
//
#include "DCbias.h"
#include "Variants.h"
#include "Hardware.h"
#include "Table.h"
#include "Errors.h"

extern DialogBox DCbiasDialog;
extern bool NormalStartup;

// DC bias profiles
#define   NumProfiles 10
float     *DCbiasProfiles[NumProfiles];
bool      DCbiasProfileApplied = false;
int       Profile1,Profile2,CurrentProfile;
int       ProfileDwell;
DIhandler *ProfileTrig=NULL;

// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.5         // Weak filter coefficent
#define  StrongFilter 0.05  // Strong filter coefficent

//MIPS Threads
Thread DCbiasThread  = Thread();

int   NumberOfDCChannels = 0;
int   SelectedDCBoard=0;        // Active board, 0 or 1 = A or B
int   CalChannel=1;             // User selected channel to calibrate
float Readback[8];              // Displayed readback buffer
float MaxDCbiasVoltage=0;       // This value is set to the highest DC bias voltage
float Verror = 0;               // Used to detect if the DC bias supply is "stressed"
int   VerrorCh = 0;             // Channel that has readback error
char  mess[25];                 // Message buffer for readback error message
float VerrorFiltered = 0;       // Used to trip supply if error level is exceeded
int   MonitorDelay;             // Delay a number of loop interations before monitoring voltages for tripping PS

bool  DCbiasUpdate = true;          // Flag set to update all DCbias channels
bool  DCbiasTestEnable = true;      // Set false to disable readback testing
uint8_t DCBtstMask[4] = {0,0,0,0};  // Individual channel test masks, set bit to 1 to disable testing on that bits channel
uint8_t DCBchngMask[4]={0,0,0,0};   // Individual channel change masks, set bit to 1 to disable changing the DAC values

#define DCbD DCbDarray[SelectedDCBoard]

DCbiasData  *DCbDarray[4] = {NULL,NULL,NULL,NULL};
DCbiasState *DCbiasStates[4] = {NULL,NULL,NULL,NULL};
DCbiasData dcbd;    // Holds the selected channel's data

// The following variables support automatic reset of a power supply trip
#define   TrippedTries 3         // Number of times to try resetting in 30 secs
bool      AutoReset=false;       // Automatic trip reset enable if true
bool      Tripped=false;         // True when supply trips
uint8_t   TrippedTimes=0;        // Number of times the supply tripped
uint32_t  TrippedTime=0;         // Tripped time
uint32_t  ResetTime=0;           // Time to reset power supply if not 0

// Waveform generation variables

MIPStimer WFMtimer(4);
Waveforms *wfs = NULL;

DialogBoxEntry DCDialogEntriesPage1[] = {
  {" Ch 1, V",  0, 1, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[0].VoltageSetpoint, NULL, NULL},
  {" Ch 2, V",  0, 2, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[1].VoltageSetpoint, NULL, NULL},
  {" Ch 3, V",  0, 3, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[2].VoltageSetpoint, NULL, NULL},
  {" Ch 4, V",  0, 4, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[3].VoltageSetpoint, NULL, NULL},
  {" Ch 5, V",  0, 5, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[4].VoltageSetpoint, NULL, NULL},
  {" Ch 6, V",  0, 6, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[5].VoltageSetpoint, NULL, NULL},
  {" Ch 7, V",  0, 7, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[6].VoltageSetpoint, NULL, NULL},
  {" Ch 8, V",  0, 8, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[7].VoltageSetpoint, NULL, NULL},
  {" Offset V", 0, 9, D_FLOAT, dcbd.MinVoltage, dcbd.MaxVoltage, 1, 11, false, NULL, &dcbd.DCoffset.VoltageSetpoint, NULL, UpdateLimits},
  {"         Request Actual", 0, 0, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page", 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextDCbiasPage, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DCDialogEntriesPage1a[] = {
  {" Ch 9, V",   0, 1, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[0].VoltageSetpoint, NULL, NULL},
  {" Ch 10, V",  0, 2, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[1].VoltageSetpoint, NULL, NULL},
  {" Ch 11, V",  0, 3, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[2].VoltageSetpoint, NULL, NULL},
  {" Ch 12, V",  0, 4, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[3].VoltageSetpoint, NULL, NULL},
  {" Ch 13, V",  0, 5, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[4].VoltageSetpoint, NULL, NULL},
  {" Ch 14, V",  0, 6, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[5].VoltageSetpoint, NULL, NULL},
  {" Ch 15, V",  0, 7, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[6].VoltageSetpoint, NULL, NULL},
  {" Ch 16, V",  0, 8, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[7].VoltageSetpoint, NULL, NULL},
  {" Offset V",  0, 9, D_FLOAT, dcbd.MinVoltage, dcbd.MaxVoltage, 0.1, 11, false, NULL, &dcbd.DCoffset.VoltageSetpoint, NULL, UpdateLimits},
  {"         Request Actual", 0, 0, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page", 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextDCbiasPage, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DCDialogEntriesPage1b[] = {
  {" Ch 17, V",  0, 1, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[0].VoltageSetpoint, NULL, NULL},
  {" Ch 18, V",  0, 2, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[1].VoltageSetpoint, NULL, NULL},
  {" Ch 19, V",  0, 3, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[2].VoltageSetpoint, NULL, NULL},
  {" Ch 20, V",  0, 4, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[3].VoltageSetpoint, NULL, NULL},
  {" Ch 21, V",  0, 5, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[4].VoltageSetpoint, NULL, NULL},
  {" Ch 22, V",  0, 6, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[5].VoltageSetpoint, NULL, NULL},
  {" Ch 23, V",  0, 7, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[6].VoltageSetpoint, NULL, NULL},
  {" Ch 24, V",  0, 8, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[7].VoltageSetpoint, NULL, NULL},
  {" Offset V",  0, 9, D_FLOAT, dcbd.MinVoltage, dcbd.MaxVoltage, 0.1, 11, false, NULL, &dcbd.DCoffset.VoltageSetpoint, NULL, UpdateLimits},
  {"         Request Actual", 0, 0, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page", 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextDCbiasPage, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DCDialogEntriesPage1c[] = {
  {" Ch 25, V",  0, 1, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[0].VoltageSetpoint, NULL, NULL},
  {" Ch 26, V",  0, 2, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[1].VoltageSetpoint, NULL, NULL},
  {" Ch 27, V",  0, 3, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[2].VoltageSetpoint, NULL, NULL},
  {" Ch 28, V",  0, 4, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[3].VoltageSetpoint, NULL, NULL},
  {" Ch 29, V",  0, 5, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[4].VoltageSetpoint, NULL, NULL},
  {" Ch 30, V",  0, 6, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[5].VoltageSetpoint, NULL, NULL},
  {" Ch 31, V",  0, 7, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[6].VoltageSetpoint, NULL, NULL},
  {" Ch 32, V",  0, 8, D_FLOAT, dcbd.MinVoltage+dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage+dcbd.DCoffset.VoltageSetpoint, 1, 11, false, NULL, &dcbd.DCCD[7].VoltageSetpoint, NULL, NULL},
  {" Offset V",  0, 9, D_FLOAT, dcbd.MinVoltage, dcbd.MaxVoltage, 0.1, 11, false, NULL, &dcbd.DCoffset.VoltageSetpoint, NULL, UpdateLimits},
  {"         Request Actual", 0, 0, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page", 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextDCbiasPage, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DCDialogEntriesPage2[] = {
  {" Calibrate channel"  , 0, 1, D_INT, 1, 8, 1, 20, false, "%2d", &CalChannel, NULL, DCbiasChanCal},
  {" Calibrate Offset "  , 0, 2, D_INT, 1, 8, 1, 20, false, "%2d", &CalChannel, DCbiasOffsetInitChan, DCbiasOffsetCal},
  {" Power source "      , 0, 3, D_ONOFF, 0, 1, 1, 20, false, NULL, &MIPSconfigData.PowerEnable, NULL, SetPowerSource},
  {" Save settings"      , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveDCbiasSettings, NULL},
  {" Restore settings"   , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreDCbiasSettings, NULL},
  {" First page"         , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetFirstDCbiasPage, NULL},
  {" Return to main menu", 0, 10,D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DCDialogEntriesActualVoltages[] = {
  {" ",  12, 1, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[0], NULL, NULL},
  {" ",  12, 2, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[1], NULL, NULL},
  {" ",  12, 3, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[2], NULL, NULL},
  {" ",  12, 4, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[3], NULL, NULL},
  {" ",  12, 5, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[4], NULL, NULL},
  {" ",  12, 6, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[5], NULL, NULL},
  {" ",  12, 7, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[6], NULL, NULL},
  {" ",  12, 8, D_FLOAT, 0, 0, 1, 6, true, NULL, &Readback[7], NULL, NULL},
  {NULL},
};

DialogBox DCbiasDialog = {
  {
    "DC bias parameters",
    ILI9340_BLACK,ILI9340_WHITE,2,0, 0,300, 220,B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,DCDialogEntriesPage1
};

MenuEntry MEDCbiasMonitor = {" DC bias module", M_DIALOG,0,0,0,NULL,&DCbiasDialog,NULL,NULL};

// This functions sets up the dialog box entry structure based on the DCbias card range. This funcion will:
// 1.) Define the step size, .1 for board up to 100 volt range and 1 for over 100 volt range.
// 2.) Defines the format string
// 3.) Defines the format string for the readback dialog entries
// It is assumed that entries 0 to 7 are channel 1 through 8 and entry 8 is for offset.
void SetupEntry(DCbiasData *dc, DialogBoxEntry *dbe)
{
  int   i;
  static char *Format1 = "%5.1f";
  static char *Format2 = "%5.0f";
  
  if((dc->MaxVoltage - dc->MinVoltage) <= 140)
  {
    for(i=0;i<=8;i++)
    {
      dbe[i].StepSize = 0.1;
      dbe[i].fmt = Format1;
    }
    for(i=0;i<8;i++) DCDialogEntriesActualVoltages[i].fmt = Format1;
  }
  else
  {
    for(i=0;i<=8;i++)
    {
      dbe[i].StepSize = 1;
      dbe[i].fmt = Format2;
    }
    for(i=0;i<8;i++) DCDialogEntriesActualVoltages[i].fmt = Format2;  
  }
  // Setup the min and max values for the user interface
  for(i=0;i<dc->NumChannels;i++)
  {
    dbe[i].Min = dc->MinVoltage+dc->DCoffset.VoltageSetpoint;
    dbe[i].Max = dc->MaxVoltage+dc->DCoffset.VoltageSetpoint;
  }
  // Enable the voltage entries for the number of defined channels
  for(i=0;i<8;i++)
  {
    if(i < dc->NumChannels) dbe[i].Type = D_FLOAT;
    else dbe[i].Type = D_OFF;
  }
  dbe[8].Min = dc->MinVoltage;
  dbe[8].Max = dc->MaxVoltage;
}

void UpdateLimits(void)
{
  if(DCbiasDialog.Entry == DCDialogEntriesPage1) SetupEntry(&dcbd, DCDialogEntriesPage1);
  if(DCbiasDialog.Entry == DCDialogEntriesPage1a) SetupEntry(&dcbd, DCDialogEntriesPage1a);
  if(DCbiasDialog.Entry == DCDialogEntriesPage1b) SetupEntry(&dcbd, DCDialogEntriesPage1b);
  if(DCbiasDialog.Entry == DCDialogEntriesPage1c) SetupEntry(&dcbd, DCDialogEntriesPage1c);
}

void SetNextDCbiasPage(void)
{
  int i;
  
  // Go to page 1a if we have 16 output channel, 2 boards
  if((DCbiasDialog.Entry == DCDialogEntriesPage1) && (NumberOfDCChannels > 8))
  {
    DCbiasDialog.Entry = DCDialogEntriesPage1a;
    //SelectedDCBoard = 1;
    SelectedDCBoard = DCbiasCH2Brd(8);
    SelectBoard(SelectedDCBoard);
    // Init the dcbd structure
    dcbd = *DCbD;
    // Setup the min and max values in the user interface
    SetupEntry(&dcbd, DCDialogEntriesPage1a);
  }
  else if((DCbiasDialog.Entry == DCDialogEntriesPage1a) && (NumberOfDCChannels > 16))
  {
    DCbiasDialog.Entry = DCDialogEntriesPage1b;
    //SelectedDCBoard = 2;
    SelectedDCBoard = DCbiasCH2Brd(16);
    SelectBoard(SelectedDCBoard);
    // Init the dcbd structure
    dcbd = *DCbD;
    // Setup the min and max values in the user interface
    SetupEntry(&dcbd, DCDialogEntriesPage1b);
  }
  else if((DCbiasDialog.Entry == DCDialogEntriesPage1b) && (NumberOfDCChannels > 24))
  {
    DCbiasDialog.Entry = DCDialogEntriesPage1c;
    //SelectedDCBoard = 3;
    SelectedDCBoard = DCbiasCH2Brd(24);
    SelectBoard(SelectedDCBoard);
    // Init the dcbd structure
    dcbd = *DCbD;
    // Setup the min and max values in the user interface
    SetupEntry(&dcbd, DCDialogEntriesPage1c);
  }
  else DCbiasDialog.Entry = DCDialogEntriesPage2;

  DCbiasDialog.Selected = 0;
  DCbiasDialog.State = M_SCROLLING;
  DialogBoxDisplay(&DCbiasDialog);
}

void SetFirstDCbiasPage(void)
{
  int i;
  
  DCbiasDialog.Entry = DCDialogEntriesPage1;
  // Select the first board
  SelectedDCBoard = DCbiasCH2Brd(0);
//  if(DCbDarray[0] != NULL) SelectedDCBoard=0;
//  else if(DCbDarray[1] != NULL) SelectedDCBoard=1;
  SelectBoard(SelectedDCBoard);
  // Init the dcbd structure
  dcbd = *DCbD;
  // Setup the min and max values in the user interface
  SetupEntry(&dcbd, DCDialogEntriesPage1);
  //
  DCbiasDialog.Selected = 0;
  DCbiasDialog.State = M_SCROLLING;
  DialogBoxDisplay(&DCbiasDialog);
}

// Returns true if the DC bias board is present
bool isDCbiasBoard(int Board)
{
  if(DCbDarray[Board] == NULL) return false;
  return true;
}

// Convert DC bias channel (0 to 31) into board number (0 to 3).
// If no valid board is found -1 is returned.
int DCbiasCH2Brd(int ch)
{
  int  i,j=8;

  if(ch < 0) return(-1);
  for(i=0;i<MAXDCbiasMODULES;i++)
  {
    if(DCbDarray[i] != NULL) 
    {
       if(ch < j) return(i);
       else j += 8;
    }
  }
  return(-1);
}

// Return the channel number 1 thur total channels based on
// board address (0 to 3) and channel index (0 to 7)
int DCBadd2chan(int brd, int ch)
{
  int i, j = 0;
  
  for(i=0;i<MAXDCbiasMODULES;i++)
  {
    if(DCbDarray[i] != NULL) 
    {
      if(i == brd) return(j + ch + 1);
      else j += 8;
    }
  }
  return(0);
}

// This function allows calibration of the seleted channel
void DCbiasChanCal(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  
  if((b=DCbiasCH2Brd(CalChannel-1)) == -1) return;
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b]->MinVoltage;
  CC.Max=DCbDarray[b]->MaxVoltage;
  CC.DACaddr=DCbDarray[b]->DACspi;  
  CC.ADCaddr=DCbDarray[b]->ADCadr;
  CC.DACout=&DCbDarray[b]->DCCD[(CalChannel-1) & 7].DCctrl;
  CC.ADCreadback=&DCbDarray[b]->DCCD[(CalChannel-1) & 7].DCmon;
  // Define this channels name
  sprintf(Name,"       Channel %2d",CalChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  dcbd = *DCbD;
  DCbiasUpdate = true;
}

void DCbiasOffsetInitChan(void)
{
  CalChannel = 1;
}

void DCbiasOffsetCal(void)
{
  ChannelCal CC;
  char       Name[30];
  int        b=0;
  
  if((b=DCbiasCH2Brd(CalChannel-1)) == -1) return;
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b]->MinVoltage;
  CC.Max=DCbDarray[b]->MaxVoltage;
  CC.DACaddr=DCbDarray[b]->DACadr;  
  CC.ADCaddr=DCbDarray[b]->ADCadr;
  CC.DACout=&DCbDarray[b]->DCoffset.DCctrl;
  CC.ADCreadback=NULL;
  if((NumberOfDCChannels > 8) && (DCbDarray[0]->OffsetReadback) && (b == 1)) CC.ADCreadback=&DCbDarray[b]->DCCD[7].DCmon;
  if((NumberOfDCChannels < 8) && (DCbDarray[0]->OffsetReadback) && (b == 0)) CC.ADCreadback=&DCbDarray[b]->DCCD[7].DCmon;
  // If the DACadr is 0x10 or 0x11 then setup for the AD5593 chip
  if((DCbDarray[b]->DACadr & 0xFE) == 0x10)
  {
    // Here if using the AD5593
    CC.ADCpointer = &AD5593readADC;
    CC.ADCaddr=DCbDarray[b]->DACadr;
    CC.ADCreadback=&DCbDarray[b]->DCoffset.DCmon;;
  }
  // Define this channels name
  if(CalChannel <= 8) sprintf(Name," Offset for channel 1-8");
  else sprintf(Name,"Offset for channel 9-16");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  dcbd = *DCbD; 
  DCbiasUpdate = true;     
}

void SaveDCbiasSettings(void)
{
  bool Success = true;
  
  for(int i = 0; i < MAXDCbiasMODULES; i++)
  {
     if(DCbDarray[i] != NULL)
     {
       SelectBoard(i);
       if(WriteEEPROM(DCbDarray[i], DCbDarray[i]->EEPROMadr, 0, sizeof(DCbiasData)) != 0) Success = false;
     } 
  }
  SelectBoard(SelectedDCBoard);
  if(Success)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Error saving!",2000);
}

void RestoreDCbiasSettings(void)
{
  RestoreDCbiasSettings(false);
}

void RestoreDCbiasSettings(bool NoDisplay)
{
  int b;
  DCbiasData dcb_data;
  bool Success=true;
  bool Corrupted=false;
  
  DelayMonitoring();
  for(b=0;b<MAXDCbiasMODULES;b++)
  {
    if(DCbDarray[b] == NULL) continue;
    SelectBoard(b);
    if(ReadEEPROM(&dcb_data, DCbDarray[b]->EEPROMadr, 0, sizeof(DCbiasData)) == 0)
    {
       if(strcmp(dcb_data.Name,DCbDarray[b]->Name) == 0)
       {
         // Here if the name matches so copy the data to the operating data structure
         dcb_data.EEPROMadr = DCbDarray[b]->EEPROMadr;
         memcpy(DCbDarray[b], &dcb_data, dcb_data.Size);  // sizeof(DCbiasData)); 01-27-2020
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  SelectBoard(SelectedDCBoard);
  dcbd = *DCbD;
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
}

// This function converts the value to DAC counts and returns the count value.
// This function is used by the time table (pulse sequence generation) function.
// The channel value range is from 0 to maxchannel-1
int DCbiasValue2Counts(int chan, float value)
{
  int brd;
  
  if((brd=DCbiasCH2Brd(chan)) == -1) return -1;
  chan &= 7;
  return Value2Counts(value - DCbDarray[brd]->DCoffset.VoltageSetpoint,&DCbDarray[brd]->DCCD[chan].DCctrl);
}

// This function converts the channel number to the DAC output channel.
// This function is used by the time table (pulse sequence generation) function.
// The channel value range is from 0 to maxchannel-1
int DCbiasChan2DAC(int chan)
{
  int brd;
  
  if((brd=DCbiasCH2Brd(chan & 31)) == -1) return -1;
  chan &= 7;
  return DCbDarray[brd]->DCCD[chan].DCctrl.Chan;
}

// This function converts the value to DAC counts and return the count value.
// This function is used by the time table (pulse sequence generation) function.
// The channel value range is from 0 to maxchannel-1
float DCbiasCounts2Value(int chan, int counts)
{
  int brd;
  
  if((brd=DCbiasCH2Brd(chan)) == -1) return -1;
  chan &= 7;
  return Counts2Value(counts, &DCbDarray[brd]->DCCD[chan].DCctrl) + DCbDarray[brd]->DCoffset.VoltageSetpoint;
}

// This function sends the DAC counts to the selected channel.
// This function is used by the time table generation code.
// The function writes the DAC register but a LDAC pulse is needed to load the value
// The channel value range is from 0 to maxchannel-1
void DCbiasDACupdate(int chan, int counts)
{
  int brd;

  if(counts == -1) return;
  if((brd=DCbiasCH2Brd(chan)) == -1) return;
  chan &= 7;
  SelectBoard(brd);
  AD5668(DCbDarray[brd]->DACspi,DCbDarray[brd]->DCCD[chan].DCctrl.Chan,counts);
}

// This function will turn on or off the bias supply when called based on the setting of
// PowerEnable.
void SetPowerSource(void)
{
  if(MIPSconfigData.PowerEnable) 
  {
    // Turn power on
    digitalWrite(PWR_ON,LOW);
    // Delay to let hardware stabalize
    noInterrupts();
    // 100 mS delay to let hardware stabalize
    for(int i=0;i<10;i++) delayMicroseconds(10000);
    interrupts();
    // Clear the voltage error variables
    Verror = VerrorFiltered = 0;
    // Delay output alarm monitoring to let things stabalize
    DelayMonitoring();
    LogMessage("DC bias power on");
  }
  else 
  {
    // Turn power off
    digitalWrite(PWR_ON,HIGH);
    LogMessage("DC bias power off");
  }
}

// Returns true if power is on to HV supply
bool IsPowerON(void)
{
  if(--MonitorDelay > 0) return false;
  if(digitalRead(PWR_ON) == 0) return true;
  return false;
}

// Call this function to cause a 1 sec delay on monitoring the outputs for the trip logic. This
// needs to be called when a change is made in the output voltages.
void DelayMonitoring(void)
{
  MonitorDelay = 100;
  VerrorFiltered = 0;  // Reset the error filtered value
}

// This function sets the OffsetOffset value and updates the Offset DAC
void SetOffsetOffset(int brd, float fval)
{
  if(DCbDarray[brd] == NULL) return;
  DCbDarray[brd]->OffsetOffset = fval;
  if(SelectedDCBoard == brd) dcbd.OffsetOffset = fval;  // Sets the UI buffer if this module is selected
  if(digitalRead(PWR_ON) != 0) return;
  AcquireTWI();
  int b = SelectedBoard();
  SelectBoard(brd);
  float V = DCbDarray[brd]->DCoffset.VoltageSetpoint + DCbDarray[brd]->OffsetOffset;
  if(V > DCbDarray[brd]->MaxVoltage) V = DCbDarray[brd]->MaxVoltage;
  if(V < DCbDarray[brd]->MinVoltage) V = DCbDarray[brd]->MinVoltage;
  if((DCbDarray[brd]->DACadr & 0xFE) == 0x10) AD5593writeDAC(DCbDarray[brd]->DACadr,DCbDarray[brd]->DCoffset.DCctrl.Chan,Value2Counts(V,&DCbDarray[brd]->DCoffset.DCctrl));
  else AD5625(DCbDarray[brd]->DACadr,DCbDarray[brd]->DCoffset.DCctrl.Chan,Value2Counts(V,&DCbDarray[brd]->DCoffset.DCctrl),3);
  if(b != brd) SelectBoard(b);
  ReleaseTWI();
}

// This function sets the Channel offset value and updates the output DACs
void SetChannelOffset(int brd, float fval)
{
  float delta;
  
  if(DCbDarray[brd] == NULL) return;
  delta = fval - DCbDarray[brd]->ChannelOffset;
  DCbDarray[brd]->ChannelOffset = fval;
  if(SelectedDCBoard == brd) dcbd.ChannelOffset = fval;  // Sets the UI buffer if this module is seleted
  if(digitalRead(PWR_ON) != 0) return;
  AcquireTWI();
  int b = SelectedBoard();
  SelectBoard(brd);
  for(int i=0;i<DCbDarray[brd]->NumChannels;i++)
  {
    if((DCbDarray[brd]->OffsetChanMsk & (1 << i)) != 0)
    {
      if(SelectedDCBoard == brd) DCbiasStates[brd]->Readbacks[i] += delta;
      // If here update the DAC channel
      float V = DCbDarray[brd]->DCCD[i].VoltageSetpoint - DCbDarray[brd]->DCoffset.VoltageSetpoint + DCbDarray[brd]->ChannelOffset;
      if(V > DCbDarray[brd]->MaxVoltage) {V = DCbDarray[brd]->MaxVoltage; DCbDarray[brd]->DCCD[i].VoltageSetpoint = V + DCbDarray[brd]->DCoffset.VoltageSetpoint; }
      if(V < DCbDarray[brd]->MinVoltage) {V = DCbDarray[brd]->MinVoltage; DCbDarray[brd]->DCCD[i].VoltageSetpoint = V + DCbDarray[brd]->DCoffset.VoltageSetpoint; }
      if((DCBchngMask[brd]&(1<<i))==0) AD5668(DCbDarray[brd]->DACspi,DCbDarray[brd]->DCCD[i].DCctrl.Chan,Value2Counts(V,&DCbDarray[brd]->DCCD[i].DCctrl),3);
    }
  }
  if(b != brd) SelectBoard(b);
  MonitorDelay += 1;  // think about this a bit
  ReleaseTWI();
}

// Init the AD5593 (Analog and digital IO chip) for the DCbias module. The following 
// setup requirements:
// CH0 = DAC out, offset control
// CH1 = ADC in, offset readback
// CH2 = ADC in, positive HV
// CH3 = ADC in, negative HV
// CH4 = ADC in, 3.3V logic supply
// External 2.5V reference with 0 to 2.5V range
// No pullups
void DCbiasAD5593init(int8_t addr)
{
   // General purpose configuration
   AD5593write(addr, 3, 0x0100);
   // Set reference
   if(DCbD->Rev == 3) AD5593write(addr, 11, 0x0200);     // Set int reference
   else AD5593write(addr, 11, 0x0000);     // Set ext reference
   // Set LDAC mode
   AD5593write(addr, 7, 0x0000);
   // Set DAC outputs channels
   AD5593write(addr, 5, 0x0001);
   // Init DAC channel 0 to mid range
   AD5593writeDAC(addr, 0, 32767);
   // Set ADC input channels
   AD5593write(addr, 4, 0x001E);
   // Turn off all pulldowns
   AD5593write(addr, 6, 0x0000);   
}

// This function is called at powerup to initiaize the DC bias board(s).
void DCbias_init(int8_t Board, int8_t addr)
{
  int i;

  // On first call set the profile pointers to NULL
  if(NumberOfDCChannels == 0) for(i=0;i<NumProfiles;i++) DCbiasProfiles[i] = NULL;
  // If there are already two or more modules add 2 to the board number
//  if(NumberOfDCChannels >= 16) Board += 2;
  if(DCbDarray[Board] != NULL) Board += 2;
  // Allocate the module data structure based on board number passed
  DCbDarray[Board] = new DCbiasData;
  DCbiasStates[Board] = new DCbiasState;
  // Init the data structures
  if(Board < 2) *DCbDarray[Board] = DCbD_250_Rev_1;
  else  *DCbDarray[Board] = DCbD_250_Rev_2;
  for(i=0;i<8;i++)
  {
    DCbiasStates[Board]->Readbacks[i] = 0.0;
    DCbiasStates[Board]->DCbiasV[i] = 0.0;
  }
  DCbiasStates[Board]->DCbiasO = 0.0;
  // Set active board to board being inited
  SelectedDCBoard = Board;
  SelectBoard(Board);
  // Init the dcbd structure
  DCbD->EEPROMadr = addr;
  dcbd = *DCbD;
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreDCbiasSettings(true);
    *DCbD = dcbd;        // Copy back into the configuration data structure array
  }
  // If the DAC TWI address is 0x10 or 0x11 then the device is a AD5593.
  // Setup as needed
  if((DCbDarray[Board]->DACadr & 0xFE) == 0x10)
  {
    DCbiasAD5593init(DCbDarray[Board]->DACadr);
  }
  if(NumberOfDCChannels == 0)
  {
    // Setup the min and max values in the user interface
    SetupEntry(&dcbd, DCDialogEntriesPage1);
    // If this board is not offsetable then disable the menu option and set the offset value to 0
    if(!DCbD->Offsetable)
    {
      DCbD->DCoffset.VoltageSetpoint = 0;
      DCDialogEntriesPage1[8].Type = D_OFF;
    }
    // Setup the menu
    AddMainMenuEntry(&MEDCbiasMonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&DCbiasDialog);
    // Configure Threads
    DCbiasThread.setName("DCbias");
    DCbiasThread.onRun(DCbias_loop);
    DCbiasThread.setInterval(100);
    // Add threads to the controller
    control.add(&DCbiasThread);
  }
  else
  {
    if(!DCbD->Offsetable)
    {
      DCbD->DCoffset.VoltageSetpoint = 0;
      DCDialogEntriesPage1a[8].Type = D_OFF;
    }    
    // If here we are setting up the second DCbias card so point back to the first one
    SelectedDCBoard = 0;
    SelectBoard(0);
    dcbd = *DCbD;
  }
  NumberOfDCChannels += 8;
  DCDialogEntriesPage2[0].Max = NumberOfDCChannels;
  DCDialogEntriesPage2[1].Max = NumberOfDCChannels;
  #if DCBanalog
  dcbctrlEnable(NumberOfDCChannels - 7);
  #endif
  #if DCBcurrent
  dcbcurEnable(NumberOfDCChannels - 7);
  #endif
}

// This function is called every 100 mS to process the DC bias board(s).
// Only update the DACs when the values have changed.
void DCbias_loop(void)
{
  static  bool  inited = false;
  float   errorPercentage,V,flt;
  static  float offsetV;
  static  float filteredOffset[4] = {0,0,0,0};
  static  int disIndex = 0;
  static  bool SuppliesOff = true;
  static  bool LastSuppliesState = true;
  static  int  SuppliesStableCount = 10;
  int     i,b;
  uint16_t ADCvals[8];

  if(!inited) SetPowerSource();
  inited = true;
  // Monitor power on output bit. If power comes on, hold all outputs at zero until power is stable, short delay.
  if(digitalRead(PWR_ON) != 0) 
  {
    // Here if power is off, set supply off flag and delay loop counter
    SuppliesOff = true;
    SuppliesStableCount = 10;
  }
  else
  {
    // Here when supplies are on
    if(--SuppliesStableCount <= 0) SuppliesOff = false;
  }
  // Copy any UI updates, first look to see if any setpoints have changed, if so delay monitoring
  for(i=0;i<DCbD->NumChannels;i++) if(abs(DCbD->DCCD[i].VoltageSetpoint - dcbd.DCCD[i].VoltageSetpoint) > 1.0) DelayMonitoring();
  if(abs(DCbD->DCoffset.VoltageSetpoint - dcbd.DCoffset.VoltageSetpoint) > 1.0) DelayMonitoring();
  if(ActiveDialog == &DCbiasDialog)
  {
    *DCbD = dcbd;   // This writes any changes to the DCbias array  
  } 
  #if FAIMScode
    *DCbD = dcbd;
  #endif
  //if(DCbDarray[0]->UseOneOffset) DCbDarray[SelectedDCBoard ^ 1]->DCoffset.VoltageSetpoint = DCbDarray[SelectedDCBoard]->DCoffset.VoltageSetpoint;
  for(i=0;i<MAXDCbiasMODULES;i++)
  {
     if(DCbDarray[0]->UseOneOffset) if(DCbDarray[i] != 0) DCbDarray[i]->DCoffset.VoltageSetpoint = DCbDarray[SelectedDCBoard]->DCoffset.VoltageSetpoint;    
  }
  MaxDCbiasVoltage = 0;
  Verror = 0;
  // This logic was add in version 1.150. This code will ramp the voltages back up after
  // the power supply is enabled. The goal it to stop any big voltage impluses. This code
  // requires 100mS to ramp up and this only happen when the supply state changes from off
  // to on.
  for(int StepUp = 0; StepUp < 100; StepUp++)
  {
    float Mult;
    if(!SuppliesOff && !LastSuppliesState) Mult = 1.0;
    else Mult = (float)(StepUp+1)/100.0; 
    if(SuppliesOff) Mult = 1.0;
    for(b=0;b<MAXDCbiasMODULES;b++)
    {
      if(DCbDarray[b] == NULL) continue;
      SelectBoard(b);
      // Update the offset output, its TWI not SPI!
      if(SuppliesOff == false)
      {
        if((DCbDarray[b]->DCoffset.VoltageSetpoint != DCbiasStates[b]->DCbiasO) || DCbiasUpdate)
        {
          DCbiasStates[b]->DCbiasO = DCbDarray[b]->DCoffset.VoltageSetpoint * Mult;
          V = DCbDarray[b]->DCoffset.VoltageSetpoint * Mult + DCbDarray[b]->OffsetOffset * Mult;
          if(V > DCbDarray[b]->MaxVoltage) V = DCbDarray[b]->MaxVoltage;
          if(V < DCbDarray[b]->MinVoltage) V = DCbDarray[b]->MinVoltage;
          if((DCbDarray[b]->DACadr & 0xFE) == 0x10) AD5593writeDAC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCctrl.Chan,Value2Counts(V,&DCbDarray[b]->DCoffset.DCctrl));
          else AD5625(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCctrl.Chan,Value2Counts(V,&DCbDarray[b]->DCoffset.DCctrl),3);
        }
      }
      else
      {
        // Set to zero if power is off
        DCbiasStates[b]->DCbiasO = 0;
        if((DCbDarray[b]->DACadr & 0xFE) == 0x10) AD5593writeDAC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCctrl.Chan,Value2Counts(0,&DCbDarray[b]->DCoffset.DCctrl));
        else AD5625(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCctrl.Chan,Value2Counts(0,&DCbDarray[b]->DCoffset.DCctrl),3);      
      }
      // Update all output channels. SPI interface for speed
      if(DCbiasUpdate) DelayMonitoring();
      for(i=0;i<DCbDarray[b]->NumChannels;i++)
      {
        if(SuppliesOff == false)
        {
          V = DCbDarray[b]->DCCD[i].VoltageSetpoint*Mult - DCbDarray[b]->DCoffset.VoltageSetpoint*Mult;
          if((DCbDarray[b]->OffsetChanMsk & (1 << i)) != 0) V += DCbDarray[b]->ChannelOffset * Mult;
          if(V > DCbDarray[b]->MaxVoltage) {V = DCbDarray[b]->MaxVoltage; DCbDarray[b]->DCCD[i].VoltageSetpoint = V + DCbDarray[b]->DCoffset.VoltageSetpoint*Mult; }
          if(V < DCbDarray[b]->MinVoltage) {V = DCbDarray[b]->MinVoltage; DCbDarray[b]->DCCD[i].VoltageSetpoint = V + DCbDarray[b]->DCoffset.VoltageSetpoint*Mult; }
          //DCbDarray[b]->DCCD[i].VoltageSetpoint = V + DCbDarray[b]->DCoffset.VoltageSetpoint*Mult;
          if((V != DCbiasStates[b]->DCbiasV[i]) || DCbiasUpdate)
          {
            DCbiasStates[b]->DCbiasV[i] = V;
            if(!DCbiasProfileApplied) if((DCBchngMask[b]&(1<<i))==0) AD5668(DCbDarray[b]->DACspi,DCbDarray[b]->DCCD[i].DCctrl.Chan,Value2Counts(V,&DCbDarray[b]->DCCD[i].DCctrl),3);
          }
        }
        else
        {
          // Set to zero if power is off
          if((DCBchngMask[b]&(1<<i))==0) AD5668(DCbDarray[b]->DACspi,DCbDarray[b]->DCCD[i].DCctrl.Chan,Value2Counts(0,&DCbDarray[b]->DCCD[i].DCctrl), 3);
          DCbiasStates[b]->DCbiasV[i] = 0;
        }
      }
    }
    if(LastSuppliesState == SuppliesOff) break;
    if(SuppliesOff) break;
    //delay(1); 
  }
  LastSuppliesState = SuppliesOff;
  // End of version 1.150 rampup updates  
  for(b=0;b<4;b++)
  {
    if(DCbDarray[b] == NULL) continue;
    SelectBoard(b);
    // Read the monitor inputs and update the display buffer
    ValueChange = false;
    //delay(1);
    if(AD7998(DCbDarray[b]->ADCadr, ADCvals)!=0) LogMessage("ADC read fault!"); 
    else for(i=0;i<DCbDarray[b]->NumChannels;i++)
    {
      if(!ValueChange)
      {
         if((DCbDarray[0] != NULL) && (DCbDarray[0]->OffsetReadback) && (i == 0))   // Added i==0 April 6, 2019.
         {
           if((DCbDarray[b]->DACadr & 0xFE) == 0x10)
           {
              int ov=0;
              if(DCbDarray[0]->UseOneOffset)
              {
                // If we are using one offset for all board and offset readback is enabled then we assume the
                // last module is providing the readback. This code was updated to support running all modules
                // on the same board address. 5/23/24
                if(NumberOfDCChannels > 24)      { if(b == DCbiasCH2Brd(24)) offsetV = Counts2Value(ov =AD5593readADC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCmon.Chan),&DCbDarray[b]->DCoffset.DCmon); }
                else if(NumberOfDCChannels > 16) { if(b == DCbiasCH2Brd(16)) offsetV = Counts2Value(ov =AD5593readADC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCmon.Chan),&DCbDarray[b]->DCoffset.DCmon); }
                else if(NumberOfDCChannels > 8)  { if(b == DCbiasCH2Brd(8))  offsetV = Counts2Value(ov =AD5593readADC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCmon.Chan),&DCbDarray[b]->DCoffset.DCmon); }
                else if(NumberOfDCChannels <= 8) { if(b == DCbiasCH2Brd(0))  offsetV = Counts2Value(ov =AD5593readADC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCmon.Chan),&DCbDarray[b]->DCoffset.DCmon); }
              }
              else offsetV = Counts2Value(ov =AD5593readADC(DCbDarray[b]->DACadr,DCbDarray[b]->DCoffset.DCmon.Chan),&DCbDarray[b]->DCoffset.DCmon);
              if(ov==-1) 
              {
                offsetV = DCbDarray[b]->DCoffset.VoltageSetpoint;
                LogMessage("ADC readoff fault!");
              }
           }
           else if(DCbDarray[0]->UseOneOffset)
           {
              // This is legacy code, if offset readback is enabled and DAC is not a AD5593 then offset readback is
              // assumed to use channel 8 of the last module. The boards have to have a jummper from offset amp
              // monitor to ch8 monitor. Not, the offset variable is static and must be for this to work.
              if((NumberOfDCChannels > 8) && (b == 1)) offsetV = Counts2Value(ADCvals[7],&DCbDarray[b]->DCCD[7].DCmon); 
              else if((NumberOfDCChannels < 8) && (b == 0)) offsetV = Counts2Value(ADCvals[7],&DCbDarray[b]->DCCD[7].DCmon); 
           }
         }
         else if(i==0) offsetV = DCbDarray[b]->DCoffset.VoltageSetpoint;
         if(SuppliesOff) offsetV = 0;
         //DCbiasStates[b]->Readbacks[i] = Filter * (Counts2Value(ADCvals[DCbDarray[b]->DCCD[i].DCmon.Chan],&DCbDarray[b]->DCCD[i].DCmon) + offsetV) + (1-Filter) * DCbiasStates[b]->Readbacks[i];
         // 9/7/23, increased the filtering to improve the readbacks stability
         // 10/8/23, updated this code, it caused readback failures on systems with multiple DCbias module. This code makes it dynamic, low filter
         // value for big differences
         // Updated 8/4/2024 to test offset seperate from the channels
         if(i == 0)
         {
            flt = 0.5;
            if(abs(DCbDarray[b]->DCoffset.VoltageSetpoint - offsetV) < 2) flt = 0.1;
            filteredOffset[b] = flt * offsetV + (1-flt) * filteredOffset[b];
         }
         flt = 0.5;
         V = Counts2Value(ADCvals[DCbDarray[b]->DCCD[i].DCmon.Chan],&DCbDarray[b]->DCCD[i].DCmon) + offsetV;
//       V = Counts2Value(ADCvals[DCbDarray[b]->DCCD[i].DCmon.Chan],&DCbDarray[b]->DCCD[i].DCmon) + DCbDarray[b]->DCoffset.VoltageSetpoint;
         if(abs(V - DCbiasStates[b]->Readbacks[i]) < 2) flt = 0.1;
         DCbiasStates[b]->Readbacks[i] = flt * V + (1-flt) * DCbiasStates[b]->Readbacks[i];
         if(abs(DCbiasStates[b]->Readbacks[i]) > MaxDCbiasVoltage) MaxDCbiasVoltage = abs(DCbiasStates[b]->Readbacks[i]);
         if(b == SelectedDCBoard) Readback[i] = DCbiasStates[b]->Readbacks[i];
      }
    }
    // Determine the largest error between the output setpoint and the actual value, scan all channels
    // only do this test if the power supply is on and its a 250 volt or higher board
    if(IsPowerON() && (DCbDarray[b]->MaxVoltage >= 200)) for(i=0;i<DCbDarray[b]->NumChannels;i++)
    {
      if((TableMode == LOC) && (DCbiasTestEnable))
      {
        V = 0;
        if((DCbDarray[b]->OffsetChanMsk & (1 << i)) != 0) V = DCbDarray[b]->ChannelOffset;
        errorPercentage = (abs(DCbiasStates[b]->Readbacks[i] - DCbDarray[b]->DCCD[i].VoltageSetpoint - V) / DCbDarray[b]->MaxVoltage) * 100.0;
        if((errorPercentage > Verror) && ((DCBtstMask[b] & (1 << i)) == 0)) { Verror = errorPercentage; VerrorCh = DCBadd2chan(b,i); }
        if(i == 0)
        {
          errorPercentage = (abs(filteredOffset[b] - DCbDarray[b]->DCoffset.VoltageSetpoint) / DCbDarray[b]->MaxVoltage) * 100.0;
          if(errorPercentage > Verror) { Verror = errorPercentage; VerrorCh = -(b + 1); }
        }
      } 
    }
  }
  DCbiasProfileApplied = false;
  DCbiasUpdate = false;
  dcbd = *DCbD;
  VerrorFiltered = StrongFilter * Verror + (1-StrongFilter) * VerrorFiltered;
  // If the VerrorFiltered value exceeds the threshold then turn off the DC bias power supply and popup a message
  if(IsPowerON())
  {
    if(Tripped)
    {
      // If here then the power supply tripped and was reset
      DismissMessageIfButton();     // Dismiss the popup message
      Tripped = false;
    }
    // Test the threshold
    if((VerrorFiltered > MIPSconfigData.VerrorThreshold) && (MIPSconfigData.VerrorThreshold > 0))
    {
      // Turn off the power supply
      MIPSconfigData.PowerEnable = false;
      SetPowerSource();
      // Display a popup error message
      if(VerrorCh >= 0) sprintf(mess,"Output Verror, ch %1d",VerrorCh);
      else sprintf(mess,"Offset Verror, brd %1d",(-VerrorCh) - 1);
      LogMessage("DCBias Verror!");
      DisplayMessageButtonDismiss(mess);
      Tripped = true;
      // Disable all ESI channels if mode flag is set
      #ifdef DCBtripsESI
        ESIarray[0].ESIchan[0].Enable = ESIarray[0].ESIchan[1].Enable = false; 
        ESIarray[0].Enable = ESIarray[0].Enable = esi.Enable = false;
        ESIarray[1].ESIchan[0].Enable = ESIarray[1].ESIchan[1].Enable = false; 
        esi.ESIchan[0].Enable = esi.ESIchan[1].Enable = false;
        esich.Enable = false;
      #endif
      if(AutoReset)
      {
        // Here if auto reset is enabled
        if(millis() > (TrippedTime + 45000))  // If it has not tripped in last 30 seconds then reset the number of tries
        {
          TrippedTime = millis();
          TrippedTimes = 0;
        }
        if(TrippedTimes++ < TrippedTries) ResetTime = millis() + 5000;
      }
    }
  }
  if(ResetTime != 0)
  {
    if(millis() > ResetTime)
    {
      ResetTime = 0;
      MIPSconfigData.PowerEnable = true;
      SetPowerSource();
    }
  }
  SelectBoard(SelectedDCBoard);
  // Display update the readback values update
  if(ActiveDialog != &DCbiasDialog) return;
  if(DCbiasDialog.Entry == DCDialogEntriesPage2) return;
  if(DCbiasDialog.LastUpdated == 0)
  {
    if(disIndex < dcbd.NumChannels) DisplayDialogEntry(&DCbiasDialog.w, &DCDialogEntriesActualVoltages[disIndex], false);
    if(++disIndex >= 8) 
    {
      disIndex=0;
      RefreshAllDialogEntries(&DCbiasDialog);
    }
  }
  else RefreshAllDialogEntries(&DCbiasDialog);
}

//
// This section contains all the DCbias command processing routines.
//

// This function returns the board number for the channel number passed in.
// chan range is 1 to NumberOfDCChannels
// returns - 1 on error
int GetDCbiasBoard(int chan, bool Response = true)
{
  int brd=DCbiasCH2Brd(chan-1);
  if((chan < 1) || (chan > NumberOfDCChannels) || (brd == -1))
  {
    if(Response)
    {
      SetErrorCode(ERR_INVALIDCHAN);
      SendNAK;
    }
    return(-1);
  }
  return(brd);
}

// This function determines the data structure that holds the data for the channel number passed.
// NULL is returned if it can't be found.
// chan is in the range of 1 to 32
// If there is a channel error this function will send the NAK
// out the serial port.
DCbiasData *GetDCbiasDataPtr(int chan, bool Response = true)
{
  int board;
  
  board = GetDCbiasBoard(chan,false);
  if(board == -1)
  {
    if(Response)
    {
      SetErrorCode(ERR_INVALIDCHAN);
      SendNAK;
    }
    return NULL;
  }
  return DCbDarray[board];
}

// This function returns true if the channel number is valid.
// chan is in the range 1 to 16.
// If there is a channel error this function will send the NAK
// out the serial port.
bool CheckChannel(int chan, bool Response = true)
{
  if((chan >= 1) && (chan <= NumberOfDCChannels)) return true;
  if(!Response) return false;
  SetErrorCode(ERR_INVALIDCHAN);
  SendNAK;
  return false;
}

// This function tests the value to make sure its in range
bool CheckValue(DCbiasData *dc, float value, bool Response = true)
{
  if((value > dc->MaxVoltage) || (value < dc->MinVoltage))
  {
    if(!Response) return false;
    SetErrorCode(ERR_VALUERANGE);
    SendNAK;
    return false;
  }
  return true;
}

// Returns the number of DC bias output channels
void DCbiasNumber(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfDCChannels);
}


// This fuction sets the offsetable flag in the data structure for the selected channnel.
void DCbiasOffsetable(char *schan, char *state)
{
  DCbiasData *DCbData;
  int        chan;

  // Scan the parameters
  sscanf(schan,"%d",&chan);
  // Process the request
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  if((strcmp(state,"TRUE") != 0) && (strcmp(state,"FALSE") != 0)) BADARG;
  if(strcmp(state,"TRUE") == 0) DCbData->Offsetable = true;
  if(strcmp(state,"FALSE") == 0) DCbData->Offsetable = false;
  SendACK;
}

// Set the DCbias channel voltage
void DCbiasSet(int chan, float value)
{
    DCbiasData *DCbData;

  // Process the request
  if(!CheckChannel(chan,false)) return;
  if((DCbData = GetDCbiasDataPtr(chan,false)) == NULL) return;
  // Exit if value range error
  if(!CheckValue(DCbData, value - DCbData->DCoffset.VoltageSetpoint,false)) return;
  // Now set the value in the boards data structure and let the processing
  // loop update the outputs.
  if(abs(DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint - value) > 1.0 ) DelayMonitoring();
  DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint = value;
  // If this channel is being displayed in a dialog then refresh the display
  if(GetDCbiasBoard(chan) == SelectedDCBoard) dcbd.DCCD[(chan-1) & 0x07].VoltageSetpoint = value;
}

void DCbiasSet(char *Chan, char *Value)
{
  DCbiasData *DCbData;
  int        chan;
  float      value;
  
  // Scan the parameters
  sscanf(Chan,"%d",&chan);
  sscanf(Value,"%f",&value);
  // Process the request
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  // Exit if value range error
  if(!CheckValue(DCbData, value - DCbData->DCoffset.VoltageSetpoint)) return;
  // Now set the value in the boards data structure and let the processing
  // loop update the outputs.
  if(abs(DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint - value) > 1.0 ) DelayMonitoring();
  DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint = value;
  // If this channel is being displayed in a dialog then refresh the display
  if(GetDCbiasBoard(chan) == SelectedDCBoard) dcbd.DCCD[(chan-1) & 0x07].VoltageSetpoint = value;
  SendACK;
}

// This command adds a delta value to all the DCbias channels in the MIPS system.
// If any channel is out of range an error is returned.
void DCbiasDelta(char *Value)
{
  DCbiasData *DCbData;
  int        chan;
  float      fVal,value;
  
  // Scan the parameters
  sscanf(Value,"%f",&value);
  // Check the range on all the channels, exit if any errors are detected
  for(chan = 1; chan <= NumberOfDCChannels; chan++)
  {
     if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
     // Exit if value range error
     fVal = DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint;
     fVal += value;
     if(!CheckValue(DCbData, fVal - DCbData->DCoffset.VoltageSetpoint)) return;
  }
  // Now set the values in the boards data structure and let the processing
  // loop update the outputs.
  for(chan = 1; chan <= NumberOfDCChannels; chan++)
  {
    DCbData = GetDCbiasDataPtr(chan);
    fVal = DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint;
    fVal += value;
    if(abs(DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint - fVal) > 1.0 ) DelayMonitoring();
    DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint = value;
    // If this channel is being displayed in a dialog then refresh the display
    if(GetDCbiasBoard(chan) == SelectedDCBoard) dcbd.DCCD[(chan-1) & 0x07].VoltageSetpoint = fVal;
  }
  SendACK;
}


// Read the selected channel requested voltage
void DCbiasRead(int chan, float **fVal)
{
  DCbiasData *DCbData;
  
  *fVal = NULL;
  if(!CheckChannel(chan,false)) return;
  if((DCbData = GetDCbiasDataPtr(chan,false)) == NULL) return;
  *fVal = &DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint;
}

void DCbiasRead(int chan)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  SendACKonly;
  if(!SerialMute) serial->println(DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint);
}

// Read actual channel voltage
void DCbiasReadV(int chan)
{
  int brd;
  
  if(!CheckChannel(chan)) return;
  if((brd = GetDCbiasBoard(chan)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(DCbiasStates[brd]->Readbacks[(chan-1) & 0x07]);
}

// Set the float or offset voltage
void DCbiasSetFloat(char *Chan, char *Value)
{
  DCbiasData *DCbData;
  int        chan;
  int8_t     i;
  float      value;
  
  // Scan the parameters
  sscanf(Chan,"%d",&chan);
  sscanf(Value,"%f",&value);
  // Process the request
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  // Exit if value range error
  if(!CheckValue(DCbData, value)) return;
  // Exit with error if this board is not offsetable
  if(!DCbData->Offsetable)
  {
    SetErrorCode(ERR_NOTOFFSETABLE);
    SendNAK;
    return;
  }
  // Now set the value in the boards data structure and let the processing
  // loop update the outputs.
  DelayMonitoring();
  DCbData->DCoffset.VoltageSetpoint = value;
  //if(DCbDarray[0]->UseOneOffset) DCbDarray[0]->DCoffset.VoltageSetpoint = DCbDarray[1]->DCoffset.VoltageSetpoint = value;
  if(DCbDarray[0]->UseOneOffset) for(i=0;i<MAXDCbiasMODULES;i++) if(DCbDarray[i] != NULL) DCbDarray[i]->DCoffset.VoltageSetpoint = value;
  // If this channel is being displayed in a dialog then refresh the display
  if(GetDCbiasBoard(chan) == SelectedDCBoard) dcbd = *DCbD;
//  {
//    dcbd = *DCbD;
//    if((ActiveDialog == &DCbiasDialog) && (DCbiasDialog.Entry != DCDialogEntriesPage2))
//    {
//      DisplayAllDialogEntries(&DCbiasDialog);
//      DCbiasDialog.State = M_SCROLLING;
//    }
//  }
  SendACK;
}

// Read the requested float or offset voltage
void DCbiasReadFloat(int chan)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  SendACKonly;
  if(!SerialMute) serial->println(DCbData->DCoffset.VoltageSetpoint);
}

// Returns the range minimum value
bool DCbiasReadMin(int chan, float *fVal)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan,false)) return false;
  if((DCbData = GetDCbiasDataPtr(chan,false)) == NULL) return false;
  *fVal = DCbData->MinVoltage;
  return true;
}

void DCbiasReadMin(int chan)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  SendACKonly;
  if(!SerialMute) serial->println(DCbData->MinVoltage);
}

// Returns the range maximum value
bool DCbiasReadMax(int chan, float *fVal)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan,false)) return false;
  if((DCbData = GetDCbiasDataPtr(chan,false)) == NULL) return false;
  *fVal = DCbData->MaxVoltage;
  return true;
}

void DCbiasReadMax(int chan)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  SendACKonly;
  if(!SerialMute) serial->println(DCbData->MaxVoltage);
}

// This function turns on and off the power supply driving the DC bias board.
// This only works of +-250 volt and up power supplies. The +-50 volt board
// does not have this feature. This command will turn on and off all DC bias
// boards in one MIPS system.
void DCbiasPowerSet(char *cmd)
{
  if((strcmp(cmd,"ON")==0) || (strcmp(cmd,"OFF")==0))
  {
    SendACK;
    if(strcmp(cmd,"ON")==0) MIPSconfigData.PowerEnable = true;
    else MIPSconfigData.PowerEnable = false;
    // Now update the display if displaying...
    if((ActiveDialog == &DCbiasDialog) && (DCbiasDialog.Entry == DCDialogEntriesPage2))
    {
      DisplayAllDialogEntries(&DCbiasDialog);
      DCbiasDialog.State = M_SCROLLING;
    }
    SetPowerSource();
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Return the power supply on/off state
void DCbiasPower(void)
{
  SendACKonly;
  if(MIPSconfigData.PowerEnable) if(!SerialMute) serial->println("ON");
  if(!MIPSconfigData.PowerEnable) if(!SerialMute) serial->println("OFF");
}

// This command is used for system setup, it sets the number of channels on the defined board.
// The input parameters are board number 0 or 1, and number of channels 1 through 8.
void DCbiasSetNumBoardChans(int board, int num)
{
  if((board >= 0) && (board <= 1))
  {
    if((num >= 1) && (num <= 8))
    {
      DCbDarray[board]->NumChannels = num;
      dcbd = *DCbD;
      SendACK;
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Sets the DCbias board voltage range. This is a setup command used only in the factory
void SetDCbiasRange(int board, int range)
{
  if((board >= 0) && (board <= 3))
  {
    if(DCbDarray[board] != NULL)
    {
      DCbDarray[board]->MaxVoltage = abs(range);
      DCbDarray[board]->MinVoltage = -abs(range);
      // Set the gains and offsets for the ADC and DAC channels for this board
      for(int i=0; i < 8; i++)
      {
        DCbDarray[board]->DCCD[i].DCctrl.b = 32767;
        DCbDarray[board]->DCCD[i].DCctrl.m = (32767.0 / (float) range) * 0.95;
        DCbDarray[board]->DCCD[i].DCmon.b = 32767;
        DCbDarray[board]->DCCD[i].DCmon.m = (32767.0 / (float) range) * 0.95;
      }
      DCbDarray[board]->DCoffset.DCctrl.b = 32767;
      DCbDarray[board]->DCoffset.DCctrl.m = (-32767.0 / (float) range) * 0.95;
      DCbDarray[board]->DCoffset.DCmon.b = 32767;
      DCbDarray[board]->DCoffset.DCmon.m = (-32767.0 / (float) range) * 0.95;
      SendACK;
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Sets the DCbias board voltage range limit. This is a setup command used only in the factory.
// This command only limits the board's voltage range and does not adjust calibration parameters.
void SetDCbiasLimit(int board, int range)
{
  if((board >= 0) && (board <= 3))
  {
    if(DCbDarray[board] != NULL)
    {
      DCbDarray[board]->MaxVoltage = abs(range);
      DCbDarray[board]->MinVoltage = -abs(range);
      SendACK;
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This command sets the TWI addresses for the DAC and ADC and the SPI address for the octal DAC
// for extended operation. The addressed set are:
// SPI = 0
// ADC = 0x20
// DAC = 0x1A
void SetDCbiasExtended(int board)
{
  if((board >= 0) && (board <= 3))
  {
    if(DCbDarray[board] != NULL)
    {
      DCbDarray[board]->DACspi = 0;
      DCbDarray[board]->ADCadr = 0x20;
      DCbDarray[board]->DACadr = 0x1A;
      SendACK;
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This function will set all the DCbias channel values. This function is called with the
// arguments in the input ring buffer. All channel must be defined or an error is returned.
void SetAllDCbiasChannels(void)
{
   DCbiasData *DCbData;
   char   *Token;
   String sToken;
   int    ch;
   float  *vals;

   vals = new float [NumberOfDCChannels];
   if(vals == NULL) return;
   while(1)
   {
     // Read the DCbias channel values
     for(ch = 0; ch < NumberOfDCChannels; ch++)
     {
       GetToken(true);
       if((Token = GetToken(true)) == NULL) goto SetDCbiasProfileError;
       sToken = Token;
       vals[ch] = sToken.toFloat();
     }
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range of the parameters passed
     for(ch = 0; ch < NumberOfDCChannels; ch++)
     {
       if((DCbData = GetDCbiasDataPtr((ch+1),false)) == NULL) goto SetDCbiasProfileError;
       if(!CheckValue(DCbData, vals[ch] - DCbData->DCoffset.VoltageSetpoint,false)) goto SetDCbiasProfileError;
     }
     SendACKonly;
     // Output the data to the DACs
     for(ch = 0; ch < NumberOfDCChannels; ch++)
     {
        // Update the DAC
        DCbiasDACupdate(ch, DCbiasValue2Counts(ch, vals[ch]));
        // Update the display
        DCbiasSet(ch+1, vals[ch]);
        // If this channel is being displayed in a dialog then refresh the display
        if(GetDCbiasBoard(ch+1) == SelectedDCBoard) dcbd.DCCD[(ch) & 0x07].VoltageSetpoint = vals[ch];
     }
     // Pulse LDAC to latch them all at the same time!
     LDAClow;
     LDAChigh;
     // Set flag indicating DACs updated
     DCbiasProfileApplied = true; 
     delete [] vals;
     return;
   }
SetDCbiasProfileError:    // Sorry but it works in this case...
   // If here then we had bad arguments!
   SetErrorCode(ERR_BADARG);
   SendNAK;
   delete [] vals;
}

// Report all the DC bias channels voltage setpoints
void DCbiasReportAllSetpoints(void)
{
  DCbiasData *DCbData;
  int i;

  if(SerialMute) return;
  SendACKonly;
  for (i=1;i<=MAXDCbiasMODULES * 8;i++)
  {
    if((DCbData = GetDCbiasDataPtr(i,false)) == NULL) break;
    if((DCbData->NumChannels - 1) >= ((i-1) & 0x07)) 
    {
      if(i > 1) serial->print(",");
      serial->print(DCbData->DCCD[(i-1) & 0x07].VoltageSetpoint);
    }
  }
  serial->println("");
}

// Report all the DC bias channels readback voltages
void DCbiasReportAllValues(void)
{
  DCbiasData *DCbData;
  int brd;
  int i;

  if(SerialMute) return;
  SendACKonly;
  for (i=1;i<=MAXDCbiasMODULES * 8;i++)
  {
    if((DCbData = GetDCbiasDataPtr(i,false)) == NULL) break;
    if((brd = GetDCbiasBoard(i, false)) == -1) break;
    if((DCbData->NumChannels - 1) >= ((i-1) & 0x07)) 
    {
      if(i > 1) serial->print(",");
      serial->print(DCbiasStates[brd]->Readbacks[(i-1) & 0x07]);
    }
  }
  serial->println("");
}

void  DCbiasUseOneOffset(char *state)
{
  if((strcmp(state,"TRUE")==0) || (strcmp(state,"FALSE")==0) || (DCbDarray[0] = NULL))
  {
    if(strcmp(state,"TRUE")==0) DCbDarray[0]->UseOneOffset = true;
    else DCbDarray[0]->UseOneOffset = false;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

void  DCbiasOffsetReadback(char *state)
{
  if((strcmp(state,"TRUE")==0) || (strcmp(state,"FALSE")==0) || (DCbDarray[0] = NULL))
  {
    SendACKonly;
    if(strcmp(state,"TRUE")==0) DCbDarray[0]->OffsetReadback = true;
    else DCbDarray[0]->OffsetReadback = false;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;    
}

// Sets the ADC TWI address, radix 10
void SetDCbiasADCtwiADD(int board, int add)
{
  int b;
  
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  DCbDarray[board]->ADCadr = add;
  SendACK;  
}

// Sets the DAC TWI address, radix 10
void SetDCbiasDACtwiADD(int board, int add)
{
  int b;
  
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  DCbDarray[board]->DACadr = add;
  SendACK;  
}

// board = 0 thru 3
void ReportDCbiasSuppplies(int board)
{
  int b;
  
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  if((DCbDarray[board]->DACadr & 0xFE) != 0x10)
  {
     SetErrorCode(ERR_NOTSUPPORTED);
     SendNAK;    
     return;  
  }
  SendACKonly;
  if(SerialMute) return;
  // Report logic voltage
  int brd = SelectedBoard();
  SelectBoard(board);
  if(DCbDarray[board]->MaxVoltage < 100)
  {
     int i = AD5593readADC(DCbDarray[board]->DACadr, 4, 10);
     serial->print("Logic supply = "); serial->print((2.5 * i / 65536) * 2.0); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 2, 10);
     serial->print("Positive supply = "); serial->print((2.5 * i / 65536) * 101); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 3, 10);
     serial->print("Negative supply = "); serial->print((-3.3 + (2.5 * i / 65536)) * 31.3); serial->println(" volts"); 
  }
  else if(DCbDarray[board]->MaxVoltage < 300)
  {
     int i = AD5593readADC(DCbDarray[board]->DACadr, 4, 10);
     serial->print("Logic supply = "); serial->print((2.5 * i / 65536) * 2.0); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 3, 10);
     serial->print("Positive supply = "); serial->print((2.5 * i / 65536) * 201); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 2, 10);
     serial->print("Negative supply = "); serial->print((-3.3 + (2.5 * i / 65536)) * 201); serial->println(" volts");
  } 
  else
  {
     int i = AD5593readADC(DCbDarray[board]->DACadr, 4, 10);
     serial->print("Logic supply = "); serial->print((2.5 * i / 65536) * 2.0); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 3, 10);
     serial->print("Positive supply = "); serial->print((2.5 * i / 65536) * 402); serial->println(" volts"); 
     i = AD5593readADC(DCbDarray[board]->DACadr, 2, 10);
     serial->print("Negative supply = "); serial->print((-3.3 + (2.5 * i / 65536)) * 402); serial->println(" volts");
  } 
  SelectBoard(brd);
}

// October 15, 2016. Added the DCbias profile capability. This allow up to 10 voltage
// profiles. The following commands support the voltage profiles.
//  SDCBPRO,num,ch1,ch2,ch3...    // Set a profile, must provide all channels
//  GDCBPRO,num                   // Return a profile
//  ADCBPRO,num                   // Apply a profile
//  CDCBPRO,num                   // Set profile with current values
//  TDCBPRO,p1,p1,dwell           // Toggle between profile p1 and p2, dwell at each profile
//                                // for dwell millisec
//  TDCBSTP                       // Stops the toggling

// This function is called with the parameters in the input ring buffer. The full command
// line has been received before this function is called. The first argument is the profile
// number followed by the channel values. All channels must be defined for a valid command.
// If the profile number is valid and data is invalid then the profile is set to 0.
void SetDCbiasProfile(void)
{
   DCbiasData *DCbData;
   char   *Token;
   String sToken;
   int    num,ch;
   float  val;

   num = -1;
   while(1)
   {
     // Read The profile number
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     num = sToken.toInt();
     if((num < 1)||(num > NumProfiles)) break;
     num--;
     // If profile pointer is NULL then allocate
     if(DCbiasProfiles[num] == NULL)
     {
       DCbiasProfiles[num] = new float [NumberOfDCChannels];
       if(DCbiasProfiles[num] == NULL)
       {
           SetErrorCode(ERR_CANTALLOCATE);
           SendNAK;
           return;
       }
     }
     // Read the DCbias channel values
     for(ch = 0; ch < NumberOfDCChannels; ch++)
     {
       GetToken(true);
       if((Token = GetToken(true)) == NULL) goto SetDCbiasProfileError;
       sToken = Token;
       DCbiasProfiles[num][ch] = sToken.toFloat();
     }
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range of the parameters passed
     for(ch = 0; ch < NumberOfDCChannels; ch++)
     {
       if((DCbData = GetDCbiasDataPtr((ch+1),false)) == NULL) goto SetDCbiasProfileError;
       if(!CheckValue(DCbData, DCbiasProfiles[num][ch] - DCbData->DCoffset.VoltageSetpoint,false)) goto SetDCbiasProfileError;
     }
     // Exit with no error!
     SendACK;
     return;
   }
SetDCbiasProfileError:    // Sorry but it works in this case...
   // If here then we had bad arguments!
   // If num is valid set the profile to 0
   if((num >= 0)&&(num < NumProfiles)) for(ch = 0; ch < NumberOfDCChannels; ch++) DCbiasProfiles[num][ch] = 0.0;
   SetErrorCode(ERR_BADARG);
   SendNAK;
}

// This function returns the selected DCbias profile.
void GetDCbiasProfile(int num)
{
   if((num < 1)||(num > NumProfiles)) BADARG;
   SendACKonly;
   num--;
   // If profile pointer is NULL then exit with error
   if(DCbiasProfiles[num] == NULL)
   {
     SetErrorCode(ERR_PROFILENOTDEFINED);
     SendNAK;
     return;
   }
   if(SerialMute) return;
   for(int ch = 0; ch < NumberOfDCChannels; ch++)
   {
     serial->print(DCbiasProfiles[num][ch]);
     if(ch == (NumberOfDCChannels-1)) serial->println("");
     else serial->print(",");
   }
}

// This function applies a DCbias voltage profile. All the values are written to
// the DAC and then LDAC is pulsed to make all channels update at the
// same time. The values are also written to the DCbias buffers and display buffers
// to allow the screen to update. A flag is set in this function to stop the DCbias
// polling loop from detecting the change and updating the DACs in the pollong loop.
//
// This function is assumed to be called with the needed profile buffers allocated,
// if not it exits without action.
void ApplyDCbiasProfile(int num)
{
   // If profile pointer is NULL then exit
   if(DCbiasProfiles[num-1] == NULL) return;
   CurrentProfile = num;
   num--;
   for(int ch = 0; ch < NumberOfDCChannels; ch++)
   {
      // Update the DAC
      DCbiasDACupdate(ch, DCbiasValue2Counts(ch, DCbiasProfiles[num][ch]));
      // Update the display
      DCbiasSet(ch+1, DCbiasProfiles[num][ch]);
   }
   // Pulse LDAC to latch them all at the same time!
   LDAClow;
   LDAChigh;
   // Set flag indicating DACs updated
   DCbiasProfileApplied = true;  
}

void SetApplyDCbiasProfile(int num)
{
   if((num < 1)||(num > NumProfiles)) BADARG;
   // If profile pointer is NULL then exit with error
   if(DCbiasProfiles[num-1] == NULL) 
   {
     SetErrorCode(ERR_PROFILENOTDEFINED);
     SendNAK;
     return;
   }
   ApplyDCbiasProfile(num);
   SendACK;
}

// This function will set the defined profile number with the current DCbias settings.
void SetDCbiasProfileFromCurrent(int num)
{
   int   ch;
   float *fval;

   // If profile buffer is NULL then allocate...
   if(DCbiasProfiles[num-1] == NULL)
   {
     DCbiasProfiles[num-1] = new float [NumberOfDCChannels];
     if(DCbiasProfiles[num-1] == NULL)
     {
         SetErrorCode(ERR_CANTALLOCATE);
         SendNAK;
         return;
     }
   }   
   if((num < 1)||(num > NumProfiles)) BADARG;
   // Move current values to the select profile
   for(ch = 0; ch < NumberOfDCChannels; ch++)
   {
      DCbiasRead(ch+1, &fval);
      DCbiasProfiles[num-1][ch] = *fval;
   }
   SendACK;
}

// This function enabled toggeling between two different DCbias voltage profiles.
// Additionally a dwell time, in milli seconds, defines how long the system remains 
// at each profile. This function is called with all the arguments in the ringbuffer.
void ProfileISR(void)
{
  if(CurrentProfile == Profile1) ApplyDCbiasProfile(Profile2);
  else ApplyDCbiasProfile(Profile1);
}

void ProfileTrigISR(void)
{
  if(digitalRead(DI1) == 0) ApplyDCbiasProfile(Profile1);
  else ApplyDCbiasProfile(Profile2);
}

void SetDCbiasProfileToggle(void)
{
   DCbiasData *DCbData;
   char   *Token;
   String sToken;

   while(1)
   {
     // Read the first profile number
     GetToken(true);  // This is because the command string is followed by a comma, this gets the comma
     if((Token = GetToken(true)) == NULL) break;  // Gets the string
     sToken = Token;
     Profile1 = sToken.toInt();
     // Read the second profile number
     GetToken(true);  // Gets the comma
     if((Token = GetToken(true)) == NULL) break;  // Gets the string
     sToken = Token;
     Profile2 = sToken.toInt();
     // Read the dwell time 
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ProfileDwell = sToken.toInt();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range of the parameters passed
     if((Profile1 < 1) || (Profile1 > NumProfiles)) break;
     if((Profile2 < 1) || (Profile2 > NumProfiles)) break;
     // Make sure the profiles are allocated
     if((DCbiasProfiles[Profile1-1] == NULL) || (DCbiasProfiles[Profile2-1] == NULL))
     {
       SetErrorCode(ERR_PROFILENOTDEFINED);
       SendNAK;
       return;
     }
     if(sToken == "R")
     {
       // Here if the user selected the R input to control the toggle
       if(ProfileTrig == NULL) ProfileTrig = new DIhandler;
       ProfileTrig->detach();
       ProfileTrig->attached('R', CHANGE, ProfileTrigISR);
       ProfileTrigISR();
       SendACK;
       return;
     }
     // Apply the first profile and setup the timer and ISR for
     // profile toggling
     if(ProfileDwell < 0) break;
     ApplyDCbiasProfile(Profile1);
     TMR_Profiles.attachInterrupt(ProfileISR);
     TMR_Profiles.start(ProfileDwell*1000);
     // Exit with no error!
     SendACK;
     return;
   }
   // If here then we had bad arguments!
   SetErrorCode(ERR_BADARG);
   SendNAK;  
}

void StopProfileToggle(void)
{
  if(ProfileTrig != NULL) ProfileTrig->detach();
  TMR_Profiles.stop();
  SendACK;
}

//
// DCbias channel pulse generation function.
//
//  This code allows you to define a number of parameters to enable generation of a pulse on
//  one DCbias channel. The Channel will pulse from its current value to the user defined 
//  value. The user can control the following parameters:
//
//  Channel
//  Voltage
//  Delay from trigger, in uS
//  Width, in uS
//  Trigger source, Q thru X or t
//  Enable/disable
//
//  This function uses timer channel 5, same channel used for the profiles so these two capabilities
//  can not be used together.
//
//  Host commands, note: limited error checking!
//
//  SDCBPCH,channel
//  GDCBPCH
//  SDCBPV,voltage
//  GDCBPV
//  SDCBPD,delay
//  GDCBPD
//  SDCBPW,width
//  GDCBPW
//  SDCBPT,trigger
//  GDCBPT
//  SDCBPENA,true or false
//  GDCBPENA
//

// DCbias pulse channel variables
bool  DCbiasGening   = false;
bool  DCbiasPena     = false;
int   DCbiasPchan    = 1;
float DCbiasPvoltage = 20;
int   DCbiasPdelay   = 100;
int   DCbiasPwidth   = 100;
char  DCbiasPtrigger = 't';
bool  SavedPulseTestEnable;

DIhandler *DIDCbiasPulse = NULL;
MIPStimer *DCbiasPulseTMR = NULL;

// Interrupt service routine for the timer interrupt that happens
// at the end of the timer count. This is the point that the pulse
// ends the DC bias channel is reset to its default value
void DCbiasPulseTMR_ISR(void)
{
  if(!DCbiasGening) return;
  // Pulse LDAC
  PulseLDAC;
  ValueChange = true;
  // Load the pulse value
  if(DCbiasCH2Brd(DCbiasPchan-1) == SelectedBoard()) DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage));
  else if(AcquireTWI()) 
  { 
    int b=SelectedBoard();
    DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage)); 
    SelectBoard(b);
    ReleaseTWI(); 
  }
  else TWIqueue(DCbiasDACupdate, DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage));
  DCbiasTestEnable = SavedPulseTestEnable;
  DCbiasGening = false;
}

// This function will raise the DCbias channel to the pulse value.
// The pluse voltage value has to be loaded into the DAC, this
// function will pulse LDAC and load the default value.
void DCbiasPulseHigh(void)
{
  if(!DCbiasGening) return;
  SavedPulseTestEnable = DCbiasTestEnable;
  DCbiasTestEnable = false;
  // Pulse LDAC
  PulseLDAC;
  ValueChange = true;
  // Load the default value
  int brd = DCbiasCH2Brd(DCbiasPchan-1);
  if(brd == SelectedBoard()) DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbDarray[brd]->DCCD[(DCbiasPchan-1) & 7].VoltageSetpoint));
  else if(AcquireTWI()) 
  { 
    int b=SelectedBoard();
    if(brd != -1) DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbDarray[brd]->DCCD[(DCbiasPchan-1) & 7].VoltageSetpoint));
    SelectBoard(b);
    ReleaseTWI(); 
  }
  else TWIqueue(DCbiasDACupdate, DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbDarray[brd]->DCCD[(DCbiasPchan-1) & 7].VoltageSetpoint));
}

// Interrupt service routine to trigger the DC bias pulse. This event
// starts the pulse generation.
// Setup the timer to generate the pulse.
void DCbiasPulse_ISR(void)
{
  if(DCbiasPdelay <= 0) DCbiasPdelay = 1;
  if(DCbiasPwidth <= 0) DCbiasPwidth = 5;
  // Set RC count to delay time plus width
  DCbiasPulseTMR->setRC(((DCbiasPdelay + DCbiasPwidth) * 105)/10);
  // Set RA count to delay time
  DCbiasPulseTMR->setRA((DCbiasPdelay * 105)/10);
  // Start the timer
  DCbiasPulseTMR->enableTrigger();
  DCbiasPulseTMR->softwareTrigger();
  // Load the pulse value into the DAC
  if(DCbiasCH2Brd(DCbiasPchan-1) == SelectedBoard()) DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage));
  else if(AcquireTWI()) 
  { 
    int b=SelectedBoard();
    DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage));
    SelectBoard(b);
    ReleaseTWI(); 
  }
  else TWIqueue(DCbiasDACupdate, DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbiasPvoltage));
  DCbiasGening = true;
}

// Enable / Disable the DCbias pulse output. This function is called from the
// command processor. The trigger source has to be defined before this call
// and will be ignored if changed after the system is enabled. Valid parameters
// are TRUE or FALSE
void SetDCbiasPena(char *state)
{
  String token;

  token = state;
  // Validate input values
  if((token != "TRUE") && (token != "FALSE"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;  
     return;
  }
  if(token == "FALSE")
  {
    DCbiasPena = false;
    if(DCbiasPulseTMR != NULL) DCbiasPulseTMR->stop();
    if(DIDCbiasPulse != NULL)  DIDCbiasPulse->detach();
    SendACK;
    return;
  }
  // Load the default because we don't know the state
  int b=SelectedBoard();
  int brd = DCbiasCH2Brd(DCbiasPchan-1);
  if(brd != -1) DCbiasDACupdate(DCbiasPchan-1, DCbiasValue2Counts(DCbiasPchan-1, DCbDarray[brd]->DCCD[(DCbiasPchan-1) & 7].VoltageSetpoint));
  SelectBoard(b);
  // Allocate handler and timer if null
  if(DCbiasPulseTMR == NULL) DCbiasPulseTMR = new MIPStimer(TMR_DCbiasPulse);
  DCbiasPulseTMR->stop();
  DCbiasPulseTMR->begin();
  DCbiasPulseTMR->attachInterruptRA(DCbiasPulseHigh);
  DCbiasPulseTMR->attachInterrupt(DCbiasPulseTMR_ISR);
  DCbiasPulseTMR->setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
  DCbiasPulseTMR->setTrigger(TC_CMR_EEVTEDG_NONE);
  DCbiasPulseTMR->setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);   // 10.5 MHz clock
  DCbiasPulseTMR->stopOnRC();
  if(DCbiasPtrigger == 't')
  {
    // Queue trigger function to start the pulse
    QueueTpulseFunction(DCbiasPulse_ISR,true);
  }  
  else
  {
     QueueTpulseFunction(DCbiasPulse_ISR,false);
     if(DIDCbiasPulse == NULL) DIDCbiasPulse = new DIhandler;
     if((DCbiasPulseTMR == NULL) || (DIDCbiasPulse == NULL))
     {
       SetErrorCode(ERR_INTERNAL);
       SendNAK;  
       return;
     }
     // Setup trigger
     DIDCbiasPulse->detach();
     DIDCbiasPulse->attached(DCbiasPtrigger, RISING, DCbiasPulse_ISR);
  }
  LDACrelease;
  pinMode(LDAC,OUTPUT);
  LDAChigh;
  DCbiasPena = true;
  SendACK;
  return;
}

void SetDCbiasPtrigger(char *src)
{
  if(((src[0] >= 'Q') && (src[0] <= 'X')) || (src[0] == 't'))
  {
    DCbiasPtrigger = src[0];
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

void GetDCbiasPtrigger(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(DCbiasPtrigger);  
}

// End of DCbias channel pulse generation function.

// This function saves the DC bias module data to EEPROM. All detected DC bias modules are saved.
void SaveDCB2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<4; b++)
  {
    if(DCbDarray[b] != NULL)
    {
      SelectBoard(b);
      if (WriteEEPROM(DCbDarray[b], DCbDarray[b]->EEPROMadr, 0, sizeof(DCbiasData)) != 0) berr = true;
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

void DCbiasCalParms(int chan)
{
  DCbiasData *DCbData;
  
  if(!CheckChannel(chan)) return;
  if((DCbData = GetDCbiasDataPtr(chan)) == NULL) return;
  SendACKonly;
  if(SerialMute) return;
  serial->print(DCbData->DCCD[(chan-1) & 0x07].DCctrl.m);
  serial->print(",");
  serial->println(DCbData->DCCD[(chan-1) & 0x07].DCctrl.b);  
}

void DCbiasCalsetM(char *chan, char *val)
{
  String sToken;
  int    ch;
  float  fval;
  DCbiasData *DCbData;

  sToken = chan;
  ch = sToken.toInt();
  if(!CheckChannel(ch)) return;
  if((DCbData = GetDCbiasDataPtr(ch)) == NULL) return;
  SendACK;
  sToken = val;
  fval = sToken.toFloat();
  DCbData->DCCD[(ch-1) & 0x07].DCctrl.m = fval;
}

void DCbiasCalsetB(char *chan, char *val)
{
  String sToken;
  int    ch;
  float  fval;
  DCbiasData *DCbData;

  sToken = chan;
  ch = sToken.toInt();
  if(!CheckChannel(ch)) return;
  if((DCbData = GetDCbiasDataPtr(ch)) == NULL) return;
  SendACK;
  sToken = val;
  fval = sToken.toFloat();
  DCbData->DCCD[(ch-1) & 0x07].DCctrl.b = fval;  
}

// The following functions support host commands for the channel and global offset capability

// Set the global offset parameter. board = 0 thru 3.
void SetDCbiasOffOff(char *brd, char *fval)
{
  String sToken;
  float  f;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  sToken = fval;
  f = sToken.toFloat();
  SetOffsetOffset(board, f);
  SendACK;
}

void GetDCbiasOffOff(int board)
{
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->println(DCbDarray[board]->OffsetOffset);    
}

void SetDCbiasCHOff(char *brd, char *fval)
{
  String sToken;
  float  f;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  sToken = fval;
  f = sToken.toFloat();
  SetChannelOffset(board, f);
  SendACK;  
}

void GetDCbiasCHOff(int board)
{
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->println(DCbDarray[board]->ChannelOffset);      
}

void SetDCbiasCHMK(char *brd, char *mask)
{
  String sToken;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  DCbDarray[board]->OffsetChanMsk = strtol(mask,NULL,16);
  if(SelectedDCBoard == board) dcbd.OffsetChanMsk = DCbDarray[board]->OffsetChanMsk;  // Sets the UI buffer if this module is seleted
  SetChannelOffset(board, DCbDarray[board]->ChannelOffset);
  SendACK;
}

void GetDCbiasCHMK(int board)
{
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->println(DCbDarray[board]->OffsetChanMsk, HEX);  
}

// The following functions allow the user to connect the ADC value change function to the DCbias
// module's offset control. This allows an external analog voltages to be read and used to adjust
// DCbias module output voltages.

// This ISR will apply an offset voltage to the DCbias module's offset channel and or channel offset. 
void ADCoffsetAdjustISR(int i)
{
  int   b;
  float pol;

  if(AcquireTWI()) 
  {
    for(b=0;b<MAXDCbiasMODULES;b++) if(DCbDarray[b] != NULL)   
    {
       if(DCbDarray[b]->ADCgainOff != 0) 
       {  
          pol = 1;
          if(DCbDarray[b]->PolDIO != 0) if(ReadDIO(DCbDarray[b]->PolDIO) == HIGH) pol = -1;
          SetOffsetOffset(b, (float)i * DCbDarray[b]->ADCgainOff * pol);
       }
       if(DCbDarray[b]->ADCgainCh != 0) 
       {
          pol = 1;
          if(DCbDarray[b]->PolDIO != 0) if(ReadDIO(DCbDarray[b]->PolDIO) == HIGH) pol = -1;
          if(DCbDarray[b]->OffsetChanMsk != 0) SetChannelOffset(b, (float)i * DCbDarray[b]->ADCgainCh * pol);
       }
    }
  }
  else 
  {
    for(b=0;b<MAXDCbiasMODULES;b++) if(DCbDarray[b] != NULL)   
    {
       if(DCbDarray[b]->ADCgainOff != 0) 
       {
           pol = 1;
           if(DCbDarray[b]->PolDIO != 0) if(ReadDIO(DCbDarray[b]->PolDIO) == HIGH) pol = -1;
           TWIqueue(SetOffsetOffset, b, (float)i * DCbDarray[b]->ADCgainOff * pol);
       }
       if(DCbDarray[b]->ADCgainCh != 0) if(DCbDarray[b]->OffsetChanMsk != 0) 
       {
           pol = 1;
           if(DCbDarray[b]->PolDIO != 0) if(ReadDIO(DCbDarray[b]->PolDIO) == HIGH) pol = -1;
           TWIqueue(SetChannelOffset, b, (float)i * DCbDarray[b]->ADCgainCh * pol);
       }
    }
  }
}

void SetADCoffsetAdjust(char *brd, char *gain)
{
  String sToken;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  sToken = gain;
  DCbDarray[board]->ADCgainOff = sToken.toFloat();
  ADCattachInterrupt(ADCoffsetAdjustISR);
  SendACK;
}

void SetADCchannelAdjust(char *brd, char *gain)
{
  String sToken;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  sToken = gain;
  DCbDarray[board]->ADCgainCh = sToken.toFloat();
  ADCattachInterrupt(ADCoffsetAdjustISR);
  SendACK;
}

void SetADCgainPol(char *brd, char *dio)
{
  String sToken;
  int    board;

   sToken = brd;
   if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
   sToken = dio;
   if(sToken == "Q")      DCbDarray[board]->PolDIO = 'Q';
   else if(sToken == "R") DCbDarray[board]->PolDIO = 'R';
   else if(sToken == "S") DCbDarray[board]->PolDIO = 'S';
   else if(sToken == "T") DCbDarray[board]->PolDIO = 'T';
   else if(sToken == "U") DCbDarray[board]->PolDIO = 'U';
   else if(sToken == "V") DCbDarray[board]->PolDIO = 'V';
   else if(sToken == "W") DCbDarray[board]->PolDIO = 'W';
   else if(sToken == "X") DCbDarray[board]->PolDIO = 'X';
   else if(sToken == "NA")DCbDarray[board]->PolDIO = 0;
   else BADARG;
   SendACK;  
}

void GetADCgainPol(int board)
{
  if((DCbDarray[board] == NULL) || (board < 0) || (board >3)) BADARG;
  SendACKonly;
  if(SerialMute) return;
  if(DCbDarray[board]->PolDIO == 0) serial->println("NA");
  else serial->println(DCbDarray[board]->PolDIO); 
}

// The following functions support the Level Detection module and allow its
// connection to the MIPS controller auxilary input. This module can be used
// to detect a voltage change and apply the voltage to the DCbias module
// offset.

// This ISR is called when an event is detected my the level detection module.
void LevelDetISR(void)
{
  float fval;
  byte *b;
  int  i=0;

  b = (byte *)&fval;
  Wire1.beginTransmission(LevelDetAdd);
  Wire1.write(TWI_LEVDET_READ_ADC);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)LevelDetAdd, (uint8_t)4);
  while (Wire1.available()) b[i++] = Wire1.read();
  //serial->println(fval);
  if(AcquireTWI()) 
  {
     for(i=0;i<MAXDCbiasMODULES;i++) if(DCbDarray[i] != NULL)
     {  
       if(DCbDarray[i]->ADCgainOff != 0) SetOffsetOffset(i, fval);
       if(DCbDarray[i]->ADCgainCh != 0) if(DCbDarray[i]->OffsetChanMsk != 0) SetChannelOffset(i, fval);
     }
  }
  else
  {
     for(i=0;i<MAXDCbiasMODULES;i++) if(DCbDarray[i] != NULL)   
     {
       if(DCbDarray[i]->ADCgainOff != 0) TWIqueue(SetOffsetOffset, i, fval);
       if(DCbDarray[i]->ADCgainCh != 0) if(DCbDarray[i]->OffsetChanMsk != 0) TWIqueue(SetChannelOffset, i, fval);
     }
 }
}

void SetLevelDetOffsetAdjust(char *brd, char *TWIadd)
{
  String sToken;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  LevelDetAdd = strtol(TWIadd,NULL,16);
  DCbDarray[board]->ADCgainOff = 1;
  // Init the Level Detector
  pinMode(SCL1,INPUT);
  pinMode(SDA1,INPUT);
  pinMode(LEVCHANGE,INPUT);
  Wire1.begin();
  Wire1.setClock(Wire1DefaultSpeed);
  attachInterrupt(digitalPinToInterrupt(LEVCHANGE), LevelDetISR, RISING);
  SendACK;
}

void SetLevelDetChOffsetAdjust(char *brd, char *TWIadd)
{
  String sToken;
  int    board;

  sToken = brd;
  if((DCbDarray[board = sToken.toInt()] == NULL) || (board < 0) || (board >3)) BADARG;
  LevelDetAdd = strtol(TWIadd,NULL,16);
  DCbDarray[board]->ADCgainCh = 1;
  // Init the Level Detector
  pinMode(SCL1,INPUT);
  pinMode(SDA1,INPUT);
  pinMode(LEVCHANGE,INPUT);
  Wire1.begin();
  Wire1.setClock(Wire1DefaultSpeed);
  attachInterrupt(digitalPinToInterrupt(LEVCHANGE), LevelDetISR, RISING);
  SendACK;
}

// This function returns true only if all the DCbias channels and offsets are
// set to zero.
bool DCbiasAllZeroCheck(void)
{
  DCbiasData *DCbData;
  int i;

  // Check all the channels
  for(int i=1;i<=NumberOfDCChannels;i++)
  {
     if((DCbData = GetDCbiasDataPtr(i,false)) == NULL) return false;
     if(DCbData->DCCD[(i-1) & 0x07].VoltageSetpoint != 0.0) return false;
     if(DCbData->DCoffset.VoltageSetpoint != 0.0) return false;
  }
  return true;
}

// Host interface based calibration functions.

// This function allows calibration of the selected channel
bool CalDCbiasChannel(int channel)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  bool       stat;
  
  if((b=DCbiasCH2Brd(channel-1)) == -1) return false;
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b]->MinVoltage;
  CC.Max=DCbDarray[b]->MaxVoltage;
  CC.DACaddr=DCbDarray[b]->DACspi;  
  CC.ADCaddr=DCbDarray[b]->ADCadr;
  CC.DACout=&DCbDarray[b]->DCCD[(channel-1) & 7].DCctrl;
  CC.ADCreadback=&DCbDarray[b]->DCCD[(channel-1) & 7].DCmon;
  // Define this channels name
  sprintf(Name,"Channel %2d",channel);
  // Calibrate this channel
  stat = ChannelCalibrateSerial(&CC, Name);
  dcbd = *DCbD;
  DCbiasUpdate = true;
  return stat;
}

// This function allows calibration of all DCbias channels in the system
void CalDCbiasChannels(void)
{
  if(DCbiasAllZeroCheck() == false)
  {
    serial->println("All DCbias channels must be 0!");
    return;
  }
  serial->println("Calibrate all DCbias channels.");
  for(int i=1;i<=NumberOfDCChannels;i++)
  {
    if(!CalDCbiasChannel(i)) return;
  }
  serial->println("All channels processed!");
  SaveDCbiasSettings();
}

bool CalDCbiasOffset(int channel)
{
  ChannelCal CC;
  char       Name[30];
  int        b=0;
  bool       stat;
  
  if((b=DCbiasCH2Brd(channel-1)) == -1) return false;
  if(DCbiasAllZeroCheck() == false)
  {
    serial->println("All DCbias channels must be 0!");
    return false;
  }
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b]->MinVoltage;
  CC.Max=DCbDarray[b]->MaxVoltage;
  CC.DACaddr=DCbDarray[b]->DACadr;  
  CC.ADCaddr=DCbDarray[b]->ADCadr;
  CC.DACout=&DCbDarray[b]->DCoffset.DCctrl;
  CC.ADCreadback=NULL;
  if((NumberOfDCChannels > 8) && (DCbDarray[0]->OffsetReadback) && (b == 1)) CC.ADCreadback=&DCbDarray[b]->DCCD[7].DCmon;
  if((NumberOfDCChannels < 8) && (DCbDarray[0]->OffsetReadback) && (b == 0)) CC.ADCreadback=&DCbDarray[b]->DCCD[7].DCmon;
  // If the DACadr is 0x10 or 0x11 then setup for the AD5593 chip
  if((DCbDarray[b]->DACadr & 0xFE) == 0x10)
  {
    // Here if using the AD5593
    CC.ADCpointer = &AD5593readADC;
    CC.ADCaddr=DCbDarray[b]->DACadr;
    CC.ADCreadback=&DCbDarray[b]->DCoffset.DCmon;;
  }
  // Define this channels name
  if(channel <= 8) sprintf(Name,"Offset for channel 1-8");
  else sprintf(Name,"Offset for channel 9-16");
  // Calibrate this channel
  stat = ChannelCalibrateSerial(&CC, Name);
  dcbd = *DCbD; 
  DCbiasUpdate = true;    
  return stat; 
}

// The waveform generation feature allows generation of triangle waveforms
// on DCBias output channels. Multiple waveforms can be defined. All waveforms
// share the same output update rate (sample rate). The waveforms are generated
// in an interrupt service routine. A timer is used to generate interrupts at
// the user defined sample rate.

// Waveform generation functions

// This ISR will process all waveforms in the linked list and update
// the output values. If the board select is not correct then TWI acquire
// if needed to change the board select output. 
void WaveformISR(void)
{
   Waveform *w;
   
   if(wfs == NULL) return;
   w = wfs->wf;
   while(w != NULL)
   {
      // Process this waveform
      w->val = w->val + w->step;
      if(w->val > w->max) { w->val = 2 * w->max - w->val; w->step *= -1; }
      if(w->val < w->min) { w->val = 2 * w->min - w->val; w->step *= -1; }
      int cnts = DCbiasValue2Counts(w->ch, w->val);
      if((DCbiasCH2Brd(w->ch)&1) == SelectedBoard()) DCbiasDACupdate(w->ch,cnts);
      else
      {
         // If here we will need to change the board select value so we need
         // to allocate TWI to make sure its not in the middle of a cycle,
         // if we can't acquire then queue up the operation
         if(AcquireTWI()) { DCbiasDACupdate(w->ch,cnts); ReleaseTWI(); }
         else TWIqueue(DCbiasDACupdate,w->ch,cnts);     
      }
      // Advance to next waveform
      w = w->wf;
   }
}

// Waveform generation command processing functions. These functions are called
// from the serial command processor

// This function will initialize the waveform generation capability. Any existing
// waveforms will be deleted. The variable sps is the desired waveform generation
// update rate in samples per second. sps is an integer.
// The waveform generation uses timer 4 to generate an interrupt at the defined 
// sample rate. 
void WFMinit(int sps)
{
   Waveform *w;

   if(wfs != NULL) 
   {
      WFMtimer.stop();
      // Delete all the current waveforms
      while(wfs->wf != NULL)
      {
         w = wfs->wf;
         while(w != NULL) w = w->wf;
         delete w;
         w = NULL; 
      }
   }
   else wfs = new Waveforms;
   wfs->wf = NULL;
   wfs->sps = sps;
  // Start the real time interrupt
  WFMtimer.attachInterrupt(WaveformISR);
  WFMtimer.setFrequency((double)sps);
}

// This function defines a waveform and adds it to the Waveforms structure's
// linked list of waveforms. This function is called from the command processor
// with the following arguments in the ring buffer:
// Channel, DCbias channel number
// Frequency, in Hz
// Min, minimum voltage
// Max, maximum voltage
// Note: if the waveform has already been defined for the given channel then
// it will be updated.
void WFMaddWF(void)
{
   char     *tkn;
   String   arg;
   DCbiasData *DCbData;
   Waveform *w,**new_w;
   
  w = new Waveform;
  while(true)
  {
     if(wfs == NULL) break;
     // Read and validate the parameters
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     w->ch = arg.toInt() - 1;
     if(!CheckChannel(w->ch+1,false)) break;
     if((DCbData = GetDCbiasDataPtr(w->ch+1,false)) == NULL) break;
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     w->freq = arg.toFloat();
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     w->min = arg.toFloat();
     if(!CheckValue(DCbData, w->min,false)) break;
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     w->max = arg.toFloat();
     if(!CheckValue(DCbData, w->max,false)) break;
     // Calculate the step size and set initial value
     w->wf = NULL;
     w->val = (w->max - w->min)/2.0;
     w->step = 2 * (w->max - w->min) * w->freq / wfs->sps;
     // Insert in the linked list
     new_w = &wfs->wf;
     while(*new_w != NULL) 
     {
       if((*new_w)->ch == w->ch)
       {
          // If here do an update and exit with ACK
          **new_w = *w;
          delete w;
          SendACK;
          return;
       }
       new_w = &((*new_w)->wf);
     }
     *new_w = w;
     // Exit
     SendACK;
     return;
  }
  // Error exit
  delete w;
  BADARG;
}

// Enable the waveform generation
void WFMenable(void)
{
   if(wfs == NULL) BADARG;
   WFMtimer.start(-1, 0, false);
   DCbiasTestEnable = false;
   SendACK;
}

// Disable the waveform generation
void WFMdisable(void)
{
   if(wfs == NULL) BADARG;
   WFMtimer.stop();
   DCbiasUpdate = true;
   DCbiasTestEnable = true;
   SendACK;
}

// Report a channel's waveform parameter. valid parameters are:
// 0 = freq
// 1 = min
// 2 = max
void WFMreport(int ch,int parm)
{
   Waveform *w;

   if((wfs == NULL) || (parm < 0) || (parm > 2)) BADARG;
   // Find the requested channel and report the requested parameter
   w = wfs->wf;
   while(w != NULL) 
   {
      if(w->ch == ch-1)
      {
         SendACKonly;
         if(SerialMute) return;
         if(parm == 0) serial->println(w->freq);
         else if(parm == 1) serial->println(w->min);
         else serial->println(w->max);
         return;
      }
      w = w->wf;
   }
   BADARG;
}

// This function will mask a specific channel's readback testing. Set to TRUE to
// disable testing on secified channel or FALSE to enable testing. Note this data
// is rest to FALSE on reset.
void setDCBchanMask(char *channel, char *value)
{
   int    ch;
   String token;

   token = channel;
   ch = token.toInt() - 1;
   if((ch < 0) || (ch > 31)) BADARG;
   token = value;
   if(token == "TRUE") DCBtstMask[ch/8] |= 1 << (ch % 8);
   else if(token == "FALSE") DCBtstMask[ch/8] &= ~(1 << (ch % 8));
   else BADARG;
   SendACK;
}

void getDCBchanMask(int ch)
{
   if((ch < 1) || (ch > 32)) BADARG;
   SendACKonly;
   if(SerialMute) return;
   if((DCBtstMask[(ch-1)/8] & (1 << ((ch-1) % 8))) != 0) serial->println("TRUE");
   else serial->println("FALSE");
}

