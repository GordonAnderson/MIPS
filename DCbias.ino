//
// DCbias
//
// This file supports the DC bias modules on the MIPS system. The MIPS controller
// supports up to two DCbias modules.
//
// Gordon Anderson
//
#include "DCbias.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

extern DialogBox DCbiasDialog;

extern bool NormalStartup;

// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.5         // Weak filter coefficent
#define  StrongFilter 0.05  // Strong filter coefficent

//MIPS Threads
Thread DCbiasThread  = Thread();

#ifdef TestMode
int  NumberOfDCChannels = 8; // Defines the number of DC channels supported. Set during intaliztion.
                             // valid values are 0, 8, or 16
bool  DCbiasBoards[2] = {true,false};  // Defines the boards that are present in the system
#else
int  NumberOfDCChannels = 0;
bool  DCbiasBoards[2] = {false,false};  
#endif
int   SelectedDCBoard=0;     // Active board, 0 or 1 = A or B
int   CalChannel=1;          // User selected channel to calibrate
float Readbacks[2][8];       // Monitor voltage readback storage
float Readback[8];           // Displayed readback buffer
float MaxDCbiasVoltage=0;    // This value is set to the highest DC bias voltage
float Verror = 0;            // Used to detect if the DC bias supply is "stressed"
float VerrorFiltered = 0;    // Used to trip supply is error level is exceeded
int   MonitorDelay;          // Delay a number of loop interations before monitoring voltages for tripping PS

#define DCbD DCbDarray[SelectedDCBoard]

DCbiasData  DCbDarray[2] = {DCbD_250_Rev_1,DCbD_50_Rev_1};

DCbiasData dcbd;    // Holds the selected channel's data

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
  M_SCROLLING,0,0,DCDialogEntriesPage1
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
  
  if((dc->MaxVoltage - dc->MinVoltage) <= 100)
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
}

void SetNextDCbiasPage(void)
{
  int i;
  
  // Go to page 1a if we have 16 output channel, 2 boards
  if((DCbiasDialog.Entry == DCDialogEntriesPage1) && (NumberOfDCChannels == 16))
  {
    DCbiasDialog.Entry = DCDialogEntriesPage1a;
    SelectedDCBoard = 1;
    SelectBoard(SelectedDCBoard);
    // Init the dcbd structure
    dcbd = DCbD;
    // Setup the min and max values in the user interface
    SetupEntry(&dcbd, DCDialogEntriesPage1a);
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
  if(DCbiasBoards[0]) SelectedDCBoard=0;
  else if(DCbiasBoards[1]) SelectedDCBoard=1;
  SelectBoard(SelectedDCBoard);
  // Init the dcbd structure
  dcbd = DCbD;
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
  return(DCbiasBoards[Board]);
}

// This function allows calibration of the seleted channel
void DCbiasChanCal(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  
  if(CalChannel > 8) b = 1;
  SelectedDCBoard = b;
  if((CalChannel <= 8) && (!DCbiasBoards[b]) && DCbiasBoards[1]) b = 1;
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b].MinVoltage;
  CC.Max=DCbDarray[b].MaxVoltage;
  CC.DACaddr=DCbDarray[b].DACspi;  
  CC.ADCaddr=DCbDarray[b].ADCadr;
  CC.DACout=&DCbDarray[b].DCCD[(CalChannel-1) & 7].DCctrl;
  CC.ADCreadback=&DCbDarray[b].DCCD[(CalChannel-1) & 7].DCmon;
  // Define this channels name
  sprintf(Name,"       Channel %2d",CalChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  dcbd = DCbD;
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
  
  if(CalChannel > 8) b = 1;  
  SelectedDCBoard = b;
  if((CalChannel <= 8) && (!DCbiasBoards[b]) && DCbiasBoards[1]) b = 1;
  SelectBoard(b);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min=DCbDarray[b].MinVoltage;
  CC.Max=DCbDarray[b].MaxVoltage;
  CC.DACaddr=DCbDarray[b].DACadr;  
  CC.ADCaddr=DCbDarray[b].ADCadr;
  CC.DACout=&DCbDarray[b].DCoffset.DCctrl;
  CC.ADCreadback=NULL;
  // Define this channels name
  if(CalChannel <= 8) sprintf(Name," Offset for channel 1-8");
  else sprintf(Name,"Offset for channel 9-16");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  dcbd = DCbD;      
}

void SaveDCbiasSettings(void)
{
  bool Success = true;
  
  if(DCbiasBoards[0])
  {
    SelectBoard(0);
    if(WriteEEPROM(&DCbDarray[0], DCbDarray[0].EEPROMadr, 0, sizeof(DCbiasData)) != 0) Success = false;
  } 
  if(DCbiasBoards[1])
  {
    SelectBoard(1);
    if(WriteEEPROM(&DCbDarray[1], DCbDarray[1].EEPROMadr, 0, sizeof(DCbiasData)) != 0) Success = false;
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
  for(b=0;b<2;b++)
  {
    if(!DCbiasBoards[b]) continue;
    SelectBoard(b);
    if(ReadEEPROM(&dcb_data, DCbDarray[b].EEPROMadr, 0, sizeof(DCbiasData)) == 0)
    {
       if(strcmp(dcb_data.Name,DCbDarray[b].Name) == 0)
       {
         // Here if the name matches so copy the data to the operating data structure
         memcpy(&DCbDarray[b], &dcb_data, sizeof(DCbiasData));
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  SelectBoard(SelectedDCBoard);
  dcbd = DCbD;
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
}

// This function converts the value to DAC counts and return the count value.
// This function is used by the time table (pulse sequence generation) function.
// The channel value range is from 0 to maxchannel-1
int DCbiasValue2Counts(int chan, float value)
{
  int brd = -1;
  
  if(chan < 8)
  {
    if(DCbiasBoards[0]) brd = 0;
    else if(DCbiasBoards[1]) brd = 1;
  }
  else if(DCbiasBoards[1]) brd = 1;
  if(brd == -1) return -1;
  chan &= 7;
  return Value2Counts(value - DCbDarray[brd].DCoffset.VoltageSetpoint,&DCbDarray[brd].DCCD[chan].DCctrl);
}

// This function converts the value to DAC counts and return the count value.
// This function is used by the time table (pulse sequence generation) function.
// The channel value range is from 0 to maxchannel-1
float DCbiasCounts2Value(int chan, int counts)
{
  int brd = -1;
  
  if(chan < 8)
  {
    if(DCbiasBoards[0]) brd = 0;
    else if(DCbiasBoards[1]) brd = 1;
  }
  else if(DCbiasBoards[1]) brd = 1;
  if(brd == -1) return -1;
  chan &= 7;
  return Counts2Value(counts, &DCbDarray[brd].DCCD[chan].DCctrl) + DCbDarray[brd].DCoffset.VoltageSetpoint;
}

// This function sends the DAC counts to the selected channel.
// This function is used by the time table generation code.
void DCbiasDACupdate(int chan, int counts)
{
  int brd = -1;
  
  if(counts == -1) return;
  if(chan < 8)
  {
    if(DCbiasBoards[0]) brd = 0;
    else if(DCbiasBoards[1]) brd = 1;
  }
  else if(DCbiasBoards[1]) brd = 1;
  if(brd == -1) return;
  chan &= 7;
  SelectBoard(brd);
  AD5668(DCbDarray[brd].DACspi,DCbDarray[brd].DCCD[chan].DCctrl.Chan,counts);
}

// This function will turn on or off the bias supply when called based on the setting of
// PowerEnable.
void SetPowerSource(void)
{
  if(MIPSconfigData.PowerEnable) 
  {
    // Turn power on
    digitalWrite(PWR_ON,LOW);
    // Clear the voltage error variables
    Verror = VerrorFiltered = 0;
    // Delay output alarm monitoring to let things stabalize
    DelayMonitoring();
  }
  else 
  {
    // Turn power off
    digitalWrite(PWR_ON,HIGH);
  }
}

// Returns true if power is on to HV supply
bool IsPowerON(void)
{
  if(--MonitorDelay > 0) return false;
  if(digitalRead(PWR_ON) == 0) return true;
  return false;
}

// Call this function to cause a 1 sec delay ion monitoring the outputs for the trip logic. This
// needs to be called when a change is made in the output voltages.
void DelayMonitoring(void)
{
  MonitorDelay = 100;
  VerrorFiltered = 0;  // Reset the error filtered value
}

// This function is called at powerup to initiaize the DC bias board(s).
void DCbias_init(int8_t Board)
{
  int i;
  
  // Flag the board as present
  DCbiasBoards[Board] = true;
  // Set active board to board being inited
  SelectedDCBoard = Board;
  SelectBoard(Board);
  // Init the dcbd structure
  dcbd = DCbD;
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreDCbiasSettings(true);
    DCbD = dcbd;        // Copy back into the configuration data structure array
  }
  // Setup the min and max values in the user interface
  SetupEntry(&dcbd, DCDialogEntriesPage1);
  if(NumberOfDCChannels == 0)
  {
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
    // If here we are setting up the second DCbias card so point back to the first one
    SelectedDCBoard = 0;
    SelectBoard(0);
    dcbd = DCbD;
  }
  NumberOfDCChannels += 8;
  DCDialogEntriesPage2[0].Max = NumberOfDCChannels;
  DCDialogEntriesPage2[1].Max = NumberOfDCChannels;
  SetPowerSource();
}

// This function is called every 100 mS to process the DC bias board(s).
void DCbias_loop(void)
{
  float   errorPercentage,V;
  static  int disIndex = 0;
  static  bool SuppliesOff = true;
  static  int  SuppliesStableCount = 10;
  int     i,b;
  uint16_t ADCvals[8];

  // Monitor power on output bit. If power comes on hold all output at zero until power is stable, short delay.
  if(digitalRead(PWR_ON) != 0) 
  {
    // Here is power is off, set supply off flag and delay loop counter
    SuppliesOff = true;
    SuppliesStableCount = 10;
  }
  else
  {
    // Here when supplies are on
    if(--SuppliesStableCount <= 0) SuppliesOff = false;
  }
  // Copy any UI updates, first look to see if any setpoints have changed, if so delay monitoring
  for(i=0;i<DCbD.NumChannels;i++) if(abs(DCbD.DCCD[i].VoltageSetpoint - dcbd.DCCD[i].VoltageSetpoint) > 1.0) DelayMonitoring();
  if(abs(DCbD.DCoffset.VoltageSetpoint - dcbd.DCoffset.VoltageSetpoint) > 1.0) DelayMonitoring();
  DCbD = dcbd;      
  MaxDCbiasVoltage = 0;
  Verror = 0;
  for(b=0;b<2;b++)
  {
    SelectBoard(b);
    if(!DCbiasBoards[b]) continue;
    // Update the offset output, its TWI not SPI!
    if(SuppliesOff == false)
    {
      AD5625(DCbDarray[b].DACadr,DCbDarray[b].DCoffset.DCctrl.Chan,Value2Counts(DCbDarray[b].DCoffset.VoltageSetpoint,&DCbDarray[b].DCoffset.DCctrl));
    }
    else
    {
      // Set to zero if power is off
      AD5625(DCbDarray[b].DACadr,DCbDarray[b].DCoffset.DCctrl.Chan,Value2Counts(0,&DCbDarray[b].DCoffset.DCctrl));      
    }
    // Update all output channels. SPI interface for speed
    for(i=0;i<DCbDarray[b].NumChannels;i++)
    {
      if(SuppliesOff == false)
      {
        V = DCbDarray[b].DCCD[i].VoltageSetpoint - DCbDarray[b].DCoffset.VoltageSetpoint;
        if(V > DCbDarray[b].MaxVoltage) V = DCbDarray[b].MaxVoltage;
        if(V < DCbDarray[b].MinVoltage) V = DCbDarray[b].MinVoltage;
        DCbDarray[b].DCCD[i].VoltageSetpoint = V + DCbDarray[b].DCoffset.VoltageSetpoint;
        AD5668(DCbDarray[b].DACspi,DCbDarray[b].DCCD[i].DCctrl.Chan,Value2Counts(V,&DCbDarray[b].DCCD[i].DCctrl));
      }
      else
      {
        // Set to zero if power is off
        AD5668(DCbDarray[b].DACspi,DCbDarray[b].DCCD[i].DCctrl.Chan,Value2Counts(0,&DCbDarray[b].DCCD[i].DCctrl));
      }
    }
    // Read the monitor inputs and update the display buffer
    if(AD7998(DCbDarray[b].ADCadr, ADCvals)==0) for(i=0;i<DCbDarray[b].NumChannels;i++)
    {
      Readbacks[b][i] = Filter * (Counts2Value(ADCvals[i],&DCbDarray[b].DCCD[i].DCmon) + DCbDarray[b].DCoffset.VoltageSetpoint) + (1-Filter) * Readbacks[b][i];
      if(abs(Readbacks[b][i]) > MaxDCbiasVoltage) MaxDCbiasVoltage = abs(Readbacks[b][i]);
      if(b == SelectedDCBoard) Readback[i] = Readbacks[b][i];
    }
    // Determine the largest error between the output setpoint and the actual value, scan all channels
    // only do this test if the power supply is on and its a 250 volt or higher board
    if(IsPowerON() && (DCbDarray[b].MaxVoltage >= 250)) for(i=0;i<DCbDarray[b].NumChannels;i++)
    {
       errorPercentage = (abs(Readbacks[b][i] - DCbDarray[b].DCCD[i].VoltageSetpoint) / DCbDarray[b].MaxVoltage) * 100.0;
       if(errorPercentage > Verror) Verror = errorPercentage;
    }
  }
  dcbd = DCbD;
  VerrorFiltered = StrongFilter * Verror + (1-StrongFilter) * VerrorFiltered;
  // If the VerrorFiltered value exceeds the threshold then turn off the DC bias power supply and popup a message
  if(IsPowerON())
  {
    // Test the threshold
    if((VerrorFiltered > MIPSconfigData.VerrorThreshold) && (MIPSconfigData.VerrorThreshold > 0))
    {
      // Turn off the power supply
      MIPSconfigData.PowerEnable = false;
      SetPowerSource();
      // Display a popup error message
      DisplayMessageButtonDismiss("Output Voltage Error!");
    }
  }
  SelectBoard(SelectedDCBoard);
  // Display/update the readback values 
  if(ActiveDialog != &DCbiasDialog) return;
  if(DCbiasDialog.Entry == DCDialogEntriesPage2) return;
  if(disIndex < dcbd.NumChannels) DisplayDialogEntry(&DCbiasDialog.w, &DCDialogEntriesActualVoltages[disIndex], false);
  if((ActiveDialog->Selected != disIndex) || (ActiveDialog->State == M_SCROLLING)) if(disIndex < dcbd.NumChannels) DisplayDialogEntry(&ActiveDialog->w, &ActiveDialog->Entry[disIndex], false);
  if(++disIndex >= 8) disIndex = 0;
}

//
// This section contains all the DCbias command processing routines.
//

// This function returns the board number for the channel number passed in.
// chan range is 1 to 16
// returns - 1 on error
int GetDCbiasBoard(int chan)
{
  if((chan < 1) || (chan > 16))
  {
    SetErrorCode(ERR_INVALIDCHAN);
    SendNAK;
    return(-1);
  }
  if(chan > 8)
  { 
    if(DCbiasBoards[1]) return 1;
    SetErrorCode(ERR_INVALIDCHAN);
    SendNAK;
    return -1;
  }
  // If here the channel is 1 through 8
  if(DCbiasBoards[0]) return 0;
  if(DCbiasBoards[1]) return 1;
  SetErrorCode(ERR_INVALIDCHAN);
  SendNAK;
  return(-1);  
}

// This function determines the data structure that holds the data for the channel number passed.
// NULL is returned if it can't be found.
// chan is in the range of 1 to 16
// If there is a channel error this function will send the NAK
// out the serial port.
DCbiasData *GetDCbiasDataPtr(int chan, bool Response = true)
{
  int board;
  
  board = GetDCbiasBoard(chan);
  if(board == -1)
  {
    if(Response)
    {
      SetErrorCode(ERR_INVALIDCHAN);
      SendNAK;
    }
    return NULL;
  }
  return &DCbDarray[board];
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

// Read the selected channel requested voltage
void DCbiasRead(int chan, float *fVal)
{
  DCbiasData *DCbData;
  
  fVal = NULL;
  if(!CheckChannel(chan,false)) return;
  if((DCbData = GetDCbiasDataPtr(chan,false)) == NULL) return;
  fVal = &DCbData->DCCD[(chan-1) & 0x07].VoltageSetpoint;
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
  if(!SerialMute) serial->println(Readbacks[brd][(chan-1) & 0x07]);
}

// Set the float or offset voltage
void DCbiasSetFloat(char *Chan, char *Value)
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
  if(!CheckValue(DCbData, value)) return;
  // Now set the value in the boards data structure and let the processing
  // loop update the outputs.
  DelayMonitoring();
  DCbData->DCoffset.VoltageSetpoint = value;
  // If this channel is being displayed in a dialog then refresh the display
  if(GetDCbiasBoard(chan) == SelectedDCBoard)
  {
    dcbd = DCbD;
    if((ActiveDialog == &DCbiasDialog) && (DCbiasDialog.Entry != DCDialogEntriesPage2))
    {
      DisplayAllDialogEntries(&DCbiasDialog);
      DCbiasDialog.State = M_SCROLLING;
    }
  }
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
      DCbDarray[board].NumChannels = num;
      dcbd = DCbD;
      SendACK;
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

