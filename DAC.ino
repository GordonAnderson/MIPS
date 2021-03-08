//
// DAC module driver.
//
//  This is a low voltage output only module, no readbacks are provided. Intended for control signal generation.
//  This module uses a 8 channel 16 bit DAC. Channels 0 through 7 have -10 to 10 volt single ended outputs and channel
//  7 is a 0 to 10 volt differential output.
//  This interface allows the user to define engineering units for each channel this include a channel name as well.
//
//#include "DAC.h"
#include "Variants.h"
//#include "Hardware.h"
//#include "Table.h"
//#include "Errors.h"

//MIPS Threads
Thread DACthread  = Thread();

int   NumberOfDACchannels = 0;
int   DACcalChannel=1;
int   SelectedDACboard=0;    // Active board, 0 or 1
bool  DACupdate = true;      // Flag set to update all DCbias channels
int   DACmodule = 1;         // Selected DAC module

DACdata  *DACarray[2] = {NULL,NULL};       // Pointer to dac data arrays, one for each module. Allocated at init time
DACdata  dacd;                             // Copy of the selected channels data, must be static used in menu structs

DACstate *DACstates[2] = {NULL,NULL};

extern DialogBoxEntry DACdialogEntriesPage2[];

DialogBoxEntry DACdialogEntriesPage1[] = {
  {"Module"             , 1, 0, D_INT      , 1, 1, 1, 20, false, "%2d", &DACmodule, NULL, DACmoduleSelected},
  {"1"                  , 1, 1, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[0].Value, NULL, NULL},
  {"2"                  , 1, 2, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[1].Value, NULL, NULL},
  {"3"                  , 1, 3, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[2].Value, NULL, NULL},
  {"4"                  , 1, 4, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[3].Value, NULL, NULL},
  {"5"                  , 1, 5, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[4].Value, NULL, NULL},
  {"6"                  , 1, 6, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[5].Value, NULL, NULL},
  {"7"                  , 1, 7, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[6].Value, NULL, NULL},
  {"8"                  , 1, 8, D_FLOAT    , 0, 0, 0, 17, false, NULL, &dacd.DACCD[7].Value, NULL, NULL},
  {"Next page"          , 1, 10, D_PAGE    , 0, 0, 0, 0 , false, NULL, DACdialogEntriesPage2, NULL, NULL},
  {"Return to main menu", 1, 11, D_MENU    , 0, 0, 0, 0 , false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DACdialogEntriesPage2[] = {
  {"Calibrate channel"  , 1, 1, D_INT, 1, 8, 1, 20, false, "%2d", &DACcalChannel, NULL, DACchanCal},
  {"Save settings"      , 1, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveDACsettings, NULL},
  {"Restore settings"   , 1, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreDACsettings, NULL},
  {"First page"         , 1, 9, D_PAGE, 0, 0, 0, 0, false, NULL, DACdialogEntriesPage1, NULL, NULL},
  {"Return to main menu", 1, 10,D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox DACdialog = {
  {
    "DAC parameters",
    ILI9340_BLACK,ILI9340_WHITE,2,0, 0,300, 220,B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,DACdialogEntriesPage1
};

MenuEntry MEDACmonitor = {" DAC module", M_DIALOG,0,0,0,NULL,&DACdialog,NULL,NULL};

// This function finds a channel by name and returns its array index and 
// channel index. Returns 0 if found and -1 if not found
int DACname2Indexs(char *name, int *index, int *chan)
{
   String   Target,Name;

   Target = name;
   for(*index=0; *index < 2; (*index)++)
   {
     if(DACarray[*index] != NULL)
     {
       for(*chan=0; *chan < 8; (*chan)++)
       {
         Name = DACarray[*index]->DACCD[*chan].Name;
         if(Target == Name) return(0);
       }
     }
   }
   return(-1);
}

int DACmodule2Index(int module)
{
  if(module == 1)
  {
    if(DACarray[0] != NULL) return(0);
    else if(DACarray[1] != NULL) return(1);
    else return(-1);
  }
  else if(module == 2)
  {
    if((DACarray[0] != NULL) && (DACarray[1] != NULL)) return(1);
    else return(-1);    
  }
  return(-1);
}

// This function is called when the user selects a DAC module number
void DACmoduleSelected(void)
{
  // Determine the board address for the selected module
  if(DACmodule == 1)
  {
    if(DACarray[0] != NULL) SelectedDACboard = 0;
    else if(DACarray[1] != NULL) SelectedDACboard = 1;
    else return;
  }
  else
  {
    if((DACarray[0] != NULL) && (DACarray[1] != NULL)) SelectedDACboard = 1;
    else return;    
  }
  // Update the dialog box data
  dacd = *DACarray[SelectedDACboard];
  SetupDACentry(&dacd, &DACdialogEntriesPage1[1]);
  // Repaint the display
  DialogBoxDisplay(&DACdialog);
}

// This function sends a control byte to the control CPLD
void SendDACcpldCommand(int8_t addr, int8_t cmd)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Set the SPI address
  SetAddress(addr);
  // Send data
  SPI.setDataMode(SPI_CS, SPI_MODE2);
  SPI.transfer(SPI_CS, cmd);
  // Pulse strobe and exit
  digitalWrite(DACstrobe,HIGH);
  digitalWrite(DACstrobe,LOW);
  SetAddress(0);  
}

// Calibrate the selected channel. Always calibrate in voltage.
// Channel 1 thru 7 are -10 to 10
// Channel 8 is 0 to 10
void DACchanCal(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;

  b = SelectedDACboard;
  SelectBoard(b);
  // Enable the DAC
  SendDACcpldCommand(DACarray[b]->CPLDspi, (1 << DACcpldSELECTED) | (1 << DACcpldDACenable));
  // Set up the calibration data structure
  CC.ADCpointer = NULL;
  if(DACcalChannel <= 7) CC.Min = -10;
  else CC.Min = 0;
  CC.Max = 10;
  CC.DACaddr=DACarray[b]->DACspi;  
  CC.ADCaddr=0;
  CC.DACout=&DACarray[b]->DACCD[DACcalChannel-1].DCctrl;
  CC.ADCreadback=NULL;
  // Define this channels name
  sprintf(Name,"       Channel %2d",DACcalChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
  dacd = *DACarray[SelectedDACboard];
  // Disable the DAC
  SendDACcpldCommand(DACarray[b]->CPLDspi, 0);
}

void SaveDACsettings(void)
{
  bool Success = true;
  
  if(DACarray[SelectedDACboard] != NULL)
  {
    SelectBoard(SelectedDACboard);
    if(WriteEEPROM(DACarray[SelectedDACboard], DACarray[SelectedDACboard]->EEPROMadr, 0, sizeof(DACdata)) != 0) Success = false;
  } 
  if(Success)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Error saving!",2000);
}

void RestoreDACsettings(void)
{
  RestoreDACsettings(false);
}

void RestoreDACsettings(bool NoDisplay)
{
  int b;
  DACdata dac_data;
  bool Success=true;
  bool Corrupted=false;
  
  b = SelectedDACboard;
  if(DACarray[b] == NULL) return;
  SelectBoard(b);
  if(ReadEEPROM(&dac_data, DACarray[b]->EEPROMadr, 0, sizeof(DACdata)) == 0)
  {
     if(strcmp(dac_data.Name,DACarray[b]->Name) == 0)
     {
       // Here if the name matches so copy the data to the operating data structure
       dac_data.EEPROMadr = DACarray[b]->EEPROMadr;
       memcpy(DACarray[b], &dac_data, sizeof(DACdata));
     } 
     else Corrupted=true;
  }
  else Success=false;
  dacd = *DACarray[SelectedDACboard];
  SetupDACentry(&dacd, &DACdialogEntriesPage1[1]);
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
  if(ActiveDialog == &DACdialog) DialogBoxDisplay(&DACdialog);
}

// This function sets up the dialog structure for the channel selected.
// The 8 channels must be the first 8 entries in the dialog entries
// structure.
// - Load the channel names
// - Load the min and max
// - Define the format string
// - Define the step size
void SetupDACentry(DACdata *dc, DialogBoxEntry *dbe)
{
  int  i;
  static char *Format1 = "%5.2f"; 
  static char *Format2 = "%5.1f"; 
  static char *Format3 = "%5.0f"; 

  for(i=0;i<8;i++)
  {
    dbe[i].Name = dc->DACCD[i].Name; 
    dbe[i].Min = dc->DACCD[i].Min; 
    dbe[i].Max = dc->DACCD[i].Max;
    if((dbe[i].Max - dbe[i].Min) <= 2)
    {
      dbe[i].StepSize = 0.01;
      dbe[i].fmt = Format1;
    }
    else if((dbe[i].Max - dbe[i].Min) <= 20)
    {
      dbe[i].StepSize = 0.1;
      dbe[i].fmt = Format2;      
    }
    else
    {
      dbe[i].StepSize = 1;
      dbe[i].fmt = Format3;      
    }
  }
}

void DACvalue2voltage(DACchannellData *dcd, int8_t ch)
{
  float m,b;

  // Calculate the slope and offset for the conversion.
  // Value * m + b = Voltage
  m = 10 / (dcd[ch].Max - dcd[ch].Min);
  if(ch < 7) m *= 2;
  b = 10 - dcd[ch].Max * m;
  // Calculate the voltage
  dcd[ch].VoltageSetpoint = dcd[ch].Value * m + b;
}

// This function is called at powerup to initiaize the DAC modules(s).
void DAC_init(int8_t Board, int8_t addr)
{
  DialogBoxEntry *de = GetDialogEntries(DACdialogEntriesPage1, "Module");
  
  // Allocate the module data structure based on board number passed
  DACarray[Board] = new DACdata;
  DACstates[Board] = new DACstate;
  SelectBoard(Board);
  // Fill the array with default data
  *DACarray[Board] = DAC_Rev1;
  DACarray[Board]->EEPROMadr = addr;
  // Restore parameters from module's memory
  if(NormalStartup) RestoreDACsettings(true);
  // Setup menu and start the thread
  if(NumberOfDACchannels == 0)
  {
    pinMode(DACstrobe, OUTPUT);
    digitalWrite(DACstrobe, LOW);
    SelectedDACboard = Board;
    SelectBoard(SelectedDACboard);
    // Setup the min and max values in the user interface
    SetupDACentry(&dacd, &DACdialogEntriesPage1[1]);
    // Setup the menu
    AddMainMenuEntry(&MEDACmonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&DACdialog);
    // Configure Threads
    DACthread.setName("DAC");
    DACthread.onRun(DAC_loop);
    DACthread.setInterval(100);
    // Add threads to the controller
    control.add(&DACthread);
  }
  NumberOfDACchannels += 8;
  if(NumberOfDACchannels > 8) de->Max = 2;
  SelectBoard(SelectedDACboard);
}

void DAC_loop(void)
{
  int8_t  b,ch;
  bool    Changed;
  
  if (ActiveDialog == &DACdialog)
  {
    if(ActiveDialog->Changed)
    {
      // Here if the user has made a change through the user interface
      *DACarray[SelectedDACboard] = dacd;
      ActiveDialog->Changed = false;
    }
  }
  // Process both modules
  for(b=0;b<2;b++)
  {
    if(DACarray[b] == NULL) continue;
    SelectBoard(b);
    // Look for a changed value and set the update flag is found
    Changed = false;
    for(ch=0;ch<8;ch++) if(DACarray[b]->DACCD[ch].Value != DACstates[b]->Value[ch]) Changed = true;
    // If a change is found then enable the DAC and update needed channels
    if((Changed) || (DACupdate))
    {
      // Enable the DAC
      SendDACcpldCommand(DACarray[b]->CPLDspi, (1 << DACcpldSELECTED) | (1 << DACcpldDACenable));
      // Process changes
      for(ch=0;ch<8;ch++)
      {
        if((DACarray[b]->DACCD[ch].Value != DACstates[b]->Value[ch]) || (DACupdate))
        {
          // Calculate the voltage from the value
          DACvalue2voltage(DACarray[b]->DACCD,ch);
          DACstates[b]->Value[ch] = DACarray[b]->DACCD[ch].Value;
          // Send the data to the DAC
          AD5668(DACarray[b]->DACspi,DACarray[b]->DACCD[ch].DCctrl.Chan,Value2Counts(DACarray[b]->DACCD[ch].VoltageSetpoint,&DACarray[b]->DACCD[ch].DCctrl),3);
        }
      }
      // Disable the DAC
      SendDACcpldCommand(DACarray[b]->CPLDspi, 0);
    }
  }
  // Select the selected board
  SelectBoard(SelectedDACboard);
  // Write any changes back to display structure
  dacd = *DACarray[SelectedDACboard];
  // Update the display if needed
  if (ActiveDialog == &DACdialog) RefreshAllDialogEntries(&DACdialog);
  DACupdate = false;
}

// 
// DAC module serial command processing routines.
//
// SDACV,name,value    Set the dac channel value
// GDACV,name          Return the dac channel value
// SDACMAX,name,val    Set the DAC's maximum value
// GDACMAX,name        Returns the DAC's maximum value
// SDACMIN,name,val    Set the DAC's minimum value
// GDACMIN,name        Returns the DAC's minimum value
// SDACUN,name,units   Set the channel units
// GDACUN,name         Return the channel units
// SDACNM,mn,ch,name   Set the channel name
// GDACNM,mn,ch        Return the channel name
// 

// Returns the number of DAC output channels
void ReportDACchannels(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfDACchannels);
}

void SetDACValue(char *name, char *value)
{
  String Value;
  int    i,ch;
  float  val;

  Value = value;
  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  val = Value.toFloat();
  if((val > DACarray[i]->DACCD[ch].Max) || (val < DACarray[i]->DACCD[ch].Min))
  {
    SetErrorCode(ERR_VALUERANGE);
    SendNAK;
    return;    
  }
  DACarray[i]->DACCD[ch].Value = val;
  SendACK;
}

void GetDACValue(char *name)
{
  int    i,ch;

  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(!SerialMute) serial->println(DACarray[i]->DACCD[ch].Value,3);
}

void SetDACMax(char *name, char *value)
{
  String Value;
  int    i,ch;

  Value = value;
  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  DACarray[i]->DACCD[ch].Max = Value.toFloat();
  SendACK;
}

void GetDACMax(char *name)
{
  int    i,ch;

  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(!SerialMute) serial->println(DACarray[i]->DACCD[ch].Max);
}

void SetDACMin(char *name, char *value)
{
  String Value;
  int    i,ch;

  Value = value;
  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  DACarray[i]->DACCD[ch].Min = Value.toFloat();
  SendACK;
}

void GetDACMin(char *name)
{
  int    i,ch;

  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(!SerialMute) serial->println(DACarray[i]->DACCD[ch].Min);
}

void SetDACUnits(char *name, char *value)
{
  int    i,ch;

  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  strncpy(DACarray[i]->DACCD[ch].Units, value,6);
  DACarray[i]->DACCD[ch].Units[5] = 0;
  SendACK;
}

void GetDACUnits(char *name)
{
  int    i,ch;

  if(DACname2Indexs(name, &i, &ch) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(!SerialMute) serial->println(DACarray[i]->DACCD[ch].Units);
}

void SetDACName(int module, int channel, char *Name)
{
  int index;

  index = DACmodule2Index(module);
  if((index == -1) || (channel < 1) || (channel > 8))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  strncpy(DACarray[index]->DACCD[channel - 1].Name, Name,6);
  DACarray[index]->DACCD[channel - 1].Name[5] = 0;
  SendACK;
}

void GetDACName(int module, int channel)
{
  int index;

  index = DACmodule2Index(module);
  if((index == -1) || (channel < 1) || (channel > 8))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  SendACKonly;
  if(!SerialMute) serial->println(DACarray[index]->DACCD[channel - 1].Name);
}
