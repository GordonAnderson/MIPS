//
// File: HOFAIMS
//
// This file provides the function to control the high order FAIMS system, HOFAIMS. The HOFAIMS system
// adds two additional harmonics (4th and 5th) to produce a more square FAIMS waveform. The HOFAIMS 
// controller holds two of these modules and is linked to a FAIMS system. A rev 3 FAIMS controller
// is required in the FAIMS system and it provides the clocks used by these modules.
//
// The "T" input is monitored and if its detected high then the global enable is cleared to
// turn off the HOFAIMS drivers.
//
// Features:
//   1.) Power monitoring and limiting logic
//   2.) Drive level limiting
//   2.) Extern disable input to shut down the drivers, input T
//
// Gordon Anderson
// October 29, 2016
//
#include "HOFAIMS.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

#if HOFAIMSmodule
//MIPS Threads
Thread HOFAIMSthread  = Thread();

#define HOFAIMS HOFAIMSarray[SelectedHOFAIMSboard]

HOFAIMSdata  HOFAIMSarray[2] = {HOFAIMS_Rev_1, HOFAIMS_Rev_1};
HOFAIMSdata  hofaims;   // This is the display / UI data structure

// The servo timer runs with a 656,250 Hz and is set to with a terminal
// count of 32,000 to generate a ~20 Hz rep rate, 
MIPStimer ServoTMR(TMR_servos); 
int ServoCount[2] = {ServoMaxTimerCount - 1000,ServoMaxTimerCount - 1000};

int   HOFAIMSmodule            = 1;
int   NumberOfHOFAIMSchannels  = 0;
bool  HOFAIMSboards[2]         = {false, false};
int   SelectedHOFAIMSboard     = 0;    // Active board, 0 or 1 = A or B
int   HOFAIMSPWMchan[2]        = {5,7};
int   CurrentHOFAIMSmodule     = -1;

// System parameters
bool  HOFAIMSenable = false;
float HOFAIMSdrive = 10.0;

// ADC monitor values
float HOFAIMSmonitors[2][4];
float HOFAIMSKVout;
float HOFAIMSPower;
float HOFAIMSTotalPower;

extern DialogBoxEntry HOFAIMSentriesPage1[];
extern DialogBoxEntry HOFAIMSentriesLimits[];


DialogBoxEntry HOFAIMSentries[] = {
  {" Enable"             , 0, 1, D_ONOFF, 0, 1  , 1, 20, false, NULL, &HOFAIMSenable, NULL, NULL},
  {" Drive"              , 0, 2, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &HOFAIMSdrive, NULL, NULL},
  {" RF, KV"             , 0, 3, D_FLOAT   , 0, 1, 1, 10, true, "%5.2f", &HOFAIMSmonitors[0][2], NULL, NULL},
  {""                    , 0, 3, D_FLOAT   , 0, 1, 1, 18, true, "%5.2f", &HOFAIMSmonitors[1][2], NULL, NULL},  
  {" Power"              , 0, 4, D_FLOAT   , 0, 0, 0, 20, true, "%3.0f", &HOFAIMSTotalPower, NULL, NULL},
  {" Tune"               , 0, 8, D_PAGE,  0, 0  , 0, 0,  false, NULL, &HOFAIMSentriesPage1, NULL, NULL},
  {" Save all"           , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveHOFAIMSall, NULL},
  {" Restore all"        , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreHOFAIMSall, NULL},
  {" Return to main menu", 0,11, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry HOFAIMSentriesPage1[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &HOFAIMSmodule, NULL, NULL},
  {" Enable"             , 0, 2, D_ONOFF, 0, 1  , 1, 20, false, NULL, &hofaims.Enable, NULL, NULL},
  {" Drive"              , 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &hofaims.Drive, NULL, NULL},
  {" RF, KV"             , 0, 4, D_FLOAT   , 0, 1, 1, 18, true, "%5.2f", &HOFAIMSKVout, NULL, NULL},
  {" Power"              , 0, 5, D_FLOAT   , 0, 0, 0, 20, true, "%3.0f", &HOFAIMSPower, NULL, NULL},
  {" Capacitance"        , 0, 6, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &hofaims.Cap, NULL, NULL},
  {" Phase"        ,       0, 7, D_INT, 0, 255, 1, 20, false, "%3d", &hofaims.Delay, NULL, NULL},
  {" Limits"             , 0, 8, D_PAGE,  0, 0  , 0, 0,  false, NULL, &HOFAIMSentriesLimits, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveHOFAIMSsettings, NULL},
  {" Restore settings"   , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreHOFAIMSsettings, NULL},
  {" Return first page"  , 0,11, D_PAGE,     0, 0, 0, 0, false, NULL, &HOFAIMSentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry HOFAIMSentriesLimits[] = {
  {" Max power"          , 0,1, D_FLOAT , 1, 100 , 1, 18, false, "%5.1f", &hofaims.MaxPower, NULL, NULL},
  {" Max drive"          , 0,2, D_FLOAT , 0, 100 , 1, 18, false, "%5.1f", &hofaims.MaxDrive, NULL, NULL},
  {" Return"             , 0,9, D_PAGE,     0, 0, 0, 0, false, NULL, &HOFAIMSentriesPage1, NULL, NULL},
  {NULL},
};

DialogBox HOFAIMSdialog = {
  {
    "HOFAIMS parameters",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0,false, HOFAIMSentries
};

MenuEntry MEHOFAIMSmodule = {" HOFAIMS module", M_DIALOG, 0, 0, 0, NULL, &HOFAIMSdialog, NULL, NULL};

void SaveHOFAIMSall(void)
{
  int  sb = SelectedHOFAIMSboard;
  bool stat;

  SelectedHOFAIMSboard = 0;
  hofaims = HOFAIMS;
  if(stat = SaveHOFAIMSsettings(true))
  {
     SelectedHOFAIMSboard = 1;
     hofaims = HOFAIMS;
     if(stat = SaveHOFAIMSsettings(true)) DisplayMessage("Parameters Saved!", 2000);
  }
  if(!stat) DisplayMessage("Unable to Save!", 2000);
  SelectedHOFAIMSboard = sb;
  SelectBoard(SelectedHOFAIMSboard);
  hofaims = HOFAIMS;
}

void RestoreHOFAIMSall(void)
{
  int  sb = SelectedHOFAIMSboard;
  bool stat;

  SelectedHOFAIMSboard = 0;
  if(stat = RestoreHOFAIMSsettings(true))
  {
    SelectedHOFAIMSboard = 1;
    if(stat = RestoreHOFAIMSsettings(true)) DisplayMessage("Parameters Restored!", 2000);
  }
  if(!stat) ("Unable to Restore!", 2000);
  SelectedHOFAIMSboard = sb;
  SelectBoard(SelectedHOFAIMSboard);
  hofaims = HOFAIMS;
}

// Write the current board parameters to the EEPROM on the HOFAIMS board.
bool SaveHOFAIMSsettings(bool NoDisplay)
{
  SelectBoard(SelectedHOFAIMSboard);
  hofaims.Size = sizeof(HOFAIMSdata);
  if (WriteEEPROM(&hofaims, hofaims.EEPROMadr, 0, sizeof(HOFAIMSdata)) == 0)
  {
    if (!NoDisplay) DisplayMessage("Parameters Saved!", 2000);
    return(true);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Save!", 2000);
  return(false);
}

void SaveHOFAIMSsettings(void)
{
  SaveHOFAIMSsettings(false);
}

bool RestoreHOFAIMSsettings(bool NoDisplay)
{
  HOFAIMSdata ho;

  SelectBoard(SelectedHOFAIMSboard);
  if (ReadEEPROM(&ho, HOFAIMS.EEPROMadr, 0, sizeof(HOFAIMSdata)) == 0)
  {
    if (strcmp(ho.Name, HOFAIMS.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (ho.Size > sizeof(HOFAIMSdata)) ho.Size = sizeof(HOFAIMSdata);
      ho.EEPROMadr = HOFAIMS.EEPROMadr;
      memcpy(&HOFAIMS, &ho, ho.Size);
      hofaims = HOFAIMS;
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      return(true);
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
  return(false);
}

void RestoreHOFAIMSsettings(void)
{
  RestoreHOFAIMSsettings(false);
}

// Update the dialog box and display
void SelectHOFAIMSmodule(bool paint = true)
{
  if(HOFAIMSmodule == 1)
  {
    if(HOFAIMSboards[0]) SelectedHOFAIMSboard = 0;
    else SelectedHOFAIMSboard = 1;
  }
  if(HOFAIMSmodule == 2) SelectedHOFAIMSboard = 1;
  SelectBoard(SelectedHOFAIMSboard);
  hofaims = HOFAIMS;
  if(paint) 
  {
    DialogBoxDisplay(&HOFAIMSdialog);
    HOFAIMSdialog.State = M_SCROLLING;
  }
}

static int servo = 0;
bool ServosEnabled = false;

void ServoActive(void)
{
  if(!ServosEnabled) return;
  if(servo == 0)
  {
    digitalWrite(Servo1,LOW);
    ServoTMR.setRA(ServoCount[0]+500);
    servo = 1;
  }
  else if(servo == 1)
  {
    digitalWrite(Servo1,HIGH);
    digitalWrite(Servo2,LOW);
    ServoTMR.setRA(ServoCount[0]+ServoCount[1]+500);    
    servo = 2;
  }
  else
  {
    servo = 0;
    digitalWrite(Servo1,HIGH);
    digitalWrite(Servo2,HIGH); 
   }
}

void ServoLow(void)
{
  digitalWrite(Servo1,HIGH);
  digitalWrite(Servo2,HIGH); 
  ServoTMR.setRA(500);
  servo = 0; 
}

void SetDelay(int Board, int dVal)
{
   SelectBoard(Board);
   SetAddress(HOFAIMSarray[Board].DELAYspi);
   SPI.setDataMode(SPI_CS, SPI_MODE2);
   SPI.transfer(SPI_CS, dVal);
   SetAddress(0);
   serial->println(dVal);
}

// This function is called on power up to init the module
void HOFAIMS_init(int8_t Board, int8_t addr)
{
  // Flag the board as present
  HOFAIMSboards[Board] = true;
  // Set active board to board being inited
  SelectedHOFAIMSboard = Board;
  SelectBoard(Board);
  HOFAIMS.EEPROMadr = addr;
  // If normal startup load the EEPROM parameters from the Filament card.
  if (NormalStartup)
  {
    RestoreHOFAIMSsettings(true);
  }
  HOFAIMS.EEPROMadr = addr;
  // Init the hardware here...
  pinMode(Servo1,OUTPUT);
  pinMode(Servo2,OUTPUT);
  digitalWrite(Servo1,LOW);
  digitalWrite(Servo1,LOW);
  // Setup the PWM outputs and set levels
  analogWriteResolution(12);
  pinMode(HOFAIMSPWMchan[Board], OUTPUT);
  analogWrite(HOFAIMSPWMchan[Board], 0);
  // Setup the menu if this is the very first call to this init function
  if (NumberOfHOFAIMSchannels == 0)
  {
    hofaims = HOFAIMS;
    AddMainMenuEntry(&MEHOFAIMSmodule);
    SelectHOFAIMSmodule(true);
    CurrentHOFAIMSmodule = HOFAIMSmodule;
    if (ActiveDialog == NULL) ActiveDialog = &HOFAIMSdialog;
    // Configure Threads
    HOFAIMSthread.setName("HOFAIMS");
    HOFAIMSthread.onRun(HOFAIMS_loop);
    HOFAIMSthread.setInterval(100);
    // Add threads to the controller
    control.add(&HOFAIMSthread);
    // Setup the timer and ISR for the servo control
    ServoTMR.begin();  
    ServoTMR.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);
    ServoTMR.attachInterruptRA(ServoActive);
    ServoTMR.attachInterrupt(ServoLow);
    ServoTMR.setRC(ServoMaxTimerCount);
    ServoTMR.setTIOAeffect(ServoCount[0],TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE);
    ServoTMR.enableTrigger();
    ServoTMR.softwareTrigger();
  }
  else
  {
    // Set back to first board
    SelectedHOFAIMSboard = 0;
    SelectBoard(0);
  }
  NumberOfHOFAIMSchannels += 1; 
  // Set the maximum number of channels in the selection menu
  HOFAIMSentriesPage1[0].Max = NumberOfHOFAIMSchannels;

/*
  HOFAIMSarray[0].RFmon.m = 53571;
  HOFAIMSarray[0].RFmon.b = 11000;
  HOFAIMSarray[1].RFmon.m = 62500;
  HOFAIMSarray[1].RFmon.b = 13000;
*/
}

// This is the HOFAIMS main processing loop called every 100 mS by
// the thread controller
void HOFAIMS_loop(void)
{
  int        b,i;
  uint16_t   HOFAIMSADCvals[4];
  static int CurrentDelay[2] = {-1,-1};
  static bool ShutdownArmed = false;
  float      Drives[2];

  if (ActiveDialog == &HOFAIMSdialog)
  {
    if(ActiveDialog->Changed)
    {
      HOFAIMS = hofaims;
    }
    if(CurrentHOFAIMSmodule != HOFAIMSmodule)
    {
      SelectHOFAIMSmodule();
      CurrentHOFAIMSmodule = HOFAIMSmodule;
    }
  }
  // Calculate the drive levels. This allows a global drive level to control both channels and keep the 
  // ratio fixed is posible.
  if(HOFAIMSarray[0].Drive > HOFAIMSarray[1].Drive)
  {
    Drives[0] = HOFAIMSdrive;
    Drives[1] = HOFAIMSdrive * HOFAIMSarray[1].Drive/HOFAIMSarray[0].Drive;
  }
  else
  {
    Drives[1] = HOFAIMSdrive;
    Drives[0] = HOFAIMSdrive * HOFAIMSarray[0].Drive/HOFAIMSarray[1].Drive;    
  }
  for(b = 0; b < 2; b++)
  {
    if(HOFAIMSboards[b])
    {
      SelectBoard(b);
      // Update the servo position
      ServoCount[b] = (HOFAIMSarray[b].Cap * 12.0 + 900) * ServoTimePerCount;
      // Update the drive level
      if((HOFAIMSarray[b].Enable) && (HOFAIMSenable)) analogWrite(HOFAIMSPWMchan[b], (Drives[b] * PWMFS) / 100);
      else analogWrite(HOFAIMSPWMchan[b], 0);
      // Set the delay line data
      if(CurrentDelay[b] != HOFAIMSarray[b].Delay)
      {
        SetDelay(b,HOFAIMSarray[b].Delay);
        CurrentDelay[b] = HOFAIMSarray[b].Delay;
      }
      // Read the monitor inputs and update the display buffer
      ValueChange = false;
      delay(1);
      if(AD7994(HOFAIMSarray[b].ADCadr, (uint16_t *)HOFAIMSADCvals)==0)
      {
        if(!ValueChange)
        {
           i = HOFAIMSarray[b].Vmon.Chan;
           HOFAIMSmonitors[b][i] = Filter * Counts2Value(HOFAIMSADCvals[i],&HOFAIMSarray[b].Vmon) + (1-Filter) * HOFAIMSmonitors[b][i];
           i = HOFAIMSarray[b].Imon.Chan;
           HOFAIMSmonitors[b][i] = Filter * Counts2Value(HOFAIMSADCvals[i],&HOFAIMSarray[b].Imon) + (1-Filter) * HOFAIMSmonitors[b][i];
           i = HOFAIMSarray[b].RFmon.Chan;
           HOFAIMSmonitors[b][i] = Filter * Counts2Value(HOFAIMSADCvals[i],&HOFAIMSarray[b].RFmon) + (1-Filter) * HOFAIMSmonitors[b][i];
           if(b == SelectedHOFAIMSboard) 
           {
              HOFAIMSPower = HOFAIMSmonitors[b][0] * HOFAIMSmonitors[b][1];
              HOFAIMSKVout = HOFAIMSmonitors[b][2];
           }
        }
      }
    }
    // Power and drive limit testing
    if(HOFAIMSarray[b].Drive > HOFAIMSarray[b].MaxDrive) HOFAIMSarray[b].Drive = HOFAIMSdrive -= 1.0;
    if((HOFAIMSmonitors[b][0] * HOFAIMSmonitors[b][1]) > HOFAIMSarray[b].MaxPower) HOFAIMSdrive -= 1.0;
    if(HOFAIMSdrive < 0) HOFAIMSdrive=0;
  }
  HOFAIMSTotalPower = HOFAIMSmonitors[0][0] * HOFAIMSmonitors[0][1] + HOFAIMSmonitors[1][0] * HOFAIMSmonitors[1][1];
  SelectBoard(SelectedHOFAIMSboard);
  if (ActiveDialog == &HOFAIMSdialog) RefreshAllDialogEntries(&HOFAIMSdialog);
  hofaims = HOFAIMS;
  ServosEnabled = true;
  // Remote shutdown processing. If the T input goes high then the remote
  // shutdown is armed and the HOFAIMS will disable when T goes low.
  if(HOFAIMSenable == true)
  {
    if(digitalRead(DI3) == HIGH) ShutdownArmed = true;
    if((ShutdownArmed) && (digitalRead(DI3) == LOW)) 
    {
      if (ActiveDialog != NULL) ActiveDialog->State = M_SCROLLING;
      HOFAIMSenable = false;
      DisplayMessage("External stop!", 2000);
    }
  }
  else ShutdownArmed = false;
}
#endif
