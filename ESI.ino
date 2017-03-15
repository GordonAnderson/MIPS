//
// ESI
//
// ESI module code. Supports two ESI supplies on each module, one positive and one negative. 
// Two modules can be installed in one MIPS system. These modules also monitor and display
// the actual output voltage and output current.
//
// Updated Feb 22, 2017
//  1.) Added the following capabilities
//      - Enable/disable to allow tripping on over current
//      - Max voltage limit
//      - Max current limit (trip point)
//
// Rev 1.0 of the ESI modules was desiged for the Matsusada 30 watt supplies, a 6KV and -6KV
// model. 
//
// Rev 3.0 of the ESI hardware supports the EMCO C40 and C40N modules and includes relay polarity
// switching. The EMCO modules do not have readbacks or current monitoring. The input power to the
// modules is monitoried and used to estimate load current.
//
// Gordon Anderson
// March 28, 2015
// Revised Feb 22, 2017
// Revised Feb 25, 2017 added the rev 3 updated but not yet tested
//
#include "ESI.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

extern bool NormalStartup;

// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.1         // Weak filter coefficent
#define  StrongFilter 0.05  // Strong filter coefficent

#define ESIstepSize 10

//MIPS Threads
Thread ESIthread  = Thread();

int   NumberOfESIchannels = 0;       // Defines the number of ESI channels supported. Set during intaliztion.
                                     // valid values are 0, 2, or 4
int   ESIchannel = 1;                // Selected channel
int   ESIchannelLoaded = 1;          // ESI channel loaded into working data structures
bool  ESIboards[2] = {false,false};  // Defines the boards that are present in the system
int   SelectedESIboard=0;            // Active board, 0 or 1 = A or B
float MaxESIvoltage=0;               // This value is set to the highest ESI voltage
// Readback monitor buffers
float ReadbackV[2][2];               // Readback voltage [board][channel]
float ReadbackI[2][2];               // Readback currect [board][channel]
float Imonitor[2];                   // Used for rev 3 current readback monitor

#define esidata ESIarray[SelectedESIboard]

ESIdata  ESIarray[2] = {ESI_Rev_1,ESI_Rev_1};

ESIdata         esi;     // Holds the selected modules's data structure
ESIChannellData esich;   // Holds the selected channel's data, only used in Rev 1

extern DialogBoxEntry ESIentriesLimits[];
extern DialogBoxEntry ESIentriesLimitsR3[];

DialogBoxEntry ESIentries[] = {
  {" ESI channel"        , 0, 1, D_INT      , 1, 2, 1, 21,      false, "%2d",     &ESIchannel, NULL, SelectESIChannel},
  {" Enable"             , 0, 2, D_ONOFF    , 0, 1, 1, 21,      false, NULL,      &esich.Enable, NULL, NULL},
  {" Voltage"            , 0, 3, D_FLOAT    , 0, 6000, 10, 18,  false, "%5.0f",   &esich.VoltageSetpoint, NULL, NULL},
  {" Monitor"            , 0, 4, D_FLOAT    , 0, 1, 1, 8,       true,  "%5.0f V", &ReadbackV[0][0], NULL, NULL},
  {""                    , 0, 4, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&ReadbackI[0][0], NULL, NULL},
  {" Limits"             , 0, 8, D_PAGE     , 0, 0, 0, 0,       false, NULL, ESIentriesLimits, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,SaveESIsettings, NULL},
  {" Restore settings"   , 0, 10,D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,RestoreESIsettings, NULL},
  {" Return to main menu", 0, 11,D_MENU     , 0, 0, 0, 0,       false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry ESIentriesLimits[] = {
  {" Voltage limit"      , 0, 1, D_FLOAT   , 0, 6000, 10, 18,  false, "%5.0f",   &esich.VoltageLimit, NULL, UpdateESIdialog},
  {" Current trip, mA"   , 0, 2, D_FLOAT   , 0, 5, 0.05, 18,   false, "%5.3f",   &esich.MaxCurrent, NULL, NULL},
  {" Calibrate"          , 0, 8, D_FUNCTION , 0, 0, 0, 0,      false, NULL, NULL, ESIcalibrate, NULL},
  {" Return to ESI menu" , 0, 10,D_PAGE    , 0, 0, 0, 0,       false, NULL, &ESIentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry ESIentriesR3[] = {
  {" ESI module"         , 0, 1, D_INT      , 1, 2, 1, 21,      false, "%2d",     &ESIchannel, NULL, SelectESIChannel},
  {" Enable"             , 0, 2, D_ONOFF    , 0, 1, 1, 21,      false, NULL,      &esi.Enable, NULL, NULL},
  {" Voltage"            , 0, 3, D_FLOAT    , -4000,4000,10,18, false, "%5.0f",   &esi.VoltageSetpoint, NULL, NULL},
  {" Monitor"            , 0, 4, D_FLOAT    , 0, 1, 1, 8,       true,  "%5.0f V", &ReadbackV[0][0], NULL, NULL},
  {""                    , 0, 4, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&Imonitor[0], NULL, NULL},
//  {" Monitor"            , 0, 4, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&Imonitor[0], NULL, NULL},
  {" Limits"             , 0, 8, D_PAGE     , 0, 0, 0, 0,       false, NULL, ESIentriesLimitsR3, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,SaveESIsettings, NULL},
  {" Restore settings"   , 0, 10,D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,RestoreESIsettings, NULL},
  {" Return to main menu", 0, 11,D_MENU     , 0, 0, 0, 0,       false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry ESIentriesLimitsR3[] = {
  {" Pos Voltage limit"  , 0, 1, D_FLOAT   , 0, 4000, 10, 18,  false, "%5.0f",   &esi.ESIchan[0].VoltageLimit, NULL, UpdateESIdialog},
  {" Neg Voltage limit"  , 0, 2, D_FLOAT   , -4000,0, 10, 18,  false, "%5.0f",   &esi.ESIchan[1].VoltageLimit, NULL, UpdateESIdialog},
  {" Current trip, mA"   , 0, 3, D_FLOAT   , 0, 5, 0.05, 18,   false, "%5.3f",   &esi.ESIchan[0].MaxCurrent, NULL, NULL},
  {" Calibrate Pos"      , 0, 7, D_FUNCTION , 0, 0, 0, 0,      false, NULL, NULL, ESIcalibratePos, NULL},
  {" Calibrate Neg"      , 0, 8, D_FUNCTION , 0, 0, 0, 0,      false, NULL, NULL, ESIcalibrateNeg, NULL},
  {" Return to ESI menu" , 0, 10,D_PAGE     , 0, 0, 0, 0,      false, NULL, &ESIentriesR3, NULL, NULL},
  {NULL},
};

DialogBox ESIdialog = {
  {
    "ESI parameters",
    ILI9340_BLACK,ILI9340_WHITE,
    2,0,0,300,220,
    B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,ESIentries
};

MenuEntry MEESImonitor = {" ESI module", M_DIALOG,0,0,0,NULL,&ESIdialog,NULL,NULL};

// This function converts the ESI channel number to a board address, 0 or 1.
// channel is 1 or 2 for rev 3 and 1 through 4 for rev 1
int ESIchannel2board(int channel)
{
  int i;
  
  // First find an active board and test the rev
  if(ESIboards[0]) i = ESIarray[0].Rev;
  if(ESIboards[1]) i = ESIarray[1].Rev;
  if(i == 1)  // Rev 1
  {
    if(channel > 2) return(1); // has to be if more than 2 channels
    if(ESIboards[0]) return(0);
    return(1);
  }
  else if(i == 3)  // Rev 3
  {
    if(channel > 1) return(1); // has to be if more than 1 channel
    if(ESIboards[0]) return(0);
    return(1);    
  }
}

// Called via dialog box selection to calibrate the current channel
void ESIcalibrate(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  
  SelectBoard(SelectedESIboard);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7994;
  CC.Min=0;
  CC.Max=esidata.ESIchan[(ESIchannel-1)&1].MaxVoltage;
  CC.DACaddr=esidata.DACadr;  
  CC.ADCaddr=esidata.ADCadr;
  CC.DACout=&esidata.ESIchan[(ESIchannel-1)&1].DCctrl;
  CC.ADCreadback=&esidata.ESIchan[(ESIchannel-1)&1].DCVmon;
  // Define this channels name
  sprintf(Name,"     ESI Channel %2d",ESIchannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, CC.Max/2);
  esi = esidata;
}

void ESIcalibratePos(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  
  SelectBoard(SelectedESIboard);
  ESIrelay(0);
  delay(10);
  ESIrelay(1);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7994;
  CC.Min=0;
  CC.Max=esidata.ESIchan[0].MaxVoltage;
  CC.DACaddr=esidata.DACadr;  
  CC.ADCaddr=esidata.ADCadr;
  CC.DACout=&esidata.ESIchan[0].DCctrl;
  CC.ADCreadback=NULL;  // No readback
  // Define this channel's name
  sprintf(Name,"     ESI Module %2d",ESIchannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, CC.Max/2);
  esi = esidata;  
  ESIrelay(0);
  if(esidata.VoltageSetpoint >= 0) ESIrelay(1);
  else ESIrelay(2);
}

void ESIcalibrateNeg(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;
  
  SelectBoard(SelectedESIboard);
  ESIrelay(0);
  delay(10);
  ESIrelay(2);
  // Set up the calibration data structure
  CC.ADCpointer = &AD7994;
  CC.Min=0;
  CC.Max=esidata.ESIchan[1].MaxVoltage;
  CC.DACaddr=esidata.DACadr;  
  CC.ADCaddr=esidata.ADCadr;
  CC.DACout=&esidata.ESIchan[1].DCctrl;
  CC.ADCreadback=NULL;  // No readback
  // Define this channel's name
  sprintf(Name,"     ESI Module %2d",ESIchannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name, 0, CC.Max/2);
  esi = esidata;  
  ESIrelay(0);  
  delay(10);
  if(esidata.VoltageSetpoint >= 0) ESIrelay(1);
  else ESIrelay(2);
}

// Updates all the dialog box limits and updates monitor
// pointers based on channel selected
void UpdateESIdialog(void)
{
  int b,c;
  DialogBoxEntry *de;
  
  if(esidata.Rev == 1)
  {
    de = GetDialogEntries(ESIentries, "Voltage");
    b = (ESIchannel - 1) / 2;
    c = (ESIchannel - 1) & 1;
    SelectedESIboard = b;
    esi = esidata;
    esich = esi.ESIchan[c];
    // Set the voltage adjust entries min and max
    de->Min = de->Max = 0;
    if(esich.VoltageLimit > 0) de->Max = esich.VoltageLimit;
    else de->Min = esich.VoltageLimit; 
    // Set the voltage limit entries min and max
    de = GetDialogEntries(ESIentriesLimits, "Voltage limit");
    de->Min = de->Max = 0;
    if(esich.MaxVoltage > 0) de->Max = esich.MaxVoltage;
    else de->Min = esich.MaxVoltage;   
    // Readback value pointers
    de = GetDialogEntries(ESIentries, "Monitor");
    de[0].Value = &ReadbackV[b][c];
    de[1].Value = &ReadbackI[b][c];
  }
  if(esidata.Rev == 3)
  {
    de = GetDialogEntries(ESIentriesR3, "Voltage");
    b = ESIchannel2board(ESIchannel);
    SelectedESIboard = b;
    esi = esidata;
    esich = esi.ESIchan[0];
    // Set the voltage adjust entries min and max
    de->Min = esi.ESIchan[1].VoltageLimit;
    de->Max = esi.ESIchan[0].VoltageLimit;
    // Set the voltage limit entries min and max
    de = GetDialogEntries(ESIentriesLimitsR3, "Pos Voltage limit");
    de->Min=0;
    de->Max = esi.ESIchan[0].MaxVoltage;
    de = GetDialogEntries(ESIentriesLimitsR3, "Neg Voltage limit");
    de->Max=0;
    de->Min = esi.ESIchan[1].MaxVoltage;
    // Readback value pointers
    de = GetDialogEntries(ESIentriesR3, "Monitor");
    de[0].Value = &ReadbackV[b][0];
    de[1].Value = &Imonitor[b];
  }
}

// Called after the user selects a channel
void SelectESIChannel(void)
{
  UpdateESIdialog();
  ESIchannelLoaded = ESIchannel;
  DialogBoxDisplay(&ESIdialog);
}

// This function is used for ESI rev 3 hardware and controls the HV reed relays.
// The relays are controlled using DAC channels 3 and 4
// action:
//        0 = both relays off
//        1 = pos relay on
//        2 = neg relay on
void ESIrelay(int action)
{
  switch (action)
  {
    case 0:
      AD5625(esidata.DACadr,2,0);
      AD5625(esidata.DACadr,3,0);
      break;
    case 1:
      AD5625(esidata.DACadr,3,0);
      AD5625(esidata.DACadr,2,65535);
      break;
    case 2:
      AD5625(esidata.DACadr,2,0);
      AD5625(esidata.DACadr,3,65535);
      break;
    default:
      break;
  }
}

void SaveESIsettings(void)
{
  bool Success = true;
  
  esidata = esi;
  if(esidata.Rev == 1) esidata.ESIchan[(ESIchannel-1)&1] = esich;
  if(ESIboards[0])
  {
    SelectBoard(0);
    if(WriteEEPROM(&ESIarray[0], ESIarray[0].EEPROMadr, 0, sizeof(ESIdata)) != 0) Success = false;
  } 
  if(ESIboards[1])
  {
    SelectBoard(1);
    if(WriteEEPROM(&ESIarray[1], ESIarray[1].EEPROMadr, 0, sizeof(ESIdata)) != 0) Success = false;
  }
  SelectBoard(SelectedESIboard);
  if(Success)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Error saving!",2000);
}

void RestoreESIsettings(void)
{
  RestoreESIsettings(false);
}

void RestoreESIsettings(bool NoDisplay)
{
  int b;
  ESIdata esi_data;
  bool Success=true;
  bool Corrupted=false;
  
  for(b=0;b<2;b++)
  {
    if(!ESIboards[b]) continue;
    SelectBoard(b);
    if(ReadEEPROM(&esi_data, ESIarray[b].EEPROMadr, 0, sizeof(ESIdata)) == 0)
    {
       if(strcmp(esi_data.Name,ESIarray[b].Name) == 0)
       {
         // Here if the name matches so copy the data to the operating data structure
         esi_data.ESIchan[0].Enable = false;
         esi_data.ESIchan[1].Enable = false;
         esi_data.Enable = false;
         memcpy(&ESIarray[b], &esi_data, sizeof(ESIdata));
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  SelectBoard(SelectedESIboard);
  esi = esidata;
  if(esidata.Rev == 1) esich = esi.ESIchan[(ESIchannel-1)&1];
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
}

// This function is called at powerup to initiaize the ESI board(s).
void ESI_init(int8_t Board)
{
  // Flag the board as present
  ESIboards[Board] = true;
  // Set active board to board being inited
  SelectedESIboard = Board;
  SelectBoard(Board);
  // Init the esi structure
  esi   = esidata;
  esich = esidata.ESIchan[0];
  // Init the DAC to use internal ref
  AD5625_EnableRef(esi.DACadr);
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreESIsettings(true);
    esidata = esi;        // Copy back into the configuration data structure array
  }
  if(esidata.Rev == 3) 
  {
    ESIdialog.Entry = ESIentriesR3;
  }
  if(NumberOfESIchannels == 0)
  {
    // Setup the menu
    AddMainMenuEntry(&MEESImonitor);
    if(ActiveDialog == NULL) SelectESIChannel();
    else UpdateESIdialog();
    // Configure Threads
    ESIthread.setName("ESI");
    ESIthread.onRun(ESI_loop);
    ESIthread.setInterval(100);
    // Add threads to the controller
    control.add(&ESIthread);
  }
  else
  {
    // If here we are setting up the second DCbias card so point back to the first one
    SelectedESIboard = 0;
    SelectBoard(0);
    esi   = esidata;
    esich = esidata.ESIchan[0];
  }
  if(esidata.Rev == 3) 
  {
    NumberOfESIchannels++;
    ESIentriesR3[0].Max = NumberOfESIchannels;
  }
  else 
  {
    NumberOfESIchannels += 2;
    ESIentries[0].Max = NumberOfESIchannels;
  }
}

void ESI_loop(void)
{
  int i,b,c;
  uint16_t ADCvals[4];
  static   float Setpoints[2][2] = {0,0,0,0};
  static   float SetpointsR3[2] = {0,0};
  static   bool Enabled[2] = {false,false};
  
  SelectBoard(SelectedESIboard);
  if (ActiveDialog == &ESIdialog)
  {
    if(ActiveDialog->Changed)
    {
      if(ESIarray[SelectedESIboard].Rev == 1)
      {
        ESIarray[ESIchannel2board(ESIchannelLoaded)] = esi;
        ESIarray[ESIchannel2board(ESIchannelLoaded)].ESIchan[(ESIchannelLoaded-1)&1] = esich;
      }
      else if(ESIarray[SelectedESIboard].Rev == 3)
      {
        ESIarray[ESIchannel2board(ESIchannelLoaded)] = esi;
      }
      ActiveDialog->Changed = false;        
    }
  }
  // If this is a rev 3 module then setup the individual power supply module data 
  // structures
  if(ESIarray[SelectedESIboard].Rev == 3)
  {
    for(b=0;b<2;b++)
    {
      if(!ESIboards[b]) continue;
      SelectBoard(b);
      if(Enabled[b] != ESIarray[b].Enable)
      {
        Enabled[b] = ESIarray[b].Enable;
        if(!Enabled)
        {
          ESIrelay(0);
          delay(10);
        }
        else
        {
          if(ESIarray[b].VoltageSetpoint >= 0)
          {
            ESIrelay(0);
            delay(10);
            ESIrelay(1);
          }
          else
          {
            ESIrelay(0);
            delay(10);
            ESIrelay(2);            
          }
        }
      }
//      if(!ESIboards[b]) continue;
//      SelectBoard(b);
      if(ESIarray[b].VoltageSetpoint != SetpointsR3[b])
      {
        // If signs do not match then set the relays
        if((ESIarray[b].VoltageSetpoint >= 0) && (SetpointsR3[b] < 0))
        {
          ESIrelay(0);
          delay(10);
          ESIrelay(1);
        }
        if((ESIarray[b].VoltageSetpoint < 0) && (SetpointsR3[b] >= 0))
        {
          ESIrelay(0);
          delay(10);
          ESIrelay(2);
        }
        SetpointsR3[b] = ESIarray[b].VoltageSetpoint;
      }
      ESIarray[b].ESIchan[0].Enable = ESIarray[b].Enable;
      ESIarray[b].ESIchan[1].Enable = ESIarray[b].Enable;
      ESIarray[b].ESIchan[1].MaxCurrent = ESIarray[b].ESIchan[0].MaxCurrent;
      if(ESIarray[b].VoltageSetpoint >= 0)
      {
        ESIarray[b].ESIchan[0].VoltageSetpoint = ESIarray[b].VoltageSetpoint;
        ESIarray[b].ESIchan[1].VoltageSetpoint = 0;
        Setpoints[b][1] = 0;
      }
      else
      {
        ESIarray[b].ESIchan[0].VoltageSetpoint = 0;
        ESIarray[b].ESIchan[1].VoltageSetpoint = ESIarray[b].VoltageSetpoint;
        Setpoints[b][0] = 0;
      }      
    }
  }
  // End of Rev 3 loop setup
  MaxESIvoltage = 0;
  for(b=0;b<2;b++)
  {
    if(ESIboards[b])
    {
      // If disabled drive setpoint to 0, else ramp it to value
      for(c=0;c<2;c++)
      {
        if(!ESIarray[b].ESIchan[c].Enable) Setpoints[b][c] = 0;
        else
        {
          if(abs(Setpoints[b][c]) < abs(ESIarray[b].ESIchan[c].VoltageSetpoint)) 
          {
            if(ESIarray[b].ESIchan[c].VoltageSetpoint < 0) Setpoints[b][c] -= ESIstepSize;
            else Setpoints[b][c] += ESIstepSize;
          }
          if(abs(Setpoints[b][c]) > abs(ESIarray[b].ESIchan[c].VoltageSetpoint)) Setpoints[b][c] = ESIarray[b].ESIchan[c].VoltageSetpoint;
        }        
      }
      // Test the voltage setpoint and force it to be in range
      if(ESIarray[b].ESIchan[0].VoltageLimit > 0) 
      {
        if(ESIarray[b].ESIchan[0].VoltageSetpoint > ESIarray[b].ESIchan[0].VoltageLimit) ESIarray[b].ESIchan[0].VoltageSetpoint = ESIarray[b].ESIchan[0].VoltageLimit;
      }
      else if(ESIarray[b].ESIchan[0].VoltageSetpoint < ESIarray[b].ESIchan[0].VoltageLimit) ESIarray[b].ESIchan[0].VoltageSetpoint = ESIarray[b].ESIchan[0].VoltageLimit;

      if(ESIarray[b].ESIchan[1].VoltageLimit > 0) 
      {
        if(ESIarray[b].ESIchan[1].VoltageSetpoint > ESIarray[b].ESIchan[1].VoltageLimit) ESIarray[b].ESIchan[1].VoltageSetpoint = ESIarray[b].ESIchan[1].VoltageLimit;
      }
      else if(ESIarray[b].ESIchan[1].VoltageSetpoint < ESIarray[b].ESIchan[1].VoltageLimit) ESIarray[b].ESIchan[1].VoltageSetpoint = ESIarray[b].ESIchan[1].VoltageLimit;
      // Output the control voltages
      SelectBoard(b);
      if(ESIarray[b].ESIchan[0].Enable) AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[0].DCctrl.Chan,Value2Counts(Setpoints[b][0],&ESIarray[b].ESIchan[0].DCctrl));
      else AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[0].DCctrl.Chan,Value2Counts(0,&ESIarray[b].ESIchan[0].DCctrl));

      if(ESIarray[b].ESIchan[1].Enable) AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[1].DCctrl.Chan,Value2Counts(Setpoints[b][1],&ESIarray[b].ESIchan[1].DCctrl));
      else  AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[1].DCctrl.Chan,Value2Counts(0,&ESIarray[b].ESIchan[1].DCctrl));

      // Read the readback ADC values for voltage and current
      if(AD7994(ESIarray[b].ADCadr, ADCvals)==0)
      {
        i = ESIarray[b].ESIchan[0].DCVmon.Chan;
        ReadbackV[b][0] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[0].DCVmon)) + (1-Filter) * ReadbackV[b][0];
        if(ReadbackV[b][0] > MaxESIvoltage) MaxESIvoltage = ReadbackV[b][0];
        i = ESIarray[b].ESIchan[0].DCImon.Chan;
        ReadbackI[b][0] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[0].DCImon)) + (1-Filter) * ReadbackI[b][0];
        i = ESIarray[b].ESIchan[1].DCVmon.Chan;
        ReadbackV[b][1] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[1].DCVmon)) + (1-Filter) * ReadbackV[b][1];
        if(ReadbackV[b][1] > MaxESIvoltage) MaxESIvoltage = ReadbackV[b][1];
        i = ESIarray[b].ESIchan[1].DCImon.Chan;
        ReadbackI[b][1] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[1].DCImon)) + (1-Filter) * ReadbackI[b][1];
      }
      // Test the current limits and disable if limit is exceeded
      if(ESIarray[b].Rev == 1)
      {
        if(ReadbackI[b][0] > ESIarray[b].ESIchan[0].MaxCurrent) 
        {
          if(ESIarray[b].ESIchan[0].MaxCurrent > 0)
          {
            ESIarray[b].ESIchan[0].Enable = false;
            ESIarray[b].Enable = false;
            DisplayMessageButtonDismiss("ESI excess current!");
          }
        }
        if(ReadbackI[b][1] > ESIarray[b].ESIchan[1].MaxCurrent) 
        {
          if(ESIarray[b].ESIchan[1].MaxCurrent > 0)
          {
            ESIarray[b].ESIchan[1].Enable = false;
            ESIarray[b].Enable = false;
            DisplayMessageButtonDismiss("ESI excess current!");
          }
        }
      }
      if(ESIarray[b].Rev == 3)
      {
        if(SetpointsR3[b] >= 0) ReadbackV[b][0] = Setpoints[b][0];
        else ReadbackV[b][0] = Setpoints[b][1];
        if(SetpointsR3[b] >=0 ) Imonitor[b] = (ReadbackI[b][0] - (SetpointsR3[b] / 40000)) / 1.53;
        else Imonitor[b] = (ReadbackI[b][1] - (abs(SetpointsR3[b]) / 40000)) / 1.53;
        if(Imonitor[b] < 0) Imonitor[b] = 0;
        if(ESIarray[b].ESIchan[0].MaxCurrent > 0)
        {
          if((Imonitor[b] > ESIarray[b].ESIchan[0].MaxCurrent) || (Imonitor[b] > ESIarray[b].ESIchan[1].MaxCurrent))
          {
            ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
            ESIarray[b].Enable = false;
            DisplayMessageButtonDismiss("ESI excess current!");
          }
        }
      }
    }
    if(abs(ReadbackV[b][0]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][0]);
    if(abs(ReadbackV[b][1]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][1]);
  }
  if(ESIarray[SelectedESIboard].Rev == 1)
  {
    esi = ESIarray[ESIchannel2board(ESIchannelLoaded)];
    esich = ESIarray[ESIchannel2board(ESIchannelLoaded)].ESIchan[(ESIchannelLoaded-1)&1];    
  }
  else if(ESIarray[SelectedESIboard].Rev == 3)
  {
    esi = ESIarray[ESIchannel2board(ESIchannelLoaded)];
  }
  if (ActiveDialog == &ESIdialog) RefreshAllDialogEntries(&ESIdialog);
}

//
// This section contains all the host computer interface command processing functions
//
// SHV, Set channel high voltage
// GHV, Returns the high voltage setpoint
// GHVV, Returns the actual high voltage output
// GHVI, Returns the output current in mA
// GHVMAX, Returns the maximum high voltage outut value
// Added Feb 22, 2017
// SHVENA, Enables a selected channel
// SHVDIS, Disables a selected channel
// GHVSTATUS, Returns the selected channel's status
// Added for rev 3 module with polarity switching
// GHVMIN, Returns the maximum high voltage outut value
// SHVPSUP, Sets a module's positive supply voltage
// SHVNSUP, Sets a module's negative supply voltage
//

// Returns the number of ESI output channels
void ESInumberOfChannels(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfESIchannels);
}

bool ValidESIchannel(int chan)
{
  if((chan < 1) || (chan > NumberOfESIchannels))
  {
    SetErrorCode(ERR_INVALIDCHAN);
    SendNAK;
    return false;
  }
  return true;
}

bool ValidESIvalue(int chan, float value)
{
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)
  {
    if((ESIarray[ESIchannel2board(chan)].ESIchan[0].VoltageLimit < value) || (ESIarray[ESIchannel2board(chan)].ESIchan[1].VoltageLimit > value))
    {
      SetErrorCode(ERR_VALUERANGE);
      SendNAK;
      return false;
    }
    return true;
  }
  if(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].MaxVoltage < 0)
  {
    if((value > 0) || (value < ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageLimit))
    {
      SetErrorCode(ERR_VALUERANGE);
      SendNAK;
      return false;
    }
  }
  else
  {
    if((value < 0) || (value > ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageLimit))
    {
      SetErrorCode(ERR_VALUERANGE);
      SendNAK;
      return false;
    }
  }
  return true;
}

// Sets the selected channels setpoint value
void SetESIchannel(char *Chan, char *Value)
{
  int   chan;
  float value;
  
  sscanf(Chan,"%d",&chan);
  sscanf(Value,"%f",&value);
  if(!ValidESIchannel(chan)) return;
  if(!ValidESIvalue(chan,value)) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)
  {
    ESIarray[ESIchannel2board(chan)].VoltageSetpoint = value;
    SendACK;
    return;
  }
  ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageSetpoint = value;
  SendACK;
}

// Reads the selected channels setpoint value
void GetESIchannel(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3) serial->println(ESIarray[ESIchannel2board(chan)].VoltageSetpoint);
  else serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageSetpoint);
}

// Reads the selected channels output voltage readback
void GetESIchannelV(int chan)
{
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)
  {
      SetErrorCode(ERR_BADCMD);
      SendNAK;
      return;    
  }
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(!SerialMute) serial->println(ReadbackV[ESIchannel2board(chan)][(chan-1)&1]);
}

// Reads the selected channels output current
void GetESIchannelI(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3) serial->println(Imonitor[ESIchannel2board(chan)]);
  else serial->println(ReadbackI[ESIchannel2board(chan)][(chan-1)&1]);
}

// Reads the selected channels maximum voltage output
void GetESIchannelMax(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3) serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[0].VoltageLimit);
  else serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageLimit);
}

// Reads the selected channels minimum voltage output
void GetESIchannelMin(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3) serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[1].VoltageLimit);
  else serial->println(0);
}

void SetESIchannelEnable(int chan)
{
  if(!ValidESIchannel(chan)) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)  ESIarray[ESIchannel2board(chan)].Enable = true;
  else ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].Enable = true;
  SendACK;  
}

void SetESIchannelDisable(int chan)
{
  if(!ValidESIchannel(chan)) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)  ESIarray[ESIchannel2board(chan)].Enable = false;
  else ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].Enable = false;
  SendACK;  
}

void GetESIstatus(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 3)
  {
    if(ESIarray[ESIchannel2board(chan)].Enable) serial->println("ON");
    else serial->println("OFF");
    return;
  }
  if(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].Enable) serial->println("ON");
  else serial->println("OFF");
}

// This function sets a modules positive supply voltage. module is 1 or 2
// reguardless of rev level
void SetESImodulePos(int module, int value)
{
  if((module < 1) || (module >2))
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;        
  }
  ESIarray[module -1].ESIchan[0].MaxVoltage = value;
  // Adjust the calibration gain
  ESIarray[module -1].ESIchan[0].DCctrl.m = 65535/value;
  SendACK;
}

// This function sets a modules negative supply voltage. module is 1 or 2
// reguardless of rev level
void SetESImoduleNeg(int module, int value)
{
  if((module < 1) || (module >2))
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
    return;        
  }
  ESIarray[module -1].ESIchan[1].MaxVoltage = value;
  // Adjust the calibration gain
  ESIarray[module -1].ESIchan[1].DCctrl.m = 65535/(value);
  SendACK;
}




