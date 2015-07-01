//
// ESI
//
// ESI module code. Supports two ESI supplies on each module, one positive and one negative. 
// Two modules can be installed in one MIPS system. These modules also monitor and display
// the actual output voltage and output current.
//
// To dos:
//  1.) Add current trip capability
//
// Gordon Anderson
// March 28, 2015
//
#include "ESI.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

extern bool NormalStartup;

// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.5         // Weak filter coefficent
#define  StrongFilter 0.05  // Strong filter coefficent

//MIPS Threads
Thread ESIthread  = Thread();

int   NumberOfESIchannels = 0;       // Defines the number of ESI channels supported. Set during intaliztion.
                                     // valid values are 0, 2, or 4
int   ESIchannel = 1;                   // Selected channel
bool  ESIboards[2] = {false,false};  // Defines the boards that are present in the system
int   SelectedESIboard=0;            // Active board, 0 or 1 = A or B
float MaxESIvoltage=0;               // This value is set to the highest ESI voltage
// Readback monitor buffers
float ReadbackV[2][2];               // Readback voltage [board][channel]
float ReadbackI[2][2];               // Readback currect [board][channel]

#define esidata ESIarray[SelectedESIboard]

ESIdata  ESIarray[2] = {ESI_Rev_1,ESI_Rev_1};

ESIdata  esi;     // Holds the selected channel's data

DialogBoxEntry ESIentries[] = {
  {" ESI channel"        , 0, 1, D_INT      , 1, 2, 1, 20,      false, "%2d",    &ESIchannel, NULL, SelectESIChannel},
  {" Voltage"            , 0, 2, D_FLOAT    , 0, 6000, 10, 17,  false, "%5.0f",   &esi.ESIchan[0].VoltageSetpoint, NULL, NULL},
  {" Montor"             , 0, 3, D_FLOAT    , 0, 1, 1, 7,       true,  "%5.0f V", &ReadbackV[0][0], NULL, NULL},
  {""                    , 0, 3, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&ReadbackI[0][0], NULL, NULL},
  {" Calibrate"          , 0, 8, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL, ESIcalibrate, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,SaveESIsettings, NULL},
  {" Restore settings"   , 0, 10,D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,RestoreESIsettings, NULL},
  {" Return to main menu", 0, 11,D_MENU     , 0, 0, 0, 0,       false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox ESIdialog = {
  {
    "ESI parameters",
    ILI9340_BLACK,ILI9340_WHITE,
    2,0,0,300,220,
    B_DOUBLE,12
  },
  M_SCROLLING,0,ESIentries
};

MenuEntry MEESImonitor = {" ESI module", M_DIALOG,0,0,0,NULL,&ESIdialog,NULL,NULL};

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
  ChannelCalibrate(&CC, Name);
  esi = esidata;
}

// Called after the user selects a channel
void SelectESIChannel(void)
{
  int b,c;

  b = (ESIchannel - 1) / 2;
  c = (ESIchannel - 1) & 1;
  SelectedESIboard = b;
  esi = esidata;
  ESIentries[1].Value = &esi.ESIchan[c].VoltageSetpoint;
  // Set the voltage adjust entries min and max
  ESIentries[1].Min = ESIentries[1].Max = 0;
  if(esi.ESIchan[c].MaxVoltage > 0) ESIentries[1].Max = esi.ESIchan[c].MaxVoltage;
  else ESIentries[1].Min = esi.ESIchan[c].MaxVoltage; 
  // Readback value pointers
  ESIentries[2].Value = &ReadbackV[b][c];
  ESIentries[3].Value = &ReadbackI[b][c];
  // Update the display
  DialogBoxDisplay(&ESIdialog);
}

void SaveESIsettings(void)
{
  bool Success = true;
  
  if(ESIboards[0])
  {
    SelectBoard(0);
    if(WriteEEPROM(&ESIarray[0], ESIarray[0].EEPROMadr, 0, sizeof(ESIdata)) != 0) Success = false;
  } 
  if(DCbiasBoards[1])
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
         memcpy(&ESIarray[b], &esi_data, sizeof(ESIdata));
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  SelectBoard(SelectedESIboard);
  esi = esidata;
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
  esi = esidata;
  // Init the DAC to use internal ref
  AD5625_EnableRef(esi.DACadr);
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreESIsettings(true);
    esidata = esi;        // Copy back into the configuration data structure array
  }
  if(NumberOfESIchannels == 0)
  {
    // Setup the menu
    AddMainMenuEntry(&MEESImonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&ESIdialog);
    // Configure Threads
    ESIthread.onRun(ESI_loop);
    ESIthread.setInterval(100);
    // Add threads to the controller
    controll.add(&ESIthread);
  }
  else
  {
    // If here we are setting up the second DCbias card so point back to the first one
    SelectedESIboard = 0;
    SelectBoard(0);
    esi = esidata;
  }
  NumberOfESIchannels += 2;
  ESIentries[0].Max = NumberOfESIchannels;
}

void ESI_loop(void)
{
  int i;
  int b;
  uint16_t ADCvals[4];
  
  SelectBoard(SelectedESIboard);
  esidata = esi;
  MaxESIvoltage = 0;
  for(b=0;b<2;b++)
  {
    if(ESIboards[b])
    {
      // Output the control voltages
      SelectBoard(b);
      AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[0].DCctrl.Chan,Value2Counts(ESIarray[b].ESIchan[0].VoltageSetpoint,&ESIarray[b].ESIchan[0].DCctrl));
      AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[1].DCctrl.Chan,Value2Counts(ESIarray[b].ESIchan[1].VoltageSetpoint,&ESIarray[b].ESIchan[1].DCctrl));
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
    }
  }
  if (ActiveDialog == &ESIdialog) UpdateNoEditDialogEntries(&ESIdialog);
}

//
// This section contains all the host computer interface command processing functions
//
// SHV, Set channel high voltage
// GHV, Returns the high voltage setpoint
// GHVV, Returns the actual high voltage output
// GHVI, Returns the output current in mA
// GHVMAX, Returns the maximum high voltage outut value
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
  if(ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].MaxVoltage < 0)
  {
    if((value > 0) || (value < ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].MaxVoltage))
    {
      SetErrorCode(ERR_VALUERANGE);
      SendNAK;
      return false;
    }
  }
  else
  {
    if((value < 0) || (value > ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].MaxVoltage))
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
  ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].VoltageSetpoint = value;
  SendACK;
  // update the ui if needed
  esi = esidata;
  if (ActiveDialog == &ESIdialog) if(chan == ESIchannel) DisplayAllDialogEntries(&ESIdialog);
}

// Reads the selected channels setpoint value
void GetESIchannel(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(!SerialMute) serial->println(ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].VoltageSetpoint);
}

// Reads the selected channels output voltage readback
void GetESIchannelV(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(!SerialMute) serial->println(ReadbackV[(chan-1)/2][(chan-1)&1]);
}

// Reads the selected channels output current
void GetESIchannelI(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(!SerialMute) serial->println(ReadbackI[(chan-1)/2][(chan-1)&1]);
}

// Reads the selected channels maximum voltage output, minimum is always 0
void GetESIchannelMax(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(!SerialMute) serial->println(ESIarray[(chan-1)/2].ESIchan[(chan-1)&1].MaxVoltage);
}




