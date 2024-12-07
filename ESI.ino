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
// Updated Nov 30, 2019
//  1.) Added support for hardware rev 4.1 that uses the EMCO G series supplies, added firmware
//      rev 5 to support this hardware
//
// This firmware module supports a lot of different hardware board revs, the hardware rev details 
//    are defined below:
//  - Rev 1.0, this is the orginal design supporting the Matsusada 30 watt supplies. The module has a 
//    positive and degative output, 1 channel for each
//  - Rev 2.0, this is very simalar to rev 1.0 but adds hardware support for the EMCO C series modules.
//    These modules do not have voltate readbacks or current monitors. Output current is estimated by 
//    measureing the modules input current.
//  - Rev 3.0, this version has a single output and uses relays to switch between the positive and 
//    negative supplies. The EMCO C series C40 / C40N are supported
//  - Rev 3.1, this version is the same as 3.0 but support the C50 and C60 EMCO supplies, these have a
//    different formfactor and use different relays
//  - Rev 4.1, 4.2, 4.3, this version uses the EMCO G series proportional supplies. The hardware has a closed loop 
//    controller to maintain the setpoint. This module has voltage monitor feedback. Current is estimated
//    from module input current. This version has two module (+ and -) and uses relays to switch polatity.
//    Implemented 11/29/2019.
//  
//
// The firmware Rev variable defines how this module functions and what hardware rev(s) are supported:
//  - Rev 1.0 of the ESI modules was desiged for the Matsusada 30 watt supplies, a 6KV and -6KV
//    model. Works with hardware rev 1 and 2.
//  - Rev 2.0 support the rev 2.0 hardware with the EMCO modules installed
//  - Rev 3.0 of the ESI hardware supports the EMCO C40 and C40N modules and includes relay polarity
//    switching. The EMCO modules do not have readbacks or current monitoring. The input power to the
//    modules is monitoried and used to estimate load current.
//  - Rev 4.0 supports the Rev 3.1 hardware with the two outputs sent to two SHV connectors. The relays
//    are used to disable and enable the outputs. This was added for the softlanding system 8/28/2019
//  - Rev 5.0 supports the rev 4.1, 4.2, and 4.3 hardware. 
//  - Rev 6.0 supports the rev 2.1 hardware connected to a ESI module PCB with relay switching, in this 
//    case one switched output using CB101 and CB101N 10KV module with both voltage and current readbacks.
//  - Rev 7.0 supports the rev 2.1 hardware connected to a ESI module PCB with relay switching, in this 
//    case two switched outputs using CB101 and CB101N 10KV module with both voltage and current readbacks.
//  - Rev 8.0 supports the rev 2.1 hardware connected to APD G10 modules with external relay switching, in this 
//    case two switched outputs using G10 modules with voltage readbacks only.
//
//  Note:
//    When using rev 4.1 or 4.2 (firmware rev set to 5). The current monitor gains and offsets must be set to
//    1 and 0. The following commands will set the gains and offstes:
//      SETEEPADD,ESI,board address (0 or 1)
//      SETOFFSET,48
//      WRITE,FLOAT,1
//      SETOFFSET,4C
//      WRITE,FLOAT,0
//      SETOFFSET,80
//      WRITE,FLOAT,1
//      SETOFFSET,84
//      WRITE,FLOAT,0
//    When using rev 4.3 (firmware rev set to 5). The current monitor gains and offsets must be set to
//    as shown below. 
//      SETEEPADD,ESI,board address (0 or 1)
//      SETOFFSET,48
//      WRITE,FLOAT,250000
//      SETOFFSET,4C
//      WRITE,FLOAT,0 (33400), 39000
//      SETOFFSET,80
//      WRITE,FLOAT,250000
//      SETOFFSET,84
//      WRITE,FLOAT,0
//
// Develope calibration functon for rev 4.3 hardware using G series power supplies.
// This is the perfered ESI version. This version has a 50M divider on the outputs
// for readbacks and output regulation. This also provides the calibration load for
// system setup.
//    - Calibration function for ESI 4.3. Performs the following:
//      1.) Request the power supply voltage
//      2.) Set the voltage to first point and record the actual voltage.
//          Also record the current reading
//      3.) Set the voltage to second point and record the actual voltage and currwent.
//      4.) Calculate the calibration constants and apply
// do this for both board channels. Command name CAL4P3,mod,arg. Where mod is 
// 1 or 2 for ESI supply module number and arg is POS or NEG for the two channels of
// the module.
//    
//
// Gordon Anderson
// March 28, 2015
// Revised Feb 22, 2017
// Revised Feb 25, 2017 added the rev 3 updates
// Revised Aug 28, 2019 added rev 4 updates
// Revised Jan 8, 2022 added rev 8 updates
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

//#define ESIstepSize 50      // Changed from 10 3/12/18
int ESIstepSize = 50;         // Changed 07/28/2020

//MIPS Threads
Thread ESIthread  = Thread();

int   NumberOfESIchannels = 0;       // Defines the number of ESI channels supported. Set during intaliztion.
                                     // valid values are 0, 2, or 4
int   ESIchannel = 1;                // Selected channel
int   ESIchannelLoaded = 1;          // ESI channel loaded into working data structures
bool  ESIboards[2] = {false,false};  // Defines the boards that are present in the system
int   SelectedESIboard=0;            // Active board, 0 or 1 = A or B
float MaxESIvoltage=0;               // This value is set to the highest ESI voltage
bool  ESIcurrentTest=true;           // Flag to enable over current testing, true by default
// Readback monitor buffers
float ReadbackV[2][2];               // Readback voltage [board][channel]
float ReadbackI[2][2];               // Readback currect [board][channel]
float Imonitor[2];                   // Used for rev 3 current readback monitor

#define esidata ESIarray[SelectedESIboard]

ESIdata  ESIarray[2] = {ESI_Rev_1,ESI_Rev_1};

ESIdata         esi;     // Holds the selected modules's data structure
ESIChannellData esich;   // Holds the selected channel's data, only used in Rev 1, 2, and 4

#define HVPOS_OFF   AD5625(esidata.DACadr,2,0)
#define HVPOS_ON    AD5625(esidata.DACadr,2,65535)
#define HVNEG_OFF   AD5625(esidata.DACadr,3,0)
#define HVNEG_ON    AD5625(esidata.DACadr,3,65535)

extern DialogBoxEntry ESIentriesLimits[];
extern DialogBoxEntry ESIentriesLimitsR3[];

bool ESIgate = false;

DialogBoxEntry ESIentries[] = {
  {" ESI channel"        , 0, 1, D_INT      , 1, 2, 1, 21,      false, "%2d",     &ESIchannel, NULL, SelectESIChannel},
  {" Enable"             , 0, 2, D_ONOFF    , 0, 1, 1, 21,      false, NULL,      &esich.Enable, NULL, NULL},
  {" Voltage"            , 0, 3, D_FLOAT    , 0, 6000, 10, 17,  false, "%6.0f",   &esich.VoltageSetpoint, NULL, NULL},
  {" Monitor"            , 0, 4, D_FLOAT    , 0, 1, 1, 8,       true,  "%6.0f V", &ReadbackV[0][0], NULL, NULL},
  {""                    , 0, 4, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&ReadbackI[0][0], NULL, NULL},
  {" Gate ena"           , 0, 5, D_OFF      , 0, 1, 1, 21,      false, NULL,      &ESIgate, NULL, ESIgateChange},
  {" Limits"             , 0, 8, D_PAGE     , 0, 0, 0, 0,       false, NULL, ESIentriesLimits, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,SaveESIsettings, NULL},
  {" Restore settings"   , 0, 10,D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,RestoreESIsettings, NULL},
  {" Return to main menu", 0, 11,D_MENU     , 0, 0, 0, 0,       false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char  *LevelModeList = "OFF,IF>,IF<";
char  LevelMode[4] = "OFF";
float Vthreshold = 0;

DialogBoxEntry ESIentriesLimits[] = {
  {" Voltage limit"       , 0, 1, D_FLOAT   , 0, 6000, 10, 18,  false, "%5.0f",   &esich.VoltageLimit, NULL, UpdateESIdialog},
  {" Current trip, mA"    , 0, 2, D_FLOAT   , 0, 5, 0.05, 18,   false, "%5.3f",   &esich.MaxCurrent, NULL, NULL},
  {" ESI chg,V/0.1s"      , 0, 3, D_INT     , 5, 5000, 1, 18,   false, "%5d",     &esi.ESIstepsize, NULL, NULL},
  {" Level mode"          , 0, 4, D_OFF     , 0, 0, 3, 20,      false, LevelModeList, LevelMode, NULL, LevelChanged},
  {" Level threshold"     , 0, 5, D_OFF     , -200, 200, 1, 18, false, "%5.0f",   &Vthreshold, NULL, LevelChanged},
  
  {" Calibrate"           , 0, 8, D_FUNCTION , 0, 0, 0, 0,      false, NULL, NULL, ESIcalibrate, NULL},
  {" Return to ESI menu"  , 0, 10,D_PAGE    , 0, 0, 0, 0,       false, NULL, &ESIentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry ESIentriesR3[] = {
  {" ESI module"         , 0, 1, D_INT      , 1, 2, 1, 21,      false, "%2d",     &ESIchannel, NULL, SelectESIChannel},
  {" Enable"             , 0, 2, D_ONOFF    , 0, 1, 1, 21,      false, NULL,      &esi.Enable, NULL, NULL},
  {" Voltage"            , 0, 3, D_FLOAT    , -4000,4000,10,17, false, "%6.0f",   &esi.VoltageSetpoint, NULL, NULL},
  {" Monitor"            , 0, 4, D_FLOAT    , 0, 1, 1, 8,       true,  "%6.0f V", &ReadbackV[0][0], NULL, NULL},
  {""                    , 0, 4, D_FLOAT    , 0, 1, 1, 16,      true,  "%5.3f mA",&Imonitor[0], NULL, NULL},
  {" Limits"             , 0, 8, D_PAGE     , 0, 0, 0, 0,       false, NULL, ESIentriesLimitsR3, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,SaveESIsettings, NULL},
  {" Restore settings"   , 0, 10,D_FUNCTION , 0, 0, 0, 0,       false, NULL, NULL,RestoreESIsettings, NULL},
  {" Return to main menu", 0, 11,D_MENU     , 0, 0, 0, 0,       false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry ESIentriesLimitsR3[] = {
  {" Pos Voltage limit"  , 0, 1, D_FLOAT   , 0, 4000, 10, 18,  false, "%6.0f",   &esi.ESIchan[0].VoltageLimit, NULL, UpdateESIdialog},
  {" Neg Voltage limit"  , 0, 2, D_FLOAT   , -4000,0, 10, 18,  false, "%6.0f",   &esi.ESIchan[1].VoltageLimit, NULL, UpdateESIdialog},
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

void ESILevelDet(void)
{
  float fval;
  byte *b;
  int  i=0,brd,ch;

  // Make sure a module is looking for this signal, if not exit
  for(brd=0;brd<2;brd++)
  {
    if(ESIboards[brd])
    {
      if(((ESIarray[brd].LevelMode[0] != ESI_OFF)&&(ESIarray[brd].ESIchan[0].Enable)) || ((ESIarray[brd].LevelMode[1] != ESI_OFF)&&(ESIarray[brd].ESIchan[1].Enable))) break;
    }
  }
  if(brd==2) return;
  // Read the voltage level from the level detector and process
  b = (byte *)&fval;
  for(int j=0;j<3;j++)
  {
    i=0;
    Wire1.beginTransmission(LevelDetAdd);
    Wire1.write(TWI_LEVDET_READ_ADC);
    if(Wire1.endTransmission() != 0)
    {
      Wire1.begin();
      //delayMicroseconds(10);
      continue;
    }
    Wire1.requestFrom((uint8_t)LevelDetAdd, (uint8_t)4);
    while (Wire1.available()) b[i++] = Wire1.read();
    break;
  }
  for(brd=0;brd<2;brd++)
  {
    if(ESIboards[brd]) for(ch=0; ch<2; ch++) if(ESIarray[brd].ESIchan[ch].Enable && ESIarray[brd].EnableGatting[ch])
    {
      if((ESIarray[brd].LevelMode[ch] == ESI_IFGT) && (fval >= ESIarray[brd].LevelThreshold[ch])) ESIarray[brd].Gate[ch] = true;
      else if((ESIarray[brd].LevelMode[ch] == ESI_IFGT) && (fval < ESIarray[brd].LevelThreshold[ch])) ESIarray[brd].Gate[ch] = false;
      if((ESIarray[brd].LevelMode[ch] == ESI_IFLT) && (fval <= ESIarray[brd].LevelThreshold[ch])) ESIarray[brd].Gate[ch] = true;
      else if((ESIarray[brd].LevelMode[ch] == ESI_IFLT) && (fval > ESIarray[brd].LevelThreshold[ch]))  ESIarray[brd].Gate[ch] = false;
      if(ESIarray[brd].Gate[ch] == true)
      {
        LEDstate |=  (1 << ch);
        int cb=SelectedBoard();
        SelectBoard(brd);
        AD5625(ESIarray[brd].DACadr,2+ch,65535); 
        delayMicroseconds(1000);
        if(cb != brd) SelectBoard(cb);
      }
      else
      {
        LEDstate &=  ~(1  << ch);
        int cb=SelectedBoard();
        SelectBoard(brd);
        AD5625(ESIarray[brd].DACadr,2+ch,0);
        delayMicroseconds(1000);
        if(cb != brd) SelectBoard(cb);   
      }
    }
    if(ESIboards[brd]) for(ch=0; ch<2; ch++) if(!ESIarray[brd].ESIchan[ch].Enable) LEDstate &=  ~(1<<ch);
  }  
}

// Here when a change has been deteced by the Level Detection module
void ESILevelDetISR(void)
{
  if(AcquireTWI()) ESILevelDet();
  else TWIqueue(ESILevelDet);
}

// This function converts the ESI channel number to a board address, 0 or 1.
// channel is 1 or 2 for rev 3, 5, 6, and 8 and 1 through 4 for rev 1, 2, 4, and 7
int ESIchannel2board(int channel)
{
  int i;
  
  // First find an active board and test the rev
  if(ESIboards[0]) i = ESIarray[0].Rev;
  if(ESIboards[1]) i = ESIarray[1].Rev;
  if((i == 1) || (i == 2) || (i == 4) || (i == 7)) // Rev 1 or Rev 2 or Rev 4 or Rev 7
  {
    if(channel > 2) return(1); // has to be if more than 2 channels
    if(ESIboards[0]) return(0);
    return(1);
  }
  else if((i == 3) || (i==5) || (i==6) || (i==8))  // Rev 3 or 5 or 6 or 8
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
  if(esidata.Rev == 4) CC.ADCaddr=NULL;
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
  if((esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8)) CC.ADCreadback=&esidata.ESIchan[0].DCVmon;
  else CC.ADCreadback=NULL;  // No readback
  // Define this channel's name
  sprintf(Name,"     ESI Module %2d",ESIchannel);
  // Calibrate this channel
  if((esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8)) ChannelCalibrate(&CC, Name, CC.Max/10, CC.Max/2);
  else ChannelCalibrate(&CC, Name, 0, CC.Max/2);
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
  if((esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8)) CC.ADCreadback=&esidata.ESIchan[1].DCVmon;
  else CC.ADCreadback=NULL;  // No readback
  // Define this channel's name
  sprintf(Name,"     ESI Module %2d",ESIchannel);
  // Calibrate this channel
  if((esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8)) ChannelCalibrate(&CC, Name, CC.Max/10, CC.Max/2);
  else ChannelCalibrate(&CC, Name, 0, CC.Max/2);
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
  
  if((esidata.Rev == 1) || (esidata.Rev == 2) || (esidata.Rev == 4) || (esidata.Rev == 7))
  {
    de = GetDialogEntries(ESIentries, "Voltage");
    b = ESIchannel2board(ESIchannel);
//    b = (ESIchannel - 1) / 2;
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
    de[0].Xpos = 8;
    de[1].Type = D_FLOAT;
  }
  if((esidata.Rev == 3) || (esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8))
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
    de[0].Xpos = 8;
    de[1].Type = D_FLOAT;
  }
  if(esidata.Rev == 8)
  {
    // Turn off the current monitor and reposition the voltage readback
    de = GetDialogEntries(ESIentriesR3, "Monitor");
    de[0].Xpos = 16;
    de[1].Type = D_OFF;
    // Turn off the current limit interface
  }
}

// Called after the user changes a level control parameter
void LevelChanged(void)
{
  esi.LevelThreshold[ESIchannel-1] = Vthreshold;
  esidata.LevelThreshold[ESIchannel-1] = Vthreshold;
  if(strcmp(LevelMode, "OFF") == 0) esi.LevelMode[ESIchannel-1] = esidata.LevelMode[ESIchannel-1] = ESI_OFF;
  else if(strcmp(LevelMode, "IF>") == 0) esi.LevelMode[ESIchannel-1] = esidata.LevelMode[ESIchannel-1] = ESI_IFGT;
  else if(strcmp(LevelMode, "IF<") == 0) esi.LevelMode[ESIchannel-1] = esidata.LevelMode[ESIchannel-1] = ESI_IFLT;
  if((esi.LevelMode[0] != ESI_OFF) || (esi.LevelMode[1] != ESI_OFF)) 
  {
    pinMode(SCL1,INPUT);
    pinMode(SDA1,INPUT);
    pinMode(LEVCHANGE,INPUT);
    Wire1.begin();
    Wire1.setClock(Wire1DefaultSpeed);
    attachInterrupt(digitalPinToInterrupt(LEVCHANGE), ESILevelDetISR, RISING);
  }
}

// Called after the user changes the gate parameter

// gate = true or false
// chan = 1 or 2
void ESIgateChangeProcess(bool gate, int chan)
{
  if(gate) 
  {
    LEDoverride  = true;
    pinMode(SCL1,INPUT);
    pinMode(SDA1,INPUT);
    pinMode(LEVCHANGE,INPUT);
    Wire1.begin();
    Wire1.setClock(Wire1DefaultSpeed);
    attachInterrupt(digitalPinToInterrupt(LEVCHANGE), ESILevelDetISR, RISING);
    ESILevelDetISR();
  }
  else
  {
    // If channel is enabled that make sure relay is on
    int brd = ESIchannel2board(chan);
    if(ESIarray[brd].ESIchan[chan-1].Enable)
    {
      SelectBoard(brd);
      if(chan == 1) HVPOS_ON;
      else HVNEG_ON;
    }
  }
  // If any channels are enabled then keep LED in override mode.
  for(int brd=0;brd<1;brd++)
  {
    for(int ch=0;ch<2;ch++)
    {
      if(ESIarray[brd].EnableGatting[ch]) return;
    }
  }
  LEDoverride  = false;
}

void ESIgateChange(void)
{
  esi.EnableGatting[ESIchannel-1] = esidata.EnableGatting[ESIchannel-1] = ESIgate;
  ESIgateChangeProcess(ESIgate,ESIchannel);
  return;

  if(ESIgate) 
  {
    LEDoverride  = true;
    pinMode(SCL1,INPUT);
    pinMode(SDA1,INPUT);
    pinMode(LEVCHANGE,INPUT);
    Wire1.begin();
    Wire1.setClock(Wire1DefaultSpeed);
    attachInterrupt(digitalPinToInterrupt(LEVCHANGE), ESILevelDetISR, RISING);
    ESILevelDetISR();
  }
  else
  {
    // If channel is enabled that make sure relay is on
    if(esich.Enable)
    {
      SelectBoard(SelectedESIboard);
      if(ESIchannel == 1) HVPOS_ON;
      else HVNEG_ON;
    }
  }
  // If any channels are enabled then keep LED in override mode.
  for(int brd=0;brd<1;brd++)
  {
    for(int ch=0;ch<2;ch++)
    {
      if(ESIarray[brd].EnableGatting[ch]) return;
    }
  }
  LEDoverride  = false;
}

// Called after the user selects a channel
void SelectESIChannel(void)
{
  UpdateESIdialog();
  ESIchannelLoaded = ESIchannel;
  Vthreshold = esi.LevelThreshold[ESIchannel-1];
  ESIgate = esi.EnableGatting[ESIchannel-1];
  if(esi.LevelMode[ESIchannel-1] == ESI_OFF) strcpy(LevelMode, "OFF");
  else if(esi.LevelMode[ESIchannel-1] == ESI_IFGT) strcpy(LevelMode, "IF>");
  else if(esi.LevelMode[ESIchannel-1] == ESI_IFLT) strcpy(LevelMode, "IF<");  
  DialogBoxDisplay(&ESIdialog);
}

// This function is used for ESI rev 3 or 5 or 6 or 8, hardware and controls the HV reed relays.
// The relays are controlled using DAC channels 3 and 4
// action:
//        0 = both relays off
//        1 = pos relay on
//        2 = neg relay on
void ESIrelay(int action)
{
  if(esidata.Rev == 6)
    if(action == 1) action = 2;
    else if(action == 2) action = 1;
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
  if((esidata.Rev == 1) || (esidata.Rev == 2) || (esidata.Rev == 4) || (esidata.Rev == 7)) esidata.ESIchan[(ESIchannel-1)&1] = esich;
  if(ESIboards[0])
  {
    SelectBoard(0);
    if(WriteEEPROM(&ESIarray[0], ESIarray[0].EEPROMadr, 0, ESIarray[0].Size = sizeof(ESIdata)) != 0) Success = false;
  } 
  if(ESIboards[1])
  {
    SelectBoard(1);
    if(WriteEEPROM(&ESIarray[1], ESIarray[1].EEPROMadr, 0, ESIarray[0].Size = sizeof(ESIdata)) != 0) Success = false;
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
         esi_data.EEPROMadr = ESIarray[b].EEPROMadr;
         esi_data.ESIchan[0].Enable = false;
         esi_data.ESIchan[1].Enable = false;
         esi_data.EnableGatting[0] = false;
         esi_data.EnableGatting[1] = false;
         esi_data.Enable = false;
         //memcpy(&ESIarray[b], &esi_data, sizeof(ESIdata));
         memcpy(&ESIarray[b], &esi_data, esi_data.Size);
       } 
       else Corrupted=true;
    }
    else Success=false;
  }
  SelectBoard(SelectedESIboard);
  esi = esidata;
  if((esidata.Rev == 1) || (esidata.Rev == 2) || (esidata.Rev == 4) || (esidata.Rev == 7)) esich = esi.ESIchan[(ESIchannel-1)&1];
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
}

// This function is called at powerup to initiaize the ESI board(s).
void ESI_init(int8_t Board, int8_t addr)
{
  // Flag the board as present
  ESIboards[Board] = true;
  // Set active board to board being inited
  SelectedESIboard = Board;
  SelectBoard(Board);
  esidata.EEPROMadr = addr;
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
  if((esidata.Rev == 3) || (esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8))
  {
    ESIdialog.Entry = ESIentriesR3;
  }
  if((esidata.Rev == 4) || (esidata.Rev == 7))
  {
    HVPOS_OFF;
    HVNEG_OFF;
  }
  // Check the system gatable flag, if set enable the menu options
  if(esi.SystemGatable)
  {
    DialogBoxEntry *de;
    de = GetDialogEntries(ESIentries, "Gate ena");
    if(de != NULL) de->Type = D_ONOFF;
    de = GetDialogEntries(ESIentriesLimits, "Level mode");
    if(de != NULL) de->Type = D_LIST;
    de = GetDialogEntries(ESIentriesLimits, "Level threshold");
    if(de != NULL) de->Type = D_FLOAT;
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
  if((esidata.Rev == 3) || (esidata.Rev == 5) || (esidata.Rev == 6) || (esidata.Rev == 8))
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

// ESI module polling loop, called from the scheduler at a 10 Hz rate.
void ESI_loop(void)
{
  int             i,b,c;
  uint16_t        ADCvals[4];
  static   int    relayDly = 100;
  static   float  Setpoints[2][2] = {0,0,0,0};
  static   float  SetpointsR3[2] = {0,0};
  static   bool   Enabled[2] = {false,false};
  static   bool   Enable[2][2] = {false,false,false,false};
  static   int8_t ESIoverCurrentTimer[2] = {5,5};
  
  if(--relayDly < 0) relayDly = 100;
  SelectBoard(SelectedESIboard);
  if (ActiveDialog == &ESIdialog)
  {
    if(ActiveDialog->Changed)
    {
      if((ESIarray[SelectedESIboard].Rev == 1) || (ESIarray[SelectedESIboard].Rev == 2) || (ESIarray[SelectedESIboard].Rev == 4) || (ESIarray[SelectedESIboard].Rev == 7))
      {
        ESIarray[ESIchannel2board(ESIchannelLoaded)] = esi;
        ESIarray[ESIchannel2board(ESIchannelLoaded)].ESIchan[(ESIchannelLoaded-1)&1] = esich;
      }
      else if((ESIarray[SelectedESIboard].Rev == 3) || (ESIarray[SelectedESIboard].Rev == 5) || (ESIarray[SelectedESIboard].Rev == 6) || (ESIarray[SelectedESIboard].Rev == 8))
      {
        ESIarray[ESIchannel2board(ESIchannelLoaded)] = esi;
      }
      ActiveDialog->Changed = false;        
    }
  }
  // If this is a rev 3 or 5 or 6 or 8 module then setup the individual power supply module data 
  // structures
  if((ESIarray[SelectedESIboard].Rev == 3) || (ESIarray[SelectedESIboard].Rev == 5) || (ESIarray[SelectedESIboard].Rev == 6) || (ESIarray[SelectedESIboard].Rev == 8))
  {
    for(b=0;b<2;b++)
    {
      if(!ESIboards[b]) continue;
      SelectBoard(b);
      if(Enabled[b] != ESIarray[b].Enable)
      {
        // Enable state has changed so process the change
        Enabled[b] = ESIarray[b].Enable;
        if(!Enabled[b])
        {
          // Here if disabled so open both relays
          ESIrelay(0);
          delay(10);
        }
        else
        {
          // Here if just enabled
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
      if(ESIarray[b].VoltageSetpoint != SetpointsR3[b])
      {
        // Update SetpointR3
        if(Setpoints[b][0] != 0) SetpointsR3[b] = Setpoints[b][0];   // != could be the bug, should be >= 0, not sure?
        else SetpointsR3[b] = Setpoints[b][1];
      }
      ESIarray[b].ESIchan[0].Enable = ESIarray[b].Enable;
      ESIarray[b].ESIchan[1].Enable = ESIarray[b].Enable;
      ESIarray[b].ESIchan[1].MaxCurrent = ESIarray[b].ESIchan[0].MaxCurrent;
      if(ESIarray[b].VoltageSetpoint >= 0)
      {
        ESIarray[b].ESIchan[1].VoltageSetpoint = 0;
        if(Setpoints[b][1] == 0) 
        {
          if((ESIarray[b].ESIchan[0].VoltageSetpoint == 0) && (ESIarray[b].VoltageSetpoint != 0) && Enabled[b])
          {
            // Set the relays for positive operation
            ESIrelay(0);
            delay(10);
            ESIrelay(1);
          }
          ESIarray[b].ESIchan[0].VoltageSetpoint = ESIarray[b].VoltageSetpoint;
          // Relay should be set here, set it every few seconds to be sure
          if((relayDly == 0) && Enabled[b]) ESIrelay(1);
        }
      }
      else
      {
        if(ESIarray[SelectedESIboard].Rev != 8) ESIarray[b].ESIchan[0].VoltageSetpoint = 0;  // This is a bandaid
        if(Setpoints[b][0] == 0) 
        {
          if((ESIarray[b].ESIchan[1].VoltageSetpoint == 0) && Enabled[b])
          {
            // Set the relays for negative operation
            ESIrelay(0);
            delay(10);
            ESIrelay(2);
          }
          ESIarray[b].ESIchan[1].VoltageSetpoint = ESIarray[b].VoltageSetpoint;
          // Relay should be set here, set it every few seconds to be sure
          if((relayDly == 0) && Enabled[b]) ESIrelay(2);
        }
      }      
    }
  }
  // End of Rev 3/5/6 loop setup
  MaxESIvoltage = 0;
  for(b=0;b<2;b++)
  {
    if(ESIboards[b])
    {
      SelectBoard(b);  // 1/4/2022
      ESIstepSize = ESIarray[b].ESIstepsize;
      // If disabled drive setpoint to 0, else ramp it to value
      for(c=0;c<2;c++)
      {
        if((ESIarray[b].Rev == 4) && (Enable[b][c] != ESIarray[b].ESIchan[c].Enable))
        {
          if(c==0)
          {
             if(ESIarray[b].ESIchan[c].Enable) HVPOS_ON;
             else HVPOS_OFF;
          }
          else
          {
             if(ESIarray[b].ESIchan[c].Enable) HVNEG_ON;
             else HVNEG_OFF;
          }
          Enable[b][c] = ESIarray[b].ESIchan[c].Enable;
          ESILevelDetISR();
        }
        if((ESIarray[b].Rev == 7) && (Enable[b][c] != ESIarray[b].ESIchan[c].Enable))
        {
          if(c==1)
          {
             if(ESIarray[b].ESIchan[c].Enable) HVPOS_ON;
             else HVPOS_OFF;
          }
          else
          {
             if(ESIarray[b].ESIchan[c].Enable) HVNEG_ON;
             else HVNEG_OFF;
          }
          Enable[b][c] = ESIarray[b].ESIchan[c].Enable;
          ESILevelDetISR();
        }
        if(!ESIarray[b].ESIchan[c].Enable) Setpoints[b][c] = 0;
        else
        {
          // Ramp up or down and always pause at 0
          if(abs(Setpoints[b][c]-ESIarray[b].ESIchan[c].VoltageSetpoint) <= ESIstepSize) Setpoints[b][c]=ESIarray[b].ESIchan[c].VoltageSetpoint;
          else
          {
            float sp = Setpoints[b][c];
            if(ESIarray[b].ESIchan[c].VoltageSetpoint < Setpoints[b][c]) Setpoints[b][c] -= ESIstepSize;
            else Setpoints[b][c] += ESIstepSize;
            if(((sp > 0) && (Setpoints[b][c] < 0)) || ((sp < 0) && (Setpoints[b][c] > 0))) Setpoints[b][c] = 0;
          }
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
      //
      // Read the readback ADC values for voltage and current
      //
      if(AD7994(ESIarray[b].ADCadr, ADCvals)==0)
      {
        i = ESIarray[b].ESIchan[0].DCVmon.Chan;
        ReadbackV[b][0] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[0].DCVmon)) + (1-Filter) * ReadbackV[b][0];
        if(ReadbackV[b][0] > MaxESIvoltage) MaxESIvoltage = ReadbackV[b][0];
        i = ESIarray[b].ESIchan[0].DCImon.Chan;
        if(ESIarray[b].Rev == 2)
        {
          ReadbackV[b][0] = Setpoints[b][0];
          float current = Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[0].DCImon);
          current = (current - (abs(ReadbackV[b][0]) / abs(ESIarray[b].ESIchan[0].MaxVoltage * 10))) / 0.4;
          ReadbackI[b][0] = Filter * (current + (1-Filter) * ReadbackI[b][0]);
          if(ReadbackI[b][0] < 0) ReadbackI[b][0] = 0;          
        }
        else ReadbackI[b][0] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[0].DCImon)) + (1-Filter) * ReadbackI[b][0];
        i = ESIarray[b].ESIchan[1].DCVmon.Chan;
        ReadbackV[b][1] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[1].DCVmon)) + (1-Filter) * ReadbackV[b][1];
        if(ReadbackV[b][1] > MaxESIvoltage) MaxESIvoltage = ReadbackV[b][1];
        i = ESIarray[b].ESIchan[1].DCImon.Chan;
        if(ESIarray[b].Rev == 2)
        {
          ReadbackV[b][1] = Setpoints[b][1];
          float current = Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[1].DCImon);
          current = (current - (abs(ReadbackV[b][1]) / abs(ESIarray[b].ESIchan[1].MaxVoltage * 10))) / 0.4;
          ReadbackI[b][1] = Filter * (current + (1-Filter) * ReadbackI[b][1]);
          if(ReadbackI[b][1] < 0) ReadbackI[b][1] = 0;
        }
        else ReadbackI[b][1] = Filter * (Counts2Value(ADCvals[i],&ESIarray[b].ESIchan[1].DCImon)) + (1-Filter) * ReadbackI[b][1];
      }
      // Test the current limits and disable if limit is exceeded
      if((ESIarray[b].Rev == 1) || (ESIarray[b].Rev == 2))
      {
        if((ReadbackI[b][0] > ESIarray[b].ESIchan[0].MaxCurrent) && (ESIarray[b].ESIchan[0].Enable))
        {
          if((ESIarray[b].ESIchan[0].MaxCurrent > 0) && (ESIcurrentTest))
          {
            ESIarray[b].ESIchan[0].Enable = false;
            ESIarray[b].Enable = false;
            DisplayMessageButtonDismiss("ESI excess current!");
          }
        }
        if((ReadbackI[b][1] > ESIarray[b].ESIchan[1].MaxCurrent) && (ESIarray[b].ESIchan[1].Enable))
        {
          if((ESIarray[b].ESIchan[1].MaxCurrent > 0) && (ESIcurrentTest))
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
        if(SetpointsR3[b] >=0 ) Imonitor[b] = (ReadbackI[b][0] - (SetpointsR3[b] / abs(ESIarray[b].ESIchan[0].MaxVoltage * 10))) / 1.53;
        else Imonitor[b] = (ReadbackI[b][1] - (abs(SetpointsR3[b]) / abs(ESIarray[b].ESIchan[1].MaxVoltage *10))) / 1.53;
        if(Imonitor[b] < 0) Imonitor[b] = 0;
        if(ESIarray[b].ESIchan[0].MaxCurrent > 0)
        {
          if((Imonitor[b] > ESIarray[b].ESIchan[0].MaxCurrent) || (Imonitor[b] > ESIarray[b].ESIchan[1].MaxCurrent))
          {
            if((--ESIoverCurrentTimer[b] <= 0) && (ESIcurrentTest))
            {
               ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
               ESIarray[b].Enable = false;
               DisplayMessageButtonDismiss("ESI excess current!");
            }
          }
          else ESIoverCurrentTimer[b] = 5;
        }
      }
      if(ESIarray[b].Rev == 5)
      {
        // This supports the rev 4.1, 4.2, and 4.3 hardware. This hardware uses the G series modules. Rev
        // 4.1 and 4.2 read the current into the modules and load current is estimated using an eperical equation.
        // Rev 4.3 hardware has actual current output monitors. The gain and offset are also flags for this
        // calibration function if m=1 and b=0 then its assumed to be hardware 4.1 or 4.2. There are debug 
        // commands that allow the setup memory to be edited.
        // Need to develope a calibration function for these supplies.
        // Uncomment to force parameters for rev 4.1 or 4.2
        //ESIarray[b].ESIchan[0].DCImon.m = 1; 
        //ESIarray[b].ESIchan[0].DCImon.b = 0;
        //ESIarray[b].ESIchan[1].DCImon.m = 1;
        //ESIarray[b].ESIchan[1].DCImon.b = 0;
        if((ESIarray[b].ESIchan[0].DCImon.m == 1) && (ESIarray[b].ESIchan[0].DCImon.b == 0))
        {
          if(SetpointsR3[b] < 0) ReadbackV[b][0] = ReadbackV[b][1];
          if(SetpointsR3[b] >=0 ) Imonitor[b] = (ReadbackI[b][0] - (SetpointsR3[b] * 1.3728 - 166.04))/19600.0;
          else Imonitor[b] = (ReadbackI[b][1] - (abs(SetpointsR3[b]) * 1.5797 - 147.91))/19600.0;
        } else
        {
          if(SetpointsR3[b] < 0) ReadbackV[b][0] = ReadbackV[b][1];
          if(SetpointsR3[b] >=0 ) Imonitor[b] = ReadbackI[b][0];
          else Imonitor[b] = ReadbackI[b][1];
        }
        if(Imonitor[b] < 0) Imonitor[b] = 0;
        if(ESIarray[b].ESIchan[0].MaxCurrent > 0)
        {
          if((Imonitor[b] > ESIarray[b].ESIchan[0].MaxCurrent) || (Imonitor[b] > ESIarray[b].ESIchan[1].MaxCurrent))
          {
            if((--ESIoverCurrentTimer[b] <= 0) && (ESIcurrentTest))
            {
               ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
               ESIarray[b].Enable = false;
               DisplayMessageButtonDismiss("ESI excess current!");
            }
          }
          else ESIoverCurrentTimer[b] = 5;
        }
      }
      if((ESIarray[b].Rev == 6) || (ESIarray[b].Rev == 8))
      {
        if(SetpointsR3[b] < 0) ReadbackV[b][0] = ReadbackV[b][1];
        if(SetpointsR3[b] >=0 ) Imonitor[b] = ReadbackI[b][0];
        else Imonitor[b] = ReadbackI[b][1];
        if(Imonitor[b] < 0) Imonitor[b] = 0;
        if(ESIarray[b].ESIchan[0].MaxCurrent > 0)
        {
          if((Imonitor[b] > ESIarray[b].ESIchan[0].MaxCurrent) || (Imonitor[b] > ESIarray[b].ESIchan[1].MaxCurrent))
          {
            if((--ESIoverCurrentTimer[b] <= 0) && (ESIcurrentTest))
            {
               ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
               ESIarray[b].Enable = false;
               DisplayMessageButtonDismiss("ESI excess current!");
            }
          }
          else ESIoverCurrentTimer[b] = 5;
        }
      }
      if(ESIarray[b].Rev == 7)
      {

        if(((ESIarray[b].ESIchan[0].MaxCurrent > 0) && (ReadbackI[b][0] > ESIarray[b].ESIchan[0].MaxCurrent)) ||
           ((ESIarray[b].ESIchan[1].MaxCurrent > 0) && (ReadbackI[b][1] > ESIarray[b].ESIchan[1].MaxCurrent)))
        {
          if((--ESIoverCurrentTimer[b] <= 0) && (ESIcurrentTest))
          {
             ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
             ESIarray[b].Enable = false;
             DisplayMessageButtonDismiss("ESI excess current!");
          }
        }
        else ESIoverCurrentTimer[b] = 5;
      }
      if(ESIarray[b].Rev == 4)
      {
        ReadbackV[b][0] = Setpoints[b][0];
        ReadbackV[b][1] = Setpoints[b][1];
        ReadbackI[b][0] = abs((ReadbackI[b][0] - (abs(Setpoints[b][0]) / abs(ESIarray[b].ESIchan[0].MaxVoltage * 10))) / 1.53);
        ReadbackI[b][1] = abs((ReadbackI[b][1] - (abs(Setpoints[b][1]) / abs(ESIarray[b].ESIchan[1].MaxVoltage *10))) / 1.53);

        if(((ESIarray[b].ESIchan[0].MaxCurrent > 0) && (ReadbackI[b][0] > ESIarray[b].ESIchan[0].MaxCurrent)) ||
           ((ESIarray[b].ESIchan[1].MaxCurrent > 0) && (ReadbackI[b][1] > ESIarray[b].ESIchan[1].MaxCurrent)))
        {
            if((--ESIoverCurrentTimer[b] <= 0) && (ESIcurrentTest))
            {
               ESIarray[b].ESIchan[0].Enable = ESIarray[b].ESIchan[1].Enable = false;
               ESIarray[b].Enable = false;
               DisplayMessageButtonDismiss("ESI excess current!");
            }   
        }
        else ESIoverCurrentTimer[b] = 5;
      }
      MaxESIvoltage = 0;
    }
    //if(abs(ReadbackV[b][0]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][0]);
    //if(abs(ReadbackV[b][1]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][1]);
  }
  MaxESIvoltage = 0;
  for(b=0;b<2;b++)
  {
    if(!ESIboards[b]) continue;
    if(abs(ReadbackV[b][0]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][0]);
    if(abs(ReadbackV[b][1]) > MaxESIvoltage) MaxESIvoltage = abs(ReadbackV[b][1]);
  }
  if((ESIarray[SelectedESIboard].Rev == 1) || (ESIarray[SelectedESIboard].Rev == 2) || (ESIarray[SelectedESIboard].Rev == 4) || (ESIarray[SelectedESIboard].Rev == 7))
  {
    esi = ESIarray[ESIchannel2board(ESIchannelLoaded)];
    esich = ESIarray[ESIchannel2board(ESIchannelLoaded)].ESIchan[(ESIchannelLoaded-1)&1];    
  }
  else if((ESIarray[SelectedESIboard].Rev == 3) || (ESIarray[SelectedESIboard].Rev == 5) || (ESIarray[SelectedESIboard].Rev == 6) || (ESIarray[SelectedESIboard].Rev == 8))
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
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8))
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
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8))
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
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8)) serial->println(ESIarray[ESIchannel2board(chan)].VoltageSetpoint);
  else serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageSetpoint);
}

// Reads the selected channels output voltage readback
void GetESIchannelV(int chan)
{
//  if(ESIarray[ESIchannel2board(chan)].Rev == 3)
//  {
//      SetErrorCode(ERR_BADCMD);
//      SendNAK;
//      return;    
//  }
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8)) serial->println(ReadbackV[ESIchannel2board(chan)][0]);
  else serial->println(ReadbackV[ESIchannel2board(chan)][(chan-1)&1]);
  //if(!SerialMute) serial->println(ReadbackV[ESIchannel2board(chan)][(chan-1)&1]);
}

// Reads the selected channels output current
void GetESIchannelI(int chan)
{
  if(!ValidESIchannel(chan)) return;
  if(ESIarray[ESIchannel2board(chan)].Rev == 8) BADARG;
  SendACKonly;
  if(SerialMute) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6)) serial->println(Imonitor[ESIchannel2board(chan)]);
  else serial->println(ReadbackI[ESIchannel2board(chan)][(chan-1)&1]);
}

// Reads the selected channels maximum voltage output
void GetESIchannelMax(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8)) serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[0].VoltageLimit);
  else serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].VoltageLimit);
}

// Reads the selected channels minimum voltage output
void GetESIchannelMin(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8)) serial->println(ESIarray[ESIchannel2board(chan)].ESIchan[1].VoltageLimit);
  else serial->println(0);
}

void SetESIchannelEnable(int chan)
{
  if(!ValidESIchannel(chan)) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8))  ESIarray[ESIchannel2board(chan)].Enable = true;
  else ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].Enable = true;
  SendACK;  
}

void SetESIchannelDisable(int chan)
{
  if(!ValidESIchannel(chan)) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8))  ESIarray[ESIchannel2board(chan)].Enable = false;
  else ESIarray[ESIchannel2board(chan)].ESIchan[(chan-1)&1].Enable = false;
  SendACK;  
}

void GetESIstatus(int chan)
{
  if(!ValidESIchannel(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  if((ESIarray[ESIchannel2board(chan)].Rev == 3) || (ESIarray[ESIchannel2board(chan)].Rev == 5) || (ESIarray[ESIchannel2board(chan)].Rev == 6) || (ESIarray[ESIchannel2board(chan)].Rev == 8))
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
  ESIarray[module-1].ESIchan[0].MaxVoltage = value;
  // Adjust the calibration gain
  if(ESIarray[module-1].Rev == 5) ESIarray[module-1].ESIchan[0].DCctrl.m = 25000/value;
  else ESIarray[module-1].ESIchan[0].DCctrl.m = 65535/value;
  // For rev 6 set the current readback gains
  if((ESIarray[module-1].Rev == 6)||(ESIarray[module-1].Rev == 8)||(ESIarray[module-1].Rev == 7))
  {
    ESIarray[module-1].ESIchan[0].DCImon.m = 65535.0 / (1000.0 / (float)value);
    ESIarray[module-1].ESIchan[0].DCImon.b = 0; 
  }
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
  if(ESIarray[module-1].Rev == 5) ESIarray[module-1].ESIchan[1].DCctrl.m = 24000/(value);
  else ESIarray[module-1].ESIchan[1].DCctrl.m = 65535/(value);
  // For rev 6 set the current readback gains
  if((ESIarray[module-1].Rev == 6)||(ESIarray[module-1].Rev == 8)||(ESIarray[module-1].Rev == 7))
  {
    ESIarray[module-1].ESIchan[1].DCImon.m = 65535.0 / (1000.0 / (float)abs(value));
    ESIarray[module-1].ESIchan[1].DCImon.b = 0; 
  }
  SendACK;
}

// This function saves the ESI module data to EEPROM. All detected ESI modules are saved.
void SaveESI2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<2; b++)
  {
    if(ESIboards[b])
    {
      SelectBoard(b);
      if (WriteEEPROM(&ESIarray[b], ESIarray[b].EEPROMadr, 0, sizeof(ESIdata)) != 0) berr = true;
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

void SetESIgate(char *channel, char *state)
{
  String token;

  token = channel;
  int ch = token.toInt();
  if((ch < 1) || (ch >2)) BADARG;
  int brd = ESIchannel2board(ch);
  token = state;
  if((token != "TRUE") && (token != "FALSE")) BADARG;
  if(token == "TRUE") ESIarray[brd].EnableGatting[ch-1] = true;
  else ESIarray[brd].EnableGatting[ch-1] = false;
  ESIgate = esi.EnableGatting[ESIchannel-1] = esidata.EnableGatting[ESIchannel-1];
  ESIgateChangeProcess(ESIarray[brd].EnableGatting[ch-1],ch);
  SendACK;
}

void GetESIgate(int channel)
{
  if((channel < 1) || (channel >2)) BADARG;
  int brd = ESIchannel2board(channel);
  SendACKonly;
  if(SerialMute) return;
  if(ESIarray[brd].EnableGatting[channel-1] == true) serial->println("TRUE");
  else serial->println("FALSE");
}

void SetESIgateEnable(char *module, char *state)
{
  String token;

  token = module;
  int mod = token.toInt();
  if((mod < 1) || (mod >2)) BADARG;
  token = state;
  if((token != "TRUE") && (token != "FALSE")) BADARG;
  if(token == "TRUE") ESIarray[mod -1].SystemGatable = true;
  else ESIarray[mod -1].SystemGatable = false;
  SendACK;
}

void SetESIramp(int  module, int ramp)
{
  if((module < 1) || (module >2)) BADARG;
  ESIarray[module -1].ESIstepsize = ramp;
  SendACK;
}

void GetESIramp(int module)
{
  if((module < 1) || (module >2)) BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->println(ESIarray[module -1].ESIstepsize);
}

// Interactive calibration function for ESI rev 4.3
void calESI4P3(char *mod, char *chan)
{
  String token;

  token = mod;
  int module = token.toInt();
  if((module < 1) || (module >2)) BADARG;
  int b = module - 1;
  token = chan;
  int ch;
  if(token == "POS") ch = 0;
  else if(token == "NEG") ch = 1;
  else BADARG;
  // Enable the module and set the relays for select voltage
  ESIarray[b].Rev = 5;
  ESIarray[b].Enable = true;
  if(ch == 0) ESIarray[b].VoltageSetpoint = 1;
  else ESIarray[b].VoltageSetpoint = -1;
  ESI_loop();
  // Ask for the power supply voltage
  ESIarray[b].ESIchan[ch].MaxVoltage = UserInputFloat("Enter module voltaage : ", ReadAllSerial);
  ESIarray[b].ESIchan[ch].VoltageLimit = ESIarray[b].ESIchan[ch].MaxVoltage;
  // Set voltage point 1 and ask for actual voltage
  SelectBoard(b);
  AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[ch].DCctrl.Chan,3000);    // Set voltage point 1 DAC value
  float v1 = UserInputFloat("Enter measured ESI voltaage : ", ReadAllSerial);
  int Vadc1 = AD7994(ESIarray[b].ADCadr, ESIarray[b].ESIchan[ch].DCVmon.Chan, 20);
  int Iadc1 = AD7994(ESIarray[b].ADCadr, ESIarray[b].ESIchan[ch].DCImon.Chan, 20);
  // Set voltage point 2 and ask for actual voltage
  SelectBoard(b);
  AD5625(ESIarray[b].DACadr,ESIarray[b].ESIchan[ch].DCctrl.Chan,10000);    // Set voltage point 2 DAC value
  float v2 = UserInputFloat("Enter measured ESI voltaage : ", ReadAllSerial);
  int Vadc2 = AD7994(ESIarray[b].ADCadr, ESIarray[b].ESIchan[ch].DCVmon.Chan, 20);
  int Iadc2 = AD7994(ESIarray[b].ADCadr, ESIarray[b].ESIchan[ch].DCImon.Chan, 20);
  // Calculate and apply calibration parameters
  // count = value * m + b
  // value = (count - b)/m
  // v1 = 1000/m   - b/m
  // v2 = 10000/m - b/m
  // v1 - v2 = (200 - 10000)/m 
  // m = (1000 - 10000)/(v1 - v2)
  // b = 10000 - v2 * m
  float M = (3000 - 10000)/(v1 - v2);
  float B = 10000 - v2 * M;
  serial->println("DAC calibration parameters:");
  serial->println(M);
  serial->println(B);
  ESIarray[b].ESIchan[ch].DCctrl.m = M;
  ESIarray[b].ESIchan[ch].DCctrl.b = B;
  M = (Vadc1 - Vadc2)/(v1 - v2);
  B = Vadc2 - v2 * M;
  serial->println("ADC voltage readback calibration parameters:");
  serial->println(M);
  serial->println(B);
  ESIarray[b].ESIchan[ch].DCVmon.m = M;
  ESIarray[b].ESIchan[ch].DCVmon.b = B;
  // Convert measured voltage into current in milli amps
  v1 = abs(v1) / 50000.0;
  v2 = abs(v2) / 50000.0;
  M = (float)(Iadc1 - Iadc2)/(v1 - v2);
  B = (float)Iadc2 - v2 * M;
  serial->println("ADC current monitor calibration parameters:");
  serial->println(M);
  serial->println(B);
  ESIarray[b].ESIchan[ch].DCImon.m = M;
  ESIarray[b].ESIchan[ch].DCImon.b = B;
  //
  ESIarray[b].VoltageSetpoint=0;
}
