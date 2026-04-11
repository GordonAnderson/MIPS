//
// HVPSinterface
//
// This is an optional module that provides a UI on the MIPS box for the HV power supply interface.
// This interface is designed to control applied kilovolt HV supplies.
// The interface connects to the Wire1 TWI interface using the Aux connector on MIPS. The host communicates
// using the GTWI and STWI commands so this driver is not required for the host to talk to the
// interface.
// Up to 4 interfaces can be used and there TWI addresses are expect in the range of 0x20 through 0x23.
//
// Gordon Anderson
// Dec 27, 2025
//
// To do:
//  1.) If the host changes a parameter this interface will not know.

#include "Variants.h"
#if HVPSinterface
#include "Arduino.h"
#include <Wire.h>
#include "Hardware.h"
#include "HVPSinterface.h"
#include <WireServer.h>

//MIPS Threads
Thread HVPSinterfaceThread  = Thread();

WireServer  ws;
int         numHVPSinterface = 0;
uint8_t     HVPSinterfaceTWIadd[4];

HVPSinterfaceData hvpsi;

float Vrb = 0;
float Irb = 0;
bool  Prb = true;

int HVPSinterfaceSelectModule = 0;

extern DialogBoxEntry HVPSinterfaceTWIsettings[];

char   *PolList = "POS,NEG";
char   Polarity[4] = "POS";

DialogBoxEntry HVPSinterfaceTWIHome[] = {
  {" Module"             , 0, 1, D_INT    , 1, 1, 1, 21,      false, "%2d",     &HVPSinterfaceSelectModule, NULL, HVPSinterfaceModuleSelected},
  {" Enable"             , 0, 2, D_ONOFF  , 0, 1, 1, 20,      false, NULL,      &hvpsi.power, NULL, NULL},
  {" Voltage"            , 0, 3, D_FLOAT  , 0, 20000, 10, 17, false, "%6.0f",   &hvpsi.setpoint, NULL, NULL},
  {" Polairty"           , 0, 4, D_LIST   , 0, 0, 10, 13,     true,  PolList,   Polarity, NULL, PolChange},
  {" V Monitor,V"        , 0, 5, D_FLOAT  , 0, 1, 1,17,       true,  "%6.0f",   &Vrb, NULL, NULL},
  {" I Monitor,mA"       , 0, 6, D_FLOAT  , 0, 1, 1,17,       true,  "%5.4f",   &Irb, NULL, NULL},
  {" Settings" , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, HVPSinterfaceTWIsettings, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry HVPSinterfaceTWIsettings[] = {
  {" Range, V"           , 0, 1, D_FLOAT  , -20000, 20000, 10, 17, false, "%6.0f",   &hvpsi.range, NULL, NULL},
  {" Reversable"         , 0, 2, D_YESNO  , 0, 1, 1, 20,      false, NULL,           &hvpsi.reversable, NULL, NULL},
  {" Current ena"        , 0, 3, D_YESNO  , 0, 1, 1, 20,      false, NULL,           &hvpsi.curMonEnabled, NULL, NULL},
  
  {" Max V, V"           , 0, 5, D_FLOAT  , -20000, 20000, 10, 17, false, "%6.0f",   &hvpsi.maxVoltage, NULL, NULL},
  {" Max I, mA"          , 0, 6, D_FLOAT  , 0, 10, 10, 17, false, "%6.3f",           &hvpsi.maxCurrent, NULL, NULL},
 
  {" Save settings"      , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveHVPSISettings, NULL},
  {" Restore settings"   , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreHVPSISettings, NULL},
  {" Return" , 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, HVPSinterfaceTWIHome, NULL, NULL},
  {NULL},
};

DialogBox HVPSinterfaceDialog = {
  {"HVPS interface", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, HVPSinterfaceTWIHome
};

MenuEntry MEHVPSinterface = {" HVPS interface", M_DIALOG, 0, 0, 0, NULL, &HVPSinterfaceDialog, NULL, NULL};

void SaveHVPSISettings(void)
{
  uint8_t  cmd;

  cmd = TWI_HVPSI_SET_Save;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1);
  DisplayMessage("Parameters Saved!", 2000);
}

void RestoreHVPSISettings(void)
{
  uint8_t  cmd;

  cmd = TWI_HVPSI_SET_Restore;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1);
  DisplayMessage("Parameters Restored!", 2000);
  HVPSinterfaceModuleSelected();
}

void PolChange(void)
{
  uint8_t  cmd;

  if(strcmp(Polarity,"POS") == 0) hvpsi.polarity = true;
  else hvpsi.polarity = false;
  cmd = TWI_HVPSI_SET_Pol;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
  ws.SendBool(hvpsi.polarity);
}

void HVPSinterfaceModuleSelected(void)
{
   HVPSinterfaceConfigure(HVPSinterfaceSelectModule-1);
}

void HVPSinterfaceUpdateUIoptions(void)
{
  // Set the UI based current monitoring flag
  DialogBoxEntry *de = GetDialogEntries(HVPSinterfaceTWIHome, "I Monitor,mA");
  if(hvpsi.curMonEnabled) de->Type = D_FLOAT;
  else de->Type = D_OFF;
  // Set the UI based reversable monitoring flag
  de = GetDialogEntries(HVPSinterfaceTWIHome, "Polairty");
  if(hvpsi.reversable) de->NoEdit = false;
  else 
  {
    de->NoEdit = true;
    // Set polarity in UI based on sign of range
    strcpy(Polarity,"POS");
    if(hvpsi.range < 0) strcpy(Polarity,"NEG");
  }
}

void HVPSinterfaceConfigure(int module)
{
  uint8_t  cmd;

  // Read the selected module parameters and update the data structure
  cmd = TWI_HVPSI_GET_Power;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadBool(&hvpsi.power);
  cmd = TWI_HVPSI_GET_Voltage;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadFloat(&hvpsi.setpoint);
  cmd = TWI_HVPSI_GET_Pol;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadBool(&hvpsi.polarity);
  cmd = TWI_HVPSI_GET_Range;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadFloat(&hvpsi.range);
  cmd = TWI_HVPSI_GET_MaxV;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadFloat(&hvpsi.maxVoltage);
  cmd = TWI_HVPSI_GET_MaxI;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadFloat(&hvpsi.maxCurrent);
  cmd = TWI_HVPSI_GET_Reversable;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadBool(&hvpsi.reversable);
  cmd = TWI_HVPSI_GET_CurEnable;
  ws.SendCommand(HVPSinterfaceTWIadd[module],&cmd,1);
  ws.ReadBool(&hvpsi.curMonEnabled);
  // Set the UI based on parameters
  HVPSinterfaceUpdateUIoptions();
}

void HVPSinterface_init(void)
{
  uint8_t cmd;

  Wire1.begin();
  Wire1.setClock(Wire1DefaultSpeed);
  ws.SetInterface(&Wire1);
  // Look for HVPSinterface modules, expected on Wire1 at
  // addresses 0x20 through 0x23. 
  for(uint8_t i=0x20;i<0x24;i++)
  {
    cmd = TWI_HVPSI_READ_ID;
    ws.SendCommand(i,&cmd,1);
    ws.ReadUnsignedByte(&cmd);
    if(cmd == DeviceID_HVPSI)
    {
      HVPSinterfaceTWIadd[numHVPSinterface] = i;
      numHVPSinterface++;
    }
  }
  if (numHVPSinterface > 0)
  {
    HVPSinterfaceConfigure(0);
    HVPSinterfaceSelectModule = 1;
    AddMainMenuEntry(&MEHVPSinterface);
    if (ActiveDialog == NULL) DialogBoxDisplay(&HVPSinterfaceDialog);
    HVPSinterfaceTWIHome[0].Max = numHVPSinterface;
    // Configure Threads
    HVPSinterfaceThread.setName("HVPSinterface");
    HVPSinterfaceThread.onRun(HVPSinterface_loop);
    HVPSinterfaceThread.setInterval(100);
    // Add threads to the controller
    control.add(&HVPSinterfaceThread);
  }
}

void HVPSinterface_loop(void)
{
  uint8_t  cmd;

  if (ActiveDialog != &HVPSinterfaceDialog) return;
  if(ActiveDialog->Changed)
  {
    uint8_t cmd;
    ActiveDialog->Changed = false;
    // User changed a value so send all data to the hardware
    if(ActiveDialog->Entry == HVPSinterfaceTWIHome)
    {
      cmd = TWI_HVPSI_SET_Power;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendBool(hvpsi.power);
      cmd = TWI_HVPSI_SET_Voltage;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendFloat(hvpsi.setpoint);
    }
    if(ActiveDialog->Entry == HVPSinterfaceTWIsettings)
    {
      cmd = TWI_HVPSI_SET_Range;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendFloat(hvpsi.range);
      cmd = TWI_HVPSI_SET_Reversable;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendBool(hvpsi.reversable);
      cmd = TWI_HVPSI_SET_CurEnable;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendBool(hvpsi.curMonEnabled);
      cmd = TWI_HVPSI_SET_MaxV;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendFloat(hvpsi.maxVoltage);
      cmd = TWI_HVPSI_SET_MaxI;
      ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1,true);
      ws.SendFloat(hvpsi.maxCurrent);
      HVPSinterfaceUpdateUIoptions();
    }
  }
  // Update the readbacks
  cmd = TWI_HVPSI_GET_Vrb;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1);
  ws.ReadFloat(&Vrb);
  cmd = TWI_HVPSI_GET_Irb;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1);
  ws.ReadFloat(&Irb);
  cmd = TWI_HVPSI_GET_Prb;
  ws.SendCommand(HVPSinterfaceTWIadd[HVPSinterfaceSelectModule-1],&cmd,1);
  ws.ReadBool(&Prb);

  RefreshAllDialogEntries(&HVPSinterfaceDialog);
}
#endif