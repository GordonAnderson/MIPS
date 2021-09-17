// HVPS module driver
//
#if HVPScode

#include "HVPS.h"
#include "Arduino.h"

//MIPS Threads
Thread HVPSthread  = Thread();

#define HVPS HVPSarray[SelectedHVPSboard]

HVPSdata     *HVPSarray[4]  = {NULL,NULL,NULL,NULL};
HVPSstate    *HVPSstates[4] = {NULL,NULL,NULL,NULL};
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
  {"SHVPS",   CMDfunction, 2, (char *)SetHVPSvoltage},          // Set HVPS channel voltage
  {"GHVPS",   CMDfunction, 1, (char *)GetHVPSvoltage},          // Return HVPS channel voltage, setpoint
  {"GHVPSV",  CMDfunction, 1, (char *)GetHVPSvoltageRB},        // Return HVPS channel voltage, readback
  {"SHVPSENA",CMDfunctionStr, 2, (char *)SetHVPSenable},        // Set HVPS channel enable, ON or OFF
  {"GHVPSENA",CMDfunction, 1, (char *)GetHVPSenable},           // Return HVPS channel enable, ON or OFF
  {0},
};

CommandList HVPSCmdList = { (Commands *)HVPSCmdArray, NULL };

// This function initalizes the UI based on the HVPS modules loaded
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
  ChannelCalibrate(&CC, Name,500,2000);
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
  ChannelCalibrate(&CC, Name,-500, -2000);
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
// This function send the DAC counts to the selected DAC channel.
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

// Returns board number for the HVPS channel number, 0 thru 7
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

// Processig loop called every 100mS
void HVPS_loop(void)
{
  int   i,j,k;
  float fVal;
  
  MaxHVvoltage = 0;
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
// SHVPS,chan,value
// GHVPS,chan
// GHVPSV,chan
// SHVPSENA,chan,value
// GHVPSENA,chan
//

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
  serial->println(HVPSarray[b]->HVPSCH[c].DCPctrl.m);
  serial->println(HVPSarray[b]->HVPSCH[c].DCPctrl.b);
  serial->println(HVPSarray[b]->HVPSCH[c].DCPmon.m);
  serial->println(HVPSarray[b]->HVPSCH[c].DCPmon.b);
  serial->println(HVPSarray[b]->HVPSCH[c].DCNctrl.m);
  serial->println(HVPSarray[b]->HVPSCH[c].DCNctrl.b);
  serial->println(HVPSarray[b]->HVPSCH[c].DCNmon.m);
  serial->println(HVPSarray[b]->HVPSCH[c].DCNmon.b);
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

#endif
