#if RFdriver2 == true

#include "Variants.h"
//#include <DIhandler.h>

#define MinFreq    400000
#define MaxFreq    5000000

extern bool NormalStartup;

// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.05         // Strong filter coefficent

//MIPS Threads
Thread RFdriverThread  = Thread();

#define RFDD RFDDarray[SelectedRFBoard]

RFdriverData *RFDDarray[4] = {NULL,NULL,NULL,NULL};
RFchannelData RFCD;          // Holds the selected channel's data
int  NumberOfRFChannels = 0; // Defines the number of RF channels supported. Set during intaliztion.
int  Channel = 1;            // User selected channel
int  SelectedRFChan  = 0;    // Active channel
int  SelectedRFBoard = 0;    // Active board, 0 thru 3
float MaxRFVoltage = 0;      // This value is set to the highest RF output voltage
bool Tuning = false;
bool TuneAbort = false;      // Not used in this driver, defined because its referenced elsewhere
int  TuningChannel = 0;

RFDRVstate   *RFstate[4][2] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
RFreadBacks  *RFrb[4][2]    = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
// These are the actively displayed values
float RFpVpp;
float RFnVpp;
float Power;

// Arc detection parameters. Not implemented in this RFdriver code, place holders so it will compile
int   RFarcCH = 0;
float RFarcDrv = 0;
float RFarcV = 0;

extern DialogBoxEntry RFdriverDialogEntriesPage2[];

DialogBoxEntry RFdriverDialogEntriesPage1[] = {
  {" RF channel", 0, 1, D_INT, 1, 2, 1, 21, false, "%2d", &Channel, NULL, SelectChannel},
  {" Freq, Hz"  , 0, 2, D_INT, MinFreq, MaxFreq, 1000, 16, false, "%7d", &RFCD.Freq, NULL, NULL},
  {" Drive %"   , 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &RFCD.DriveLevel, NULL, NULL},
  {" Vout Vpp"  , 0, 3, D_OFF, 0, 5000, 1, 18, false, "%5.0f", &RFCD.Setpoint, NULL, NULL},
  {" RF+ Vpp"   , 0, 5, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFpVpp, NULL, NULL},
  {" RF- Vpp"   , 0, 6, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFnVpp, NULL, NULL},
  {" Power, W"  , 0, 7, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.1f", &Power, NULL, NULL},
  {" Next page" , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, RFdriverDialogEntriesPage2, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char   *ModeList = "MANUAL,AUTO";
char   RFmode[8] = "MANUAL";
char   rfgateDI;
int8_t rfgateTrig;

DialogBoxEntry RFdriverDialogEntriesPage2[] = {
  {" Drive limit, %", 0, 1, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &RFCD.MaxDrive, NULL, NULL},
  {" Power limit, W", 0, 2, D_FLOAT, 0, 50, 1, 18, false, "%5.1f", &RFCD.MaxPower, NULL, NULL},
  {" Mode"          , 0, 3, D_LIST, 0, 0, 7, 16, false, ModeList, RFmode, NULL, RFmodeChange},
  {" Auto tune"     , 0, 6, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFat, NULL},
  {" Auto retune"   , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFart, NULL},
  {" Save settings" , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFdriverSettings, NULL},
  {" Restore settings", 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFdriverSettings, NULL},
  {" First page"         , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, RFdriverDialogEntriesPage1, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox RFdriverDialog = {
  {"RF driver parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, RFdriverDialogEntriesPage1
};

MenuEntry MERFdriverMonitor = {" RF driver module", M_DIALOG, 0, 0, 0, NULL, &RFdriverDialog, NULL, NULL};

// Called from the UI to enable auto tune, only works in manual mode
void RFat(void)
{
  if(RFCD.RFmode != RF_MANUAL) return;
  // Send command to RF driver to auto tune and pop up the tune box and set tuning flag.
  // The RFdriver loop to poll RF driver card and dismiss message when tuning has finished.
  // When tuniing is finished we need to read the frequency from the RF driver module.
  Tuning = true;
  TuningChannel = SelectedRFChan;
  TWIsetByte(RFdrvCMDadd(RFDDarray[SelectedRFBoard]->EEPROMadr), SelectedRFBoard, TWI_RF_SET_CHAN, (SelectedRFChan & 1) +1);
  TWIcmd(RFdrvCMDadd(RFDDarray[SelectedRFBoard]->EEPROMadr), SelectedRFBoard, TWI_RF_SET_ATUNE);
  DisplayMessage("Tune in process");
}

// Called from the UI to enable auto retune, only works in manual mode
void RFart(void)
{
  if(RFCD.RFmode != RF_MANUAL) return;
  // Send command to RF driver to auto retune and pop up the tune box and set tuning flag
  // The RFdriver loop to poll RF driver card and dismiss message when tuning has finished
  // When tuniing is finished we need to read the frequency from the RF driver module.
  Tuning = true;
  TuningChannel = SelectedRFChan;
  TWIsetByte(RFdrvCMDadd(RFDDarray[SelectedRFBoard]->EEPROMadr), SelectedRFBoard, TWI_RF_SET_CHAN, (SelectedRFChan & 1) +1);
  TWIcmd(RFdrvCMDadd(RFDDarray[SelectedRFBoard]->EEPROMadr), SelectedRFBoard, TWI_RF_SET_RTUNE);
  DisplayMessage("Retune in process");
}

void RFmodeChange(void)
{
  String mode;

  mode = RFmode;
  if (mode == "MANUAL")
  {
    RFCD.RFmode = RF_MANUAL;
    RFdriverDialogEntriesPage1[2].Type = D_FLOAT;
    RFdriverDialogEntriesPage1[3].Type = D_OFF;
  }
  else if (mode == "AUTO")
  {
    // If RFmode just changed to auto then set the voltage setpoint to the current value
    if(RFCD.RFmode != RF_AUTO)
    {
        RFCD.Setpoint = (RFpVpp + RFnVpp) / 2;
    }
    RFCD.RFmode = RF_AUTO;
    RFdriverDialogEntriesPage1[2].Type = D_OFF;
    RFdriverDialogEntriesPage1[3].Type = D_FLOAT;
  }
  if(ActiveDialog == &RFdriverDialog) ActiveDialog->Changed = true;
}

void RFgateChange(void)
{
}

uint8_t RFdrvCMDadd(uint8_t addr)
{
  if((addr & 0x20) == 0) return(addr | 0x20);
  return(addr | 0x18);
}

// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveRFdriverSettings(void)
{
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  RFDD->Size = sizeof(RFdriverData);
  if (WriteEEPROM(RFDD, RFDD->EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

// Restore the parameters from the EEPROM on the RFdriver board. Read the EEPROM and make sure the board name
// matches what is expected, only load if its correct.
void RestoreRFdriverSettings(void)
{
  RestoreRFdriverSettings(false);
}

void RestoreRFdriverSettings(bool NoDisplay)
{
  RFdriverData rfdd;

  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  if (ReadEEPROM(&rfdd, RFDD->EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    if (strcmp(rfdd.Name, "RFdriver") == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if(rfdd.Size != sizeof(RFdriverData)) rfdd.Size = sizeof(RFdriverData);
      rfdd.EEPROMadr = RFDD->EEPROMadr;
      memcpy(RFDD, &rfdd, rfdd.Size);
      RFCD = RFDD->RFCD[SelectedRFChan & 1];
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      SelectChannel();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

// Called after the user selects a channel
void SelectChannel(void)
{
  SelectedRFChan = Channel - 1;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  RFCD = RFDD->RFCD[(Channel - 1) & 1];
  SelectBoard(SelectedRFBoard);
  if (RFCD.RFmode == RF_MANUAL) strcpy(RFmode, "MANUAL");
  else strcpy(RFmode, "AUTO");
  RFmodeChange();
  RFgateChange();
  // Set the power limit
  RFdriverDialogEntriesPage2[1].Max = RFDD->PowerLimit;
  // If rev 2 then do not display the RF- channel
  if((RFDD->Rev == 2) || (RFDD->Rev == 4)) RFdriverDialogEntriesPage1[5].Type = D_OFF;
  else RFdriverDialogEntriesPage1[5].Type = D_FLOAT;
  // Update the display
  if (ActiveDialog == &RFdriverDialog) DialogBoxDisplay(&RFdriverDialog);
}

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected.
// The selected channel SC, is 0 thru number of channels - 1
int8_t BoardFromSelectedChannel(int8_t SC)
{
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if (SC <= 1)
  {
    if (RFDDarray[0] != NULL) return (0);
    if (RFDDarray[1] != NULL) return (1);
    return (-1);
  }
  if(SC > 7) return(-1);
  if(RFDDarray[SC >> 1] == NULL) 
  {
    if(RFDDarray[2] != NULL) return(2);
    return(-1);
  }
  return(SC >> 1);
}

// This function is called at powerup to initiaize the RF driver.
// The board parameter (0 or 1) defines the board select parameter where this card was found.
// Up to two boards are supported. Board A, or 0, is always called first.
// If only one board is installed it can be board 0 or 1.
void RFdriver_init(int8_t Board, int8_t addr)
{
  DialogBox *sd;

  if(NumberOfRFChannels >= 4) Board += 2;
  if(NumberOfRFChannels >= 2) if(Board == 0) Board += 2;
  // Allocate the RFdriver structures
  RFDDarray[Board]  = new RFdriverData;
  RFstate[Board][0] = new RFDRVstate;
  RFstate[Board][1] = new RFDRVstate;
  RFstate[Board][0]->update = RFstate[Board][0]->update = false;
  RFrb[Board][0]    = new RFreadBacks;
  RFrb[Board][1]    = new RFreadBacks;
  // Set active board to board being inited
  SelectedRFBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  sd = ActiveDialog;
  ActiveDialog = NULL;
  Channel = NumberOfRFChannels + 1;
  SelectedRFChan = Channel - 1;
  RFDDarray[Board]->EEPROMadr = addr;
//  SelectedRFChan=0;
  RestoreRFdriverSettings(true);
  // Define the initial selected channel as 0 and setup
  Channel = 1;
  SelectChannel();
  ActiveDialog = sd;
  // Setup the menu if this is the very first call to this init function
  if (NumberOfRFChannels == 0)
  {
    AddMainMenuEntry(&MERFdriverMonitor);
    if (ActiveDialog == NULL) DialogBoxDisplay(&RFdriverDialog);
    // Configure Threads
    RFdriverThread.setName("RFdriver");
    RFdriverThread.onRun(RFdriver_loop);
    RFdriverThread.setInterval(100);
    // Add threads to the controller
    control.add(&RFdriverThread);
  }
  NumberOfRFChannels += 2;  // Always add two channels for each board
  // Set the maximum number of channels in the selection menu
  RFdriverDialogEntriesPage1[0].Max = NumberOfRFChannels;
}

void RFdriver_loop(void)
{
  float fval;
  int   ival;
  
  MaxRFVoltage = 0;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  if(ActiveDialog == &RFdriverDialog)
  {
    if(ActiveDialog->Changed) 
    {
      // This stores any changes back to the selected channels data structure
      RFDD->RFCD[SelectedRFChan & 1] = RFCD;    
      // Set the Drive level limit in the UI menu
      RFdriverDialogEntriesPage1[2].Max = RFCD.MaxDrive;
      ActiveDialog->Changed = false;
    }
  }
  // Process each RF driver channel. Look for changes in parameters and update as needed
  for(int c = 0; c < NumberOfRFChannels; c++)
  { 
    int b = BoardFromSelectedChannel(c);
    int C = (c & 1);  // Driver channel number
    if(RFDDarray[b] != NULL)
    {
      SelectBoard(b);

      TWIsetByte(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_CHAN, C+1);  // Select this channel on the selected board
      // Update the readback structure
      if(RFrb[b] != NULL)
      {
        TWIreadBlock(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b,TWI_RF_READ_READBACKS, (void *)RFrb[b][C], sizeof(RFreadBacks));
        if(RFrb[b][C]->RFP > MaxRFVoltage) MaxRFVoltage = RFrb[b][C]->RFP;
        if(RFrb[b][C]->RFN > MaxRFVoltage) MaxRFVoltage = RFrb[b][C]->RFN;
        if(c == SelectedRFChan)
        {
          // Update the display values
          RFpVpp = RFrb[b][C]->RFP;
          RFnVpp = RFrb[b][C]->RFN;
          Power  = RFrb[b][C]->PWR;
        }
      }
      if(Tuning & (TuningChannel == c))
      {
        // Get tuning status and if finished then clear flag and dismiss message
        if(TWIread8bitUnsigned(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b,TWI_RF_READ_TUNE, &ival))
        {
          if(!ival)
          {
            Tuning = false;
            DismissMessage();
          }
        }
      }
      if(RFstate[b][C]->update || (RFstate[b][C]->DriveLevel != RFDDarray[b]->RFCD[C].DriveLevel)) TWIsetFloat(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_DRIVE,  RFstate[b][C]->DriveLevel = RFDDarray[b]->RFCD[C].DriveLevel);
      if(RFstate[b][C]->update || (RFstate[b][C]->Freq       != RFDDarray[b]->RFCD[C].Freq))       TWIsetInt  (RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_FREQ,   RFstate[b][C]->Freq =       RFDDarray[b]->RFCD[C].Freq);
      if(RFstate[b][C]->update || (RFstate[b][C]->Setpoint   != RFDDarray[b]->RFCD[C].Setpoint))   TWIsetFloat(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_VRF,    RFstate[b][C]->Setpoint =   RFDDarray[b]->RFCD[C].Setpoint);
      if(RFstate[b][C]->update || (RFstate[b][C]->MaxDrive   != RFDDarray[b]->RFCD[C].MaxDrive))   TWIsetFloat(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_MAXDRV, RFstate[b][C]->MaxDrive =   RFDDarray[b]->RFCD[C].MaxDrive);
      if(RFstate[b][C]->update || (RFstate[b][C]->MaxPower   != RFDDarray[b]->RFCD[C].MaxPower))   TWIsetFloat(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_MAXPWR, RFstate[b][C]->MaxPower =   RFDDarray[b]->RFCD[C].MaxPower);
      if(RFstate[b][C]->update || (RFstate[b][C]->RFmode     != RFDDarray[b]->RFCD[C].RFmode))
      {
        if(RFDDarray[b]->RFCD[C].RFmode == RF_AUTO) TWIsetBool(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_MODE, true);
        else TWIsetBool(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_MODE, false);
        RFstate[b][C]->RFmode = RFDDarray[b]->RFCD[C].RFmode;
      }
      // Readback Drive and Frequency in case the module changed the values
      if(TWIreadFloat(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b,TWI_RF_READ_DRIVE, &fval)) RFstate[b][C]->DriveLevel = RFDDarray[b]->RFCD[C].DriveLevel = fval;
      if(TWIread32bitInt(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b,TWI_RF_READ_FREQ, &ival)) RFstate[b][C]->Freq = RFDDarray[b]->RFCD[C].Freq = ival;
      RFstate[b][C]->update = false;      
    }
  }
  // Reselect the active channel's board
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  // Read the ADC monitor values for the selected channel.
  RFpVpp = RFrb[SelectedRFBoard][SelectedRFChan & 1]->RFP;
  RFnVpp = RFrb[SelectedRFBoard][SelectedRFChan & 1]->RFN;
  Power = RFrb[SelectedRFBoard][SelectedRFChan & 1]->PWR;
  if (ActiveDialog == &RFdriverDialog) RefreshAllDialogEntries(&RFdriverDialog);
  RFCD = RFDD->RFCD[SelectedRFChan & 1]; 
}

//
// This section contains all the serial command processing routines.
//

// Reports the number of RF output channels avalible.
void RFnumber(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfRFChannels);
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsChannelValid(int channel, bool Response = true)
{
  if ((channel >= 1) && (channel <= NumberOfRFChannels)) return true;
  if(!Response) return false;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Set a channels freqency
void RFfreq(int channel, int freq)
{
  if (!IsChannelValid(channel)) return;
  // If freq value is invalid send NAK and exit
  if ((freq < MinFreq) || (freq > MaxFreq))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the frequency
  SendACK;
  RFDDarray[BoardFromSelectedChannel(channel - 1)]->RFCD[(channel - 1) & 1].Freq = freq;
}

void RFdrive(int channel, float Drive)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  i = BoardFromSelectedChannel(channel - 1);
  // If Drive value is invalid exit
  if ((Drive < 0) || (Drive > RFDDarray[i]->RFCD[(channel - 1) & 1].MaxDrive)) return;
  RFDDarray[i]->RFCD[(channel - 1) & 1].DriveLevel = Drive;
}

void RFdrive(char *Chan, char *Val)
{
  int   channel, i;
  float Drive;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Drive);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  i = BoardFromSelectedChannel(channel - 1);
  // If Drive value is invalid send NAK and exit
  if ((Drive < 0) || (Drive > RFDDarray[i]->RFCD[(channel - 1) & 1].MaxDrive))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  RFDDarray[i]->RFCD[(channel - 1) & 1].DriveLevel = Drive;
  if (channel - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
}

void RFvoltage(int channel, float Voltage)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  // If Drive value is invalid exit
  if ((Voltage < 0) || (Voltage > 5000.0)) return;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i]->RFCD[(channel - 1) & 1].Setpoint = Voltage;
}

void RFvoltage(char *Chan, char *Val)
{
  int   channel, i;
  float Voltage;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Voltage);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If Drive value is invalid send NAK and exit
  if ((Voltage < 0) || (Voltage > 4000.0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i]->RFCD[(channel - 1) & 1].Setpoint = Voltage;
}

// Reports the selected channels mode, MANUAL or AUTO
void RFmodeReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // report the mode
  SendACKonly;
  if(SerialMute) return;
  int i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode == RF_MANUAL) serial->println("MANUAL");
  else serial->println("AUTO");
}

// Sets the selected channels mode, MANUAL or AUTO
void RFmodeSet(char *chan, char *mode)
{
  int    channel;
  String sToken;

  sToken = chan;
  channel = sToken.toInt();
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  sToken = mode;
  if((sToken != "AUTO") && (sToken != "MANUAL"))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;    
  }
  SendACK;
  int i = BoardFromSelectedChannel(channel - 1);
  if(sToken == "MANUAL") RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode = RF_MANUAL;
  else 
  {
    RFDDarray[i]->RFCD[(channel - 1) & 1].Setpoint = (RFrb[i][(channel - 1) & 1]->RFP + RFrb[i][(channel - 1) & 1]->RFN) / 2;
    RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode = RF_AUTO;
  }
  // If this channel is displayed on the MIPS UI then update
  if((channel-1) == SelectedRFChan)
  {
    RFCD.Setpoint = RFDDarray[i]->RFCD[(channel - 1) & 1].Setpoint;
    RFCD.RFmode = RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode;
    if(RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode == RF_MANUAL) strcpy(RFmode, "MANUAL");
    else strcpy(RFmode, "AUTO");
    RFmodeChange();
    // If this page is being dispayed then refresh
    if(ActiveDialog == &RFdriverDialog) DialogBoxDisplay(&RFdriverDialog);
  }
}

// Report a channels frequency
void RFfreqReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Report the channels frequency
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i]->RFCD[(channel - 1) & 1].Freq);
}

// Auto tune a selected channel
void RFautoTune(int channel)
{
  int i,b;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning)
  {
    SetErrorCode(ERR_TUNEINPROCESS);
    SendNAK;
    return;
  }
  // Exit if not in manual mode for this channel
  i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode != RF_MANUAL)
  {
    SetErrorCode(ERR_NOTINMANMODE);
    SendNAK;
    return;    
  }
  // Set the tune flag and exit
  SendACK;
  Tuning = true;
  TuningChannel = channel-1;
  b = BoardFromSelectedChannel(channel-1);
  TWIsetByte(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_CHAN, (TuningChannel & 1) +1);
  TWIcmd(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_ATUNE);
  DisplayMessage("Tune in process");
}

// Auto retune a selected channel, this function starts and the current freq and power settings and
// peaks the output by making small adjustments
void RFautoRetune(int channel)
{
  int i;
  int b;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning)
  {
    SetErrorCode(ERR_TUNEINPROCESS);
    SendNAK;
    return;
  }
  // Exit if not in manual mode for this channel
  i = BoardFromSelectedChannel(channel - 1);
  if(RFDDarray[i]->RFCD[(channel - 1) & 1].RFmode != RF_MANUAL)
  {
    SetErrorCode(ERR_NOTINMANMODE);
    SendNAK;
    return;    
  }
  // Set the tune flag and exit
  SendACK;
  Tuning = true;
  TuningChannel = channel-1;
  b = BoardFromSelectedChannel(channel-1);
  TWIsetByte(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_CHAN, (TuningChannel & 1) +1);
  TWIcmd(RFdrvCMDadd(RFDDarray[b]->EEPROMadr), b, TWI_RF_SET_RTUNE);
  DisplayMessage("Retune in process");
}

void RFvoltageReportP(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFrb[i][(channel - 1) & 1]->RFP);
}

void RFvoltageReportN(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFrb[i][(channel - 1) & 1]->RFN);
}

void RFdriveReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i]->RFCD[(channel - 1) & 1].DriveLevel);
}

// Reports the voltage setpoint
void RFvoltageReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i]->RFCD[(channel - 1) & 1].Setpoint);
}

void RFheadPower(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFrb[i][(channel - 1) & 1]->PWR);
}

// Reports the frequency and Vpp for + and - channels for each RF channel in system.
void RFreportAll(void)
{
  int  i,brd;

  if (SerialMute) return;
  SendACKonly;
  for(i=1;i<=8;i++)
  {
    if (!IsChannelValid(i,false)) break;
    brd = BoardFromSelectedChannel(i - 1);
    if(i > 1) serial->print(",");
    serial->print(RFDDarray[brd]->RFCD[(i - 1) & 1].Freq);
    serial->print(",");
    serial->print(RFDDarray[brd]->RFCD[(i - 1) & 1].DriveLevel);
    serial->print(",");
    serial->print(RFrb[brd][(i - 1) & 1]->RFP);
    serial->print(",");
    serial->print(RFrb[brd][(i - 1) & 1]->RFN);
  }
  serial->println("");
}

// Sets the defined channels calibration parameters to the values passed. The 
// parameters are in the ring buffer when this function is called.
// channel,slope, intercept
// This function sets the pos and neg monitor calibration to the same values.
void RFcalParms(void)
{
   char   *Token;
   String sToken;
   int    ch,brd;
   float  m,b;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     m = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     b = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the channel and exit if error
     if (!IsChannelValid(ch,false)) break;
     brd = BoardFromSelectedChannel(ch - 1);
     RFDDarray[brd]->RFCD[(ch-1) & 1].RFpADCchan.m = m;
     RFDDarray[brd]->RFCD[(ch-1) & 1].RFpADCchan.b = b;
     RFDDarray[brd]->RFCD[(ch-1) & 1].RFnADCchan.m = m;
     RFDDarray[brd]->RFCD[(ch-1) & 1].RFnADCchan.b = b;  
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
  
}

// This function saves the RF driver module data to EEPROM. All detected RF modules are saved.
// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveRF2EEPROM(void)
{
  int  brd;
  bool berr = false;
  
  brd = SelectedBoard();
  for(int b=0; b<4; b++)
  {
    if(RFDDarray[b] != NULL)
    {
      SelectBoard(b);
      if (WriteEEPROM(RFDDarray[b], RFDDarray[b]->EEPROMadr, 0, sizeof(RFdriverData)) != 0) berr = true;
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

// Software calibration functions to make minor adjustment to the RF level readback detectors. 
// These routines interate to converge on the desired output. To use this feature you need
// to set the RF level and read its level then calibrate the channel by defining the desired
// readback level.
void RFcalP(char *channel, char *Vpp)
{
  String sToken;
  int ch,brd,ADCraw;
  float vpp,MVpp, m;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  brd = BoardFromSelectedChannel(ch - 1);
  if(vpp < 0.0)
  {
    RFDDarray[brd]->RFCD[(ch-1) & 1].RFpADCchan.m = 32;
    SendACK;
    return;
  }
  SendACK;
  TWIsetByte(RFdrvCMDadd(RFDDarray[brd]->EEPROMadr), brd, TWI_RF_SET_CHAN, ((ch-1) & 1) +1);
  TWIsetFloat(RFdrvCMDadd(RFDDarray[brd]->EEPROMadr), brd, TWI_RF_CALP, vpp);
}

void RFcalN(char *channel, char *Vpp)
{
  String sToken;
  int ch,brd,ADCraw;
  float vpp,MVpp, m;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  brd = BoardFromSelectedChannel(ch - 1);
  if(vpp < 0.0)
  {
    RFDDarray[brd]->RFCD[(ch-1) & 1].RFnADCchan.m = 32;
    SendACK;
    return;
  }
  SendACK;
  TWIsetByte(RFdrvCMDadd(RFDDarray[brd]->EEPROMadr), brd, TWI_RF_SET_CHAN, ((ch-1) & 1) +1);
  TWIsetFloat(RFdrvCMDadd(RFDDarray[brd]->EEPROMadr), brd, TWI_RF_CALN, vpp);
}

void GetRFpwrLimit(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i]->PowerLimit);
}

void SetRFpwrLimit(int channel, int Power)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  if((Power < 1) || (Power > 100)) BADARG;
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i]->PowerLimit = Power;
  if(i == SelectedRFBoard) RFDD->PowerLimit = Power;
}

//
// The following functions support the piece wise linear calibration model.
//

// These functions will generate the calibration table. The user will adjust the 
// drive to set a desired voltage and then enter the value. Up to ten points
// be definded. Enter an empty string to terminate data entry. It is assumed 
// that the channel has been tuned and is properly connected.
// channel is 1 through number of channels
// phase is RF+ or RF-
uint8_t PWLch;

void RFdriveAllowADJ(void)
{
  int   brd;
  float Drive;
  
  // Call the task system to allow all system processing
  control.run();
  // Process any encoder event
  if (ButtonRotated)
  {
    ButtonRotated = false;
    Drive = ((float)encValue)/10.0;
    encValue = 0;
    // Get the RFdiver structure and adjust the drive level 
    if (!IsChannelValid(PWLch,false)) return;
    brd = BoardFromSelectedChannel(PWLch - 1);
    // If Drive value is invalid exit
    Drive += RFDDarray[brd]->RFCD[(PWLch - 1) & 1].DriveLevel;
    if ((Drive < 0) || (Drive > RFDDarray[brd]->RFCD[(PWLch - 1) & 1].MaxDrive)) return;
    RFDDarray[brd]->RFCD[(PWLch - 1) & 1].DriveLevel = Drive;
    if (PWLch - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
  }
}

void genPWLcalTable(char *channel, char *phase)
{
  // Send the user instructions.
  serial->println("This version of the RF driver requires you to");
  serial->println("connect to the modules USB port or to use");
  serial->println("the TWITALK capability. This calibration is");
  serial->println("Performed using the modules CPU and host");
  serial->println("interface.");
}

#endif
