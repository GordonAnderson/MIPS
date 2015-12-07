//
// RFdriver
//
// This file supports the RF driver board on the MIPS system. Two RF driver boards can be installed
// in one MIPS system. Each RF driver can drive 2 high Q heads so a single MIPS system can drive
// 4 high Q heads.
//
// To Dos:
//  1.) Add auto tune
//  2.) Make the PWM 12 bit but this requires editing varient.h in the ardunio source code.
//
// Gordon Anderson
//
#include "RFdriver.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"
#include <DIhandler.h>

#define MinFreq    500000
#define MaxFreq    5000000

extern bool NormalStartup;

#define  PWMFS  4095          // Full scale PWM output value
// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.05         // Strong filter coefficent

//MIPS Threads
Thread RFdriverThread  = Thread();

#define RFDD RFDDarray[SelectedRFBoard]

RFdriverData  RFDDarray[2] = {RFDD_A_Rev_1, RFDD_B_Rev_1};

#ifdef TestMode
int  NumberOfRFChannels = 2; // Defines the number of RF channels supported. Set during intaliztion.
// valid values are 0, 2, or 4
bool RFdriverBoards[2] = {true, false}; // Defines the boards that are present in the system
#else
int  NumberOfRFChannels = 0;
bool RFdriverBoards[2] = {false, false};
#endif
int  Channel = 1;            // User selected channel
int  SelectedRFChan = 0;     // Active channel
int  SelectedRFBoard = 0;    // Active board, 0 or 1 = A or B
float MaxRFVoltage = 0;      // This value is set to the highest RF output voltage
RFchannelData RFCD;          // Holds the selected channel's data

// These arrays hold the monitor values, array indexes are board,channel. These are scanned
// in the loop and filtered. These values are both displayed and also used for control
// and limit testing.
float RFpVpps[2][2];
float RFnVpps[2][2];
float Powers[2][2];
// These are the actively displayed values
float RFpVpp;
float RFnVpp;
float Power;

DIhandler *DIh[2][2];
void (*GateTriggerISRs[2][2])(void) = {RF_A1_ISR, RF_A2_ISR, RF_B1_ISR, RF_B2_ISR};

DialogBoxEntry RFdriverDialogEntriesPage1[] = {
  {" RF channel", 0, 1, D_INT, 1, 2, 1, 21, false, "%2d", &Channel, NULL, SelectChannel},
  {" Freq, Hz", 0, 2, D_INT, 500000, 5000000, 1000, 16, false, "%7d", &RFCD.Freq, NULL, NULL},
  {" Drive %", 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &RFCD.DriveLevel, NULL, NULL},
  {" Vout Vpp", 0, 3, D_OFF, 0, 400, 1, 18, false, "%5.1f", &RFCD.Setpoint, NULL, NULL},
  {" RF+ Vpp", 0, 5, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFpVpp, NULL, NULL},
  {" RF- Vpp", 0, 6, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFnVpp, NULL, NULL},
  {" Power, W", 0, 7, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.1f", &Power, NULL, NULL},
  {" Next page", 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextRFPage, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char *ModeList = "MANUAL,AUTO";
char RFmode[8] = "MANUAL";
char RFgateDI[5] = "NA";
char RFgateTrig[5] = "NA";

DialogBoxEntry RFdriverDialogEntriesPage2[] = {
  {" Drive limit, %", 0, 1, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &RFCD.MaxDrive, NULL, NULL},
  {" Power limit, W", 0, 2, D_FLOAT, 0, 50, 1, 18, false, "%5.1f", &RFCD.MaxPower, NULL, NULL},
  {" Mode"          , 0, 3, D_LIST, 0, 0, 7, 16, false, ModeList, RFmode, NULL, RFmodeChange},
  {" Gate input"    , 0, 4, D_LIST, 0, 0, 5, 18, false, DIlist, RFgateDI, NULL, RFgateChange},
  {" Gate level"    , 0, 5, D_LIST, 0, 0, 5, 18, false, DITlist, RFgateTrig, NULL, RFgateChange},
  {" Save settings", 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFdriverSettings, NULL},
  {" Restore settings", 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFdriverSettings, NULL},
  {" First page"         , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetFirstRFPage, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox RFdriverDialog = {
  {"RF driver parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0, RFdriverDialogEntriesPage1
};

MenuEntry MERFdriverMonitor = {" RF driver module", M_DIALOG, 0, 0, 0, NULL, &RFdriverDialog, NULL, NULL};

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
    RFCD.RFmode = RF_AUTO;
    RFdriverDialogEntriesPage1[2].Type = D_OFF;
    RFdriverDialogEntriesPage1[3].Type = D_FLOAT;
  }
}

void RFgateChange(void)
{
  String trig;

  RFDD.RFgateDI[SelectedRFChan] = RFgateDI[0];
  if (RFDD.RFgateDI[SelectedRFChan] == 'N') RFDD.RFgateDI[SelectedRFChan] = 0;
  trig = RFgateTrig;
  if (trig == "NA") RFDD.RFgateTrig[SelectedRFChan] = -1;
  else if (trig == "BOTH") RFDD.RFgateTrig[SelectedRFChan] = CHANGE;
  else if (trig == "HIGH") RFDD.RFgateTrig[SelectedRFChan] = HIGH;
  else if (trig == "LOW") RFDD.RFgateTrig[SelectedRFChan] = LOW;
  else if (trig == "POS") RFDD.RFgateTrig[SelectedRFChan] = RISING;
  else if (trig == "NEG") RFDD.RFgateTrig[SelectedRFChan] = FALLING;
}

void SetNextRFPage(void)
{
  RFdriverDialog.Entry = RFdriverDialogEntriesPage2;
  RFdriverDialog.Selected = 0;
  RFdriverDialog.State = M_SCROLLING;
  DialogBoxDisplay(&RFdriverDialog);
}

void SetFirstRFPage(void)
{
  RFdriverDialog.Entry = RFdriverDialogEntriesPage1;
  RFdriverDialog.Selected = 0;
  RFdriverDialog.State = M_SCROLLING;
  DialogBoxDisplay(&RFdriverDialog);
}

// Write the current board parameters to the EEPROM on the RFdriver board.
void SaveRFdriverSettings(void)
{
  RFDD.Size = sizeof(RFdriverData);
  if (WriteEEPROM(&RFDD, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
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

  if (ReadEEPROM(&rfdd, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    if (strcmp(rfdd.Name, RFDD.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if(rfdd.Size > sizeof(RFdriverData)) rfdd.Size = sizeof(RFdriverData);
      memcpy(&RFDD, &rfdd, rfdd.Size);
      RFCD = RFDD.RFCD[SelectedRFChan];
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      SelectChannel();
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected
int8_t BoardFromSelectedChannel(int8_t SC)
{
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if (SC <= 1)
  {
    if (RFdriverBoards[0]) return (0);
    if (RFdriverBoards[1]) return (1);
    return (-1);
  }
  //  if selected channel is 2 or 3 then use board 1 and make sure board 0 is present
  if ((SC == 2) || (SC == 3))
  {
    if ((RFdriverBoards[0]) && (RFdriverBoards[1])) return (1);
  }
  return (-1);
}

// Called after the user selects a channel
void SelectChannel(void)
{
  RFCD = RFDD.RFCD[Channel - 1];
  SelectedRFChan = Channel - 1;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
//  Powers[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = RFnVpps[SelectedRFBoard][SelectedRFChan] = 0;
  RFgateDI[0] = RFDD.RFgateDI[SelectedRFChan];
  RFgateDI[1] = 0;
  if (RFgateDI[0] == 0) strcpy(RFgateDI, "NA");
  if (RFDD.RFgateTrig[SelectedRFChan] == -1) strcpy(RFgateTrig, "NA");
  else if (RFDD.RFgateTrig[SelectedRFChan] == CHANGE) strcpy(RFgateTrig, "BOTH");
  else if (RFDD.RFgateTrig[SelectedRFChan] == HIGH) strcpy(RFgateTrig, "HIGH");
  else if (RFDD.RFgateTrig[SelectedRFChan] == LOW) strcpy(RFgateTrig, "LOW");
  else if (RFDD.RFgateTrig[SelectedRFChan] == RISING) strcpy(RFgateTrig, "POS");
  else if (RFDD.RFgateTrig[SelectedRFChan] == FALLING) strcpy(RFgateTrig, "NEG");
  if (RFCD.RFmode == RF_MANUAL) strcpy(RFmode, "MANUAL");
  else strcpy(RFmode, "AUTO");
  RFmodeChange();
  RFgateChange();
  // Update the display
  if (ActiveDialog == &RFdriverDialog) DialogBoxDisplay(&RFdriverDialog);
}

// This function is called at powerup to initiaize the RF driver.
// The board parameter (0 or 1) defines the board select parameter where this card was found.
// Up to two boards are supported. Board A, or 0, is always called first.
// If only one board is installed it can be board 0 or 1.
void RFdriver_init(int8_t Board)
{
  // Flag the board as present
  RFdriverBoards[Board] = true;
  // Create the digital input gate objects
  DIh[Board][0] = new DIhandler;
  DIh[Board][1] = new DIhandler;
  // Set active board to board being inited
  SelectedRFBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  if (NormalStartup)
  {
    RestoreRFdriverSettings(true);
  }
  // Init the clock generator and set the frequencies
  SetRef(20000000);
  CY_Init(RFDD.CLOCKadr);
  SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[0].Freq);
  SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[1].Freq);
  // Setup the PWM outputs and set levels
  analogWriteResolution(12);
  pinMode(RFDD.RFCD[0].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[0].PWMchan, (RFDD.RFCD[0].DriveLevel * PWMFS) / 100);
  pinMode(RFDD.RFCD[1].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[1].PWMchan, (RFDD.RFCD[1].DriveLevel * PWMFS) / 100);
  // Define the initial selected channel as 0 and setup
  Channel = 1;
  SelectChannel();
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

// This function checks the power limits and performs control functions
void RFcontrol(void)
{
  int board, chan;

  // Check the power and reduce the drive level if power is over its limit
  for (board = 0; board < 2; board++)
  {
    for (chan = 0; chan < 2; chan++)
    {
      if (Powers[board][chan] > RFDDarray[board].RFCD[chan].MaxPower) RFDDarray[board].RFCD[chan].DriveLevel -= 0.1;
      if (RFDDarray[board].RFCD[chan].DriveLevel < 0) RFDDarray[board].RFCD[chan].DriveLevel = 0;
      if ((SelectedRFBoard == board) && (SelectedRFChan == chan))
      {
        RFCD.DriveLevel = RFDDarray[board].RFCD[chan].DriveLevel;
      }
      // If we are in auto mode (closed loop control) then adjust drive as needed to maintain Vpp
      if ((RFDDarray[board].RFCD[chan].RFmode == RF_AUTO) && (DIh[board][chan]->activeLevel()))
      {
        if (((RFpVpps[board][chan] + RFnVpps[board][chan]) / 2) > RFDDarray[board].RFCD[chan].Setpoint) RFDDarray[board].RFCD[chan].DriveLevel -= .01;
        else RFDDarray[board].RFCD[chan].DriveLevel += .01;
        if (RFDDarray[board].RFCD[chan].DriveLevel < 0) RFDDarray[board].RFCD[chan].DriveLevel = 0;
        if (RFDDarray[board].RFCD[chan].DriveLevel > RFDDarray[board].RFCD[chan].MaxDrive) RFDDarray[board].RFCD[chan].DriveLevel = RFDDarray[board].RFCD[chan].MaxDrive;
        if ((SelectedRFBoard == board) && (SelectedRFChan == chan))
        {
          RFCD.DriveLevel = RFDDarray[board].RFCD[chan].DriveLevel;
        }
      }
    }
  }
}

// This function is called by the main loop every 100 millisec
void RFdriver_loop(void)
{
  int i;
  uint16_t ADCvals[8];
  float V, I;
  static int LastFreq[2][2] = { -1, -1, -1, -1};
  static  int disIndex = 0;

  MaxRFVoltage = 0;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  RFDD.RFCD[SelectedRFChan] = RFCD;    // This stores any changes back to the selected channels data structure
  // Set the Drive level limit in the UI menu
  RFdriverDialogEntriesPage1[2].Max = RFCD.MaxDrive;
  // Update the clock generator and set the frequencies for
  // all boards that are present in system. Only update if the freq
  // has actually changed
  if (LastFreq[SelectedRFBoard][0] != RFDD.RFCD[0].Freq)
  {
    for (i = 0; i < 5; i++) if (SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[0].Freq) == 0) break;
    LastFreq[SelectedRFBoard][0] = RFDD.RFCD[0].Freq;
  }
  if (LastFreq[SelectedRFBoard][1] != RFDD.RFCD[1].Freq)
  {
    for (i = 0; i < 5; i++) if (SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[1].Freq) == 0) break;
    LastFreq[SelectedRFBoard][1] = RFDD.RFCD[1].Freq;
  }
  // Update the PWM outputs and set levels
  {
    AtomicBlock< Atomic_RestoreState > 	a_Block;
    if (DIh[SelectedRFBoard][0]->activeLevel()) analogWrite(RFDD.RFCD[0].PWMchan, (RFDD.RFCD[0].DriveLevel * PWMFS) / 100);
    else analogWrite(RFDD.RFCD[0].PWMchan, 0);
    if (DIh[SelectedRFBoard][1]->activeLevel()) analogWrite(RFDD.RFCD[1].PWMchan, (RFDD.RFCD[1].DriveLevel * PWMFS) / 100);
    else analogWrite(RFDD.RFCD[1].PWMchan, 0);
  }
  // Check and update the RF gate controls
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    SelectedRFBoard = BoardFromSelectedChannel(i);
    if ((RFDD.RFgateDI[i] != DIh[SelectedRFBoard][i]->di) || (RFDD.RFgateTrig[SelectedRFChan] != DIh[SelectedRFBoard][i]->mode))
    {
      DIh[SelectedRFBoard][i]->detach();
      DIh[SelectedRFBoard][i]->attached(RFDD.RFgateDI[i], RFDD.RFgateTrig[i], GateTriggerISRs[SelectedRFBoard][i]);
    }
  }
  // Read all the voltage monitors and caculate all the RF head power
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    // Select board and read parameters
    SelectedRFBoard = BoardFromSelectedChannel(i);
    SelectBoard(SelectedRFBoard);
    // Read the ADC monitor values for the selected channels.
    if ((i == 0) || (i == 2)) if (AD7998(RFDD.ADCadr, ADCvals) != 0)  // this logic is not correct!
      {
        i++;
        continue;
      }
    if (DIh[SelectedRFBoard][i]->activeLevel())
    {
      if (RFpVpps[SelectedRFBoard][i] == 0) RFpVpps[SelectedRFBoard][i] = Counts2Value(ADCvals[RFDD.RFCD[i].RFpADCchan.Chan], &RFDD.RFCD[i].RFpADCchan);
      if (RFnVpps[SelectedRFBoard][i] == 0) RFnVpps[SelectedRFBoard][i] = Counts2Value(ADCvals[RFDD.RFCD[i].RFnADCchan.Chan], &RFDD.RFCD[i].RFnADCchan);
      // Filter with 1st order difference equation
      RFpVpps[SelectedRFBoard][i] = Filter * Counts2Value(ADCvals[RFDD.RFCD[i].RFpADCchan.Chan], &RFDD.RFCD[i].RFpADCchan) + (1 - Filter) * RFpVpps[SelectedRFBoard][i];
      RFnVpps[SelectedRFBoard][i] = Filter * Counts2Value(ADCvals[RFDD.RFCD[i].RFnADCchan.Chan], &RFDD.RFCD[i].RFnADCchan) + (1 - Filter) * RFnVpps[SelectedRFBoard][i];
      // Limit test
      if (RFpVpps[SelectedRFBoard][i] < 0) RFpVpps[SelectedRFBoard][i] = 0;
      if (RFnVpps[SelectedRFBoard][i] < 0) RFnVpps[SelectedRFBoard][i] = 0;
      if (abs(RFpVpps[SelectedRFBoard][i]) > MaxRFVoltage) MaxRFVoltage = abs(RFpVpps[SelectedRFBoard][i]);
      if (abs(RFnVpps[SelectedRFBoard][i]) > MaxRFVoltage) MaxRFVoltage = abs(RFnVpps[SelectedRFBoard][i]);
      // Calculate RF head power
      V = Counts2Value(ADCvals[RFDD.RFCD[i].DriveVADCchan.Chan], &RFDD.RFCD[i].DriveVADCchan);
      I = Counts2Value(ADCvals[RFDD.RFCD[i].DriveIADCchan.Chan], &RFDD.RFCD[i].DriveIADCchan);
      // Filter with 1st order difference equation
      Powers[SelectedRFBoard][i] = Filter * (V * I) + (1 - Filter) * Powers[SelectedRFBoard][i];
      if (Powers[SelectedRFBoard][i] < 0) Powers[SelectedRFBoard][i] = 0;
    }
  }
  // Reselect the active channel's board
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  // Read the ADC monitor values for the selected channel.
  RFpVpp = RFpVpps[SelectedRFBoard][SelectedRFChan];
  RFnVpp = RFnVpps[SelectedRFBoard][SelectedRFChan];
  Power = Powers[SelectedRFBoard][SelectedRFChan];
  if (ActiveDialog->Entry == RFdriverDialogEntriesPage1)
  {
    if((ActiveDialog->Selected != disIndex+1) || (ActiveDialog->State == M_SCROLLING)) DisplayDialogEntry(&ActiveDialog->w, &ActiveDialog->Entry[disIndex+1], false);
    if(++disIndex >= 6) disIndex = 0;
  }
  RFcontrol();
}

// Gate ISR functions
void RF_A1_ISR(void)
{
  if (DIh[0][0]->activeLevel()) analogWrite(RFDDarray[0].RFCD[0].PWMchan, (RFDDarray[0].RFCD[0].DriveLevel * PWMFS) / 100);
  else analogWrite(RFDDarray[0].RFCD[0].PWMchan, 0);
}
void RF_A2_ISR(void)
{
  if (DIh[0][1]->activeLevel()) analogWrite(RFDDarray[0].RFCD[1].PWMchan, (RFDDarray[0].RFCD[1].DriveLevel * PWMFS) / 100);
  else analogWrite(RFDDarray[0].RFCD[1].PWMchan, 0);
}
void RF_B1_ISR(void)
{
  if (DIh[1][0]->activeLevel()) analogWrite(RFDDarray[1].RFCD[0].PWMchan, (RFDDarray[1].RFCD[0].DriveLevel * PWMFS) / 100);
  else analogWrite(RFDDarray[1].RFCD[0].PWMchan, 0);
}
void RF_B2_ISR(void)
{
  if (DIh[1][1]->activeLevel()) analogWrite(RFDDarray[1].RFCD[1].PWMchan, (RFDDarray[1].RFCD[1].DriveLevel * PWMFS) / 100);
  else analogWrite(RFDDarray[1].RFCD[1].PWMchan, 0);
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
  int   i;

  // If channel is invalid send NAK and exit
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
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[channel - 1].Freq = freq;
  if (channel - 1 == SelectedRFChan) RFCD.Freq = freq;
}

void RFdrive(int channel, float Drive)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  i = BoardFromSelectedChannel(channel - 1);
  // If Drive value is invalid exit
  if ((Drive < 0) || (Drive > RFDDarray[i].RFCD[channel - 1].MaxDrive)) return;
  RFDDarray[i].RFCD[channel - 1].DriveLevel = Drive;
  if (channel - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
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
  if ((Drive < 0) || (Drive > RFDDarray[i].RFCD[channel - 1].MaxDrive))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  RFDDarray[i].RFCD[channel - 1].DriveLevel = Drive;
  if (channel - 1 == SelectedRFChan) RFCD.DriveLevel = Drive;
}

void RFvoltage(int channel, float Voltage)
{
  int   i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel,false)) return;
  // If Drive value is invalid exit
  if ((Voltage < 0) || (Voltage > 400.0)) return;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[channel - 1].Setpoint = Voltage;
  if (channel - 1 == SelectedRFChan) RFCD.Setpoint = Voltage;
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
  if ((Voltage < 0) || (Voltage > 400.0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  i = BoardFromSelectedChannel(channel - 1);
  RFDDarray[i].RFCD[channel - 1].Setpoint = Voltage;
  if (channel - 1 == SelectedRFChan) RFCD.Setpoint = Voltage;
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
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[channel - 1].Freq);
}

void RFvoltageReportP(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFpVpps[i][channel - 1]);
}

void RFvoltageReportN(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFnVpps[i][channel - 1]);
}

void RFdriveReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[channel - 1].DriveLevel);
}

// Reports the voltage setpoint
void RFvoltageReport(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(RFDDarray[i].RFCD[channel - 1].Setpoint);
}

void RFheadPower(int channel)
{
  int i;

  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel - 1);
  if (!SerialMute) serial->println(Powers[i][channel - 1]);
}



