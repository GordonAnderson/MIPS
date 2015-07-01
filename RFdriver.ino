//
// RFdriver
//
// This file supports the RF driver board on the MIPS system. Two RF driver boards can be installed
// in one MIPS system. Each RF driver can drive 2 high Q heads so a single MIPS system can drive
// 4 high Q heads.
//
// To Dos:
//  1.) Add automated level control
//  2.) Add all the serial commands, most in place and tested
//  3.) Add auto tune
//
// Gordon Anderson
//
#include "RFdriver.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

#define MinFreq    500000
#define MaxFreq    5000000

extern bool NormalStartup;

#define  PWMFS  255          // Full scale PWM output value
// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.05         // Strong filter coefficent

//MIPS Threads
Thread RFdriverThread  = Thread();

#define RFDD RFDDarray[SelectedRFBoard]

RFdriverData  RFDDarray[2] = {RFDD_A_Rev_1,RFDD_B_Rev_1};

#ifdef TestMode
int  NumberOfRFChannels = 2; // Defines the number of RF channels supported. Set during intaliztion.
                             // valid values are 0, 2, or 4
bool RFdriverBoards[2] = {true,false};  // Defines the boards that are present in the system
#else
int  NumberOfRFChannels = 0; 
bool RFdriverBoards[2] = {false,false}; 
#endif
int  Channel = 1;            // User selected channel
int  SelectedRFChan=0;       // Active channel
int  SelectedRFBoard=0;      // Active board, 0 or 1 = A or B
float MaxRFVoltage=0;        // This value is set to the highest RF output voltage
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

DialogBoxEntry RFdriverDialogEntriesPage1[] = {
  {" RF channel", 0, 1, D_INT, 1, 2, 1, 21, false, "%2d", &Channel, NULL, SelectChannel},
  {" Freq, Hz", 0, 2, D_INT, 500000, 5000000, 1000, 16, false, "%7d", &RFCD.Freq, NULL, NULL},
  {" Drive %", 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &RFCD.DriveLevel, NULL, NULL},
  {" RF+ Vpp", 0, 5, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFpVpp, NULL, NULL},
  {" RF- Vpp", 0, 6, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFnVpp, NULL, NULL},
  {" Power, W", 0, 7, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.1f", &Power, NULL, NULL},
  {" Next page", 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetNextRFPage, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry RFdriverDialogEntriesPage2[] = {
  {" Drive limit, %", 0, 1, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &RFCD.MaxDrive, NULL, NULL},
  {" Power limit, W", 0, 2, D_FLOAT, 0, 50, 1, 18, false, "%5.1f", &RFCD.MaxPower, NULL, NULL},
  {" Save settings", 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFdriverSettings, NULL},
  {" Restore settings", 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFdriverSettings, NULL},
  {" First page"         , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SetFirstRFPage, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox RFdriverDialog = {
  {
    "RF driver parameters",
    ILI9340_BLACK,
    ILI9340_WHITE,
    2,
    0, 0,
    300, 220,
    B_DOUBLE,
    12
  },
  M_SCROLLING,
  0,
  RFdriverDialogEntriesPage1
};

MenuEntry MERFdriverMonitor = {" RF driver module", M_DIALOG,0,0,0,NULL,&RFdriverDialog,NULL,NULL};

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
  if(WriteEEPROM(&RFDD, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Unable to Save!",2000);
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
  
  if(ReadEEPROM(&rfdd, RFDD.EEPROMadr, 0, sizeof(RFdriverData)) == 0)
  {
     if(strcmp(rfdd.Name,RFDD.Name) == 0)
     {
       // Here if the name matches so copy the data to the operating data structure
       memcpy(&RFDD, &rfdd, sizeof(RFdriverData));
       RFCD = RFDD.RFCD[SelectedRFChan];
       if(!NoDisplay) DisplayMessage("Parameters Restored!",2000);
     } 
     else if(!NoDisplay) DisplayMessage("Corrupted EEPROM data!",2000);
  }
  else if(!NoDisplay) DisplayMessage("Unable to Restore!",2000);
}

// This function uses the selected channel to determine the boad number, 0 or 1.
// Returns -1 if error condition is detected
int8_t BoardFromSelectedChannel(int8_t SC)
{
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if(SC <= 1)
  {
    if(RFdriverBoards[0]) return(0);
    if(RFdriverBoards[1]) return(1);
    return(-1);
  }
  //  if selected channel is 2 or 3 then use board 1 and make sure board 0 is present
  if((SC == 2) || (SC ==3))
  {
    if((RFdriverBoards[0]) && (RFdriverBoards[1])) return(1);
  }
  return(-1);
}

// Called after the user selects a channel
void SelectChannel(void)
{
  RFCD = RFDD.RFCD[Channel - 1];
  SelectedRFChan = Channel - 1;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  Powers[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = 0;
  // Update the display
  DialogBoxDisplay(&RFdriverDialog);
}

// This function is called at powerup to initiaize the RF driver.
// The board parameter (0 or 1) defines the board select parameter where this card was found.
// Up to two boards are supported. Board A, or 0, is always called first.
// If only one board is installed it can be board 0 or 1.
void RFdriver_init(int8_t Board)
{
  // Flag the board as present
  RFdriverBoards[Board] = true;
  // Set active board to board being inited
  SelectedRFBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  if(NormalStartup)
  {
    RestoreRFdriverSettings(true);
  }
  // Init the clock generator and set the frequencies
  SetRef(20000000);
  CY_Init(RFDD.CLOCKadr);
  SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[0].Freq);
  SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[1].Freq);
  // Setup the PWM outputs and set levels
  pinMode(RFDD.RFCD[0].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[0].PWMchan, (RFDD.RFCD[0].DriveLevel * PWMFS) / 100);
  pinMode(RFDD.RFCD[1].PWMchan, OUTPUT);
  analogWrite(RFDD.RFCD[1].PWMchan, (RFDD.RFCD[1].DriveLevel * PWMFS) / 100);
  // Define the initial selected channel as 0 and setup
  SelectedRFChan = 0;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  RFCD = RFDD.RFCD[SelectedRFChan];
  Powers[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = RFpVpps[SelectedRFBoard][SelectedRFChan] = 0;
  // Setup the menu if this is the very first call to this init function
  if(NumberOfRFChannels == 0)
  {
    AddMainMenuEntry(&MERFdriverMonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&RFdriverDialog);
    // Configure Threads
    RFdriverThread.onRun(RFdriver_loop);
    RFdriverThread.setInterval(100);
    // Add threads to the controller
    controll.add(&RFdriverThread);
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
  for(board = 0; board < 2; board++)
  {
    for(chan = 0; chan < 2; chan++)
    {
      if(Powers[board][chan] > RFDDarray[board].RFCD[chan].MaxPower) RFDDarray[board].RFCD[chan].DriveLevel -= 0.1;
      if(RFDDarray[board].RFCD[chan].DriveLevel < 0) RFDDarray[board].RFCD[chan].DriveLevel = 0;
      if((SelectedRFBoard == board) && (SelectedRFChan == chan))
      {
        RFCD.DriveLevel = RFDDarray[board].RFCD[chan].DriveLevel;
      }
    }
  }
}

// This function is called by the main loop every 100 millisec
void RFdriver_loop(void)
{
  int i;
  uint16_t ADCvals[8];
  float V,I;
  static int LastFreq[2][2] = {-1,-1,-1,-1};

  MaxRFVoltage = 0;
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  RFDD.RFCD[SelectedRFChan] = RFCD;    // This stores any changes back to the selected channels data structure
  // Set the Drive level limit in the UI menu
  RFdriverDialogEntriesPage1[2].Max = RFCD.MaxDrive;
  // Update the clock generator and set the frequencies for 
  // all boards that are present in system. Only update if the freq
  // has actually changed
  if(LastFreq[SelectedRFBoard][0] != RFDD.RFCD[0].Freq)
  {
    for(i=0;i<5;i++) if(SetPLL2freq(RFDD.CLOCKadr, RFDD.RFCD[0].Freq)==0) break;
    LastFreq[SelectedRFBoard][0] = RFDD.RFCD[0].Freq;
  }
  if(LastFreq[SelectedRFBoard][1] != RFDD.RFCD[1].Freq)
  {
    for(i=0;i<5;i++) if(SetPLL3freq(RFDD.CLOCKadr, RFDD.RFCD[1].Freq)==0) break;
    LastFreq[SelectedRFBoard][1] = RFDD.RFCD[1].Freq;
  }
  // Update the PWM outputs and set levels
  analogWrite(RFDD.RFCD[0].PWMchan, (RFDD.RFCD[0].DriveLevel * PWMFS) / 100);
  analogWrite(RFDD.RFCD[1].PWMchan, (RFDD.RFCD[1].DriveLevel * PWMFS) / 100);
  // Read all the voltage monitors and caculate all the head powers
  for(i=0; i<NumberOfRFChannels;i += 2)
  {
    // Select board and read parameters
    SelectedRFBoard = BoardFromSelectedChannel(i);
    SelectBoard(SelectedRFBoard);
    // Read the ADC monitor values for the selected channel.
    if (AD7998(RFDD.ADCadr, ADCvals) == 0)
    {
      // channel 1 first
      if(RFpVpps[SelectedRFBoard][0] == 0) RFpVpps[SelectedRFBoard][0] = Counts2Value(ADCvals[RFDD.RFCD[0].RFpADCchan.Chan],&RFDD.RFCD[0].RFpADCchan);
      if(RFnVpps[SelectedRFBoard][0] == 0) RFnVpps[SelectedRFBoard][0] = Counts2Value(ADCvals[RFDD.RFCD[0].RFnADCchan.Chan],&RFDD.RFCD[0].RFnADCchan);
      // Filter with 1st order difference equation
      RFpVpps[SelectedRFBoard][0] = Filter * Counts2Value(ADCvals[RFDD.RFCD[0].RFpADCchan.Chan],&RFDD.RFCD[0].RFpADCchan) + (1-Filter) * RFpVpps[SelectedRFBoard][0];
      RFnVpps[SelectedRFBoard][0] = Filter * Counts2Value(ADCvals[RFDD.RFCD[0].RFnADCchan.Chan],&RFDD.RFCD[0].RFnADCchan) + (1-Filter) * RFnVpps[SelectedRFBoard][0];
      // Limit test
      if(RFpVpps[SelectedRFBoard][0] < 0) RFpVpps[SelectedRFBoard][0] = 0;
      if(RFnVpps[SelectedRFBoard][0] < 0) RFnVpps[SelectedRFBoard][0] = 0;
      if(abs(RFpVpps[SelectedRFBoard][0]) > MaxRFVoltage) MaxRFVoltage = abs(RFpVpps[SelectedRFBoard][0]);
      if(abs(RFnVpps[SelectedRFBoard][0]) > MaxRFVoltage) MaxRFVoltage = abs(RFnVpps[SelectedRFBoard][0]);
      // Calculate RF head power
      V = Counts2Value(ADCvals[RFDD.RFCD[0].DriveVADCchan.Chan],&RFDD.RFCD[0].DriveVADCchan);
      I = Counts2Value(ADCvals[RFDD.RFCD[0].DriveIADCchan.Chan],&RFDD.RFCD[0].DriveIADCchan);
      // Filter with 1st order difference equation
      Powers[SelectedRFBoard][0] = Filter * (V * I) + (1-Filter) * Powers[SelectedRFBoard][0];
      if(Powers[SelectedRFBoard][0] < 0) Powers[SelectedRFBoard][0] = 0;
      // now channel 2
      if(RFpVpps[SelectedRFBoard][1] == 0) RFpVpps[SelectedRFBoard][1] = Counts2Value(ADCvals[RFDD.RFCD[1].RFpADCchan.Chan],&RFDD.RFCD[1].RFpADCchan);
      if(RFnVpps[SelectedRFBoard][1] == 0) RFnVpps[SelectedRFBoard][1] = Counts2Value(ADCvals[RFDD.RFCD[1].RFnADCchan.Chan],&RFDD.RFCD[1].RFnADCchan);
      // Filter with 1st order difference equation
      RFpVpps[SelectedRFBoard][1] = Filter * Counts2Value(ADCvals[RFDD.RFCD[1].RFpADCchan.Chan],&RFDD.RFCD[1].RFpADCchan) + (1-Filter) * RFpVpps[SelectedRFBoard][1];
      RFnVpps[SelectedRFBoard][1] = Filter * Counts2Value(ADCvals[RFDD.RFCD[1].RFnADCchan.Chan],&RFDD.RFCD[1].RFnADCchan) + (1-Filter) * RFnVpps[SelectedRFBoard][1];
      // Limit test
      if(RFpVpps[SelectedRFBoard][1] < 0) RFpVpps[SelectedRFBoard][1] = 0;
      if(RFnVpps[SelectedRFBoard][1] < 0) RFnVpps[SelectedRFBoard][1] = 0;
      if(abs(RFpVpps[SelectedRFBoard][1]) > MaxRFVoltage) MaxRFVoltage = abs(RFpVpps[SelectedRFBoard][1]);
      if(abs(RFnVpps[SelectedRFBoard][1]) > MaxRFVoltage) MaxRFVoltage = abs(RFnVpps[SelectedRFBoard][1]);
      // Calculate RF head power
      V = Counts2Value(ADCvals[RFDD.RFCD[1].DriveVADCchan.Chan],&RFDD.RFCD[1].DriveVADCchan);
      I = Counts2Value(ADCvals[RFDD.RFCD[1].DriveIADCchan.Chan],&RFDD.RFCD[1].DriveIADCchan);
      // Filter with 1st order difference equation
      Powers[SelectedRFBoard][1] = Filter * (V * I) + (1-Filter) * Powers[SelectedRFBoard][1];
      if(Powers[SelectedRFBoard][1] < 0) Powers[SelectedRFBoard][1] = 0;
    }
  }
  // Reselect the active channel's board
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  // Read the ADC monitor values for the selected channel.
  RFpVpp = RFpVpps[SelectedRFBoard][SelectedRFChan];
  RFnVpp = RFnVpps[SelectedRFBoard][SelectedRFChan];
  Power = Powers[SelectedRFBoard][SelectedRFChan];
  if(ActiveDialog == &RFdriverDialog)
  {
     if(RFdriverDialog.Entry == RFdriverDialogEntriesPage1) UpdateNoEditDialogEntries(&RFdriverDialog);
  }
  RFcontrol();
}

//
// This section contains all the serial command processing routines.
//

// Reports the number of RF output channels avalible.
void RFnumber(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfRFChannels);
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsChannelValid(int channel)
{
  if((Channel >= 1) && (Channel <= NumberOfRFChannels)) return true;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Set a channels freqency
void RFfreq(int channel, int freq)
{
  int   i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  // If freq value is invalid send NAK and exit
  if((freq < MinFreq) || (freq > MaxFreq))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the frequency
  SendACK;
  i = BoardFromSelectedChannel(channel-1);
  RFDDarray[i].RFCD[channel-1].Freq = freq;
  if(channel-1 == SelectedRFChan)
  {
    RFCD.Freq = freq;
    if(ActiveDialog == &RFdriverDialog)
    {
      DisplayAllDialogEntries(&RFdriverDialog);
      RFdriverDialog.State = M_SCROLLING;
    }
  }
}

void RFdrive(char *Chan, char *Val)
{
  int   channel,i;
  float Drive;
  
  sscanf(Chan,"%d", &channel);
  sscanf(Val,"%f", &Drive);
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
    // If Drive value is invalid send NAK and exit
  if((Drive < 0) || (Drive > 100.0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If here ACK the command and set the drive level
  SendACK;
  i = BoardFromSelectedChannel(channel-1);
  RFDDarray[i].RFCD[channel-1].DriveLevel = Drive;
  if(channel-1 == SelectedRFChan)
  {
    RFCD.DriveLevel = Drive;
    if(ActiveDialog == &RFdriverDialog)
    {
      DisplayAllDialogEntries(&RFdriverDialog);
      RFdriverDialog.State = M_SCROLLING;
    }
  }  
}

// Report a channels frequency
void RFfreqReport(int channel)
{
  int i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  // Report the channels frequency
  SendACKonly;
  i = BoardFromSelectedChannel(channel-1);
  if(!SerialMute) serial->println(RFDDarray[i].RFCD[channel-1].Freq);
}

void RFvoltageReportP(int channel)
{
  int i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel-1);
  if(!SerialMute) serial->println(RFpVpps[i][channel-1]);
}

void RFvoltageReportN(int channel)
{
  int i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel-1);
  if(!SerialMute) serial->println(RFnVpps[i][channel-1]);
}

void RFdriveReport(int channel)
{
  int i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel-1);
  if(!SerialMute) serial->println(RFDDarray[i].RFCD[channel-1].DriveLevel);
}

void RFheadPower(int channel)
{
  int i;
  
  // If channel is invalid send NAK and exit
  if(!IsChannelValid(channel)) return;
  SendACKonly;
  i = BoardFromSelectedChannel(channel-1);
  if(!SerialMute) serial->println(Powers[i][channel-1]);
}



