//
// ARB
//
// This file supports the ARB module for the MIPS system. The MIPS controller
// supports up to two ARB modules.
//
// July 31, 2016
//  Added a second mode traditional ARM mode of operation. With this mode you can
//  define channel output level / pulses. This mode supports rev 2.0 of the ARB
//  and adds the aux output channel as well as offset output.
//
// October 22, 2016
//  - Add support for two ARB modules (done)
//  - Update the host commands to support module number (done)
//  - Add host commands
//    - Direction (done)
//    - Arb waveform (done)
//  - Add TWI aquire and release around all functions (done)
//  - Add external direction input (done, needs testing)
//  - Make sure sync supports both channels (done needs testing)
//  - ARB module firmware updated
//    - Add save and restore (done)
//    - Add calibration (done)
//    - Fix enable bugwavefor
//    - Add new functions
//      * Sync enable / disable (done)
//  - Add compression logic
//    - This will require external clock logic, add flag to indicate external clock.
//      Need to also send command to ARB module as to the frequency so it can mode
//      change as needed. External clock is generated by MIPS controller and provided
//      to ARB module.
//
// Gordon Anderson
//
#include "ARB.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

//MIPS Threads
Thread ARBthread  = Thread();

MIPStimer *ARBclock;

#define ARB ARBarray[SelectedARBboard]

ARBdata  ARBarray[2] = {ARB_Rev_1, ARB_Rev_1};
ARBdata  arb;   // This is the display / UI data structure

DIhandler *ARBsyncIN[2];
DIhandler *DIdirARB[2];
void (*ARBdirISRs[2])(void) = {ARB_1_DIR_ISR, ARB_2_DIR_ISR};

int   ARBmodule            = 1;
int   NumberOfARBchannels  = 0;
bool  ARBboards[2]         = {false, false};
int   SelectedARBboard     = 0;    // Active board, 0 or 1 = A or B
bool  ARBwaveformUpdate[2] = {true,true};
int   CurrentModule        = -1;

// Array used to edit the ARB waveform
int ARBwaveform[32];

// Variables used for ARB channel editing
int   ARBchan      = 1;
float ChanLevel    = 0;
float ChanLevelAll = 0;
int   startI       = 1;
int   stopI        = 10;

char *ARBwfrmList = "SIN,RAMP,TRI,PULSE,ARB";
char WFT[8] = "SIN";

char *ARBmodeList = "TWAVE,ARB";

extern DialogBoxEntry ARBentriesPage2[];
extern DialogBoxEntry ARBwaveformEdit[];
extern DialogBoxEntry ARBentriesRange[];

// TWAVE mode main menu
DialogBoxEntry ARBentriesPage1[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &ARBmodule, NULL, NULL},
  {" Mode"               , 0, 2, D_LIST,  0, 0  , 7, 16, false, ARBmodeList, arb.Mode, NULL, ModeSelected},
  {" Enable"             , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &arb.Enable, NULL, NULL},
  {" Frequency"          , 0, 4, D_INT  , 100, 40000, 100, 16, false, "%7d", &arb.Frequency, NULL, NULL},
  {" Amplitude, Vp-p"    , 0, 5, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &arb.Voltage, NULL, NULL},
  {" Waveform"           , 0, 6, D_LIST,  0, 0  , 7, 16, false, ARBwfrmList, WFT, NULL, NULL},
  {" Dir"                , 0, 7, D_FWDREV, 0, 1  , 1, 20, false, NULL, &arb.Direction, NULL, NULL},
  {" ARB waveform"       , 0, 8, D_PAGE,  0, 0  , 0, 0,  false, NULL, ARBwaveformEdit, Waveform2EditBuffer, NULL},
  {" Next page"          , 0, 9, D_PAGE,  0, 0  , 0, 0,  false, NULL, ARBentriesPage2, NULL, NULL},
  {" Return to main menu", 0,10, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

// ARB mode main menu
DialogBoxEntry ARBentriesPage1arb[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &ARBmodule, NULL, NULL},
  {" Mode"               , 0, 2, D_LIST,  0, 0  , 7, 16, false, ARBmodeList, arb.Mode, NULL, ModeSelected},
  {" Enable"             , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &arb.Enable, NULL, NULL},
  {" Frequency"          , 0, 4, D_INT  , 100, 1000000, 1000, 16, false, "%7d", &arb.Frequency, NULL, NULL},
  {" Amplitude, Vp-p"    , 0, 5, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &arb.Voltage, NULL, NULL},
  {" Buf length"         , 0, 6, D_INT  , 100, 8000, 10, 17, false, "%6d", &arb.BufferLength, NULL, NULL},
  {" Num buf"            , 0, 7, D_INT  , 0, 1000000, 1, 17, false, "%6d", &arb.NumBuffers, NULL, NULL},
  {" Set all channels"   , 0, 8, D_FLOAT, -100, 100, .1, 17, false, "%6.1f", &ChanLevelAll, NULL, SetChans},
  {" Set chan range"     , 0, 9, D_PAGE , 0, 0  , 0, 0,  false, NULL, ARBentriesRange, NULL, NULL},
  {" Next page"          , 0,10, D_PAGE,  0, 0  , 0, 0,  false, NULL, ARBentriesPage2, NULL, NULL},
  {" Return to main menu", 0,11, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry ARBentriesRange[] = {
  {" Set channel"        , 0, 1, D_INT   , 1, 8, 1, 21, false, "%2d", &ARBchan, NULL, NULL},
  {" Set start index"    , 0, 2, D_INT   , 1, 8000, 1, 17, false, "%6d", &startI, NULL, NULL},
  {" Set stop index"     , 0, 3, D_INT   , 1, 8000, 1, 17, false, "%6d", &stopI, NULL, NULL},
  {" Set chan level"     , 0, 4, D_FLOAT , -100, 100, .1, 17, false, "%6.1f", &ChanLevel, NULL, NULL},
  {" Set chan range"     , 0, 5, D_FUNCTION , 0, 0  , 0, 0,  false, NULL, NULL, SetChannelRange, NULL},
  {" First page"         , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, ARBentriesPage1arb, NULL, NULL},
  {NULL},
};

DialogBoxEntry ARBentriesPage2[] = {
  {" Aux voltage"        , 0, 1, D_FLOAT, -50, 50, 0.1, 18, false, "%5.1f", &arb.Aux, NULL, NULL},
  {" Offset, Volts"      , 0, 2, D_FLOAT, -50, 50, 0.1, 18, false, "%5.1f", &arb.Offset, NULL, NULL},
  {" Sync input"         , 0, 3, D_DI     , 0, 0, 2, 21, false, DIlist, &arb.ARBsyncIn, NULL, NULL},
  {" Sync level"         , 0, 4, D_DILEVEL, 0, 0, 4, 19, false, DILlist, &arb.ARBsyncLevel, NULL, NULL},
  {" Dir input"          , 0, 5, D_DI     , 0, 0, 2, 21, false, NULL, &arb.ARBdirDI, NULL, NULL},
  {" Dir level"          , 0, 6, D_DILEVEL, 0, 0, 4, 19, false, NULL, &arb.ARBdirLevel, NULL, NULL},
  {" Compressor"         , 0, 7, D_OFF    , 0, 0, 0, 0, false, NULL, &ARBCompressorDialog, NULL,NULL},
  {" Save settings"      , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveARBsettings, NULL},
  {" Restore settings"   , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorARBsettings, NULL},
  {" First page"         , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, ARBentriesPage1, NULL, NULL},
  {NULL},
};

DialogBoxEntry ARBwaveformEdit[] = {
  {"Index     values (0-255)", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"00-03 ",  1, 2, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[0], NULL, NULL},
  {" "     , 11, 2, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[1], NULL, NULL},
  {" "     , 15, 2, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[2], NULL, NULL},
  {" "     , 19, 2, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[3], NULL, NULL},

  {"04-07 ",  1, 3, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[4], NULL, NULL},
  {" "     , 11, 3, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[5], NULL, NULL},
  {" "     , 15, 3, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[6], NULL, NULL},
  {" "     , 19, 3, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[7], NULL, NULL},

  {"08-11 ",  1, 4, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[8], NULL, NULL},
  {" "     , 11, 4, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[9], NULL, NULL},
  {" "     , 15, 4, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[10], NULL, NULL},
  {" "     , 19, 4, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[11], NULL, NULL},

  {"12-15 ",  1, 5, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[12], NULL, NULL},
  {" "     , 11, 5, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[13], NULL, NULL},
  {" "     , 15, 5, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[14], NULL, NULL},
  {" "     , 19, 5, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[15], NULL, NULL},

  {"16-19 ",  1, 6, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[16], NULL, NULL},
  {" "     , 11, 6, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[17], NULL, NULL},
  {" "     , 15, 6, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[18], NULL, NULL},
  {" "     , 19, 6, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[19], NULL, NULL},

  {"20-23 ",  1, 7, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[20], NULL, NULL},
  {" "     , 11, 7, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[21], NULL, NULL},
  {" "     , 15, 7, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[22], NULL, NULL},
  {" "     , 19, 7, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[23], NULL, NULL},

  {"24-27 ",  1, 8, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[24], NULL, NULL},
  {" "     , 11, 8, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[25], NULL, NULL},
  {" "     , 15, 8, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[26], NULL, NULL},
  {" "     , 19, 8, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[27], NULL, NULL},

  {"28-31 ",  1, 9, D_INT, -100, 100, 1, 7, false, "%3d", &ARBwaveform[28], NULL, NULL},
  {" "     , 11, 9, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[29], NULL, NULL},
  {" "     , 15, 9, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[30], NULL, NULL},
  {" "     , 19, 9, D_INT, -100, 100, 1, 1, false, "%3d", &ARBwaveform[31], NULL, NULL},

  {" Return to ARB menu", 0, 11, D_PAGE, 0, 0, 0, 0, false, NULL, ARBentriesPage1, EditBuffer2Waveform, NULL},
  {NULL},
};

DialogBox ARBdialog = {
  {
    "ARB control parameters",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0,false, ARBentriesPage1
};

MenuEntry MEARBmodule = {" ARB module", M_DIALOG, 0, 0, 0, NULL, &ARBdialog, NULL, NULL};


// ISRs
void ARB_1_DIR_ISR(void)
{
  if (DIdirARB[0]->activeLevel()) ARBarray[0].Direction = true;
  else ARBarray[0].Direction = false;
  if (0 == SelectedARBboard)
  {
    if (DIdirARB[0]->activeLevel()) arb.Direction = true;
    else arb.Direction = false;
  }
  // Need to make ARB loop run, put at top of queue
  ARBthread.setNextRunTime(millis());
}
void ARB_2_DIR_ISR(void)
{
  if (DIdirARB[1]->activeLevel()) ARBarray[1].Direction = true;
  else ARBarray[1].Direction = false;
  if (1 == SelectedARBboard)
  {
    if (DIdirARB[1]->activeLevel()) arb.Direction = true;
    else arb.Direction = false;
  }
  // Need to make ARB loop run, put at top of queue
  ARBthread.setNextRunTime(millis());
}
//

// This function sends the mode command to the ARB module
// defined by board
void SetMode(int board, char *mode)
{
  if(strcmp(mode,"TWAVE") == 0) SetByte(board, TWI_SET_MODE, 0);
  else SetByte(board, TWI_SET_MODE, 1);
}

// The function setups the dialogbox for the selected mode.
void SetModeMenus(bool paint = true)
{
  DialogBoxEntry *de;

  de = GetDialogEntries(ARBentriesPage2, "First page");
  if(strcmp(arb.Mode,"TWAVE") == 0)
  {
    // Define main menu
    ARBdialog.Entry = ARBentriesPage1;
    // Set the return page menu
    de->Value = (void *)ARBentriesPage1;
    ARBdialog.State = M_SCROLLING;
  }
  else
  {
    // Define main menu
    ARBdialog.Entry = ARBentriesPage1arb;
    // Set the return page menu
    de->Value = (void *)ARBentriesPage1arb;
    ARBdialog.State = M_SCROLLING;
  }
  // Repaint main menu
  if(paint) DialogBoxDisplay(&ARBdialog);
}

void ModeSelected(void)
{
  SetModeMenus(true);
}

void SetChans(void)
{
  SetFloat(SelectedARBboard, TWI_SET_SET_BUFFER, ChanLevelAll);
}

// Sets channel range in ARB mode
void SetChannelRange(void)
{
  SetChannelRangeMessage(SelectedARBboard,ARBchan,startI,stopI,ChanLevel);
}

void SetChannelRangeMessage(int board, int channel, int strtI, int stpI, float val)
{
  uint8_t *b;

  SelectBoard(board);
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_SET_CHN_RNG);

  channel--;
  b = (uint8_t *)&channel;
  Wire.write(b[0]);

  b = (uint8_t *)&strtI;
  Wire.write(b[0]);
  Wire.write(b[1]);

  b = (uint8_t *)&stpI;
  Wire.write(b[0]);
  Wire.write(b[1]);

  b = (uint8_t *)&val;
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();
  ReleaseTWI();
}

void SetBufferLength(int board, int BufferLength)
{
  uint8_t *b;

  SelectBoard(board);
  b = (uint8_t *)&BufferLength;
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_BUFFER_LEN);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.endTransmission();
  ReleaseTWI();
}

void SetNumBuffers(int board, int NumBuffers)
{
  uint8_t *b;

  SelectBoard(board);
  b = (uint8_t *)&NumBuffers;
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_NUM_BUFFER);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.endTransmission();  
  ReleaseTWI();
}

// Accepts the waveform type and returns a descriptive string
String GetWaveformString(WaveFormTypes wft)
{
  switch (wft)
  {
    case ARB_SIN:
      return ("SIN");
    case ARB_RAMP:
      return ("RAMP");
    case ARB_TRIANGLE:
      return ("TRI");
    case ARB_PULSE:
      return ("PULSE");
    case ARB_ARB:
      return ("ARB");
    default:
      return ("");
  }
}

// Accepts a waveform type descriptive string and retuns the type
WaveFormTypes GetWaveformType(String WaveformString)
{
  if (WaveformString == String("SIN")) return (ARB_SIN);
  else if (WaveformString == String("RAMP")) return (ARB_RAMP);
  else if (WaveformString == String("TRI")) return (ARB_TRIANGLE);
  else if (WaveformString == String("PULSE")) return (ARB_PULSE);
  else if (WaveformString == String("ARB")) return (ARB_ARB);
}

// Copies the edit buffer waveform to the modules storage buffer.
// This function also sends the ARB vector to the ARB module.
void EditBuffer2Waveform(void)
{
  int i;

  for (i = 0; i < arb.PPP; i++) arb.WaveForm[i] = ARBwaveform[i];
  // Set flag to cause ARB waveform update in the processing loop.
  ARBwaveformUpdate[SelectedARBboard] = true;
}

void Waveform2EditBuffer(void)
{
  int i;

  for (i = 0; i < arb.PPP; i++) ARBwaveform[i] = arb.WaveForm[i];
}

// Update the dialog box and display
void SelectARBmodule(bool paint = true)
{
  String wft_string;

  wft_string = GetWaveformString(ARB.wft);
  strcpy(WFT, wft_string.c_str());
  if(ARBmodule == 1)
  {
    if(ARBboards[0]) SelectedARBboard = 0;
    else SelectedARBboard = 1;
  }
  if(ARBmodule == 2) SelectedARBboard = 1;
  SelectBoard(SelectedARBboard);
  arb = ARB;
  SetModeMenus(paint);
//  DialogBoxDisplay(&ARBdialog);
}

void SetBool(int board, int cmd, bool flag)
{
  SelectBoard(board);
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(cmd);
  Wire.write(flag);
  Wire.endTransmission();
  ReleaseTWI();  
}

void SetByte(int board, int cmd, byte bval)
{
  SelectBoard(board);
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(cmd);
  Wire.write(bval);
  Wire.endTransmission();
  ReleaseTWI();  
}

void SetFloat(int board, int cmd, float fval)
{
  SelectBoard(board);
  AcquireTWI();
  uint8_t *b = (uint8_t *)&fval;
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();
  ReleaseTWI();  
}

void SetARBcommonClock(int freq)
{
  int clkdiv;
  int actualF;
  
  ARBclock->stop();
  if(strcmp(ARBarray[0].Mode,"TWAVE") == 0)
  {
    clkdiv = VARIANT_MCK / (2 * ppp * freq);
    actualF = VARIANT_MCK / (2 * ppp * clkdiv);
    ARBclock->setFrequency(actualF * ppp);
  }
  else
  {
    clkdiv = VARIANT_MCK / (2 * freq);
    actualF = VARIANT_MCK / (2 * clkdiv);
    ARBclock->setFrequency(actualF);
  }
  ARBclock->setTIOAeffect(clkdiv - 2, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
  ARBclock->start(-1, 0, true);
}

void SetFrequency(int board, int freq)
{
  uint8_t *b;

  SelectBoard(board);
  b = (uint8_t *)&freq;
  // Send the frequency to the ARB module
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_FREQ);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.endTransmission();
  ReleaseTWI();
  // Set common clock freqquency
  SetARBcommonClock(freq);
}

void SetAmplitude(int board, float Voltage)
{
  uint8_t *b;
  
  SelectBoard(board);
  // Amplitude is 0 to 100Vp-p for 0 to 4095 ref level on the ARB
  int i = Voltage * 32.8;
  if (i > 4095) i = 4095;
  if (i < 0) i = 0;
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_REF);
  Wire.write(i & 0xFF);
  Wire.write((i >> 8) & 0xFF);
  Wire.endTransmission();
  ReleaseTWI();

  SetFloat(board, TWI_SET_RANGE, Voltage);
}

// Here after user selects a waveform type, update the data structure.
void SetWaveform(int board, WaveFormTypes wft)
{
  AcquireTWI();
  SelectBoard(board);
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_WAVEFORM);
  switch (wft)
  {
    case ARB_SIN:
      Wire.write(TWI_WAVEFORM_SIN);
      break;
    case ARB_RAMP:
      Wire.write(TWI_WAVEFORM_RAMP);
      break;
    case ARB_TRIANGLE:
      Wire.write(TWI_WAVEFORM_TRI);
      break;
    case ARB_PULSE:
      Wire.write(TWI_WAVEFORM_PULSE);
      break;
    case ARB_ARB:
      Wire.write(TWI_WAVEFORM_ARB);
      break;
    default:
      Wire.write(TWI_WAVEFORM_SIN);
      break;
  }
  Wire.endTransmission();
  ReleaseTWI();
}

void SetARBwaveform(int board)
{
  int i;
  
  SelectBoard(board);
  // Send to the ARB module
  AcquireTWI();
  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_VECTOR);
  Wire.write(0);
  Wire.write(16);
  for (i = 0; i < 16; i++) Wire.write(ARBarray[board].WaveForm[i]);
  Wire.endTransmission(); 

  Wire.beginTransmission(ARBarray[board].ARBadr);
  Wire.write(TWI_SET_VECTOR);
  Wire.write(16);
  Wire.write(16);
  for (i = 0; i < 16; i++) Wire.write(ARBarray[board].WaveForm[i+16]);
  Wire.endTransmission(); 
  ReleaseTWI();
}

// Write the current board parameters to the EEPROM on the ARB board.
void SaveARBsettings(void)
{
  SelectBoard(SelectedARBboard);
  arb.Size = sizeof(ARBdata);
  if (WriteEEPROM(&arb, arb.EEPROMadr, 0, sizeof(ARBdata)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

void RestorARBsettings(bool NoDisplay)
{
  ARBdata ad;
//  bool SaveEnableFlag;

  SelectBoard(SelectedARBboard);
  if (ReadEEPROM(&ad, ARB.EEPROMadr, 0, sizeof(ARBdata)) == 0)
  {
    if (strcmp(ad.Name, ARB.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (ad.Size > sizeof(ARBdata)) ad.Size = sizeof(ARBdata);
      ad.EEPROMadr = ARB.EEPROMadr;
      memcpy(&ARB, &ad, ad.Size);
      arb = ARB;
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

void RestorARBsettings(void)
{
  RestorARBsettings(false);
}

// External trigger interrupt vectors here!
void ARBsyncISR(void)
{
   digitalWrite(ARBsync,HIGH);
   digitalWrite(ARBsync,LOW); 
}

// This function is called at powerup to initiaize the ARB board(s).
void ARB_init(int8_t Board, int8_t addr)
{
  // If this is the first board then reset the ARB CPUs
/*
  if (NumberOfARBchannels == 0)
  {
     pinMode(14,OUTPUT);
     digitalWrite(14,LOW);
     delay(10);
     digitalWrite(14,HIGH);
     delay(100);
  }
*/
  // Flag the board as present
  ARBboards[Board] = true;
  // Set active board to board being inited
  SelectedARBboard = Board;
  SelectBoard(Board);
  ARB.EEPROMadr = addr;
  ARBsyncIN[Board] = new DIhandler;
  DIdirARB[Board] = new DIhandler;
  // If normal startup load the EEPROM parameters from the Filament card.
  if (NormalStartup)
  {
    RestorARBsettings(true);
  }
  ARB.EEPROMadr = addr;
  ARBsyncIN[Board]->detach();
  ARBsyncIN[Board]->attached(ARB.ARBsyncIn, ARB.ARBsyncLevel, ARBsyncISR);
  // Init the hardware here...
  pinMode(ARBsync,OUTPUT);
  digitalWrite(ARBsync,LOW);
  // Make sure compress mode is off
  if (NumberOfARBchannels == 0)
  {
    arb = ARB;
    AddMainMenuEntry(&MEARBmodule);
    SelectARBmodule(false);
    CurrentModule = ARBmodule;
    if (ActiveDialog == NULL) ActiveDialog = &ARBdialog;
    // Configure Threads
    ARBthread.setName("ARB");
    ARBthread.onRun(ARB_loop);
    ARBthread.setInterval(100);
    // Add threads to the controller
    control.add(&ARBthread);
    // Start the command clock in case we need it
    ARBclock = new MIPStimer(TMR_ARBclock);
    ARBcompressor_init();
  }
  else
  {
    // Set back to first board
    SelectedARBboard = 0;
    SelectBoard(0);
  }
  // Set the frequency
  SetFrequency(Board,ARBarray[Board].Frequency);
  // If we are using a common clock then signal the ARB module to use external clock
  if(ARBarray[Board].UseCommonClock) SetBool(Board, TWI_SET_EXT_CLOCK, true);
  NumberOfARBchannels += 8;  // Always add eight channels for each board
  // Set the maximum number of channels in the selection menu
  ARBentriesPage1[0].Max = NumberOfARBchannels / 8;
  ARBentriesPage1arb[0].Max = NumberOfARBchannels / 8;
}

// This is the ARB processing loop. This function is called by the task controller
void ARB_loop(void)
{
  int    b;
  bool   bstate;
  static bool FirstPass = true;
  String WFS;
  static DialogBoxEntry *de = GetDialogEntries(ARBentriesPage1, "Frequency");
  static char  CurrentMode[2][7]      = {"------","------"};
  static int   CurrentFreq[2]         = {-1,-1};
  static float CurrentAmplitude[2]    = {-1,-1};
  static float CurrentOffset[2]       = {-1,-1};
  static float CurrentAux[2]          = {-1,-1};
  static int   CurrentBufferLength[2] = {-1,-1};
  static int   CurrentNumBuffers[2]   = {-1,-1};
  static int   CurrentDirection[2]    = {-2,-2};
  static int   CurrentWFT[2]          = {-2,-2};
  static int   CurrentEnable[2]       = {-2,-2};

  if ((ActiveDialog == &ARBdialog) || (ActiveDialog == &ARBCompressorDialog))
  {
    if(ActiveDialog->Changed)
    {
      ARB = arb;
      WFS = WFT;
      ARB.wft = GetWaveformType(WFS);  // In case this value changed
      ARBdialog.Changed = false;
    }
    // If the mode has changed then update the display with the new mode
    arb=ARB;
    if(CurrentModule != ARBmodule)
    {
      SelectARBmodule();
      CurrentModule = ARBmodule;
    }
  }
  ProcessSweep();
  ARBcompressor_loop();
  // Process each ARB board. Look for changes in parameters and update as needed
  for(b = 0; b < 2; b++)
  {
    if(ARBboards[b])
    {
      SelectBoard(b);
      // If mode is Twave make sure frequency is not over the maximum value
      if(strcmp("TWAVE",ARBarray[b].Mode) == 0)
      {
        if(ARBarray[b].Frequency > de->Max) ARBarray[b].Frequency = de->Max;
      }
      if(strcmp(CurrentMode[b],ARBarray[b].Mode) != 0)
      {
        SetMode(b,ARBarray[b].Mode);
        strcpy(CurrentMode[b],ARBarray[b].Mode);
        CurrentEnable[b] = -2;
        if(b == SelectedARBboard)
        {
           strcpy(arb.Mode,ARBarray[b].Mode);
           if(ActiveDialog == &ARBdialog) SetModeMenus(true);
           else SetModeMenus(false);
        }
      }
      if(CurrentFreq[b] != ARBarray[b].Frequency)
      {
        if (ARBarray[0].UseCommonClock) // Make both clocks match if use common clock flag is set
        {
          ARBarray[(b + 1) & 1].Frequency = ARBarray[b].Frequency;
        }
        SetFrequency(b,ARBarray[b].Frequency);
        CurrentFreq[0] =CurrentFreq[1] = ARBarray[b].Frequency;
      }
      if(CurrentAmplitude[b] != ARBarray[b].Voltage)
      {
        SetAmplitude(b, ARBarray[b].Voltage);
        CurrentAmplitude[b] = ARBarray[b].Voltage;
      }
      if(CurrentOffset[b] != ARBarray[b].Offset)
      {
        SetFloat(b, TWI_SET_OFFSETV, ARBarray[b].Offset);
        CurrentOffset[b] = ARBarray[b].Offset;
      }
      if(CurrentAux[b] != ARBarray[b].Aux)
      {
        SetFloat(b, TWI_SET_AUX, ARBarray[b].Aux);
        CurrentAux[b] = ARBarray[b].Aux;
      }
      if(CurrentBufferLength[b] != ARBarray[b].BufferLength)
      {
        SetBufferLength(b, ARBarray[b].BufferLength);
        CurrentBufferLength[b] = ARBarray[b].BufferLength;
      }
      if(CurrentNumBuffers[b] != ARBarray[b].NumBuffers)
      {
        SetNumBuffers(b, ARBarray[b].NumBuffers);
        CurrentNumBuffers[b] = ARBarray[b].NumBuffers;
      }
      if(CurrentDirection[b] != ARBarray[b].Direction)
      {
        SetBool(b, TWI_SET_DIR, !ARBarray[b].Direction);
        CurrentDirection[b] = ARBarray[b].Direction;
      }
      if(CurrentWFT[b] != ARBarray[b].wft)
      {
        SetWaveform(b, ARBarray[b].wft);
        CurrentWFT[b] = ARBarray[b].wft;
      }
      if(CurrentEnable[b] != ARBarray[b].Enable)
      {
        SetBool(b, TWI_SET_ENABLE, ARBarray[b].Enable);
        CurrentEnable[b] = ARBarray[b].Enable;
      }
      if(ARBwaveformUpdate[b])
      {
        SetARBwaveform(b);
        ARBwaveformUpdate[b] = false;
      }
      if ((ARBarray[b].ARBsyncIn != ARBsyncIN[b]->di) || (ARBarray[b].ARBsyncLevel != ARBsyncIN[b]->mode) || FirstPass)
      {
         ARBsyncIN[b]->detach();
         bstate = ARBsyncIN[b]->attached(ARBarray[b].ARBsyncIn, ARBarray[b].ARBsyncLevel, ARBsyncISR);
         SetBool(b, TWI_SET_SYNC_ENA, bstate);
      }
      if ((ARBarray[b].ARBdirDI != DIdirARB[b]->di) || (ARBarray[b].ARBdirLevel != DIdirARB[b]->mode) || FirstPass)
      {
         DIdirARB[b]->detach();
         DIdirARB[b]->attached(ARBarray[b].ARBdirDI, ARBarray[b].ARBdirLevel, ARBdirISRs[b]);
      }
    }
  }
  FirstPass = false;
  SelectBoard(SelectedARBboard);
  if (ActiveDialog->Entry == ARBentriesPage1) RefreshAllDialogEntries(&ARBdialog);
  if (ActiveDialog->Entry == ARBentriesPage1arb) RefreshAllDialogEntries(&ARBdialog);
  if (ActiveDialog->Entry == ARBentriesPage2) RefreshAllDialogEntries(&ARBdialog);
  if (ActiveDialog->Entry == ARBwaveformEdit) RefreshAllDialogEntries(&ARBdialog);
  arb = ARB;
}

//
// ARB Twave serial host commands
//

// Returns the number of ARB channels
void ReportARBchannels(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(NumberOfARBchannels);
}

// This function converts the ARB module number (1 or 2) into the board index.
// The board index is returned, -1 if invalud. If the report flag is
// true this function will send NAK to host.
int ARBmoduleToBoard(int Module, bool report)
{
  int b=-1;

  if(Module == 1)
  {
    if(ARBboards[0]) b = 0;
    if(!ARBboards[0] && ARBboards[1]) b = 1;
  }
  if((Module == 2) && ARBboards[0] && ARBboards[1]) b = 1;
  if(b != -1) return(b);
  if(report)
  {
    SetErrorCode(ERR_BADCMD);
    SendNAK;
  }
  return(b);
}

void SetARBbufferLength(int module, int len)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  ARBarray[b].BufferLength = len;
  SetBufferLength(b, len);
  SendACK;
}

void GetARBbufferLength(int module)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(ARBarray[b].BufferLength);
}

void SetARBbufferNum(int module, int num)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  ARBarray[b].NumBuffers = num;
  SetNumBuffers(b, num);
  SendACK;
}

void GetARBbufferNum(int module)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(ARBarray[b].NumBuffers);
}

void SetARBMode(char *module, char *mode)
{
  String smode;
  int b,mod;

  smode = module;
  mod = smode.toInt();
  if((b = ARBmoduleToBoard(mod,true)) == -1) return;
  smode = mode;
  if((smode == String("TWAVE")) || (smode == String("ARB")))
  {
    strcpy(ARBarray[b].Mode,mode);
    SetMode(b, mode);
    SendACK;
    return;
  } 
   SetErrorCode(ERR_BADARG);
   SendNAK;   
}

void GetARBMode(int module)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  serial->println(ARBarray[b].Mode);
}

void SetWFfreq(int module, int freq)
{
   int b;
  
   if((b = ARBmoduleToBoard(module,true)) == -1) return;
   if(((strcmp(ARBarray[b].Mode,"TWAVE") == 0) && (freq >= 0) && (freq <= 45000)) || ((strcmp(ARBarray[b].Mode,"ARB") == 0) && (freq >= 0) && (freq <= 1500000)))
   {
      ARBarray[b].Frequency = freq;
      SetFrequency(b,freq);
      SendACK;
      return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;   
}

void GetWFfreq(int module)
{
   int clkdiv;
   int actualF;
   int b;
  
   if((b = ARBmoduleToBoard(module,true)) == -1) return;
   SendACKonly;
   if(strcmp(ARBarray[b].Mode,"TWAVE") == 0)
   {
     clkdiv = VARIANT_MCK / (2 * ppp * ARBarray[b].Frequency);
     actualF = VARIANT_MCK / (2 * ppp * clkdiv);
   }
   else
   {
     clkdiv = VARIANT_MCK / (2 * ARBarray[b].Frequency);
     actualF = VARIANT_MCK / (2 * clkdiv);
   }
   if (!SerialMute) serial->println(actualF);
}

void SetWFdisable(int module)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACK;
  ARBarray[b].Enable = false;
  SetBool(b,TWI_SET_ENABLE,false);
}

void SetWFenable(int module)
{
  int b;
  
  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACK;
  ARBarray[b].Enable = true;
  SetBool(b,TWI_SET_ENABLE,true);
}

// This function sets the ARB DAC voltage range
void SetWFrange(char *module, char *srange)
{
  float  range;
  String spar,smod;
  int b,mod;

  smod = module;
  mod = smod.toInt();
  if((b = ARBmoduleToBoard(mod,true)) == -1) return;
  spar = srange;
  range = spar.toFloat();
  if((range >= 0) && (range <= 100))
  {
    ARBarray[b].Voltage = range;
    SetAmplitude(b, range);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetWFrange(int module)
{
  int b;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(ARBarray[b].Voltage);
}

void SetWFoffsetV(char *module, char *srange)
{
  float  range;
  String spar, smod;
  int b, mod;

  smod = module;
  mod = smod.toInt();
  if((b = ARBmoduleToBoard(mod,true)) == -1) return;
  spar = srange;
  range = spar.toFloat();
  if((range >= -50) && (range <= 50))
  {
    ARBarray[b].Offset = range;
    SetFloat(b, TWI_SET_OFFSETV, range);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetWFoffsetV(int module)
{
  int b;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(ARBarray[b].Offset);  
}

void SetWFaux(char *module, char *srange)
{
  float  range;
  String spar, smod;
  int b,mod;

  smod = module;
  mod = smod.toInt();
  if((b = ARBmoduleToBoard(mod,true)) == -1) return;
  spar = srange;
  range = spar.toFloat();
  if((range >= -50) && (range <= 50))
  {
    ARBarray[b].Aux = range;
    SetFloat(b, TWI_SET_AUX, range);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetWFaux(int module)
{
  int b;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(ARBarray[b].Aux);  
}

void SetARBdirection(char *module, char *dir)
{
   String sToken;
   int    b,mod;

   sToken = module;
   mod = sToken.toInt();
   if((b = ARBmoduleToBoard(mod,true)) == -1) return;
   if(strcmp(dir,"FWD") == 0)
   {
     ARBarray[b].Direction = true;
     SendACK;
     return;
   }
   if(strcmp(dir,"REV") == 0)
   {
     ARBarray[b].Direction = false;
     SendACK;
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;
}

void GetARBdirection(int module)
{
  int b;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if (ARBarray[b].Direction)
  {
    if (!SerialMute) serial->println("FWD");
  }
  else if (!SerialMute) serial->println("REV");  
}

void SetARBwfType(char *sMod, char *Swft)
{
   String sToken;
   int    b,mod;

   sToken = sMod;
   mod = sToken.toInt();
   if((b = ARBmoduleToBoard(mod,true)) == -1) return;
   if(strcmp(Swft,"SIN") == 0) ARBarray[b].wft = ARB_SIN;
   else if(strcmp(Swft,"RAMP") == 0) ARBarray[b].wft = ARB_RAMP;
   else if(strcmp(Swft,"TRI") == 0) ARBarray[b].wft = ARB_TRIANGLE;
   else if(strcmp(Swft,"PULSE") == 0) ARBarray[b].wft = ARB_PULSE;
   else if(strcmp(Swft,"ARB") == 0) ARBarray[b].wft = ARB_ARB;
   else
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
   }
   SendACK;
}

void GetARBwfType(int module)
{
  int b;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(GetWaveformString(ARBarray[b].wft));
}

// This function defines an arbitrary waveform. This function pulls
// all the arguments from the input ring buffer
// Module,val1,val2...
void SetARBwaveform(void)
{
   char   *Token;
   String sToken;
   int    i,mod,b;
   float  vals[ppp];

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     mod = sToken.toInt();
     for(i=0;i<ppp;i++)
     {
        GetToken(true);
        if((Token = GetToken(true)) == NULL) break;
        sToken = Token;
        vals[i] = sToken.toFloat();
     }
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test arguments
     if((b = ARBmoduleToBoard(mod,true)) == -1) return;
     for(i=0;i<ppp;i++) if((vals[i] < -100) || (vals[i] > 100))
     {
       SetErrorCode(ERR_BADARG);
       SendNAK;
       return;
     }
     // Fill the waveform buffer and set flag to send waveform
     for(i=0;i<ppp;i++) ARBarray[b].WaveForm[i] = vals[i];
     ARBwaveformUpdate[b] = true;
     // If the user interface is displaying this channel then update the display array
     if((ActiveDialog->Entry == ARBwaveformEdit) && (SelectedARBboard == b))
     {
        for (i = 0; i < ppp; i++) ARBwaveform[i] = vals[i];
     }
     SendACK;
   }
}

// Reports the ARB waveform
void GetARBwaveform(int module)
{
  int b,i;

  if((b = ARBmoduleToBoard(module,true)) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  for(i=0;i<ppp;i++)
  {
    serial->print(ARBarray[b].WaveForm[i]);
    if(i == (ppp-1)) serial->println("");
    else serial->print(","); 
  }
  SendACK;
}

void SetARBchns(char *module, char *sval)
{
   String sToken;
   float  val;
   int    b,mod;

   sToken = module;
   mod = sToken.toInt();
   if((b = ARBmoduleToBoard(mod,true)) == -1) return;
   sToken = sval;
   val = sToken.toFloat();
   if((val < -100) || (val > 100))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
   }
   SetFloat(b, TWI_SET_SET_BUFFER, val);
   SendACK;
}

// This command will set an ARB channel to a defined value over its full range.
// All parameters are pulled from the ring buffer.
//  Module,Channel,value
//  Module is 1 or 2
//  Channel is 1 through 8
void SetARBchannel(void)
{
   char   *Token;
   String sToken;
   int    ch,mod,b;
   float  val;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     mod = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     val = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range arguments
     if((ch < 1) || (ch > 8)) break;
     if((val < -100) || (val > 100)) break;
     if((b = ARBmoduleToBoard(mod,true)) == -1) return;
     SetChannelRangeMessage(b,ch,0,ARBarray[b].BufferLength,val);
     SendACK;
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;  
}

// This command will set an ARB channel to a defined value over a defined range.
// All parameters are pulled from the ring buffer.
//  Module,Channel,Start index,Stop index,value
void SetARBchanRange(void)
{
   char   *Token;
   String sToken;
   int    ch,b,StartI,StopI,mod;
   float  val;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     mod = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     StartI = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     StopI = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     val = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range
     if((b = ARBmoduleToBoard(mod,true)) == -1) return;
     if((ch < 1) || (ch > 8)) break;
     if((StartI < 1) || (StartI >= ARBarray[b].BufferLength)) break;
     if((StopI < 1) || (StopI >= ARBarray[b].BufferLength)) break;
     if(StartI > StopI) break;
     if((val < -100) || (val > 100)) break;
     // Now we can call the function!
     SendACK;
     SetChannelRangeMessage(b,ch,StartI,StopI,val);
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}




