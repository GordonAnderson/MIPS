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
// Gordon Anderson
//
#include "ARB.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

//MIPS Threads
Thread ARBthread  = Thread();

#define ARB ARBarray[SelectedARBboard]

ARBdata  ARBarray[2] = {ARB_Rev_1, ARB_Rev_1};

DIhandler *ARBsyncIN;

int  ARBmodule = 1;
int  NumberOfARBchannels = 0;
bool ARBboards[2] = {false, false};
int  SelectedARBboard = 0;    // Active board, 0 or 1 = A or B

// Array used to edit the ARB waveform
int ARBwaveform[32];

// Variables used for ARB channel editing
int   ARBchan = 0;
float ChanLevel = 0;
float ChanLevelAll = 0;
int   startI = 0;
int   stopI = 10;

char *ARBwfrmList = "SIN,RAMP,TRI,PULSE,ARB";
char WFT[8] = "SIN";

char *ARBmodeList = "TWAVE,ARB";

extern DialogBoxEntry ARBentriesPage2[];
extern DialogBoxEntry ARBwaveformEdit[];
extern DialogBoxEntry ARBentriesRange[];

// TWAVE mode main menu
DialogBoxEntry ARBentriesPage1[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &ARBmodule, NULL, SelectARBmodule},
  {" Mode"               , 0, 2, D_LIST,  0, 0  , 7, 16, false, ARBmodeList, ARB.Mode, NULL, SetMode},
  {" Enable"             , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &ARB.Enable, NULL, SetEnable},
  {" Frequenncy"         , 0, 4, D_INT   , 100, 50000, 100, 18, false, "%5d", &ARB.Frequency, NULL, SetFrequency},
  {" Amplitude, Vp-p"    , 0, 5, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &ARB.Voltage, NULL, SetAmplitude},
  {" Waveform"           , 0, 6, D_LIST,  0, 0  , 7, 16, false, ARBwfrmList, WFT, NULL, SetWaveform},
  {" Dir"                , 0, 7, D_FWDREV, 0, 1  , 1, 20, false, NULL, &ARB.Direction, NULL, SetDirection},
  {" ARB waveform"       , 0, 8, D_PAGE,  0, 0  , 0, 0,  false, NULL, ARBwaveformEdit, Waveform2EditBuffer, NULL},
  {" Next page"          , 0, 9, D_PAGE,  0, 0  , 0, 0,  false, NULL, ARBentriesPage2, NULL, NULL},
  {" Return to main menu", 0,10, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

// ARB mode main menu
DialogBoxEntry ARBentriesPage1arb[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &ARBmodule, NULL, SelectARBmodule},
  {" Mode"               , 0, 2, D_LIST,  0, 0  , 7, 16, false, ARBmodeList, ARB.Mode, NULL, SetMode},
  {" Enable"             , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &ARB.Enable, NULL, SetEnable},
  {" Frequenncy"         , 0, 4, D_INT   , 100, 1000000, 1000, 16, false, "%7d", &ARB.Frequency, NULL, SetFrequency},
  {" Amplitude, Vp-p"    , 0, 5, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &ARB.Voltage, NULL, SetAmplitude},
  {" Buf length"         , 0, 6, D_INT   , 100, 8000, 10, 17, false, "%6d", &ARB.BufferLength, NULL, SetBufferLength},
  {" Num buf"            , 0, 7, D_INT   , 0, 1000000, 1, 17, false, "%6d", &ARB.NumBuffers, NULL, SetNumBuffers},
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
  {" Aux voltage"        , 0, 1, D_FLOAT, -50, 50, 0.1, 18, false, "%5.1f", &ARB.Aux, NULL, SetAux},
  {" Offset, Volts"      , 0, 2, D_FLOAT, -50, 50, 0.1, 18, false, "%5.1f", &ARB.Offset, NULL, SetOffset},

  {" Sync input"         , 0, 4, D_DI     , 0, 0, 2, 21, false, DIlist, &ARB.ARBsyncIn, NULL, NULL},
  {" Sync level"         , 0, 5, D_DILEVEL, 0, 0, 4, 19, false, DILlist, &ARB.ARBsyncLevel, NULL, NULL},

  {" Save settings"      , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveARBsettings, NULL},
  {" Restore settings"   , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorARBsettings, NULL},
  {" First page"         , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, ARBentriesPage1, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry ARBwaveformEdit[] = {
  {"Index     values (0-255)", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"00-03 ",  1, 2, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[0], NULL, NULL},
  {" "     , 11, 2, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[1], NULL, NULL},
  {" "     , 15, 2, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[2], NULL, NULL},
  {" "     , 19, 2, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[3], NULL, NULL},

  {"04-07 ",  1, 3, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[4], NULL, NULL},
  {" "     , 11, 3, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[5], NULL, NULL},
  {" "     , 15, 3, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[6], NULL, NULL},
  {" "     , 19, 3, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[7], NULL, NULL},

  {"08-11 ",  1, 4, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[8], NULL, NULL},
  {" "     , 11, 4, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[9], NULL, NULL},
  {" "     , 15, 4, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[10], NULL, NULL},
  {" "     , 19, 4, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[11], NULL, NULL},

  {"12-15 ",  1, 5, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[12], NULL, NULL},
  {" "     , 11, 5, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[13], NULL, NULL},
  {" "     , 15, 5, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[14], NULL, NULL},
  {" "     , 19, 5, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[15], NULL, NULL},

  {"16-19 ",  1, 6, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[16], NULL, NULL},
  {" "     , 11, 6, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[17], NULL, NULL},
  {" "     , 15, 6, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[18], NULL, NULL},
  {" "     , 19, 6, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[19], NULL, NULL},

  {"20-23 ",  1, 7, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[20], NULL, NULL},
  {" "     , 11, 7, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[21], NULL, NULL},
  {" "     , 15, 7, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[22], NULL, NULL},
  {" "     , 19, 7, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[23], NULL, NULL},

  {"24-27 ",  1, 8, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[24], NULL, NULL},
  {" "     , 11, 8, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[25], NULL, NULL},
  {" "     , 15, 8, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[26], NULL, NULL},
  {" "     , 19, 8, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[27], NULL, NULL},

  {"28-31 ",  1, 9, D_INT, 0, 255, 1, 7, false, "%3d", &ARBwaveform[28], NULL, NULL},
  {" "     , 11, 9, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[29], NULL, NULL},
  {" "     , 15, 9, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[30], NULL, NULL},
  {" "     , 19, 9, D_INT, 0, 255, 1, 1, false, "%3d", &ARBwaveform[31], NULL, NULL},

  {" Return to ARB menu", 0, 11, D_PAGE, 0, 0, 0, 0, false, NULL, ARBentriesPage1, EditBuffer2Waveform, NULL},
  {NULL},
};

DialogBox ARBdialog = {
  {
    "ARB control parameters",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0, ARBentriesPage1
};

MenuEntry MEARBmodule = {" ARB module", M_DIALOG, 0, 0, 0, NULL, &ARBdialog, NULL, NULL};

void SetModeMenus(void)
{
  DialogBoxEntry *de;

  de = GetDialogEntries(ARBentriesPage2, "First page");
  if(strcmp(ARB.Mode,"TWAVE") == 0)
  {
    if(ARB.Frequency > 45000) ARB.Frequency = 45000;
    // Define main menu
    ARBdialog.Entry = ARBentriesPage1;
    // Set the return page menu
    de->Value = (void *)ARBentriesPage1;
    Wire.beginTransmission(ARB.ARBadr);
    Wire.write(TWI_SET_MODE);
    Wire.write(0);
    Wire.endTransmission();  
  }
  else
  {
    // Define main menu
    ARBdialog.Entry = ARBentriesPage1arb;
    // Set the return page menu
    de->Value = (void *)ARBentriesPage1arb;
    Wire.beginTransmission(ARB.ARBadr);
    Wire.write(TWI_SET_MODE);
    Wire.write(1);
    Wire.endTransmission();  
  }  
}

// Sets mode between ARB and TWAVE
void SetMode(void)
{
  SetModeMenus();
  if(strcmp(ARB.Mode,"TWAVE") == 0)
  {
  }
  else
  {
  }
  // Repaint main menu
  DialogBoxDisplay(&ARBdialog);
}

// Sets all channels in ARB mode
void SetChans(void)
{
  uint8_t *b;
  
  b = (uint8_t *)&ChanLevelAll;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_SET_BUFFER);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();  
}

// Sets channel range in ARB mode
void SetChannelRange(void)
{
  uint8_t *b;
  int ch;
  
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_SET_CHN_RNG);

  ch = ARBchan - 1;
  b = (uint8_t *)&ch;
  Wire.write(b[0]);

  b = (uint8_t *)&startI;
  Wire.write(b[0]);
  Wire.write(b[1]);

  b = (uint8_t *)&stopI;
  Wire.write(b[0]);
  Wire.write(b[1]);

  b = (uint8_t *)&ChanLevel;
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();    
}

void SetBufferLength(void)
{
  uint8_t *b;

  b = (uint8_t *)&ARB.BufferLength;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_BUFFER_LEN);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.endTransmission();
}

void SetNumBuffers(void)
{
  uint8_t *b;

  b = (uint8_t *)&ARB.NumBuffers;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_NUM_BUFFER);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.endTransmission();  
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

  for (i = 0; i < ARB.PPP; i++) ARB.WaveForm[i] = ARBwaveform[i];
  // Send to the ARB module
  SetARBwaveform();
}

void Waveform2EditBuffer(void)
{
  int i;

  for (i = 0; i < ARB.PPP; i++) ARBwaveform[i] = ARB.WaveForm[i];
}

// The following Set... function send the user selected parameters to the ARB board.

// Update the dialog box and display
void SelectARBmodule(void)
{
  String wft_string;

  wft_string = GetWaveformString(ARB.wft);
  strcpy(WFT, wft_string.c_str());
}

void SetEnable(void)
{
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_ENABLE);
  if(ARB.Enable) Wire.write(1);
  else Wire.write(0);
  Wire.endTransmission();
}

void SetFrequency(void)
{
  uint8_t *b;
  
  b = (uint8_t *)&ARB.Frequency;
  // Send the frequency to the ARB module
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_FREQ);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.endTransmission();
}

void SetAmplitude(void)
{
  uint8_t *b;
  
  // Amplitude is 0 to 100Vp-p for 0 to 4095 ref level on the ARB
  int i = ARB.Voltage * 32.8;
  if (i > 4095) i = 4095;
  if (i < 0) i = 0;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_REF);
  Wire.write(i & 0xFF);
  Wire.write((i >> 8) & 0xFF);
  Wire.endTransmission();

  b = (uint8_t *)&ARB.Voltage;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_RANGE);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();
}

void SetOffset(void)
{
  uint8_t *b;
  
  // Offset is in % of peak voltage.
  b = (uint8_t *)&ARB.Offset;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_OFFSETV);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();
}

void SetAux(void)
{
  uint8_t *b;
  
  b = (uint8_t *)&ARB.Aux;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_AUX);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  Wire.endTransmission();  
}

// Here after user selects a waveform type, update the data structure.
void SetWaveform(void)
{
  String WFS;

  WFS = WFT;
  ARB.wft = GetWaveformType(WFS);
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_WAVEFORM);
  switch (ARB.wft)
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
}

void SetDirection(void)
{
  int i;

  if (ARB.Direction) i = 1;
  else i = 0;
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_DIR);
  Wire.write(i & 0xFF);
  Wire.endTransmission();
}

void SetARBwaveform(void)
{
  int i;
  
  // Send to the ARB module
  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_VECTOR);
  Wire.write(0);
  Wire.write(16);
  for (i = 0; i < 16; i++) Wire.write(ARB.WaveForm[i]);
  Wire.endTransmission(); 

  Wire.beginTransmission(ARB.ARBadr);
  Wire.write(TWI_SET_VECTOR);
  Wire.write(16);
  Wire.write(16);
  for (i = 0; i < 16; i++) Wire.write(ARB.WaveForm[i+16]);
  Wire.endTransmission(); 
}

// Write the current board parameters to the EEPROM on the ARB board.
void SaveARBsettings(void)
{
  SelectBoard(SelectedARBboard);
  ARB.Size = sizeof(ARBdata);
  if (WriteEEPROM(&ARB, ARB.EEPROMadr, 0, sizeof(ARBdata)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

void RestorARBsettings(bool NoDisplay)
{
  ARBdata ad;
  bool SaveEnableFlag;

  SelectBoard(SelectedARBboard);
  if (ReadEEPROM(&ad, ARB.EEPROMadr, 0, sizeof(ARBdata)) == 0)
  {
    if (strcmp(ad.Name, ARB.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (ad.Size > sizeof(ARBdata)) ad.Size = sizeof(ARBdata);
      memcpy(&ARB, &ad, ad.Size);
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      // Select the module and set all the ARB parameters
      SaveEnableFlag = ARB.Enable;
      ARB.Enable = false;
      SetEnable(); delay(10);
      ARB.Enable = SaveEnableFlag;
      SelectARBmodule(); delay(10);
      SetFrequency(); delay(10);
      SetAmplitude(); delay(10);
      SetOffset(); delay(10);
      SetAux(); delay(10);
      SetBufferLength(); delay(10);
      SetNumBuffers(); delay(10);
      SetWaveform(); delay(100);
      SetDirection(); delay(100);
      SetARBwaveform(); delay(1);
      SetEnable(); delay(1);
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
void ARB_init(int8_t Board)
{
  // Flag the board as present
  ARBboards[Board] = true;
  // Set active board to board being inited
  SelectedARBboard = Board;
  SelectBoard(Board);
  ARBsyncIN = new DIhandler;
  // If normal startup load the EEPROM parameters from the Filament card.
  if (NormalStartup)
  {
    RestorARBsettings(true);
  }
  ARBsyncIN->detach();
  ARBsyncIN->attached(ARB.ARBsyncIn, ARB.ARBsyncLevel, ARBsyncISR);

  // Init the hardware here...
  pinMode(ARBsync,OUTPUT);
  digitalWrite(ARBsync,LOW);

  SetModeMenus();
  // Setup the menu if this is the very first call to this init function
  if (NumberOfARBchannels == 0)
  {
    AddMainMenuEntry(&MEARBmodule);
    if (ActiveDialog == NULL) DialogBoxDisplay(&ARBdialog);
    // Configure Threads
    ARBthread.setName("ARB");
    ARBthread.onRun(ARB_loop);
    ARBthread.setInterval(100);
    // Add threads to the controller
    control.add(&ARBthread);
  }
  NumberOfARBchannels += 8;  // Always add eight channels for each board
  // Set the maximum number of channels in the selection menu
  ARBentriesPage1[0].Max = NumberOfARBchannels / 8;
  // Use trig input on pin 12 as a trigger to reset the sequence generator
//  attachInterrupt(12, ARBsyncISR, RISING);
}

// This is the ARB processing loop. This function is called by the task controller
void ARB_loop(void)
{
  SelectBoard(SelectedARBboard);
  if (ActiveDialog->Entry == ARBentriesPage1) RefreshAllDialogEntries(&ARBdialog);
  if (ActiveDialog->Entry == ARBentriesPage1arb) RefreshAllDialogEntries(&ARBdialog);
  if (ActiveDialog->Entry == ARBentriesPage2) RefreshAllDialogEntries(&ARBdialog);
  
  if ((ARB.ARBsyncIn != ARBsyncIN->di) || (ARB.ARBsyncLevel != ARBsyncIN->mode))
  {
     ARBsyncIN->detach();
     ARBsyncIN->attached(ARB.ARBsyncIn, ARB.ARBsyncLevel, ARBsyncISR);
  }
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

void SetARBbufferLength(int len)
{
  SendACKonly;
  ARB.BufferLength = len;
  SetBufferLength();
}

void SetARBbufferNum(int num)
{
  SendACKonly;  
  ARB.NumBuffers = num;
  SetNumBuffers();
}

void SetARBMode(char *mode)
{
  String smode;

  smode = mode;
  if((smode == String("TWAVE")) || (smode == String("ARB")))
  {
    strcpy(ARB.Mode,mode);
    SetMode();
    SendACK;
    return;
  } 
   SetErrorCode(ERR_BADARG);
   SendNAK;   
}

void GetARBMode(void)
{
  SendACKonly;
  serial->println(ARB.Mode);
}

void SetWFfreq(int freq)
{
   if(((strcmp(ARB.Mode,"TWAVE") == 0) && (freq >= 0) && (freq <= 45000)) || ((strcmp(ARB.Mode,"ARB") == 0) && (freq >= 0) && (freq <= 1500000)))
   {
      SetFrequency();
      ARB.Frequency = freq;
      SetFrequency();
      SendACK;
      return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;   
}

void GetWFfreq(void)
{
   int clkdiv;
   int actualF;

   SendACKonly;
   if(strcmp(ARB.Mode,"TWAVE") == 0)
   {
     clkdiv = VARIANT_MCK / (2 * ppp * ARB.Frequency);
     actualF = VARIANT_MCK / (2 * ppp * clkdiv);
   }
   else
   {
     clkdiv = VARIANT_MCK / (2 * ARB.Frequency);
     actualF = VARIANT_MCK / (2 * clkdiv);
   }
   if (!SerialMute) serial->println(actualF);
}

// This function sets the ARB DAC voltage range
void SetWFdisable(void)
{
    ARB.Enable = false;
    SetEnable();
    SendACK;
}

void SetWFenable(void)
{
    ARB.Enable = true;
    SetEnable();
    SendACK;
}

void SetWFrange(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if((range >= 0) && (range <= 100))
  {
    ARB.Voltage = range;
    SetAmplitude();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFoffsetV(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if((range >= -50) && (range <= 50))
  {
    ARB.Offset = range;
    SetOffset();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFaux(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if((range >= -50) && (range <= 50))
  {
    ARB.Aux = range;
    SetAux();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetARBchns(char *sval)
{
   String sToken;
   float  val;

   sToken = sval;
   val = sToken.toFloat();
   if((val < -100) || (val > 100))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
   }
   ChanLevelAll = val;
   SetChans();
   SendACK;
}

void SetARBchannel(char * sch, char *sval)
{
   String sToken;
   int    ch;
   float  val;

   while(1)
   {
     sToken = sch;
     ch = sToken.toInt();
     if((ch < 1) || (ch > 8)) break;
     sToken = sval;
     val = sToken.toFloat();
     if((val < -100) || (val > 100)) break;
     ARBchan = ch;
     ChanLevel = val;
     startI = 0;
     stopI = ARB.BufferLength;
     SetChannelRange();
     SendACK;
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;  
}

// This command will set an ARB channel to a defined value over a defined range.
// All parameters are pulled from the ring buffer.
//  Channel,Start index,Stop index,value
void SetARBchanRange(void)
{
   char   *Token;
   String sToken;
   int    ch;
   float  val;

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
     startI = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     stopI = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     val = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the range
     if((ch < 1) || (ch > 8)) break;
     if((startI < 1) || (startI >= ARB.BufferLength)) break;
     if((stopI < 1) || (stopI >= ARB.BufferLength)) break;
     if(startI > stopI) break;
     if((val < -100) || (val > 100)) break;
     // Now we can call the function!
     ARBchan = ch;
     ChanLevel = val;
     SetChannelRange();
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}


