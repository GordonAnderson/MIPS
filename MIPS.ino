 #include <DueFlashStorage.h>
#include "SD.h"
#include "utility/Sd2card.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>
#include "Variants.h"
#include "Encoder.h"
#include <stdarg.h>
#include <Wire.h>
#include "AtomicBlock.h"
#include "ethernet.h"
#include <Thread.h>
#include <ThreadController.h>
#include <MIPStimer.h>
#include <DIhandler.h>
//
// MIPS
//
// This sketch controls the MIPS system. This MIPS firmware uses the thread controller developed by
// Seidel Gomes, for details, go to https://github.com/ivanseidel/ArduinoThread. This is a round robin
// tasking system. Basically every module in MIPS is its own task. The modules are discovered on power up
// and there init functions are called. The modules then insert there menu option into the main system
// menu. There are a couple of additional threads defined in this main loop, one for the LED light
// flashing and a system thread that monitors power. All module tasks run at 10 herts, serial commands
// allow you to see all running tasks and you can stop tasks if you wish.
//
// The code in the modules do not block unless it is desired to stop all other tasks. Very little is
// done in ISRs with the exception of the table execution for pulse sequence generation.
//
// MIPS used ADCs and DACs with TWI and SPI interfaces. The TFT display is also SPI as does the SD card
// interface.
//
// Host computer communications is supported through both the native USB and serial ports. On power up
// it defaults to native USB and if serial traffic is seen it switches to the active port automatically.
// Ethernet and WiFi interfaces are supported by connecting interfaces to a processor serial port.
//
// Incoming serial characters are placed in a ring buffer and processed in the main loop. All commands
// are processed in the polling loop when there is avalible time. The serial command processor is table
// driven and the command table is in the serial.cpp file.
//
// The Variants.h file has a number of options for building the application. See the Variants file for
// details.
//
// All modules have a data structures and the first three parameters are the same in all structures, size,
// name, and rev. These structures are saved on the module hardware in serial EPROM. When the system starts
// it scans all posible locations for EEPROMS looking for modules. These modules are identified by the name
// string. The MIPS system has a board select line and modules all have a A/B jumper to select there board
// address. Modules address are 50,52,54,56 (hex) and with board selet this allows 8 modules maximum in one
// MIPS system.
//
// The rev parameter in the module data structure will effect the module behavior and is used to signal
// different things on different modules. The module code comments define what the rev value will do, there
// is a serial command to allow you to define the rev.
//
// Modules such as the FAIMSFB and RFdriver2 have a M0 on module processor. This processor emulates the
// serial EEPROM. The ARB module also has a M3 on module processor. These modules also have a USB port
// that will allow you to communicate with the M0 processor for setup, testing, and calibration. Communications
// with the on module processor is also enabled through MIPS using the TWITALK capability. TWITALK
// redirects communications through the TWI port between the MIPS controller and the module. In the case
// of the ARB you can also update the ARB firmware through this interfaces if you use the MIPS host app.
//
// General Notes:
//  1.) When SAVE command is sent MIPS will save its configuration data structure to a file called
//      default.cfg on the SD card. This file is automatially loaded on startup.
//  2.) MIPS code uses a first order difference equation to filter the readback data. Here is the
//      transfer function of this filter:
//      Filter time constant is:
//      TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
//  3.) The PWM output frequency needs to be increased from its default setting of 1000 Hz
//      to 50KHz for the RF driver module. The module used the following PWM output pins,
//      5 and 7 for board A and 6 and 8 for board B. This can be changed my editing the
//      variant.h file in the arduino source and changing two #defines:
//            #define PWM_FREQUENCY    50000
//            #define TC_FREQUENCY    50000
//      Also the PWM resolution is set to 10 bits by editing the following parameters in the
//      same file:
//            #define PWM_MAX_DUTY_CYCLE  1023
//            #define PWM_RESOLUTION      10
//            #define TC_MAX_DUTY_CYCLE   1023
//            #define TC_RESOLUTION       10
//      If the IDE is updated then these values need to also be updated. I am looking for
//      a cleaner way to do this.
//  4.) Use of Push button LEDs:
//        1.) When output voltages exceed 0 the blue LED will be on indicating output are on.
//        2.) If any output votlages exceeds 50 volts the red LED will be on to provide warning.
//        3.) If any output exceeds 100 volts the red LED will flash.
//        4.) Commands are provided to allow an application to override the LED
//
// Revision history, bugs, and to do action can be found in the 1_History tab
//
// Serial.println(); will print '\r' and '\n',
//
// Gordon Anderson
// GAA Custom Electronics, LLC
// gaa@owt.com
// 509.628.6851 (cell)
// 509.588.5410
//

#pragma GCC optimize "-Os"

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

//#define IDE 1.6.5

#define UseWDT

// Define my names for the native USB driver
#define  USB_PRODUCT      "MIPS, native USB"
//#pragma "-DUSB_PRODUCT=MIPS, native USB"
#define  USB_MANUFACTURER "GAA Custom Electronics, LLC"

DueFlashStorage    dueFlashStorage;
const PROGMEM byte NonVolStorage[1000] = {};

bool Suspend = false;

// These values are defaulted to 1000 in Variant.h, The RFdriver needs this PWM
// frequency set to 50KHz for its adjustable power supply. Not sure if this overrides
// the values in the .h file?
#define PWM_FREQUENCY	      50000
#define PWM_MAX_DUTY_CYCLE	1023
#define PWM_RESOLUTION		  10
#define TC_FREQUENCY        50000
#define TC_MAX_DUTY_CYCLE   1023
#define TC_RESOLUTION		    10

bool NormalStartup = true;
bool SDcardPresent = false;

bool DisableDisplay = false;

bool LEDoverride = false;
int  LEDstate = 0;

uint32_t BrightTime = 0;

#if FAIMSFBcode
#pragma message "FAIMSFB module enabled."
#define FAIMSFBvf "b"
#else
#define FAIMSFBvf ""
#endif
#if FAIMScode
#pragma message "FAIMS module enabled."
#define FAIMSvf "f"
#else
#define FAIMSvf ""
#endif
#if HOFAIMcode
#pragma message "HOFAIMS module enabled."
#define HOFAIMSvf "h"
#else
#define HOFAIMSvf ""
#endif
#if TABLE2code
#pragma message "Table 2 module enabled."
#define TABLE2vf "t"
#else
#define TABLE2vf ""
#endif
#if RFdriver2
#pragma message "RFdriver 2 module enabled."
#define RFdriver2vf "r"
#else
#define RFdriver2vf ""
#endif
#if HVPScode
#pragma message "HVPS module enabled."
#define HVPSv "v"
#else
#define HVPSv ""
#endif
#if DMSDMSMB
#pragma message "DMSDMSMB module enabled."
#define DMSDMSMBv "d"
#else
#define DMSDMSMBv ""
#endif
#if DCBanalog
#pragma message "DCB analog module enabled."
#define DCBanalogv "a"
#else
#define DCBanalogv ""
#endif
#if DCBcurrent
#pragma message "DCB current monitor enabled."
#define DCBcurrentv "c"
#else
#define DCBcurrentv ""
#endif
#if DCBswitchCode
#pragma message "DCB switch module enabled."
#define DCBswitchCodev "s"
#else
#define DCBswitchCodev ""
#endif

const char Version[] PROGMEM = "Version 1.249" DCBswitchCodev DCBanalogv DCBcurrentv FAIMSFBvf FAIMSvf HOFAIMSvf TABLE2vf RFdriver2vf HVPSv DMSDMSMBv ",Aug 28,2024";

// ThreadController that will control all threads
ThreadController control = ThreadController();
//MIPS Threads
Thread MIPSsystemThread = Thread();
Thread LEDThread        = Thread();

Encoder enc;
int encValue = 0;
bool ButtonPressed = false;
bool ButtonRotated = false;

bool EnableSerialNavigation = false;

extern Menu HardwareTestMenu;
extern DialogBox TwaveDialog;
extern DialogBox MIPSconfig;
extern DialogBox MIPSabout;
extern DialogBox ModuleConfig;
extern DialogBox MacroOptions;

#define MaxMainMenuEntries  16

int SerialWatchDog = 0;

void SerialPortReset(void);
void DisplayAbout(void);

char BoardAddress[20] = "";
char BoardName[20] = "";

// MIPS main menu
MenuEntry MainMenuEntries[MaxMainMenuEntries] = {
  {" MIPS Configuration", M_DIALOG, 0, 0, 0, NULL, &MIPSconfig, NULL, NULL},
  {" About this system",  M_DIALOG, 0, 0, 0, NULL, &MIPSabout, NULL, DisplayAbout},
  {" Reset serial port",  M_FUNCTION, 0, 0, 0, NULL, NULL, SerialPortReset, NULL},
  {""}
};

Menu MainMenu = {
  {"MIPS main menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, MainMenuEntries
};

extern DialogBoxEntry MIPSconfigEntriesP2[];

// MIPS configuration dialog box
DialogBoxEntry MIPSconfigEntries[] = {
  {" Controller rev"     , 0, 1, D_INT,    1, 10, 1, 21, false, "%2d", &MIPSconfigData.Rev, NULL, NULL},
  {" Config modules"     , 0, 2, D_DIALOG, 0, 0, 0, 0, false, NULL, &ModuleConfig, SetupModuleConfig, NULL},
  {" Startup delay"      , 0, 3, D_INT,    0, 100, 1, 20, false, "%3d", &MIPSconfigData.StartupDelay, NULL, NULL},
  {" Startup hold"       , 0, 4, D_YESNO,  0, 1, 1, 21, false, NULL, &MIPSconfigData.StartupHold, NULL, NULL},
  {" DCbias supply"      , 0, 5, D_ONOFF,  0, 1, 1, 20, false, NULL, &MIPSconfigData.PowerEnable, NULL, NULL},
  {" DCbias trip, %%FS"  , 0, 6, D_FLOAT,  0, 100, 0.1, 18, false, "%5.1f", &MIPSconfigData.VerrorThreshold, NULL, NULL},
  {" Reboot"             , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, Software_Reset, NULL},
  {" Backlight, %%"      , 0, 8, D_INT,      5, 100, 1, 20, false, "%3d", &MIPSconfigData.BackLight, NULL, SetBackLight},
  {" Next page"          , 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, MIPSconfigEntriesP2, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry MIPSconfigEntriesP2[] = {
  {" Macro options"     , 0, 1,  D_DIALOG,   0, 0, 0, 0, false, NULL, &MacroOptions, SetupMacroOptions, NULL},
  {" Interlock input"   , 0, 2,  D_DI      , 0, 0, 2, 21, false, DIlist, &MIPSconfigData.InterlockIn, NULL, NULL},
  {" Interlock Output"  , 0, 3,  D_DO      , 0, 0, 2, 21, false, DOlist, &MIPSconfigData.InterlockOut, NULL, NULL},
  {" Enable log"        , 0, 4,  D_ONOFF   , 0, 1, 1, 20, false, NULL, &logdata.enabled, NULL, NULL},
  {" Show log"          , 0, 5,  D_DIALOG  , 0, 0, 0, 0, false, NULL, &MIPSlog, NULL, DisplayLog},
  {" Save settings"     , 0, 9,  D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveMIPSSettings, NULL},
  {" Restore settings"  , 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreMIPSSettings, NULL},
  {" First page"        , 0, 11, D_PAGE    , 0, 0, 0, 0, false, NULL, MIPSconfigEntries, NULL, NULL},
  {NULL},
};

DialogBox MIPSconfig = {
  {"MIPS Configuration", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MIPSconfigEntries
};

DialogBoxEntry MIPSaboutEntries[] = {
  {"Return to main menu/Next", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox MIPSabout = {
  {"About this system", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MIPSaboutEntries
};

// MIPS module setup / config dialog box
DialogBoxEntry ModuleConfigEntries[] = {
  {" Board addr", 0, 3, D_LIST, 0, 0, 10, 12, false, BoardAddressList, BoardAddress, NULL, NULL},
  {" Board type", 0, 4, D_LIST, 0, 0, 10, 12, false, BoardVariantsNames, BoardName, NULL, NULL},
  {" Format!",    0, 6, D_FUNCTION, 0, 0, 0, 16, false, NULL, NULL, BoardFormat, NULL},
  {" Return to config menu", 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &MIPSconfig, NULL, NULL},
  {" Format Module's EEPROM", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {NULL},
};

DialogBox ModuleConfig = {
  {"Module Configuration", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, ModuleConfigEntries
};

// Macro dialog box supporting the following
//  - Select and play a macro
//  - Select a macro to play at power up

char *MacroNameList = NULL;
char PlayMacro[20] = "";

DialogBoxEntry MacroEntries[] = {
  {" Play", 0, 2, D_LIST, 0, 0, 10, 12, false, MacroNameList, PlayMacro, NULL, UIplayMacro},
  {" Startup", 0, 3, D_LIST, 0, 0, 10, 12  , false, MacroNameList, MIPSconfigData.StartupMacro, NULL, NULL},
  {" Return to config menu", 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &MIPSconfig, NULL, NULL},
  {NULL},
};

DialogBox MacroOptions = {
  {"Macro options", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MacroEntries
};

// This function is called before the macro menu loads.
void SetupMacroOptions(void)
{
  MacroNameList = MacroBuildList("NONE");
  MacroEntries[0].fmt = MacroNameList;
  MacroEntries[1].fmt = MacroNameList;
}

void SetBackLight(void)
{
  static bool inited = false;

  if (!inited)
  {
    inited = true;
    analogWriteResolution(12);
    pinMode(BACKLIGHT, OUTPUT);
  }
  analogWrite(BACKLIGHT, (MIPSconfigData.BackLight * 4095) / 100);
}

extern uint32_t _usbInitialized;
void SerialPortReset(void)
{
  LogMessage("USB port reset.");
  SerialUSB.flush();
  SerialUSB.end();
  // Reset the serial USB port
  otg_disable();             // Disable the hardware
  otg_disable_pad();
  otg_freeze_clock();
  pmc_disable_upll_clock();  // This is a critical step, the USB clock gets locked up by noise and the USB port freezes
  delay(10);
  pmc_enable_upll_clock();
  otg_unfreeze_clock();
  otg_enable();              // Enable the hardware
  otg_enable_pad();
  UDD_Init();                // Sets up the USB clock PLL, PMC
  otg_unfreeze_clock();
  _usbInitialized = 1UL;
  digitalWrite(72, HIGH);
  USBDevice.attach();  // Inits the USB serial port, this is where it gets stuck
  digitalWrite(72, HIGH);

  SerialInit();
  SerialUSB.accept();
}

// This function is called by the serial command processor. This function will report the following:
// MIPS version
// MIPS name
// List of modules, board, address, name, rev
void About(void)
{
  uint8_t addr;
  char    signature[100];
  int8_t  rev;
  int     iStat;

  SendACKonly;
  if (SerialMute) return;
  int b = SelectedBoard();
  // Report the system version and name
  serial->print("MIPS: ");
  serial->println(Version);
  serial->print("MIPS name: ");
  serial->println(MIPSconfigData.Name);
  if (MIPSconfigData.UseWiFi) serial->println("WiFi module enabled");
  if (EthernetPresent) serial->println("Ethernet module enabled");
  // Report all the modules and revisions
  serial->println("System modules:");
  serial->println("  Board,Address,Name,Rev");
  for (int i = 0; i < NumModAdd; i++)
  {
    addr = ModuleAddresses[i];
    // Set board select to A
    ENA_BRD_A;
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      iStat = ReadEEPROM(signature, addr, 0, 100);
    }
    if (iStat == 0)
    {
      serial->print("  a,");
      serial->print(addr, HEX);
      serial->print(",");
      serial->print(&signature[2]);
      serial->print(",");
      rev = signature[22];
      serial->println(rev);
    }
    if (!MIPSconfigData.UseBRDSEL) continue;
    // Set board select to B
    ENA_BRD_B;
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      iStat = ReadEEPROM(signature, addr, 0, 100);
    }
    if (iStat == 0)
    {
      serial->print("  b,");
      serial->print(addr, HEX);
      serial->print(",");
      serial->print(&signature[2]);
      serial->print(",");
      rev = signature[22];
      serial->println(rev);
    }
  }
  SelectBoard(b);
}

// This function is called after the About dialog box is created. This function
// needs to display all the about data.
// If they are more lines than will fit on the display then this function will
// return with the menu system set to display the remaining items the next time
// this function is called.
void DisplayAbout(void)
{
  static int8_t i = 0;
  uint8_t addr;
  char    signature[100], buf[25];
  int y = 0;

  if (i == 0)
  {
    PrintDialog(&MIPSabout, 1, y++, "MIPS Version:");
    PrintDialog(&MIPSabout, 2, y++, (char *)&Version[8]);
    PrintDialog(&MIPSabout, 1, y, "MIPS Name:");
    PrintDialog(&MIPSabout, 11, y++, MIPSconfigData.Name);
    if (MIPSconfigData.UseWiFi) PrintDialog(&MIPSabout, 1, y++, "WiFi module enabled");
    if (EthernetPresent) PrintDialog(&MIPSabout, 1, y++, "Ethernet module enabled");
    PrintDialog(&MIPSabout, 1, y++, "System modules:");
    PrintDialog(&MIPSabout, 1, y++, "  Board,Addr,Name,Rev");
  }
  for (i; i < NumModAdd; i++)
  {
    if (y > 9)
    {
      // Set dialog exit to call this function again
      MIPSaboutEntries[0].Type = D_DIALOG;
      MIPSaboutEntries[0].Value = &MIPSabout;
      MIPSaboutEntries[0].PostFunction = DisplayAbout;
      return;
    }
    addr = ModuleAddresses[i];
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      sprintf(buf, "  a,%x,%s,%d", addr, &signature[2], signature[22]);
      PrintDialog(&MIPSabout, 1, y++, buf);
    }
    if (!MIPSconfigData.UseBRDSEL) continue;
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      sprintf(buf, "  b,%x,%s,%d", addr, &signature[2], signature[22]);
      PrintDialog(&MIPSabout, 1, y++, buf);
    }
  }
  i = 0;
  // Set dialog exit to return to main menu
  MIPSaboutEntries[0].Type = D_MENU;
  MIPSaboutEntries[0].Value = &MainMenu;
  MIPSaboutEntries[0].PostFunction = NULL;
}

// This function is called by the serial command processor. This function will set a modules rev level to the value
// defined. All the parameters are pulled from the input ring buffer. The parameters are board,addr,rev
void SetModuleRev(void)
{
  char    *Token;
  char    *ptr;
  String  sToken;
  char    brd;
  uint8_t addr, rev;

  while (1)
  {
    // Read the board address
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    brd = toupper(Token[0]);
    // Read the hex address
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    addr = strtol(Token, &ptr, 16);
    // Read the target rev level
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    rev = sToken.toInt();
    // Validate the parameters
    if ((brd != 'A') && (brd != 'B')) break;
    if ((addr != 0x50) && (addr != 0x52) && (addr != 0x54) && (addr != 0x56) && (addr != 0x60)) break;
    // Write the rev number to EEPROM
    if (brd == 'A') ENA_BRD_A;
    else ENA_BRD_B;
    if (WriteEEPROM(&rev, addr, 22, 1) != 0)
    {
      SetErrorCode(ERR_EEPROMWRITE);
      SendNAK;
      return;
    }
    SendACK;
    return;
  }
  // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Called after macro is selected to play
void UIplayMacro(void)
{
  MacroPlay(PlayMacro, true);
}

// This function is called before the module configuration menu is loaded.
// This function fills the initial board address and name variables.
void SetupModuleConfig(void)
{
  sprintf(BoardAddress, "%s", GetEntry(BoardAddressList, 1));
  sprintf(BoardName, "%s", GetEntry(BoardVariantsNames, 1));
}

// This function is called from the host command processor and the arguments are in the
// serial input ring buffer on call. The arguments are case sesative and the same as the
// MIPS UI configure menu format options, example usage:
//
// FORMAT,A 0x50,RFdrvA R1
void FormatEEPROM(void)
{
  char   *Token;

  while (1)
  {
    // Read the address
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    strcpy(BoardAddress, Token);
    // Read the board name
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    strcpy(BoardName, Token);
    GetToken(true);
    SendACKonly;
    if (BoardFormatEEPROM()) serial->println("Module formatted!");
    else serial->println("Unable to format");
    return;
  }
  // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This function is called by the UI to format a board's EEPROM.
void BoardFormat(void)
{
  if (BoardFormatEEPROM()) DisplayMessage("Module formatted!", 2000);
  else DisplayMessage("Unable to format!", 2000);
}

bool BoardFormatEEPROM(void)
{
  void *BoardConfigData;
  uint8_t addr;
  uint8_t brd;
  int CurrentBoard;

  // Scan the board number and board address from the BoardAddress string
  // Two board posibilities, A and B, and 4 addresses, 50,52,54, and 56.
  if (BoardAddress[0] == 'A') brd = 0;
  if (BoardAddress[0] == 'B') brd = 1;
  if (strcmp(&BoardAddress[2], "0x50") == 0) addr = 0x50;
  if (strcmp(&BoardAddress[2], "0x52") == 0) addr = 0x52;
  if (strcmp(&BoardAddress[2], "0x54") == 0) addr = 0x54;
  if (strcmp(&BoardAddress[2], "0x56") == 0) addr = 0x56;
  // Setup a pointer to the selected default data structure and get its size
  BoardConfigData = BoardVariants[FindInList(BoardVariantsNames, BoardName) - 1];
  // Write it to the selected board
  CurrentBoard = digitalRead(BRDSEL);  // Save current board select
  if (brd == 0) ENA_BRD_A;             // Select board
  else ENA_BRD_B;
  // Write to EEPROM
  if (WriteEEPROM(BoardConfigData, addr, 0, *(int16_t *)BoardConfigData) != 0)
  {
    digitalWrite(BRDSEL, CurrentBoard);  // Restore board select
    return false;
  }
  digitalWrite(BRDSEL, CurrentBoard);  // Restore board select
  return true;
}

void SaveMIPSSettings(void)
{

  if (SAVEparms("default.cfg") == 0) DisplayMessage("Parameters Saved!", 2000);
  else DisplayMessage("Error saving!", 2000);
}

void RestoreMIPSSettings(void)
{
  if (LOADparms("default.cfg")) DisplayMessage("Parameters Restored!", 2000);
  else DisplayMessage("Unable to Restore!", 2000);
}

// This function will add the menu entry to the MIPS main menu.
// This is used by modules as they init to add them selfs to
// the system.
void AddMainMenuEntry(MenuEntry *me);
void AddMainMenuEntry(MenuEntry *me)
{
  int   i;

  for (i = 0; i < MaxMainMenuEntries - 1; i++)
  {
    if (MainMenuEntries[i].Name[0] == 0)
    {
      memcpy(&(MainMenuEntries[i]), me, sizeof(MenuEntry));
      MainMenuEntries[i + 1].Name[0] = 0;
      return;
    }
  }
}

// Using software SPI is really not suggested, its incredibly slow
// Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _mosi, _sclk, _rst, _miso);
// Use hardware SPI
Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);
//HX8347 tft = HX8347(_cs, _dc, _rst);

File myFile;

// This function is called when the button is changed or pressed
// to raise the intensity. If the intensity is less then 50% it is set
// to 50% and a timer is started, after 30 seconds the intensity is
// reduced to the user setting
void DisplayIntensity(void)
{
  if (MIPSconfigData.BackLight >= 50) return;
  analogWrite(BACKLIGHT, 2047);  // Set to 50%
  BrightTime = millis();
}

void encChanged(void)
{
  DisplayIntensity();
  if (DisableDisplay)
  {
    // If serial is enabled then send message
    if (!SerialMute) serial->println("Button rotated.");
    return;
  }
  ButtonRotated = true;
}

void encPB(void)
{
  DisplayIntensity();
  if (DisableDisplay)
  {
    // If serial is enabled then send message
    if (!SerialMute) serial->println("Button pressed.");
    if (strlen(MIPSconfigData.BootImage) <= 0) return;
    // If here the button was pressed with a boot image so enable the display
    DisableDisplay = false;
    return;
  }
  ButtonPressed = true;
  // If we are auto tuning then abort
  TuneAbort = true;
}

// Watchdog timer setup routine, this function will enable the watchdog timer.
// call WDT_Restart(WDT) at least every 2.5 sec to stop an automatic reset.
// the setup routine runs once when you press reset:
#ifdef UseWDT
//double timeout = 5.0;  // number of seconds for timeout
double timeout = 8.0;  // number of seconds for timeout
int timeout2 = 0;
void WDT_Setup () {

#ifdef TestMode
  return;
#endif

  timeout2 = (int)( timeout * 227);

  timeout2 = 0x0fff2000 + timeout2;
  WDT_Enable(WDT, timeout2);

  // number of loops:
  // 0x0fff2000 0
  // 0x0fff200f 231
  // 0x0fff2fff 2981
}
#endif

#define WDT_KEY (0xA5)
void watchdogSetup(void) {
  /*** watchdogDisable (); ***/
}

void watchdogEnable(void)
{
  // Enable watchdog.
  WDT->WDT_MR = WDT_MR_WDD(0xFFF)
                //              | WDT_MR_WDRPROC       // This flag will cause a processor reset only, without it all hardware is reset
                | WDT_MR_WDRSTEN
                | WDT_MR_WDV(256 * 8); // Watchdog triggers a reset after 2 seconds if underflow
  // 2 seconds equal 84000000 * 2 = 168000000 clock cycles
  /* Slow clock is running at 32.768 kHz
    watchdog frequency is therefore 32768 / 128 = 256 Hz
    WDV holds the periode in 256 th of seconds  */
}


void Signon(void)
{
  bool PBwasPressed = false;

  PB_GREEN_OFF;
  PB_RED_OFF;
  PB_BLUE_OFF;
  // If the controll push button is pressed then wait for it to be released before
  // proceeding.
#ifndef TestMode
  while (digitalRead(PB) == HIGH) PBwasPressed = true;
#endif
  delay(100); // bounce delay
  //
  tft.fillScreen(ILI9340_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_YELLOW, ILI9340_BLACK); tft.setTextSize(8);
  tft.println("MIPS");
  tft.setTextSize(1);
  tft.println("  Modular");
  tft.println("  Intelligent");
  tft.println("  Power");
  tft.println("  Sources\n");
  tft.println(Version);
  tft.println("GAA Custom Electronics, LLC\n\n");
  tft.setTextSize(2);
  tft.setTextColor(ILI9340_GREEN, ILI9340_BLACK);
  tft.println("\nSystem initialized!");
  if (MIPSconfigData.StartupHold)
  {
    tft.println("\nPress knob to load");
    tft.println("parameters and startup.");
  }
  else
  {
    tft.println("\nPress knob to startup");
    if (PBwasPressed) tft.println("using default parameters.");
  }
  ButtonPressed = false;
  int i = 0;
  while (true)
  {
    if ((i >= MIPSconfigData.StartupDelay) && (i >= 1)) break;
    if (ButtonPressed)
    {
      if (!MIPSconfigData.StartupHold) if (PBwasPressed) NormalStartup = false; // if false don't load from EEPROM on cards, use defaults!
      break;
    }
    PB_GREEN_ON;
    delay(500);
    WDT_Restart(WDT);
    PB_GREEN_OFF;
    delay(500);
    WDT_Restart(WDT);
    if (!MIPSconfigData.StartupHold) i++;
  }
  PB_GREEN_OFF;
}

// This function is called every 500 milli sec and its designed to drive the
// LEDs in the push button. It processes requests defined in:
// int    PBled;
// PBledStates  PBledMode;
void ProcessLED()
{
  static int i = 0;

  i ^= 1;   // Toggle i between 0 and 1
  //if(i) digitalWrite(LT,HIGH);     // This is the ethernet config line, can't do this is ethernet interface! removed 8/29/23
  //else digitalWrite(LT,LOW);
  if (LEDoverride)
  {
    if ((i == 1) && ((LEDstate & 8) != 0))
    {
      // Blinking and this is off time!
      PB_RED_OFF;
      PB_GREEN_OFF;
      PB_BLUE_OFF;
      return;
    }
    // Here if the LED override flag is set
    if ((LEDstate & 1) != 0) PB_RED_ON;
    else PB_RED_OFF;
    if ((LEDstate & 2) != 0) PB_BLUE_ON;
    else PB_BLUE_OFF;
    if ((LEDstate & 4) != 0) PB_GREEN_ON;
    else PB_GREEN_OFF;
    return;
  }
  // If in suspend mode let LED glow orange
  if (Suspend)
  {
    PB_RED_ON;
    PB_GREEN_ON;
    PB_BLUE_OFF;
    return;
  }
  if (PBledMode == NOTHING) return;
  if (PBledMode == ON)
  {
    PB_RED_OFF;
    PB_GREEN_OFF;
    PB_BLUE_OFF;
    digitalWrite(PBled, LOW);
  }
  if (PBledMode == OFF)
  {
    PB_RED_OFF;
    PB_GREEN_OFF;
    PB_BLUE_OFF;
    digitalWrite(PBled, HIGH);
  }
  if (PBledMode == FLASH)
  {
    if (digitalRead(PBled) == LOW)
    {
      PB_RED_OFF;
      PB_GREEN_OFF;
      PB_BLUE_OFF;
      digitalWrite(PBled, HIGH);
    }
    else digitalWrite(PBled, LOW);
  }
}

// I think this will intercept the system tic interrupt, test it!
// Tested but it is not getting called!
int sysTickHook(void)
{
  //  LDAChigh;
  //  LDAClow;
  //  return(0);
}

// this one was getting called until the init function ended...
void yield(void)
{
  //  LDAChigh;
  //  LDAClow;
}

//int sysTickHook(void) __attribute__ ((weak, alias("__mysysTickHook")));

void testISR()
{
  //   AtomicBlock< Atomic_RestoreState > a_Block;
  //   NVIC_SetPriority(TC1_IRQn, 15);
  //Timer1.setPriority(15);
  //for (int i = 0; i < 100; i++) delayMicroseconds(100);
}

void test(void)
{
  Timer1.attachInterrupt(testISR);
  Timer1.start(100000); // Calls every 100ms
}

void setup()
{
  // Set LT as output and turn off
  pinMode(LT, OUTPUT);
  digitalWrite(LT,LOW);
  // Flash the LED to indicate we are starting up
  for(int i=0;i<6;i++)
  {
    if(i&1) digitalWrite(LT,LOW);
    else digitalWrite(LT,HIGH);
    delay(125);
  }
  delay(250);
  pinMode(73, OUTPUT);  // TXL
  pinMode(72, OUTPUT);  // RXL
  digitalWrite(73, HIGH);  // TXL
  digitalWrite(72, HIGH);  // RXL
  // A watchdog timer reboot does not reset the USB interface for some reason, so
  // this code will detect if a watchdog timer reset happened and force a software
  // reset. Jan 18, 2019. Fixed this issue by making sure watchdog reset did a full reset
  uint32_t i = REG_RSTC_SR;  // Reads the boot flag
  i >>= 8;
  i &= 7;
  if(i==2) Software_Reset();
  // Start the real time clock and set default date and time, 1/1/2020 12:00:00
  digitalWrite(LT,HIGH);
  rtc.begin();
  rtc.setTime(12, 0, 0);
  rtc.setDate(1, 1, 2020);
  digitalWrite(LT,LOW);
  //
  analogReadResolution(12);
  Reset_IOpins();
  ClearDOshiftRegs();
  SPI.setClockDivider(21);
  pinMode(_sdcs, OUTPUT);
  digitalWrite(_sdcs, HIGH);
  delay(100);
  // Init the display and setup all the hardware
  pinMode(BACKLIGHT, INPUT_PULLUP);
  SetBackLight();
  // Pin 49 defines the display type, if grounded is the HX8347_DSP else ILI9340_DSP
  pinMode(49, INPUT_PULLUP);
  tft.SetDisplayType(ILI9340_DSP);
  tft.begin();
  tft.fillScreen(ILI9340_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
  tft.setTextSize(1);
  tft.println("Initializing....");

  SerialInit();
  delay(250);

  SPI.setClockDivider(21);
  SPI.setClockDivider(30);
  pinMode(_sdcs, OUTPUT);
  digitalWrite(_sdcs, HIGH);

  digitalWrite(_sdcs, LOW);
  delay(100);
  digitalWrite(_sdcs, HIGH);

  delay(100);
  SD.begin(_sdcs);
  WDT_Restart(WDT);
  delay(100);
  if (!SD.begin(_sdcs))
  {
    tft.println("SD disk failed to initalize!");
    delay(1000);
    SDcardPresent = false;
  }
  else
  {
    SDcardPresent = true;
    //LOADparms("default.cfg"); // Load the defaults if the file is present
  }
  LOADparms("default.cfg");   // Load the defaults if the file is present
  SPI.setClockDivider(11);    // If SD card init fails it will slow down the SPI interface
  SetBackLight();
  DisplayIntensity();
  MIPSsystemLoop();
  if (bmpDraw(MIPSconfigData.BootImage, 0, 0))
  {
    DisableDisplay = true;
    tft.disableDisplay(DisableDisplay);
  }
  else MIPSconfigData.BootImage[0] = 0;
  // Setup the hardware pin directions. Note setting pinMode to output
  // will drive the output pin high.
  Init_IOpins();
  // Clear the digital IO
  ClearDOshiftRegs();
  PulseLDAC;
  // Startup the encoder
  enc.start(PB_PHASE_A, PB_PHASE_B, PB);
  enc.setValue(&encValue);
  enc.attachInterruptChange(encChanged);
  enc.attachInterruptPushButton(encPB);
  // Start the SPI interface
  SPI.begin(SPI_CS);
  SPI.setClockDivider(SPI_CS, 8);  // Default divider is 21 and equals 4 MHz
  // Increase speed for better performance of tables
  // Start the TWI interface
  TWI_RESET();

  Wire.begin();
  Wire.setClock(WireDefaultSpeed);
  //  Timer3.attachInterrupt(RealTimeISR);
  //  Timer3.start(100000); // Calls every 100ms
  // Initial splash screen
  Signon();
  ButtonPressed = false;
  ButtonRotated = false;
  // Set ActiveDialog to not NULL, this will stop any module from displaying.
  // This happen only when the startup delay is set to 0
  if(MIPSconfigData.StartupDelay == 0) ActiveDialog = (DialogBox *)~NULL;
  // Init the baords
  ScanHardware();
  DIO_init();
  if(ActiveDialog == (DialogBox *)~NULL) MenuDisplay(&MainMenu);
  // Configure Threads
  MIPSsystemThread.setName("System");
  MIPSsystemThread.onRun(MIPSsystemLoop);
  MIPSsystemThread.setInterval(10);
  LEDThread.setName("LED");
  LEDThread.onRun(ProcessLED);
  LEDThread.setInterval(500);
  // Add threads to the controller
  control.add(&MIPSsystemThread);
  control.add(&LEDThread);
  // If a startup masco is defined, play it now
  if (strlen(MIPSconfigData.StartupMacro) > 0)
  {
    MacroPlay(MIPSconfigData.StartupMacro, true);
  }
  Ethernet_init();
  watchdogEnable();
  LogMessage("System starting up!");
}

// This task is called every 10 milliseconds and handels a number of tasks.
void MIPSsystemLoop(void)
{
  char cbuf[20];

  if ((BrightTime + 30000) < millis()) SetBackLight();
  float MaxVoltage;
  // Process any serial commands
  // Not sure why this is here??
  //  while(ProcessCommand()==0);  // Process until flag that there is nothing to do, removed Dec 2, 2017
  // Determine the maximum output voltage and set the proper button color
  MaxVoltage = MaxRFVoltage;
  if (MaxDCbiasVoltage > MaxVoltage) MaxVoltage = MaxDCbiasVoltage;
  if (MaxTwaveVoltage > MaxVoltage) MaxVoltage = MaxTwaveVoltage;
  if (MaxFAIMSVoltage > MaxVoltage) MaxVoltage = MaxFAIMSVoltage;
  if (MaxESIvoltage > MaxVoltage) MaxVoltage = MaxESIvoltage;
#if HVPScode
  if (MaxHVvoltage > MaxVoltage) MaxVoltage = MaxHVvoltage;
#endif
  // Test output voltages and update the button LEDs to provide warning to user
  // Blue LED on if voltgaes are enabled and above 0
  if (MaxVoltage == 0) {
    PB_OFF(PB_RED);
  }
  if (MaxVoltage >  0) {
    PB_ON(PB_BLUE);
  }
  if (MaxVoltage > 50) {
    PB_ON(PB_RED);
  }
  if (MaxVoltage > 75) {
    PB_FLASH(PB_RED);
  }
  // Test Vin, if power is off then shut down the hardware control lines and print
  // and warding on the display. Process serial commands and stay in this loop until
  // power is reapplied. Reset when power is reapplied
#ifdef TestMode
  return;
#endif
  if (!AcquireADC()) return;
  if (ReadVin() < 10.0)
  {
    tft.disableDisplay(false);
    // Set all control lines to input, this will keep the systems from sourcing power to
    // the modules through driven outputs.
    FilamentShutdown();
    ClearDOshiftRegs();
    Reset_IOpins();
    // Display a message on the screen that MIPS power is off.
    tft.fillScreen(ILI9340_BLACK);
    tft.setRotation(1);
    if (strlen(MIPSconfigData.BootImage) <= 0)
    {
      tft.setCursor(0, 0);
      tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
      tft.setTextSize(2);
      tft.println("");
      tft.println("MIPS power is off!");
      tft.println("USB powering controller!");
      tft.println("Apply power to operate...");
    }
    // Dim the display, we could be like this for a long time!
    if (MIPSconfigData.EnetUseTWI) analogWrite(BACKLIGHT, 0);
    else analogWrite(BACKLIGHT, 400);
    //    bmpDraw(MIPSconfigData.BootImage, 0, 0);
    // Wait for power to appear and then reset the system.
    while (ReadVin() < 10.0)
    {
      // Look for the POWER command to start up!
      ReadAllSerial();
      if (RB_Commands(&RB))
      {
        // Read a line from the ring buffer
        GetLine(&RB, cbuf, 20);
        if (strcmp(cbuf, "POWER") == 0) PowerControl();
        if (strcmp(cbuf, "ISPWR") == 0) serial->println("I'm off!");
      }
      WDT_Restart(WDT);
    }
    ReleaseADC();
    Software_Reset();
  }
  ReleaseADC();
}

// This function will scan through all the posible addresses for the EEPROMs that are loacted on
// the IO cards. If a EEPROM is found its read to determine the type of card. The proper init
// function is then called. If the signature is not valid then nothing is done.
// The following are valid addresses for the EEPROM:
// 0x50
// 0x52
// 0x54
// 0x56
// This same set of addresses appears for board select A and B
void ScanHardware(void)
{
  uint8_t addr;
  char    signature[100];

#ifdef TestFilament
  Filament_init(0);
#endif
#ifdef TestTwave
  Twave_init(0, 0x52);
  Twave_init(1, 0x52);
#endif
  // Loop through all the addresses looking for signatures
  for (int i = 0; i < NumModAdd; i++)
  {
    addr = ModuleAddresses[i];
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "DAC") == 0) DAC_init(0, addr);
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(0, addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(0, addr);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(0, addr);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(0, addr);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(0, addr);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(0, addr);
      if (strcmp(&signature[2], "FPGA") == 0) FPGA_init(0, addr);
#if DMSDMSMB
      if (strcmp(&signature[2], "CVBIAS") == 0) CVBIAS_init(0, addr);
      if (strcmp(&signature[2], "WAVEFORMS") == 0) WAVEFORMS_init(0, addr);
#endif
#if HOFAIMScode
      if (strcmp(&signature[2], "HOFAIMS") == 0) HOFAIMS_init(0, addr);
#endif
      if (strcmp(&signature[2], "RFamp") == 0) RFA_init(0, addr);
#if FAIMSFBcode
      if (strcmp(&signature[2], "FAIMSfb") == 0) FAIMSfb_init(0, addr);
#endif
#if HVPScode
      if (strcmp(&signature[2], "HVPS") == 0) HVPS_init(0, addr);
#endif
#if DCBswitchCode
      if (strcmp(&signature[2], "DCBswitch") == 0) DCBswitch_init(0, addr);
#endif
    }
    if (!MIPSconfigData.UseBRDSEL) continue;
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "DAC") == 0) DAC_init(1, addr);
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(1, addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(1, addr);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(1, addr);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(1, addr);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(1, addr);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(1, addr);
      if (strcmp(&signature[2], "FPGA") == 0) FPGA_init(1, addr);
#if DMSDMSMB
      if (strcmp(&signature[2], "CVBIAS") == 0) CVBIAS_init(0, addr);
      if (strcmp(&signature[2], "WAVEFORMS") == 0) WAVEFORMS_init(0, addr);
#endif
#if HOFAIMScode
      if (strcmp(&signature[2], "HOFAIMS") == 0) HOFAIMS_init(1, addr);
#endif
      if (strcmp(&signature[2], "RFamp") == 0) RFA_init(1, addr);
#if FAIMSFBcode
      if (strcmp(&signature[2], "FAIMSfb") == 0) FAIMSfb_init(1, addr);
#endif
#if HVPScode
      if (strcmp(&signature[2], "HVPS") == 0) HVPS_init(1, addr);
#endif
#if DCBswitchCode
      if (strcmp(&signature[2], "DCBswitch") == 0) DCBswitch_init(1, addr);
#endif
    }
  }
  // Now look for the FAIMS board. This is done last because field driven FAIMS mode is selected if
  // a DCbias module is found.
#if FAIMScode
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "FAIMS") == 0) FAIMS_init(0);
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "FAIMS") == 0) FAIMS_init(1);
    }
  }
#endif
  WiFi_init();
  // The last thig we do is look for the analog input option
  if (MIPSconfigData.UseAnalog) Analog_init();
}

// This function allows using a host computer to navigate the MIPS UI,
// This feature has to first be enabled using a host command to enable.
bool SerialNavigation(char c)
{
  if (!EnableSerialNavigation) return false;
  switch ((int)c)
  {
    case 9:
      enc.SetPB();
      return true;
    case 28:
      enc.SetChange(1);
      enc.SetRotate();
      return true;
    case 29:
      enc.SetChange(10);
      enc.SetRotate();
      return true;
    case 30:
      enc.SetChange(-1);
      enc.SetRotate();
      return true;
    case 31:
      enc.SetChange(-10);
      enc.SetRotate();
      return true;
    default:
      return false;
      break;
  }
  return false;
}

void SerialWD(void)
{
  static ulong lastcharT;

  if (SerialUSB.available() > 0)
  {
    lastcharT = millis();
    return;
  }
  if (SerialWatchDog <= 0) return;
  if (((millis() - lastcharT) / (ulong)1000) < SerialWatchDog) return;
  // If here rest the comm power
  SerialPortReset();
  lastcharT = millis();
}

void ReadAllSerial(void)
{
  WDT_Restart(WDT);
  SerialWD();
  // Put serial received characters in the input ring buffer
  //if(SerialUSB.dtr()) while (SerialUSB.available() > 0)  // Mike's app does not set DTR! maybe make this an options and default to not needed
  while (SerialUSB.available() > 0)
  {
    ResetFilamentSerialWD();
    serial = &SerialUSB;
    char c = SerialUSB.read();
    if (Serial1Echo)
    {
      Serial1.write(c);
      Serial1.flush();
    }
    if (!SerialNavigation(c)) PutCh(c);
  }
#ifdef EnableSerial
  if ((!MIPSconfigData.UseWiFi) || (wifidata.SerialPort != 0))
  {
    while (Serial.available() > 0)
    {
      ResetFilamentSerialWD();
      serial = &Serial;
      PutCh(Serial.read());
    }
  }
#endif
/* This code caused the ethernet interface to fail, not sure I understand why its here, commented 8/29/23
  while (WiFiSerial->available() > 0)
  {
    ResetFilamentSerialWD();
    serial = WiFiSerial;
    PutCh(WiFiSerial->read());
  }
*/
  ProcessEthernet();
}

void (*onProcessSerial)(void) = NULL;

void attachProcessSerial(void (*func)(void))
{
   onProcessSerial = func;
}

// This function process all the serial IO and commands
void ProcessSerial(void)
{
  ReadAllSerial();
  if(onProcessSerial != NULL) onProcessSerial();
  // If there is a command in the input ring buffer, process it!
  while (RB_Commands(&RB) > 0) // Process until flag that there is nothing to do
  {
    while (ProcessCommand() == 0); // WDT_Restart(WDT);
  }
  SerialUSB.flush();     // Added 9/2/2017
  DIOopsReport();
  ReportADCchange();
  LevelDetChangeReport();
}

void USBportTest(void)
{
  static bool wasConnected = false;
  static int  unusableCount = 0;

  if (SerialUSB.dtr()) wasConnected = true;
  if (SerialUSB.dtr()) digitalWrite(72, LOW);
  else digitalWrite(72, HIGH);
  if (!Is_otg_clock_usable()) unusableCount++;
  else unusableCount = 0;
  if (!SerialUSB.dtr() && wasConnected)
  {
    wasConnected = false;
    SerialPortReset();
  }
  if (isUSBerror() || (unusableCount > 500))
  {
    clearUSBerror();
    unusableCount = 0;
    SerialPortReset();
  }
}

// Main processing loop
void loop()
{
  static bool DisableDisplayStatus = false;
  static uint32_t lastTouched = millis();

  USBportTest();
  if ((!DisableDisplay) && (DisableDisplayStatus))
  {
    // Here is the display was disabled and it now going to be enabled.
    // Clear the display and reprint the current menu or dialog.
    tft.disableDisplay(DisableDisplay);
    tft.fillScreen(ILI9340_BLACK);
    tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
    tft.setRotation(1);
    tft.setTextSize(2);
    if (ActiveMenu != NULL) MenuDisplay(ActiveMenu);
    if (ActiveDialog != NULL) DialogBoxDisplay(ActiveDialog);
  }
  DisableDisplayStatus = DisableDisplay;
  tft.disableDisplay(DisableDisplay);
  WDT_Restart(WDT);
  // run ThreadController
  // this will check every thread inside ThreadController,
  // if it should run. If yes, he will run it;
  if (!Suspend) control.run();
  else  ProcessLED();
  // Return to main menu if the startup delay is 0 and no button activity for
  // 1 min
  if(MIPSconfigData.StartupDelay == 0)
  {
    if((ActiveDialog != NULL) && (ActiveMenu != &MainMenu))
    {
      if((millis() - lastTouched) > (1000 * 60)) MenuDisplay(&MainMenu);
    }
  }
  // Process any encoder event
  if (ButtonRotated)
  {
    ButtonRotated = false;
    if (ActiveMenu != NULL) MenuProcessChange(ActiveMenu, encValue);
    else if (ActiveDialog != NULL) DialogBoxProcessChange(ActiveDialog, encValue);
    encValue = 0;
    lastTouched = millis();
  }
  if (ButtonPressed)
  {
    Suspend = false;
    delay(10);
    ButtonPressed = false;
    encValue = 0;
    if (ActiveMenu != NULL) MenuButtonPress(ActiveMenu);
    else if (ActiveDialog != NULL) DialogButtonPress(ActiveDialog);
    DismissMessageIfButton();
    lastTouched = millis();
  }
  ProcessSerial();
  // Interlock processing.
  // If the user has defined an interlock input then monitor this channel.
  // Arm when it goes high, if armed and it goes lown trip the power supplies.
  // If the user has defined an interlock output then drive the signal high
  // when the power supplies are on.
  static bool initInterlock = true;
  static int  WasPowerON = digitalRead(PWR_ON);  // LOW = on
  static bool InterlockArmed = false;
  if ((MIPSconfigData.InterlockIn >= 'Q') && (MIPSconfigData.InterlockIn <= 'X'))
  {
    if (InterlockArmed)
    {
      if (ReadInput(MIPSconfigData.InterlockIn) == LOW)
      {
        // Trip power supply and disarm interlock
        MIPSconfigData.PowerEnable = false;
        digitalWrite(PWR_ON, HIGH);
        DisplayMessageButtonDismiss("Interlock Trip!");
        InterlockArmed = false;
      }
    }
    else if (ReadInput(MIPSconfigData.InterlockIn) == HIGH) InterlockArmed = true;
  }
  if ((initInterlock) || (WasPowerON != digitalRead(PWR_ON)))
  {
    initInterlock = false;
    if ((MIPSconfigData.InterlockOut >= 'A') && (MIPSconfigData.InterlockOut <= 'P'))
    {
      // If power supply is on then output interlock high
      if (NumberOfDCChannels > 0)
      {
        WasPowerON = digitalRead(PWR_ON);
        if (digitalRead(PWR_ON) == LOW) SetOutput(MIPSconfigData.InterlockOut, HIGH);
        else SetOutput(MIPSconfigData.InterlockOut, LOW);
      }
      else SetOutput(MIPSconfigData.InterlockOut, HIGH);
      UpdateDigitialOutputArray();
    }
  }
}


// Host command processing function.

// Save parameters to default.cfg file on SD card
int SAVEparms(char *filename)
{
  File file;

  MIPSconfigData.DisableDisplay = DisableDisplay;
  MIPSconfigData.signature = 0xA55AE99E;
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    // Save to FLASH as well in case SD card fails or is not present
    dueFlashStorage.writeAbs((uint32_t)NonVolStorage, (byte *)&MIPSconfigData, sizeof(MIPSconfigStruct));
  }
  // Test SD present flag, exit and NAK if no card or it failed to init
  if (!SDcardPresent) return (ERR_NOSDCARD);
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(filename);
  // Open file and write config structure to disk
  if (!(file = SD.open(filename, FILE_WRITE))) return (ERR_CANTCREATEFILE);
  MIPSconfigData.Size = sizeof(MIPSconfigStruct);
  file.write((byte *)&MIPSconfigData, sizeof(MIPSconfigStruct));
  file.close();
  return (0);
}

void SAVEparms(void)
{
  int iStat;

  if ((iStat = SAVEparms("default.cfg")) == 0)
  {
    SendACK;
    return;
  }
  SetErrorCode(iStat);
  SendNAK;
  return;
}

// This function load the MIPS config structure from the filename passed.
// The size of the struct on disk is used, this allows the struct to grow
// and new additional (at the bottom of the struct) will use the default
// values.
// Returns true if all went well!
bool LOADparms(char *filename)
{
  File file;
  MIPSconfigStruct MCS;
  int  i, fVal;
  byte *b;

  while (1)
  {
    // Test SD present flag
    if (!SDcardPresent) break;
    SD.begin(_sdcs);
    // Open the file
    if (!(file = SD.open(filename, FILE_READ))) break;
    // read the data
    b = (byte *)&MCS;
    for (i = 0; i < sizeof(MIPSconfigStruct); i++)
    {
      if ((fVal = file.read()) == -1) break;
      b[i] = fVal;
    }
    file.close();
    // Copy to MIPS config struct
    if (MCS.signature == 0xA55AE99E)
    {
      memcpy(&MIPSconfigData, &MCS, MCS.Size);
      DisableDisplay = MIPSconfigData.DisableDisplay;
      return true;
    }
    break;
  }
  // If here then the SD card read failed so try loading from FLASH
  b = (byte *)&MCS;
  for (i = 0; i < sizeof(MIPSconfigStruct); i++)
  {
    b[i] = dueFlashStorage.readAbs(((uint32_t)NonVolStorage) + i);
  }
  // Check signature, if correct then update
  if (MCS.signature == 0xA55AE99E)
  {
    memcpy(&MIPSconfigData, &MCS, MCS.Size);
    DisableDisplay = MIPSconfigData.DisableDisplay;
    return true;
  }
  return false;
}

void PowerControl(void)
{
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH);
  delay(1000);
  digitalWrite(POWER, LOW);
}

// ADC channel 0 is 24VDC supply, rev 3.1 power module
// ADC channel 1 is 12VDC supply, rev 3.1 power module
// ADC channel 3 is input current, rev 3.2 power module
void ReportSupplies(void)
{
  SendACKonly;
  if (SerialMute) return;
  serial->print("24 volt supply: ");
  serial->println(((float)analogRead(0) / 4095.0) * 3.3 * 12.0);
  serial->print("12 volt supply: ");
  serial->println(((float)analogRead(1) / 4095.0) * 3.3 * 6.3);
  serial->print("Input current, Amps: ");
  serial->println(((float)analogRead(3) / 4095.0) * 3.3 / 0.2);
}
