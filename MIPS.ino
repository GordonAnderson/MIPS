//
// MIPS
//
// This sketch controls the MIPS system. This MIPS firmware uses the thread controller developed by
// Seidel Gomes, for details, go to https://github.com/ivanseidel/ArduinoThread. This is a round robin
// tasking system. Basically every module in MIPS is its own task. The modules are discoved on power up
// and there init functions are called. The modules then insert there menu option into the main system
// menu. There are a couple of additional threads defined in this main loop, one for the LED light 
// flashing and a system thread that monitors power.
//
// The code in the modules does not block unless it is desired to stop all other tasks. Very little is
// done in ISRs with the exception of the table execution for pulse sequence generation.
//
// MIPS used ADCs and DACs with TWI and SPI interfaces. The TFT display is also SPI as if the SD card
// interface.
//
// Host computer communications is supported through both the native USB and serial ports. On power up
// it defaults to native USB and if serial traffic is seen it switches to the active port automatically.
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
//            #define PWM_FREQUENCY		50000
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
//        2.) If any output voltgae exceeds 50 volts the red LED will be on to provide warning.
//        3.) If any output exceeds 100 volts the red LED will flash.
//
// To do list: --------------------------------------------------------------------------------------
//   1.) MIPS configuration menu and data structure, things that are needed
//        1.) MIPS config menu options:
//            - MIPS box name (this will require a new UI construct to do)
//            - Save all, saves all the boards and the MIPS config data
//   2.) Add macro command to list a table out the serial port
//   3.) Commands to add
//        - Name, get and set MIPS box name
//   4.) Add three point calibration to FAIMS high voltage levels 
//   5.) Make the USB id text indicate GAACE
//   6.) Add the FAIMS serial commands
//   7.) Add watchdog timer to system. Read this page for important information: http://forum.arduino.cc/index.php?topic=314647.0
//       will need to modify the arduino source code for this to work.
//   8.) Change the PWM to 12 bit from 8 bit in RFdriver and the FAIMS driver. This requires editing variant.h
//       in the arduino source code.
//
//     
// Revision history and notes *********************************************************************************
//
//    V1.0, January 13, 2015
//      1.) Orginal release
//    V1.1, January 25, 2015
//      1.) Cleaned up the code for the external trigger and tested. I added a ISR to disable the external trigger after it happens 
//          so the system will not be re-triggerable
//      2.) I updated the DCbias board trip logic so it will turn off the outputs on the +- 50 volt board
//      3.) Fixed a minor display bug in the case where two DC bias board are in the system
//    V1.2, January 28, 2015
//      1.) Fixed a table bug that prevented table nesting in the case where
//          to channel entries are present just a nested table. Below is an example of a table 
//          that did not work:
//              STBLDAT;0:[1:2,163:[2:2,0:3:0,326:3:300,815:3:0],3260:3:300,3423:3:0];
//    V1.3, January 30, 2015
//      1.) Fixed a number of bugs that prevented two DC bias modules from working in one MIPS system.
//    V1.4, February 3, 2015
//      1.) Added a delay to hold all DC bias DAC at zero until power is up and stable for 1 sec. The offset driver was 
//          latching up on power up when set at 250 volts.
//      2.) Updated the power on voltage test to average the voltage 100 times. It has a looping problem sometimes that
//          I have not solved.
//      3.) Fix a table bug when 16 DC bias channels are present, this was a board select but that is now fixed.
//    V1.5, February 9, 2015
//      1.) Added support and commands for multiple tables and table advanceing.
//      2.) Added initial FAIMS module support.
//      3.) Fixed a bug in the serial ring buffer, I was using unsigned chars for ringer buffer pointers!
//    V1.6, March 18, 2015
//      1.) Added arc detection to FAIMS
//    V1.7, March 26, 2015
//      1.) Fixed bugs in the DC cv scanning on FAIMS module.
//      2.) Added the ESI control module code.
//    V1.8, April 3, 2015
//      1.) Fixed a couple ESI module bugs.
//      2.) Fixed table system bugs.
//          a.) Can not set a count of 0, is seen it will be incremented to 1.
//          b.) Tried to add a dialog box when in the table mode but this interfered with the SPI
//              interface needed in table processing, code was removed and it can be seen in tabledialog.cpp
//      3.) Added table voltage value edit function.
//      4.) Added serial support for bluetooth module. The system will now automatically switch back and forth between
//          serialUSB and serial. The bluetooth interface has problems with long strings, likely needs flow control.
//          Adding 3mSec of delay between transmitted characters at 9600 baud fixes the problem.
//      5.) Added the macro menu to MIPS config. This allows selecting and playing a macro and defining a 
//          macro to play at startup
//      6.) Updated the Twave system to support the rev 2 module
//    V1.9, May 29, 2015
//      1.) Fixed the SendACK and SendNAK macros to end with \n\r instead of just \n, this is consistant with all other 
//          serial responces and AMPS.
//      2.) Redesiged FAIMS to support field driven FAIMS option, not tested yet
//      3.) Updated the tack scheduler code so the execution time of the process is not added to the reoccurance time.
//    V1.10, June 6, 2015
//      1.) Fixed the bug with table commands containing 0 values
//      2.) Fixed a few minor serial commands
//      3.) Fixed disk io problems that happen with DC bias cards are present
//      4.) Upgraded to Arduino IDE version 1.6.4
//      5.) There is a problem with the SD interface when using multiple SPI CS options on the DUE. The only way I can make
//          the macro recording and work is to only record the macro in record mode and not execute the commands during macro
//          recording. The macro functions were upgraded in this version.
//    V1.11, June 9, 2015
//      1.) Fixed the FAIMS timer problems and on time problems
//      2.) Debugged the field driven FAIMS supplies
//    V1.12, June 18, 2015
//      1.) Added all the serial commands for the TWAVE module
//      3.) Fixed the table issue with a count of zero, I still think there are a few issues to resolve
//      2.) This verions was programmed into Matt Bush's system and Jim Bruce's system
//    V1.13, July 1, 2015
//      1.) Fixed FAIMS timing bugs and added the support for temp and pressure sensor
//      2.) Added the serial port reset function
//    V1.14, July 7, 2015
//      1.) Fixed a frequency bug with Twave rev 2 and added internal clock mode.
//    V1.15, July 20, 2015
//      1.) Added the analog input module based on the Adafruit ADS1115 boards, 8 channels max.
//      2.) Updated the thread controller to support names.
//      3.) Added the THREADS command to display the running threads and there last run times.
//      4.) Updated the RFdriver and DCbias modules to improve the run time and screen updates.
//      5.) Add the following new features to the RFdriver:
//          a.) Gate input selection and logic level
//          b.) Automatic control mode
//          c.) Created DIhandler class to support a.)
//      6.) Added the 750 volt DCbias board option
//    V1.16, July 26, 2015
//      1.) Added support for the 2 channel Filament module.
//      2.) Added watchdog timer support
//      3.) updated PWM to 12 bit operation
//   V1.17, August 25, 2015
//      1.) Fixed a number of minor bugs in the filament driver code
//   V1.18, August 26, 2015
//      1.) Fixed CV scan parameter limit bug in FAIMS code
//   V1.19, September 6, 2015
//      1.) Update The FAIMS module:
//          -- Added output level locking
//          -- Updated the display processing
//          -- Added commands to support calibration
//          -- Enabled the pressure and temp compensation
//   V1.20, October 13, 2015
//      1.) Added delay in the sequence loading function for Twave. The function failed at low frequencies.
//   V1.21, November 6, 2015
//      1.) Added support for ARB module
//   V1.22, December 19,2015
//      1.) Added WiFi support with ESP8266 module from Adafruit
//      2.) Added support for Rev 3 of Twave module
//   V1.23, December 31, 2015
//      1.) Updated WiFi interface
//      2.) Fixes Twave serial command errors
//      3.) Updated the test mode to support Twave
//      4.) Allow and valid EEPROM address for the Twave board, this ability will be migrated to all modules
//   V1.24, January 3, 2016
//      1.) Fixed a number of Twave module bugs and added support for two Twave modules
//      2.) Added the GNAME and SNAME commands.
//   V1.25, January 4, 2016
//      1.) Removed delay from the ISR function in Twave, this was causing it to crash.
//   V1.26. January 6, 2016
//      1.) Fixed a table bug that was allowing a table to be retriggered.
//      2.) Added a ONCe option to the SMOD command so that a table will only play one time them the table mode will exit
//   V1.27. January 22, 2016
//      1.) Added the cycling feaature to the Filament control module.
//      2.) Started the Compressor function for Twave, not yet finished or tested.
//
//
// Gordon Anderson
// GAA Custom Engineering,LLC
// gaa@owt.com
// 509.588.5410
//
#include "SD.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>
#include "Encoder.h"
#include <stdarg.h>
#include "Menu.h"
#include "Dialog.h"
#include "Hardware.h"
#include "Serial.h"
#include <Wire.h>
#include "AtomicBlock.h"
#include "DIO.h"
#include "Twave.h"
#include "DCbias.h"
#include "FAIMS.h"
#include "ESI.h"
#include "ARB.h"
#include "Variants.h"
#include "Errors.h"
#include "Serial.h"
#include <Thread.h>
#include <ThreadController.h>
#include <MIPStimer.h>
#include <DIhandler.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

#define UseWDT

// Define my names for the native USB driver
#define  USB_PRODUCT
#define  USB_PRODUCT      "MIPS, native USB"
#define  USB_MANUFACTURER "GAA Custom Engineering, LLC"

// These values are defaulted to 1000 in Variant.h, The RFdriver needs this PWM
// frequency set to 50KHz for its adjustable power supply. Not sure if this overrides
// the values in the .h file?
#define PWM_FREQUENCY	    50000
#define TC_FREQUENCY      50000

bool NormalStartup = true;
bool SDcardPresent = false;

char Version[] = "Version 1.27, January 22, 2016";

// ThreadController that will controll all threads
ThreadController control = ThreadController();
//MIPS Threads
Thread MIPSsystemThread = Thread();
Thread LEDThread        = Thread();

Encoder enc;
int encValue = 0;
bool ButtonPressed = false;
bool ButtonRotated = false;

extern Menu HardwareTestMenu;
extern DialogBox TwaveDialog;
extern DialogBox MIPSconfig;
extern DialogBox ModuleConfig;
extern DialogBox MacroOptions;

#define MaxMainMenuEntries  16

void SerialPortReset(void);

char BoardAddress[20] = "";
char BoardName[20] = "";

// MIPS main menu
MenuEntry MainMenuEntries[MaxMainMenuEntries] = {
  {" MIPS Configuration", M_DIALOG, 0, 0, 0, NULL, &MIPSconfig, NULL, NULL},
  {" Reset serial port",  M_FUNCTION, 0, 0, 0, NULL, NULL, SerialPortReset, NULL},
  {""}};
  
Menu MainMenu = {
  {"MIPS main menu",ILI9340_BLACK,ILI9340_WHITE,2,0,0,300,220,B_DOUBLE,12},
  M_SCROLLING,0,0,MainMenuEntries};
  
// MIPS configuration dialog box
DialogBoxEntry MIPSconfigEntries[] = {
  {" Controller rev", 0, 1, D_INT,    1, 10, 1, 18, false, "%2d", &MIPSconfigData.Rev, NULL, NULL},
  {" Config modules", 0, 2, D_DIALOG, 0, 0, 0, 0, false, NULL, &ModuleConfig, SetupModuleConfig, NULL},
  {" Startup delay" , 0, 3, D_INT,    1, 100, 1, 17, false, "%3d", &MIPSconfigData.StartupDelay, NULL, NULL},
  {" Startup hold"  , 0, 4, D_YESNO,  0, 1, 1, 18, false, NULL, &MIPSconfigData.StartupHold, NULL, NULL},
  {" DCbias supply" , 0, 5, D_ONOFF,  0, 1, 1, 18, false, NULL, &MIPSconfigData.PowerEnable, NULL, NULL},
  {" DCbias trip, %FS"  , 0, 6, D_FLOAT,  0, 100, 0.1, 18, false, "%5.1f", &MIPSconfigData.VerrorThreshold, NULL, NULL},
  {" Reboot", 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, Software_Reset, NULL},
  {" Macro options", 0, 8, D_DIALOG, 0, 0, 0, 0, false, NULL, &MacroOptions, SetupMacroOptions, NULL},  
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveMIPSSettings, NULL},
  {" Restore settings"   , 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreMIPSSettings, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox MIPSconfig = {
  {"MIPS Configuration",ILI9340_BLACK,ILI9340_WHITE,2,0,0,300,220,B_DOUBLE,12},
  M_SCROLLING,0,0,MIPSconfigEntries};
  
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
  {"Module Configuration",ILI9340_BLACK,ILI9340_WHITE,2,0,0,300,220,B_DOUBLE,12},
  M_SCROLLING,0,0,ModuleConfigEntries};
  
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
  {"Macro options",ILI9340_BLACK,ILI9340_WHITE,2,0,0,300,220,B_DOUBLE,12},
  M_SCROLLING,0,0,MacroEntries};
  
// This function is called before the macro menu loads.
void SetupMacroOptions(void)
{
  MacroNameList = MacroBuildList("NONE");
  MacroEntries[0].fmt = MacroNameList;
  MacroEntries[1].fmt = MacroNameList;
}

void SerialPortReset(void)
{
  SerialUSB.flush();
  SerialUSB.end();
  SerialInit();
}

// Called after macro is selected to play
void UIplayMacro(void)
{
  MacroPlay(PlayMacro,true);
}

// This function is called before the module configuration menu is loaded.
// This function fills the initial board address and name variables.
void SetupModuleConfig(void)
{
  sprintf(BoardAddress,"%s", GetEntry(BoardAddressList,1));
  sprintf(BoardName,"%s", GetEntry(BoardVariantsNames,1));
}

// This function is called to format a board's EEPROM.
void BoardFormat(void)
{
  void *BoardConfigData;
  uint8_t addr;
  uint8_t brd;
  int CurrentBoard;
  
  // Scan the board number and board address from the BoardAddress string
  // Two board posibilities, A and B, and 4 addresses, 50,52,54, and 56.
  if(BoardAddress[0] == 'A') brd = 0;
  if(BoardAddress[0] == 'B') brd = 1;
  if(strcmp(&BoardAddress[2], "0x50") == 0) addr = 0x50;
  if(strcmp(&BoardAddress[2], "0x52") == 0) addr = 0x52;
  if(strcmp(&BoardAddress[2], "0x54") == 0) addr = 0x54;
  if(strcmp(&BoardAddress[2], "0x56") == 0) addr = 0x56;
  // Setup a pointer to the selected default data structure and get its size
  BoardConfigData = BoardVariants[FindInList(BoardVariantsNames,BoardName)-1];
  // Write it to the selected board
  CurrentBoard = digitalRead(BRDSEL);  // Save current board select
  if(brd == 0) ENA_BRD_A;              // Select board
  else ENA_BRD_B;
  // Write to EEPROM
  if(WriteEEPROM(BoardConfigData,addr,0,*(int16_t *)BoardConfigData)==0) DisplayMessage("Module formatted!",2000);
  else DisplayMessage("Unable to format!",2000);
  digitalWrite(BRDSEL,CurrentBoard);   // Restore board select
}

void SaveMIPSSettings(void)
{
  if(SAVEparms("default.cfg") == 0) DisplayMessage("Parameters Saved!",2000);
  else DisplayMessage("Error saving!",2000);
}

void RestoreMIPSSettings(void)
{
  if(LOADparms("default.cfg")) DisplayMessage("Parameters Restored!",2000);
  else DisplayMessage("Unable to Restore!",2000);
}

// This function will add the menu entry to the MIPS main menu.
// This is used by modules as they init to add them selfs to
// the system.
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

File myFile;

void encChanged(void)
{
  ButtonRotated = true;
}

void encPB(void)
{
  ButtonPressed = true;
}

// Watchdog timer setup routine, this function will enable the watchdog timer.
// call WDT_Restart(WDT) at least every 2.5 sec to stop an automatic reset.
// the setup routine runs once when you press reset:
#ifdef UseWDT
//double timeout = 5.0;  // number of seconds for timeout
double timeout = 8.0;  // number of seconds for timeout
int timeout2 = 0;
void WDT_Setup (){ 

  timeout2 =(int)( timeout * 227); 
  
  timeout2 = 0x0fff2000 + timeout2;
  WDT_Enable(WDT,timeout2);  
  
  // number of loops:
  // 0x0fff2000 0
  // 0x0fff200f 231
  // 0x0fff2fff 2981
}  
#endif      

void Signon(void)
{
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
  tft.println("GAA Custom Engineering, LLC\n\n");
  tft.setTextSize(2);
  tft.setTextColor(ILI9340_GREEN, ILI9340_BLACK);
  tft.println("\nSystem initialized!");
  if(MIPSconfigData.StartupHold)
  {
    tft.println("\nPress knob to load");
    tft.println("parameters and startup.");
  }
  else
  {
    tft.println("\nPress knob to startup");
    tft.println("using default parameters.");
  }
  ButtonPressed = false;
  int i = 0;
  while(true)
  {
    if(i >= MIPSconfigData.StartupDelay) break;
    if (ButtonPressed) 
    {
      if(!MIPSconfigData.StartupHold) NormalStartup = false; // if false don't load from EEPROM on cards, use defaults!
      break;
    }
    PB_GREEN_ON;
    delay(500);
    WDT_Restart(WDT);
    PB_GREEN_OFF;
    delay(500);
    WDT_Restart(WDT);
    if(!MIPSconfigData.StartupHold) i++;
  }
  PB_GREEN_OFF;
}

// This function is called every 500 milli sec and its designed to drive the
// LEDs in the push button. It processes requests defined in:
// int    PBled;
// PBledStates  PBledMode;
void ProcessLED()
{
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

// This function is called 10 times each second and is used
// for real time control and monitoring
void RealTimeISR()
{
}

void setup()
{
  Reset_IOpins();
  Serial.begin(115200);
  Stream *stm = &Serial;
//  stm->println("MIPS!");
  
  // Init the display and setup all the hardware
  tft.begin();
  tft.fillScreen(ILI9340_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
  tft.setTextSize(1);
  tft.println("Initializing....");

  SerialInit();
  
  delay(250);

  MIPSsystemLoop();

  if (!SD.begin(_sdcs))
  {
    tft.println("SD disk failed to initalize!");
    SDcardPresent = false;
  }
  else 
  {
    SDcardPresent = true;
    LOADparms("default.cfg"); // Load the defaults if the file is present
  }
  SPI.setClockDivider(11);    // If SD card init fails it will slow down the SPI interface

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
  SPI.setClockDivider(SPI_CS,8);   // Default divider is 21 and equals 4 MHz
                                    // Increase speed for better performance of tables
  // Start the TWI interface
  TWI_RESET();

  Wire.begin();
  Wire.setClock(100000);
  // Start the real time interrupt
  Timer3.attachInterrupt(RealTimeISR);
  Timer3.start(100000); // Calls every 100ms
  // Initial splash screen
  Signon();
  ButtonPressed = false;
  ButtonRotated = false;
  // Init the baords
  ScanHardware();
  DIO_init();
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
  if(strlen(MIPSconfigData.StartupMacro) > 0)
  {
    MacroPlay(MIPSconfigData.StartupMacro,true);
  }
//  CompressorInit();  // For testing only! Make sure and remove
}

// This task is called every 10 milliseconds and handels a number of tasks.
void MIPSsystemLoop(void)
{
  float MaxVoltage;
//WDT_Disable(WDT);
  // Process any serial commands
  while(ProcessCommand()==0);  // Process until flag that there is nothing to do
  // Determine the maximum output voltage and set the proper button color
  MaxVoltage = MaxRFVoltage;
  if(MaxDCbiasVoltage > MaxVoltage) MaxVoltage = MaxDCbiasVoltage;
  if(MaxTwaveVoltage > MaxVoltage) MaxVoltage = MaxTwaveVoltage;
  if(MaxFAIMSVoltage > MaxVoltage) MaxVoltage = MaxFAIMSVoltage;
  if(MaxESIvoltage > MaxVoltage) MaxVoltage = MaxESIvoltage;
    // Test output voltages and update the button LEDs to provide warning to user
  // Blue LED on if voltgaes are enabled and above 0
  if(MaxVoltage == 0) {PB_OFF(PB_RED);}
  if(MaxVoltage >  0) {PB_ON(PB_BLUE);}
  if(MaxVoltage > 50) {PB_ON(PB_RED);}
  if(MaxVoltage > 75) {PB_FLASH(PB_RED);}
  // Test Vin, if power is off then shut down the hardware control lines and print
  // and warding on the display. Process serial commands and stay in this loop until
  // power is reapplied. Reset when power is reapplied
  #ifdef TestMode
  return;
  #endif
  if (ReadVin() < 10.0)
  {
    // Set all control lines to input, this will keep the systems from souring power to
    // the boards through driven outputs.
    Reset_IOpins();
    // Display a message on the screen that MIPS power is off.
    tft.fillScreen(ILI9340_BLACK);
    tft.setRotation(1);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
    tft.setTextSize(2);
    tft.println("");
    tft.println("MIPS power is off!");
    tft.println("USB powering controller!");
    tft.println("Apply power to operate...");
    // Wait for power to appear and then reset the system.
    while (ReadVin() < 10.0) WDT_Restart(WDT);
    Software_Reset();
  }
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
  Twave_init(0,0x52);
  #endif
  // Loop through all the addresses looking for signatures
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(0,addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(0);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(0);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(0);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(0);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(0);
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(1,addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(1);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(1);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(1);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(1);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(1);
    }
  }
  // Now look for the FAIMS board...
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
  WiFi_init();
  // The last thig we do is look for the analog input option
  if(MIPSconfigData.UseAnalog) Analog_init();
}

// Main processing loop
void loop()
{
  WDT_Restart(WDT);
  // run ThreadController
  // this will check every thread inside ThreadController,
  // if it should run. If yes, he will run it;
  control.run();
  // Process any encoder event
  if (ButtonRotated)
  {
    ButtonRotated = false;
    if (ActiveMenu != NULL) MenuProcessChange(ActiveMenu, encValue);
    else if (ActiveDialog != NULL) DialogBoxProcessChange(ActiveDialog, encValue);
    encValue = 0;
  }
  if (ButtonPressed)
  {
    delay(10);
    ButtonPressed = false;
    encValue = 0;
    if (ActiveMenu != NULL) MenuButtonPress(ActiveMenu);
    else if (ActiveDialog != NULL) DialogButtonPress(ActiveDialog);
    DismissMessageIfButton();
  }
  // Put serial received characters in the input ring buffer
  if (SerialUSB.available() > 0) 
  {
    serial = &SerialUSB;
    PutCh(SerialUSB.read());
  }
  if (Serial.available() > 0) 
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
  // If there is a command in the input ring buffer, process it!
  if(RB_Commands(&RB) > 0) while(ProcessCommand()==0);  // Process until flag that there is nothing to do
//  if (serial->available() > 0) serial->print((char)serial->read());
}


// Host command processing function.

// Save parameters to default.cfg file on SD card
int SAVEparms(char *filename)
{
  File file;
  
  // Test SD present flag, exit and NAK if no card or it failed to init
  if(!SDcardPresent) return(ERR_NOSDCARD);
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(filename);
  // Open file and write config structure to disk
  if(!(file=SD.open(filename,FILE_WRITE))) return(ERR_CANTCREATEFILE);
  MIPSconfigData.Size = sizeof(MIPSconfigStruct);
  file.write((byte *)&MIPSconfigData,sizeof(MIPSconfigStruct));
  file.close();
  return(0);
}

void SAVEparms(void)
{
  int iStat;
  
  if((iStat = SAVEparms("default.cfg")) == 0)
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
  int  i,fVal;
  byte *b;
  
  // Test SD present flag
  if(!SDcardPresent) return false;
  SD.begin(_sdcs);
  // Open the file
  if(!(file=SD.open(filename,FILE_READ))) return false;
  // read the data
  b = (byte *)&MCS;
  for(i=0;i<sizeof(MIPSconfigStruct);i++)
  {
    if((fVal = file.read()) == -1) break;
    b[i] = fVal;
  }
  file.close();
  // Copy to MIPS config struct
  memcpy(&MIPSconfigData, &MCS, MCS.Size);
  return true;
}




