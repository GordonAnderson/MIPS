//
// FAIMS
//
// This file supports the FAIMS module on the MIPS system. Only one FAIMS driver can be installed
// in a MIPS system.
//
// There are two modes for FAIMS, the standard FAIMS with a DCcv output and a DC bias output, and
// field driven FAIMS with 4 DC outputs. This version uses a +-250V DCbias board with 4 channels
// populated. In this case the DC bias board has its board address set to B. On powerup the FAIMS
// driver looks for a DC bias board at location B, if found the field driven mode is enerted.
//
// Added output voltage level lock mode. Select this mode by pressing the control buttom when the
// output voltage level is selected. The main menu will turn green in this mode. The drive is adjusted to
// hold the output with in +- 20 volts. This only adjusts for slow drifts in output voltage.
//
// If the presure sensor is found using TWI wrire1 channel then the Envionrment menu is enabled.
// This menu allows you to define adjustment limits and coefficents for presure and temp. 
//      Here is how this works:
//        - When the RF is enabled the base pressure and temp are recorded
//        - Measure pressure and temp and calculate the difference from the base values
//        - Multiply this difference by the coefficents
//        - Apply the limts
//        - Adjust the output voltage level only if we are in lock mode
//      Added enviornment menu to the tune menu
//        - Display the current press
//        - Display the current temp
//        - Enable
//        - Temp and tress coeef and limts
//
//
// To Dos:
//  1.) Add all the serial commands to support FAIMS.List of commands to add:
//      - Add FAIMS count to indicate its present in system. Count or 0 or 1
//      - Enable, ON OFF
//      - Frequency
//      - Drv enable
//      - Power
//      - Drv power
//      - Max pawer
//      - Max drv power
//      - Peak pos voltage
//      - Peak neg voltage
//      - Lots more to add!
//  2.) Consider adding DC bias power supply ON/OFF and tie the bias to RF on/off??
//  3.) Add calibration procedure for RF amplitude. This needs to be a three point
//      second order calibration function.
//
// Gordon Anderson
//
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include "FAIMS.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"
#include <Adafruit_PWMServoDriver.h>

extern DialogBox FAIMSMainMenu;
extern DialogBox FAIMSTuneMenu;
extern DialogBox FAIMSDriveMenu;
extern DialogBox FAIMSPowerMenu;
extern DialogBox FAIMSDCMenu;
extern DialogBox FAIMScalMenu;
extern DialogBox FAIMSEnvMenu;

#define  PWMFS  4095          // Full scale PWM output value
#define  FreqMultiplier 32
// Filter time constant is:
// TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
#define  Filter 0.05               // Strong filter coefficent
#define  EnvFilter 0.03            // Pressure and temp filter coefficent

FAIMSdata  faims = FAIMS_Rev_1;    // FAIMS main data structure

// Pressure temp sensor variables
Adafruit_PWMServoDriver pwm;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
bool   bmpSensor;            // Flag set is the pressure / temp sensor is found
float  basePressure;         // Initial pressure
float  baseTemp;             // Initial temp
float  currentPressure;      // Current pressure
float  currentTemp;          // Current temp
float  deltaPressure;        // Delta pressure
float  deltaTemp;            // Delta temp
float  EnvCorrection=0;      // Drive level correct in %

// FAIMS variables
int   NumberOfFAIMS = 0;      // Number of FAIMS modules
float MaxFAIMSVoltage = 0;    // This value is set to the highest DC bias voltage

float  DrvChange = 0;         // Global drive change that is applied to all drive channels
float  KVoutP = 0;            // Positive peak output voltage in KV
float  KVoutN = 0;            // Negative peak output voltage in KV

float  TotalPower = 0;        // Total power into the RF deck
float  Drv1Power = 0;
float  Drv2Power = 0;
float  Drv3Power = 0;

// FAIMS scan mode variables
bool   FAIMSscan = false;      // True when the DCcv scan is requensted
bool   FAIMSscanning = false;  // True when the system is scanning
bool   FieldDriven = false;    // This flag is set if this is a field driven FAIMS system
float  ScanTime = 0;           // This is how long the system has been scanning
float  ScanCV;
int    Loops = 1;

// Output level control variables
bool   Lock = false;          // This flag is true when in the output level lock mode
float  LockSetpoint = 0;      // Output lock level setpoint
float  MaxDriveChange = 10;   // Maximum about of drive level adjustment posible by the level control code
float  DriveChange=0;         // The abount of drive level changed from the level control code

bool    FAIMSpresent = false; // Set to true if FAIMS system is detected
int8_t  FAIMSBoardAddress;    // Board address for FAIMS
unsigned long OnMillis;       // Millisecond timer value at system startup
int     OnTime = 0;
int     FAIMSclockChange;

int    DiableArcDetectTimer = 0;  // If this value is non zero then the arc detected is disabled until it reaches 0.
                                  // it is decremented in the main loop.
// DC output readback values
float  DCoffsetRB = 0;
float  DCbiasRB   = 0;
float  DCcvRB     = 0;

void DelayArcDetect(void);

//MIPS Threads
Thread FAIMSThread  = Thread();

DialogBoxEntry FAIMSentriesMainMenu[] = {
  {" Enable"             , 0, 1, D_ONOFF   , 0, 1, 1, 19, false, NULL, &faims.Enable, NULL, NULL},
  {" Drive"              , 0, 2, D_FLOAT   , 5, 100, 0.1, 17, false, "%5.1f", &faims.Drv, NULL, NULL},
  {" RF, KV"             , 0, 3, D_FLOAT   , 0, 1, 1, 10, true, "%4.2f", &KVoutP, LockLevel, NULL},
  {""                    , 0, 3, D_FLOAT   , 0, 1, 1, 18, true, "%4.2f", &KVoutN, NULL, NULL},
  {" Power"              , 0, 4, D_FLOAT   , 0, 0, 0, 19, true, "%3.0f", &TotalPower, NULL, NULL},
  {" Tune menu"          , 0, 5, D_DIALOG  , 0, 0, 0, 0,  false, NULL, &FAIMSTuneMenu, NULL, NULL},
  {" Drive menu"         , 0, 6, D_DIALOG  , 0, 0, 0, 0,  false, NULL, &FAIMSDriveMenu, NULL, NULL},
  {" Power menu"         , 0, 7, D_DIALOG  , 0, 0, 0, 0,  false, NULL, &FAIMSPowerMenu, NULL, NULL},
  {" DC drive menu"      , 0, 8, D_DIALOG  , 0, 0, 0, 0,  false, NULL, &FAIMSDCMenu, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0,  false, NULL, NULL, SaveFAIMSSettings, NULL},
  {" Restore settings"   , 0, 10, D_FUNCTION, 0, 0, 0, 0,  false, NULL, NULL, RestoreFAIMSSettings, NULL},
  {" Return to main menu", 0, 11, D_MENU    , 0, 0, 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox FAIMSMainMenu = {
  {"FAIMS main menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesMainMenu
};

char FundamentalDelayName[] = " Fundamental phase";

DialogBoxEntry FAIMSentriesTuneMenu[] = {
  {" Frequency"              , 0, 1, D_INT     , 500000, 2000000, 1000, 16, false, "%7d", &faims.Freq, NULL, NULL},
  {" Coarse phase"           , 0, 2, D_INT     , 0, 7, 1, 20, false, "%3d", &faims.PhaseC, NULL, NULL},
  {" Fine phase"             , 0, 3, D_INT     , 0, 255, 1, 20, false, "%3d", &faims.PhaseF, NULL, NULL},
  {" Pri capacitance"        , 0, 4, D_FLOAT   , 0, 100, 0.1, 18, false, "%5.1f", &faims.Pcap, NULL, NULL},
  {" Har capacitance"        , 0, 5, D_FLOAT   , 0, 100, 0.1, 18, false, "%5.1f", &faims.Hcap, NULL, NULL},
  {" Arc Det Level"          , 0, 6, D_FLOAT   , 0, 100, 1, 19, false, "%4.0f", &faims.ArcSens, NULL, NULL},
  {" Environment menu"       , 0, 7, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSEnvMenu, NULL, NULL},
  {" Drive menu"             , 0, 8, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSDriveMenu, NULL, NULL},
  {" Power menu"             , 0, 9, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSPowerMenu, NULL, NULL},
  {" Return to FAIMS menu"   , 0, 10, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSMainMenu, NULL, NULL},
  {NULL},
};

DialogBox FAIMSTuneMenu = {
  {"FAIMS tune menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesTuneMenu
};

DialogBoxEntry FAIMSentriesEnvMenu[] = {
  {" Enable comp"            , 0, 1, D_ONOFF   ,  0, 1, 1, 20, false, NULL, &faims.Compensation, NULL, NULL},
  {" Pressure coeff"         , 0, 2, D_FLOAT   , -1, 1, .001, 17, false, "%6.3f", &faims.PressureCoeff, NULL, NULL},
  {" Temp coeff"             , 0, 3, D_FLOAT   , -5, 5, .001, 17, false, "%6.3f", &faims.TempCoeff, NULL, NULL},
  {" Max adj, %"             , 0, 4, D_FLOAT   ,  0, 10, 0.1, 18, false, "%5.1f", &faims.PressTempLimit, NULL, NULL},
  {" Pressure, hPa"          , 0, 5, D_FLOAT   ,  0, 10, 0.1, 15, true, "%8.1f", &currentPressure, NULL, NULL},
  {" Temp, C"                , 0, 6, D_FLOAT   ,  0, 10, 0.1, 18, true, "%5.1f", &currentTemp, NULL, NULL},
  {" Delta pressure"         , 0, 7, D_FLOAT   ,  0, 10, 0.1, 17, true, "%6.2f", &deltaPressure, NULL, NULL},
  {" Delta temp"             , 0, 8, D_FLOAT   ,  0, 10, 0.1, 18, true, "%5.2f", &deltaTemp, NULL, NULL},
  {" Correction, %"          , 0, 9, D_FLOAT   ,  0, 10, 0.1, 18, true, "%5.2f", &EnvCorrection, NULL, NULL},
  {" Return to Tune menu"    , 0, 10, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSTuneMenu, NULL, NULL},
  {NULL},
};

DialogBox FAIMSEnvMenu = {
  {"FAIMS enviornment menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesEnvMenu
};

DialogBoxEntry FAIMSentriesDriveMenu[] = {
  {" Enable Drv1"            , 0, 1, D_ONOFF   , 0, 1, 1, 19, false, NULL, &faims.Drv1.Enable, NULL, DelayArcDetect},
  {" Drv1 level"             , 0, 2, D_FLOAT   , 5, 100, 0.1, 17, false, "%5.1f", &faims.Drv1.Drv, NULL, NULL},
  {" Drv1 power"             , 0, 3, D_FLOAT   , 0, 1, 1, 19, true, "%3.0f", &Drv1Power, NULL, NULL},
  {" Enable Drv2"            , 0, 4, D_ONOFF   , 0, 1, 1, 19, false, NULL, &faims.Drv2.Enable, NULL, DelayArcDetect},
  {" Drv2 level"             , 0, 5, D_FLOAT   , 5, 100, 0.1, 17, false, "%5.1f", &faims.Drv2.Drv, NULL, NULL},
  {" Drv2 power"             , 0, 6, D_FLOAT   , 0, 1, 1, 19, true, "%3.0f", &Drv2Power, NULL, NULL},
  {" Enable Drv3"            , 0, 7, D_ONOFF   , 0, 1, 1, 19, false, NULL, &faims.Drv3.Enable, NULL, DelayArcDetect},
  {" Drv3 level"             , 0, 8, D_FLOAT   , 5, 100, 0.1, 17, false, "%5.1f", &faims.Drv3.Drv, NULL, NULL},
  {" Drv3 power"             , 0, 9, D_FLOAT   , 0, 1, 1, 19, true, "%3.0f", &Drv3Power, NULL, NULL},
  {" Tune menu"              , 0, 10, D_DIALOG  , 0, 0, 0, 0,  false, NULL, &FAIMSTuneMenu, NULL, NULL},
  {" Return to FAIMS menu"   , 0, 11, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSMainMenu, NULL, NULL},
  {NULL},
};

DialogBox FAIMSDriveMenu = {
  {"FAIMS drive menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesDriveMenu
};

DialogBoxEntry FAIMSentriesPowerMenu[] = {
  {" Drv1 power limit"       , 0, 1, D_FLOAT   , 0, 50, 1, 19, false, "%3.0f", &faims.Drv1.MaxPower, NULL, NULL},
  {" Drv2 power limit"       , 0, 2, D_FLOAT   , 0, 50, 1, 19, false, "%3.0f", &faims.Drv2.MaxPower, NULL, NULL},
  {" Drv3 power limit"       , 0, 3, D_FLOAT   , 0, 50, 1, 19, false, "%3.0f", &faims.Drv3.MaxPower, NULL, NULL},
  {" Total power limit"      , 0, 4, D_FLOAT   , 0, 150, 1, 19, false, "%3.0f", &faims.MaxPower, NULL, NULL},
  {" Max drive level"        , 0, 5, D_FLOAT   , 25, 100, 1, 19, false, "%3.0f", &faims.MaxDrive, NULL, NULL},
  {" Max time, hrs"          , 0, 6, D_FLOAT   , .1, 1000, 0.1, 17, false, "%6.1f", &faims.MaxOnTime, NULL, NULL},
  {" Return to FAIMS menu"   , 0, 11, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSMainMenu, NULL, NULL},
  {NULL},
};

DialogBox FAIMSPowerMenu = {
  {"FAIMS power menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesPowerMenu
};

DialogBoxEntry FAIMSentriesDCMenu[] = {
  {" DC bias"                , 0, 1, D_FLOAT   , -250, 250, 0.1, 11, false, "%5.1f", &faims.DCbias.VoltageSetpoint, NULL, NULL},
  {" DC CV"                  , 0, 2, D_FLOAT   , -250, 250, 0.1, 11, false, "%5.1f", &faims.DCcv.VoltageSetpoint, NULL, NULL},
  {" Offset"                 , 0, 3, D_FLOAT   , -250, 250, 0.1, 11, false, "%5.1f", &faims.DCoffset.VoltageSetpoint, NULL, UpdateNFDlimits},
  {"         Request Actual" , 0, 0, D_TITLE   , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {""                        , 0, 1, D_FLOAT   , 0, 0, 0, 18, true, "%5.1f", &DCbiasRB, NULL, NULL},
  {""                        , 0, 2, D_FLOAT   , 0, 0, 0, 18, true, "%5.1f", &DCcvRB, NULL, NULL},
  {""                        , 0, 3, D_FLOAT   , 0, 0, 0, 18, true, "%5.1f", &DCoffsetRB, NULL, NULL},
  {" CV start"               , 0, 5, D_FLOAT   , -250, 250, 0.1, 18, false, "%5.1f", &faims.CVstart, NULL, NULL},
  {" CV end"                 , 0, 6, D_FLOAT   , -250, 250, 0.1, 18, false, "%5.1f", &faims.CVend, NULL, NULL},
  {" Duration"               , 0, 7, D_FLOAT   , 1, 10000, 1, 15, false, "%8.1f", &faims.Duration, NULL, NULL},
  {" Scan"                   , 0, 8, D_ONOFF   , 0, 1, 1, 20, false, NULL, &FAIMSscan, NULL, NULL},
  {" Loops"                  , 0, 9, D_INT     , 1, 100, 1, 20, false, "%3d", &Loops, NULL, NULL},
  {" Calibration menu"       , 0, 10, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMScalMenu, NULL, NULL},
  {" Return to FAIMS menu"   , 0, 11, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSMainMenu, NULL, NULL},
  {NULL},
};

// This is the menu for the field driven mode when a DC bias board is detected at location B
DialogBoxEntry FAIMSentriesDCMenuFD[] = {
  {" DC bias"                , 0, 1, D_FLOAT   , dcbd.MinVoltage + dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage + dcbd.DCoffset.VoltageSetpoint, 0.1, 9, false, "%5.1f", &dcbd.DCCD[0].VoltageSetpoint, NULL, NULL},
  {"  "                      , 14,1, D_FLOAT   , dcbd.MinVoltage + dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage + dcbd.DCoffset.VoltageSetpoint, 0.1, 4, false, "%5.1f", &dcbd.DCCD[1].VoltageSetpoint, NULL, NULL},
  {" DC CV"                  , 0, 2, D_FLOAT   , dcbd.MinVoltage + dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage + dcbd.DCoffset.VoltageSetpoint, 0.1, 9, false, "%5.1f", &dcbd.DCCD[2].VoltageSetpoint, NULL, NULL},
  {"  "                      , 14,2, D_FLOAT   , dcbd.MinVoltage + dcbd.DCoffset.VoltageSetpoint, dcbd.MaxVoltage + dcbd.DCoffset.VoltageSetpoint, 0.1, 4, false, "%5.1f", &dcbd.DCCD[3].VoltageSetpoint, NULL, NULL},
  {" Offset"                 , 0, 4, D_FLOAT   , dcbd.MinVoltage, dcbd.MaxVoltage, 0.1, 9, false, "%5.1f", &dcbd.DCoffset.VoltageSetpoint, NULL, UpdateFDlimits},
  {"         Input   Output" , 0, 0, D_TITLE   , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" CV start"               , 0, 6, D_FLOAT   , -250, 250, 0.1, 18, false, "%5.1f", &faims.CVstart, NULL, NULL},
  {" CV end"                 , 0, 7, D_FLOAT   , -250, 250, 0.1, 18, false, "%5.1f", &faims.CVend, NULL, NULL},
  {" Duration"               , 0, 8, D_FLOAT   , 0, 10000, 1, 15, false, "%8.1f", &faims.Duration, NULL, NULL},
  {" Scan"                   , 0, 9, D_ONOFF   , 0, 1, 1, 20, false, NULL, &FAIMSscan, NULL, NULL},
  {" Return to FAIMS menu"   , 0, 11, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSMainMenu, NULL, NULL},
  {NULL},
};

// This function is called when the user selects the RF voltage. This button will toggle output level 
// control on and off.
void LockLevel(void)
{
  if(!faims.Enable) return;
  if(!Lock)
  {
    Lock = true;
    // Initalize all the control parameters
    LockSetpoint = KVoutP;
    DriveChange = 0;
    FAIMSMainMenu.w.Fcolor = ILI9340_GREEN;
    DisplayMessage("Voltage Lock Enabled", 2000);
  }
  else
  {
    Lock = false;
    FAIMSMainMenu.w.Fcolor = ILI9340_WHITE;
    DisplayMessage("Voltage Lock Disabled", 2000);
  }
}

void DelayArcDetect(void)
{
  DiableArcDetectTimer=50;
}

// This functions sets up the dialog box entry structure based on the DCbias card range.
void SetupFDEntry(DCbiasData *dc)
{
  int   i;
  
// Setup the min and max values for the user interface
  for(i=0;i<4;i++)
  {
    FAIMSentriesDCMenuFD[i].Min = dc->MinVoltage+dc->DCoffset.VoltageSetpoint;
    FAIMSentriesDCMenuFD[i].Max = dc->MaxVoltage+dc->DCoffset.VoltageSetpoint;
  }
  FAIMSentriesDCMenuFD[4].Min = dc->MinVoltage;
  FAIMSentriesDCMenuFD[4].Max = dc->MaxVoltage;
// Set the CV scan limits based on offset
  FAIMSentriesDCMenuFD[6].Min = dc->MinVoltage+dc->DCoffset.VoltageSetpoint;
  FAIMSentriesDCMenuFD[6].Max = dc->MaxVoltage+dc->DCoffset.VoltageSetpoint;
  FAIMSentriesDCMenuFD[7].Min = dc->MinVoltage+dc->DCoffset.VoltageSetpoint;
  FAIMSentriesDCMenuFD[7].Max = dc->MaxVoltage+dc->DCoffset.VoltageSetpoint;
}

void SetupNFDEntry(DCbiasData *dc)
{
  int   i;
  
// Setup the min and max values for the user interface
  for(i=0;i<2;i++)
  {
    FAIMSentriesDCMenu[i].Min = -250.0 + faims.DCoffset.VoltageSetpoint;
    FAIMSentriesDCMenu[i].Max =  250.0 + faims.DCoffset.VoltageSetpoint;
    FAIMSentriesDCMenu[7+i].Min = -250.0 + faims.DCoffset.VoltageSetpoint;
    FAIMSentriesDCMenu[7+i].Max =  250.0 + faims.DCoffset.VoltageSetpoint;
  }
}

void UpdateFDlimits(void)
{
  SetupFDEntry(&dcbd);
}

void UpdateNFDlimits(void)
{
  SetupNFDEntry(&dcbd);
}

DialogBox FAIMSDCMenu = {
  {"FAIMS DC menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesDCMenu
};

DialogBoxEntry FAIMSentriesDCcalMenu[] = {
  {" Cal DC bias"            , 0, 1, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateDCbias, NULL},
  {" Cal DC CV"              , 0, 2, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateDCcv, NULL},
  {" Cal Offset"             , 0, 3, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, CalibrateDCoffset, NULL},
  {" Return to DC drive menu", 0, 10, D_DIALOG  , 0, 0, 0, 0, false, NULL, &FAIMSDCMenu, NULL},
  {NULL},
};

DialogBox FAIMScalMenu = {
  {"FAIMS DC cal menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, FAIMSentriesDCcalMenu
};

MenuEntry MEFAIMSMonitor = {" FAIMS module", M_DIALOG, 0, 0, 0, NULL, &FAIMSMainMenu, NULL, NULL};

void CalibrateDCbias(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = -250;
  CC.Max =  250;
  CC.DACaddr = faims.DACadr;
  CC.ADCaddr = faims.ADC1adr;
  CC.DACout = &faims.DCbias.DCctrl;
  CC.ADCreadback = &faims.DCbias.DCmon;
  // Define this channels name
  sprintf(Name, "       DC bias");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalibrateDCcv(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = -250;
  CC.Max =  250;
  CC.DACaddr = faims.DACadr;
  CC.ADCaddr = faims.ADC1adr;
  CC.DACout = &faims.DCcv.DCctrl;
  CC.ADCreadback = &faims.DCcv.DCmon;
  // Define this channels name
  sprintf(Name, "       DC cv");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

void CalibrateDCoffset(void)
{
  ChannelCal CC;
  char       Name[20];

  // Set up the calibration data structure
  CC.ADCpointer = &AD7998;
  CC.Min = -250;
  CC.Max =  250;
  CC.DACaddr = faims.DACadr;
  CC.ADCaddr = faims.ADC1adr;
  CC.DACout = &faims.DCoffset.DCctrl;
  CC.ADCreadback = &faims.DCoffset.DCmon;
  // Define this channels name
  sprintf(Name, "       DC offset");
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

// Write the current board parameters to the EEPROM on the FAIMS board.
void SaveFAIMSSettings(void)
{
  if (FieldDriven)
  {
    if(DCbiasBoards[1])
    {
      SelectBoard(1);
      WriteEEPROM(&DCbDarray[1], DCbDarray[1].EEPROMadr, 0, sizeof(DCbiasData));
      SelectBoard(0);
    }
  }
TWI_RESET();
Wire.begin();
  SelectBoard(FAIMSBoardAddress);
  if (WriteEEPROM(&faims, faims.EEPROMadr, 0, sizeof(FAIMSdata)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

// Restore the parameters from the EEPROM on the FAIMS board. Read the EEPROM and make sure the board name
// matches what is expected, only load if its correct.
void RestoreFAIMSSettings(void)
{
  RestoreFAIMSSettings(false);
}

void RestoreFAIMSSettings(bool NoDisplay)
{
  FAIMSdata faimsT;

  if (FieldDriven)  // Read the DC bias card parameters as well for the field driven version.
  {
    RestoreDCbiasSettings(true);
    SelectBoard(0);
  }
TWI_RESET();
Wire.begin();
  SelectBoard(FAIMSBoardAddress);
  if (ReadEEPROM(&faimsT, faims.EEPROMadr, 0, sizeof(FAIMSdata)) == 0)
  {
    if (strcmp(faimsT.Name, faims.Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      memcpy(&faims, &faimsT, faimsT.Size);
      faims.Size = sizeof(FAIMSdata);
      Loops = faims.Loops;
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

int BuildGPIOimage(void)
{
  int   image;

  if(faims.Rev <= 2) image = faims.PhaseC & 0x07;
  if (faims.Drv1.Enable) image |= 0x10;
  if (faims.Drv2.Enable) image |= 0x20;
  if (faims.Drv3.Enable) image |= 0x40;
  if (!faims.Enable) image &= 0x8F;
  image &= ~0x08;  // Latch for 2nd harmonic delay, keep low
  if(faims.Rev == 3) image &= ~0x04;  // Latch for fundemental osc delay, keep low
  return image;
}

// This function sets the servo pulse with to the value passed in millisec.
// The function was provided by Adafruit and is not claimed to be accurate.
void setServoPulse(uint8_t n, double pulse)
{
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;       // 60 Hz
  pulselength /= 4096;     // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}

// External scan trigger interrupt vectors here!
void FAIMSscanISR(void)
{
  FAIMSscan = true;
}

// This ISR increments a counter on clock rising edge, part of the clock
// detection logic
void FAIMSclockDetechISR(void)
{
  FAIMSclockChange++;
}

// This function is called at powerup to initiaize the FAIMS board.
void FAIMS_init(int8_t Board)
{
  DialogBoxEntry *de = GetDialogEntries(FAIMSentriesTuneMenu, " Environment menu");
  DialogBoxEntry *de2 = GetDialogEntries(FAIMSentriesTuneMenu, " Coarse phase");

  // Flag the board as present
  FAIMSpresent = true;
  NumberOfFAIMS = 1;
  // Set active board to board being inited
  FAIMSBoardAddress = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the FAIMS card.
  if (NormalStartup)
  {
    RestoreFAIMSSettings(true);
  }
  Loops = faims.Loops;
  // Force global enable to false and set global drive to highest driver channel value
  faims.Enable = false;
  faims.Drv = faims.Drv1.Drv;
  if (faims.Drv2.Drv > faims.Drv) faims.Drv = faims.Drv2.Drv;
  if (faims.Drv3.Drv > faims.Drv) faims.Drv = faims.Drv3.Drv;
  DrvChange = 0;
  // Init all the hardware...
  // Set GPIO direction registers
  MCP2300(faims.GPIOadr, 0, (uint8_t)0x80);
  MCP2300(faims.DELAYadr, 0, (uint8_t)0);
  // Init the clock generator and set the frequencies
  if(faims.Rev <= 2)
  {
     SetRef(20000000);
     CY_Init(faims.CLOCKadr);
     SetPLL2freq(faims.CLOCKadr, faims.Freq * FreqMultiplier);
  }
  if(faims.Rev == 3) 
  {
     SetRef(20000000);
     FAIMSclockSet(faims.CLOCKadr, faims.Freq);
  }
  // Setup the PWM outputs and set levels
  analogWriteResolution(12);
  if(faims.Enable == false)
  {
    pinMode(faims.Drv1.PWMchan, OUTPUT);
    analogWrite(faims.Drv1.PWMchan, 0);
    pinMode(faims.Drv2.PWMchan, OUTPUT);
    analogWrite(faims.Drv2.PWMchan, 0);
    pinMode(faims.Drv3.PWMchan, OUTPUT);
    analogWrite(faims.Drv3.PWMchan, 0);
  }
  else
  {
    pinMode(faims.Drv1.PWMchan, OUTPUT);
    analogWrite(faims.Drv1.PWMchan, (faims.Drv1.Drv * PWMFS) / 100);
    pinMode(faims.Drv2.PWMchan, OUTPUT);
    analogWrite(faims.Drv2.PWMchan, (faims.Drv2.Drv * PWMFS) / 100);
    pinMode(faims.Drv3.PWMchan, OUTPUT);
    analogWrite(faims.Drv3.PWMchan, (faims.Drv3.Drv * PWMFS) / 100);
  }
  // Setup the servo drives
  pwm = Adafruit_PWMServoDriver(0x40);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  setServoPulse(1, 1 + (faims.Pcap / 100));
  setServoPulse(0, 1 + (faims.Hcap / 100));
  // Set field driven flag if DCbias board is found at B
  if (isDCbiasBoard(1))
  {
    FieldDriven = true;
    // Change the DCbias menu option
    FAIMSDCMenu.Entry = FAIMSentriesDCMenuFD;
    SetupFDEntry(&dcbd);
  }
  else SetupNFDEntry(&dcbd);
  // Setup the menu and the loop processing thread
  AddMainMenuEntry(&MEFAIMSMonitor);
  DialogBoxDisplay(&FAIMSMainMenu);
  // Configure Threads
  FAIMSThread.setName("FAIMS");
  FAIMSThread.onRun(FAIMS_loop);
  FAIMSThread.setInterval(100);
  // Add threads to the controller
  control.add(&FAIMSThread);
  // Use trig input on pin 12 as a trigger to start a scan
  attachInterrupt(12, FAIMSscanISR, RISING);
  // Turn DC power supply on
  digitalWrite(PWR_ON, LOW);
  // Pressure sensor to be used for FAIMS voltage correction
  pinMode(A3,OUTPUT);
  pinMode(A4,OUTPUT);    // Turns on pull ups for the TWI lines
  bmpSensor = false;
  if(bmp.begin())
  {
    Wire1.setClock(100000);
    // Here if the pressure / temp sensor is found. Set flag that its present
    bmpSensor = true;
    bmp.getPressure(&basePressure);
    bmp.getTemperature(&baseTemp);
    currentPressure = basePressure;
    currentTemp = baseTemp;
    deltaPressure = deltaTemp = 0;
  }
  else
  {
    de->Type = D_OFF;
  }
  if(faims.Rev == 3)
  {
    de2->Max  = 255;
    de2->Name = FundamentalDelayName;
  }
}

// Monitor power limits and reduce drive if over the limit. First reduce DriveChange until
// its zero then if still over power reduce individual drive level to lower limit. If it still
// over power then disable the drive.
void FAIMSpowerMonitor(void)
{
  // Exit if global enable is false
  if (!faims.Enable) return;
  // If any power level is over limit by more that 2x then turn off the global enable
  if ((TotalPower > 2 * faims.MaxPower) || (Drv1Power > 2 * faims.Drv1.MaxPower) || (Drv2Power > 2 * faims.Drv2.MaxPower) || (Drv3Power > 2 * faims.Drv3.MaxPower))
  {
    faims.Enable = false;
    DisplayMessage(" Excessive Power! ", 2000);
  }
  // If over power reduce drive until its in safe limit. Use global drive level and reduce to lower limit,
  // if still too high turn off global enable
  if ((TotalPower > faims.MaxPower) || (Drv1Power > faims.Drv1.MaxPower) || (Drv2Power > faims.Drv2.MaxPower) || (Drv3Power > faims.Drv3.MaxPower))
  {
    faims.Drv -= 0.1;
    if (faims.Drv < 25.0)
    {
      faims.Drv = 25;
      faims.Enable = false;
      DisplayMessage(" Power limit! ", 2000);
    }
    else if (ActiveDialog == &FAIMSMainMenu) UpdateNoEditDialogEntries(&FAIMSMainMenu);
  }
}

// This function tests the clock and returns true if its runninng and near the correct frequency.
// Use DUE input on pin 2 as a clock detector, this will have the fundamental divided by 16.
// This function turns on the pin 2 interrupt and counts edges for 10 millisec.
bool TestClock(void)
{
  unsigned int mt;

  if(faims.Rev == 3) return(true);  // Disable for rev 3 hardware
  mt = millis();
  FAIMSclockChange = 0;
  attachInterrupt(2, FAIMSclockDetechISR, RISING);
  while ((mt + 10) > millis())
  {
  }
  detachInterrupt(2);
  if ((FAIMSclockChange * 1600) > faims.Freq * .90) return (true);    
  return (false);
}

// FAIMS dc bias control for normal, non field driven FAIMS operation.
void FAIMSDCbiasControl(void)
{
  static bool SuppliesOff = true;
  static int  SuppliesStableCount = 10;
  static unsigned long StartTime;

  // Monitor power on output bit. If power comes on hold all output at zero until power is stable, short delay.
  if (digitalRead(PWR_ON) != 0)
  {
    // Here if power is off, set supply off flag and delay loop counter
    SuppliesOff = true;
    SuppliesStableCount = 10;
  }
  else
  {
    // Here when supplies are on
    if (--SuppliesStableCount <= 0) SuppliesOff = false;
  }
  // Output the DC supply voltages. Set all to 0 if power supply is off.
  // If the system is scanning then calculate the next value and output.
  if (SuppliesOff)
  {
    AD5625(faims.DACadr, faims.DCoffset.DCctrl.Chan, Value2Counts(0, &faims.DCoffset.DCctrl));
    AD5625(faims.DACadr, faims.DCbias.DCctrl.Chan, Value2Counts(0, &faims.DCbias.DCctrl));
    AD5625(faims.DACadr, faims.DCcv.DCctrl.Chan, Value2Counts(0, &faims.DCcv.DCctrl));
  }
  else
  {
    AD5625(faims.DACadr, faims.DCoffset.DCctrl.Chan, Value2Counts(faims.DCoffset.VoltageSetpoint, &faims.DCoffset.DCctrl));
    AD5625(faims.DACadr, faims.DCbias.DCctrl.Chan, Value2Counts(faims.DCbias.VoltageSetpoint - faims.DCoffset.VoltageSetpoint, &faims.DCbias.DCctrl));
    if (!FAIMSscan)
    {
      if (FAIMSscanning) DCcvRB = faims.DCcv.VoltageSetpoint;
      FAIMSscanning = false;
      AD5625(faims.DACadr, faims.DCcv.DCctrl.Chan, Value2Counts(faims.DCcv.VoltageSetpoint - faims.DCoffset.VoltageSetpoint, &faims.DCcv.DCctrl));
    }
    else
    {
      // Here if FAIMS scan flag is true.
      // Check if we are currently scanning and update the voltage if true.
      // If we are not currently scanning then init the scan mode and start.
      if (!FAIMSscanning)
      {
        // Pulse output A to trigger a mass spec, added 6/17/2016
        // Set output A
        // Set the image bits
        SDIO_Set_Image('A', '1');
        // Send to the hardware
        DOrefresh;
        // Toggle LDAC
        PulseLDAC;
        delay(10);
        // Reset output A
        // Set the image bits
        SDIO_Set_Image('A', '0');
        // Send to the hardware
        DOrefresh;
        // Toggle LDAC
        PulseLDAC;
        // Setup the scanning parameters
        ScanTime = 0;
        StartTime = millis();
        FAIMSscanning = true;
        ScanCV = faims.CVstart;
        DCcvRB = ScanCV;  // Reset the filter
        // Refresh the menu in case this was an external trigger
        if (ActiveDialog == &FAIMSDCMenu) 
        {
          DisplayAllDialogEntries(&FAIMSDCMenu);
          FAIMSDCMenu.State = M_SCROLLING;
        }
      }
      else
      {
        // Here if system is scanning
        // Determine the total scan time in seconds
        ScanTime = (millis() - StartTime)/1000;
        if (ScanTime > faims.Duration)
        {
          // Stop scanning and clear all the scanning flags if all loops are done
          if(--Loops <= 0)
          {
             FAIMSscan = false;
             FAIMSscanning = false;
             DCcvRB = faims.DCcv.VoltageSetpoint;
             Loops = faims.Loops;
             // Refresh the menu
             if (ActiveDialog == &FAIMSDCMenu) 
             {
               DisplayAllDialogEntries(&FAIMSDCMenu);
               FAIMSDCMenu.State = M_SCROLLING;
             }
          }
          else
          {
            FAIMSscanning = false;
          }
        }
        else
        {
          // Update the scan voltage
          ScanCV = faims.CVstart + (faims.CVend - faims.CVstart) * ScanTime / faims.Duration;
        }
      }
      AD5625(faims.DACadr, faims.DCcv.DCctrl.Chan, Value2Counts(ScanCV - faims.DCoffset.VoltageSetpoint, &faims.DCcv.DCctrl));
    }
  }
}

// This function supports the field driven FAIMS DC bias control
void FieldDrivenFAIMSDCbiasControl(void)
{
  static float CVdelta;
  static float SavedCV;
  static unsigned long StartTime;
  
  if (!FAIMSscan)
  {
    if (FAIMSscanning)
    {
      // Here when scanning was stopped and first detected so perform any needed cleanup
      dcbd.DCCD[2].VoltageSetpoint = SavedCV;
      dcbd.DCCD[3].VoltageSetpoint = SavedCV + CVdelta;
      // Refresh the menu in case this was an external trigger
      if (ActiveDialog == &FAIMSDCMenu) 
      {
        DisplayAllDialogEntries(&FAIMSDCMenu);
        FAIMSDCMenu.State = M_SCROLLING;      
      }
      DelayMonitoring();
    }
    FAIMSscanning = false;
  }
  else
  {
    // Here if FAIMS scan flag is true.
    // Check if we are currently scanning and update the voltage if true.
    // If we are not currently scanning then init the scan mode and start.
    if (!FAIMSscanning)
    {
      // Calculate the input output delta voltage
      CVdelta = dcbd.DCCD[3].VoltageSetpoint - dcbd.DCCD[2].VoltageSetpoint;
      SavedCV = dcbd.DCCD[2].VoltageSetpoint;
      // Setup the scanning parameters
      StartTime = millis();
      FAIMSscanning = true;
      ScanCV = faims.CVstart;
      // Set the DCbias voltages
      dcbd.DCCD[2].VoltageSetpoint = ScanCV;
      dcbd.DCCD[3].VoltageSetpoint = ScanCV + CVdelta;
      DelayMonitoring();
      // Refresh the menu in case this was an external trigger
      if (ActiveDialog == &FAIMSDCMenu) 
      {
        DisplayAllDialogEntries(&FAIMSDCMenu);
        FAIMSDCMenu.State = M_SCROLLING;
      }
    }
    else
    {
      // Here if system is scanning
      // Determine the total scan time in seconds
      ScanTime = (millis() - StartTime)/1000;
      if (ScanTime > faims.Duration)
      {
        // Stop scanning and clear all the scanning flags
        FAIMSscan = false;
        FAIMSscanning = false;
        // Set the DCbias voltages
        dcbd.DCCD[2].VoltageSetpoint = SavedCV;
        dcbd.DCCD[3].VoltageSetpoint = SavedCV + CVdelta;
        DelayMonitoring();
      }
      else
      {
        // Update the scan voltage
        ScanCV = faims.CVstart + (faims.CVend - faims.CVstart) * ScanTime / faims.Duration;
        // Set the DCbias voltages
        dcbd.DCCD[2].VoltageSetpoint = ScanCV;
        dcbd.DCCD[3].VoltageSetpoint = ScanCV + CVdelta;
      }
      // Refresh the menu
      if (ActiveDialog == &FAIMSDCMenu) DisplayAllDialogEntries(&FAIMSDCMenu);
    }
  }
}

// This function processes the enviornment sensors, pressure and temp. These parameters
// are used to adjust the drive level based on the coeff values defined by the user.
void ProcessEnvionment(void)
{
  float fVal;

  if(!bmpSensor) return;  // Exit if no sensor was found
  if(!bmp.begin()) bmpSensor = false;
  // Read and filter the current pressure and temp
  bmp.getPressure(&fVal);
  currentPressure = EnvFilter * fVal + (1 - EnvFilter) * currentPressure;
  bmp.getTemperature(&fVal);
  currentTemp = EnvFilter * fVal + (1 - EnvFilter) * currentTemp;
  // Calculate the pressure and temp delta
  deltaPressure = currentPressure - basePressure;
  deltaTemp = currentTemp - baseTemp;
  // Calculate the correction factor and apply the limits
  EnvCorrection = deltaTemp * faims.TempCoeff + deltaPressure * faims.PressureCoeff;
  if(EnvCorrection > faims.PressTempLimit) EnvCorrection = faims.PressTempLimit;
  if(EnvCorrection < -faims.PressTempLimit) EnvCorrection = -faims.PressTempLimit;
  // If the env dialogbox is displayed update the displayed values
  if (ActiveDialog == &FAIMSEnvMenu) RefreshAllDialogEntries(&FAIMSEnvMenu);
}

// This function is called every 100 mS to process the FAIMS module.
void FAIMS_loop(void)
{
  uint8_t GPIOreg;
  int   i;
  float MaxDrv;
  uint16_t ADCvals[8];
  float V, Vp, Vn, I, SP;
  static int  LastFreq = -1;
  static int  LastGPIO = -1;
  static int  LastDelay = -1;
  static int  LastFunDelay = -1;
  static int  LastDrv  = -1;
  static bool LastEnable = false;
  static DialogBoxEntry *TMde = GetDialogEntries(FAIMSentriesTuneMenu, " Frequency");
  static DialogBoxEntry *PCde = GetDialogEntries(FAIMSentriesTuneMenu, " Pri capacitance");
  static DialogBoxEntry *HCde = GetDialogEntries(FAIMSentriesTuneMenu, " Har capacitance");

  if(DiableArcDetectTimer > 0) DiableArcDetectTimer--;
  // If the active dialog is the tune menu or the drive menu and there has been an entry
  // change then disable the arc detector for a bit.
  if((ActiveDialog == &FAIMSTuneMenu) || (ActiveDialog == &FAIMSDriveMenu))
  {
     if(ActiveDialog->Changed) DelayArcDetect(); 
     ActiveDialog->Changed = false;
  }
  // Select the board
  SelectBoard(FAIMSBoardAddress);
  MaxFAIMSVoltage = 0;
  // Limit the drive levels to MaxDrive
  if (faims.Drv > faims.MaxDrive) faims.Drv = faims.MaxDrive;
  if (faims.Drv1.Drv > faims.MaxDrive) faims.Drv1.Drv = faims.MaxDrive;
  if (faims.Drv2.Drv > faims.MaxDrive) faims.Drv2.Drv = faims.MaxDrive;
  if (faims.Drv3.Drv > faims.MaxDrive) faims.Drv3.Drv = faims.MaxDrive;
  // If enabled then disable frequency changing, only allow if drives are off
  if (faims.Enable) TMde->NoEdit = true;
  else TMde->NoEdit = false;
  // If drive is enabled and over 30% then disable the capacitance adjustment.
  if ((faims.Enable) && (faims.Drv > 30))
  {
    PCde->NoEdit = true;
    HCde->NoEdit = true;
  }
  else
  {
    PCde->NoEdit = false;
    HCde->NoEdit = false;
  }
  // If the global enable has just changed state to on, then check the clock to
  // make sure its running and correct. If not do not enable and issue a warning.
  if ((!LastEnable) && (faims.Enable))
  {
    if (!TestClock())
    {
      faims.Enable = false;
      DisplayMessage(" Clock Error! ", 2000);
    }
    else
    {
      // If here the system was justed enabled so save the millisecond timer value
      OnMillis = millis();
      // Record the base pressure and temp for compensation
      // Read the base pressure and temp (barometric pressure is measure in hPa) 
      if(bmpSensor)
      {
        basePressure = currentPressure;
        baseTemp = currentTemp;
        deltaPressure = deltaTemp = 0;
      }
    }
  }
  LastEnable = faims.Enable;
  // Turn on RF on led in faims RF deck if any drive is enabled and global enable
  // is true.
  if (faims.Enable && (faims.Drv1.Enable || faims.Drv2.Enable || faims.Drv3.Enable))
  {
    // Turn on RF on LED
    RFON_ON;
    // Calculate the on time in seconds
    OnTime = (millis() - OnMillis) / 1000;
  }
  else
  {
    // Turn off RF on LED
    RFON_OFF;
    // Clear on time counter
    OnTime = 0;
    OnMillis = 0;
  }
  // If OnTime is over the limit, in hours, then disable the drives
  if ((faims.MaxOnTime * 3600) < OnTime)
  {
    faims.Enable = false;
    OnTime = 0;
    if (ActiveDialog == &FAIMSMainMenu) DisplayAllDialogEntries(&FAIMSMainMenu);
  }
  // Update the frequency if it has changed
  if (faims.Freq != LastFreq)
  {
    for (i = 0; i < 5; i++) 
    {
      if((faims.Rev <= 2) && (SetPLL2freq(faims.CLOCKadr, faims.Freq * FreqMultiplier) == 0)) break;
      if(faims.Rev == 3) if(FAIMSclockSet(faims.CLOCKadr, faims.Freq) == 0) break;
      TWI_RESET();
      Wire.begin();
    }
    LastFreq = faims.Freq;
  }
  // Process the global drive level changes and set PWM output level
  if (LastDrv != faims.Drv)
  {
    MaxDrv = faims.Drv1.Drv;
    if (faims.Drv2.Drv > MaxDrv) MaxDrv = faims.Drv2.Drv;
    if (faims.Drv3.Drv > MaxDrv) MaxDrv = faims.Drv3.Drv;
    DrvChange = faims.Drv - MaxDrv;
    if(faims.Drv > LastDrv) DiableArcDetectTimer = 0;
    else DelayArcDetect();
    LastDrv = faims.Drv;
  }
  if(faims.Enable == false)
  {
    analogWrite(faims.Drv1.PWMchan, 0);
    analogWrite(faims.Drv2.PWMchan, 0);
    analogWrite(faims.Drv3.PWMchan, 0);
  }
  else
  {
    analogWrite(faims.Drv1.PWMchan, ((faims.Drv1.Drv + DrvChange) * PWMFS) / 100);
    analogWrite(faims.Drv2.PWMchan, ((faims.Drv2.Drv + DrvChange) * PWMFS) / 100);
    analogWrite(faims.Drv3.PWMchan, ((faims.Drv3.Drv + DrvChange) * PWMFS) / 100);
  }
  // Update GPIO if it has changed
  if (BuildGPIOimage() != LastGPIO)
  {
    LastGPIO = BuildGPIOimage();
    // Retry if the GPIO fails
    for(int j = 0; j < 10; j++) if (MCP2300(faims.GPIOadr, 0x0A, LastGPIO) == 0) break;
    delay(1);
  }
  // Update the delay GPIO if needed
  if (faims.PhaseF != LastDelay)
  {
    LastDelay = faims.PhaseF;
    // Retry if the GPIO fails
    for (int j = 0; j < 10; j++) if (MCP2300(faims.GPIOadr, 0x0A, LastGPIO |= 0x08) == 0) break;
    delay(1);
    // Retry if the GPIO fails
    for (int j = 0; j < 10; j++) if (MCP2300(faims.DELAYadr, 0x0A, faims.PhaseF) == 0) break;
    delay(1);
  }
  // For rev 3 update fundamental delay if needed
  if (faims.Rev == 3) if (faims.PhaseC != LastFunDelay)
  {
    LastFunDelay = faims.PhaseC;
    // Retry if the GPIO fails
    for (int j = 0; j < 10; j++) if (MCP2300(faims.GPIOadr, 0x0A, LastGPIO |= 0x04) == 0) break;
    delay(1);
    // Retry if the GPIO fails
    for (int j = 0; j < 10; j++) if (MCP2300(faims.DELAYadr, 0x0A, faims.PhaseC) == 0) break;    
    delay(1);
  }
  // Update the capacitor servos
  setServoPulse(1, 1 + (faims.Pcap / 100));
  setServoPulse(0, 1 + (faims.Hcap / 100));
  // Monitor the driver voltage and current then calculate power
  if (AD7998(faims.ADC2adr, ADCvals) == 0)
  {
    V = Counts2Value(ADCvals[faims.Drv1.Vmon.Chan], &faims.Drv1.Vmon);
    I = Counts2Value(ADCvals[faims.Drv1.Imon.Chan], &faims.Drv1.Imon);
    Drv1Power = Filter * (V * I) + (1 - Filter) * Drv1Power;

    V = Counts2Value(ADCvals[faims.Drv2.Vmon.Chan], &faims.Drv2.Vmon);
    I = Counts2Value(ADCvals[faims.Drv2.Imon.Chan], &faims.Drv2.Imon);
    Drv2Power = Filter * (V * I) + (1 - Filter) * Drv2Power;

    V = Counts2Value(ADCvals[faims.Drv3.Vmon.Chan], &faims.Drv3.Vmon);
    I = Counts2Value(ADCvals[faims.Drv3.Imon.Chan], &faims.Drv3.Imon);
    Drv3Power = Filter * (V * I) + (1 - Filter) * Drv3Power;

    TotalPower = Drv1Power + Drv2Power + Drv3Power;
  }
  // Monitor the voltage levels
  if (AD7998(faims.ADC1adr, ADCvals) == 0)
  {
    // Monitor the output voltage both positive and negative peak
    Vp = Counts2Value(ADCvals[faims.RFharP.Chan], &faims.RFharP);
    if (Vp < 0) Vp = 0;
    KVoutP = Filter * Vp + (1 - Filter) * KVoutP;
    if (KVoutP * 1000 > MaxFAIMSVoltage) MaxFAIMSVoltage = KVoutP * 1000;

    Vn = Counts2Value(ADCvals[faims.RFharN.Chan], &faims.RFharN);
    if (Vn < 0) Vn = 0;
    KVoutN = Filter * Vn + (1 - Filter) * KVoutN;
    if (KVoutN * 1000 > MaxFAIMSVoltage) MaxFAIMSVoltage = KVoutN * 1000;
    // Arc detection
    // Compare the unfiltered value to the filtered value and if there is a sudden drop it
    // could indicate an arc so turn off the system. (add this code! here)
    if(DiableArcDetectTimer == 0) if (faims.Enable) if (((KVoutP + KVoutN) - (Vp + Vn)) > (KVoutP + KVoutN) / 3)
    {
      if((KVoutP + KVoutN) > (100 - faims.ArcSens)/100)
      {
        // Arc detected! Shutdown
        // Disable the drivers and display a estop warning
        faims.Enable = false;
        if (ActiveDialog != NULL) ActiveDialog->State = M_SCROLLING;
        DisplayMessage(" Arc detected! ", 2000);
      }
    }
    // Monitor all the readback voltages
    DCoffsetRB = Filter * Counts2Value(ADCvals[faims.DCoffset.DCmon.Chan], &faims.DCoffset.DCmon) + (1 - Filter) * DCoffsetRB;
    if (DCoffsetRB > MaxFAIMSVoltage) MaxFAIMSVoltage = DCoffsetRB;
    DCbiasRB = Filter * Counts2Value(ADCvals[faims.DCbias.DCmon.Chan], &faims.DCbias.DCmon) + (1 - Filter) * DCbiasRB;
    if (DCbiasRB > MaxFAIMSVoltage) MaxFAIMSVoltage = DCbiasRB;
    DCcvRB = Filter * Counts2Value(ADCvals[faims.DCcv.DCmon.Chan], &faims.DCcv.DCmon) + (1 - Filter) * DCcvRB;
    if (DCcvRB > MaxFAIMSVoltage) MaxFAIMSVoltage = DCcvRB;
  }
  // Display the monitored values based on the dialog box curently being displayed
  if (ActiveDialog == &FAIMSMainMenu) RefreshAllDialogEntries(&FAIMSMainMenu);
  if (ActiveDialog == &FAIMSDriveMenu) RefreshAllDialogEntries(&FAIMSDriveMenu);
  if (ActiveDialog == &FAIMSDCMenu) RefreshAllDialogEntries(&FAIMSDCMenu);
  // Process the Estop input, if pressed then set global enable to false
  if (faims.Enable)
  {
    for(int j=0;j<100;j++) if (MCP2300(faims.GPIOadr, 0x09, &GPIOreg) == 0) break;
    //    MCP2300(faims.GPIOadr, 0x09, &GPIOreg);
    if ((GPIOreg & 0x80) != 0)
    {
      // Disable the drivers and display a estop warning
      faims.Enable = false;
      if (ActiveDialog != NULL) ActiveDialog->State = M_SCROLLING;
      DisplayMessage(" Emergency Stop! ", 2000);
    }
  }
  // Monitor power limits and reduce drive if over the limit.
  FAIMSpowerMonitor();
  // Process all DC bias outputs. Two options, standard FAIMS and field driven FAIMS
  if (FieldDriven)
  {
    // Here is field driven FAIMS. In this case a DCbias board is used to provide the needed DC supplies.
    // This requires 4 channels, in and out for each FAIMS plate.
    FieldDrivenFAIMSDCbiasControl();
  }
  else
  {
    FAIMSDCbiasControl();
  }
  ProcessEnvionment();
  // Output level control
  if(Lock)
  {
    SP = LockSetpoint;
    if(faims.Compensation) SP *= (1+EnvCorrection/100);
    if(!faims.Enable)
    {
      Lock = false;
      DriveChange = 0;
      FAIMSMainMenu.w.Fcolor = ILI9340_WHITE;
      if (ActiveDialog == &FAIMSMainMenu) 
      {
        FAIMSMainMenu.State = M_SCROLLING;
        DialogBoxDisplay(&FAIMSMainMenu);
      }
    }
    else if(fabs(LockSetpoint - KVoutP) > 0.02)
    {
       if(LockSetpoint < KVoutP) 
       {
         DriveChange -= 0.01;
         if(DriveChange < -MaxDriveChange) DriveChange = -MaxDriveChange;
         else faims.Drv -= 0.01;
       }
       else 
       {
         DriveChange += 0.01;
         if(DriveChange > MaxDriveChange) DriveChange = MaxDriveChange;
         else faims.Drv += 0.01;
       }
    }
  }
}

//
// This section contains all of the serial command processing routines.
//

// Returns the number of FAMES drivers, only 0 or 1 supported at this time...
void FAIMSnumberOfChannels(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(NumberOfFAIMS);
}

void FAIMSsetRFharPcal(char *m, char *b)
{
  String res;
  
  res = m;
  faims.RFharP.m = res.toFloat();
  res = b;
  faims.RFharP.b = res.toFloat();
}

void FAIMSsetRFharNcal(char *m, char *b)
{
  String res;
  
  res = m;
  faims.RFharN.m = res.toFloat();
  res = b;
  faims.RFharN.b = res.toFloat();
}






