//
// Analog
//
//  This file contains the code to support analog inputs. If the analog inputs are detected this code allows the user to define mapping from
//  each analog input (up to 8 maximum) to the desired paramter to control. The control minimum and maximum range can also be defined.
//  This setup information is saved to the systems SD drive.
//
// The Adafruit ADS1115 4 channels breakout boards are used to implement the capability.
//
//  Valid channels that can be controlled
//  NONE, default mapping that does nothing
//  DCB1 through DCB8 or DCB16 depending on how many channels are avalible. These parameters set the output voltage for the defined channel.
//  RFD1 through RFD2 or RFD4 depending on how manny channels are avalible.
//
//
// Gordon Anderson
// July 9, 2015
//
#include <stdarg.h>
#include "wire.h"
#include "SD.h"
#include "string.h"
#include "Serial.h"
#include "Analog.h"
#include "dialog.h"
#include "DCbias.h"
#include "RFdriver.h"

//MIPS Threads
Thread AnalogThread  = Thread();

// Setup objects for the two ADCs
Adafruit_ADS1115 *ads1;  // (0x48);
Adafruit_ADS1115 *ads2;  // (0x49);

int CalAnalogChannel=1;

Analog analog = {sizeof(Analog), "Analog", 1, false, 8, 0x48, 0x49,
                 "NONE", 0, 0.0, -250, 250, 0, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 1, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 2, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 3, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 0, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 1, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 2, 2667, 0, NULL,
                 "NONE", 0, 0.0, -250, 250, 3, 2667, 0, NULL,
                };

char *AnalogOptionList;

extern DialogBoxEntry AnalogEntriesPage2[];
extern DialogBox AnalogMonitorDialog;

DialogBoxEntry AnalogEntriesPage1[] = {
  {" 1" , 0, 1, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[0].Name, NULL, UpdateMinMax},
  {" "  , 8, 1, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[0].Min, NULL, NULL},
  {" "  , 15, 1, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[0].Max, NULL, NULL},
  {" 2" , 0, 2, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[1].Name, NULL, UpdateMinMax},
  {" "  , 8, 2, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[1].Min, NULL, NULL},
  {" "  , 15, 2, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[1].Max, NULL, NULL},
  {" 3" , 0, 3, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[2].Name, NULL, UpdateMinMax},
  {" "  , 8, 3, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[2].Min, NULL, NULL},
  {" "  , 15, 3, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[2].Max, NULL, NULL},
  {" 4" , 0, 4, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[3].Name, NULL, UpdateMinMax},
  {" "  , 8, 4, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[3].Min, NULL, NULL},
  {" "  , 15, 4, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[3].Max, NULL, NULL},
  {" 5" , 0, 5, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[4].Name, NULL, UpdateMinMax},
  {" "  , 8, 5, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[4].Min, NULL, NULL},
  {" "  , 15, 5, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[4].Max, NULL, NULL},
  {" 6" , 0, 6, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[5].Name, NULL, UpdateMinMax},
  {" "  , 8, 6, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[5].Min, NULL, NULL},
  {" "  , 15, 6, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[5].Max, NULL, NULL},
  {" 7" , 0, 7, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[6].Name, NULL, UpdateMinMax},
  {" "  , 8, 7, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[6].Min, NULL, NULL},
  {" "  , 15, 7, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[6].Max, NULL, NULL},
  {" 8" , 0, 8, D_LIST,     0,   0, 6,  2, false, AnalogOptionList, &analog.ac[7].Name, NULL, UpdateMinMax},
  {" "  , 8, 8, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[7].Min, NULL, NULL},
  {" "  , 15, 8, D_FLOAT, -250, 250, 1,  1, false, "%6.0f", &analog.ac[7].Max, NULL, NULL},
  {"Ch   Par    Min    Max", 0, 0, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Next page"          , 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, AnalogEntriesPage2, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry AnalogEntriesPage2[] = {
  {" Monitor inputs"     , 0, 1, D_DIALOG, 0, 0, 0, 0, false, NULL, &AnalogMonitorDialog, NULL, NULL},
  {" Calibrate channel"  , 0, 5, D_INT, 1, 8, 1, 20, false, "%2d", &CalAnalogChannel, NULL, AnalogChanCal},
  {" Save settings"      , 0, 6, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveAnalogSettings, NULL},
  {" Restore settings"   , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreAnalogSettings, NULL},
  {" First page"         , 0, 8, D_PAGE, 0, 0, 0, 0, false, NULL, AnalogEntriesPage1, NULL, NULL},
  {" Return to main menu", 0, 9, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox AnalogDialog = {
  {
    "Analog input parameters",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0,0,false, AnalogEntriesPage1
};

DialogBoxEntry AnalogMonitorEntries[] = {
  {" CH 1, Volts", 0, 1, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[0].ADCvalue, NULL, NULL},
  {" CH 2, Volts", 0, 2, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[1].ADCvalue, NULL, NULL},
  {" CH 3, Volts", 0, 3, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[2].ADCvalue, NULL, NULL},
  {" CH 4, Volts", 0, 4, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[3].ADCvalue, NULL, NULL},
  {" CH 5, Volts", 0, 5, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[4].ADCvalue, NULL, NULL},
  {" CH 6, Volts", 0, 6, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[5].ADCvalue, NULL, NULL},
  {" CH 7, Volts", 0, 7, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[6].ADCvalue, NULL, NULL},
  {" CH 8, Volts", 0, 8, D_FLOAT, 0, 0, 0, 16, true, "%6.1f", &analog.ac[7].ADCvalue, NULL, NULL},
  {" Return to analog menu"     , 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &AnalogDialog, NULL, NULL},
  {NULL},
};

DialogBox AnalogMonitorDialog = {
  {
    "Analog input monitor",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0,0,false, AnalogMonitorEntries
};

MenuEntry MEAnalogModule = {" Analog input module", M_DIALOG, 0, 0, 0, NULL, &AnalogDialog, NULL, NULL};

void UpdateMinMax(void)
{
  int    i, chan;
  float  fVal;
  String str;

  // Set min/max
  for (i = 0; i < analog.NumChannels; i++)
  {
    str = analog.ac[i].Name;
    if (str.startsWith("NONE"))
    {
      analog.ac[i].ValueUpdate = NULL;
    }
    if (str.startsWith("DCB"))
    {
      chan = str.substring(3).toInt();
      analog.ac[i].Chan = chan;
      if(DCbiasReadMin(chan, &fVal))
      {
         AnalogEntriesPage1[(i * 3) + 1].Min = (fVal) * 2.0;
         AnalogEntriesPage1[(i * 3) + 2].Min = (fVal) * 2.0;
      }
      if(DCbiasReadMax(chan, &fVal))
      {
         AnalogEntriesPage1[(i * 3) + 1].Max = (fVal) * 2.0;
         AnalogEntriesPage1[(i * 3) + 2].Max = (fVal) * 2.0;
      }
      // Set the value adjust pointer
      analog.ac[i].ValueUpdate = DCbiasSet;
    }
    if (str.startsWith("RFD"))
    {
      chan = str.substring(3).toInt();
      analog.ac[i].Chan = chan;
      // Set range
      AnalogEntriesPage1[(i * 3) + 1].Min = 0;
      AnalogEntriesPage1[(i * 3) + 2].Min = 0;
      AnalogEntriesPage1[(i * 3) + 1].Max = 100;
      AnalogEntriesPage1[(i * 3) + 2].Max = 100;
      // Set the value adjust pointer
      analog.ac[i].ValueUpdate = RFdrive;
    }
    if (str.startsWith("RFV"))
    {
      chan = str.substring(3).toInt();
      analog.ac[i].Chan = chan;
      // Set range
      AnalogEntriesPage1[(i * 3) + 1].Min = 0;
      AnalogEntriesPage1[(i * 3) + 2].Min = 0;
      AnalogEntriesPage1[(i * 3) + 1].Max = 400;
      AnalogEntriesPage1[(i * 3) + 2].Max = 400;
      // Set the value adjust pointer
      analog.ac[i].ValueUpdate = RFvoltage;
    }
  }
}

int ADS1115readAverage(int8_t adr, int8_t chan, int8_t num)
{
  int i,sum=0;

  for(i=0;i<num;i++)
  {
    if(adr == analog.ADC1add) sum += (int16_t)ads1->readADC_SingleEnded(chan);
    else sum += (int16_t)ads2->readADC_SingleEnded(chan);
  }
  return(sum/num);
}

// This function allows calibration of the seleted channel
void AnalogChanCal(void)
{
  ChannelCal CC;
  char       Name[20];
  int        b=0;

  // Set up the calibration data structure
  CC.ADCpointer = &ADS1115readAverage;
  CC.Min=0;
  CC.Max=10.0;
  CC.DACaddr=0;  
  if(CalAnalogChannel <= 4) CC.ADCaddr=analog.ADC1add;
  else CC.ADCaddr=analog.ADC2add;
  CC.DACout = NULL;
  CC.ADCreadback=&analog.ac[CalAnalogChannel-1].DCmon;
  // Define this channels name
  sprintf(Name,"       Channel %2d",CalAnalogChannel);
  // Calibrate this channel
  ChannelCalibrate(&CC, Name);
}

// Saves analog parameters to a file on the SD card. The file is named Analog.cfg"
void SaveAnalogSettings(void)
{
  File file;

  // Test SD present flag, exit if no card or it failed to init
  while (1)
  {
    if (!SDcardPresent) break;
    SD.begin(_sdcs);
    // Remove the existing file
    SD.remove("Analog.cfg");
    // Open file and write structure to disk
    if (!(file = SD.open("Analog.cfg", FILE_WRITE))) break;
    analog.Size = sizeof(Analog);
    file.write((byte *)&analog, sizeof(Analog));
    file.close();
    // Save finished, display message and exit
    DisplayMessage("Parameters Saved!", 2000);
    return;
  }
  // Save failed, display error message and exit
  DisplayMessage("Error saving!", 2000);
}

void RestoreAnalogSettings(void)
{
  RestoreAnalogSettings(false);
}

void RestoreAnalogSettings(bool NoDisplay)
{
  File file;
  Analog a;
  int  i, iVal;
  byte *b;

  while (1)
  {
    // Test SD present flag
    if (!SDcardPresent) break;
    SD.begin(_sdcs);
    // Open the file
    if (!(file = SD.open("Analog.cfg", FILE_READ))) break;
    // read the data
    b = (byte *)&a;
    for (i = 0; i < sizeof(Analog); i++)
    {
      if ((iVal = file.read()) == -1) break;
      b[i] = iVal;
    }
    file.close();
    // Copy to MIPS config struct
    memcpy(&analog, &a, a.Size);
    // Update the min / max values this also update the control function pointers
    UpdateMinMax();
    if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
    return;
  }
  if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

// Build a list of valid analog control options based on hardware in the MIPS system.
char *BuildAnalogOptionList(void)
{
  static String OptionList;
  char token[10];
  int   i;

  OptionList = "NONE";
  for (i = 0; i < NumberOfDCChannels; i++)
  {
    sprintf(token, ",DCB%1d", i + 1);
    OptionList += token;
  }
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    sprintf(token, ",RFD%1d", i + 1);
    OptionList += token;
  }
  for (i = 0; i < NumberOfRFChannels; i++)
  {
    sprintf(token, ",RFV%1d", i + 1);
    OptionList += token;
  }
  return (char *)OptionList.c_str();
}

// Read all the analog input channels and save results in data structure.
//  2/3x gain +/- 6.144V (default)
//  single ended operation means only 1/2 range of ADC is used 0 to 27,000 for 5 volt operation with 2/3 gain
void ReadAnalog(void)
{
  int i, iVal;

  if (analog.Enabled)
  {
    for (i = 0; i < analog.NumChannels; i++)
    {
      // Read raw ADC counts
      if (i < 4) iVal = (int16_t)ads1->readADC_SingleEnded(analog.ac[i].DCmon.Chan);
      else iVal = (int16_t)ads2->readADC_SingleEnded(analog.ac[i].DCmon.Chan);
      // Convert to voltage
      analog.ac[i].ADCvalue = Counts2Value(iVal, &analog.ac[i].DCmon);
    }
  }
}

// Set the control outputs based on the analog input voltage
// Input voltages are 0 to 10 volts. This function uses the min a max values to convert the input to
// an control setpoint and then calls the control function.
void AnalogControl(void)
{
  int i;
  float m, b, setpoint;

  if (analog.Enabled)
  {
    for (i = 0; i < analog.NumChannels; i++)
    {
      b = analog.ac[i].Min;
      m = (analog.ac[i].Max - analog.ac[i].Min) / 10.0;
      setpoint = analog.ac[i].ADCvalue * m + b;
      if (analog.ac[i].ValueUpdate != NULL) analog.ac[i].ValueUpdate(analog.ac[i].Chan, setpoint);
    }
  }
}

// This function is called on power up initilization after all other systems have been setup. This function
// will enable the analog input system if ADC(s) are found.
void Analog_init(void)
{
  int16_t adc;
  int     i, j, stat;

  // Read the configuration data from the SD. If is not found then use the default settings
  // If normal startup load the parameters from the SD card.
  if (NormalStartup)
  {
    RestoreAnalogSettings(true);
  }
  // Test for the presents of the analog input hardware. This hardware is connected to the second TWI channel
  // and uses the wire1 interface. This feature uses the Adafruit ADS1115 4 channel ADC breakout board. This board supprts
  // 4 channels and two boards can be used for a total of 8 analog inputs. TWI addresses are: 0x48 primary and 0x49 secondary
  pinMode(70, INPUT_PULLUP);
  pinMode(71, INPUT_PULLUP);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  ads1 = new Adafruit_ADS1115(analog.ADC1add);
  ads2 = new Adafruit_ADS1115(analog.ADC2add);
  ads1->begin();
  ads2->begin();
  Wire1.setClock(Wire1DefaultSpeed);
  adc = ads1->readADC_SingleEnded(0);
  stat = ads1->getStatus();
  adc = ads1->readADC_SingleEnded(0);
  stat |= ads1->getStatus();
  if (stat == 0)
  {
    analog.Enabled = true;
    analog.NumChannels = 4;
    adc = ads2->readADC_SingleEnded(0);
    stat = ads2->getStatus();
    adc = ads2->readADC_SingleEnded(0);
    stat |= ads2->getStatus();
    if (stat == 0) analog.NumChannels = 8;
  }
  if (analog.Enabled)
  {
    // Enable menu selections based on number of channels
    for (i = 0; i < 8; i++)
    {
      if ((analog.NumChannels - 1) < i)
      {
        AnalogMonitorEntries[i].Type = D_OFF;
        for (j = 0; j < 3; j++) AnalogEntriesPage1[j + (i * 3)].Type = D_OFF;
      }
    }
    // Build list of contol options
    AnalogOptionList = BuildAnalogOptionList();
    for (i = 0; i < 8; i++) AnalogEntriesPage1[i * 3].fmt = AnalogOptionList;
    UpdateMinMax();
    // Setup the menu
    AddMainMenuEntry(&MEAnalogModule);
    // Configure Threads
    AnalogThread.setName("Analog");
    AnalogThread.onRun(Analog_loop);
    AnalogThread.setInterval(100);
    // Add threads to the controller
    control.add(&AnalogThread);
    // Read the inputs now
    ReadAnalog();
  }
}

// The process is scheduled by the Analog_init function if the hardware is detected.
void Analog_loop(void)
{
  static int disIndex = 0;
  uint32_t   startTime;

  if (!analog.Enabled) return;
  // Read all the ADC input channels and save them in the data structure.
  // This TWI device can fail and take a long time to complete, so test the
  // function run time, if over 50 milli sec then kill the this thead and 
  // display an error message.
  startTime = millis();
  ReadAnalog();
  if((millis()-startTime) > 50000)
  {
    DisplayMessage("Analog module failed!",2000);
    AnalogThread.enabled = false;
    return;
  }
  // Adjust all the mapped values
  AnalogControl();
  // If we are displaying the analog values then update one each pass through this loop
  if (ActiveDialog->Entry == AnalogMonitorEntries)
  {
    DisplayDialogEntry(&AnalogMonitorDialog.w, &AnalogMonitorDialog.Entry[disIndex++], false);
    if(disIndex >= analog.NumChannels) disIndex = 0;
  }
}
