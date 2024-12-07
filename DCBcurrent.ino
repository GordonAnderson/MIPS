#include "DCBcurrent.h"
#include "Arduino.h"
#include <Wire.h>

// To do list
//  - Add the serial mute logic to all the commands
//  - Document the code a bit

#if DCBcurrent

DCBcurData     *dcbcurdata[4]   = {NULL,NULL,NULL,NULL};
Currents       *currents[4]     = {NULL,NULL,NULL,NULL};
Thread         DCBiasCurThread  = Thread();

DCBcurData dcbcurTemplate
{
  1,sizeof(DCBcurData),
  false,1,
  {
    {0,-5260,38300},
    {1,-5260,38300},
    {2,-5260,38300},
    {3,-5260,38300},
    {4,-5260,38300},
    {5,-5260,38300},
    {6,-5260,38300},
    {7,-5260,38300},
  }
};

const Commands DCBiasCurCmdArray[] = {
  {"CURLOAD",   CMDfunction,    1, (char *)dccurRestore},             // Load saved parameters from SD, ch = any channel on module
  {"CURSAVE",   CMDfunction,    1, (char *)dccurSave},                // Save parameters to the SD, ch = any channel on module
  {"SCURTST",   CMDfunctionStr, 2, (char *)dccurSetTest},
  {"GCURTST",   CMDfunction,    1, (char *)dccurGetTest},
  {"SCURLIM",   CMDfunctionStr, 2, (char *)dccurSetLim},
  {"GCURLIM",   CMDfunction,    1, (char *)dccurGetLim},
  {"GCURRENT",  CMDfunction,    1, (char *)dccurGetCur},
  {"CALDCBCUR",CMDfunction,    1, (char *)dccurCalCH},
  {0}
};

CommandList DCBiasCurList = {(Commands *)DCBiasCurCmdArray, NULL };

int ADS7828(TwoWire *wire, int8_t adr, int8_t chan, int num)
{
  int sum = 0;

  for(int i=0;i<num;i++) sum += ADS7828(wire,adr,chan);
  return sum/num;
}

void DCbiasCur_loop(void)
{
  float maxI   = 0;
  int   maxIch = 0;

  for(int brd=0;brd<4;brd++)
  {
    if(currents[brd] != NULL)
    {
      // Read and filter all the current values
      for(int i=0;i<8;i++)
      {
        float fval = Counts2Value(ADS7828(&Wire1, DCBCurAdd + (brd), dcbcurdata[brd]->curCH[i].Chan), &dcbcurdata[brd]->curCH[i]);
        currents[brd]->ch[i] = fval * FILTER + (1-FILTER) *  currents[brd]->ch[i];
      }
      // Limit test if enabled and power is on
      if((dcbcurdata[brd]->TestEna) && IsPowerON()) for(int i=0;i<8;i++)
      {
        // Find the maximum current level
        if(abs(currents[brd]->ch[i]) > dcbcurdata[brd]->TrigLevel)
        {
          if(abs(currents[brd]->ch[i]) > maxI)
          {
            maxI   = abs(currents[brd]->ch[i]);
            maxIch = DCBadd2chan(brd, i);
          }
        }
      }
    }
  }
  // If a limit happened then trip the supplies and popup a message
  if(maxI > 0)
  {
    // Turn off the power supply
    MIPSconfigData.PowerEnable = false;
    SetPowerSource();
    // Display a popup error message
    DisplayMessageButtonDismiss("DCBias Over Current!");
    LogMessage("DCBias over current!");
  }
}

// This command will enable the DCbias current monitor module for the board that contains
// the channel number passed. The ADS7828 is tested at the expected address (TWI), if found
// the module is enabled. The current monitor BNC output modules are connected to the WIRE1
// interface.
bool dcbcurEnable(int CH)
{
  int brd;

  Wire1.begin();
  // Get board address for the channel passed
  if((brd=DCbiasCH2Brd(CH-1)) == -1) return false;
  // Test if hardware is installed
  Wire1.beginTransmission(DCBCurAdd + (brd));
  if(Wire1.endTransmission()!=0)
  {
    // Try again
    Wire1.beginTransmission(DCBCurAdd + (brd));
    if(Wire1.endTransmission()!=0) return false;
  }
  // Allocate the data structure
  if(dcbcurdata[brd] == NULL)
  {
    // Allocate space for array
    dcbcurdata[brd] = new DCBcurData;
    if(dcbcurdata[brd] == NULL) return false;
    currents[brd] = new Currents;
    if(currents[brd] == NULL) return false;
    for(int i = 0;i<8;i++) currents[brd]->ch[i] = 0;
  }
  // If this is the first defined insert commands into the command processor
  int num = 0;
  for(int i = 0;i<4;i++) if(dcbcurdata[i] != NULL) num++;
  if(num == 1) 
  {
    // Add conmmands 
    AddToCommandList(&DCBiasCurList);
    // Start a thread to read the ADC channels
    // Configure Threads
    DCBiasCurThread.setName("DCbiasCur");
    DCBiasCurThread.onRun(DCbiasCur_loop);
    DCBiasCurThread.setInterval(100);
    // Add threads to the controller
    control.add(&DCBiasCurThread);
  }
  // Set default settings
  *dcbcurdata[brd] = dcbcurTemplate;
  // Load the settings 
  dccurLoad(brd);
  return true;
}

int checkCurBoard(int CH)
{
  int brd=DCbiasCH2Brd(CH-1);
  if(dcbcurdata[brd] == NULL) { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
  if(brd == -1) { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
  return brd;
}

int checkCurBoard(char *chan)
{
  String token = chan;
  return checkCurBoard(token.toInt()-1);
}

bool dccurLoad(int brd)
{
  DCBcurData data;
  File file;
  String  filename = "CURPARM";
 
  filename+=char('0' + brd);
  if(!SDcardPresent) return false;
  SD.begin(_sdcs);
  // Open the file
  if((file = SD.open(filename.c_str(), FILE_READ)))
  {
    if(file.read(&data, sizeof(DCBcurData)) == sizeof(DCBcurData))
    {
      if((data.rev == 1) && (data.size == sizeof(DCBcurData)))
      {
        *dcbcurdata[brd] = data;
        return true;
      }
    }
  }
  return false;
}

void dccurRestore(int ch) 
{
  int brd;
  // convert channel number to board address
  if((brd=checkCurBoard(ch--)) == -1) return;
  if(dccurLoad(brd)) { SendACK; }
  else BADARG;
}

void dccurSave(int ch) 
{
  File file;
  String  filename = "CURPARM";
  int brd;
  // convert channel number to board address
  if((brd=checkCurBoard(ch--)) == -1) return;
  filename+=char('0' + brd);
  // Test SD present flag, exit and NAK if no card or it failed to init
  if (!SDcardPresent) ERR(ERR_NOSDCARD);
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(filename.c_str());
  // Open file and write config structure to disk
  if (!(file = SD.open(filename.c_str(), FILE_WRITE))) ERR(ERR_CANTCREATEFILE);
  file.write((byte *)dcbcurdata[brd], sizeof(DCBcurData));
  file.close();
  SendACK;
}
void dccurSetTest(char *chan, char *state) 
{
  int brd,ch;
  String token = chan;

  // convert channel number to board address
  if((brd=checkCurBoard(ch = token.toInt())) == -1) return;
  ch--;
  token = state;
  if(token == "TRUE") dcbcurdata[brd]->TestEna = true;
  else if(token == "FALSE") dcbcurdata[brd]->TestEna = false;
  else BADARG;
  SendACK;
}
void dccurGetTest(int ch) 
{
  int brd;
  // convert channel number to board address
  if((brd=checkCurBoard(ch--)) == -1) return;
  SendACKonly;
  if(dcbcurdata[brd]->TestEna) serial->println("TRUE");
  else serial->println("FALSE");
}
void dccurSetLim(char *chan, char *value) 
{
  int brd,ch;
  String token = chan;

  // convert channel number to board address
  if((brd=checkCurBoard(ch = token.toInt())) == -1) return;
  ch--;
  token = value;
  dcbcurdata[brd]->TrigLevel = token.toFloat();
  SendACK;
}
void dccurGetLim(int ch) 
{
  int brd;
  // convert channel number to board address
  if((brd=checkCurBoard(ch--)) == -1) return;
  SendACKonly;
  serial->println(dcbcurdata[brd]->TrigLevel);
}
void dccurGetCur(int ch) 
{
  int brd;
  // convert channel number to board address
  if((brd=checkCurBoard(ch--)) == -1) return;
  SendACKonly;
  serial->println(currents[brd]->ch[ch & 7]);
}
void dccurCalCH(int ch) 
{
  int brd;

  // convert channel number to board address
  if((brd=checkCurBoard(ch)) == -1) return;
  ch--;
  // Apply voltage 1 and read the ADC channels raw data
  float val1 = UserInputFloat("Apply current 1 and enter value, mA : ");
  int adc1   = ADS7828(&Wire1, DCBCurAdd + (brd), dcbcurdata[brd]->curCH[ch & 7].Chan);
  // Apply voltage 2 and read the ADC channels raw data
  float val2 = UserInputFloat("Apply current 2 and enter value, mA : ");
  int adc2   = ADS7828(&Wire1, DCBCurAdd + (brd), dcbcurdata[brd]->curCH[ch & 7].Chan);
  // Calculate and apply calibration parameters
  dcbcurdata[brd]->curCH[ch & 7].m = ((float)adc1 - (float)adc2) / (val1 - val2);
  dcbcurdata[brd]->curCH[ch & 7].b = (float)adc1 - dcbcurdata[brd]->curCH[ch & 7].m * val1;
  serial->println(dcbcurdata[brd]->curCH[ch & 7].m);
  serial->println(dcbcurdata[brd]->curCH[ch & 7].b);
}

#endif

