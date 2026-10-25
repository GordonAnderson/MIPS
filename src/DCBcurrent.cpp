#include "Variants.h"
#include "DCBcurrent.h"
#include "Arduino.h"
#include <Wire.h>
#include "SD.h"

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

// Forward declarations
extern ThreadController control;
extern bool SDcardPresent;
bool IsPowerON(void);
void SetPowerSource(void);
int DCBadd2chan(int brd, int ch);
void DCbiasCur_loop(void);
bool dccurLoad(int brd);
void dccurRestore(int ch);
void dccurSave(int ch);
void dccurSetTest(char *chan, char *state);
void dccurGetTest(int ch);
void dccurSetLim(char *chan, char *value);
void dccurGetLim(int ch);
void dccurGetCur(int ch);
void dccurCalCH(int ch);

const Commands DCBiasCurCmdArray[] = {
// Start of command block
// 
// DC bias current monitor output module commands
// 
  {"CURLOAD",   CMDfunction,    1, (char *)dccurRestore},             // Load saved parameters from SD, ch = any channel on module
  {"CURSAVE",   CMDfunction,    1, (char *)dccurSave},                // Save parameters to the SD, ch = any channel on module
  {"SCURTST",   CMDfunctionStr, 2, (char *)dccurSetTest},             //
  {"GCURTST",   CMDfunction,    1, (char *)dccurGetTest},             //
  {"SCURLIM",   CMDfunctionStr, 2, (char *)dccurSetLim},              //
  {"GCURLIM",   CMDfunction,    1, (char *)dccurGetLim},              //
  {"GCURRENT",  CMDfunction,    1, (char *)dccurGetCur},              //
  {"CALDCBCUR",CMDfunction,    1, (char *)dccurCalCH},                //
// End of table marker
  {0}
};

CommandList DCBiasCurList = {(Commands *)DCBiasCurCmdArray, NULL };

// I2C transport for the current monitor ADCs, hardware Wire1 or soft I2C on the Wire1 pins,
// selected with DCBcurUseSoftI2C in DCBcurrent.h. curProbe() tests for a device and curADC()
// reads one ADS7828 channel, returning 16 bit left justified counts or -1 on error.
#if DCBcurUseSoftI2C
#include "soft_i2c.h"

static Pio      *curSDApio, *curSCLpio;
static uint32_t  curSDAmask, curSCLmask;

// Open drain line control using the Arduino pin functions, the same calls as the DSET/DWRITE
// commands. The pin's stored output value is held LOW (set in curTakePins), so pinMode OUTPUT drives
// the line low and pinMode INPUT_PULLUP releases it to the pull ups. Reads are only done while the
// pin is an input, digitalRead of an output pin returns the stored value, not the pin level.
static void curSCLfloat(void) { pinMode(DCBcurSoftSCL, INPUT_PULLUP); }
static void curSCLlow(void)   { pinMode(DCBcurSoftSCL, OUTPUT); }
static bool curSCLread(void)  { return digitalRead(DCBcurSoftSCL) == HIGH; }
static void curSDAfloat(void) { pinMode(DCBcurSoftSDA, INPUT_PULLUP); }
static void curSDAlow(void)   { pinMode(DCBcurSoftSDA, OUTPUT); }
static bool curSDAread(void)  { return digitalRead(DCBcurSoftSDA) == HIGH; }
static void curDelay(void)    { delayMicroseconds(DCBcurSoftI2CdelayUS); }

static const soft_i2c_hal_t curHAL = {curSCLfloat, curSCLlow, curSCLread,
                                      curSDAfloat, curSDAlow, curSDAread,
                                      curDelay, DCBcurSoftI2Cstretch};

// Takes the Wire1 pins for soft I2C: PIO controlled, multi drive off, output data 0, output disabled
// (released high) with the internal pull ups enabled. The current monitors are the only devices on
// these pins, so the soft driver owns them
// and they are not handed back to the TWI0 peripheral. This is done at the start of every
// transaction in case another driver's init calls Wire1.begin() and reassigns the pins.
static void curTakePins(void)
{
  static bool inited = false;

  if(!inited)
  {
    inited     = true;
    curSDApio  = g_APinDescription[DCBcurSoftSDA].pPort;
    curSDAmask = g_APinDescription[DCBcurSoftSDA].ulPin;
    curSCLpio  = g_APinDescription[DCBcurSoftSCL].pPort;
    curSCLmask = g_APinDescription[DCBcurSoftSCL].ulPin;
    pmc_enable_periph_clk(g_APinDescription[DCBcurSoftSDA].ulPeripheralId);   // Needed to read the pins
    pmc_enable_periph_clk(g_APinDescription[DCBcurSoftSCL].ulPeripheralId);
  }
  // Release both lines, then store LOW as the output value so pinMode OUTPUT drives low. Writing LOW
  // to an input pin turns its pull up off, so release again afterwards to turn the pull ups back on.
  pinMode(DCBcurSoftSDA, INPUT_PULLUP);
  pinMode(DCBcurSoftSCL, INPUT_PULLUP);
  digitalWrite(DCBcurSoftSDA, LOW);
  digitalWrite(DCBcurSoftSCL, LOW);
  pinMode(DCBcurSoftSDA, INPUT);
  pinMode(DCBcurSoftSCL, INPUT);
  pinMode(DCBcurSoftSDA, INPUT_PULLUP);
  pinMode(DCBcurSoftSCL, INPUT_PULLUP);
}

static void curBusInit(void)
{
  curTakePins();
  soft_i2c_bus_clear(&curHAL);
}

static bool curProbe(int8_t adr)
{
  curTakePins();
  return soft_i2c_write(&curHAL, adr, NULL, 0);
}

// Released (idle) bus line levels, both should read 1 if the pull ups are present
static bool curSDAidle(void) { curTakePins(); curSDAfloat(); curSCLfloat(); delayMicroseconds(10); return curSDAread(); }
static bool curSCLidle(void) { curTakePins(); curSDAfloat(); curSCLfloat(); delayMicroseconds(10); return curSCLread(); }

static int curADC(int8_t adr, int8_t chan)
{
  uint8_t cmd = 0x8C | ((chan & 0x06) << 3) | ((chan & 0x01) << 6);
  uint8_t buf[2];
  int     val = -1;

  curTakePins();
  if(soft_i2c_write(&curHAL, adr, &cmd, 1) && soft_i2c_read(&curHAL, adr, buf, 2))
  {
    val = ((buf[0] << 8) | buf[1]) & 0xFFF;
    val <<= 4;
  }
  return val;
}
#else
static void curBusInit(void) { Wire1.begin(); }

static bool curProbe(int8_t adr)
{
  Wire1.beginTransmission(adr);
  return Wire1.endTransmission() == 0;
}

static int curADC(int8_t adr, int8_t chan) { return ADS7828(&Wire1, adr, chan); }
#endif

// CURSCAN diagnostic, in the main command table since the current monitor commands are only added
// once a module is found. Reports the interface in use, the idle bus line levels (soft I2C only)
// and every address on the current monitor bus that acknowledges.
void curScan(void)
{
  SendACKonly;
  if(SerialMute) return;
  curBusInit();
#if DCBcurUseSoftI2C
  serial->print("Soft I2C, idle SDA = ");
  serial->print(curSDAidle());
  serial->print(", SCL = ");
  serial->println(curSCLidle());
  // PIO state for the two pins, 1 = set: PSR PIO controls the pin (0 = peripheral), OSR output
  // enabled, MDSR open drain, ODSR output data, PDSR actual pin level
  curTakePins();
  const char *nm[2]    = {"SDA pin ", "SCL pin "};
  int         pn[2]    = {DCBcurSoftSDA, DCBcurSoftSCL};
  Pio        *pio[2]   = {curSDApio, curSCLpio};
  uint32_t    msk[2]   = {curSDAmask, curSCLmask};
  for(int i=0;i<2;i++)
  {
    serial->print(nm[i]);
    serial->print(pn[i]);
    serial->print(": PSR ");  serial->print((pio[i]->PIO_PSR  & msk[i]) != 0);
    serial->print(" OSR ");   serial->print((pio[i]->PIO_OSR  & msk[i]) != 0);
    serial->print(" MDSR ");  serial->print((pio[i]->PIO_MDSR & msk[i]) != 0);
    serial->print(" ODSR ");  serial->print((pio[i]->PIO_ODSR & msk[i]) != 0);
    serial->print(" PDSR ");  serial->print((pio[i]->PIO_PDSR & msk[i]) != 0);
    // Drive the line low with the same function the I2C code uses and read the actual pin level
    // back from the PIO, should read 0
    if(i == 0) curSDAlow(); else curSCLlow();
    delayMicroseconds(10);
    bool lv = (pio[i]->PIO_PDSR & msk[i]) != 0;
    if(i == 0) curSDAfloat(); else curSCLfloat();
    serial->print(", driven low reads "); serial->println(lv);
  }
#else
  serial->println("Hardware Wire1");
#endif
  serial->print("Found:");
  for(int adr=0x08; adr<0x78; adr++)
  {
    if(curProbe(adr))
    {
      serial->print(" 0x");
      serial->print(adr, HEX);
    }
  }
  serial->println("");
}

int ADS7828(TwoWire *wire, int8_t adr, int8_t chan, int num)
{
  int sum = 0;

  for(int i=0;i<num;i++) sum += ADS7828(wire,adr,chan);
  return sum/num;
}

void DCbiasCur_loop(void)
{
  float maxI   = 0;

  for(int brd=0;brd<4;brd++)
  {
    if(currents[brd] != NULL)
    {
      // Read and filter all the current values
      for(int i=0;i<8;i++)
      {
        float fval = Counts2Value(curADC(DCBCurAdd + (brd), dcbcurdata[brd]->curCH[i].Chan), &dcbcurdata[brd]->curCH[i]);
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

  curBusInit();
  // Get board address for the channel passed
  if((brd=DCbiasCH2Brd(CH-1)) == -1) return false;
  // Test if hardware is installed, try twice
  if(!curProbe(DCBCurAdd + (brd)))
  {
    if(!curProbe(DCBCurAdd + (brd))) return false;
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
  int adc1   = curADC(DCBCurAdd + (brd), dcbcurdata[brd]->curCH[ch & 7].Chan);
  // Apply voltage 2 and read the ADC channels raw data
  float val2 = UserInputFloat("Apply current 2 and enter value, mA : ");
  int adc2   = curADC(DCBCurAdd + (brd), dcbcurdata[brd]->curCH[ch & 7].Chan);
  // Calculate and apply calibration parameters
  dcbcurdata[brd]->curCH[ch & 7].m = ((float)adc1 - (float)adc2) / (val1 - val2);
  dcbcurdata[brd]->curCH[ch & 7].b = (float)adc1 - dcbcurdata[brd]->curCH[ch & 7].m * val1;
  serial->println(dcbcurdata[brd]->curCH[ch & 7].m);
  serial->println(dcbcurdata[brd]->curCH[ch & 7].b);
}

#endif

