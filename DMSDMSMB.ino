#include "DMSDMSMB.h"
#if DMSDMSMB
#include "DMSDMSMB.h"
#include "Arduino.h"
#include <Wire.h>

MIPStimer *DMSDMSScanClock = NULL;

int CVBIASbrd    = -1;
int WAVEFORMSbrd = -1;

int DMSDMSchannel = 1;
int DMSDMSchannelRequest = 1;

//MIPS Threads
Thread DMSDMSthread  = Thread();

CVBIASdata        cvbiasdata;
CVBIASChanParams  cvbiasCH;
CVBIASstate       cvbiasState;
CVBIASreadBacks   cvbiasRB[2],cvbiasrb,cvbiasrbTemp;

WAVEFORMSdata        waveformsdata;
WAVEFORMSChanParams  waveformsCH;
WAVEFORMSstate       waveformsState;
WAVEFORMSreadBacks   waveformsRB[2],waveformsrb,waveformsrbTemp;

float        PosCurrent = 0;               // Positive electrometer channel
float        NegCurrent = 0;               // Negative electrometer channel

float        power=0;


extern DialogBoxEntry DMSDMSsettings[];
extern DialogBoxEntry DMSDMSscan[];
extern DialogBoxEntry ElectrometerSettings[];
DialogBoxEntry DMSDMSentries[] = {
  {" Channel"            , 0, 1, D_INT  , 1, 2  , 1, 21, false, "%2d", &DMSDMSchannelRequest, NULL, setDMSDMSch},
  {" Enable"             , 0, 2, D_ONOFF, 0, 1  , 1, 20, false, NULL, &cvbiasCH.Enable, NULL, NULL},
  {" Mode"               , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &waveformsCH.Mode, NULL, NULL},
  {" Drive"              , 0, 4, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &waveformsCH.Drive, NULL, NULL},
  {"        Request  Actual", 0, 5, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Vrf"                , 0, 6, D_FLOAT, 0, 2000, 10, 8, false, "%5.0f", &waveformsCH.Vrf, NULL, NULL},
  {" CV"                 , 0, 7, D_FLOAT, -48, 48, 0.1, 8, false, "%5.1f", &cvbiasCH.CV, NULL, NULL},
  {""                    , 0, 6, D_FLOAT, 0, 2000, 10, 18, true, "%5.0f", &waveformsrb.Vrf, NULL, NULL},
  {""                    , 0, 7, D_FLOAT, -48, 48, 0.1, 18, true, "%5.1f", &cvbiasrb.CV, NULL, NULL},
  {" Power"              , 0, 8, D_FLOAT, 0, 100, 1, 18, true, "%5.1f", &power, NULL, NULL},
  {" Settings"           , 0, 9, D_PAGE,  0, 0  , 0, 0,  false, NULL, DMSDMSsettings, NULL, NULL},
  {" Electrometer"       , 0,10, D_PAGE,  0, 0  , 0, 0,  false, NULL, ElectrometerSettings, NULL, NULL},
  {" Return to main menu", 0,11, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry DMSDMSsettings[] = {
  {" Scan"               , 0, 1, D_PAGE,  0, 0  , 0, 0,  false, NULL, DMSDMSscan, NULL, NULL},
  {" Frequency"          , 0, 2, D_INT  , 250000, 2000000  , 1000, 16, false, "%6d", &waveformsCH.Freq, NULL, NULL},
  {" Duty cycle"         , 0, 3, D_INT , 0, 90  , 1, 21, false, "%2d", &waveformsCH.Duty, NULL, NULL},
  {" Max drive"          , 0, 4, D_FLOAT, 0, 100 , 1, 20, false, "%3.0f", &waveformsCH.MaxDrive, NULL, NULL},
  {" Max power"          , 0, 5, D_FLOAT, 0, 50 , 1, 20, false, "%3.0f", &waveformsCH.MaxPower, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveDMSDMSsettings, NULL},
  {" Restore settings"   , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, static_cast<void (*)(void)>(&RestoreDMSDMSsettings), NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, DMSDMSentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry DMSDMSscan[] = {
  {" Start CV"           , 0, 1, D_FLOAT, -48, 48 , 0.1, 18, false, "%5.1f", &cvbiasCH.CVstart, NULL, NULL},
  {" End CV"             , 0, 2, D_FLOAT, -48, 48 , 0.1, 18, false, "%5.1f", &cvbiasCH.CVend, NULL, NULL},
  {" Start Vrf"          , 0, 3, D_FLOAT, 0, 2000 , 10,  19, false, "%4.0f", &waveformsCH.VRFstart, NULL, NULL},
  {" End Vrf"            , 0, 4, D_FLOAT, 0, 2000 , 10,  19, false, "%4.0f", &waveformsCH.VRFend, NULL, NULL},
  {" Num steps"          , 0, 5, D_INT, 10, 2000 , 1,  19, false, "%4d", &cvbiasdata.Steps, NULL, NULL},
  {" Step duration"      , 0, 6, D_INT, 1, 2000 , 1,  19, false, "%4d", &cvbiasdata.StepDuration, NULL, NULL},
  {" Start scan"         , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, DMSDMSstartScan, NULL},
  {" Abort scan"         , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, DMSDMSscanAbort, NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, DMSDMSentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry ElectrometerSettings[] = {
  {" Pos current, pA"    , 0, 1, D_FLOAT, 0, 0 , 0, 17, true, "%6.2f", &PosCurrent, NULL, NULL},
  {" Neg current, pA"    , 0, 2, D_FLOAT, 0, 0 , 0, 17, true, "%6.2f", &NegCurrent, NULL, NULL},
  {" Pos offset,V"       , 0, 3, D_FLOAT, 0, 5.0 , 0.01, 18, false, "%5.3f", &cvbiasdata.electrometer.PosOffset, NULL, NULL},
  {" Neg offset,V"       , 0, 4, D_FLOAT, 0, 5.0 , 0.01, 18, false, "%5.3f", &cvbiasdata.electrometer.NegOffset, NULL, NULL},
  {" Zero"               , 0, 5, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, ZeroElectrometer, NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, DMSDMSentries, NULL, NULL},
  {NULL},
};

DialogBox DMSDMSdialog = {
  {
    "DMSDMS control params",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0,false, DMSDMSentries
};

MenuEntry MEDMSDMSmodule = {" DMSDMS", M_DIALOG, 0, 0, 0, NULL, &DMSDMSdialog, NULL, NULL};

const Commands DMSDMSCmdArray[] = {
  {"SDDENA",  CMDfunctionStr, 2, (char *)SetDMSDMSEnable},           // Set module number to TRUE to enable FAIMS
  {"GDDENA",  CMDfunction, 1, (char *)ReturnDMSDMSEnable},           // Returns enable status for selected module
  {"SDDMODE", CMDfunctionStr, 2, (char *)SetDMSDMSMode},             // Set module number to TRUE to enable closed loop control of Vrf
  {"GDDMODE", CMDfunction, 1, (char *)ReturnDMSDMSMode},             // Returns mode control for selected module
  {"SDDFREQ", CMDfunction, 2, (char *)SetDMSDMSFreq},                // Set FAIMS frequency, in Hz for seletced module
  {"GDDFREQ", CMDfunction, 1, (char *)ReturnDMSDMSFreq},             // Returns frequency for the selected module
  {"SDDDUTY", CMDfunction, 2, (char *)SetDMSDMSDuty},                // Set FAIMS duty cycle in percent for seletced module
  {"GDDDUTY", CMDfunction, 1, (char *)ReturnDMSDMSDuty},             // Returns duty cycle for selected module
  {"SDDDRV", CMDfunctionStr,2, (char *)SetDMSDMSDrive},              // Set FAIMS drive level in percent for the selected module
  {"GDDDRV", CMDfunction,1, (char *)ReturnDMSDMSDrive},              // Returns drive level for the selected module
  {"GDRVV", CMDfunction,  1, (char *)ReturnDMSDMSDriveV},            // Returns voltage level into drive FET, V for the selected module
  {"GDRVI", CMDfunction,  1, (char *)ReturnDMSDMSDriveI},            // Returns current into drive FET, mA for the selected module
  {"SVRF", CMDfunctionStr, 2, (char *)SetDMSDMSVrf},                 // Sets the drive level setpoint to achieve the desired voltage for the selected module
  {"GVRF", CMDfunction,  1, (char *)ReturnDMSDMSVrf},                // Returns the Vrf peak voltage for the selected module
  {"GVRFV", CMDfunction,  1, (char *)ReturnDMSDMSVrfV},              // Returns the Vrf peak voltage readback for the selected module
  {"GPWR", CMDfunction,  1, (char *)ReturnDMSDMSPWR},                // Returns the power in watts for the selected module
  {"SVRFN", CMDfunctionStr, 2, (char *)SetDMSDMSVrfNow},             // Sets the drive level setpoint achieve the desired voltage for the selected module
                                                                     // and adjusts the drive level to reach the setpoint
  {"SVRFT", CMDfunctionStr, 2, (char *)SetDMSDMSVrfTable},           // Sets the drive level setpoint achieve the desired voltage for the selected module
                                                                     // and adjusts the drive level to reach the setpoint using the drive level table
  {"GENVRFTBL", CMDfunction, 1, (char *)GenerateVrfTable},           // This function will generate the Vrf lookup table
  // DMSDMS Limits
  {"SDDMAXDRV",CMDfunctionStr,2, (char *)SetDMSDMSMaxDrive},         // Set the maximum drive level allowed for the selected module
  {"GDDMAXDRV",CMDfunction, 1,(char *)ReturnDMSDMSMaxDrive},         // Returns the maximum drive level for the selected module
  {"SDDMAXPWR",CMDfunctionStr,2, (char *)SetDMSDMSMaxPower},         // Set the maximum power allowed for the selected module
  {"GDDMAXPWR",CMDfunction,1,(char *)ReturnDMSDMSMaxPower},          // Returns the maximum power limit for the selected module
  // DMSDMS DC bias commands
  {"SCV", CMDfunctionStr,  2, (char *)SetDMSDMSCV},                  // Set the current CV for the seletced module
  {"GCV", CMDfunction,  1, (char *)ReturnDMSDMSCV},                  // Return the current CV for the seletced module
  {"GCVV", CMDfunction, 1, (char *)ReturnDMSDMSCVrb},                // Return the CV readback value for the seletced module
  {"SBIAS", CMDfunctionStr,  2, (char *)SetDMSDMSBIAS},              // Set the current BIAS for the seletced module
  {"GBIAS", CMDfunction,  1, (char *)ReturnDMSDMSBIAS},              // Return the current BIAS for the seletced module
  {"GBIASV", CMDfunction,  1, (char *)ReturnDMSDMSBIASrb},           // Set the current BIAS for the seletced module 
  // DMSDMS scanning commands
  {"SDDCVSTRT",  CMDfunctionStr, 2, (char *)SetDMSDMSCVstart},       // set the CV scan start voltage for the seletced module 
  {"GDDCVSTRT",  CMDfunction, 1, (char *)ReturnDMSDMSCVstart},       // returns the CV scan start voltage for the seletced module 
  {"SDDCVEND",   CMDfunctionStr, 2, (char *)SetDMSDMSCVend},         // set the CV scan end voltage for the seletced module 
  {"GDDCVEND",   CMDfunction, 1, (char *)ReturnDMSDMSCVend},         // returns the CV scan end voltage for the seletced module 
  {"SDDVRFSTRT", CMDfunctionStr, 2, (char *)SetDMSDMSVRFstart},      // set the Vrf scan start voltage for the seletced module 
  {"GDDVRFSTRT", CMDfunction, 1, (char *)ReturnDMSDMSVRFstart},      // returns the Vrf scan start voltage for the seletced module 
  {"SDDVRFEND",  CMDfunctionStr, 2, (char *)SetDMSDMSVRFend},        // set the Vrf scan end voltage for the seletced module 
  {"GDDVRFEND",  CMDfunction, 1, (char *)ReturnDMSDMSVRFend},        // returns the Vrf scan end voltage for the seletced module 
  {"SDDSTEPDUR", CMDfunction, 1, (char *)SetDMSDMSStepDuration},     // set the scan step duration in mS 
  {"GDDSTEPDUR", CMDint, 0, (char *)&cvbiasdata.StepDuration},       // returns the scan step duration in mS 
  {"SDDNUMSTP",  CMDfunction, 1, (char *)SetDMSDMSSteps},            // set the scan number of steps
  {"GDDNUMSTP",  CMDint, 0, (char *)&cvbiasdata.Steps},              // returns the scan number of steps 
  {"DDSCNSTRT",  CMDfunction, 0, (char *)InitDMSDMSScan},            // Start the scan 
  {"DDSCNSTP",  CMDfunction, 0, (char *)StopDMSDMSScan},             // Stop a scan that is in process
/*
  {"SFBADCSMP",  CMDint, 1, (char *)&NumSamples},                    // Set the number of adc sample to average, 1 to 16 
  {"GFBADCSMP",  CMDint, 0, (char *)&NumSamples},                    // Returns the number of adc sample to average, 1 to 16 
*/
  // Electrometer commands
  {"SELTMTRM4",  CMDfunctionStr, 1, (char *)SetEMRTM4enable},                    // Set the M4 electrometer enabled flag, TRUE or FALSE 
  {"GELTMTRM4",  CMDbool, 0, (char *)&cvbiasdata.electrometer.M4ena},            // Return the M4 electrometer enable flag
  {"GELTMTRPOS",  CMDfloat, 0, (char *)&PosCurrent},                             // Return the electrometer positive channel
  {"GELTMTRNEG",  CMDfloat, 0, (char *)&NegCurrent},                             // Return the electrometer negative channel
  {"SELTMTRPOSOFF",  CMDfunctionStr, 1, (char *)SetEMRTposOff},                  // Set the electrometer positive channel offset
  {"GELTMTRPOSOFF",  CMDfloat, 0, (char *)&cvbiasdata.electrometer.PosOffset},   // Returns the electrometer positive channel offset
  {"SELTMTRNEGOFF",  CMDfunctionStr, 1, (char *)SetEMRTnegOff},                  // Set the electrometer negative channel offset
  {"GELTMTRNEGOFF",  CMDfloat, 0, (char *)&cvbiasdata.electrometer.NegOffset},   // Returns the electrometer negative channel offset
  {"SELTMTRPOSZERO",  CMDfunctionStr, 1, (char *)SetEMRTposZero},                // Set the electrometer positive channel zero
  {"GELTMTRPOSZERO",  CMDfloat, 0, (char *)&cvbiasdata.electrometer.PosZero},    // Returns the electrometer positive channel zero
  {"SELTMTRNEGZERO",  CMDfunctionStr, 1, (char *)SetEMRTnegZero},                // Set the electrometer negative channel zero
  {"GELTMTRNEGZERO",  CMDfloat, 0, (char *)&cvbiasdata.electrometer.NegZero},    // Returns the electrometer negative channel zero
  {"ELTMTRZERO",  CMDfunction, 0, (char *)SetEMRTzero},                          // Execute the electrometer zero procedure
  // Fragmentor interface commands. This a RS232 interface using RX0 and TX0 of the MIPS ARM processor.
  // The fragmentor commands are sent using the SFRAG command and the reply is relayed through the MIPS
  // host communications channel. 
  {"SFRAG", CMDfunctionLine, 0, (char *)SendFrag},                               // Send command through fragmentor interface
  // HV supply commands
  {"DDHVINT",  CMDfunction,    0, (char *)hvmInit},                          // DMS/DMS High voltage supply init
  {"SDDHVENA", CMDfunctionStr, 1, (char *)hvmSetEnable},                     // Set DMS/DMS High voltage supply enable, TRUE or FALSE
  {"GDDHVENA", CMDfunction,    0, (char *)hvmGetEnable},                     // Get DMS/DMS High voltage supply enable, TRUE or FALSE
  {"SDDHV",    CMDfunctionStr, 1, (char *)hvmSetVoltage},                    // Set DMS/DMS High voltage supply voltage
  {"GDDHV",    CMDfunction,    0, (char *)hvmGetVoltage},                    // Get DMS/DMS High voltage supply voltage
  {"GDDHVA",   CMDfunction,    0, (char *)hvmGetReadback},                   // Get DMS/DMS High voltage supply voltage readback
  {0}
};

CommandList DMSDMSCmdList = { (Commands *)DMSDMSCmdArray, NULL };

void setDMSDMSch(void)
{
  cvbiasdata.channel[DMSDMSchannel-1] = cvbiasCH;
  waveformsdata.channel[DMSDMSchannel-1] = waveformsCH;
  DMSDMSchannel = DMSDMSchannelRequest;
  cvbiasCH    = cvbiasdata.channel[DMSDMSchannel-1];
  waveformsCH = waveformsdata.channel[DMSDMSchannel-1];
  cvbiasrb    = cvbiasRB[DMSDMSchannel-1];
  waveformsrb = waveformsRB[DMSDMSchannel-1];
  DialogBoxDisplay(&DMSDMSdialog);
}

void DMSDMSstartScan(void)
{
  StartDMSDMSScan();
}
void DMSDMSscanAbort(void)
{
  AbortDMSDMSScan();
}
void ZeroElectrometer(void)
{
  TWIcmd(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_ELEC_ZERO);  
}

// Write the current board parameters to the EEPROM on the ARB board.
void SaveDMSDMSsettings(void)
{
  SelectBoard(CVBIASbrd);
  cvbiasdata.Size = sizeof(CVBIASdata);
  cvbiasdata.Signature = SIGNATURE;
  if(!WriteEEPROM(&cvbiasdata, cvbiasdata.TWIadd, 0, sizeof(CVBIASdata)) == 0)
  {
    DisplayMessage("Unable to Save!", 2000);
    return;
  }
  SelectBoard(WAVEFORMSbrd);
  waveformsdata.Size = sizeof(WAVEFORMSdata);
  waveformsdata.Signature = SIGNATURE;
  if(WriteEEPROM(&waveformsdata, waveformsdata.TWIadd, 0, sizeof(WAVEFORMSdata)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

bool RestoreDMSDMSsettings(int brd, CVBIASdata  *cvd,bool NoDisplay)
{
  CVBIASdata d;

  SelectBoard(brd);
  if (ReadEEPROM(&d, cvd->TWIadd, 0, sizeof(CVBIASdata)) == 0)
  {
    if ((strcmp(d.Name, "CVBIAS") == 0) && (d.Signature == SIGNATURE))
    {
      // Here if the name matches so copy the data to the operating data structure
      if (d.Size > sizeof(CVBIASdata)) d.Size = sizeof(CVBIASdata);
      d.TWIadd = cvd->TWIadd;
      memcpy(cvd, &d, d.Size);
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      return true;
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
  return false;
}
bool RestoreDMSDMSsettings(int brd, WAVEFORMSdata  *wfd,bool NoDisplay)
{
  WAVEFORMSdata d;

  SelectBoard(brd);
  if (ReadEEPROM(&d, wfd->TWIadd, 0, sizeof(WAVEFORMSdata)) == 0)
  {
    if ((strcmp(d.Name, "WAVEFORMS") == 0) && (d.Signature == SIGNATURE))
    {
      // Here if the name matches so copy the data to the operating data structure
      if (d.Size > sizeof(WAVEFORMSdata)) d.Size = sizeof(WAVEFORMSdata);
      d.TWIadd = wfd->TWIadd;
      memcpy(wfd, &d, d.Size);
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
      return true;
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
  return false;
}
void RestoreDMSDMSsettings(void)
{
  if(!RestoreDMSDMSsettings(CVBIASbrd,    &cvbiasdata,   true))
  {
    DisplayMessage("Unable to Restore!", 2000);
    return;
  }
  if(!RestoreDMSDMSsettings(WAVEFORMSbrd, &waveformsdata,true))
  {
    DisplayMessage("Unable to Restore!", 2000);
    return;
  }
  DisplayMessage("Parameters Restored!", 2000);
}

void DMSDMS_init(void)
{
  // Setup and fragmentor serial port
  pinMode(15,INPUT);
  FRAGPORT.begin(FRAGBAUD);
  attachProcessSerial(PollFrag);
  // Set the wire speed to 400000
  Wire.setClock(WireDefaultSpeed);
  AddMainMenuEntry(&MEDMSDMSmodule);
  // Add the commands to the command processor
  AddToCommandList(&DMSDMSCmdList);
  // Configure Threads
  DMSDMSthread.setName("DMSDMS");
  DMSDMSthread.onRun(DMSDMS_loop);
  DMSDMSthread.setInterval(100);
  // Add threads to the controller
  control.add(&DMSDMSthread);
  if (ActiveDialog == NULL) DialogBoxDisplay(&DMSDMSdialog);
  cvbiasState.update = true;
  waveformsState.update = true;
  if(DMSDMSScanClock == NULL) DMSDMSScanClock = new MIPStimer(FAIMSFB_ScanClock);
}

void CVBIAS_init(int brd, int add)
{
  CVBIASbrd = brd;
  cvbiasdata.TWIadd = add;
  RestoreDMSDMSsettings(brd, &cvbiasdata, true);
  cvbiasdata.TWIadd = add;
  cvbiasCH = cvbiasdata.channel[0];
  if(WAVEFORMSbrd != -1) DMSDMS_init();
}

void WAVEFORMS_init(int brd, int add)
{
  WAVEFORMSbrd = brd;
  waveformsdata.TWIadd = add;
  RestoreDMSDMSsettings(brd, &waveformsdata, true);
  waveformsdata.TWIadd = add;
  waveformsCH = waveformsdata.channel[0];
  if(CVBIASbrd != -1) DMSDMS_init();
}

bool UpdateParameter(uint8_t TWIadd,int brd, int ch, int TWIcmd, float *value, float *svalue, bool update)
{
  if((*value != *svalue) || update)
  {
    int bd = TWIstart(TWIadd, brd, TWIcmd);
    // Send the channel byte
    if(ch != -1) Wire.write(ch);
    // Set the float value
    uint8_t *b = (uint8_t *)value;
    Wire.write(b[0]);
    Wire.write(b[1]);
    Wire.write(b[2]);
    Wire.write(b[3]);
    TWIend(TWIadd, bd);
    *svalue = *value;
    return true;
  }
  return false;
}

bool UpdateParameter(uint8_t TWIadd,int brd, int ch, int TWIcmd, int *value, int *svalue, bool update)
{
  if((*value != *svalue) || update)
  {
    int bd = TWIstart(TWIadd, brd, TWIcmd);
    // Send the channel byte
    if(ch != -1) Wire.write(ch);
    // Set the int value
    uint8_t *b = (uint8_t *)value;
    Wire.write(b[0]);
    Wire.write(b[1]);
    Wire.write(b[2]);
    Wire.write(b[3]);
    TWIend(TWIadd, bd);
    *svalue = *value;
    return true;
  }
  return false;
}

bool UpdateParameter(uint8_t TWIadd,int brd, int ch, int TWIcmd, bool *value, bool *svalue, bool update)
{
  if((*value != *svalue) || update)
  {
    int bd = TWIstart(TWIadd, brd, TWIcmd);
    // Send the channel byte
    if(ch != -1) Wire.write(ch);
    // Set the bool value
    uint8_t *b = (uint8_t *)value;
    Wire.write(b[0]);
    TWIend(TWIadd, bd);
    *svalue = *value;
    return true;
  }
  return false;
}

bool UpdateParameter(uint8_t TWIadd,int brd, int ch, int TWIcmd, int8_t *value, int8_t *svalue, bool update)
{
  if((*value != *svalue) || update)
  {
    int bd = TWIstart(TWIadd, brd, TWIcmd);
    // Send the channel byte
    if(ch != -1) Wire.write(ch);
    // Set the bool value
    uint8_t *b = (uint8_t *)value;
    Wire.write(b[0]);
    TWIend(TWIadd, bd);
    *svalue = *value;
    return true;
  }
  return false;
}

bool validateWaveformsRB(void)
{
  if((fpclassify(waveformsrbTemp.Vrf) != FP_NORMAL) && (fpclassify(waveformsrbTemp.Vrf) != FP_ZERO)) return false;
  if((fpclassify(waveformsrbTemp.I) != FP_NORMAL) && (fpclassify(waveformsrbTemp.I) != FP_ZERO)) return false;
  if((fpclassify(waveformsrbTemp.V) != FP_NORMAL) && (fpclassify(waveformsrbTemp.V) != FP_ZERO)) return false;
  if((waveformsrbTemp.Vrf > 3000) || (waveformsrbTemp.Vrf < -3000)) return false;
  if((waveformsrbTemp.I > 9000) || (waveformsrbTemp.I < -9000)) return false;
  if((waveformsrbTemp.V > 50) || (waveformsrbTemp.V < -1)) return false;
  return true;
}

bool validateCVbiasRB(void)
{
  if((fpclassify(cvbiasrbTemp.Bias) != FP_NORMAL) && (fpclassify(cvbiasrbTemp.Bias) != FP_ZERO)) return false;
  if((fpclassify(cvbiasrbTemp.CV) != FP_NORMAL) && (fpclassify(cvbiasrbTemp.CV) != FP_ZERO)) return false;
  if((cvbiasrbTemp.Bias > 50) || (cvbiasrbTemp.Bias < -50)) return false;
  if((cvbiasrbTemp.CV > 50) || (cvbiasrbTemp.CV < -50)) return false;
  return true;
}

void DMSDMS_loop(void)
{
  float fval;

  if (ActiveDialog == &DMSDMSdialog)
  {
    if(ActiveDialog->Changed)
    {
      cvbiasdata.channel[DMSDMSchannel-1] = cvbiasCH;
      waveformsdata.channel[DMSDMSchannel-1] = waveformsCH;
      ActiveDialog->Changed = false;
    }
  }
  waveformsdata.channel[0].Enable = cvbiasdata.channel[0].Enable;
  waveformsdata.channel[1].Enable = cvbiasdata.channel[1].Enable;
  waveformsdata.Steps = cvbiasdata.Steps;
  waveformsdata.StepDuration = cvbiasdata.StepDuration;
  // Process each DMSDMS channel. Look for changes in parameters and update as needed
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_STEPS,&cvbiasdata.Steps,&cvbiasState.Steps,cvbiasState.update);
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_DURATION,&cvbiasdata.StepDuration,&cvbiasState.StepDuration,cvbiasState.update);
  UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,-1,TWI_SET_STEPS,&waveformsdata.Steps,&waveformsState.Steps,waveformsState.update);
  UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,-1,TWI_SET_DURATION,&waveformsdata.StepDuration,&waveformsState.StepDuration,waveformsState.update);
  for(int ch = 0; ch < 2; ch++)
  {
    // Waveforms processor updates
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_ENABLE,&waveformsdata.channel[ch].Enable,&waveformsState.Enable[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_MODE,&waveformsdata.channel[ch].Mode,&waveformsState.Mode[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_FREQ,&waveformsdata.channel[ch].Freq,&waveformsState.Freq[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_DUTY,(int8_t *)&waveformsdata.channel[ch].Duty,&waveformsState.Duty[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_DRIVE,&waveformsdata.channel[ch].Drive,&waveformsState.Drive[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_VRF,&waveformsdata.channel[ch].Vrf,&waveformsState.Vrf[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_MAXPWR,&waveformsdata.channel[ch].MaxPower,&waveformsState.MaxPower[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_MAXDRV,&waveformsdata.channel[ch].MaxDrive,&waveformsState.MaxDrive[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_VRF_START,&waveformsdata.channel[ch].VRFstart,&waveformsState.VRFstart[ch],waveformsState.update);
    UpdateParameter(waveformsdata.TWIadd | 0x20,WAVEFORMSbrd,ch,TWI_SET_VRF_END,&waveformsdata.channel[ch].VRFend,&waveformsState.VRFend[ch],waveformsState.update);
    // CVBIAS processor updates
    UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,ch,TWI_SET_ENABLE,&cvbiasdata.channel[ch].Enable,&cvbiasState.Enable[ch],cvbiasState.update);
    UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,ch,TWI_SET_CV,&cvbiasdata.channel[ch].CV,&cvbiasState.CV[ch],cvbiasState.update);
    UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,ch,TWI_SET_BIAS,&cvbiasdata.channel[ch].Bias,&cvbiasState.Bias[ch],cvbiasState.update);
    UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,ch,TWI_SET_CV_START,&cvbiasdata.channel[ch].CVstart,&cvbiasState.CVstart[ch],cvbiasState.update);
    UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,ch,TWI_SET_CV_END,&cvbiasdata.channel[ch].CVend,&cvbiasState.CVend[ch],cvbiasState.update);
    // Read the readback structures
    TWIreadBlock(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd,ch,TWI_READ_READBACKS, (void *)&waveformsrbTemp, sizeof(WAVEFORMSreadBacks));
    if(validateWaveformsRB()) waveformsRB[ch] = waveformsrbTemp;
    delay(1);
    TWIreadBlock(cvbiasdata.TWIadd | 0x20, CVBIASbrd,ch,TWI_READ_READBACKS, (void *)&cvbiasrbTemp, sizeof(CVBIASreadBacks));
    if(validateCVbiasRB()) cvbiasRB[ch] = cvbiasrbTemp;
    // Read selected values that the controller can change depending on mode
    if(TWIreadFloat(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd,ch,TWI_READ_DRIVE, &fval)) waveformsState.Drive[ch] = waveformsdata.channel[ch].Drive = fval;
    if(TWIreadFloat(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd,ch,TWI_READ_VRF, &fval)) waveformsState.Vrf[ch] = waveformsdata.channel[ch].Vrf = fval;
    if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,ch,TWI_READ_CV, &fval)) cvbiasState.CV[ch] = cvbiasdata.channel[ch].CV = fval;
  }
  // Electrometer updates
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_ELEC_POSOFF,&cvbiasdata.electrometer.PosOffset,&cvbiasState.PosOffset,cvbiasState.update);
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_ELEC_NEGOFF,&cvbiasdata.electrometer.NegOffset,&cvbiasState.NegOffset,cvbiasState.update);
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_ELEC_POSZ,&cvbiasdata.electrometer.PosZero,&cvbiasState.PosZero,cvbiasState.update);
  UpdateParameter(cvbiasdata.TWIadd | 0x20,CVBIASbrd,-1,TWI_SET_ELEC_NEGZ,&cvbiasdata.electrometer.NegZero,&cvbiasState.NegZero,cvbiasState.update);
  // Read electometer currents and zero values
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_POS, &fval)) PosCurrent = fval;
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_NEG, &fval)) NegCurrent = fval;
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_POSZ, &fval)) cvbiasdata.electrometer.PosZero = fval;
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_NEGZ, &fval)) cvbiasdata.electrometer.NegZero = fval;

  cvbiasState.update = false;
  waveformsState.update = false;
  cvbiasCH    = cvbiasdata.channel[DMSDMSchannel-1];
  waveformsCH = waveformsdata.channel[DMSDMSchannel-1];
  cvbiasrb    = cvbiasRB[DMSDMSchannel-1];
  waveformsrb = waveformsRB[DMSDMSchannel-1];
  power = waveformsrb.V * waveformsrb.I / 1000;
  if (ActiveDialog == &DMSDMSdialog) RefreshAllDialogEntries(&DMSDMSdialog);
}

//
// Scanning functions and variables;
//
typedef struct
{
  bool      Updated;
  uint32_t  StartTime;
  uint32_t  TimeStamp;
  int       Point;
  int       steps;
} MIPSscanData;

volatile MIPSscanData  *msd = NULL;

// Scanning ISR
void DMSDMSscanISR(void)
{
  float fval;

  //interrupts();
  __enable_irq();
  msd->TimeStamp = millis();
  msd->Point++;
  // Read the electrometer values
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_POS, &fval)) PosCurrent = fval;
  if(TWIreadFloat(cvbiasdata.TWIadd | 0x20, CVBIASbrd,TWI_READ_ELEC_NEG, &fval)) NegCurrent = fval;
  // Advance to next scan point
  digitalWrite(MIPSscanAdv,HIGH);
  digitalWrite(MIPSscanAdv,LOW);
  // Set flag that data is ready
  msd->Updated = true;
}

// This function starts a scan on the DMSDMS system. 
//
// The system is configured to use the external scan advance signal. The
// system is also configured to return no report.
void StartDMSDMSScan(void)
{
  int num;

  DMSDMSScanClock->stop();
  if(msd == NULL) msd = new MIPSscanData;
  msd->Updated = false;
  msd->Point = 0;
  msd->steps = cvbiasdata.Steps;
  // Set the scan advance pin to output and set low
  pinMode(MIPSscanAdv,OUTPUT);
  digitalWrite(MIPSscanAdv,LOW);
  // Setup DMSDMS for external scan trigger
  TWIsetBool(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_EXTSTEP, true);
  TWIsetBool(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_SCNRPT, false);
  TWIcmd(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_STEPSTR);
  TWIsetBool(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, TWI_SET_EXTSTEP, true);
  TWIsetBool(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, TWI_SET_SCNRPT, false);
  TWIcmd(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, TWI_SET_STEPSTR);
  msd->StartTime = millis();
  // Setup the timer ISR
  DMSDMSScanClock->attachInterrupt(DMSDMSscanISR);
  DMSDMSScanClock->setPriority(8);
  NVIC_SetPriority (SysTick_IRQn, 0);
  NVIC_SetPriority(WIRE1_ISR_ID, 8);
  NVIC_SetPriority(WIRE_ISR_ID, 8);
  NVIC_SetPriority((IRQn_Type) ID_UOTGHS, 8UL);
  DMSDMSScanClock->setPeriod(cvbiasdata.StepDuration * 1000,8);
   // Start the timer
  DMSDMSScanClock->start(-1, 8, false);
}

void ReportDMSDMSScan(void)
{
  // Wait for scan data and report
  while(1)
  {
    WDT_Restart(WDT);
    if(msd->Updated)
    {
       msd->Updated = false;
       serial->print(msd->Point); serial->print(",");
       serial->print(msd->TimeStamp - msd->StartTime);
       serial->print(","); serial->print(PosCurrent);
       serial->print(","); serial->print(NegCurrent);
       serial->println();
       if(msd->Point >= msd->steps) 
       {
          DMSDMSScanClock->stop();
          return;
       }
    }
  }
}

// Stops a scan in progress
void AbortDMSDMSScan(void)
{
  int brd;
    
  DMSDMSScanClock->stop();
  TWIcmd(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_STEPSTP);
  TWIcmd(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, TWI_SET_STEPSTP);
}

//
// Host commands
//
int checkChannel(int ch)
{
  if((ch<1)||(ch>2)||(CVBIASbrd==-1)||(WAVEFORMSbrd==-1))
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return -1;
  }
  return ch-1;
}

int checkChannel(char *ch)
{
  int    i;

  sscanf(ch,"%d",&i);
  if((i<1)||(i>2)||(CVBIASbrd==-1)||(WAVEFORMSbrd==-1))
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return -1;
  }
  return i-1;
}

/* Defined in FPGA module, need to clean this up!
bool setVariable(int *v,int value, int LL, int UL)
{
  if((value<LL)||(value>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = value;
  return true;
}
*/

bool setVariable(float *v,char *value, float LL, float UL)
{
  float   d;
  String  token;

  token = value;
  d = token.toFloat();
  if((d<LL)||(d>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = d;
  return true;
}
/*
bool setVariable(bool *v,char *value)
{
  float   d;
  String  T=value;

  if(T == "TRUE") *v = true;
  else if(T == "FALSE") *v = false;
  else 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  return true;
}
*/

void SetDMSDMSEnable(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&cvbiasdata.channel[i].Enable,value); waveformsdata.channel[i].Enable = cvbiasdata.channel[i].Enable;}
void ReturnDMSDMSEnable(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) cvbiasdata.channel[checkChannel(chan)].Enable ?  serial->println("TRUE"): serial->println("FALSE");}
void SetDMSDMSMode(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].Mode,value);}
void ReturnDMSDMSMode(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) waveformsdata.channel[checkChannel(chan)].Mode ?  serial->println("TRUE"): serial->println("FALSE");}
void SetDMSDMSFreq(int chan, int value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].Freq,value,100000,2000000);}
void ReturnDMSDMSFreq(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].Freq);}
void SetDMSDMSDuty(int chan, int value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].Duty,value,1,99);}
void ReturnDMSDMSDuty(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].Duty);}
void SetDMSDMSDrive(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].Drive,value,0,100);}
void ReturnDMSDMSDrive(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].Drive);}
void ReturnDMSDMSDriveV(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsRB[checkChannel(chan)].V);}
void ReturnDMSDMSDriveI(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsRB[checkChannel(chan)].I);}
void SetDMSDMSVrf(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].Vrf,value,0,2000);}
void ReturnDMSDMSVrf(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].Vrf);}
void ReturnDMSDMSVrfV(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsRB[checkChannel(chan)].Vrf);}
void ReturnDMSDMSPWR(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsRB[checkChannel(chan)].V * waveformsRB[checkChannel(chan)].I / 1000);}

void SetDMSDMSMaxDrive(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].MaxDrive,value,0,100);}
void ReturnDMSDMSMaxDrive(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].MaxDrive);}
void SetDMSDMSMaxPower(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].MaxPower,value,0,50);}
void ReturnDMSDMSMaxPower(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].MaxPower);}

void SetDMSDMSCV(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&cvbiasdata.channel[i].CV,value,-50,50);}
void ReturnDMSDMSCV(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasdata.channel[checkChannel(chan)].CV);}
void ReturnDMSDMSCVrb(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasRB[checkChannel(chan)].CV);}
void SetDMSDMSBIAS(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&cvbiasdata.channel[i].Bias,value,-50,50);}
void ReturnDMSDMSBIAS(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasdata.channel[checkChannel(chan)].Bias);}
void ReturnDMSDMSBIASrb(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasRB[checkChannel(chan)].Bias);}

void SetDMSDMSCVstart(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&cvbiasdata.channel[i].CVstart,value,-50,50);}
void ReturnDMSDMSCVstart(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasdata.channel[checkChannel(chan)].CVstart);}
void SetDMSDMSCVend(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&cvbiasdata.channel[i].CVend,value,-50,50);}
void ReturnDMSDMSCVend(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(cvbiasdata.channel[checkChannel(chan)].CVend);}
void SetDMSDMSVRFstart(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].VRFstart,value,100,2000);}
void ReturnDMSDMSVRFstart(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].VRFstart);}
void SetDMSDMSVRFend(char *chan, char *value) {int i; if((i = checkChannel(chan)) == -1) return; setVariable(&waveformsdata.channel[i].VRFend,value,100,2000);}
void ReturnDMSDMSVRFend(int chan) {if(checkChannel(chan) == -1) return; SendACKonly; if(!SerialMute) serial->println(waveformsdata.channel[checkChannel(chan)].VRFend);}
void SetDMSDMSStepDuration(int value) {setVariable(&cvbiasdata.StepDuration,value,5,100000);}
void SetDMSDMSSteps(int value) {setVariable(&cvbiasdata.Steps,value,3,100000);}
void InitDMSDMSScan(void) {StartDMSDMSScan(); SendACK; ReportDMSDMSScan();}
void StopDMSDMSScan(void) {AbortDMSDMSScan(); SendACK;}

void SetEMRTM4enable(char *value) 
{
  setVariable(&cvbiasdata.electrometer.M4ena,value);
  TWIsetBool(cvbiasdata.TWIadd | 0x20, CVBIASbrd, TWI_SET_ELEC_M4,cvbiasdata.electrometer.M4ena);
}
void SetEMRTposOff(char *value) {setVariable(&cvbiasdata.electrometer.PosOffset,value,0,5);}
void SetEMRTnegOff(char *value) {setVariable(&cvbiasdata.electrometer.NegOffset,value,0,5);}
void SetEMRTposZero(char *value) {setVariable(&cvbiasdata.electrometer.PosZero,value,0,5);}
void SetEMRTnegZero(char *value) {setVariable(&cvbiasdata.electrometer.NegZero,value,0,5);}
void SetEMRTzero(void) {SendACK; ZeroElectrometer();}

void SetDMSDMSVrfNow(char *chan, char *value)
{
  int i; if((i = checkChannel(chan)) == -1) return;
  setVariable(&waveformsdata.channel[i].Vrf,value,100,2000);
  // Send command to set the Vrf level
  TWIsetFloat(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, i, TWI_SET_VRF_NOW, waveformsdata.channel[i].Vrf);
}

void SetDMSDMSVrfTable(char *chan, char *value)
{
  int i; if((i = checkChannel(chan)) == -1) return;
  setVariable(&waveformsdata.channel[i].Vrf,value,100,2000);
  // Send command to set the Vrf level
  TWIsetFloat(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, i, TWI_SET_VRF_TABLE, waveformsdata.channel[i].Vrf);
}

void GenerateVrfTable(int chan)
{
  int i; if((i = checkChannel(chan)) == -1) return;
  SendACK;
  TWIcmd(waveformsdata.TWIadd | 0x20, WAVEFORMSbrd, i, TWI_SET_CAL);
}

void PollFrag(void)
{
  while(FRAGPORT.available() > 0)
  {
    serial->print((char)FRAGPORT.read());
    serial->flush();
    delay(1);
    if(FRAGPORT.available() == 0) delay(10);
  }
}

void SendFrag(void)
{
  char ch;

  GetToken(true);  // removes the comma
  while(true)
  {
    ch = RB_Get(&RB);
    if(ch == 0xFF) break;
    if(ch == ';') ch = '\n';
    FRAGPORT.write(ch);
    if(ch == '\n') break;
    PollFrag();
  }
  FRAGPORT.flush();
  delay(25);
  PollFrag();
}

// The following code supports the high voltage module used for an ion source. This module
// Plugs into the MIPS controller and uses the second TWI channel on the DUE to communicate
// with a AD5593 that controls all the parameters in the HV module.

HVmodule       hvm = 
{
  sizeof(HVmodule),"HVM", 1,
  HVM_TWI,false,false,
  0, 4000,
  {CHhvpCtrl, 8.4388, 1881.9},
  {CHhvpMon,  16.506, 3761.9},
  {CHhvnCtrl, 8.9928, 440.65},
  {CHhvnMon,  17.618, 884.27}
};

void hvmInitalize(void)
{
   MIPSconfigData.Ser1ena = false;
   pinMode(18, INPUT);
   pinMode(19, INPUT);

   pinMode(17, INPUT);
   pinMode(70, INPUT);
   pinMode(71, INPUT);
  // Init the wire1 interface and the AD5592
   Wire1.begin();
   Wire1.setClock(Wire1DefaultSpeed);

   // General purpose configuration, set range for 2.5 volts
   AD5593writeWire1(hvm.TWIadd, 3, 0x0100);
   // Set ext reference
   AD5593writeWire1(hvm.TWIadd, 11, 0x0200);
   // Set LDAC mode
   AD5593writeWire1(hvm.TWIadd, 7, 0x0000);
   // Set digital outputs channels
   AD5593writeWire1(hvm.TWIadd, 8, 0x0009);
   // Set DAC outputs channels
   AD5593writeWire1(hvm.TWIadd, 5, 0x0012);
   // Set ADC input channels
   AD5593writeWire1(hvm.TWIadd, 4, 0x0024);
   // Turn off all pulldowns
   AD5593writeWire1(hvm.TWIadd, 6, 0x0000);

   // Set all DACs to zero
   AD5593writeDACWire1(hvm.TWIadd, 1, 0);
   AD5593writeDACWire1(hvm.TWIadd, 4, 0);
   // Outputs off
   AD5593writeWire1(hvm.TWIadd, 9, 0);
   hvm.Inited = true;
}

void hvmInit(void) 
{
  if(!hvm.Inited) hvmInitalize(); 
  SendACK;
}

void hvmSetEnable(char *val) 
{
  bool res;

  if(!hvm.Inited) hvmInitalize(); 
  if(!setVariable(&res,val)) return;
  //if(hvm.Enable == res) return;
  hvm.Enable = res;
  if(res)
  {
    // Here to enable, set voltage outputs and enable switches
    if(hvm.voltage >= 0)
    {
      AD5593writeDACWire1(hvm.TWIadd, hvm.VnegCtrl.Chan, Value2Counts(0,&hvm.VnegCtrl));
      delay(1000);
      AD5593writeDACWire1(hvm.TWIadd, hvm.VposCtrl.Chan, Value2Counts(hvm.voltage,&hvm.VposCtrl));
      AD5593writeWire1(hvm.TWIadd, 9, 1 << CHhvpENA);
    }
    else
    {
      AD5593writeDACWire1(hvm.TWIadd, hvm.VposCtrl.Chan, Value2Counts(0,&hvm.VposCtrl));
      delay(1000);
      AD5593writeDACWire1(hvm.TWIadd, hvm.VnegCtrl.Chan, Value2Counts(abs(hvm.voltage),&hvm.VnegCtrl));
      AD5593writeWire1(hvm.TWIadd, 9, 1 << CHhvnENA);
    }
  }
  else
  {
    // Here to disable, set voltage outputs to 0 and disable both pos and neg supplies
    AD5593writeDACWire1(hvm.TWIadd, hvm.VposCtrl.Chan, Value2Counts(0,&hvm.VposCtrl));
    AD5593writeDACWire1(hvm.TWIadd, hvm.VnegCtrl.Chan, Value2Counts(0,&hvm.VnegCtrl));
    AD5593writeWire1(hvm.TWIadd, 9, 0);
  }
}
void hvmGetEnable(void) 
{
  SendACKonly;
  if(SerialMute) return;
  if(hvm.Enable) serial->println("TRUE");
  else serial->println("FALSE");
}

void hvmSetVoltage(char *val) 
{
  float v;

  if(!hvm.Inited) hvmInitalize(); 
  if(!setVariable(&v,val,-3500,3500)) return;
  if(!hvm.Enable)
  {
    hvm.voltage = v;
    return;
  }
  if(((v>=0) && (hvm.voltage < 0)) || ((v<0) && (hvm.voltage >= 0)))
  {
    // Here with polarity change
    AD5593writeDACWire1(hvm.TWIadd, hvm.VposCtrl.Chan, Value2Counts(0,&hvm.VposCtrl));
    AD5593writeDACWire1(hvm.TWIadd, hvm.VnegCtrl.Chan, Value2Counts(0,&hvm.VnegCtrl));
    AD5593writeWire1(hvm.TWIadd, 9, 0);
    delay(2000);
  }
  hvm.voltage = v;
  if(v>=0)
  {
    AD5593writeWire1(hvm.TWIadd, 9, 1 << CHhvpENA);
    AD5593writeDACWire1(hvm.TWIadd, hvm.VposCtrl.Chan, Value2Counts(hvm.voltage,&hvm.VposCtrl));
  }
  else
  {
    AD5593writeWire1(hvm.TWIadd, 9, 1 << CHhvnENA);
    AD5593writeDACWire1(hvm.TWIadd, hvm.VnegCtrl.Chan, Value2Counts(abs(hvm.voltage),&hvm.VnegCtrl));
  }
}
void hvmGetVoltage(void) {SendACKonly; if(!SerialMute) serial->println(hvm.voltage);}
void hvmGetReadback(void) 
{
  float v;

  if(!hvm.Inited) hvmInitalize(); 
  SendACKonly;
  if(hvm.voltage >= 0) v = Counts2Value(AD5593readADCWire1(hvm.TWIadd, CHhvpMon, 10), &hvm.VposMon);
  else v = -Counts2Value(AD5593readADCWire1(hvm.TWIadd, CHhvnMon, 10), &hvm.VnegMon);
  if(!SerialMute) serial->println(v);
}

#endif