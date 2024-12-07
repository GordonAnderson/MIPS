//
// File: Compressor
//
// This file contains compressor variables and function that are common to the ARB and TWAVE 
// compressor capability, this include the frequency and ampitude sweeping functions and command
// processing routines.
//
// Gordon Anderson
//

MIPStimer CompressorTimer(TMR_TwaveCmp);

SweepTable *sweeptable= NULL;

char TwaveCompressorTable[131] = "C";

bool CompressorDisable = false;

char *CmodeList = "Normal,Compress";
char Cmode[12]  = "Normal";

char *CswitchList    = "Open,Close";
char CswitchState[6] = "Open";

DIhandler *CtrigInput;

// Time delay parameters. All the time values in milli secs are converted into timer counts and saved in these 
// variables.
uint32_t  C_Td;               // Trigger delay
uint32_t  C_Tc;               // Compressed time
uint32_t  C_Tn;               // Normal time
uint32_t  C_Tnc;              // Non compressed cycle time
uint32_t  C_Delay;            // Delay time
uint32_t  C_NextEvent;        // Next event counter value
uint32_t  C_SwitchTime;       // Switch open time
uint32_t  C_GateOpenTime;     // Switch event time from start of table

volatile int       C_NormAmpMode = 0;             // If set to 1 then the normal mode will use the TW channel 1 amplitude

CompressorState CState;
CompressorSwitchState CSState;
char CompressorSelectedSwitch;
int CompressorSelectedSwitchLevel;
int CurrentPass;

int CStackLevel;
CompressorStack cStack[5];

void CompressorStackInit(void)
{
  CStackLevel = 0;
}

bool CompressorLoopStart(int Index)
{
  // If stack is full exit and return false
  if(CStackLevel >= 5) return false;
  // Init the stack entry
  CStackLevel++;
  cStack[CStackLevel - 1].StartOfLoop = Index;
  cStack[CStackLevel - 1].Inited = false;
}

int CompressorProcessLoop(int Count)
{
  if(CStackLevel <= 0) return -1;
  if(cStack[CStackLevel - 1].Inited == false) cStack[CStackLevel - 1].Count = Count;
  cStack[CStackLevel - 1].Inited = true;
  cStack[CStackLevel - 1].Count--;
  if(cStack[CStackLevel - 1].Count == 0) 
  {
    CStackLevel--;
    return -1;
  }
  return cStack[CStackLevel - 1].StartOfLoop;
}

//
// ************************************************************************************************************
// Twave voltage and frequency sweep functions
// ************************************************************************************************************
//
// This voltage and frequency sweep function works with both the Twave module and the ARB module in Twave 
// mode. These function support sweeping the frequency and/or voltage over a user defined range for
// module channels 1 and/or 2.
//  -Start freq
//  -Stop freq
//  -Start voltage
//  -Stop voltage 
//  -sweep time in seconds
//  -Start and Stop
//  -support two channels, with syncronized start
//  - Commands:
//      STWSSTRT,chan,freq    // defines start freq
//      GTWSSTRT,chan
//      STWSSTP,chan,freq     // defines stop freq
//      GTWSSTP,chan
//      STWSSTRTV,chan,volts  // defines start voltage
//      GTWSSTRTV,chan
//      STWSSTPV,chan,volts   // defines stop voltage
//      GTWSSTPV,chan
//      STWSTM,chan,time      // defines sweep time in seconds
//      GTWSTM,chan
//      STWSGO,chan           // Starts the sweep on channel 1 or 2, or 3 for both
//      STWSHLT,chan          // Stops the sweep on channel 1 or 2, or 3 for both
//      GTWSTA,chan           // Returns selected channel scan status, running or stopped
//   Implementation details
//    - Parameters are not saved
//    - Updated 10 times per second

FreqSweep fSweep[2] = {{100000,100000,200000,20,30,30,10,0,0,SS_IDLE},
                       {100000,100000,200000,20,30,30,10,0,0,SS_IDLE}};

// The ISR sets the sweep state and exits
void ARBTWAVEsweepISR(void)
{
  if(fSweep[0].State == SS_IDLE) fSweep[0].State = SS_START;
  if(fSweep[1].State == SS_IDLE) fSweep[1].State = SS_START;
  ARBthread.setNextRunTime(millis());
}

bool SweepLookup(int TP, float *V1, float *V2, float *F)
{
  int i;
  
  if(sweeptable == NULL) return false;
  if(sweeptable->num < 2) return false;
  for(i=0;i<sweeptable->num-1;i++)
  {
    if(TP < sweeptable->timePnt[i]) break;
    if((TP >= sweeptable->timePnt[i]) && (TP <= sweeptable->timePnt[i+1])) break;
  }
  if(i == sweeptable->num-1) i--;
  // The points at i and i+1 will be used to calculate the output voltages and frequency
  // y = y1 + (x-x1) * (y2-y1)/(x2-x1)
  if(V1 != NULL) *V1 = (float)sweeptable->voltage1[i] + (float)(TP - sweeptable->timePnt[i]) * (float)(sweeptable->voltage1[i+1] - sweeptable->voltage1[i]) / (float)(sweeptable->timePnt[i+1] - sweeptable->timePnt[i]);
  if(V2 != NULL) *V2 = (float)sweeptable->voltage2[i] + (float)(TP - sweeptable->timePnt[i]) * (float)(sweeptable->voltage2[i+1] - sweeptable->voltage2[i]) / (float)(sweeptable->timePnt[i+1] - sweeptable->timePnt[i]);
  if(F != NULL)  *F = (float)sweeptable->frequency[i] + (float)(TP - sweeptable->timePnt[i]) * (float)(sweeptable->frequency[i+1] - sweeptable->frequency[i]) / (float)(sweeptable->timePnt[i+1] - sweeptable->timePnt[i]);
  return true;
}

// This function is called in the Twave and ARB polling loops and processes frequency/voltage sweep requests. The FreqSweep
// data structure contains all of the needed parameters. This code looks at the modules present to determine if its Twave or ARB.
void ProcessSweep(void)
{
  int           chan;
  uint32_t      RightNow;
  float         rnf,rnv;

  RightNow = millis();
  for(chan = 0; chan < 2; chan++)
  {
    if(fSweep[chan].State == SS_IDLE) continue;
    if(fSweep[chan].State == SS_START)
    {
      if(NumberOfTwaveModules > 0)
      {
        fSweep[chan].OrginalFreq = TDarray[chan].Velocity;
        fSweep[chan].OrginalVoltage = TDarray[chan].TWCD[0].VoltageSetpoint;
        TDarray[chan].Velocity = fSweep[chan].StartFreq;
        TDarray[chan].TWCD[0].VoltageSetpoint = fSweep[chan].StartVoltage;
      }
      if(NumberOfARBchannels > 0)
      {
        fSweep[chan].OrginalFreq = ARBarray[chan]->Frequency;
        fSweep[chan].OrginalVoltage = ARBarray[chan]->Voltage;
        ARBarray[chan]->Frequency = fSweep[chan].StartFreq;
        ARBarray[chan]->Voltage = fSweep[chan].StartVoltage;        
      }
      fSweep[chan].SweepStartTime = RightNow;
      fSweep[chan].CurrentSweepTime = RightNow;
      fSweep[chan].State = SS_SWEEPING;
    }
    if(fSweep[chan].State == SS_STOP)
    {
      if(NumberOfTwaveModules > 0)
      {
        TDarray[chan].Velocity = fSweep[chan].OrginalFreq;
        TDarray[chan].TWCD[0].VoltageSetpoint = fSweep[chan].OrginalVoltage;
      }
      if(NumberOfARBchannels > 0)
      {
        ARBarray[chan]->Frequency = fSweep[chan].OrginalFreq;
        ARBarray[chan]->Voltage = fSweep[chan].OrginalVoltage;        
      }
      fSweep[chan].State = SS_IDLE;
    }
    if(fSweep[chan].State == SS_SWEEPING)
    {
      // Calculate the right now frequency.
      // Right now freq = StartFreq + ((StopFreq - StartFreq) * SweepTime/(RightNow - SweepStartTime))
      rnv = rnf = ((float)RightNow - (float)fSweep[chan].SweepStartTime) / (fSweep[chan].SweepTime * 1000);
      rnf = fSweep[chan].StartFreq + (float)(fSweep[chan].StopFreq - fSweep[chan].StartFreq) * rnf;
      // Calculate the right now voltage
      rnv = fSweep[chan].StartVoltage + (float)(fSweep[chan].StopVoltage - fSweep[chan].StartVoltage) * rnv;      
      if(RightNow >= (fSweep[chan].SweepStartTime + fSweep[chan].SweepTime * 1000))
      {
        fSweep[chan].State = SS_STOP;
      }
      if(NumberOfTwaveModules > 0)
      {
        TDarray[chan].Velocity = rnf;
        TDarray[chan].TWCD[0].VoltageSetpoint = rnv;
      }
      if(NumberOfARBchannels > 0)
      {
        if(chan == 0) SweepLookup(RightNow - fSweep[chan].SweepStartTime, &rnv, NULL, &rnf);
        if(chan == 1) SweepLookup(RightNow - fSweep[chan].SweepStartTime, NULL, &rnv, &rnf);
        ARBarray[chan]->Frequency = rnf;
        ARBarray[chan]->Voltage = rnv;   
      }
      fSweep[chan].CurrentSweepTime = RightNow;
    }
    //ARBthread.run();
  }  
}

// Following are the serial command processing functions that support the frequency sweep function for Twave
// The following commands are supported:
//      STWSSTRT,chan,freq    // defines start freq
//      GTWSSTRT,chan
//      STWSSTP,chan,freq     // defines stop freq
//      GTWSSTP,chan
//      STWSTM,chan,time      // defines sweep time in seconds
//      GTWSTM,chan
//      STWSGO,chan           // Starts the sweep on channel 1 or 2, or 3 for both
//      STWSHLT,chan          // Stops the sweep on channel 1 or 2, or 3 for both
//      GTWSTA,chan           // Returns selected channel scan status, running or stopped

// This function validates the channel and frequency range. Returns true if valid
// or false with range error. If range error then NAK is sent.
bool TWSWvalidateCF(int chan, int freq)
{
  DialogBoxEntry *de;
  int MaxChan = 0;

  if(NumberOfTwaveModules > 0) de = GetDialogEntries(TwaveDialogEntries2, "Clock freq, Hz");
  if(NumberOfARBchannels > 0) de = GetDialogEntries(ARBentriesPage1, "Frequency");
  if(NumberOfTwaveModules > 0) MaxChan = NumberOfTwaveModules;
  if(NumberOfARBchannels > 0) MaxChan = NumberOfARBchannels/8;
  if((chan <= 0) || (chan > MaxChan) || (freq > de->Max) || (freq < de->Min))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return false;
  }
  return true;
}

bool TWSWvalidateCV(int chan, int volts)
{
  DialogBoxEntry *de;
  int MaxChan = 0;

  if(NumberOfTwaveModules > 0) de = GetDialogEntries(TwaveDialogEntries2, "Pulse voltage");
  if(NumberOfARBchannels > 0) de = GetDialogEntries(ARBentriesPage1, "Amplitude, Vp-p");
  if(NumberOfTwaveModules > 0) MaxChan = NumberOfTwaveModules;
  if(NumberOfARBchannels > 0) MaxChan = NumberOfARBchannels/8;
  if((chan <= 0) || (chan > MaxChan) || (volts > de->Max) || (volts < de->Min))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return false;
  }
  return true;
}

bool TWSWvalidateC(int chan)
{
  int MaxChan = 0;

  if(NumberOfTwaveModules > 0) MaxChan = NumberOfTwaveModules;
  if(NumberOfARBchannels > 0) MaxChan = NumberOfARBchannels/8;
  if((chan <= 0) || (chan > MaxChan))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return false;
  }
  return true;
}

void SetStartFreqTWSW(int chan, int freq)
{
  int b;
  
  if(!TWSWvalidateCF(chan,freq)) return;
  if(chan <= 2) fSweep[chan-1].StartFreq = freq;
  if((b = ARBmoduleToBoard(chan, false)) != -1) ARBarray[b]->StartFreq = freq;
  SendACK;
}

void GetStartFreqTWSW(int chan)
{
  int b;
  
  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if((b = ARBmoduleToBoard(chan, false)) != -1)
  {
    if(!SerialMute) serial->println(ARBarray[b]->StartFreq);
    return;
  }
  if(!SerialMute) serial->println(fSweep[chan-1].StartFreq);
}

void SetStopFreqTWSW(int chan, int freq)
{
  int b;
  
  if(!TWSWvalidateCF(chan,freq)) return;
  if(chan <= 2) fSweep[chan-1].StopFreq = freq;
  if((b = ARBmoduleToBoard(chan, false)) != -1) ARBarray[b]->StopFreq = freq;
  SendACK;
}

void GetStopFreqTWSW(int chan)
{
  int b;
  
  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if((b = ARBmoduleToBoard(chan, false)) != -1)
  {
    if(!SerialMute) serial->println(ARBarray[b]->StopFreq);
    return;
  }
  if(!SerialMute) serial->println(fSweep[chan-1].StopFreq);
}

void SetStartVoltageTWSW(char *schan, char *svolts)
{
  String sToken;
  int    chan;
  float  volts;
  int b;

  sToken = schan;
  chan = sToken.toInt();
  sToken = svolts;
  volts = sToken.toFloat();
  if(!TWSWvalidateCV(chan,volts)) return;
  if(chan <= 2) fSweep[chan-1].StartVoltage = volts;
  if((b = ARBmoduleToBoard(chan, false)) != -1) ARBarray[b]->StartVoltage = volts;
  SendACK;
}

void GetStartVoltageTWSW(int chan)
{
  int b;

  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if((b = ARBmoduleToBoard(chan, false)) != -1)
  {
    if(!SerialMute) serial->println(ARBarray[b]->StartVoltage);
    return;
  }
  if(!SerialMute) serial->println(fSweep[chan-1].StartVoltage);
}

void SetStopVoltageTWSW(char *schan, char *svolts)
{
  String sToken;
  int    chan;
  float  volts;
  int b;

  sToken = schan;
  chan = sToken.toInt();
  sToken = svolts;
  volts = sToken.toFloat();
  if(!TWSWvalidateCV(chan,volts)) return;
  if(chan <= 2) fSweep[chan-1].StopVoltage = volts;
  if((b = ARBmoduleToBoard(chan, false)) != -1) ARBarray[b]->StopVoltage = volts;
  SendACK;
}

void GetStopVoltageTWSW(int chan)
{
  int b;

  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if((b = ARBmoduleToBoard(chan, false)) != -1)
  {
    if(!SerialMute) serial->println(ARBarray[b]->StopVoltage);
    return;
  }
  if(!SerialMute) serial->println(fSweep[chan-1].StopVoltage);
}

void SetSweepTimeTWSW(char *schan, char *stime)
{
  int    chan;
  float  SweepTime;
  String Token;
  int b;

  Token = schan;
  chan = Token.toInt();
  Token = stime;
  SweepTime = Token.toFloat();
  if(!TWSWvalidateC(chan)) return;
  if(chan <= 2) fSweep[chan-1].SweepTime = SweepTime;
  if((b = ARBmoduleToBoard(chan, false)) != -1) ARBarray[b]->SweepTime = SweepTime;
  SendACK;
}

void GetSweepTimeTWSW(int chan)
{
  int b;

  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if((b = ARBmoduleToBoard(chan, false)) != -1)
  {
    if(!SerialMute) serial->println(ARBarray[b]->SweepTime);
    return;
  }
  if(!SerialMute) serial->println(fSweep[chan-1].SweepTime); 
}

void StartSweepTWSW(int chan)
{
  if((chan < 1) || (chan > 3))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if((chan == 1) || (chan == 3)) if(fSweep[0].State == SS_IDLE) fSweep[0].State = SS_START;
  if((chan == 2) || (chan == 3)) if(fSweep[1].State == SS_IDLE) fSweep[1].State = SS_START;
  SendACK;
}

void StopSweepTWSW(int chan)
{
  if((chan < 1) || (chan > 3))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if((chan == 1) || (chan == 3)) fSweep[0].State = SS_STOP;
  if((chan == 2) || (chan == 3)) fSweep[1].State = SS_STOP;
  SendACK;  
}

void GetStatusTWSW(int chan)
{
  if(!TWSWvalidateC(chan)) return;
  SendACKonly;
  if(SerialMute) return;
  switch (fSweep[chan-1].State)
  {
    case SS_IDLE:
      serial->println("IDLE"); 
      break;
    case SS_START:
      serial->println("STARTING"); 
      break;
    case SS_STOP:
      serial->println("STOPPING"); 
      break;
    case SS_SWEEPING:
      serial->println("SWEEPING"); 
      break;
    default:
      break;
  }
}

bool CompressionTriggerQueued = false;
int CompressionTriggerTarget = 0;

void QueueCompressionTrigger(int num)
{
  CompressionTriggerQueued = true;
  CompressionTriggerTarget = num;
}

void ProcessCompressionTrigger(void)
{
  if(!CompressionTriggerQueued) return;
  CompressionTriggerQueued = false;
  if(CompressionTriggerTarget == char('A')) ARBcompressorTriggerISR();
  else if(CompressionTriggerTarget == char('T')) CompressorTriggerISR();
}

// Sweep Table command. The Sweep table supports a picewise
// linear voltage and frequecy sweep capability for ARB modules
// 1 and 2 only. This function will on operate on MIPS based
// sweeps.
// Sweep commands:
// SSWPCLR
// SSWPADD,time,voltage1,voltage2,frequency

void ClearSweepTable(void)
{
   SendACK;
   if(sweeptable == NULL) return;
   sweeptable->num = 0;  
}

void AddSweepPoint(void)
{
  char     *tkn;
  String   arg;
  int      T,V1,V2,F;

  while(true)
  {
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     T = arg.toInt();
     if((T<0) || (T > 10000)) break;
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     V1 = arg.toInt();
     if((V1 < 0) || (V1 > 100)) break; 
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     V2 = arg.toInt();
     if((V2 < 0) || (V2 > 100)) break; 
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     F = arg.toInt();
     if((F<1000) || (F > 40000)) break; 
     // Add point to table, create table if pointer is NULL
     if(sweeptable == NULL)
     {
       sweeptable = new(SweepTable);
       if(sweeptable == NULL) break;
       sweeptable->num = 0;       
     }
     if((sweeptable->num + 1) > MAXSWEEPTABLE) break;
     sweeptable->timePnt[sweeptable->num] = T;
     sweeptable->voltage1[sweeptable->num] = V1;
     sweeptable->voltage2[sweeptable->num] = V2;
     sweeptable->frequency[sweeptable->num] = F;
     sweeptable->num++;
     SendACK;
     return;
  }
  BADARG;
}

