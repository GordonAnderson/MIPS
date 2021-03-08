//
// This file contains Table code that is in development.
//
// In the time table the channel parameter defines what output action is taken at a time point.
// The following list defines all the valid channel parameters:
//   1 thru 32 are DCbias output channels (decimal 1 thru 32)
//   'A' thru 'P' are Digital output channels
// The following time sweep functions
//   'd' Time delta parameter incremented
//   'p' Time delta parameter set
// The following parameters control ARB parameters
//   'e' Set ARB module 1 AUXOUT voltage
//   'f' Set ARB module 2 AUXOUT voltage
//   'g' Set ARB module 3 AUXOUT voltage
//   'h' Set ARB module 4 AUXOUT voltage
//   'i' Set ARB module 1 output offset A
//   'j' Set ARB module 1 output offset B
//   'k' Set ARB module 2 output offset A
//   'l' Set ARB module 2 output offset B
// Misc operations
//   'r' Set the ARB compress line
//   's' Set the ARB sync line
//   't' Generate a trigger out pulse, value defines width
//   'b' Trigger a frequence burst generation
//   'c' Trigger ARB compression table
//
// Time point ramping or changing is supported by defining the time point with a negative value.
// The times points absolute values will be used and the current value of the TimeDelta parameter
// will be added to this absolute value.
//
// DCbias value Ramp functions
//
// Ramping is implemended using a timer interrupt to update the DCbias outputs channels that are
// generating ramps. The table contains flags bits in the channel number definition to indicate
// channels that are generating ramps. Two flags are used:
// RAMP to indicate this channel is ramping and the channel value contains the voltage step incremented
//      at each clock cycle.
// RAMP | INITIAL sets the initial or starting value for a ramp. This must be done before a ramp 
//                starts and it can be done at the same time point as the RAMP command.
// The ramp capability must first be enabled and the clock rate set before a table with ramp commands
// is executed. Below is a series of commands that will generate a simple ramp:
// 
// STBLRMPENA,TRUE,1000
// STBLDAT;0:[A:100,100:1:10:194:1:130:4,5000:1:15:130:-1.0,10000:130:0,13000:1:0,20000:];
// SMOD,ONCE
// TBLSTRT
// 
// April 2019
//  - Added voltage ramping capaility.
//  - Made a number of improvements to the code, performance issues.
//  - Fixed a bug on the table startup when there are timepoint events at time 0 such as ramps and
//    various internal trigggers like the trigger output or compressor.
//
/*
STBLRMPENA,TRUE,2000
STBLDAT;0:[A:100,100:1:10:194:1:130:4,5000:1:0:130:-1.0,10000:130:0,20000:];
SMOD,ONCE
TBLSTRT

STBLRMPENA,TRUE,2000
STBLDAT;0:[A:4,0:1:10:194:1:130:4,5000:1:0:130:-1.0,10000:130:0,20000:];
SMOD,ONCE
TBLSTRT

STBLRMPENA,TRUE,2000
STBLDAT;0:[A:4,0:1:10:194:1:130:4,5000:1:15:130:-1.0,10000:130:0,13000:1:0,20000:];
SMOD,ONCE
TBLSTRT


 */

#include "Arduino.h"
#include "variant.h"
#include "ADCdrv.h"
#include <stdio.h>
#include <Thread.h>
#include <ThreadController.h>
#include <MIPStimer.h>
#include <DIhandler.h>
#include "AtomicBlock.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include "SPI.h"
#include "Wire.h"
#include "Variants.h"    // This brings in most of the application specific include files

#if TABLE2code

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

//#define FAST_TABLE 1

#pragma GCC optimize "-O3"

extern ThreadController control;

MIPStimer MPT(TMR_Table);               // timer used for pulse sequence generation
#define   MPTtc   TC2->TC_CHANNEL[2]    // Pointer to the timers control block

static Pio *pio     = g_APinDescription[BRDSEL].pPort;
static uint32_t pin = g_APinDescription[BRDSEL].ulPin;
static Pio *pioA    = g_APinDescription[ADDR0].pPort;

#define NumTables 5
int     MaxTable = 0;  // Used while table is loading to allocate memory as needed for the table
unsigned int Counter;

//volatile unsigned char *VoltageTable[NumTables] = {NULL,NULL,NULL,NULL,NULL};
unsigned char *VoltageTable[NumTables] = {NULL,NULL,NULL,NULL,NULL};
volatile int ptr;     // Table pointer
volatile int CT = 0;  // Current table number
// These pointers are used by the real time processing of a table
volatile TableHeader         *Theader;
volatile TableEntryHeader    *TEheader;
volatile TableEntry          *Tentry;
volatile int                 TentryCount;    // Current entry loaded / pointed to

volatile char TablesLoaded[NumTables] = {0,0,0,0,0};       // Defines the number of tables loaded in the buffer
volatile int  TestNesting = 0;        // Used to test the nesting depth when command is parsed

volatile NestingStack NS;             // Used to manage nested stacks

volatile bool TableReady = false;     // This flag tells the system the tables are ready to use
volatile bool Aborted = false;        // Set when an abort command is received
volatile bool TableStopped = false;   //
volatile bool LOCrequest = false;     // Set when requested to enter local mode
volatile bool StopRequest;            // Used by the realtime processing to stop the timer at the end of its cycle
volatile bool StopCommanded;          // Flag set by the serial command TBLSTOP, stops the table and remains in table mode
volatile bool TableAdv = false;       // If this flag is true then the table number will be advanced to the next table
                                      // after every trigger
volatile bool SWtriggered = false;
volatile bool TableOnce = false;      // If true the table will play one time then the mode will return to local

volatile bool softLDAC = false;       // If true forces the use of software LDAC
bool TableResponse = true;            // This flag is true to enable table status response

bool TblTasks = false;                // If true enables processing tasks when there is avalible time during table execution
int  ExtFreq = 0;                     // Defines enternal clock frequency used by the table, needed for the TblTasks mode.

// The following variables are used to support the loop time incrementing or decermenting. This capability
// was added October 25, 2017, and redesiged December 6, 2020
int  TimeDelta = 0;                   // This time is added to all flaged time points 

uint8_t  Chan2Brd[32];                // Holds DCbias module SPI address and board address. Upper 4 bits hold SPI address
                                      // Used to speed the address selection process in real time code section

bool DCbiasUpdaated = false;
bool ValueChange = false;

bool TasksEnabled = false;            // Setting this flag to true will enable all tasks in table mode
bool TableTriggered = false;          // This flag is set when the table is triggered and reset when its complete
bool TimerRunning = false;

int InterTableDelay = 3;

DIhandler DIhTrig;

enum TriggerModes TriggerMode = SW;
enum ClockModes   ClockMode   = MCK128;
enum TableModes   TableMode   = LOC;

int  TableClockFreq = VARIANT_MCK/128;

bool IssueSoftwareTableStart = false;

// Used for the compress and state hardware lines to the ARBs
char Cstate;
char Sstate;

// Ramp generation variables
#define  RAMP      0x80
#define  INITIAL   0x40

#define  RAMP_DONE 0x80
#define  NUM_RAMPS 3

typedef struct
{
   uint8_t  chan;
   uint32_t initial;
   int      delta;
} RampEntry;

bool      RampEnabled = false;
bool      Ramping     = false;
MIPStimer *RampClock  = NULL;
volatile  RampEntry RE[NUM_RAMPS];
// End of ramp variables

//
// The following code supports queuing functions to be run at a later time. This is used by the
// ramping function to make sure the code is execured during the timer compare event, this is 
// when the LDAC latch pulse is generated.
// The queuing function have a Unique flag that will insure a function is present in the queue only
// one time if set to true.
//
volatile TableQueueEntry TQE[MaxTableQueue] = {{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL},{TblEmpty,NULL}};

void TABLEqueue(void (*function)(void), bool Unique)
{
  uint32_t *ptr;
  
  for(int i=0;i<MaxTableQueue;i++) 
  {
    if(Unique) if((void (*)(void))TQE[i].pointers.funcVoidVoid == (void (*)(void))function) break; 
    if(TQE[i].pointers.funcVoidVoid == NULL)
    {
       ptr = (uint32_t *)(&(TQE[i].pointers.funcVoidVoid));
       ptr[0] = (uint32_t)function;
       TQE[i].Type = TblVoidVoid;
       break;
    }
  }
}

void TABLEqueue(void (*function)(volatile TableEntry *),volatile TableEntry *te, bool Unique)
{
  uint32_t *ptr;
  
  for(int i=0;i<MaxTableQueue;i++) 
  {
    if(Unique) if((void (*)(volatile TableEntry *))TQE[i].pointers.funcTableEntryPtr == (void (*)(volatile TableEntry *))function) break;
    if(TQE[i].pointers.funcTableEntryPtr == NULL)
    {
       ptr = (uint32_t *)(&TQE[i].pointers.funcTableEntryPtr);
       ptr[0] = (uint32_t)(function);
       TQE[i].Type = TblVoidTableEntryPtr;
       TQE[i].TE   = te;
       break;
    }  
  }
}

void ProcessTableQueue(void)
{
  for(int i=0;i<MaxTableQueue;i++)
  {
    if(TQE[i].pointers.funcVoidVoid != NULL) 
    {
      if(TQE[i].Type == TblVoidVoid) TQE[i].pointers.funcVoidVoid();
      else if(TQE[i].Type == TblVoidTableEntryPtr) TQE[i].pointers.funcTableEntryPtr(TQE[i].TE);
      TQE[i].pointers.funcVoidVoid = NULL;
      TQE[i].Type = TblEmpty;
    }
  }
}
// End of queuing functions

//**************************************************************************************************
//
// This section of the file contains all the serial IO processing routines.
//
//**************************************************************************************************

// Low level routines to get token from input ring buffer.

// This function will wait for the next token or a timeout. This function blocks
// and will return NULL on a timeout.
char *NextToken(void)
{
    char *tk;
    unsigned int timeout;
    
    timeout = millis();
    while(millis() < (timeout + 3000))
    {
       if((tk=GetToken(true))!=NULL) return(tk); 
       ReadAllSerial();
    }
    SetErrorCode(ERR_TOKENTIMEOUT);
    return(NULL);
}

// This function pulls the next token and returns true if its a colon,
// returns false if timeout or other value
bool ExpectColon(void)
{
    char *TK;
    
    if((TK = NextToken()) == NULL) return false;
    if(TK[0] != ':')
    {
        SetErrorCode(ERR_EXPECTEDCOLON);
        return false;
    }
    return true;
}

// This function pulls the next token and returns true if its a comma,
// returns false if timeout or other value
bool ExpectComma(void)
{
    char *TK;
    
    if((TK = NextToken()) == NULL) return false;
    if(TK[0] != ',')
    {
        SetErrorCode(ERR_EXPECTEDCOMMA);
        return false;
    }
    return true;
}

// This function pulls a token and scans as an int, return true if all went
// ok.
bool Token2int(int *val)
{
    char *TK;
 
    if((TK = NextToken()) == NULL) return false;
    sscanf(TK,"%u",val);
    return true;
}

// This function pulls a token and scans as an float, return true if all went
// ok.
bool Token2float(float *fval)
{
    char *TK;
    
    if((TK = NextToken()) == NULL) return false;
    sscanf(TK,"%f",fval);
    return true;
}

// The following two functions enable the use of the Level Detection module and its lookup table 
// capability to trigger the table start. The value read from the lookup table is used to select 
// the table number and then a trigger is issued. This function attached an ISR to the digital 
// signal sent from the level module to MIPS.
void ChangeTriggerISR(void)
{
  float fval;
  byte *b;
  int  i=0;

  // Make sure we are in table mode and waiting for a trigger, if not exit
  if((TableMode != TBL) || (TableTriggered)) return;
  // Read the lookup value from the change detector
  b = (byte *)&fval;
  Wire1.beginTransmission(LevelDetAdd);
  Wire1.write(TWI_LEVDET_LOOKUP);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)LevelDetAdd, (uint8_t)4);
  while (Wire1.available()) b[i++] = Wire1.read();
  DefineTableNumber((int)fval);
  IssueSoftwareTableStart = true;
}

// TWIadd is the hex TWI address of the level detection module
void TriggerOnChange(char *TWIadd)
{
  LevelDetAdd = strtol(TWIadd,NULL,16);
  // Init the Level Detector
  pinMode(SCL1,INPUT);
  pinMode(SDA1,INPUT);
  pinMode(LEVCHANGE,INPUT);
  Wire1.begin();
  Wire1.setClock(100000);
  attachInterrupt(digitalPinToInterrupt(LEVCHANGE), ChangeTriggerISR, RISING);
  SendACK;  
}
// End of level detection module trigger code

// This command defines the current / active table number the system will use for all processing.
// This function was updated 6/14/2020. This update allow the table number to be changed event in
// table mode as long as the table is not triggered.

// Table number = 1 thru NumTables
int DefineTableNumber(int tblnum)  // Return -1 on error
{
  if((tblnum < 1) || (tblnum > NumTables)) return -1;
  if(TableMode == TBL)
  {
    if((TableTriggered) || (TablesLoaded[tblnum - 1] <= 0)) return -1;
    CT = --tblnum;
    SetupTimer();
    return 0;
  }
  CT = --tblnum;
  return 0;
}

// Table number = 1 thru NumTables
void SetTableNumber(int tblnum)
{
  if(DefineTableNumber(tblnum) == -1)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACK;
  return;
}

// This command returns the current / active table number the system is using for all processing
void GetTableNumber(void)
{
  SendACKonly;
  serial->println(CT+1);
}

// This command returns the table advance mode, ON or OFF
void GetTableAdvance(void)
{
  SendACKonly;
  if(TableAdv) serial->println("ON");
  else serial->println("OFF");
}

// This command sets the table advance mode, this can only be done in loc mode.
void SetTableAdvance(char *cmd)
{
    if(TableMode == TBL)
    {
        SetErrorCode(ERR_NOTBLMODE);
        SendNAK;
        return;
    }
    if(strcmp(cmd,"ON") == 0)
    {
      SendACK;
      TableAdv = true;
    }
    else if(strcmp(cmd,"OFF") == 0)
    {
      SendACK;
      TableAdv = false;
    }
    else SendNAK;
}

// This command will place the system in table mode and enable the triggers, both software
// and hardware.
void SetTableMode(char *cmd)
{
    if(strcmp(cmd,"LOC") == 0)
    {
        if(TableMode == LOC)
        {
            SetErrorCode(ERR_LOCALREADY);
            SendNAK;
            return;
        }
        SendACK;
        LOCrequest = true;
        return;
    }
    else if((strcmp(cmd,"TBL") == 0) || (strcmp(cmd,"ONCE") == 0))
    {
        if(TableMode == TBL)
        {
            SetErrorCode(ERR_TBLALREADY);
            SendNAK;
            return;
        }
        if(TablesLoaded[CT] == 0)
        {
            SetErrorCode(ERR_NOTBLLOADED);
            SendNAK;
            return;
        }
        if(strcmp(cmd,"TBL") == 0) TableOnce = false;
        else TableOnce = true;
        SendACK;
        ProcessTables();
        return;
    }
    else
    {
        SetErrorCode(ERR_BADARG);
        SendNAK;
        return;
    }
}

// This routine processes the clock command, valid input is a internal clock divider,
// internal clock frequency or EXT for externally supplied clock.
void SetTableCLK(char *cmd)
{
    int freq;
    
    if(TableMode == TBL)
    {
        SetErrorCode(ERR_NOTBLMODE);
        SendNAK;
        return;
    }
    if(strcmp(cmd,"EXT") == 0) ClockMode = EXT;
    else if(strcmp(cmd,"EXTN") == 0) ClockMode = EXTN;
    else if(strcmp(cmd,"EXTS") == 0) ClockMode = EXTS;
    else if(strcmp(cmd,"MCK2") == 0) ClockMode = MCK2;
    else if(strcmp(cmd,"MCK8") == 0) ClockMode = MCK8;
    else if(strcmp(cmd,"MCK32") == 0) ClockMode = MCK32;
    else if(strcmp(cmd,"MCK128") == 0) ClockMode = MCK128;
    else
    {
      freq = 0;
      sscanf(cmd,"%d",&freq); 
      if(freq == VARIANT_MCK/2) ClockMode = MCK2;
      else if(freq == VARIANT_MCK/8) ClockMode = MCK8;
      else if(freq == VARIANT_MCK/32) ClockMode = MCK32;
      else if(freq == VARIANT_MCK/128) ClockMode = MCK128;
      else
      {
        SetErrorCode(ERR_BADARG);
        SendNAK;
        return;
      }
    }
    TableClockFreq = 0;
    if(ClockMode == MCK2) TableClockFreq   = VARIANT_MCK/2;
    if(ClockMode == MCK8) TableClockFreq   = VARIANT_MCK/8;
    if(ClockMode == MCK32) TableClockFreq  = VARIANT_MCK/32;
    if(ClockMode == MCK128) TableClockFreq = VARIANT_MCK/128;
    SendACK;
}

// The routine sets the trigger mode flag
void SetTableTRG(char *cmd)
{
    if(TableMode == TBL)
    {
        SetErrorCode(ERR_NOTBLMODE);
        SendNAK;
        return;
    }
    if(strcmp(cmd,"SW") == 0)        TriggerMode = SW;
    else if(strcmp(cmd,"EDGE") == 0) TriggerMode = EDGE;
    else if(strcmp(cmd,"POS") == 0)  TriggerMode = POS;
    else if(strcmp(cmd,"NEG") == 0)  TriggerMode = NEG;
    else
    {
        SetErrorCode(ERR_BADARG);
        SendNAK;
        return;
    }
    SendACK;
}

// This function sets the table abort flag to stop any table processing
void SetTableAbort(void)
{
    if(!TableReady)
    {
        SetErrorCode(ERR_TBLNOTREADY);
        SendNAK;
        return;
    }
    Aborted=true;
    SendACK;
}

// This function stops the current table if its running and in table mode
void StopTable(void)
{
    if(TableMode != TBL)
    {
        SetErrorCode(ERR_NOTBLMODE);
        SendNAK;
        return;
    }
    StopCommanded = true;
    SendACK;
}

// Software trigger
void SWTableTrg(void)
{
    if(TableMode != TBL)
    {
        SetErrorCode(ERR_NOTBLMODE);
        SendNAK;
        return;
    }
    if(!TableReady)
    {
        SetErrorCode(ERR_TBLNOTREADY);
        SendNAK;
        return;
    }
    if((!MIPSconfigData.TableRetrig) && (TimerRunning == true))
    {
      SendACK;
      return;
    }
    // This function was changed Jan 22, 2019 to just set a flag and let the table real time loop processing
    // code actually call the start function. This allows the real time loop to call tasks (if enabled) while 
    // waiting for a trigger.
    IssueSoftwareTableStart = true;
    SendACK;
}

void PerformSoftwareStart(void)
{
    if(!IssueSoftwareTableStart) return;
    // If there is a update at count 0 then fire LDAC and setup for the next event
    StartTimer();
    if(MPT.getRAcounter() == 0) 
    {
      ProcessTableQueue();
      if(RampEnabled) RampClock->softwareTrigger();
      if((MIPSconfigData.Rev <= 1) || (softLDAC)) // Rev 1 used software control of LDAC
      {
        LDAClow;
        LDAChigh;
      }
      else   // here for rev 2 
      {
        // Added this LDAC toggle on 7/28/18. This loads the DACs at software trigger time
        // if there it a time point 0 event.
        LDACcapture;    
        LDACctrlLow;
        LDACctrlHigh;
        LDACrelease;
      }
      SetupNextEntry();
    }
    IssueSoftwareTableStart = false;
}

// Report table current frequency setting
void TableFreq(void)
{
  int   freq;
  
  if(ClockMode == MCK2)   freq = VARIANT_MCK/2;
  if(ClockMode == MCK8)   freq = VARIANT_MCK/8;
  if(ClockMode == MCK32)  freq = VARIANT_MCK/32;
  if(ClockMode == MCK128) freq = VARIANT_MCK/128;
  SendACKonly;
  if(!SerialMute) serial->println(freq);
}

// This function will look through the loaded table buffer and attempt to find the Table
// entry at the time count and channel specified in the call. If the table entry is found its pointer
// is returned. If its not found NULL is returned.
// Table name == 0 at the end of all tables.
TableEntry *FindTableEntry(int Count,int Chan)
{
   int   i=0;
   int   j,k;
   TableHeader  *TH;
   TableEntryHeader *TEH;
   TableEntry *TE;
   char LastChan;
    
   TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   while(1)
   {
       if(i >= MaxTable) return NULL;
       // Make sure there is a table header
       if(TH->TableName == 0) return NULL;
       // Loop thhrough all the entries in the table
       for(k=0;k<TH->NumEntries;k++)
       {
           TEH = (TableEntryHeader *) &(VoltageTable[CT][i]); i += sizeof(TableEntryHeader);
           TE = (TableEntry *) &(VoltageTable[CT][i]);
           for(j=0;j<TEH->NumChans;j++)
           {
               LastChan = TE[j].Chan;
               if((TEH->Count == Count) && (TE[j].Chan == (Chan-1))) return(&TE[j]);
           }
           // Advance to next Table entry
           i += sizeof(TableEntry) * TEH->NumChans;
       }
       // Advance to next table
       TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   }
}

// This function will look through the loaded table buffer and attempt to find the Table header
// entry at the time count and channel specified in the call. If the table header entry is found its pointer
// is returned. If its not found NULL is returned.
// Table name == 0 at the end of all tables.
TableEntryHeader *FindTableEntryHeader(int Count,int Chan)
{
   int   i=0;
   int   j,k;
   TableHeader  *TH;
   TableEntryHeader *TEH;
   TableEntry *TE;
   char LastChan;
    
   TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   while(1)
   {
       if(i >= MaxTable) return NULL;
       // Make sure there is a table header
       if(TH->TableName == 0) return NULL;
       // Loop thhrough all the entries in the table
       for(k=0;k<TH->NumEntries;k++)
       {
           TEH = (TableEntryHeader *) &(VoltageTable[CT][i]); i += sizeof(TableEntryHeader);
           TE = (TableEntry *) &(VoltageTable[CT][i]);
           for(j=0;j<TEH->NumChans;j++)
           {
               LastChan = TE[j].Chan;
               if((TEH->Count == Count) && (TE[j].Chan == (Chan-1))) return(TEH);
           }
           // Advance to next Table entry
           i += sizeof(TableEntry) * TEH->NumChans;
       }
       // Advance to next table
       TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   }
}

// Returns the voltage value at a selected time point and channel number.
void GetTableEntryValue(int Count, int Chan)
{
    float fval;
    TableEntry *TE;
    
    TE = FindTableEntry(Count, Chan);
    if(TE==NULL)
    {
        SetErrorCode(ERR_CANTFINDENTRY);
        SendNAK;
        return;
    }
    // Convert entry back to engineering units and send
    SendACKonly;
    // The value is saved as the DAC bit pattern so first convert back to DAC counts
    uint8_t  *buf = (uint8_t *)&TE->Value;
    int val = ((int)buf[1] & 0x0F) << 12;
    val |= ((int)buf[2]) << 4;
    val |= ((int)buf[3]) >> 4;
    fval = DCbiasCounts2Value(TE->Chan, val);
    if(!SerialMute) serial->println(fval);
}

// Set a voltage level at a time point channel point.
void SetTableEntryValue(int Count, int Chan, float fval)
{
    
    TableEntry *TE;
    
    TE = FindTableEntry(Count, Chan);
    if(TE==NULL)
    {
        SetErrorCode(ERR_CANTFINDENTRY);
        SendNAK;
        return;
    }
    // Convert to engineering units and write to table
    int val = DCbiasValue2Counts(TE->Chan, fval);
    // Make value into bit image the DAC wants to see. This will allow fast DAC updating
    // in the table execute mode. June 23 2016
    uint8_t  *buf = (uint8_t *)&TE->Value;
    buf[0] = 0;         // DAC command
    buf[1] = ((TE->Chan & 7) << 4) | (val >> 12);
    buf[2] = (val >> 4);
    buf[3] = (val << 4);
    SendACK;
}

// Table timing test time values in uS
#define FixedSetupTime   7
#define DCBchanTime      11
#define DIOchanTime      19
#define FunctionTime     50  // This number has not been validated

// The function checks a table and looks for any timing violations.
void TableCheck(void)
{
   int   i=0;
   int   j,k;
   int   setupT;    // Needed setup time in uS
   int   lastTimePoint = -1;
   float Period=0;
   TableHeader      *TH;
   TableEntryHeader *TEH;
   TableEntry       *TE;

   serial->println("Inspecting table....");
   if(TablesLoaded[CT] == 0)
   {
      serial->println("No table defined!\n");
      return;
   }
   // Calculate the clock period, if external clock then the ExtFreq value is used, if
   // ExtFreq is undefined this function quits.
   if(ClockMode == MCK2)   Period = 2.0/(float)VARIANT_MCK;
   if(ClockMode == MCK8)   Period = 8.0/(float)VARIANT_MCK;
   if(ClockMode == MCK32)  Period = 32.0/(float)VARIANT_MCK;
   if(ClockMode == MCK128) Period = 128.0/(float)VARIANT_MCK;
   if((Period == 0) && (ExtFreq == 0))
   {
       serial->println("System in external clock mode and external frequency");
       serial->println("is not defined, can't evaluate table!");
       return;
   }
   if(Period == 0) Period = 1.0 / (float)ExtFreq;
   Period *= 1000000.0;  // Period in uS
   //serial->println(Period);
   // Walk through the table 
   TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   while(1)
   {
       if(i >= MaxTable) break;
       // Make sure there is a table header
       if(TH->TableName == 0) break;
       // Loop thhrough all the entries in the table and sum the times
       for(k=0;k<TH->NumEntries;k++)
       {
           TEH = (TableEntryHeader *) &(VoltageTable[CT][i]); i += sizeof(TableEntryHeader);
           TE = (TableEntry *) &(VoltageTable[CT][i]);
           setupT = FixedSetupTime;
           bool DIOdetected = false;
           for(j=0;j<TEH->NumChans;j++)
           {
               // DC bias channel
               if((TE[j].Chan >= 0)&&(TE[j].Chan <= 31)) setupT += DCBchanTime;
               // if Chan is A through P its a DIO to process
               else if((TE[j].Chan >= 'A') && (TE[j].Chan <= 'P')) DIOdetected = true;
               else if(TE[j].Chan == ']') setupT += 2;
               else if(TE[j].Chan == '[') setupT += 2;
               // Everything else, assume the slowest function
               else setupT += FunctionTime;               
           }
           if(DIOdetected) setupT += DIOchanTime;
           // SetupT now has the needed time for this event to load all the parameters. The previous
           // time point must provide this time or there is a timing violation
           if(lastTimePoint > 0)
           {
              //serial->println(setupT);
              //serial->println(((TEH->Count - lastTimePoint) * Period));
              // Calculate the required number of clocks and check
              if(((TEH->Count - lastTimePoint) * Period) < setupT)
              {
                 // Here if we have a timing violation
                 serial->print("Timing violation at time point: ");
                 serial->println(abs(TEH->Count));
                 serial->print("The previous time can not exceed: ");
                 serial->println(abs(TEH->Count) - (int)(((float)setupT / Period) + 0.5));
                 serial->print("But it is: ");
                 serial->println(lastTimePoint);
              }
           }
           lastTimePoint = abs(TEH->Count);
           // Advance to next Table entry
           i += sizeof(TableEntry) * TEH->NumChans;
       }
       // Advance to next table
       TH = (TableHeader *) &(VoltageTable[CT][i]); i += sizeof(TableHeader);
   }
   serial->println("Done!");
}

// Set a new count or time value at a time point channel point.
void SetTableEntryCount(int Count, int Chan, float NewCount)
{
    
    TableEntryHeader *TEH;
    
    TEH = FindTableEntryHeader(Count, Chan);
    if(TEH==NULL)
    {
        SetErrorCode(ERR_CANTFINDENTRY);
        SendNAK;
        return;
    }
    // Write new time count value
    TEH->Count = (int)NewCount;
    SendACK;
}

//
// This function parses a table command.
//
// As discussed in this file header the table formated memory block is defined
// below.
//
// Multiple tables are in the list, 0xFF in table name flags the end of tables.
//
// Table:
//   Table name (byte)
//   Repeat count (word)
//   Max count (word)
//   NumEntries (word)
//   Entry format:
//      Count (word), timer count when the values are latched via LDAC
//      NumChans (byte), number of output values to follow, ']' flags end
//      ChanValue
//          Chan (byte), 0-15 if DC output, alpha character if DIO
//          Value (word)
//  Next table or 0xFF marker for end of tables.
//
// This function will respond with ACK or NAK and define the number of tables loaded
//
// Logic:
//  1. Get first value, it has to be a time
//  2. If next token is a [ then this is a table start so parse table name and repeat
//     count and create.
//  3. If next token is a value then its a channel number so create a table with name 0xFF
//     and repeat count of 1.
//  4. The ] is save in the table and processed at run time, it also defines a new table
//     but its not saved in nesting stack.

void ParseTableCommand(void)
{
    int         InitialOffset = 0;
    int         i,iStat;
    char        *TK;
    TableHeader *TH;
    TableEntryHeader *TEH;
   
    // If we are in table mode then flush this message and NAK
    if(TableMode == TBL)
    {
        while((TK=NextToken()) != NULL);
        SetErrorCode(ERR_NOTLOCMODE);
        SendNAK;
        return;
    }
    // Init processing loop
    ptr              = 0;    // Memory block pointer
    TablesLoaded[CT] = 0;    // Number of tables loaded
    TestNesting      = 0;    // Clear this error counter
    iStat            = 0;
    VoltageTable[CT] = (unsigned char *)realloc((void *)VoltageTable[CT], 1000);
    MaxTable = 1000;
    while(1)
    {
        // Start of table, pointer setup
        TH = (TableHeader *) &(VoltageTable[CT][ptr]); ptr += sizeof(TableHeader);
        TH->TableName = 0xFF;  // set default
        TH->RepeatCount = 1;   // set default
        TH->NumEntries=0;      // clear entries
        // Scan the counts value and read the next token that follows the :
        // if this token is a [ then the table name and repeat count will follow,
        // if is not a [ then assume its is a normal (simple) table start.
        if(iStat == 0) // Only do this the very first time through the while loop
        {
           if(!Token2int(&i)) break;
           if(!ExpectColon()) break;
           if((TK = NextToken()) == NULL) break;
        }
        if((TK[0] == '[') || (iStat == PENewNamedTable))
        {
            if((iStat == 0) && (i > 0))  // Added the i > 0 test dec 20, was broke from nov 3 to dec 20
            {
              // Here if the table starts with a non zero value, this indicates a delay so make a table entry for the delay 
              // with no output action. Nov 3, 2016
              TH->MaxCount = i;
              TH->NumEntries=1;
              TEH = (TableEntryHeader *) &(VoltageTable[CT][ptr]); ptr += sizeof(TableEntryHeader);
              TEH->Count = i;
              TEH->NumChans = 0;
              TH = (TableHeader *) &(VoltageTable[CT][ptr]); ptr += sizeof(TableHeader);
              TH->TableName = 0xFF;  // set default
              TH->RepeatCount = 1;   // set default
              TH->NumEntries=0;      // clear entries
            }
            if(iStat != PENewNamedTable) TestNesting++;
            // This is start of table, next two values define table name
            // and repeat count
            if((TK = NextToken()) == NULL) break; // Get table name
            TH->TableName = TK[0];
            if(!ExpectColon()) break;
            if(!Token2int(&TH->RepeatCount)) break;
            if(!ExpectComma()) break;
            // Now get the initial time point value
            if(!Token2int(&i)) break;                   // First time point
            if(!ExpectColon()) break;
            if((TK = NextToken()) == NULL) break;       // First channel
        }
        // At this point the table is initalized and i has the first time
        // point value for the table. This is the time point for the first
        // entry in table. TK contains the token for the first entry.
        TH->MaxCount = InitialOffset+i;
        while(1)
        {
            if((MaxTable - ptr) < (sizeof(TableHeader) + sizeof(TableEntryHeader) + sizeof(TableEntry)))
            {
                VoltageTable[CT] = (unsigned char *)realloc((void *)VoltageTable[CT], MaxTable += 1000);
                if(VoltageTable[CT] == NULL)
                {
                   // Out of space!
                   iStat = PEerror;
                   SetErrorCode(ERR_TBLTOOBIG);
                   break;
                }
            }
            iStat = ParseEntry(i,TK);
            if(iStat == PEerror) break;
            else if(iStat == PEprocessed)
            {
                // Read the next count and then the next token and
                // process next entry.
                TH->NumEntries++;
                iStat = PEerror;    // in case we break!
                if(!Token2int(&i)) break;
                // Get the next token, if its a ] then process else it better be a :
                if((TK = NextToken()) == NULL) break;
                if(TK[0] != ']')
                {
                  if(TK[0] != ':') break;
                  if((TK = NextToken()) == NULL) break;                  
                }
                TH->MaxCount = InitialOffset+i;
            }
            else if(iStat == PENewNamedTable)
            {
                // The next token will be the table name
                TablesLoaded[CT]++;
                TH->NumEntries++;  // gaa april 1, 2015
                InitialOffset=0;
                break;
            }
            else if(iStat == PENewTable)
            {
                // The next token will be the first entries count value
                iStat = PEerror;    // in case we break!
                if(!Token2int(&i)) break;
                if(!ExpectColon()) break;
                if((TK = NextToken()) == NULL) break;
                iStat = PENewTable;
                TH->NumEntries++;
                TablesLoaded[CT]++;
                InitialOffset=0;
                break;
            }
            else if(iStat == PEEndTables)
            {
                TH->NumEntries++;
                TablesLoaded[CT]++;
                // Mark the next table name with a 0 to flag the end
                TH = (TableHeader *) &(VoltageTable[CT][ptr]);
                TH->TableName = 0;
                //ReportTable(ptr);
                SendACK;
                return;
            }
        }
        if(iStat == PEerror) break;
    }
    // Error exit
    // Flush the command buffer
    while((TK=NextToken()) != NULL);
    TablesLoaded[CT]=0;
    SendNAK;
}

// This function parses the next table entry from the input string. Its called with the
// current entry count and the first string token pointer already loaded with the token.
// This function returns the following complete codes:
//
//  PEprocessed, entry was processed and the next token is the next entry.
//  PENewNamedTable, entry was processed and a [ was received indicating a new named table is next.
//  PENewTable, entry was processed and a ] was received indicating a new table is next.
//  PEerror - error exit
int ParseEntry(int Count, char *TK)
{
    int              i;
    float            fval;
    TableEntryHeader *TEH;
    TableEntry       *TE;
    
    // Define Entry pointer and initalize
    TEH = (TableEntryHeader *) &(VoltageTable[CT][ptr]); ptr += sizeof(TableEntryHeader);
    TEH->Count = Count;
    TEH->NumChans = 0;
    TE = (TableEntry *) &(VoltageTable[CT][ptr]);
    while(1)
    {
        // Process each entry, TK has token for first channel.
        // This token could be on of the following:
        //   - A DCB channel number 1 through 32
        //   - A DIO channel number, A through P or t for Trigger output
        //   - A 'W', if this is true do nothing because all we do is set max count with time point
        //   - A ']', This will define the end of the table.
        //   - A '[', Defines a new named table
        //   - A ',', Indicates its the end of this entry so return
        //   - A ';', Indicating the end of the table command
        if(TK[0] == ',')
        {
            return PEprocessed;
        }
        else if(TK[0] == ';')
        {
            return PEEndTables;
        }
        else if(TK[0] == 'W')
        {
            if((TK = NextToken()) == NULL) break;
            continue;
        }
        else if(TK[0] == '[')
        {
            TestNesting++;
            if(TestNesting > MaxNesting)
            {
                SetErrorCode(ERR_NESTINGTOODEEP);
                break;
            }
//            if(TEH->NumChans == 0) ptr -=sizeof(TableEntry);
            return PENewNamedTable;
        }
        else if(TK[0] == ']')
        {
            TestNesting--;
            if(TestNesting < 0)
            {
                SetErrorCode(ERR_MISSINGOPENBRACKET);
                break;
            }
            TE->Chan = ']';
            TE->Value = 0;
            TEH->NumChans++;
            ptr += sizeof(TableEntry);
            if((TK = NextToken()) == NULL) break;
            // If token is a ; then this is the end of all tables
            if(TK[0] == ';') return PEEndTables;
            return PENewTable;
        }
        else if(((TK[0] >= 'A') && (TK[0] <= 'P')) || (TK[0] == 'r') || (TK[0] == 's') || (TK[0] == 't') || (TK[0] == 'b') || (TK[0] == 'c') || (TK[0] == 'd') || (TK[0] == 'p'))
        {
            // DIO channel number
            TE->Chan = TK[0];
            if(!ExpectColon()) break;
            if((TE->Chan == 't') || (TE->Chan == 'b') || (TE->Chan == 'd') || (TE->Chan == 'p')) Token2int(&TE->Value);
            else
            {
               if((TK = NextToken()) == NULL) break;
               TE->Value = TK[0];
            }
        }
        else
        {
            // DC bias channel number or ARB commands (101 thru 108, 'e' thru 'l')
            sscanf(TK,"%d",&i);
            TE->Chan = i;
            if(!ExpectColon()) break;
            if(!Token2float(&fval)) break;
            *((float *)(&TE->Value)) = fval;  // Put the float value in the 32 bit int, for the ARB commands
            if(((i>=1)&&(i<=32)) || ((i&RAMP)!=0))
            {
               TE->Chan = i-1;
               // Convert value to DAC counts
               TE->Value = DCbiasValue2Counts(TE->Chan & 0x1F, fval);
               if((i&(RAMP | INITIAL))==RAMP)
               {
                  TE->Value -= DCbiasValue2Counts(TE->Chan & 0x1F, 0.0);
                  TE->Value = (TE->Value << 4) & 0xFFFF0;
               }
               else
               {
                  // Make value into bit image the DAC wants to see. This will allow fast DAC updating
                  // in the table execute mode. June 23 2016
                  uint8_t  *buf = (uint8_t *)&TE->Value;
                  int val = TE->Value;
                  buf[0] = 0;         // DAC command
                  //buf[1] = ((TE->Chan & 7) << 4) | (val >> 12);
                  buf[1] = ((DCbiasChan2DAC(TE->Chan) & 7) << 4) | (val >> 12);   // This fixes a bug in the emision production system, it has channel numbers
                                                                                  // that have been remapped for PC board layout reasons. Sept 26, 2020
                  buf[2] = (val >> 4);
                  buf[3] = (val << 4);
              }
            }
        }
        if((TK = NextToken()) == NULL) break;  // get next token to process
        // If this token is a : then another channel value pair follow
        // so read the channel
        if(TK[0] == ':') if((TK = NextToken()) == NULL) break;
        TEH->NumChans++;
        ptr += sizeof(TableEntry);
        TE = (TableEntry *) &(VoltageTable[CT][ptr]);
    }
    return PEerror;
}

// This is a debug function used to print the table data block.
void ReportTable(int count)
{
    // Test code to spit back the data stored in the voltage table as a
    // result of the parsed command string.
    int i;
    char str[60];

    if(SerialMute) return;
    sprintf(str,"TestNesting = %d\n",TestNesting);
    serial->write(str);
    sprintf(str,"TablesLoaded = %d\n",TablesLoaded[CT]);
    serial->write(str);
    sprintf(str,"Size of TableHeader = %d\n",sizeof(TableHeader));
    serial->write(str);
    sprintf(str,"Size of TableEntryHeader = %d\n",sizeof(TableEntryHeader));
    serial->write(str);
    sprintf(str,"Size of TableEntry = %d\n",sizeof(TableEntry));
    serial->write(str);
    for(i=0;i<count+1;i++)
    {
        sprintf(str,"%x\n",VoltageTable[CT][i]);
        serial->write(str);
    }
}

//**************************************************************************************************
//
// This section of the file contains all real time processing routines.
//
//**************************************************************************************************

// This function is called at LDAC trigger time to update the compress line that connects to the 
// ARBs, all ARBs share the same compress line.
void ProcessCompress(void)
{
  if(Cstate == '1') digitalWrite(ARBmode,HIGH);
  else if(Cstate == '0') digitalWrite(ARBmode,LOW);
}

// This function is called at LDAC trigger time to update the sync line that connects to the 
// ARBs, all ARBs share the same sync line.
void ProcessSync(void)
{
  if(Sstate == '1') digitalWrite(ARBsync,HIGH);
  else if(Sstate == '0') digitalWrite(ARBsync,LOW);
}

// This function will advance to the next table number that is active if the table advance
// flag is true
void AdvanceTableNumber(void)
{
  if(!TableAdv) return;
  while(1)
  {
    if(++CT >= NumTables) CT = 0;
    if(TablesLoaded[CT] != 0) break;
  }
}

void ProcessTasks(void)
{
  uint32_t TCcount,RAcount,RCcount,AvalibleClocks;

  if(!TblTasks) return;   // Flag to enable this mode that will call tasks if there is time avalible to do so
  if(Ramping) return;
  if(!TimerRunning)
  {
    // If we are software trigger mode its OK to call the task processor
    if(TriggerMode == SW) control.run();
    return;
  }
  if((ClockMode == EXT) || (ClockMode == EXTN) || (ClockMode == EXTS)) TableClockFreq = ExtFreq;
  if(TableClockFreq == 0) return;
  // If the timer is running test the avalible time
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    TCcount = MPTtc.TC_CV;
    RAcount = MPTtc.TC_RA;
    RCcount = MPTtc.TC_RC;
  }
  if(TCcount < RAcount) AvalibleClocks = RAcount - TCcount;
  if((RCcount - TCcount) < AvalibleClocks) AvalibleClocks = RCcount - TCcount;
  if(((AvalibleClocks * 1000) / TableClockFreq) > 40)
  {
    // If here its OK to call the task processor and run ready tasks
    control.run();
  }
}

//
// This function places the system in table mode and sets up the timer to play a table.
// TableModeEnabled is written to the display and all button processing is suspended.
//
void ProcessTables(void)
{
    int  InitialTableNum;
    bool bStat;
   
    InitialTableNum = CT;
    // Set mode to TBL
    TableMode = TBL;
    // Takeover the display and print message that we are in the table
    // mode. Allow pressing the button to cause an abort
    DisplayMessage("Table Mode Enabled");
    StopCommanded = false;
    StartRampClock();
    // Setup the table mode and start timer
    while(1)
    {
        if((BrightTime + 30000) < millis()) SetBackLight();
        WDT_Restart(WDT);
        // Fall into a processing loop and remain in this loop until the timer completes
        LOCrequest = false;
        Aborted = false;
        if((!SerialMute) && (TableResponse)) serial->println("TBLRDY\n");
        TableReady = true;
        StopRequest=false;
        TableStopped = false;
        // Setup the timer
        SetupTimer();
        while(1)
        {
            WDT_Restart(WDT);
            // Issue software trigger if requested
            PerformSoftwareStart();
            // Process tasks if time is avalible
            ProcessTasks();
            // Restart table if reqested by user
            if(StopCommanded)
            {
                if((!SerialMute) && (TableResponse)) serial->println("Table stoped by user");
                break;
            }
            // If the timer has been triggered update the displayed status
            {
               AtomicBlock< Atomic_RestoreState > a_Block;
               bStat = MPT.checkStatusBit(TC_SR_ETRGS);  
            }
            if(bStat || SWtriggered)
            {
              TableTriggered = true;
              SWtriggered    = false;
              // Here when triggered
              if((!SerialMute) && (TableResponse)) serial->println("TBLTRIG\n");
//            if(StopRequest == true) break;     // removed 5/5/18
            }
            // Exit this loop when the timer is stoped.
            {
               AtomicBlock< Atomic_RestoreState > a_Block;
               bStat = MPT.checkStatusBit(TC_SR_CLKSTA);  
            }
            if(!bStat || TableStopped)
            {
                // Issue the table complete message
                TableTriggered = false;
                if((!SerialMute) && (TableResponse)) serial->println("TBLCMPLT\n");
                if((TriggerMode == EDGE)||(TriggerMode == POS)||(TriggerMode == NEG)) 
                {
                  if((!SerialMute) && (TableResponse)) serial->println("TBLRDY\n");
                  TableStopped = false;
                }
                // Exit the loop and setup for another trigger
                else break;
            }
            // Also exit this loop if the table mode is aborted.
            if(Aborted || LOCrequest)
            {
                // Exit loop and exit local mode
                break;
            }
            // Look for the button being held down, if detected then exit
            #ifndef TestMode
            if(digitalRead(PB)==HIGH)
            {
                // Make sure button is released
                while(digitalRead(PB)==HIGH);
                if((!SerialMute) && (TableResponse)) serial->println("ABORTED by user\n");
                LOCrequest = true;
                break;
            }
            #endif
            // Process any serial commands
            ProcessSerial();
            if(AcquireADC())
            {
               if(ReadVin() < 10.0) break;
               ReleaseADC();           
            }
            serial->flush();
            // If full command processing in table mode is enabled then run tasks.
            if(TasksEnabled)
            {
               // delayMicroseconds(100); 
               // control.run();
               // Changed nov 26, 2017
               uint32_t now = millis();
               delayMicroseconds(200);
               while((now + InterTableDelay) > millis()) control.run();
               TRACE(8);
            }
            else
            {
              delay(InterTableDelay);
            }
        }
        if(Aborted)
        {
            // Issue an aborted message.
            if((!SerialMute) && (TableResponse)) serial->write("ABORTED\n");
        }
        if(Aborted || LOCrequest)
        {
            StopTimer();
            break;
        }
        if(AcquireADC())
        {
           if(ReadVin() < 10.0) break;
           ReleaseADC();           
        }
        if(TableOnce) break;
        StopTimer();  // not sure about this, testing
        // Advance to next table if advance mode is enabled
        AdvanceTableNumber();
        if(StopCommanded) StopCommanded = false;
        TableTriggered = false;
    }
    // Clean up and exit
    StopRampClock();
    TableTriggered = false;
    StopCommanded = false;
    TableReady = false;
    LOCrequest = false;
    Aborted = false;
    TableMode = LOC;
    StopTimer();
    LDACcapture;
    pinMode(LDAC,OUTPUT);
    LDAClow;
    // Reset all the digital outputs to there pre-table states
    SetImageRegs();
    DismissMessage();
    CT = InitialTableNum;
    DCbiasUpdate = true;
}

// This function is called after all the table parameters are defined. This function will setup timer 3
// used for the table function and prepair the system for a trigger. A table has to be loaded for this
// function to do anything.
//
// The timer is in CTC mode (12). Two interrupts are enabled, one for compare match and one for
// timer overflow.
void SetupTimer(void)
{
    AtomicBlock< Atomic_RestoreState > a_Block;
    // Exit and do nothing if no tables are loaded
    if(TablesLoaded[CT] <= 0) return;
    // Setup / init the table pointers
    Theader = (TableHeader *)&(VoltageTable[CT][0]);
    TEheader = (TableEntryHeader *)&(VoltageTable[CT][sizeof(TableHeader)]);
    Tentry = (TableEntry *)&(VoltageTable[CT][sizeof(TableHeader)+sizeof(TableEntryHeader)]);
    TentryCount = 0;             // Current table enter index value
    // If the new table is named then place it on the nesting stack
    NS.Ptr = 0;             // Clear the nesting stack
    if((Theader->TableName != 0xFF) && (Theader->TableName != 0))
    {
        NS.Table[(int)NS.Ptr] = Theader;
        NS.Count[(int)NS.Ptr] = 0;
        NS.Ptr++;
    }
    // Setup the channel to board number array this is used in table processing for speed!
    int i,k,j=0;
    
    for(i=0;i<MAXDCbiasMODULES;i++)
    {
      if(DCbDarray[i] != NULL)
      {
        for(k=0;k<8;k++) Chan2Brd[k+j] = ((DCbDarray[i]->DACspi << 4) & 0x70) | (i & 1);
        j += 8;
      }
    }
    // Setup counter, use MPT
    MPT.begin();  
    // Need to make sure the any pending interrupts are cleared on TRIGGER. So if we are in a hardware
    // triggered mode we will first enable the trigger to a dummy ISR and then detach and reattached to the
    // real ISR. We will wait a few microseconds for the interrupt to clear.
    if(TriggerMode == EDGE) {DIhTrig.attached('R', CHANGE, Dummy_ISR);  delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == POS)  {DIhTrig.attached('R', RISING, Dummy_ISR);  delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == NEG)  {DIhTrig.attached('R', FALLING, Dummy_ISR); delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == SW)   {DIhTrig.detach(); MPT.setTrigger(TC_CMR_EEVTEDG_NONE);}
    if(TriggerMode == EDGE) {DIhTrig.attached('R', CHANGE, Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_EDGE);}
    if(TriggerMode == POS)  {DIhTrig.attached('R', RISING, Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_RISING);}
    if(TriggerMode == NEG)  {DIhTrig.attached('R', FALLING,Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_FALLING);}
    if(ClockMode == EXT)    
    {
      MPT.setClock(TC_CMR_TCCLKS_XC2);
      MPTtc.TC_CMR &= ~TC_CMR_CLKI;
    }
    if(ClockMode == EXTN)   
    {
      MPT.setClock(TC_CMR_TCCLKS_XC2);
      MPTtc.TC_CMR |= TC_CMR_CLKI;
    }
    if(ClockMode == EXTS)
    {
      softLDAC = true;
      MPT.setClock(TC_CMR_TCCLKS_XC2);
    }
    if(ClockMode == MCK2)   MPT.setClock(TC_CMR_TCCLKS_TIMER_CLOCK1);
    if(ClockMode == MCK8)   MPT.setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);
    if(ClockMode == MCK32)  MPT.setClock(TC_CMR_TCCLKS_TIMER_CLOCK3);
    if(ClockMode == MCK128) MPT.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);
    // Setup the callback registers
    MPT.attachInterruptRA(RAmatch_Handler);
    MPT.attachInterrupt(RCmatch_Handler);
    // Drive LDAC high
    if((MIPSconfigData.Rev > 1) && (!softLDAC))
    {
       LDACrelease;
    }
    else
    {
       LDAChigh;
    }
    SPI.setClockDivider(SPI_CS,6);
    SPI.setDataMode(SPI_CS, SPI_MODE1);
    Spi* pSpi = SPI0;
    pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] &= 0xFFFFFF;  // Set DLYBCT delay between bytes to zero
    // If using the S input and a soft timer enable the clock
    if(ClockMode == EXTS) ClockSsetup();
    // Define the initial max counter value based on first table
    MPT.setRC(Theader->MaxCount);
    // Setup table variables
    if((MIPSconfigData.Rev <= 1) || (softLDAC)) MPT.setTIOAeffectNOIO(TEheader->Count,TC_CMR_ACPA_TOGGLE);
    // else MPT.setTIOAeffect(TEheader->Count,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE | TC_CMR_AEEVT_TOGGLE);  // Before the Nov 3, 2016 edits
    else 
    {
      // This conditional added Nov 3, 2016
      // The Toggle A output on trigger has to happen because we do not get a counter 0 event when triggered externally.
      // This does cause an issue if we do not have time 0 value in the DAC latches. If we external trigger then we need to do the
      // first SetupNext call in the trigger ISR
      // Added TC_CMR_ASWTRG_TOGGLE, 5/5/18 this fixes bug when SW trigger and time 0 point.
      // Removed TC_CMR_ASWTRG_TOGGLE on 7/28/18. The toogle happen after the software trigger and after the first clock edge
      // this caused a problem because there is no event to setup after this first clock pulse
      if(TEheader->Count == 0) MPT.setTIOAeffect(TEheader->Count,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE | TC_CMR_AEEVT_TOGGLE);
      else MPT.setTIOAeffect(TEheader->Count,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE);
    }
    //
    // Adjust the system interrupt priorities, Rev 1.07 update
    // 
    NVIC_SetPriority (SysTick_IRQn, 8);
    NVIC_SetPriority(WIRE1_ISR_ID, 8);
    NVIC_SetPriority(WIRE_ISR_ID, 8);
    NVIC_SetPriority((IRQn_Type) ID_UOTGHS, 8UL);
    //
    TimeDelta = 0;
    SetupNextEntry();
    MPT.enableTrigger();
}

// Called by hardware ISR or command to start the table processing
void StartTimer(void)
{
    SWtriggered = true;
    // Exit and do nothing if no tables are loaded
    if(TablesLoaded[CT] <= 0) return;
    // Start the timer
    TimerRunning = true;
    StartRampClock();
    MPT.softwareTrigger();
}

inline void StopTimer(void)
{
    // Exit and do nothing if no tables are loaded
    if(TablesLoaded[CT] <= 0) return;
    // Stop the timer
    MPT.stop();
    DIhTrig.detach();
    TimerRunning = false;
    if(ClockMode == EXTS) ClockSstop();
}

// Advance table pointer, this function assumes that the current table is pointing at the
// last entry.
// Return false if we reached the end of tables.
inline bool AdvanceTablePointer(void)
{
    // If the current table is named then its on the nesting stack so remove it if its 
    // repeat count has expired
    if((Theader->TableName != 0) && (Theader->TableName != 0xFF))
        if((NS.Count[NS.Ptr-1] >= NS.Table[NS.Ptr-1]->RepeatCount)  && (NS.Table[NS.Ptr-1]->RepeatCount != 0)) NS.Ptr--;
    if(NS.Ptr < 0) NS.Ptr = 0;
    // All entries have been played so advance to next table
    Theader = (TableHeader *)((char *)TEheader + (sizeof(TableEntryHeader) + (sizeof(TableEntry) * TEheader->NumChans)));
    TEheader = (TableEntryHeader *)((char *)Theader + sizeof(TableHeader));
    Tentry = (TableEntry *)((char *)TEheader + sizeof(TableEntryHeader));
    TentryCount = 0;
    if(Theader->TableName == 0) return false;
    // If the new table is named then place it on the nesting stack
    if(Theader->TableName != 0xFF)
    {
        NS.Table[(int)NS.Ptr] = Theader;
        NS.Count[(int)NS.Ptr] = 0;
        NS.Ptr++;
    }
    return true;
}

// Advance to the next entry in current table
inline void AdvanceEntryPointer(void)
{
    TEheader = (TableEntryHeader *)((char *)TEheader + (sizeof(TableEntryHeader) + (sizeof(TableEntry) * TEheader->NumChans)));
    Tentry = (TableEntry *)((char *)TEheader + sizeof(TableEntryHeader));
}

// This function is called to setup the next time point values used by the time
// table system.
// The global table structure headers are used and updated.
// There are two modes of operation defined at compile time, if the FAST_TABLE mode is defined then the inline version
// if used with a couple of missing features but it is a lot faster.

// This function will overload the weak function in the MIPStimer function. This code
// supports the maximum posible performance. This codes does not support some of the advanced
// features in the lower performance code, specificaly the t and b table functions.
#ifdef FAST_TABLE
void TC8_Handler() 
{
  static int i;
  
  i = MPTtc.TC_SR;
  if(i & TC_SR_CPAS) SetupNextEntry();
  if(i & TC_SR_CPCS)
  {
    if(StopRequest == true)
    {
        StopTimer();
        StopRequest = false;
        return;
    }
    // If compare register A is zero then at this point call setup next,
    // also if rev 1 pulse LDAC as well
    if(MPT.getRAcounter() == 0) SetupNextEntry();
  }
}
#endif

// This function allows a clock for the timer to appear on the 'S' input BNC and then the timer
// function is emulated using an internal counter register. This interrupt is called everytime 
// there is a rising edge on the 'S' input BNC. This input is also debounced.
void ISRclk(void)
{
  static Pio *pio = g_APinDescription[SoftClockDIO].pPort;
  static uint32_t pin =g_APinDescription[SoftClockDIO].ulPin;
  bool Amatch,Cmatch;

  // Exit if counter is not enabled
//  if(!TableTriggered) return;
  if(!TimerRunning) return;
  // Validate its a rising edge and debounce
  delayMicroseconds(1);    // Delay and then make sure the S input pin is high
  if((pio->PIO_PDSR & pin) == 0) return;  
  // Advance the counter
  Counter++;
  // Compare to RA and call ISR if equal
  if(Counter == MPTtc.TC_RA) Amatch = true;
  else Amatch = false;
  // Compare to RC and call ISR if equal and reset the count,
  // Changed from == to >= July 1, 2019. Bush lab found bug with nested loops and this fixed?
  // this change makes it work but the total table count is long, something else is going on.
  if(Counter >= MPTtc.TC_RC) Cmatch = true;
  else Cmatch = false;
  if(Amatch)
  {
    if(StopRequest) StopTimer();
    RAmatch_Handler();
  }
  if(Cmatch)
  {
     RCmatch_Handler();
     Counter = 0;
  }
}

void ClockSsetup(void)
{
   attachInterrupt(digitalPinToInterrupt(SoftClockDIO), ISRclk, RISING);
   Counter=0;
}

void ClockSstop(void)
{
   TableStopped = true;
   //detachInterrupt(digitalPinToInterrupt(DI2));
   detachInterrupt(digitalPinToInterrupt(SoftClockDIO));
   Counter=0;
}

inline void SetupNextEntry(void)
{
   static int   i,k,maxc;
   static bool  DIOchange;

//  ValueChange = true;
    DIOchange=false;
    // Set the address
SetupNextEntryAgain:
    // Set the next event counts. These are compare registers that will cause interrupts
    // and generate LDAC latch signal
    if(TEheader->Count >= 0) MPTtc.TC_RA = TEheader->Count;
    else MPTtc.TC_RA = -TEheader->Count + TimeDelta;
    if(Theader->MaxCount >=0) MPTtc.TC_RC = Theader->MaxCount;
    else MPTtc.TC_RC = -Theader->MaxCount + TimeDelta;    
//    MPTtc.TC_RA = TEheader->Count;
//    MPTtc.TC_RC = Theader->MaxCount;
SetupNextEntryAgain2:
    // Process the current entry, all channels
    int cb = SelectedBoard();
    while(1)
    {
        if(Theader->TableName == 0)
        {
            StopRequest = true;
            return;
        }
        for(i=0;i<TEheader->NumChans;i++)
        {
            // If chan is ']' then check loop counter and repeat table if
            // its not zero. If zero advance to next table that follows
            if(Tentry[i].Chan == ']')
            {
                // Advance the loop counter
                NS.Count[NS.Ptr-1]++;
                if((NS.Count[NS.Ptr-1] >= NS.Table[NS.Ptr-1]->RepeatCount) && (NS.Table[NS.Ptr-1]->RepeatCount != 0))  // If repeat count is zero loop forever
                {
                    // Advance to the next table if loop counter has expired
                    if(!AdvanceTablePointer())
                    {
                        // All done so stop the timer
                        StopRequest = true;
                        if(DIOchange) DOrefresh;   // Added Jan 15, 2015
                        return;
                    }
                    // TimeDelta = 0;
                    // Setup for the first event in the next table if there is a time 0
                    // event. This table's time zero event happens at the same time as the
                    // current tables top count.
                    if(TEheader->Count == 0)
                    {
                       // Update the DIO hardware if needed
                       if(DIOchange) DOrefresh;
                       goto SetupNextEntryAgain2;  // Changed from SetupNextEntryAgain on Nov 25, 2017
                       //SetupNextEntry();
                       //return;
                    }
                }
                else
                {
                    maxc = Theader->MaxCount;
                    // Replay the table on the top of the nesting stack.
                    Theader = NS.Table[NS.Ptr-1];
                    TEheader = (TableEntryHeader *)((char *)Theader + sizeof(TableHeader));
                    Tentry = (TableEntry *)((char *)TEheader + sizeof(TableEntryHeader));
                    TentryCount = 0;
                    if(maxc > Theader->MaxCount) MPTtc.TC_RC = maxc;  // added november 25, 2017
                    // Setup for the first event in the next table if there is a time 0
                    // event. This table's time zero event happens at the same time as the
                    // current tables top count.
                    if(TEheader->Count == 0)
                    {
                       // Update the DIO hardware if needed
                       if(DIOchange) DOrefresh;
                       goto SetupNextEntryAgain2;  // Changed from SetupNextEntryAgain on Nov 25, 2017
                       //SetupNextEntry();
                       //return;
                    }
                }
                if(DIOchange) DOrefresh;
                return;
            }
            // If Chan is 0 to 31 its a DC bias output so send to DAC
            // It take 3.5 uS to send one channel via SPI, the dead time between channels is 6uS,
            // The 6uS is the time around this inner channel loop
            if((Tentry[i].Chan >= 0) && (Tentry[i].Chan <= 31))
            {
              // See if the SPI address is correct, if not update
              if((pioA->PIO_PDSR & 7) != ((Chan2Brd[Tentry[i].Chan] >> 4) & 7))
              {
                   pioA->PIO_CODR = 7;
                   pioA->PIO_SODR = ((Chan2Brd[Tentry[i].Chan] >> 4) & 7);                
              }              
              DCbiasUpdaated = true;
              //int cb = SelectedBoard();                 // Added 9/3/17, takes 1uS, moved to top of loop, 3/4/21
              SelectBoard(Chan2Brd[Tentry[i].Chan] & 1);  // Takes 1uS
              k=Tentry[i].Value;
              SPI.transfer(SPI_CS, (uint8_t *)&k, 4);
              //SelectBoard(cb);                          // Added 9/3/17, takes 1uS, moved to end of loop, 3/4/21
            }
            else if((Tentry[i].Chan & RAMP)!=0) TABLEqueue(ProcessRamp,&Tentry[i]);
            else if((Tentry[i].Chan >= 101) && (Tentry[i].Chan <= 108))  // 65 hex to 6C hex
            {
              // Here if ARB commands
              if(Tentry[i].Chan == 101)      UpdateAux(0, *(float *)&Tentry[i].Value, false);     // 101, e
              else if(Tentry[i].Chan == 102) UpdateAux(1, *(float *)&Tentry[i].Value, false);     // 102, f
              else if(Tentry[i].Chan == 103) UpdateAux(2, *(float *)&Tentry[i].Value, false);     // 103, g
              else if(Tentry[i].Chan == 104) UpdateAux(3, *(float *)&Tentry[i].Value, false);     // 104, h
              else if(Tentry[i].Chan == 105) UpdateOffsetA(0, *(float *)&Tentry[i].Value, false); // 105, i
              else if(Tentry[i].Chan == 106) UpdateOffsetB(0, *(float *)&Tentry[i].Value, false); // 106, j
              else if(Tentry[i].Chan == 107) UpdateOffsetA(1, *(float *)&Tentry[i].Value, false); // 107, k
              else if(Tentry[i].Chan == 108) UpdateOffsetB(1, *(float *)&Tentry[i].Value, false); // 108, l
              TABLEqueue(ProcessARB,true);
            }
            // if Chan is A through P its a DIO to process
            else if((Tentry[i].Chan >= 'A') && (Tentry[i].Chan <= 'P'))
            {
               DIOchange = true;
               SDIO_Set_Image(Tentry[i].Chan,Tentry[i].Value);
            }
            // if Chan is d then the time count delta is adjusted
            else if(Tentry[i].Chan == 'd')
            {
               TimeDelta += Tentry[i].Value;
               //MPTtc.TC_RA = TEheader->Count + TimeDelta;
            }
            // if Chan is p then the time count delta is set
            else if(Tentry[i].Chan == 'p')
            {
               TimeDelta = Tentry[i].Value;
               //MPTtc.TC_RC = Theader->MaxCount + TimeDeltaMax;
            }
            // if Chan is r then this triggers the ARB compress line, value defines state
            else if(Tentry[i].Chan == 'r') {Cstate = Tentry[i].Value;                 TABLEqueue(ProcessCompress,true);}            
            // if Chan is s then this triggers the ARB sync line, value defines state
            else if(Tentry[i].Chan == 's') {Sstate = Tentry[i].Value;                 TABLEqueue(ProcessSync,true);}                        
            // if Chan is t then this is a trigger out pulse so queue it up for next ISR
            else if(Tentry[i].Chan == 't') {QueueTriggerOut(Tentry[i].Value);         TABLEqueue(ProcessTriggerOut,true);}
            // if Chan is b then this is a trigger burst pulse so queue it up for next ISR
            else if(Tentry[i].Chan == 'b') {QueueBurst(Tentry[i].Value);              TABLEqueue(ProcessBurst,true);}
            // if Chan is c then this is a trigger for the compression table, value = A for ARB, T for Twave
            else if(Tentry[i].Chan == 'c') {QueueCompressionTrigger(Tentry[i].Value); TABLEqueue(ProcessCompressionTrigger,true);}
        }
        break;
    }
    SelectBoard(cb);
    // Update the DIO hardware if needed
    if(DIOchange) DOrefresh;
    TentryCount++;
    // Advance to next entry
    if(TentryCount < Theader->NumEntries) AdvanceEntryPointer();
    else
    {
        if(!AdvanceTablePointer())
        {
            // All done so stop the timer
            StopRequest = true;
//            return;
        }
        else
        {
          // If this table has a time 0 event then setup for it, nov 3, 2016
          if(TEheader->Count == 0) goto SetupNextEntryAgain2;
        }
    }
} 

//**************************************************************************************************
//
// This section of the file contains all interrupt processing routines, call back functions.
//
//**************************************************************************************************

void Dummy_ISR(void)
{
}

void Trigger_ISR(void)
{
   if((TriggerMode == EDGE)||(TriggerMode == POS)||(TriggerMode == NEG)) MPT.softwareTrigger();  // Need to make sure its running!
   uint32_t csb = pio->PIO_ODSR & pin;
   if(MPT.getRAcounter() == 0)
   {
     ProcessTableQueue();
     if(DCbiasUpdaated) { ValueChange = true; DCbiasUpdaated = false; }
     if((MIPSconfigData.Rev <= 1) || (softLDAC))// Rev 1 used software control of LDAC
     {
       LDAClow;
       LDAChigh;
     }
     SetupNextEntry();
   }
   if(ClockMode == EXTS) StartTimer();
   if(csb == 0)  pio->PIO_CODR = pin;
   else pio->PIO_SODR = pin; 
   // If retigger option is false then turn off external interrupts
   if(!MIPSconfigData.TableRetrig)
   {
     MPT.setTrigger(TC_CMR_EEVTEDG_NONE);
     DIhTrig.detach();
   }
}

// This interrupt happens when the compare register matches the timer value. At this time the
// data is latched and the next values are written.
//
// Note; This interrupt does not happen when the compare register A is 0. The hardware pin 
// toggels and then LDAC is generated but the ISR does not fire. Strange.
inline void RAmatch_Handler(void)
{
  uint32_t csb = pio->PIO_ODSR & pin;
  if(DCbiasUpdaated) { ValueChange = true; DCbiasUpdaated = false; }
  ProcessTableQueue();  // This call takes 3uS even if it does nothing
  if((MIPSconfigData.Rev <= 1) || (softLDAC)) // Rev 1 used software control of LDAC
  {
    LDAClow;
    LDAChigh;
  }
  SetupNextEntry();
  if(csb == 0)  pio->PIO_CODR = pin;
  else pio->PIO_SODR = pin; 
}

// This interrupt happens when the counter reaches max count saved in RC. The function updates
// max count.
inline void RCmatch_Handler(void)
{
   uint32_t csb = pio->PIO_ODSR & pin;  // Save board select and restore on exit
   if(StopRequest == true)
   {
      // Here when the table has finished
       AtomicBlock< Atomic_RestoreState > a_Block;
       StopTimer();
       StopRequest = false;
       // If tabel is in external trigger mode then setup for a new
       // trigger here.
       if((TriggerMode == EDGE)||(TriggerMode == POS)||(TriggerMode == NEG)) // Not sure about this code / idea??
       {
          AdvanceTableNumber();
          SetupTimer();
          TableStopped = true;
       }
       return;
   }
   if(RampEnabled) RampClock->softwareTrigger();
   if(DCbiasUpdaated) { ValueChange = true; DCbiasUpdaated = false; }
   // If compare register A is zero then at this point call setup next,
   // also if rev 1 pulse LDAC as well
   if(MPT.getRAcounter() == 0)
   {
     ProcessTableQueue();
     if((MIPSconfigData.Rev <= 1) || (softLDAC)) // Rev 1 used software control of LDAC
     {
       LDAClow;
       LDAChigh;
     }
     SetupNextEntry();
     if(csb == 0)  pio->PIO_CODR = pin;
     else pio->PIO_SODR = pin; 
   }
}

//**************************************************************************************************
//
// Ramp functions, added April 16, 2019
//
// Ramping is implemended using a timer interrupt to update the DCbias outputs channels that are
// generating ramps. The table contains flags bits in the channel number definition to indicate
// channels that are generating ramps. Two flags are used:
// RAMP to indicate this channel is ramping and the channel value contains the voltage step incremented
//      at each clock cycle.
// RAMP | INITIAL sets the initial or starting value for a ramp. This must be done before a ramp 
//                starts and it can be done at the same time point as the RAMP command.
// The ramp capability must first be enabled and the clock rate set before a table with ramp commands
// is executed. Below is a series of commands that will generate a simple ramp:
// 
// STBLRMPENA,TRUE,1000
// STBLDAT;0:[A:100,100:1:10:194:1:130:4,5000:1:15:130:-1.0,10000:130:0,13000:1:0,20000:];
// SMOD,ONCE
// TBLSTRT
//
//**************************************************************************************************

// This ISR needs to select the board and set the SPI address.
void RampISR()
{
   volatile int i,sum=0,k;
   volatile uint8_t *s,*d;

   uint32_t csb = pio->PIO_ODSR & pin;  // Save board select and restore on exit
   Ramping = false;
   for(int i=0;i<NUM_RAMPS;i++) if((RE[i].chan & RAMP_DONE) == 0)
   {
      if(RE[i].delta == 0) RE[i].chan |= RAMP_DONE;
      else
      {
        s = (uint8_t *)&RE[i].initial;
        d = (uint8_t *)&sum;
        d[0] = s[3];
        d[1] = s[2];
        d[2] = s[1] & 0x0F;
        sum += RE[i].delta;
        s[3] = d[0];
        s[2] = d[1];
        s[1] = (s[1] & 0xF0) | (d[2] & 0x0F);
        s[0] = 3;
        // send to DAC output register, no LDAC
        AtomicBlock< Atomic_RestoreState > a_Block;
        k = RE[i].initial;
        if((pioA->PIO_PDSR & 7) != ((Chan2Brd[RE[i].chan] >> 4) & 7))
        {
           pioA->PIO_CODR = 7;
           pioA->PIO_SODR = ((Chan2Brd[RE[i].chan] >> 4) & 7);                
        }              
        SelectBoard(Chan2Brd[RE[i].chan & ~RAMP_DONE] & 1);
        SPI.transfer(SPI_CS, (uint8_t *)&k, 4);
        Ramping = true;
      }
   }  
   if(csb == 0)  pio->PIO_CODR = pin;
   else pio->PIO_SODR = pin; 
}

// This function is called by the command processor and enables the ramp capability.
// The freq parameter defines the update frequency for the ramp clock.
// Timer channel defined by TMR_RampClock
void EnableRamp(char *ena, char *freq)
{
  String token;
  float  Freq;

  token = ena;
  if(token == "TRUE") RampEnabled = true;
  else if(token == "FALSE") RampEnabled = false;
  else
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;    
  }
  SendACK;
  token = freq;
  Freq = token.toFloat();
  if(RampClock == NULL) RampClock = new MIPStimer(TMR_RampClock);
  RampClock->attachInterrupt(RampISR);
  RampClock->setFrequency((double)Freq);
}

// Called at table setup time to enable the ramp clock
void StartRampClock(void)
{
  if(!RampEnabled) return;
  for(int i=0;i<NUM_RAMPS;i++) RE[i].chan = 0xFF;
  RampClock->start(-1, 0, false);  
  RampClock->softwareTrigger();
}

// Called when table completes to stop the clock
void StopRampClock(void)
{
  if(!RampEnabled) return;
  RampClock->stop();
}

// This function is called from SetupNextEntry and its only called if
// the RAMP bit is set in the channel variable
void ProcessRamp(volatile TableEntry *TE)
{
  int i;

  if(!RampEnabled) return;
  if((TE->Chan & INITIAL) != 0)
  {
     for(i=0;i<NUM_RAMPS;i++)
     {
        if((RE[i].chan==0xFF) || ((RE[i].chan & (~RAMP_DONE)) == (TE->Chan & ~(RAMP | INITIAL))))
        {
           RE[i].chan    = TE->Chan & ~(RAMP | INITIAL);
           RE[i].initial = TE->Value;
           break;
        }
     }
  }
  else
  {
     for(i=0;i<NUM_RAMPS;i++) if((RE[i].chan & (~RAMP_DONE)) == (TE->Chan & ~(RAMP | INITIAL)))
     {
        RE[i].chan &= ~RAMP_DONE;
        RE[i].delta = TE->Value;
        break;
     }
  }
}

// The following functions support time table count parameter adjustment via ADC change value detection.
// Three main functions support this capability.
// - Selection of the time point for adjustment. This requires the initial count and a channel at the initial count.
// - Table trigger on ADC change detection, gain and offset for ADC value.
// - Time parameter adjustment range.

TableEntryHeader *TEHopen  = NULL;
TableEntryHeader *TEHclose = NULL;
int GateWidth              = 0;
int MinAdjTime             = 100;
int MaxAdjTime             = 10000;
float ADCttGain            = 1;
float ADCttOff             = 0;
float mzList[5]            = {0,0,0,0,0};
int   countList[5];

// This function finds the gate open and if possible the gate close table headers in the current table.
// If the close is found then the gate width is calculated and will be adjusted to keep the width constant.
void SelectTPforAdjust(int count, int chan)
{
    TableEntry *TE;
    int i;

    TEHopen = TEHclose = NULL;
    TEHopen = FindTableEntryHeader(count, chan);
    if(TEHopen==NULL)
    {
        SetErrorCode(ERR_CANTFINDENTRY);
        SendNAK;
        return;
    }
    // The next event should hold the close time event
    TEHclose = (TableEntryHeader *)((char *)TEHopen + (sizeof(TableEntryHeader) + (sizeof(TableEntry) * TEHopen->NumChans)));
    TE = (TableEntry *)((char *)TEHclose + (sizeof(TableEntryHeader)));
    // If chan is in this event then its the close event and we can use its time to calculate the gate width
    SendACK;  
    for(i=0;i<TEHclose->NumChans;i++) if(TE[i].Chan == (chan-1)) 
    {
      GateWidth = TEHclose->Count - count;
      return;
    }
    GateWidth = 0;
    TEHclose = NULL;
}

// Sets the minimum and maximum allowable tabe times. These times are in clock cycles
void DefineAdjustRange(int mint, int maxt)
{
  // Range test and adjust
  if(mint < 0) mint = 0;
  if(maxt < 0) maxt = 0;
  if(mint >= (maxt-100)) maxt = mint + 100;
  // Apply values
  MinAdjTime = mint;
  MaxAdjTime = maxt;
  SendACK;
}

// Find the closest m/z in the list, must be at least within mzerror.
// Returns the index if fould else -1
int FindCloseMZ(float mz)
{
  float mzerror = 50;
  int   mzindex = -1;
  int   i;
  
  for(i=0;i<5;i++)
  {
    if(abs(mz-mzList[i]) < mzerror)
    {
      mzerror = abs(mz-mzList[i]);
      mzindex = i;
    }
  }
  return(mzindex);
}

void ADCtableTriggerISR(int adcVal)
{
  if(TableMode != TBL) return;
  if(!TableReady) return;
  if((!MIPSconfigData.TableRetrig) && (TimerRunning == true)) return;
  if(TEHopen != NULL) 
  {
    float mz = (float)adcVal*ADCttGain + ADCttOff;
    int i = FindCloseMZ(mz);
    if(i >= 0)
    {
      int j = countList[i] - GateWidth/2;
      if(j<MinAdjTime) j = MinAdjTime;
      if(j>MaxAdjTime) j = MaxAdjTime;
      TEHopen->Count = j;
      if(TEHclose != NULL)
      {
         j = countList[i] + GateWidth/2;
         if(j<MinAdjTime) j = MinAdjTime;
         if(j>MaxAdjTime) j = MaxAdjTime;
         TEHclose->Count = j;        
      }
    }
  }
  IssueSoftwareTableStart = true;
 }

// This function allows the user to adjust the ADC value using the following equation:
// value = ADC * gain + offset
void SetADCtoMZcal(char *gain, char *offset)
{
  String sToken;

  sToken = gain;
  ADCttGain = sToken.toFloat();
  sToken = offset;
  ADCttOff = sToken.toFloat();
  SendACK;
}

// This function will set a m/z and count value in the list of targets.
// Three parameters, index,m/z,count.
// Maximum of 5 entries in the list
// The user is expected to update all 5 poosible entries.
// This function is called with the parameters in the ring buffer
void DefineMZtarget(void)
{
   char   *Token;
   String sToken;
   int    ch,count;
   float  mz;

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
     mz = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     count = sToken.toInt();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the channel and exit if error
     if((ch < 0) || (ch > 4)) break;
     mzList[ch] = mz;
     countList[ch] = count;
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This function sets the ADC change interrupt to call back to the
// Table trigger function
void TableTrigOnADC(void)
{
  ADCattachInterrupt(ADCtableTriggerISR);
  SendACK;
}

#endif
