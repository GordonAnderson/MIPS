//
//  Table.cpp
//  Created by Gordon Anderson on 9/4/14.
//
// Timer8 is used to enable output voltage sequence control.
// The MIPStimer class is used to support the needed timer 
// functions.
// The timer clock can be internal or external on the TCLK0 pin, D22.
// TIOA8 (D11) is used to latch the DAC outputs using LDAC.
//
// Four internal clock are avalible:
//   MCK/2
//   MCK/8
//   MCK/32
//   MCK/128
// MCK = 84 MMz
//
// Table syntax:
//
// STBLDAT;0:[1:20,0:1:10:2:30,50:3:70,100:[2:40,0:1:30:2:10,100:1:10:2:30,200:W],
// 0:1:0,0:2:0,200:A:1,150:A:0,400:W];
//
// The table command is parsed and data is saved in a memory block as follows:
//
// The table uses counter timer 8 in the atmel ARM cpu. The counter is in waveform (gaa stoped editing)
// mode. In this mode the counter has a max count saved in the ICR register,
// at this count the counter resets. This causes a counter overflow interrupt
// where the counter max count is reloaded with the current table max count
// value and some house keeping is done. Additionally the output compare register
// is used to cause an interrupt and to generate LDAC.
//
// The process is started by setting the counter to 0xFFFF, its max value and
// then defining the first time point values. When triggered the counter starts,
// at every interrupt the next values are defined and the current time point
// values latched by LDAC.
//
// The table command is parsed and saved in a formatted block of memory in
// a format needed for fast processing to enable fast execution.
//
// A [ defines the start of a table entry defined below. A table command
// does not need to have nesting so in this case the table name is null
// and the repeat count is 1. When a [ is encountered in the input command
// the current table is halted and a new table started. At run time when
// a table is started a first in last out stack entry is made with the
// table location and repeat count.
// Table:
//    Table name (byte), defined after [ or at start of table command
//    Repeat count (word)
//    NumEntries (word), this is calculated when the command is parsed.
//    MaxCount (word), This is caluclated when the command is parsed.
//    Entry format:
//      Count (word), timer count when this point is defined
//      NumChans (byte), this defines the number of output channels that will
//                       be updated at this count. The number of channels includes
//                       W and ], end of table flag
//      ChanValue
//          Chan (byte), 0-15 if DC output, alpha character if DIO, or W or ]
//          Value (word)
//      If Chan is ']' then this is the end of the nested table. The table count
//      is tested and if not zero its deceremented and the table repeated. If zero
//      the last stack value is removed and the next entry processed. This ']' marks
//      the end of a table but a table can end without a ']'. If a table ends without
//      a ']' then its the end of all tables. If a table ends with ']' and none follow
//      then the next table name is defined as 0xFF to flag the end of all tables.
// Note:
//      Whatever the time point is that contains a ] any output values will likely
//      not be defined because the ] represents the end of the table and the next one
//      is being defined or all tables end.
//
//
// If you want the entire table to repeat you make it a loop (like above) and then embed
// other loops inside it if needed.  Time is still absolute, but it resets to zero at the
// beginning of the Table and at every loop bracket (open or close).
//
// So with the above STBLDAT command, when triggered, the Table immediately sets DCB1 to
// 10V and DCB2 to 30V.  50 clock cycles later it sets DCB3 to 70V.  50 clock cycles later
// it starts a loop and runs it 40 times where DCB1 and DCB2 swap values every 100 clock cycles.
// When this is complete (8000 clock cycles later) it immediately sets DCB1 and DCB2 to 0V,
// waits 200 clocks and sets DIO_A high for 10 clocks and then low, then it waits 190 clocks
// before starting over and it repeats 20 times.
//
// Anyone writing a Falkor version of the Table needs to understand that time is absolute but
// resets every time a loop is started or ended.  The above command line would be sourced thusly*:
//
// Time    Type    Chan    Value
// 0       LOOP    1       20  (beginning of Loop 1 that will repeat 20 times)
// 0       DCB     1       10
// 0       DCB     2       30
// 50      DCB     3       70
// 100     LOOP    2       40  (beginning of Loop 2 that repeats 40 times)
// 0       DCB     1       30
// 0       DCB     2       10
// 100     DCB     1       10
// 100     DCB     2       30
// 200     GOTO    2           (end of Loop 2 that points back to Loop 2 beginning)
// 0       DCB     1       0
// 0       DCB     2       0
// 200     DIO     A       1
// 210     DIO     A       0
// 400     GOTO    1           (end of Loop 1 that points back to Loop 1 beginning)
//
// *Time in this Table is represented in units of microseconds, milliseconds, etc., Falkor
// will to translate to actual clock counts before sending the Table to AMPS.
//
// As before, any steps with the same Time value are a single event in time and the next
// time listed is absolute from the beginning/ending of the Table (or last LOOP bracket).
// Please let us know what limit you’d like to place on nested loops.  Hopefully we can at
// least do 3-5 for now.
//
// For now, please continue to issue a TBLCMPLT string when the entire Table has completed
// execution.  We’d like to perhaps add other status strings issued when internal Loops
// complete.  We can talk about this more as time goes on.
//
// I will send another email with other details and a sequence of operations for Table
// execution.
//
//Here are the changes we discussed with some others so please read through:
//
// 1. We will stay with a single Table for now with changes outlined in the previous email.
// 2. The STBLRPT command is gone and all repetition data will be embedded in the Table
//    in the form of Loops.
// 3. The STBLCLK command only sets the clock source for Table execution and can be issued
//    at any time, even while running a Table.
// 4. We will add a third modifier to the STBLTRG command: BOTH.  In BOTH mode, the Table
//    will start with either TBLSTRT or INT7 events.  In SW mode, the INT7 input will be
//    ignored.  In EXT mode, the TBLSTRT command will be ignored.  The STBLTRG command can
//    be issued at any time, even while running a Table.
// 5. We will put the AMPS box into Table Mode with the SMOD,TBL command.  This causes the
//    AMPS box to:
//      a. Send initial Table data to the DACs/DIOs.
//      b. Set LDAC high.
//      c. Disable the HMI and display “Executing Table” on the LCD.
//      d. Issue a “TBLRDY” string.
// 6. While in Table mode:
//      a. Only Table commands and a front panel 2-button push (LEFT/RIGHT) will be
//         recognized and acted upon.
//      b. If/when the Table completes execution, AMPS issues “TBLCMPLT” and repeats
//         the steps in #5 above in preparation for a re-trigger event.
//      c. If/when the new command TBLSTOP is received, AMPS stops Table Execution and
//         repeats the steps in #5 above in preparation for a re-trigger event.
// 7. Only a SMOD,LOC or TBLABRT command, or a front panel 2-button push will take the
//    AMPS box out of Table Mode.
// 8. Issuing the SMOD,LOC command will stop Table execution (if running) and take the
//    AMPS box out of Table mode:
//      a. Front panel buttons and LCD  (HMI) become operational and all output values
//         restored to pre-Table Mode.
//      b. LDAC is set low.
// 9. The TBLABRT command and the 2-button push will have the same effect as SMOD,LOC
//    except that an “ABORTED” string is issued.
// 10. “Remote Mode” goes away.  If precise Table timing/synchronization is required the
//     User must use hardware triggering.
// 11. Table execution can only be triggered while the AMPS box is in Table Mode and is
//     not currently running a Table.
//
// Here’s the normal sequence of events:
//
// Commands:                           AMPS Response:
//      - STBLDAT                             <ack>
//      - STBLCLK                             <ack>
//      - STBLTRG                             <ack>
//      - SMOD,TBL                            TBLRDY (HMI/LCD displays “Executing Table”)
//      - TBLSTRT or INT7 starts Table        (do we need feedback when the Table is
//                                            triggered?  If so, perhaps issue a “TRIGGERED” string?)
//      - (if table completes execution)      TBLCMPLT then TBLRDY
//      - (if TBLSTOP stops execution)        TBLRDY
//      - SMOD,LOC                            (HMI/LCD returns to normal operation and
//                                            output values to what they were when SMOD,TBL
//                                            was received)
//
// If, while in Table Mode, TBLABRT command is received or User pushes the 2-buttons on
// the front panel, the AMPS box responds with “ABORTED” and the HMI/LCD returns to normal operation.
//
// Let me know if there’s anything I missed or if something needs tweaking.  I know Spencer
// wanted to do some tricks with <ack>/<nack> with certain commands if/when a Table was running,
// but I don’t remember what or why…
//
// Command string tested examples
//
// STBLDAT;100:1:10;
// STBLDAT;100:1:10,150:2:33;
// STBLDAT;200:1:1,300:1:0,400:1:1,500:1:0;
// STBLDAT;200:A:1,300:A:0,400:A:1,500:A:0;
// STBLDAT;0:[A:1000,200:A:1,300:A:0,400:A:1,500:A:0:];
// STBLDAT;200:A:1,300:A:0,400:A:1,700:A:0;
// STBLDAT;100:A:1,150:A:0,200:A:1,250:A:0;
// STBLDAT;0:A:1,150:A:0,200:A:1,250:A:0;
// STBLDAT;0:[3:3,100:A:1,150:A:0,225:A:1,250:A:0,301:];
// STBLDAT;100:1:10:2:20:3:123,150:2:33,250:2:0;
// STBLDAT;100:1:15:3:123,150:2:33,400:[3:5,0:1:30:2:10,100:1:10:2:30,150:];
// STBLDAT;100:1:15:3:123,150:2:33,400:[3:5,0:1:30:2:10,100:1:10:2:30,150:],0:1:15,50:1:5;
// STBLDAT;0:[3:5,0:1:30:2:10,100:1:10:2:30,150:];
// STBLDAT;0:[A:1000,0:1:0,1000:1:5,1000:];
// STBLDAT;0:[A:1000,0:A:1,1000:A:0,1000:];
// STBLDAT;0:[A:1000,100:A:1,1000:A:0,1000:];
//
// Examples to be tested
//
// STBLDAT;100:1:10:3:123,150:2:33,200:[3:40,0:1:30:2:10,100:1:10:2:30,150:W];
// STBLDAT;100:1:10:3:123,150:2:33,200:[3:40,0:1:30:2:10,100:1:10:2:30,150:W],100:1:10:3:123;
// STBLDAT;0:[1:20,0:1:10:2:30,50:3:70,100:[2:40,0:1:30:2:10,100:1:10:2:30,200:W],0:1:0:2:0,150:A:1,200:A:0,400:W];
//
// STBLDAT;0:[1:50,0:1:2:2:0:3:0,328:1:0:2:1:3:2,12797:1:1:2:1:3:2,13125:];
// STBLDAT;0:[1:3,0:A:1:B:0:C:0,328:A:0:B:1:C:1,500:A:1,600:A:0,12797:A:0:B:1:C:1,13125:];
//
// SMOD,TBL
// SMOD,LOC
// TBLSTRT
// STBLCLK,EXT
// STBLCLK,MCK2
// STBLCLK,MCK8
// STBLCLK,MCK32
// STBLCLK,MCK128
// GTBLFRQ
// STBLTRG,POS
//
// Table setup for Jim's system
// STBLTRG,NEG
// STBLCLK,10500000
// STBLDAT;136500:1:6,189000:1:0,305500:1:-9,337000:1:3,589500:1:2.5,594500:1:2,5000000:1:2;
// STBLDAT;0:[1:1000,10000:1:25:A:1,30000:1:5:A:0,30100:1:5:A:0];
// SMOD,TBL
//
// Timing tests
// STBLDAT;0:[1:100000,100:1:25,1000:1:5,2000:];
// STBLDAT;0:[1:100000,100:1:25,107:1:10,1000:1:5,2000:];
// STBLDAT;0:[1:100000,100:A:1,107:A:0,1000:A:1,2000:];
// STBLDAT;0:[1:100000,0:1:25:2:30,107:1:10,2000:1:5,3000:];
// In FAST_TABLE mode:
//  June 29, version
//  6 is min count for 1 value change, 9.14uS
//  10 is min count for 2 value change, 15.2uS
//  Last event 21 min, 32uS
//  
//
// STBLDAT;0:[1:100000,100:1:25,106:1:10,1000:1:5,2000:];
//
// Testing pulse sequences
// STBLDAT;0:[1:10,1000:1:25:A:1,3000:1:5:A:0];
// STBLDAT;0:[1:5,1000:9:25:A:1,3000:9:5:A:0];
// STBLDAT;0:[1:4,1000:1:25:9:20:A:1,3000:1:5:9:6:A:0];
// 
// Spencer's is having trouble with:
// STBLDAT;0:[1:100,39062:1:25:A:1,46875:1:5:A:0];
// STBLDAT;0:[1:100,39062:1:100:A:1,46875:1:15:A:0];
// STBLDAT;0:[1:100,10000:1:100:A:1,20000:1:15:A:0,30000:1:75:A:0];
// STBLDAT;0:[1:100,39062:1:100:2:10:3:10:A:1,46875:1:15:2:0:3:0:A:0];
// STBLDAT;0:[1:100,39062:1:100:2:10:3:10:A:1,46875:1:15:2:0:3:0:A:0,50000:W];
// STBLDAT;0:[1:100,39062:1:100,46875:1:15];
// STBLDAT;0:[1:100,39062:A:1,46875:A:0];
// STBLDAT;0:[1:100,39062:1:10,46875:1:0,46876:];
// STBLDAT;0:[1:100,300:1:10,400:1:0];
// STBLDAT;0:[1:100,300:1:10,400:1:0,401:];
//
// STBLDAT;0:[1:1000,0:A:0,5:A:1,10:A:0];
// STBLDAT;0:[1:1000,0:A:0,500:A:1,10000:A:0];
//
// Issues/bugs
//  1.) Change the table memory management to dynamically allocate the space needed, do not allocate a fixed 
//      block of memory.
//  2.) If this was done with a linked list to the entry blocks then we could insert updates and support multiple
//      tables commands that build the same table. Consider this change. Use a single link list and use malloc and free.
//      The c++ new features do not support realloc.
//  3.) Add the ramp capability.
//  4.) Fix the zero count problem. This requires significant updates to the code as outlined below:
//      a.) Execute code on trigger, both software or external. This code will call SetupNext but only
//          when a 0 point update is needed
//      b.) Setup the output A toggle to happen on trigger as well. This will generate LDAC on rev 2 controller but not rev 1.
//          For rev 1 pulse LDAC on the routine called in set a.
//      c.) When looping to a zero point we need to call setup in the RC terminal count ISR
//      d.) Also need to setup RC match to toggle output A for rev 2 controller and rev 1 needs to pulse LDAC in RC match ISR
//
// Proposed updates to the table IO mode.
//  1.) call it pulse sequence generation.
//  2.) create a dialog for setup with the following features:
//      a.) Load a macro or a pulse sequence
//      b.) Allow setting clock mode
//      c.) Allow setting trigger mode
//      d.) Enable pulse sequence generation
//  3.) Have a second dialog that is displayed when we are in the table mode this
//      dialog will:
//      a.) Show table status, running or ready for trigger
//      b.) Allow abort
//      c.) Allow software trigger
//      This could be independent of the first setup dialog. Replace the current popup message.
//
//
// attachInterrupt(digitalPinToInterrupt(DI2), ISRclk, RISING);
//
#include "Arduino.h"
#include "variant.h"
#include <stdio.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include "SPI.h"
#include "table.h"
#include "serial.h"
#include "errors.h"
#include "Menu.h"
#include "Hardware.h"
#include "DIO.h"
#include "DCbias.h"
#include "Dialog.h"
#include "Variants.h"
#include <Thread.h>
#include <ThreadController.h>
#include <MIPStimer.h>
#include <DIhandler.h>
#include "AtomicBlock.h"

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

//#define FAST_TABLE 1

#pragma GCC optimize "-O3"

extern ThreadController control;

MIPStimer MPT(TMR_Table);               // timer used for pulse sequence generation
#define   MPTtc   TC2->TC_CHANNEL[2]    // Pointer to the timers control block

#define NumTables 5
#define MaxTable  4096
unsigned int Counter;

volatile unsigned char VoltageTable[NumTables][MaxTable];
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

uint8_t  Chan2Brd[32];

bool ValueChange = false;

bool TasksEnabled = false;            // Setting this flag to tue will enable all tasks in table mode
bool TableTriggered = false;          // This flag is set when the table is triggered and reset when its complete

int InterTableDelay = 3;

DIhandler DIhTrig;

enum TriggerModes TriggerMode = SW;
enum ClockModes   ClockMode   = MCK128;
enum TableModes   TableMode   = LOC;

//**************************************************************************************************
//
// This section of the file contains all the serial IO processing routines.
//
//**************************************************************************************************

// Low level routines to get token from the string.

// This function will wait for the next token or a timeout. This function blocks
// and will return NULL on a timeout.
char *NextToken(void)
{
    char *tk;
    unsigned int timeout;
    
    timeout = millis();
    while(millis() < (timeout + 3000))
    {
        WDT_Restart(WDT);
        if((tk=GetToken(true))!=NULL) 
        {
          return(tk);
        }
        // Put serial received characters in the input ring buffer.
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

// This command defines the current / active table number the system will use for all processing
void SetTableNumber(int tblnum)
{
  if((tblnum >= 1) && (tblnum <= NumTables))
  {
    CT = --tblnum;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
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
    if(strcmp(cmd,"SW") == 0) TriggerMode = SW;
    else if(strcmp(cmd,"EDGE") == 0) TriggerMode = EDGE;
    else if(strcmp(cmd,"POS") == 0) TriggerMode = POS;
    else if(strcmp(cmd,"NEG") == 0) TriggerMode = NEG;
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
    // If there is a update at count 0 then fire LDAC and setup for the next event
    if(MPT.getRAcounter() == 0)  // This can't work because the timer has not yet been setup, maybe it has??
    {
      if((MIPSconfigData.Rev <= 1) || (softLDAC)) // Rev 1 used software control of LDAC
      {
        LDAClow;
        LDAChigh;
      }
      else   // here for rev 2 
      {
        // Not sure why this was done and I removed it on Nov 3, 2016.
        // This was part of a series of bug fixes for external triggering. 
 //       LDACcapture;    
 //       LDACctrlLow;
 //       LDACctrlHigh;
 //       LDACrelease;
      }
//    SetupNextEntry();  // Nov 3, 2016
    }
    StartTimer();
    SendACK;
}

// Report table current frequency setting
void TableFreq(void)
{
  int   freq;
  
  if(ClockMode == MCK2) freq = VARIANT_MCK/2;
  if(ClockMode == MCK8) freq = VARIANT_MCK/8;
  if(ClockMode == MCK32) freq = VARIANT_MCK/32;
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
    int val = TE->Value;
    val >>= 4;
    val &= 0xFFFF;
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
    buf[1] = (TE->Chan << 4) | (val >> 12);
    buf[2] = (val >> 4);
    buf[3] = (val << 4);
    SendACK;
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
    ptr          = 0;    // Memory block pointer
    TablesLoaded[CT] = 0;    // Number of tables loaded
    TestNesting = 0;     // Clear this error counter
    iStat = 0;
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
//          if(iStat == 0) InitialOffset = i;   // This was the failed solution before the Nov 3, 2016 edits
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
                // Out of space!
                iStat = PEerror;
                SetErrorCode(ERR_TBLTOOBIG);
                break;
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
                // Mark the next table tame with a 0 to flag the end
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
// current entry count and the first string token power already loaded with the token.
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
        //   - A DCB channel number 1 through 16
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
            if(TestNesting <0)
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
        else if(((TK[0] >= 'A') && (TK[0] <= 'P')) || (TK[0] == 't') || (TK[0] == 'b'))
        {
            // DIO channel number
            TE->Chan = TK[0];
            if(!ExpectColon()) break;
            if((TE->Chan == 't') || (TE->Chan == 'b')) Token2int(&TE->Value);
            else
            {
               if((TK = NextToken()) == NULL) break;
               TE->Value = TK[0];
               if(TK[0] == 't') Token2int(&TE->Value);  // This line makes no sense
            }
        }
        else
        {
            // DC bias channel number
            sscanf(TK,"%d",&i);
            TE->Chan = i-1;
            if(!ExpectColon()) break;
            if(!Token2float(&fval)) break;
            // Convert value to DAC counts
            TE->Value = DCbiasValue2Counts(TE->Chan, fval);
            // Make value into bit image the DAC wants to see. This will allow fast DAC updating
            // in the table execute mode. June 23 2016
            uint8_t  *buf = (uint8_t *)&TE->Value;
            int val = TE->Value;
            buf[0] = 0;         // DAC command
            buf[1] = ((TE->Chan & 7) << 4) | (val >> 12);
            buf[2] = (val >> 4);
            buf[3] = (val << 4);

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

//
// This function places the system in table mode and sets up the timer to play a table.
// TableModeEnabled is written to the display and all button processing is suspended.
//
void ProcessTables(void)
{
    uint32_t TimerStatus;
    int  InitialTableNum;
   
    InitialTableNum = CT;
    // Set mode to TBL
    TableMode = TBL;
    // Takeover the display and print message that we are in the table
    // mode. Allow pressing the button to cause an abort
    DisplayMessage("Table Mode Enabled");
    StopCommanded = false;
    // Setup the table mode and start timer
    while(1)
    {
        WDT_Restart(WDT);
        // Setup the timer
//        SetupTimer();
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
            // Restart table if reqested by user
            if(StopCommanded)
            {
                if((!SerialMute) && (TableResponse)) serial->println("Table stoped by user");
                break;
            }
            TimerStatus = MPT.getStatus();
            // If the timer has been triggered update the displayed status
            if(((TimerStatus & TC_SR_ETRGS)!=0) || SWtriggered)
            {
              TableTriggered = true;
              SWtriggered = false;
              // Here when triggered
               if((!SerialMute) && (TableResponse)) serial->println("TBLTRIG\n");
               if(StopRequest == true) break;
            }
            // Exit this loop when the timer is stoped.
            if(((TimerStatus & TC_SR_CLKSTA)==0) || TableStopped)
            {
                // Issue the table complete message
                TableTriggered = false;
                if((!SerialMute) && (TableResponse)) serial->println("TBLCMPLT\n");
                // Exit the loop and setup for another trigger
                break;
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
            TRACE(7);
            ProcessSerial();
            serial->flush();
            // If full command processing in table mode is enabled then run tasks.
            if(TasksEnabled)
            {
               delayMicroseconds(100);
               control.run();
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
        if(TableOnce) break;
        StopTimer();  // not sure about this, testing
        // Advance to next table if advance mode is enabled
        AdvanceTableNumber();
        if(StopCommanded) StopCommanded = false;
        TableTriggered = false;
    }
    // Clean up and exit
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
    int i;
    if(DCbDarray[0] != NULL)
    {
       for(i=0;i<8;i++) Chan2Brd[i] = 0; 
       for(i=0;i<8;i++) Chan2Brd[i+8] = 1; 
       for(i=0;i<8;i++) Chan2Brd[i+16] = 0; 
       for(i=0;i<8;i++) Chan2Brd[i+24] = 1; 
    }
    else if(DCbDarray[1] != NULL)
    {
       for(i=0;i<8;i++) Chan2Brd[i] = 1;      
    }
    // Setup counter, use MPT
    MPT.begin();  
    // Need to make sure the any pending interrupts are cleared on TRIGGER. So if we are in a hardware
    // triggered mode we will first enable the trigger to a dummy ISR and then detach and reattached to the
    // real ISR. We will wait a few microseconds for the interrupt to clear.
    if(TriggerMode == EDGE) {DIhTrig.attached('R', CHANGE, Dummy_ISR); delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == POS)  {DIhTrig.attached('R', RISING, Dummy_ISR); delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == NEG)  {DIhTrig.attached('R', FALLING, Dummy_ISR); delayMicroseconds(10); DIhTrig.detach();}
    if(TriggerMode == SW)   {DIhTrig.detach(); MPT.setTrigger(TC_CMR_EEVTEDG_NONE);}
    if(TriggerMode == EDGE) {DIhTrig.attached('R', CHANGE, Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_EDGE);}
    if(TriggerMode == POS)  {DIhTrig.attached('R', RISING, Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_RISING);}
    if(TriggerMode == NEG)  {DIhTrig.attached('R', FALLING, Trigger_ISR); MPT.setTrigger(TC_CMR_EEVTEDG_FALLING);}
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
 //   pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] &= 0x00FF0F;  // Set DLYBCT delay between bytes to zero
 //   pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] |= 8 << 4;    // Set 16 bit transfer mode

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
      if(TEheader->Count == 0) MPT.setTIOAeffect(TEheader->Count,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE | TC_CMR_AEEVT_TOGGLE);
      else MPT.setTIOAeffect(TEheader->Count,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE);
    }
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
    MPT.softwareTrigger();
}

inline void StopTimer(void)
{
    // Exit and do nothing if no tables are loaded
    if(TablesLoaded[CT] <= 0) return;
    // Stop the timer
    MPT.stop();
    DIhTrig.detach();
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
        if(NS.Count[NS.Ptr-1] >= NS.Table[NS.Ptr-1]->RepeatCount) NS.Ptr--;
    if(NS.Ptr < 0) NS.Ptr = 0;
    // If the current table is named then its on the nesting stack so remove it
//    if((Theader->TableName != 0) && (Theader->TableName != 0xFF)) NS.Ptr--;
//    if(NS.Ptr < 0) NS.Ptr = 0;
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

  // Exit if counter is not enabled
  if(!TableTriggered) return;
  // Validate its a rising edge and debounce
  delayMicroseconds(1);    // Delay and then make sure the S input pin is high
  if((pio->PIO_PDSR & pin) == 0) return;  
  // Advance the counter
  Counter++;
  // Compare to RA and call ISR if equal
  if(Counter == MPTtc.TC_RA) 
  {
    if(StopRequest) StopTimer();
    RAmatch_Handler();
  }
  // Compare to RC and call ISR if equal and reset the count  
  if(Counter == MPTtc.TC_RC) RCmatch_Handler();
  if(Counter == MPTtc.TC_RC) Counter = 0;
}

void ClockSsetup(void)
{
   attachInterrupt(digitalPinToInterrupt(SoftClockDIO), ISRclk, RISING);
   Counter=0;
}

void ClockSstop(void)
{
   TableStopped = true;
   detachInterrupt(digitalPinToInterrupt(DI2));
   Counter=0;
}

inline void SetupNextEntry(void)
{
   static int   i,k;
   int SPIadd,cSPIadd;
   static bool  DIOchange;
   static Pio *pio = g_APinDescription[ADDR0].pPort;

    ValueChange = true;
    DIOchange=false;
    // Set the address
SetupNextEntryAgain:  // sorry
    pio->PIO_CODR = 7;                                    // Set all bits low
    if(DCbDarray[0] != NULL) SPIadd = pio->PIO_SODR = DCbDarray[0]->DACspi & 7;    // Set address for channels 0 through 15
    else if(DCbDarray[1] != NULL) SPIadd = pio->PIO_SODR = DCbDarray[1]->DACspi & 7;
    // Timer count where these values are set
    MPTtc.TC_RA = TEheader->Count;
    MPTtc.TC_RC = Theader->MaxCount;
SetupNextEntryAgain2:
    // Process the current entry, all channels
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
                    // Advance to the next table
                    if(!AdvanceTablePointer())
                    {
                        // All done so stop the timer
                        StopRequest = true;
                        if(DIOchange) DOrefresh;   // Added Jan 15, 2015
                        return;
                    }
                    // Setup for the first event in the next table if there is a time 0
                    // event. This table's time zero event happens at the same time as the
                    // current tables top count.
                    if(TEheader->Count == 0)
                    {
                       // Update the DIO hardware if needed
                       if(DIOchange) DOrefresh;
                       goto SetupNextEntryAgain;
                       //SetupNextEntry();
                       //return;
                    }
                }
                else
                {
                    // Replay the table on the top of the nesting stack.
                    Theader = NS.Table[NS.Ptr-1];
                    TEheader = (TableEntryHeader *)((char *)Theader + sizeof(TableHeader));
                    Tentry = (TableEntry *)((char *)TEheader + sizeof(TableEntryHeader));
                    TentryCount = 0;
                    // Setup for the first event in the next table if there is a time 0
                    // event. This table's time zero event happens at the same time as the
                    // current tables top count.
                    if(TEheader->Count == 0)
                    {
                       // Update the DIO hardware if needed
                       if(DIOchange) DOrefresh;
                       goto SetupNextEntryAgain;
                       //SetupNextEntry();
                       //return;
                    }
                }
                if(DIOchange) DOrefresh;
                return;
            }
            // If Chan is 0 to 31 its a DC bias output so send to DAC
            if((Tentry[i].Chan >= 0) && (Tentry[i].Chan <= 31))
            {
              // See if the SPI address is correct, if not update
              if(Tentry[i].Chan <= 15)
              {
                cSPIadd = 2; // Just in case
                if(DCbDarray[0] != NULL) cSPIadd = DCbDarray[0]->DACspi & 7;
                else if(DCbDarray[1] != NULL) cSPIadd = DCbDarray[1]->DACspi & 7;
                if(SPIadd != cSPIadd)
                {
                   pio->PIO_CODR = 7;
                   SPIadd = pio->PIO_SODR = cSPIadd;
                }
              }
              else if(Tentry[i].Chan <= 31)
              {
                cSPIadd = 0; // Just in case
                if(DCbDarray[2] != NULL) cSPIadd = DCbDarray[2]->DACspi & 7;
                if(SPIadd != cSPIadd)
                {
                   pio->PIO_CODR = 7;
                   SPIadd = pio->PIO_SODR = cSPIadd;
                }                
              }
              //DCbiasDACupdate(Tentry[i].Chan, Tentry[i].Value);
              SelectBoard(Chan2Brd[Tentry[i].Chan]);
              k=Tentry[i].Value;
              SPI.transfer(SPI_CS, (uint8_t *)&k, 4);
/*  This inline code did not give significant speed increase.
              static Spi* pSpi = SPI0;
              static uint32_t b;
              static uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(SPI_CS);
              uint16_t *buf = (uint16_t *)&Tentry[i].Value;
              pSpi->SPI_TDR = (uint32_t)buf[0] | SPI_PCS(ch);
              while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
              b = pSpi->SPI_RDR;
              pSpi->SPI_TDR = (uint32_t)buf[1] | SPI_PCS(ch) | SPI_TDR_LASTXFER;
              while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
              b = pSpi->SPI_RDR;
//              pSpi->SPI_TDR = (uint32_t)buf[2] | SPI_PCS(ch);
//             while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
//              b = pSpi->SPI_RDR;
//              pSpi->SPI_TDR = (uint32_t)buf[3] | SPI_PCS(ch) | SPI_TDR_LASTXFER;
//              while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
//              b = pSpi->SPI_RDR;
*/
            }
            // if Chan is A through P its a DIO to process
            else if((Tentry[i].Chan >= 'A') && (Tentry[i].Chan <= 'P'))
            {
                DIOchange = true;
                SDIO_Set_Image(Tentry[i].Chan,Tentry[i].Value);
            }
            // if Chan is t then this is a trigger out pulse so queue it up for next ISR
            else if(Tentry[i].Chan == 't') QueueTriggerOut(Tentry[i].Value);
            else if(Tentry[i].Chan == 'b') QueueBurst(Tentry[i].Value);
        }
        break;
    }
    // Update the DIO hardware if needed
    if(DIOchange) DOrefresh;
    TentryCount++;
    // Advance to next entry
    if(TentryCount < Theader->NumEntries)
    {
        AdvanceEntryPointer();
    }
    else
    {
        if(!AdvanceTablePointer())
        {
            // All done so stop the timer
            StopRequest = true;
            return;
        }
        else
        {
          // If this table has a time 0 even then setup for it, nov 3, 2016
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
  TRACE(3);
}

void Trigger_ISR(void)
{
  static Pio *pio = g_APinDescription[BRDSEL].pPort;
  static uint32_t pin =g_APinDescription[BRDSEL].ulPin;

   uint32_t csb = pio->PIO_ODSR & pin;
   TRACE(4);
   if(MPT.getRAcounter() == 0)
   {
     ValueChange = true;
     if((MIPSconfigData.Rev <= 1) || (softLDAC))// Rev 1 used software control of LDAC
     {
       LDAClow;
       LDAChigh;
     }
     SetupNextEntry();
     if(csb == 0)  pio->PIO_CODR = pin;
     else pio->PIO_SODR = pin; 
   }
   if(ClockMode == EXTS)
   {
     StartTimer();
   }
  // Set trigger to software, this prevents hardware retriggering. Added Jan 14, 2015 GAA
//  MPT.setTrigger(TC_CMR_EEVTEDG_NONE);
//  detachInterrupt(TRIGGER);
}

// This interrupt happens when the compare register matches the timer value. At this time the
// data is latched and the next values are written.
inline void RAmatch_Handler(void)
{
  static Pio *pio = g_APinDescription[BRDSEL].pPort;
  static uint32_t pin =g_APinDescription[BRDSEL].ulPin;

  uint32_t csb = pio->PIO_ODSR & pin;
  TRACE(5);
  ValueChange = true;
  ProcessTriggerOut();
  ProcessBurst();
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
  static Pio *pio = g_APinDescription[BRDSEL].pPort;
  static uint32_t pin =g_APinDescription[BRDSEL].ulPin;

    uint32_t csb = pio->PIO_ODSR & pin;
    TRACE(6);
    if(StopRequest == true)
    {
        StopTimer();
        StopRequest = false;
        return;
    }
    ValueChange = true;
    // If compare register A is zero then at this point call setup next,
    // also if rev 1 pulse LDAC as well
    if(MPT.getRAcounter() == 0)
    {
      ProcessTriggerOut();
      ProcessBurst();
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





