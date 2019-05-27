//
// DIO.cpp
//
// This file supports the Digitial input and output functions on the MIPS system. The MIPS controller
// has 16 digitial output channels and 8 digitial inputs.
//
// Digitial outputs are named A though P.
// Digitial input are named Q through X.
//
// Serial IO commands support reading the state and controlling the output.
//
// Gordon Anderson
//
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include "DIO.h"
#include "Hardware.h"
#include "Serial.h"
#include "Errors.h"
#include "Menu.h"
#include "Dialog.h"
#include "Variants.h"
#include "AtomicBlock.h"
#include <Thread.h>
#include <ThreadController.h>

extern Menu MainMenu;
extern ThreadController control;
extern DialogBox DODialog;
extern DialogBox DIDialog;
void   AddMainMenuEntry(MenuEntry *);

//MIPS Threads
Thread DIOThread  = Thread();

int DigitialInputs[8];
int DigitialOutputs[16];

void UpdateDigitialOutputArray(void)
{
  int   i;

  // Update the Digitial output array using the image registers
  for (i = 0; i < 8; i++)
  {
    if ((MIPSconfigData.DOlsb & (1 << i)) != 0) DigitialOutputs[i] = 1;
    else DigitialOutputs[i] = 0;
    if ((MIPSconfigData.DOmsb & (1 << i)) != 0) DigitialOutputs[i + 8] = 1;
    else DigitialOutputs[i + 8] = 0;
  }
}

void SetImageRegs(void)
{
  int   i;

  for (i = 0; i < 8; i++)
  {
    if (DigitialOutputs[i] == 1) MIPSconfigData.DOlsb |= (1 << i);
    else MIPSconfigData.DOlsb &= ~(1 << i);

    if (DigitialOutputs[i + 8] == 1) MIPSconfigData.DOmsb |= (1 << i);
    else MIPSconfigData.DOmsb &= ~(1 << i);
  }
  // Send to the hardware
  DOrefresh;
  // Toggle LDAC
  PulseLDAC;
}

DialogBoxEntry DIDialogEntries[] = {
  {" Input Q (CLK)", 0, 1, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[0], NULL, NULL},
  {" Input R (TRG)", 0, 2, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[1], NULL, NULL},
  {" Input S", 0, 3, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[2], NULL, NULL},
  {" Input T", 0, 4, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[3], NULL, NULL},
  {" Input U", 0, 5, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[4], NULL, NULL},
  {" Input V", 0, 6, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[5], NULL, NULL},
  {" Input W", 0, 7, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[6], NULL, NULL},
  {" Input X", 0, 8, D_INT, 0, 1, 1, 21, true, "%2d", &DigitialInputs[7], NULL, NULL},
  {" Digital outputs", 0, 9, D_DIALOG, 0, 0, 0, 0, false, NULL, &DODialog, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox DIDialog = {
  {
    "Digital Inputs",
    ILI9340_BLACK,
    ILI9340_WHITE,
    2,
    0, 0,
    300, 220,
    B_DOUBLE,
    12
  },
  M_SCROLLING,
  0,0,false,
  DIDialogEntries
};

DialogBoxEntry DODialogEntries[] = {
  {" Output A",  0, 1, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[0], NULL, SetImageRegs},
  {" Output B", 12, 1, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[1], NULL, SetImageRegs},
  {" Output C",  0, 2, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[2], NULL, SetImageRegs},
  {" Output D", 12, 2, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[3], NULL, SetImageRegs},
  {" Output E",  0, 3, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[4], NULL, SetImageRegs},
  {" Output F", 12, 3, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[5], NULL, SetImageRegs},
  {" Output G",  0, 4, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[6], NULL, SetImageRegs},
  {" Output H", 12, 4, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[7], NULL, SetImageRegs},
  {" Output I",  0, 5, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[8], NULL, SetImageRegs},
  {" Output J", 12, 5, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[9], NULL, SetImageRegs},
  {" Output K",  0, 6, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[10], NULL, SetImageRegs},
  {" Output L", 12, 6, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[11], NULL, SetImageRegs},
  {" Output M",  0, 7, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[12], NULL, SetImageRegs},
  {" Output N", 12, 7, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[13], NULL, SetImageRegs},
  {" Output O",  0, 8, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[14], NULL, SetImageRegs},
  {" Output P", 12, 8, D_INT, 0, 1, 1, 9, false, "%2d", &DigitialOutputs[15], NULL, SetImageRegs},
  {" Digital inputs", 0, 9, D_DIALOG, 0, 0, 0, 0, false, NULL, &DIDialog, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox DODialog = {
  {
    "Digital Outputs",
    ILI9340_BLACK,
    ILI9340_WHITE,
    2,
    0, 0,
    300, 220,
    B_DOUBLE,
    12
  },
  M_SCROLLING,
  0,0,false,
  DODialogEntries
};

MenuEntry MEDIOmonitor = {" DIO module", M_DIALOG, 0, 0, 0, NULL, &DIDialog, NULL, NULL};

// This function is called at powerup to initiaize the RF driver.
void DIO_init(void)
{
  // Update the Digitial output array using the image registers
  UpdateDigitialOutputArray();
  // Setup the menu
  AddMainMenuEntry(&MEDIOmonitor);
  if (ActiveDialog == NULL) DialogBoxDisplay(&DIDialog);
  // Configure Threads
  DIOThread.setName("DIO");
  DIOThread.onRun(DIO_loop);
  DIOThread.setInterval(100);
  // Add threads to the controller
  control.add(&DIOThread);
  // Init the output bits
  SetImageRegs();
}

// This function is call by the main loop every 100 millisec
void DIO_loop(void)
{
  int  i;

  // Read input bits and set digitial input array for the display
  for (i = 0; i < 8; i++) DigitialInputs[i] = 0;
  if (digitalRead(DI0) == HIGH) DigitialInputs[0] = 1;
  if (digitalRead(DI1) == HIGH) DigitialInputs[1] = 1;
  if (digitalRead(DI2) == HIGH) DigitialInputs[2] = 1;
  if (digitalRead(DI3) == HIGH) DigitialInputs[3] = 1;
  if (digitalRead(DI4) == HIGH) DigitialInputs[4] = 1;
  if (digitalRead(DI5) == HIGH) DigitialInputs[5] = 1;
  if (digitalRead(DI6) == HIGH) DigitialInputs[6] = 1;
  if (digitalRead(DI7) == HIGH) DigitialInputs[7] = 1;
  // If the digitial input dialog is displayed then update the data
  // For performance reasons update 2 lines on each call to this function
  if (ActiveDialog == &DIDialog)  RefreshAllDialogEntries(&DIDialog);
  if (ActiveDialog == &DODialog)  RefreshAllDialogEntries(&DODialog);
}

// This function is called by the serial command processor to handle the GDIO command.
// The string passed to this function contains the channel number, A through X. A through
// P are digitial outputs and Q through X are digitial inputs.
void GDIO_Serial(char *CH)
{
  uint8_t  DIbits;
  uint8_t  BitNum;

  // Validate the requested channel
  if ((CH[0] >= 'A') && (CH[0] <= 'X'))
  {
    SendACKonly;
    if (CH[0] >= 'Q')
    {
      // Here is this is a digitial input channel. Read the input and respond
      DIbits = DigitalIn();
      if (((1 << (CH[0] - 'Q')) & DIbits) != 0) {
        if (!SerialMute) serial->println("1");
      }
      else {
        if (!SerialMute) serial->println("0");
      }
      return;
    }
    // Here if this is a digitial output. In this case read the image register
    // to determine the output condition.
    BitNum = CH[0] - 'A';
    if (BitNum >= 8)
    {
      if (((1 << (BitNum - 8)) & MIPSconfigData.DOmsb) != 0) {
        if (!SerialMute) serial->println("1");
      }
      else {
        if (!SerialMute) serial->println("0");
      }
      return;
    }
    else
    {
      if (((1 << (BitNum)) & MIPSconfigData.DOlsb) != 0) {
        if (!SerialMute) serial->println("1");
      }
      else {
        if (!SerialMute) serial->println("0");
      }
      return;
    }
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This function sets the bits in the DO image registers.
// The passed parameters are not range tested, its assumed they are correct.
// chan 'A' through 'P'
// val '0' or '1'
void SDIO_Set_Image(char chan, char val)
{
  uint8_t  BitMask;

  BitMask = 1 << ((chan - 'A') & 7);
  if (chan >= 'I')
  {
    // MSB
    if (val == '1') MIPSconfigData.DOmsb |= BitMask;
    else MIPSconfigData.DOmsb &= ~BitMask;
  }
  else
  {
    // LSB
    if (val == '1') MIPSconfigData.DOlsb |= BitMask;
    else MIPSconfigData.DOlsb &= ~BitMask;
  }
}

// This function is called by the serial command processor to handle SDIO command.
// Channels A through P are valid output channels.
// Valid states are 0 and 1.
void SDIO_Serial(char *CH, char *State)
{
  // Validate the passed parameters
  if ((CH[0] < 'A') || (CH[0] > 'X'))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if ((State[0] != '0') && (State[0] != '1'))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACK;
  // Set the image bits
  SDIO_Set_Image(CH[0], State[0]);
  // Send to the hardware
  DOrefresh;
  // Toggle LDAC
  PulseLDAC;
  UpdateDigitialOutputArray();
}

// DIO general function used throughout the MIPS app

// Trigger output functions. These functions support pulsing the output in micro second units as
// well as queueing up the function for later execution, this support using the trigger output
// in a signal table.

// This function is valid for rev 2.0 and above controllers. This command
// will generate a output trigger pulse. The following commands are
// supported:
//  HIGH, sets the output high
//  LOW, sets the output low
//  PULSE, pulses the output from its current state for 1 milli sec
//  If an integer value is passed then we pulse in micro seconds
void TriggerOut(char *cmd)
{
  String Cmd;
  int uS;
  
  Cmd = cmd;
  uS = Cmd.toInt();
  if(uS >0)
  {
     SendACK;
     TriggerOut(uS);
     return;
  }
  if ((strcmp(cmd, "HIGH") == 0) || (strcmp(cmd, "LOW") == 0) || (strcmp(cmd, "PULSE") == 0) || (strcmp(cmd, "FollowS") == 0))
  {
    SendACK;
    if (strcmp(cmd, "FollowS") == 0) TriggerFollowS();
    if (strcmp(cmd, "HIGH") == 0) digitalWrite(TRGOUT, LOW);
    if (strcmp(cmd, "LOW") == 0) digitalWrite(TRGOUT, HIGH);
    if (strcmp(cmd, "PULSE") == 0)
    {
      if (digitalRead(TRGOUT) == HIGH)
      {
        digitalWrite(TRGOUT, LOW);
        delay(1);
        digitalWrite(TRGOUT, HIGH);
      }
      else
      {
        digitalWrite(TRGOUT, HIGH);
        delay(1);
        digitalWrite(TRGOUT, LOW);
      }
    }
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void AuxOut(char *cmd)
{
  String Cmd;
  
  Cmd = cmd;
  if ((strcmp(cmd, "HIGH") == 0) || (strcmp(cmd, "LOW") == 0) || (strcmp(cmd, "PULSE") == 0))
  {
    SendACK;
    if (strcmp(cmd, "HIGH") == 0) digitalWrite(AUXTRGOUT, LOW);
    if (strcmp(cmd, "LOW") == 0) digitalWrite(AUXTRGOUT, HIGH);
    if (strcmp(cmd, "PULSE") == 0)
    {
      if (digitalRead(AUXTRGOUT) == HIGH)
      {
        digitalWrite(AUXTRGOUT, LOW);
        delay(1);
        digitalWrite(AUXTRGOUT, HIGH);
      }
      else
      {
        digitalWrite(AUXTRGOUT, HIGH);
        delay(1);
        digitalWrite(AUXTRGOUT, LOW);
      }
    }
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

void FollowSisr(void)
{
   static Pio *pioS = g_APinDescription[DI2].pPort;
   static uint32_t pinS = g_APinDescription[DI2].ulPin;
   static Pio *pioTrig = g_APinDescription[TRGOUT].pPort;
   static uint32_t pinTrig = g_APinDescription[TRGOUT].ulPin;

   if(pioS->PIO_PDSR & pinS) pioTrig->PIO_CODR = pinTrig;
   else pioTrig->PIO_SODR = pinTrig;
}

void TriggerFollowS(void)
{
   static Pio *pioS = g_APinDescription[DI2].pPort;
   static uint32_t pinS = g_APinDescription[DI2].ulPin;
   
   attachInterrupt(digitalPinToInterrupt(DI2), FollowSisr, CHANGE);
   pioS->PIO_SCIFSR = pinS;
   pioS->PIO_IFER = pinS;  // Enable the fast deglitch
}

// This function will pulse the trigger output 
void TriggerOut(int microSec, bool WithLimits)
{
  static Pio *pio = g_APinDescription[TRGOUT].pPort;
  static uint32_t pin = g_APinDescription[TRGOUT].ulPin;
  static uint32_t dwReg;
  static int16_t skip = 0;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if((TrigMax > 0) && WithLimits)
  {
    if(++skip > ((PulseFreq-1) / TrigMax)) skip = 0;
    if(skip != 0) 
    {
      PlayTpulseFunctions();
      return;
    }
  }
  dwReg = pio->PIO_ODSR;       // Get output state
  if((dwReg & pin) == 0)
  {
    pio->PIO_SODR = pin;         // Set output high
    delayMicroseconds(microSec);
    pio->PIO_CODR = pin;         // Set output low
  }
  else
  {
    pio->PIO_CODR = pin;         // Set output low
    delayMicroseconds(microSec);
    pio->PIO_SODR = pin;         // Set output high
  }
  PlayTpulseFunctions();
}

#define NUMQTF 5
void  (*QuededTrigFunctionList[NUMQTF])() = {NULL,NULL,NULL,NULL,NULL};

// This function adds or removes a function from a list of up to five
// function pointers. These functions are called after the Trigger
// pulse is generated. 
// if add = true the function is added, if false its removed
bool QueueTpulseFunction(void (*Tfunction)(), bool add)
{
  int i;
  
  if(add)
  {
    // First test if its already in the list
    for(i=0;i<NUMQTF;i++) if(QuededTrigFunctionList[i] == Tfunction) return true;
    // Add the function to the list
    for(i=0;i<NUMQTF;i++)
    {
      if(QuededTrigFunctionList[i] == NULL) 
      {
        QuededTrigFunctionList[i] = Tfunction;
        return true;
      }
    }
    return false;
  }
  else
  {
    for(i=0;i<NUMQTF;i++)
    {
      if(QuededTrigFunctionList[i] == Tfunction) 
      {
        QuededTrigFunctionList[i] = NULL;
        return true;
      }
    }
    return false;  
  }
  return true;
}

// This function will call all the queued trigger functions
void PlayTpulseFunctions(void)
{
  for(int i=0;i<NUMQTF;i++)
  {
    if(QuededTrigFunctionList[i] != NULL) QuededTrigFunctionList[i](); 
  }
}

void AuxTrigger(void)
{
  static Pio *pio = g_APinDescription[AUXTRGOUT].pPort;
  static uint32_t pin = g_APinDescription[AUXTRGOUT].ulPin;
  static int16_t skip = 0;

  AtomicBlock< Atomic_RestoreState > a_Block;
  if(AuxTrigMax > 0)
  {
    if(++skip > ((PulseFreq-1) / AuxTrigMax)) skip = 0;
    if(skip != 0) return;
  }
  pio->PIO_CODR = pin;         // Set output high
  delayMicroseconds(PulseWidth);
  pio->PIO_SODR = pin;         // Set output low  
}

bool TriggerOutQueued = false;
int  TriggerOutmicroSec = 0;

// Call to queue the trigger out function for later used
void QueueTriggerOut(int microSec)
{
  TriggerOutQueued = true;
  TriggerOutmicroSec = microSec;
}

// If queued call it and clear the flag
void ProcessTriggerOut(void)
{
  if(TriggerOutQueued)
  {
    TriggerOutQueued = false;
    TriggerOut(TriggerOutmicroSec);
  }
}

// The following routines support DIO operations:
//  Input state changes
//  Input to output mirroring
// One common interrupt supports all 8 input channels. The attach interrupt is always done 
// with the change mode, the ISR will read the port to determine the actual change.
//
// Would also like to add a trigger command where an input edge triggers an output
// action. Like TRIG,Q,P,A,L this would cause output A to go low on a positive edge of input Q

// This function is called from the main pooling loop and sends the
// change messages when enabled.
// Updated 9/2/2017 to first build a string and then send the full string, needed on the MALID system
// to stop missing parts of the message.
void DIOopsReport(void)
{
  int  i;
  char chan;
  char sbuf[50];

  for(i=0;i<8;i++)
  {
    chan = 'Q' + i;
    if((dioops[i].Report) && (dioops[i].Changed))
    {
      uint32_t t = millis();
      if(dioops[i].ReportState == CHANGE) sprintf(sbuf,"DIC,%c,CHANGED,%u\n",chan,t);
      if(dioops[i].ReportState == RISING) sprintf(sbuf,"DIC,%c,RISING,%u\n",chan,t);
      if(dioops[i].ReportState == FALLING) sprintf(sbuf,"DIC,%c,FALLING,%u\n",chan,t);
      serial->print(sbuf);
      dioops[i].Changed = false;
    }
  }
}

// This interrupt service routine looks through the dioops structure and
// processes any changes. Reporting is done in the main polling loop.
// The attach interrupt is always done as change to make sure we see every transistion.
void DIOopsISR(void)
{
  int  i,j;
  bool DIOset=false;

  TRACE(9);
  for(i=0;i<8;i++)
  {
//    if((dioops[i].Report) || (dioops[i].Mirror))
    {
      j = digitalRead(dioops[i].DI);
      if(j != dioops[i].LastLevel)
      {
        dioops[i].LastLevel = j;
        if(!dioops[i].Mirror)
        {
          if(dioops[i].ReportState == CHANGE) dioops[i].Changed = true;
          if((dioops[i].ReportState == RISING) && (j == HIGH)) dioops[i].Changed = true;
          if((dioops[i].ReportState == FALLING) && (j == LOW)) dioops[i].Changed = true;
        }
        if(dioops[i].Mirror)
        {
          if(j == LOW) SDIO_Set_Image(dioops[i].DO, '0');
          else SDIO_Set_Image(dioops[i].DO, '1');
          DIOset = true;  // flag to cause update
        }
      }
    }
  }
  if(DIOset)
  {
    DOrefresh;
    PulseLDAC;
    UpdateDigitialOutputArray();
  }
}

// This function is a host command processing function. This function will set up an input port for
// change monitoring and will send a command to the host when the change is detected.
// port is Q,R,S,T,U,V,W, or X
// mode is RISING,FALLING,CHANGE, or OFF
// OFF will remove and monitoring function.
void DIOreportMonitor(char *port, char *mode, bool Report)
{
  int i;

  // Convert port to index
  i = port[0] - 'Q';
  if((i<0) || (i>7)) 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  if ((strcmp(mode, "RISING") == 0) || (strcmp(mode, "FALLING") == 0) || (strcmp(mode, "CHANGE") == 0))
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].ReportState = CHANGE;
    if (strcmp(mode, "RISING") == 0) dioops[i].ReportState = RISING;
    if (strcmp(mode, "FALLING") == 0) dioops[i].ReportState = FALLING;
    dioops[i].Report = Report;
    dioops[i].Changed = false;
    if(!dioops[i].Mirror) dioops[i].DIh->attached(port[0],CHANGE,DIOopsISR);
    SendACK;
    return;
  }
  if (strcmp(mode, "OFF") == 0)
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].ReportState = 0;
    dioops[i].Report = false;
    dioops[i].Changed = false;
    if(!dioops[i].Mirror) dioops[i].DIh->detach();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void DIOreport(char *port, char *mode)
{
  DIOreportMonitor(port,mode,true);
}

void DIOmonitor(char *port, char *mode)
{
  DIOreportMonitor(port,mode,false);  
}

void DIOchangeReport(char *port)
{
  int  i;

  i = port[0] - 'Q';
  if((i < 0) || (i > 7))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if(SerialMute) return;
  if(dioops[i].Changed)
  {
    if((dioops[i].ReportState == CHANGE)||(dioops[i].ReportState == RISING)||(dioops[i].ReportState == FALLING)) serial->println("TRUE");
    else serial->println("FALSE");
    return;
  }
  serial->println("FALSE");
}

// This function will mirror an input (Q through X) to an output (A through P or OFF)
void DIOmirror(char *in, char *out)
{
  int i;

  // Convert in to index
  i = in[0] - 'Q';
  if((i<0) || (i>7)) 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // If out is OFF then disable the mirror for this channel
  if (strcmp(out, "OFF") == 0)
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].Mirror = false;
    if(!dioops[i].Report) dioops[i].DIh->detach();
    SendACK;
    return;
  }
  if((out[0] >= 'A') && (out[0] <= 'P'))
  {
    if(dioops[i].DIh == NULL) dioops[i].DIh = new DIhandler;
    dioops[i].Mirror = true;
    dioops[i].DO = out[0];
    if(!dioops[i].Report) dioops[i].DIh->attached(in[0],CHANGE,DIOopsISR);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}
