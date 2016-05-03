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
#include <Thread.h>
#include <ThreadController.h>

extern Menu MainMenu;
extern ThreadController control;
extern DialogBox DODialog;
extern DialogBox DIDialog;
void AddMainMenuEntry(MenuEntry *);

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
  0,0,
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
  0,0,
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






