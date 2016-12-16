#include "ARB.h"
#include "Hardware.h"
#include "Menu.h"
#include "Dialog.h"
#include "Variants.h"
#include "Compressor.h"

DialogBoxEntry ARBCompressorEntries[] = {
  {" Mode"                , 0, 1, D_LIST   , 0,  0, 8, 15, false, CmodeList, Cmode, NULL, ARBupdateMode},
  {" Order"               , 0, 2, D_UINT8  , 0, 255, 1, 20, false, "%3d", &arb.Corder, NULL, ARBupdateCorder},
  {" Compression table"   , 0, 3, D_TITLE  , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" "                    , 0, 4, D_STRING , 0, 2, 0, 2, false, "%-20.20s", TwaveCompressorTable, NULL, NULL},
  {" Trig delay, mS"      , 0, 5, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &arb.Tdelay, NULL, NULL},
  {" Compress t, mS"      , 0, 6, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &arb.Tcompress, NULL, NULL},
  {" Normal t, mS"        , 0, 7, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &arb.Tnormal, NULL, NULL},
  {" Non Comp t, mS"      , 0, 8, D_FLOAT  , 0.1, 99999, 0.1, 16, false, "%7.1f", &arb.TnoC, NULL, NULL},
  {" Next page"           , 0, 10, D_PAGE  , 0, 0, 0, 0, false, NULL, &ARBCompressorEntries2, NULL, NULL},
  {" Return to ARB menu"  , 0, 11, D_DIALOG, 0, 0, 0, 0, false, NULL, &ARBdialog, NULL, NULL},
  {NULL},
};

DialogBoxEntry ARBCompressorEntries2[] = {
  {" Trig input"          , 0, 1, D_DI     , 0, 0, 2, 21, false, NULL, &arb.Ctrig, NULL, ARBconfigureTrig},
  {" Trig level"          , 0, 2, D_DILEVEL, 0, 0, 4, 19, false, NULL, &arb.CtrigLevel, NULL, ARBconfigureTrig},
  {" Force trigger"       , 0, 4, D_FUNCTION,0, 0, 0, 0,  false, NULL, NULL, ARBcompressorTriggerISR, ARBcompressorTriggerISR},
  {" Switch output"       , 0, 6, D_DO     , 0, 0, 2, 21, false, NULL, &arb.Cswitch, NULL, ARBsetSwitch},
  {" Switch level"        , 0, 7, D_DOLEVEL, 0, 0, 4, 19, false, NULL, &arb.CswitchLevel, NULL, ARBsetSwitch},
  {" Switch state"        , 0, 8, D_LIST   , 0, 0, 5, 18, false, CswitchList, CswitchState, NULL, ARBsetSwitch},
  {" First page"          , 0, 10,D_PAGE   , 0, 0, 0, 0,  false, NULL, &ARBCompressorEntries, NULL, NULL},
  {" Return to ARB menu"  , 0, 11,D_DIALOG , 0, 0, 0, 0,  false, NULL, &ARBdialog, NULL, NULL},
  {NULL},
};

DialogBox ARBCompressorDialog = {{"Compressor params", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0,false, ARBCompressorEntries
};

// Set compressor mode, this is done with hardware output pin to signal enabled ARB module
void ARBupdateMode(void)
{
  if(strcmp(Cmode,"Normal")==0) ARBnormal;
  else ARBcompress; 
}

void ARBupdateCorder(void)
{
  SetByte(CompressBoard, TWI_SET_COMP_ORDER, arb.Corder);
}

void ARBconfigureTrig(void)
{
  CtrigInput->detach();
  CtrigInput->attached(arb.Ctrig, arb.CtrigLevel, ARBcompressorTriggerISR);    
}

void ARBsetSwitch(void)
{
  if(strcmp(CswitchState,"Open")==0) SetOutput(arb.Cswitch,arb.CswitchLevel);
  else ClearOutput(arb.Cswitch,arb.CswitchLevel);  
}

void ARBswitchTimerISR(void)
{
  switch (CSState)
  {
    case CSS_OPEN_REQUEST:
      // Open switch and set timer to close
      SetOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
      CSState = CSS_CLOSE;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
     break;
    case CSS_CLOSE_REQUEST:
      // Close switch and set timer to close
      ClearOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
      CSState = CSS_OPEN;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
      break;
    case CSS_OPEN:
      SetOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
      break;
    case CSS_CLOSE:
      ClearOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);  
      break;
    default:
      break;
  }  
}

// Set twave channel 1 voltage to the value defined in the channel 1 data structure
void ARBCsetV(void)
{
   int b=SelectedBoard();
   SetFloat(0,TWI_SET_RANGE, ARBarray[0].Voltage);
   SelectBoard(b);   
}
// Set twave channel 2 voltage to the value defined in the channel 2 data structure
void ARBCsetv(void)
{
   int b=SelectedBoard();
   SetFloat(1,TWI_SET_RANGE, ARBarray[1].Voltage);
   SelectBoard(b);     
}
// Set the twave channel 1 voltage to the value defined in the channel 2 data structure
void ARBCsetV1toV2(void)
{
   int b=SelectedBoard();
   SetFloat(0,TWI_SET_RANGE, ARBarray[1].Voltage);
   SelectBoard(b);       
}
// Set the twave channel 2 voltage to the value defined in the channel 1 data structure
void ARBCsetV2toV1(void)
{
   int b=SelectedBoard();
   SetFloat(1,TWI_SET_RANGE, ARBarray[0].Voltage);
   SelectBoard(b);         
}
// Sets the compression order
void ARBCsetOrder(void)
{
   int b=SelectedBoard();
   SetByte(CompressBoard,TWI_SET_COMP_ORDER, ARBarray[0].Corder);
   SelectBoard(b);           
}

// Called when the timer reaches the desired time point.
// This function will update the state
void ARBcompressorTimerISR(void)
{
  char OP;

  // This interrupt occurs when the current state has timed out so advance to the next
  switch (CState)
  {
    case CS_COMPRESS:
      // If C_NormAmpMode is 1 or 2 then set the amplitude to TW1 level.
      if((C_NormAmpMode == 1) || (C_NormAmpMode == 2))
      {
         if(AcquireTWI()) ARBCsetV2toV1();
         else TWIqueue(ARBCsetV2toV1);
      }
      if(C_NormAmpMode == 2)
      {
         if(AcquireTWI()) ARBCsetV();
         else TWIqueue(ARBCsetV);          
      }
      // Next state is always normal
      ARBnormal;
      C_NextEvent += C_Tn;
      CState = CS_NORMAL;
      break;
    case CS_NORMAL:
    case CS_NONCOMPRESS:
      CurrentPass++;
    case CS_TRIG:
    case CS_DELAY:
      // State will be Compress, NonCompress or 0 indicating finished, defined by table value
      OP = ARBgetNextOperationFromTable(false);
      if(OP == 'C')
      {
        CState = CS_COMPRESS;
        C_NextEvent += C_Tc;
        ARBcompress;
        // If C_NormAmpMode is 1 or 2 then set the amplitude to TW2 level.
        if((C_NormAmpMode == 1) || (C_NormAmpMode == 2))
        {
           if(AcquireTWI()) ARBCsetv();
           else TWIqueue(ARBCsetv);
        }
        if(C_NormAmpMode == 2)
        {
           if(AcquireTWI()) ARBCsetV1toV2();
           else TWIqueue(ARBCsetV1toV2);          
        }
      }
      else if(OP == 'N')
      {
        CState = CS_NONCOMPRESS;
        C_NextEvent += C_Tnc;
        ARBnormal;
      }
      else if(OP == 'D')
      {
        CState = CS_DELAY;  // Delay, hold current mode during delay
        C_NextEvent += C_Delay;
      }
      break;
    default:
      break;
  }
  // Test if all passes are complete, if so stop the timer and exit
  if(OP == 0)
  {
    // Stop the timer
    CompressorTimer.stop();
    // Restore the mode
    ARBupdateMode();
    return;
  }
  // Update the timer
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);  
}

void ARBcompressorTriggerISR(void)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Clear and setup variables
  ARBnormal;             // Put system in normal mode
  ARBgetNextOperationFromTable(true);
  C_NextEvent = C_Td;
  CState = CS_TRIG;
  // Setup the timer used to generate interrupts
  CompressorTimer.begin();
  CompressorTimer.attachInterruptRA(ARBcompressorTimerISR);
  CompressorTimer.attachInterruptRB(ARBswitchTimerISR);
  CompressorTimer.setTrigger(TC_CMR_EEVTEDG_NONE);
  CompressorTimer.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);
  CompressorTimer.setTIOAeffectNOIO(C_NextEvent,TC_CMR_ACPA_TOGGLE);
  CompressorTimer.enableTrigger();
  CompressorTimer.softwareTrigger();  
}

char ARBgetNextOperationFromTable(bool init)
{
  int index,b;
  static int tblindex=0;
  static char OP;
  static int count = 0;

  if(init)
  {
    CompressorStackInit();
    tblindex = 0;
    count = 0;
    return(0);
  }
  if(count > 0)
  {
    count--;
    return(OP);
  }
  while(1)
  {
    // Find a valid character
    while(1)
    {
      if(TwaveCompressorTable[tblindex] == 0) return(0);
      OP = TwaveCompressorTable[tblindex++];
      count = 1;  // Default to count of 1
      if(isDigit(TwaveCompressorTable[tblindex]))
      {
        // If here then get the value, it has to be an integer
        count = int(TwaveCompressorTable[tblindex++] - '0');
        while(isDigit(TwaveCompressorTable[tblindex])) count = count * 10 + int(TwaveCompressorTable[tblindex++] - '0');
      }
      break;
    }
    if((OP=='N')||(OP=='C'))
    {
      // Only return valid options, N for non compressed and C for compressed
      count--;
      return(OP);
    }
    if(OP == 'D') //Delay
    {
      C_Delay = (((float)count) / 1000.0) * C_clock;
      count = 0;
      return(OP);
    }
    if(OP == 'g') //Time to open gate
    {
      C_GateOpenTime = (((float)count) / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_OPEN_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    if(OP == 'G') //Time to close gate
    {
      C_GateOpenTime = (((float)count) / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_CLOSE_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    if(OP == 'S')
    {
      if(count == 0) ClearOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
      if(count == 1) SetOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
    }
    if(OP == 'O')
    {
      if((count >= 0) && (count <= 255))
      {
         ARBarray[0].Corder = count;
         ARBCsetOrder();
      }
    }
    if(OP == 'V')
    {
      if((count >= 0) && (count <= 100))
      {
        if (SelectedARBboard == 0) arb.Voltage = count;
        ARBarray[0].Voltage = count; 
        if(AcquireTWI()) ARBCsetV();
        else TWIqueue(ARBCsetV);
      }   
    }
    if(OP == 'v')
    {
      if((count >= 0) && (count <= 100))
      {
        if (SelectedARBboard == 1) arb.Voltage = count;
        ARBarray[1].Voltage = count;
        if(AcquireTWI()) ARBCsetv();
        else TWIqueue(ARBCsetv);
      }      
    }
    if(OP == 'F')
    {
      if (index != -1)
      {
        if((count > 100) && (count <= 40000))
        {
           ARBarray[index].Frequency = count;
           SetARBcommonClock(count);
        }   
      }      
    }
    if(OP == 'c')
    {
      arb.Tcompress = ARBarray[0].Tcompress = count;
      C_Tc  = (ARBarray[0].Tcompress / 1000.0) * C_clock;
    }
    if(OP == 'n')
    {
      arb.Tnormal = ARBarray[0].Tnormal = count;
      C_Tn  = (ARBarray[0].Tnormal / 1000.0) * C_clock;
    }
    if(OP == 't')
    {
      arb.TnoC = ARBarray[0].TnoC = count;
      C_Tnc = (ARBarray[0].TnoC / 1000.0) * C_clock;
    }
    if(OP == 's') ARBclock->stop();                            // Stop the clock
    if(OP == 'r') ARBclock->start(-1, 0, true);                // Restart the clock
    if(OP == 'o') C_SwitchTime = (count / 1000.0) * C_clock;   // Sets the switch open time
    if(OP == 'M') C_NormAmpMode = count;                       // Set compressor normal amplitude mode
    if(OP == '[') CompressorLoopStart(tblindex);
    if(OP == ']')
    {
      int i = CompressorProcessLoop(count);
      if(i != -1) tblindex = i;
    }    
  }
}

void ARBcompressor_init(void)
{
  int            i;
  DialogBoxEntry *de;
  
  if(!ARBarray[0].CompressorEnabled) return;  // Exit if the compressor is not enabled
  ARBarray[0].UseCommonClock = true;          // If we are in compressor mode then we must use a common clock
  ARBarray[1].UseCommonClock = true;
  // Enable the compressor hardware mode contol line in the compress ARB module
  pinMode(ARBmode,OUTPUT);
  SetBool(0,TWI_SET_COMP_EXT,false);
  SetBool(1,TWI_SET_COMP_EXT,false);
  SetBool(CompressBoard,TWI_SET_COMP_EXT,true);
  // Clear the switch output
  ClearOutput(ARBarray[0].Cswitch,ARBarray[0].CswitchLevel);
  // Enable the compressor menu selection 
  de = GetDialogEntries(ARBentriesPage2, "Compressor");
  if(de != NULL) de->Type = D_DIALOG;
  // Setup the trigger line and ISR
  CtrigInput = new DIhandler;
  CtrigInput->detach();
  CtrigInput->attached(ARBarray[0].Ctrig, ARBarray[0].CtrigLevel, ARBcompressorTriggerISR);
  ARBupdateMode();
  ARBupdateCorder();
  ARBsetSwitch();
}

void ARBcompressor_loop(void)
{
  if(!ARBarray[0].CompressorEnabled) return;  // Exit if the compressor is not enabled
  // Calculate all the times in clock count units
  C_Td  = (ARBarray[0].Tdelay / 1000.0) * C_clock;
  C_Tc  = (ARBarray[0].Tcompress / 1000.0) * C_clock;
  C_Tn  = (ARBarray[0].Tnormal / 1000.0) * C_clock;
  C_Tnc = (ARBarray[0].TnoC / 1000.0) * C_clock;
  if (ActiveDialog == &ARBCompressorDialog) RefreshAllDialogEntries(&ARBCompressorDialog);  
}

//
// ARB compressor host command functions
//

void SetARBCmode(char *mode)
{
  if ((strcmp(mode, "Normal") == 0) || (strcmp(mode, "Compress") == 0))
  {
    strcpy(Cmode,mode);
    ARBupdateMode();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetARBCorder(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(ARBarray[0].Corder);
}

void SetARBCorder(int ival)
{
  if(RangeTest(ARBCompressorEntries,"Order",ival))
  {
    ARBarray[0].Corder = ival;
    arb.Corder = ival;
    ARBupdateCorder();
    SendACK;
  }
}

void SetARBCtriggerDelay(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Trig delay, mS",fval))
  {
    ARBarray[0].Tdelay = fval;
    arb.Tdelay = fval;
    SendACK;
  }
}

void SetARBCcompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Compress t, mS",fval))
  {
    ARBarray[0].Tcompress = fval;
    arb.Tcompress = fval;
    SendACK;
  }
}

void SetARBCnormalTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Normal t, mS",fval))
  {
    ARBarray[0].Tnormal = fval;
    arb.Tnormal = fval;
    SendACK;
  }
}

void SetARBCnoncompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Normal t, mS",fval))
  {
    ARBarray[0].TnoC = fval;
    arb.TnoC = fval;
    SendACK;
  }
}

void ARBCtrigger(void)
{
   SendACK;
   ARBcompressorTriggerISR();
}

void SetARBCswitch(char *mode)
{
  if ((strcmp(mode, "Open") == 0) || (strcmp(mode, "Close") == 0))
  {
    strcpy(CswitchState,mode);
    ARBsetSwitch();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}
// End of compressor host command routines


