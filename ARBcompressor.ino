#include "ARB.h"
#include "Hardware.h"
#include "Menu.h"
#include "Dialog.h"
#include "Variants.h"
#include "Compressor.h"

DialogBoxEntry ARBCompressorEntries[] = {
  {" Mode"                , 0, 1, D_LIST   , 0,  0, 8, 15, false, CmodeList, Cmode, NULL, ARBupdateMode},
  {" Order"               , 0, 2, D_INT    , 0, 65535, 1, 18, false, "%5d", &arb.Corder, NULL, ARBupdateCorder},
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
  {" Switch output"       , 0, 3, D_DO     , 0, 0, 2, 21, false, NULL, &arb.Cswitch, NULL, ARBsetSwitch},
  {" Switch level"        , 0, 4, D_DOLEVEL, 0, 0, 4, 19, false, NULL, &arb.CswitchLevel, NULL, ARBsetSwitch},
  {" Switch state"        , 0, 5, D_LIST   , 0, 0, 5, 18, false, CswitchList, CswitchState, NULL, ARBsetSwitch},
  {" Cramp"               , 0, 6, D_INT    ,-200,200,1,19,false, "%4d", &arb.Cramp, NULL, NULL},
  {" Cramp order"         , 0, 7, D_INT    , 1,100,1, 19, false, "%4d", &arb.CrampOrder, NULL, NULL},
  {" Force trigger"       , 0, 9, D_FUNCTION,0, 0, 0, 0,  false, NULL, NULL, ARBcompressorTriggerISR, ARBcompressorTriggerISR},
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
  if(arb.Corder <= 255) SetByte(CompressBoard, TWI_SET_COMP_ORDER, arb.Corder);
  else SetWord(CompressBoard, TWI_SET_COMP_ORDER_EX, arb.Corder);
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
      SetOutput(CompressorSelectedSwitch,CompressorSelectedSwitchLevel);
      CSState = CSS_CLOSE;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
     break;
    case CSS_CLOSE_REQUEST:
      // Close switch and set timer to close
      ClearOutput(CompressorSelectedSwitch,CompressorSelectedSwitchLevel);
      CSState = CSS_OPEN;
      CompressorTimer.setTIOBeffect(C_GateOpenTime + C_SwitchTime,TC_CMR_BCPB_TOGGLE);
      break;
    case CSS_OPEN:
      SetOutput(CompressorSelectedSwitch,CompressorSelectedSwitchLevel);
      break;
    case CSS_CLOSE:
      ClearOutput(CompressorSelectedSwitch,CompressorSelectedSwitchLevel);  
      break;
    default:
      break;
  }  
}

// Sets the compression order
void ARBCsetOrder(void)
{
   int b=SelectedBoard();
   if(ARBarray[0]->Corder <= 255) SetByte(CompressBoard,TWI_SET_COMP_ORDER, ARBarray[0]->Corder);
   else SetWord(CompressBoard,TWI_SET_COMP_ORDER_EX, ARBarray[0]->Corder);
   SelectBoard(b);           
}

void ARBCsetCramp(void)
{
   int b=SelectedBoard();
   Set16bitInt(CompressBoard, TWI_SET_CRAMP, ARBarray[0]->Cramp);
   SelectBoard(b);             
}

void ARBCsetCrampOrder(void)
{
   int b=SelectedBoard();
   Set16bitInt(CompressBoard, TWI_SET_CRAMPORDER, ARBarray[0]->CrampOrder);
   SelectBoard(b);               
}

void ARBsetWFT1(void)
{
  int b=SelectedBoard();
  SetWaveform(0, ARBarray[0]->wft);
  SelectBoard(b);             
}

void ARBsetWFT2(void)
{
  int b=SelectedBoard();
  SetWaveform(1, ARBarray[1]->wft);
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
         if(AcquireTWI()) SetFloat(1,TWI_SET_RANGE, ARBarray[0]->Voltage); else TWIqueue(SetFloat,1,TWI_SET_RANGE, ARBarray[0]->Voltage);
      }
      if(C_NormAmpMode == 2)
      {
         if(AcquireTWI()) SetFloat(0,TWI_SET_RANGE, ARBarray[0]->Voltage); else TWIqueue(SetFloat,0,TWI_SET_RANGE, ARBarray[0]->Voltage);
      }
      // Next state is always normal
      ARBnormal;
      C_NextEvent += C_Tn;
      CState = CS_NORMAL;
      OP = 'X';
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
           if(AcquireTWI()) SetFloat(1,TWI_SET_RANGE, ARBarray[1]->Voltage); else TWIqueue(SetFloat,1,TWI_SET_RANGE, ARBarray[1]->Voltage);
        }
        if(C_NormAmpMode == 2)
        {
           if(AcquireTWI()) SetFloat(0,TWI_SET_RANGE, ARBarray[1]->Voltage); else TWIqueue(SetFloat,0,TWI_SET_RANGE, ARBarray[1]->Voltage);
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
  CompressorTimer.setPriority(0);
 }

// Reads value from table and updates index. If no value
// found returns false and value set to 1, default.
bool ARBgetValueFromTable(int *index, int *value)
{
   bool valfound = false;
   
   *value = 1;
   if(isDigit(TwaveCompressorTable[*index]))
   {
     valfound = true;
     // If here then get the value, it has to be an integer
     *value = int(TwaveCompressorTable[(*index)++] - '0');
     while(isDigit(TwaveCompressorTable[*index])) *value = *value * 10 + int(TwaveCompressorTable[(*index)++] - '0');
   }
   return valfound;
}

// Commands processed in this routine:
//
// N      Normal cycle
// C      Compress cycle
// D      Delay
// g      Time to open gate
// G      Time to close gate
// S      This command has two options, if S is followed by a value then use the switch bit
//        defined in the ARB module data structure and the value defines open or close. If
//        S is followed by a character then it defines a port bit to set as per the 
//        value following the character.
// O      Compression order
// V      Master channel voltage p-p
// v      Slave channel voltage p-p
// F      Frequency
// c      Compress time
// n      Normal time
// t      Non compress time
// K      Cramp rate
// k      Cramp step size
// W      Waveform type, master channel
// w      Waveform type, slave channel
// s      Stop the clock
// r      Restart the clock
// o      Sets the switch open time
// M      Set compressor normal amplitude mode
// [      Compressor loop start
// ]      Compressor loop end
//
// Added Dec 6, 2018
//
// L      Channel 3 output voltage p-p
// l      Channel 4 output voltage p-p
// B      Channel 1 output ramp rate in v/s
// b      Channel 2 output ramp rate in v/s
// E      Channel 3 output ramp rate in v/s
// e      Channel 4 output ramp rate in v/s
//
// Added Jan 18, 2019
//
// m      Mode, channel, N or C
// J      Compression order, channel order
//
char ARBgetNextOperationFromTable(bool init)
{
  bool   valfound;
  char   portCH,c;
  int    b;
  bool   CE;
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
      valfound = ARBgetValueFromTable(&tblindex, &count);
      break;
    }
    if((OP=='N')||(OP=='C'))
    {
      // Only return valid options, N for non compressed and C for compressed
      count--;
      return(OP);
    }
    else if(OP == 'D') //Delay
    {
      C_Delay = (((float)count) / 1000.0) * C_clock;
      count = 0;
      return(OP);
    }
    else if(OP == 'g') //Time to open gate
    {
      if(valfound)
      {
        CompressorSelectedSwitch = ARBarray[0]->Cswitch;
        CompressorSelectedSwitchLevel = ARBarray[0]->CswitchLevel;
      }
      else
      {
        CompressorSelectedSwitch = TwaveCompressorTable[tblindex++];
        CompressorSelectedSwitchLevel = HIGH;
        ARBgetValueFromTable(&tblindex, &count);
      }
      C_GateOpenTime = (((float)count) / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_OPEN_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    else if(OP == 'G') //Time to close gate
    {
      if(valfound)
      {
        CompressorSelectedSwitch = ARBarray[0]->Cswitch;
        CompressorSelectedSwitchLevel = ARBarray[0]->CswitchLevel;
      }
      else
      {
        CompressorSelectedSwitch = TwaveCompressorTable[tblindex++];
        CompressorSelectedSwitchLevel = HIGH;
        ARBgetValueFromTable(&tblindex, &count);
      }
      C_GateOpenTime = (((float)count) / 1000.0) * C_clock;
      count = 0;
      CSState = CSS_CLOSE_REQUEST;
      CompressorTimer.setTIOBeffect(C_GateOpenTime,TC_CMR_BCPB_TOGGLE);
    }
    else if(OP == 'S')
    {
      // This command has two options, if S is followed by a value then use the switch bit
      // defined in the ARB module data structure and the value defines open or close. If
      // S is followed by a character then it defines a port bit to set as per the 
      // value following the character
      if(valfound)
      {
        if(count == 0) ClearOutput(ARBarray[0]->Cswitch,ARBarray[0]->CswitchLevel);
        if(count == 1) SetOutput(ARBarray[0]->Cswitch,ARBarray[0]->CswitchLevel);        
      }
      else
      {
        portCH = TwaveCompressorTable[tblindex++];    // Get port charaster
        ARBgetValueFromTable(&tblindex, &count);      // Get desired state
        if(count == 0) ClearOutput(portCH,HIGH);
        if(count == 1) SetOutput(portCH,HIGH);                
      }
      count = 0;
    }
    else if(OP == 'O')
    {
      if((count >= 0) && (count <= 65535))
      {
         ARBarray[0]->Corder = count;
         if(AcquireTWI()) ARBCsetOrder(); else TWIqueue(ARBCsetOrder);
      }
      count = 0;
    }
    else if(OP == 'V')
    {
      if((count >= 0) && (count <= 100))
      {
        ARBarray[0]->Voltage = count; 
        if(AcquireTWI()) SetFloat(0,TWI_SET_RANGE, ARBarray[0]->Voltage); else TWIqueue(SetFloat,0,TWI_SET_RANGE, ARBarray[0]->Voltage);
      }   
      count = 0;
    }
    else if(OP == 'v')
    {
      if((count >= 0) && (count <= 100))
      {
        ARBarray[1]->Voltage = count;
        if(AcquireTWI()) SetFloat(1,TWI_SET_RANGE, ARBarray[1]->Voltage); else TWIqueue(SetFloat,1,TWI_SET_RANGE, ARBarray[1]->Voltage);
      }      
      count = 0;
    }
    else if(OP == 'L')
    {
      if((count >= 0) && (count <= 100) && (ARBarray[2] != NULL))
      {
         ARBarray[2]->Voltage = count;
         if(AcquireTWI()) SetFloat(2,TWI_SET_RANGE, ARBarray[2]->Voltage); else TWIqueue(SetFloat,2,TWI_SET_RANGE, ARBarray[2]->Voltage);
      }      
      count = 0;
    }
    else if(OP == 'l')
    {
      if((count >= 0) && (count <= 100) && (ARBarray[3] != NULL))
      {
         ARBarray[3]->Voltage = count;
         if(AcquireTWI()) SetFloat(3,TWI_SET_RANGE, ARBarray[3]->Voltage); else TWIqueue(SetFloat,3,TWI_SET_RANGE, ARBarray[3]->Voltage);
      }      
      count = 0;
    }
    else if(OP == 'B')
    {
      if((count >= 0) && (count <= 10000) && (ARBarray[0] != NULL))
      {
         ARBarray[0]->RampRate = count;
         if(AcquireTWI()) SetFloat(0, TWI_SET_RAMP, ARBarray[0]->RampRate); else TWIqueue(SetFloat,0, TWI_SET_RAMP, ARBarray[0]->RampRate);
      }      
      count = 0;
    }
    else if(OP == 'b')
    {
      if((count >= 0) && (count <= 10000) && (ARBarray[1] != NULL))
      {
         ARBarray[1]->RampRate = count;
         if(AcquireTWI()) SetFloat(1, TWI_SET_RAMP, ARBarray[1]->RampRate); else TWIqueue(SetFloat,1, TWI_SET_RAMP, ARBarray[1]->RampRate);
      }      
      count = 0;
    }
    else if(OP == 'E')
    {
      if((count >= 0) && (count <= 10000) && (ARBarray[2] != NULL))
      {
         ARBarray[2]->RampRate = count;
         if(AcquireTWI()) SetFloat(2, TWI_SET_RAMP, ARBarray[2]->RampRate); else TWIqueue(SetFloat,2, TWI_SET_RAMP, ARBarray[2]->RampRate);
      }      
      count = 0;
    }
    else if(OP == 'e')
    {
      if((count >= 0) && (count <= 10000) && (ARBarray[3] != NULL))
      {
         ARBarray[3]->RampRate = count;
         if(AcquireTWI()) SetFloat(3, TWI_SET_RAMP, ARBarray[3]->RampRate); else TWIqueue(SetFloat,3, TWI_SET_RAMP, ARBarray[3]->RampRate);
      }      
      count = 0;
    }
    else if(OP == 'F')
    {
      if((count > 100) && (count <= (MAXARBRATE / ARBarray[0]->PPP)))
      {
         ARBarray[0]->Frequency = count;
         SetARBcommonClock(ARBarray[0], count);
      }   
      count = 0;
    }
    else if(OP == 'c')
    {
      arb.Tcompress = ARBarray[0]->Tcompress = count;
      C_Tc  = (ARBarray[0]->Tcompress / 1000.0) * C_clock;
      count = 0;
    }
    else if(OP == 'n')
    {
      arb.Tnormal = ARBarray[0]->Tnormal = count;
      C_Tn  = (ARBarray[0]->Tnormal / 1000.0) * C_clock;
      count = 0;
    }
    else if(OP == 't')
    {
      arb.TnoC = ARBarray[0]->TnoC = count;
      C_Tnc = (ARBarray[0]->TnoC / 1000.0) * C_clock;
      count = 0;
    }
    else if(OP == 'K') // Set the Cramp rate
    {
      arb.Cramp = ARBarray[0]->Cramp = count;
      if(AcquireTWI()) ARBCsetCramp(); else TWIqueue(ARBCsetCramp);
      count = 0;
    }
    else if(OP == 'k')
    {
      // Set the Cramp step size, or cramp order
      arb.CrampOrder = ARBarray[0]->CrampOrder = count;
      if(AcquireTWI()) ARBCsetCrampOrder(); else TWIqueue(ARBCsetCrampOrder); 
      count = 0;
    }
    else if(OP == 'W')
    {
      if(count < 1) count = 1;
      if(count > 5) count = 5;
      ARBarray[0]->wft = (WaveFormTypes)(count - 1);
      if(AcquireTWI()) ARBsetWFT1(); else TWIqueue(ARBsetWFT1);
    }
    else if(OP == 'w')
    {
      if(count < 1) count = 1;
      if(count > 5) count = 5;
      ARBarray[1]->wft = (WaveFormTypes)(count - 1);
      if(AcquireTWI()) ARBsetWFT2(); else TWIqueue(ARBsetWFT2); 
    }
    else if(OP == 'm')   // Set selected ARB mode to compress or normal
    {
       // syntax is m followed by ARB channel followed by N or C
      if(valfound)
      {
         c = TwaveCompressorTable[tblindex++];
         if((count>=1) && (count<=4) && ((c=='N') || (c=='C')))
         {
            if(c=='C') CE = true; else CE = false;
            if(AcquireTWI()) SetBool(count - 1, TWI_SET_COMP_ENA, CE); else TWIqueue(SetBool,count - 1, TWI_SET_COMP_ENA, CE);
         }
      }
    }
    else if(OP == 'J')   // Set selected ARB compression order
    {
       // Syntax is J followed by ARB channel (1 thru 4) followed by order
       b = count;
       int m = 1;
       while(b > 4) { b /= 10; m *= 10; }
       count -= m * b;
       if((b>=1) && (b<=4))
       {
          if(AcquireTWI()) SetWord(b-1,TWI_SET_COMP_ORDER_EX, count); else TWIqueue(SetWord,b-1,TWI_SET_COMP_ORDER_EX, count);
       }
    }
    else if(OP == 's') ARBclock->stop();                            // Stop the clock
    else if(OP == 'r') ARBclock->start(-1, 0, true);                // Restart the clock
    else if(OP == 'o') C_SwitchTime = (count / 1000.0) * C_clock;   // Sets the switch open time
    else if(OP == 'M') C_NormAmpMode = count;                       // Set compressor normal amplitude mode
    else if(OP == '[') CompressorLoopStart(tblindex);
    else if(OP == ']')
    {
      int i = CompressorProcessLoop(count);
      if(i != -1) tblindex = i;
    }    
  }
  count = 0;
}

// Compressor init function, called on startup
void ARBcompressor_init(void)
{
  int            i;
  DialogBoxEntry *de;
  
  if(!ARBarray[0]->CompressorEnabled) return;  // Exit if the compressor is not enabled
//  ARBarray[0]->UseCommonClock = true;          // If we are in compressor mode then we must use a common clock
//  ARBarray[1]->UseCommonClock = true;
  // Enable the compressor hardware mode contol line in the compress ARB module
  pinMode(ARBmode,OUTPUT);
  SetBool(0,TWI_SET_COMP_EXT,false);
  SetBool(1,TWI_SET_COMP_EXT,false);
  SetBool(CompressBoard,TWI_SET_COMP_EXT,true);
  // Clear the switch output
  ClearOutput(ARBarray[0]->Cswitch,ARBarray[0]->CswitchLevel);
  // Setup the trigger line and ISR
  CtrigInput = new DIhandler;
  ARBupdateMode();
  ARBupdateCorder();
  ARBsetSwitch();
}

// Compressor polling loop, called every 100 mS when the compressor is enabled.
void ARBcompressor_loop(void)
{
  static int init = -1;
  static int CurrentCramp = -1000;
  static int CurrentCrampOrder = -1;

  if(init == -1)
  {
    CtrigInput->detach();
    CtrigInput->setPriority(ARBarray[0]->Ctrig,0);
    CtrigInput->attached(ARBarray[0]->Ctrig, ARBarray[0]->CtrigLevel, ARBcompressorTriggerISR);
    init = 0;
  }
  if(!ARBarray[0]->CompressorEnabled) return;  // Exit if the compressor is not enabled
  // Calculate all the times in clock count units
  C_Td  = (ARBarray[0]->Tdelay / 1000.0) * C_clock;
  C_Tc  = (ARBarray[0]->Tcompress / 1000.0) * C_clock;
  C_Tn  = (ARBarray[0]->Tnormal / 1000.0) * C_clock;
  C_Tnc = (ARBarray[0]->TnoC / 1000.0) * C_clock;
  if (ActiveDialog == &ARBCompressorDialog) RefreshAllDialogEntries(&ARBCompressorDialog);  
  if(CurrentCramp != ARBarray[0]->Cramp)
  {
    CurrentCramp = ARBarray[0]->Cramp;
    Set16bitInt(CompressBoard, TWI_SET_CRAMP, CurrentCramp);
  }
  if(CurrentCrampOrder != ARBarray[0]->CrampOrder)
  {
    CurrentCrampOrder = ARBarray[0]->CrampOrder;
    Set16bitInt(CompressBoard, TWI_SET_CRAMPORDER, CurrentCrampOrder);
  }
}

//
// ARB compressor host command functions
//

void SetARBCompressorEnabled(char *flag)
{
  String smode;

  smode = flag;
  if(ARBarray[0] == NULL)
  {
    SetErrorCode(ERR_INTERNAL);
    SendNAK;       
    return;    
  }
  if((smode == String("TRUE")) || (smode == String("FALSE")))
  {
     if(smode == String("TRUE")) ARBarray[0]->CompressorEnabled = true;
     else  ARBarray[0]->CompressorEnabled = false;
     SendACK;
     return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;       
}

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
  if (!SerialMute) serial->println(ARBarray[0]->Corder);
}

void SetARBCorder(int ival)
{
  if(RangeTest(ARBCompressorEntries,"Order",ival))
  {
    ARBarray[0]->Corder = ival;
    arb.Corder = ival;
    ARBupdateCorder();
    SendACK;
  }
}

void GetARBCtriggerDelay(void)
{
  if(!IsARBmodule(0)) return;
  SendACKonly;
  if (!SerialMute) serial->println(ARBarray[0]->Tdelay);
}

void SetARBCtriggerDelay(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Trig delay, mS",fval))
  {
    ARBarray[0]->Tdelay = fval;
    arb.Tdelay = fval;
    SendACK;
  }
}

void GetARBCcompressTime(void)
{
  if(!IsARBmodule(0)) return;
  SendACKonly;
  if (!SerialMute) serial->println(ARBarray[0]->Tcompress);
}

void SetARBCcompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Compress t, mS",fval))
  {
    ARBarray[0]->Tcompress = fval;
    arb.Tcompress = fval;
    SendACK;
  }
}

void GetARBCnormalTime(void)
{
  if(!IsARBmodule(0)) return;
  SendACKonly;
  if (!SerialMute) serial->println(ARBarray[0]->Tnormal);
}

void SetARBCnormalTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Normal t, mS",fval))
  {
    ARBarray[0]->Tnormal = fval;
    arb.Tnormal = fval;
    SendACK;
  }
}

void GetARBCnoncompressTime(void)
{
  if(!IsARBmodule(0)) return;
  SendACKonly;
  if (!SerialMute) serial->println(ARBarray[0]->TnoC);
}

void SetARBCnoncompressTime(char *str)
{
  float fval;

  fval = strtof(str,NULL);
  if(RangeTest(ARBCompressorEntries,"Normal t, mS",fval))
  {
    ARBarray[0]->TnoC = fval;
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
