//
// File: DCbiasList
//
// Segment processing code. Segment operation protocol. The first segment is started by
// software. Each segment can generate a digital trigger at a user defined time. This trigger
// goes to an external device, the external device then generates a trigger to start the next 
// segment in the list of segments.
// After all the voltage states and segments are defined then the segment processing
// needs to be enabled. This enable requires defining the clock rate and setting up the timer. 
// Segment processing uses the same timer as the table processing code.
//
// Segments consist of time points and at a given time point several DC bias voltage states can
// be applied. Segment processing code sets up all the DACs and this can require more than one
// states to be setup before a time point causes a LDAC trigger to fire and latch the DAC values
// thus outputing them.
//
// The timer is setup to stop at the end of a segment and then an external trigger will start the
// timer for the next segment. The external trigger is disabled at the end of ther terminal 
// segment.
//
// The segment's maximum time needs to be defined and the trigger can not arrive before this maximum
// time. 
//
// Clock is internal and set at 10,500,000 Hz.
// External trigger is R input and set at positive edge.
//
// DSTATE,state1,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8
// DSTATE,state2,1,10,2,20,3,30,4,40,5,50,6,60
// DSTATE,state3,1,-1,2,-2,3,-3,4,-4,5,-5,6,-6,7,-7,8,-8
// DSTATE,state4,1,-10,2,-20,3,-30,4,-40,5,-50,6,-60,7,-70
// DSTATE,state5,1,11,2,22,3,33
//
// LSTATES
//
// DSEGMENT,seg1,10000
// DSEGMENT,seg2,10000
// DSEGMENT,seg3,10000
//
// ADDSEGTP,seg1,2000,state1,state2
// ADDSEGTP,seg1,3000,state1
// ADDSEGTP,seg2,3000,state2
//
// LSEGMENTS
//
// Gordon Anderson
// June, 2017
//
#include "DCbias.h"
#include "DCbiasList.h"

// States (modules)
DCstate           *DCstateList = NULL;
DCstate           *CurrentState;
uint8_t           CurrentDCbiasModule;
volatile bool     DCbiasStateBusy = false;
volatile bool     StriggerReady = false;
volatile bool     SegmentsAbort = false;

// Sequences
DCsegment           *DCsegmentList = NULL;
volatile DCsegment  *CurrentSegment = NULL;
uint8_t             CurrentTimePoint = 0;

MIPStimer MPST(TMR_Table);               // timer used for segment generation
#define   MPSTtc   TC2->TC_CHANNEL[2]    // Pointer to the timers control block

// 
// Segment real-time processing code
//

// Enabled for S input triggering of segment
void TriggerSegmentISR(void)
{
  if(!StriggerReady) return;
  MPST.softwareTrigger();
}

void RAmatchISR(void)
{
  SetupNextTimePoint();
}

void SetupTriggerSource(void)
{
     // Setup this segment starting trigger
     switch (CurrentSegment->TriggerSource)
     {
       case 0:    // Software trigger, via command
         detachInterrupt(DI2);
         MPST.setTrigger(TC_CMR_EEVTEDG_NONE);
         break;
       case 'Q':  // Q input trigger source
         detachInterrupt(DI2);
         if(CurrentSegment->TriggerEdge == POS) MPST.setTriggerQ(TC_CMR_EEVTEDG_RISING);
         if(CurrentSegment->TriggerEdge == NEG) MPST.setTriggerQ(TC_CMR_EEVTEDG_FALLING);
         break;
       case 'R':  // R input trigger source
         detachInterrupt(DI2);
         if(CurrentSegment->TriggerEdge == POS) MPST.setTrigger(TC_CMR_EEVTEDG_RISING);
         if(CurrentSegment->TriggerEdge == NEG) MPST.setTrigger(TC_CMR_EEVTEDG_FALLING);
         break;
       case 'S':  // S input trigger source, used ISR, not as deterministic
         MPST.setTrigger(TC_CMR_EEVTEDG_NONE);
         if(CurrentSegment->TriggerEdge == POS) attachInterrupt(DI2, TriggerSegmentISR, RISING);
         if(CurrentSegment->TriggerEdge == NEG) attachInterrupt(DI2, TriggerSegmentISR, FALLING);
         break;
       default:
          break;
     }
}

void RCmatchISR(void)
{
   // If abort flag is set exit and report
   if(SegmentsAbort)
   {
      MPST.stop();
      CurrentSegment = NULL;
      return;
   }
   // Advance to next segment and clear the CurrentTimePoint
   CurrentTimePoint = 0;
   if(CurrentSegment->repeat == NULL) CurrentSegment = CurrentSegment->next;
   else
   {
     if(CurrentSegment->RepeatCount == 0) CurrentSegment = CurrentSegment->repeat;
     else
     {
       CurrentSegment->CurrentCount++;
       if(CurrentSegment->CurrentCount < CurrentSegment->RepeatCount) CurrentSegment = CurrentSegment->repeat;
       else CurrentSegment = CurrentSegment->next;
     }
   }
   // If the CurrentSegment pointer is NULL we are done so stop the timer and exit
   if(CurrentSegment == NULL)
   {
     // Stop timer, clean up, and exit
     MPST.stop();
   }
   else 
   {
     StriggerReady=false;
     SetupTriggerSource();
     // Setup the next time point
     SetupNextTimePoint();
     StriggerReady=true;
   }
}

void PlaySegments(void)
{
  Spi* pSpi = SPI0;
    
  // Setup the pointers and counters
  CurrentSegment = DCsegmentList;
  CurrentTimePoint = 0;
  if(CurrentSegment ==  NULL) return;
  // Clear all the current counts
  do
  {
    CurrentSegment->CurrentCount = 0;
    CurrentSegment = CurrentSegment->next;
  } while(CurrentSegment != NULL);
  CurrentSegment = DCsegmentList;
  // Set up the DAC spi DMA
  spiDMAinit();
  // Init the SPI hardware for the DMA transfer
  SPI.begin(10);                   // Dummy DMA SPI device, needed after each xfer to trigger strobe
  SPI.setClockDivider(10,1);       // Set the dummy at max speed
  SPI.setClockDivider(SPI_CS,4);   // 21 MHz clock rate for DAC SPI speed
  SPI.setDataMode(SPI_CS, SPI_MODE1);
  pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] &= 0x00FF0F;  // Set DLYBCT delay between bytes to zero
  pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] |= 8 << 4;    // Set 16 bit transfer mode
  pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(10)] &= 0xFFFFFF;
  // Setup the timer and start the first segment. The timer will be set for external trigger and stop
  // on maximum count in RC. Clock is fixed at 10.5Mhz giving roughly 1uS per 11 counts. External trigger
  // on R input and also fixed internal clock.
  MPST.detachInterrupt();
  MPST.begin(); 
  MPST.setPriority(15);
  MPST.setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);
  SetupTriggerSource();
  MPST.attachInterruptRA(RAmatchISR);
  MPST.attachInterrupt(RCmatchISR);
  LDACrelease;
  MPST.setTIOAeffect(1000,TC_CMR_ACPA_TOGGLE);
  MPST.stopOnRC(); 
  SegmentsAbort = false;
  SetupNextTimePoint();
  MPST.enableTrigger();
  if(CurrentSegment->TriggerSource == 0) MPST.softwareTrigger();
  serial->println("Waiting for segments to complete.");
  while(CurrentSegment != NULL)
  {
    ProcessSerial();
    WDT_Restart(WDT);
  }
  if(SegmentsAbort) serial->println("Aborted!");
  serial->println("Segments complete.");
  SegmentsAbort = false;
}

void DACsetup(void)
{
  static Pio *pio = g_APinDescription[ADDR0].pPort;
  int  i,j;
  
  for(i=0;i<CurrentSegment->TimePoint[CurrentTimePoint]->NumStates;i++)
  {
     // Find the first module and setup 
     for(j=0;j<4;j++) if(CurrentSegment->TimePoint[CurrentTimePoint]->States[i]->md[j].Count > 0) break;
     if(j==4) return; // Nothing to do
     // Set the state variables
     CurrentState = CurrentSegment->TimePoint[CurrentTimePoint]->States[i];
     CurrentDCbiasModule = j;
     DCbiasStateBusy = true;
     // Start first transfer, set address and also board select
     pio->PIO_CODR = 7;                                  // Set all bits low
     pio->PIO_SODR = CurrentState->md[i].Address & 7;    // Set bit high
     SelectBoard((CurrentState->md[i].Address & 0x80) >> 7);    // Added 2/14/18
     spiDmaTX(CurrentState->md[j].data, CurrentState->md[j].Count * 3, NextBufferISR);
     if((i+1) >= CurrentSegment->TimePoint[CurrentTimePoint]->NumStates) break;
     while(DCbiasStateBusy);
  }  
}

void DIOsetup(void)
{
  static Pio *pio = g_APinDescription[ADDR0].pPort;
  int  i,j;
  Spi* pSpi = SPI0;
  uint32_t   d;

    pio->PIO_CODR = 7;    // Set all bits low
    pio->PIO_SODR = 6;    // Set bit high
     
    while ((pSpi->SPI_SR & SPI_SR_TDRE) == 0);
    pSpi->SPI_TDR = CurrentSegment->TimePoint[CurrentTimePoint]->DIOimage;

    // return SPI_Read(spi);
    while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0);
    d = pSpi->SPI_RDR;
}

// This function uses the CurrentSegment pointer and the CurrentTimePoint to setup the system to
// output the voltages at the next defined time point defined by the counter. This function assumes
// parameters are valid and no testing is done in the intrest of performance.
void SetupNextTimePoint(void)
{
  if(CurrentSegment->NumTimePoints > CurrentTimePoint)
  {
    // Apply count to timer compare register
    MPSTtc.TC_RA = CurrentSegment->TimePoint[CurrentTimePoint]->Count;
    MPSTtc.TC_RC = CurrentSegment->Length;
    CurrentSegment->TimePoint[CurrentTimePoint]->Setup();
    // Advance pointer to next time point
    CurrentTimePoint++;
  }
}

// End Segment real-time processing code

// Delete a state entry
void DeleteEntry(DCstate *dcs)
{
   int i;
   
   for(i=0;i<4;i++) if(dcs->md[i].data != NULL) delete [] dcs->md[i].data;
   delete dcs;
}

// Delete a segment entry
void DeleteEntry(DCsegment *dcs)
{
   int i,j;

   // Delete all of the state pointers in each time point
   for(i=0;i<dcs->NumTimePoints;i++)
   {
      free(dcs->TimePoint[i]->States);
   }
   free(dcs->TimePoint);
   delete dcs;
}

// This function removes a named entry from the list
void RemoveFromList(DCstate *list, char *name)
{
   DCstate *next, *last;
   
   if(list == NULL) return;
   if(strcmp(name,list->name)==0)
   {
      last = list;
      list = list->next;
      DeleteEntry(last);
      return; 
   }
   last = DCstateList;
   next = DCstateList->next;
   while(1)
   {
      if(strcmp(name,next->name)==0)
      {
         last->next = next->next;
         DeleteEntry(next);
         return;
      }
      last = next;
      next = next->next;
      if(next == NULL) return;
   }
}

void RemoveFromList(DCsegment *list, char *name)
{
   DCsegment *next, *last;
   
   if(list == NULL) return;
   if(strcmp(name,list->name)==0)
   {
      last = list;
      list = list->next;
      DeleteEntry(last);
      return; 
   }
   last = DCsegmentList;
   next = DCsegmentList->next;
   while(1)
   {
      if(strcmp(name,next->name)==0)
      {
         last->next = next->next;
         DeleteEntry(next);
         return;
      }
      last = next;
      next = next->next;
      if(next == NULL) return;
   }
}

// Return the end of list next pointer, used to add an entry to the 
// end of list.
DCstate** EndOfList(DCstate *list)
{
   DCstate *next;
   
   if(list == NULL) return(&DCstateList);
   next = list;
   while(1)
   {
      if(next->next == NULL) return(&next->next);
      next = next->next;
   }
   return(&DCstateList);  // Should never get here but just in case
}

DCsegment** EndOfList(DCsegment *list)
{
   DCsegment *next;
   
   if(list == NULL) return(&DCsegmentList);
   next = list;
   while(1)
   {
      if(next->next == NULL) return(&next->next);
      next = next->next;
   }
   return(&DCsegmentList);  // Should never get here but just in case
}

// This function finds a named entry in the last and returns the pointer,
// or NULL if not found
DCstate* FindInList(DCstate *list, char *name)
{
   DCstate *next;
   
   if(list == NULL) return(NULL);
   next = list;
   while(1)
   {
      if(strcmp(name,next->name)==0) return(next);
      next = next->next;
      if(next == NULL) return(NULL);
   }
   return(NULL);
}

DCsegment* FindInList(DCsegment *list, char *name)
{
   DCsegment *next;
   
   if(list == NULL) return(NULL);
   next = list;
   while(1)
   {
      if(strcmp(name,next->name)==0) return(next);
      next = next->next;
      if(next == NULL) return(NULL);
   }
   return(NULL);
}

// This interrupt service routine fires when DMA buffer transfer completes,
// the new module transfer is started.
void NextBufferISR(void)
{
   int i;

i = DMAC->DMAC_EBCISR;
spiDmaWait();
DMAC->DMAC_EBCIER = 0;
   // Get the next module and start transfer
   for(i=CurrentDCbiasModule+1;i<4;i++) if(CurrentState->md[i].Count > 0) break;
   if((i==4) || (CurrentDCbiasModule >= 3))
   {
      // Nothing to do, clear flag and exit
      DCbiasStateBusy = false;
      // All data sent to DACs, ready for a LDAC signal
      dmac_channel_disable(SPI_DMAC_TX_CH);
      return;
   }
   // Start transfer
   CurrentDCbiasModule = i;
   SetAddress(CurrentState->md[i].Address);
   SelectBoard((CurrentState->md[i].Address & 0x80) >> 7);    // Added 2/14/18
   spiDmaTX(CurrentState->md[i].data, CurrentState->md[i].Count * 3, NextBufferISR);
}

// This function performes the DMA transfer to the DCbias module DACs.
// The DCstate data structure passed by reference has all the formatted
// DMA buffers. Each module needs a seperate DMA block transfer. After 
// each block is complete an interrupt first to enable the next block.
// A global flag provides status information.
void SetDBbiasState(DCstate *dcs)
{
    int i;
    Spi* pSpi = SPI0;
    static bool inited = false;

    if(!inited)
    {
      spiDMAinit();
      inited = true;
    }
    // Find the first module and setup 
    for(i=0;i<4;i++) if(dcs->md[i].Count > 0) break;
    if(i==4) return; // Nothing to do
    // Set the state variables
    CurrentState = dcs;
    CurrentDCbiasModule = i;
    DCbiasStateBusy = true;

    // Init the SPI hardware for the DMA transfer
    SPI.begin(10);                   // Dummy DMA SPI device, needed after each xfer to trigger strobe
    SPI.setClockDivider(10,1);       // Set the dummy at max speed
    SPI.setClockDivider(SPI_CS,4);   // 21 MHz clock rate for DAC SPI speed
    SPI.setDataMode(SPI_CS, SPI_MODE1);
    pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] &= 0x00FF0F;  // Set DLYBCT delay between bytes to zero
    pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] |= 8 << 4;    // Set 16 bit transfer mode
    pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(10)] &= 0xFFFFFF;

    // Start first transfer, set address and also board select
    SetAddress(CurrentState->md[i].Address);
    SelectBoard((CurrentState->md[i].Address & 0x80) >> 7);    // Added 2/14/18
    spiDmaTX(CurrentState->md[i].data, CurrentState->md[i].Count * 3, NextBufferISR);
}

//
// Host commands supporting the DCbias linked list system of segments and states.
//

// DC bias states functions:
// DSTATE,name,ch,val,ch,val...     Defines a voltage state in linked list
//                                  Name followed by channel value pairs, variable length
// LSTATES                          List all the states in linked list
// SSTATE,name                      Sets all the DACs for a defined state
// RSTATE,name                      Removes a state
// RSTATES                          Removes all states, clears system
// GSTATE,name                      Returns TRUE is state is defined, else FALSE

// This function is called with the full command line, all the arguments,
// in the input ring buffer. This function will read the name and then all
// the channel value pair data. This data will be used to insert a DCbias
// state structure in the linked list.
// This function will exit with an error if the name is already present,
// or any data errors are found reading the channel value pairs.
// Error codes:
//        0                =  OK, no errors
//        ERR_CANTALLOCATE =  Memory allocation error
//        ERR_BADARG       =  Command parsing error
//        ERR_NAMEINLIST   =  Name already in linked list
int ReadDCbiasState(void)
{
   DCstate  *dcs;
   char     *Token;
   String   sToken;
   int      i,module,num,chans[8];
   uint32_t iVal;
   float    vals[8];
   
   dcs = new DCstate;
   if(dcs == NULL) return ERR_CANTALLOCATE;
   // Init the structure
   dcs->next = NULL;
   for(i=0;i<4;i++)
   {
      if((i==0) || (i==1)) dcs->md[i].Address = 2;
      else dcs->md[i].Address = 0;
      // Use Address MSB to define the board address
      if((i==1) || (i==3)) dcs->md[i].Address |= 0x80;    // Added 2/14/18
      dcs->md[i].Count = 0;
      dcs->md[i].data = NULL;
   }
   // Get name
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   strcpy(dcs->name,Token);
   // See if this entry is in list, if so exit with error
   if(FindInList(DCstateList,Token) != NULL) return ERR_NAMEINLIST;
   // Read channel value pairs
   GetToken(true); if((Token = GetToken(true)) == NULL)return ERR_BADARG;
   sToken = Token;
   while(1)   // Process each DCbias card (this is the module, 0,1,2, and 3
   {
      num = 0;
      chans[num] = sToken.toInt() - 1;
      GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
      sToken = Token;
      vals[num] = sToken.toFloat();
      module = (int)(chans[num] / 8);
      for(num=1;num<=8;num++)
      {
         GetToken(true); 
         if(strcmp(Token,"\n")==0) break;
         if((Token = GetToken(true)) == NULL) return ERR_BADARG;
         sToken = Token;
         if(((int)((sToken.toInt() - 1) / 8)) != module) break;
         chans[num] = sToken.toInt() - 1;
         GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
         sToken = Token;
         vals[num] = sToken.toFloat();
      }
      // Allocate space for this module and save
      dcs->md[module].Count = num;
      dcs->md[module].data = new uint32_t [num * 3];
      if(dcs->md[module].data == NULL)
      {
         // Delete this structure and exit
         DeleteEntry(dcs);
         return ERR_CANTALLOCATE;
      }
      // Convert the voltage values to DAC control word and save
      for(i=0;i<num;i++) 
      {
         iVal = DCbiasValue2Counts(chans[i], vals[i]);
         // Now convert into control words, 3 per entry 
         dcs->md[module].data[i*3+0] = ((0x00 << 8) & 0xF00) | ((chans[i] << 4) & 0x70) | ((iVal >> 12) & 0x0F) \
                                       | SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(SPI_CS));
         dcs->md[module].data[i*3+1] = ((iVal << 4) & 0xFFFF) \
                                       | SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(SPI_CS));
         dcs->md[module].data[i*3+2] = SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(10));
      }
      dcs->md[module].data[i*3-1] |= SPI_TDR_LASTXFER;
      // Exit on EOL
      if(strcmp(Token,"\n")==0) break;
   }
   // Here with a defined data structure, insert in linked list and exit
   // Find end of list and append
   DCstate **eol = EndOfList(DCstateList);
   *eol = dcs;
   return 0;
}

// Called with a full command line in the input ringbuffer.
void DefineState(void)
{
   int  status;
   char *Token;
   
   status = ReadDCbiasState();
   if(status == 0)
   {
      SendACK;
      return;
   }
   // If here there was an error, get the last Token and if it was not "\n"
   // then read tokens until we get a "\n"
   Token = LastToken();
   while(strcmp(Token,"\n") != 0) Token = GetToken(true);
   SetErrorCode(status);
   SendNAK;
   return;
}

// This function will set the DCbias output to the defined state is found in the list
void SetState(char *name)
{
   Spi* pSpi = SPI0;
   DCstate *dcs;
   
   if((dcs=FindInList(DCstateList, name)) == NULL)
   {
      // Not in list!
      SetErrorCode(ERR_NAMENOTFOUND);
      SendNAK;
      return;
   }
   SetDBbiasState(dcs);
   // Wait for the busy flag to clear and then pulse LDAC to update values
   while(1)
   {
    if(!DCbiasStateBusy) break;
   }
   pSpi->SPI_CSR[BOARD_PIN_TO_SPI_CHANNEL(SPI_CS)] &= 0x00FF0F;
   SPI.setClockDivider(SPI_CS,21);   // 21 MHz clock rate for DAC SPI speed
   
   // Pluse LDAC
   LDAClow;
   LDAChigh;

   SendACK;
}

// This command will list all the state names in the linked list
void ListStateNames(void)
{
   DCstate *dcs;
   
   SendACKonly;
   dcs = DCstateList;
   if(DCstateList == NULL) serial->println("List is empty!");
   else
   {
      do
      {
         serial->println(dcs->name);
         dcs = dcs->next;
      } while(dcs != NULL);
   }
}

// This command will remove a state from the linked list
void RemoveState(char *name)
{
   if(FindInList(DCstateList, name) == NULL)
   {
      // Not in list!
      SetErrorCode(ERR_NAMENOTFOUND);
      SendNAK;
      return;
   }
   RemoveFromList(DCstateList, name);
   SendACK;
}

// Deletes the entire linked list
void RemoveStates(void)
{
   DCstate *dcs;
   
   while(DCstateList != NULL)
   {
      dcs = DCstateList;
      DeleteEntry(dcs); 
      DCstateList = DCstateList->next;
   }
   SendACK;
}

// Returns true is name is found in list, else false
void IsState(char *name)
{
   SendACKonly;
   if(FindInList(DCstateList, name) == NULL) serial->println("FALSE");
   else serial->println("TRUE");
}

// DC bias segment functions:
//  DSEGMENT,name,length,next,repeat count
//  ADDSEGTP,name, count, state1, state2, ... 
//  ASEGTPTRG,name,bit,level   bit = A thru P, level = 0 or 1
//  LSEGMENTS
//  RSEGMENT,name
//  RSEGMENTS

// Error codes:
//        0  =  OK, no errors
//        ERR_CANTALLOCATE =  Memory allocation error
//        ERR_BADARG       =  Command parsing error
//        ERR_NAMEINLIST   =  Name already in linked list
//        ERR_NAMENOTFOUND =  Name not found
int ReadSegment(void)
{
   DCsegment *dcs;
   char      *Token;
   String    sToken;
   
   dcs = new DCsegment;
   if(dcs == NULL) return ERR_CANTALLOCATE;
   // Init the structure
   dcs->TriggerSource = 'R';
   dcs->TriggerEdge = POS;
   dcs->next = NULL;
   dcs->repeat = NULL;
   dcs->RepeatCount = 0;
   dcs->CurrentCount = 0;
   dcs->NumTimePoints = 0;
   dcs->TimePoint = NULL;
   // Get name
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   strcpy(dcs->name,Token);
   // See if this entry is in list, if so exit with error
   if(FindInList(DCsegmentList,Token) != NULL) return ERR_NAMEINLIST;
   // Get the segment maximum length in counts
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   dcs->Length = sToken.toInt();
   // If next token is end of line we are done else next and repeat count are expected
   if((Token = GetToken(true)) == NULL)return ERR_BADARG;
   sToken = Token;
   if(sToken == ",")
   {
      // Here if we expect next and repeat count
      if((Token = GetToken(true)) == NULL) return ERR_BADARG;  // next
      // Find the named segment and error out if not found
      if((dcs->repeat = FindInList(DCsegmentList, Token)) == NULL) return ERR_NAMENOTFOUND;     
      GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;  // repeat count
      sToken = Token;
      dcs->RepeatCount = sToken.toInt();
      GetToken(true);  // should be \n
   }
   // Here with a defined data structure, insert in linked list and exit
   // Find end of list and append
   DCsegment **eol = EndOfList(DCsegmentList);
   *eol = dcs;
   return 0;
}
// Defines a segment with the following arguments: name, next, repeat count
// next defines the the next segment and its only valid with a repeat count. next has to reference a
// defined segment. next and repeat count can be omited if not used. 
// This function is called with a full line in the input ring buffer.
void DefineSegment(void)
{
   int  status;
   char *Token;
   
   status = ReadSegment();
   if(status == 0)
   {
      SendACK;
      return;
   }
   // If here there was an error, get the last Token and if it was not "\n"
   // then read tokens until we get a "\n"
   Token = LastToken();
   while(strcmp(Token,"\n") != 0) Token = GetToken(true);
   SetErrorCode(status);
   SendNAK;
   return;  
}

// Error codes:
//        0  =  OK, no errors
//        ERR_CANTALLOCATE =  Memory allocation error
//        ERR_BADARG       =  Command parsing error
//        ERR_NAMEINLIST   =  Name already in linked list
//        ERR_NAMENOTFOUND =  Name not found
int ReadSegmentTimePoint(void)
{
   char           *Token;
   String         sToken;
   DCsegment      *dcs;
   DCstate        *dcState;
   DCsegmentTP    *dcsTP;

   // Find the named segment
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   if((dcs = FindInList(DCsegmentList, Token)) == NULL) return ERR_NAMENOTFOUND;
   // Read count
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   // Allocate the timepoint structure
   dcsTP = new DCsegmentTP;
   if(dcsTP == NULL) return ERR_CANTALLOCATE;
   dcsTP->Count = sToken.toInt();
   dcsTP->NumStates = 0;
   dcsTP->States = NULL;
   dcsTP->Setup = DACsetup;
   // Read states, check that they are defined, then build a list
   while(1)
   {
      Token = GetToken(true);
      if(strcmp(Token,"\n") == 0) break;  // Done at EOL
      if((Token = GetToken(true)) == NULL) return ERR_BADARG;  // Get the state name
      if((dcState = FindInList(DCstateList, Token)) == NULL) return ERR_NAMENOTFOUND;
      // Add the state to list of states, realloc to get space
      dcsTP->NumStates++;
      DCstate **temp = (DCstate **) realloc(dcsTP->States, dcsTP->NumStates * sizeof(DCstate *));
      dcsTP->States = temp;
      if(dcsTP->States == NULL) return(ERR_CANTALLOCATE);
      dcsTP->States[dcsTP->NumStates-1] = dcState;
   }
   // Make space for the new time point and insert it in the sequence structure
   dcs->NumTimePoints++;
   DCsegmentTP **temp = (DCsegmentTP **) realloc(dcs->TimePoint, dcs->NumTimePoints * sizeof(DCsegmentTP *));
   dcs->TimePoint = temp;
   if(dcs->TimePoint == NULL) return(ERR_CANTALLOCATE);
   dcs->TimePoint[dcs->NumTimePoints-1] = dcsTP;
   return(0);
}

// Adds a time point to a segment, arguments: name,count,state1,state 2... (variable number of states)
void AddSegmentTimePoint(void)
{
   int  status;
   char *Token;
   
   status = ReadSegmentTimePoint();
   if(status == 0)
   {
      SendACK;
      return;
   }
   // If here there was an error, get the last Token and if it was not "\n"
   // then read tokens until we get a "\n"
   Token = LastToken();
   while(strcmp(Token,"\n") != 0) Token = GetToken(true);
   SetErrorCode(status);
   SendNAK;
   return;    
}

// Error codes:
//        0  =  OK, no errors
//        ERR_CANTALLOCATE =  Memory allocation error
//        ERR_BADARG       =  Command parsing error
//        ERR_NAMEINLIST   =  Name already in linked list
//        ERR_NAMENOTFOUND =  Name not found
int ReadSegmentTrigger(void)
{
   uint16_t       mask;
   char           *Token;
   String         sToken;
   DCsegment      *dcs;
   DCsegmentTP    *dcsTP;

   // Find the named segment
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   if((dcs = FindInList(DCsegmentList, Token)) == NULL) return ERR_NAMENOTFOUND;
   // Read count
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   // Allocate the timepoint structure
   dcsTP = new DCsegmentTP;
   if(dcsTP == NULL) return ERR_CANTALLOCATE;
   dcsTP->Count = sToken.toInt();
   dcsTP->NumStates = 0;
   dcsTP->States = NULL;
   dcsTP->Setup = DIOsetup;
   // Read the port number and the level
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   if((Token[0] < 'A') || (Token[0] > 'P')) return ERR_BADARG;
   mask = (1 << (Token[0] - 'A'));
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   if((sToken.toInt() < 0) || (sToken.toInt() > 1)) return ERR_BADARG;
   dcsTP->DIOimage = (MIPSconfigData.DOmsb << 8) | MIPSconfigData.DOlsb;
   if(sToken.toInt() == 1) dcsTP->DIOimage |= mask;
   else dcsTP->DIOimage &= ~mask;
   dcsTP->DIOimage |= SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(SPI_CS));
   // Make space for the new time point and insert it in the sequence structure
   dcs->NumTimePoints++;
   DCsegmentTP **temp = (DCsegmentTP **) realloc(dcs->TimePoint, dcs->NumTimePoints * sizeof(DCsegmentTP *));
   dcs->TimePoint = temp;
   if(dcs->TimePoint == NULL) return(ERR_CANTALLOCATE);
   dcs->TimePoint[dcs->NumTimePoints-1] = dcsTP;
   return(0);
}

// This command defines a segments output trigger seginal generation
void AddSegmentTrigger(void)
{
   int  status;
   char *Token;
   
   status = ReadSegmentTrigger();
   if(status == 0)
   {
      SendACK;
      return;
   }
   // If here there was an error, get the last Token and if it was not "\n"
   // then read tokens until we get a "\n"
   Token = LastToken();
   while(strcmp(Token,"\n") != 0) Token = GetToken(true);
   SetErrorCode(status);
   SendNAK;
   return;    
}

int ReadSegmentTriggerSource(void)
{
   char           *Token,src;
   int            level;
   String         sToken;
   DCsegment      *dcs;

   // Find the named segment
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   if((dcs = FindInList(DCsegmentList, Token)) == NULL) return ERR_NAMENOTFOUND;
   // Read trigger source
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   if(sToken == "SW") src = 0;
   else if(sToken == "Q") src = 'Q';
   else if(sToken == "R") src = 'R';
   else if(sToken == "S") src = 'S';
   else return ERR_BADARG;
   // Read active edge
   GetToken(true); if((Token = GetToken(true)) == NULL) return ERR_BADARG;
   sToken = Token;
   if(sToken == "POS") level = POS;
   else if(sToken == "NEG") level = NEG;
   else return ERR_BADARG;
   // Set the parameters and exit
   dcs->TriggerSource = src;
   dcs->TriggerEdge = level;
   return(0);
}

// This command defines the input signal used to trigger a segment
// command line: ADDSEGSTRG,segname,SW | Q | R | S, POS | NEG
void AddSegmentStartTrigger(void)
{
   int  status;
   char *Token;
   
   status = ReadSegmentTriggerSource();
   if(status == 0)
   {
      SendACK;
      return;
   }
   // If here there was an error, get the last Token and if it was not "\n"
   // then read tokens until we get a "\n"
   Token = LastToken();
   while(strcmp(Token,"\n") != 0) Token = GetToken(true);
   SetErrorCode(status);
   SendNAK;
   return;      
}

// List all the segments and there time points
void ListSegments(void)
{
   DCsegment *dcs;
   int i,j;
   
   SendACKonly;
   dcs = DCsegmentList;
   if(DCsegmentList == NULL) serial->println("List is empty!");
   else
   {
      do
      {
         serial->print("Segment,"); serial->print(dcs->name);
         serial->print(", ");
         serial->print(dcs->Length);
         if(dcs->next != NULL)
         {
             serial->print(", ");
             serial->print(dcs->next->name);
             serial->print(", ");
             serial->print(dcs->RepeatCount);
         }
         serial->println("");
         for(i=0;i<dcs->NumTimePoints;i++)
         {
            if(dcs->TimePoint[i]->Setup == DACsetup)
            {
               serial->print("DSTATES,"); serial->print(dcs->TimePoint[i]->Count);
               for(j=0;j<dcs->TimePoint[i]->NumStates;j++)
               {
                  serial->print(", ");
                  serial->print(dcs->TimePoint[i]->States[j]->name);
               }
               serial->println("");
            }
            if(dcs->TimePoint[i]->Setup == DIOsetup)
            {
               serial->print("Trigger, "); serial->println(dcs->TimePoint[i]->Count);
            }
         }
         dcs = dcs->next;
      } while(dcs != NULL);
   }
}

void AbortSegments(void)
{
  SegmentsAbort = true;
  SendACK;
}

void SoftTriggerSegment(void)
{
  if(CurrentSegment != NULL)
  {
     if(CurrentSegment->TriggerSource == 0)
     {
        MPST.softwareTrigger();
        SendACK;  
        return;
     }
  }
  SetErrorCode(ERR_NOTSUPPORTED);
  SendNAK;
  return;
}

// Removes the named segment
void RemoveSegment(char *name)
{
   if(FindInList(DCsegmentList, name) == NULL)
   {
      // Not in list!
      SetErrorCode(ERR_NAMENOTFOUND);
      SendNAK;
      return;
   }
   RemoveFromList(DCsegmentList, name);
   SendACK;  
}

// Removes all segments and frees memory
void RemoveSegments(void)
{
   DCsegment *dcs;
   
   while(DCsegmentList != NULL)
   {
      dcs = DCsegmentList;
      DeleteEntry(dcs); 
      DCsegmentList = DCsegmentList->next;
   }
   SendACK;  
}


