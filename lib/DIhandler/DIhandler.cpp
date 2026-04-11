#include "DIhandler.h"
#include <stdarg.h>
#include <stdio.h>
//#include <vector.h>

static void DI_Q_ISR(void);
static void DI_R_ISR(void);
static void DI_S_ISR(void);
static void DI_T_ISR(void);
static void DI_U_ISR(void);
static void DI_V_ISR(void);
static void DI_W_ISR(void);
static void DI_X_ISR(void);

IRQn_Type DIhandler::irqs[8] = {PIOD_IRQn,PIOD_IRQn,PIOC_IRQn,PIOC_IRQn,PIOB_IRQn,PIOD_IRQn,PIOC_IRQn,PIOA_IRQn};
int DIhandler::DIpin[8] = {30,12,50,51,53,28,46,17};
Pio *DIhandler::pio[8] = {g_APinDescription[30].pPort,
						  g_APinDescription[12].pPort,
						  g_APinDescription[50].pPort,
						  g_APinDescription[51].pPort,
						  g_APinDescription[53].pPort,
						  g_APinDescription[28].pPort,
						  g_APinDescription[46].pPort,
						  g_APinDescription[17].pPort};
uint32_t DIhandler::pin[8] = {g_APinDescription[30].ulPin,
							  g_APinDescription[12].ulPin,
							  g_APinDescription[50].ulPin,
							  g_APinDescription[51].ulPin,
							  g_APinDescription[53].ulPin,
							  g_APinDescription[28].ulPin,
							  g_APinDescription[46].ulPin,
							  g_APinDescription[17].ulPin};
void (*DIhandler::DI_ISR[8])(void) = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
void (*DI_ISRs[8])(void) = {DI_Q_ISR,DI_R_ISR,DI_S_ISR,DI_T_ISR,DI_U_ISR,DI_V_ISR,DI_W_ISR,DI_X_ISR};
int DIhandler::NumHandlers = 0;
int DIhandler::NumInterrupts = 0;
DIhandler *DIhandler::handlers[MaxDIhandlers];

DIhandler::DIhandler(void)
{
   // The constructor of the class DIhandler 
   di = 0;
   mode = -1;
   userISR = NULL;
   if(NumHandlers < MaxDIhandlers) handlers[NumHandlers++] = this;
}

DIhandler::~DIhandler(void)
{
   int Index,i;
   
   detach();
   NumHandlers=0;
   for(i=0;i<MaxDIhandlers;i++)
   {
      if(handlers[i] == this) handlers[i] = NULL;
      if(handlers[i] != NULL) NumHandlers++;
   }
}

bool DIhandler::attached(char DI, int Mode,void (*isr)(void))
{
   int Index,i;
   DIhandler *d=NULL;
   
   // Make sure we are in the list else exit with error
   for(i=0;i<MaxDIhandlers;i++) if(handlers[i] == this) d = this;
   if(d != this) return false;
   Index = DI - 'Q';
   if((Index < 0) || (Index > 7)) return false;
   // If we are already attached then exit.
   if(userISR != NULL) return false;
   di = DI;
   mode = Mode;
   // If any handlers are attached then exit 
//   for(i=0;i<MaxDIhandlers;i++)
//   {
//      if((handlers[i]->mode == Mode) && (handlers[i]->di == DI) && (handlers[i]->userISR == isr)) return true;
//   }
   //
   if(DI_ISR[Index] == NULL)
   {
      userISR = isr;
      attachInterrupt(DIpin[Index], DI_ISRs[Index], CHANGE);
      DI_ISR[Index] = DI_ISRs[Index];
   }
   else userISR = isr;
   return true;
}

bool DIhandler::isAttached(void)
{
   int Index;
   
   if(di == 0) return false;
   Index = di - 'Q';
   if((Index < 0) || (Index > 7)) return false;
   if(DI_ISR[Index] == NULL) return false;
   return true;
}

void DIhandler::detach(void)
{
   int Index,i;
   
   if(di == 0) return;
   Index = di - 'Q';
   if((Index < 0) || (Index > 7)) return;
   di = 0;
   userISR = NULL;
   mode = -1;
   // If this input is no longer used then detach the port interrupt
   for(i=0;i<MaxDIhandlers;i++)
   {
      if(handlers[i] != NULL) if(handlers[i]->di == ('Q'+Index)) return;
   }
   if(DIpin[Index] != NULL) detachInterrupt(DIpin[Index]);
   DI_ISR[Index] = NULL;
}

void DIhandler::setPriority(uint8_t pri)
{
   int Index;
   
   if(di == 0) return;
   Index = di - 'Q';
   if((Index < 0) || (Index > 7)) return;

    NVIC_DisableIRQ(irqs[Index]);
	NVIC_ClearPendingIRQ(irqs[Index]);
	NVIC_SetPriority(irqs[Index], pri);
	NVIC_EnableIRQ(irqs[Index]);
}

void DIhandler::setPriority(char DI, uint8_t pri)
{
   int Index;
   
   if(DI == 0) return;
   Index = DI - 'Q';
   if((Index < 0) || (Index > 7)) return;

    NVIC_DisableIRQ(irqs[Index]);
	NVIC_ClearPendingIRQ(irqs[Index]);
	NVIC_SetPriority(irqs[Index], pri);
	NVIC_EnableIRQ(irqs[Index]);
}

uint32_t DIhandler::getPriority(void)
{
   int Index;
   
   if(di == 0) return(0);
   Index = di - 'Q';
   if((Index < 0) || (Index > 7)) return(0);
   return NVIC_GetPriority(irqs[Index]);
}

bool DIhandler::activeLevel(void)
{
   int Index,i;
   
   Index = di - 'Q';
   if((di==0) || (mode==-1)) return true;
   i = digitalRead(DIpin[Index]);
   if((i == HIGH) && (mode == CHANGE)) return true;
   if((i == HIGH) && ((mode == RISING) || (mode == HIGH))) return true;
   if((i == LOW) && ((mode == FALLING) || (mode == LOW))) return true;
   return false;
}

bool DIhandler::state(void)
{
   int Index,i;
   
   Index = di - 'Q';
   if((di==0) || (mode==-1)) return true;
   i = digitalRead(DIpin[Index]);
   if(i == HIGH) return true;
   return false;
}

bool DIhandler::test(int mode)
{
   int Index,i;
   
   Index = di - 'Q';
   if((di==0) || (mode==-1)) return true;
   i = digitalRead(DIpin[Index]);
   if((i == HIGH) && ((mode == RISING) || (mode == HIGH))) return true;
   if((i == LOW) && ((mode == FALLING) || (mode == LOW))) return true;
   return false;
}

void DI_Generic_ISR(int Index)
{
   int i,pinstate = HIGH;
   void  (*userISRptr)(void);
   
   DIhandler::NumInterrupts++;
//   if((DIhandler::pio[Index]->PIO_ODSR & DIhandler::pin[Index]) == 0) pinstate = LOW;
   pinstate = digitalRead(DIhandler::DIpin[Index]);
   for(i=0;i<MaxDIhandlers;i++)
   {
      if(DIhandler::handlers[i] != NULL)  // was, if(DIhandler::handlers != NULL)
      {
         if(DIhandler::handlers[i]->di != ('Q'+Index)) continue; 
         userISRptr = DIhandler::handlers[i]->userISR;
         if(userISRptr == NULL) continue;
         // If this event has been processed already then skip it
         for(int j=0;j<i;j++)
         {
            if((DIhandler::handlers[j]->mode == DIhandler::handlers[i]->mode)\
                && (DIhandler::handlers[j]->di == DIhandler::handlers[i]->di)\
                && (DIhandler::handlers[j]->userISR == DIhandler::handlers[i]->userISR)) userISRptr = NULL;
         }
         if(userISRptr == NULL) continue;
         if(DIhandler::handlers[i]->mode == CHANGE) userISRptr();
         else if((DIhandler::handlers[i]->mode == HIGH) && (pinstate == HIGH)) userISRptr();
         else if((DIhandler::handlers[i]->mode == LOW) && (pinstate == LOW)) userISRptr();
         else if((DIhandler::handlers[i]->mode == RISING) && (pinstate == HIGH)) userISRptr();
         else if((DIhandler::handlers[i]->mode == FALLING) && (pinstate == LOW)) userISRptr();
      }
   }
}

void DI_Q_ISR(void)
{
   DI_Generic_ISR(0);
}
void DI_R_ISR(void)
{
   DI_Generic_ISR(1);
}
void DI_S_ISR(void)
{
   DI_Generic_ISR(2);
}
void DI_T_ISR(void)
{
   DI_Generic_ISR(3);
}
void DI_U_ISR(void)
{
   DI_Generic_ISR(4);
}
void DI_V_ISR(void)
{
   DI_Generic_ISR(5);
}
void DI_W_ISR(void)
{
   DI_Generic_ISR(6);
}
void DI_X_ISR(void)
{
   DI_Generic_ISR(7);
}

