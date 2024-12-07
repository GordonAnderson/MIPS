#ifndef DIO_H_
#define DIO_H_

#include <Arduino.h>
#include "DIhandler.h"

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

extern int mapDItoPIN[8];

// Input pulse countiing data structure. Supports counting input pulses
// and then generating an output trigger
typedef struct
{
  DIhandler *ctrDI;             // Input interrupt handler
  char      din[3];             // Input trigger, Q thru X, NA to disable
  char      level[5];           // Trigger level
  uint32_t  count;              // Counter value
  uint32_t  tcount;             // Counter trigger level
  bool      resetOnTcount;      // Reset the count at threshold
  bool      triggerOnTcount;    // Generate an output trigger at threshold count
} PulseCounter;

extern PulseCounter *pulseCounter;
int FindInList(char *list, char *entry);

// Macros
#define ReadDIO(a) (digitalRead(mapDItoPIN[a-'Q']) ^ ((MIPSconfigData.DIinvert >> a-'Q') & 1))
#define DOrefresh  DigitalOut(MIPSconfigData.DOmsb, MIPSconfigData.DOlsb)

// Prototypes
void DIO_init(void);
void DIO_loop(void);
void GDIO_Serial(char *CH);
void SDIO_Set_Image(char chan,char val);
void SDIO_Serial(char *CH, char *State);
void UpdateDigitialOutputArray(void);
void SetImageRegs(void);

// General purpose DIO functions
void TriggerOut(char *cmd);
void AuxOut(char *cmd);
void FollowSisr(void);
void TriggerFollowS(void);
void TriggerOut(int microSec, bool WithLimits = false);
void QueueTriggerOut(int microSec);
void ProcessTriggerOut(void);
bool QueueTpulseFunction(void (*Tfunction)(), bool add);
void PlayTpulseFunctions(void);
void AuxTrigger(void);

void DIOopsReport(void);
void DIOreport(char *port, char *mode);
void DIOmirror(char *in, char *out);
void DIOmonitor(char *port, char *mode);
void DIOchangeReport(char *port);

// Pulse counter prototypes
void definePulseCounter(char *DI, char *TL);
void getPulseCounter(void);
void clearPulseCounter(void);
void setPulseCounterThreshold(int thres);
void getPulseCounterThreshold(void);
void triggerOnCounterhreshold(char *val);
void resetOnCounterhreshold(char *val);


#endif
