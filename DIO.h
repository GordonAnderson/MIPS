#ifndef DIO_H_
#define DIO_H_

#include <Arduino.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

extern int mapDItoPIN[8];

#define ReadDIO(a) (digitalRead(mapDItoPIN[a-'Q']) ^ ((MIPSconfigData.DIinvert >> a-'Q') & 1))

// Macros
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

#endif
