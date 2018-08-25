#ifndef ADCDRV_H_
#define ADCDRV_H_

extern volatile int ADCchannel;
extern volatile int ADCnumsamples;
extern volatile int ADCvectors;
extern volatile int ADCrate;

//Prototypes
bool AcquireADC(void);
void ReleaseADC(void);
int  ADCsetup(void);
void ADCtrigger(void);

// Host interface functions
void ADCprep(void);
void ADCsoftTrigger(void);
void ADCabort(void);

#endif



