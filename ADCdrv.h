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
void ADCattachInterrupt(void (*isr)(int));
void ReportADCchange(void);

// Host interface functions
void ADCprep(void);
void ADCsoftTrigger(void);
void ADCabort(void);
void ADCread(int chan);
void ADRsetGain(int chan, int gain);
void ADCchangeDet(int chan, int thres);
float ReadVin(void);
void MonitorADCchange(char *gain);
void SetADCchangeParms(int wc, int sc);

#endif
