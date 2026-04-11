#ifndef ADCDRV_H_
#define ADCDRV_H_

extern volatile int ADCchannel;
extern volatile int ADCnumsamples;
extern volatile int ADCvectors;
extern volatile int ADCrate;

// Prototypes
void ADCdelayedTriggerCallback(void);
bool AcquireADC(void);
void ReleaseADC(void);
int  ADCsetup(void);
bool ADCtrigger(void);
void ADCattachInterrupt(void (*isr)(int));
void ReportADCchange(void);

// Host interface functions
void  ADCprep(void);
void  ADCsoftTrigger(void);
void  ADCabort(void);
void  ADCread(int chan);
void  ADCreportAverage(void);
void  ADRsetGain(int chan, int gain);
void  ADCchangeDet(int chan, int thres);
float ReadVin(void);
void  MonitorADCchange(char *gain);
void  SetADCchangeParms(int wc, int sc);

void ADCrbInit(void);
void ADCrbTrig(void);
void ADCrbRead(int start, int num);
bool ADCfindSum(int *sum);
void ADCreadSum(void);
void ADCreadMax(void);
void ADCvectorsRead(void);
bool ADCrbTrigger(void);

void ReportV12(void);
void ReportV24(void);
void ReportCur(void);
void ReportPower(void);

#endif
