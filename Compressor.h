#ifndef COMPRESSOR_H_
#define COMPRESSOR_H_
#include <MIPStimer.h>

#define   C_clock       656250      // This is the clock frequenncy used for the timing control
                                    // for the compressor state machine. This is the clock frequency
                                    // for the timer. 
enum CompressorState
{
  CS_TRIG,
  CS_NONCOMPRESS,
  CS_COMPRESS,
  CS_NORMAL,
  CS_DELAY
};

enum CompressorSwitchState
{
  CSS_OPEN_REQUEST,
  CSS_CLOSE_REQUEST,
  CSS_OPEN,
  CSS_CLOSE,
  CSS_IDLE
};

extern MIPStimer CompressorTimer;

extern char TwaveCompressorTable[];

extern volatile int  C_NormAmpMode;

extern char *CmodeList;
extern char Cmode[];

extern char Cmode[];
extern char CswitchState[];
extern char *CswitchList;

extern CompressorState CState;
extern CompressorSwitchState CSState;
extern char CompressorSelectedSwitch;
extern int CompressorSelectedSwitchLevel;
extern int CurrentPass;

extern DIhandler *CtrigInput;

extern uint32_t  C_Td;               // Trigger delay
extern uint32_t  C_Tc;               // Compressed time
extern uint32_t  C_Tn;               // Normal time
extern uint32_t  C_Tnc;              // Non compressed cycle time
extern uint32_t  C_Delay;            // Delay time
extern uint32_t  C_NextEvent;        // Next event counter value
extern uint32_t  C_SwitchTime;       // Switch open time
extern uint32_t  C_GateOpenTime;     // Switch event time from start of table

// Multi-pass compressor stack structure used for loops
typedef struct
{
  bool Inited;
  int StartOfLoop;
  int Count;
} CompressorStack;

enum SweepState
{
  SS_IDLE,
  SS_START,
  SS_STOP,
  SS_SWEEPING
};

typedef struct
{
   int            OrginalFreq;
   int            StartFreq;
   int            StopFreq;
   float          OrginalVoltage;
   float          StartVoltage;
   float          StopVoltage;
   float          SweepTime;          // In seconds
   unsigned int   SweepStartTime;     // In millisec counts
   unsigned int   CurrentSweepTime;   // In millisec counts
   SweepState     State;  
} FreqSweep;

// Sweep command prototypes
void SetStartFreqTWSW(int chan, int freq);
void GetStartFreqTWSW(int chan);
void SetStopFreqTWSW(int chan, int freq);
void GetStopFreqTWSW(int chan);
void SetStartVoltageTWSW(char *schan, char *svolts);
void GetStartVoltageTWSW(int chan);
void SetStopVoltageTWSW(char *schan, char *svolts);
void GetStopVoltageTWSW(int chan);
void SetSweepTimeTWSW(char *schan, char *stime);
void GetSweepTimeTWSW(int chan);
void StartSweepTWSW(int chan);
void StopSweepTWSW(int chan);
void GetStatusTWSW(int chan);
#endif


