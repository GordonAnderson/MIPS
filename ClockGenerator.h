/*
 * ClockGenerator.h
 *
 * Created: 12/27/2011 9:18:16 PM
 *  Author: Gordon Anderson
 */
#ifndef CLOCKGENERATOR_H_
#define CLOCKGENERATOR_H_
#include <MIPStimer.h>

typedef struct
{
  uint8_t	RegAdd;
  uint8_t	ClkA_Div_DS0;
  uint8_t	ClkA_Div_DS1;
  uint8_t	ClkB_Div_DS0;
  uint8_t	ClkB_Div_DS1;
  uint8_t	ClkC_Div;
  uint8_t	ClkD_Div;
  uint8_t	ClkABCD_FS;
  uint8_t	Clk_ACadj;
  uint8_t	Clk_DCadj;
  uint8_t	PLL2_Q;
  uint8_t	PLL2_P;
  uint8_t	PLL2_Misc;
  uint8_t	PLL3_Q;
  uint8_t	PLL3_P;
  uint8_t	PLL3_Misc;
  uint8_t	Osc;
} CY22393_regs;

// Prototypes for function in ClockGenerator
void SetRef(int Freq);
int FindPQ(long ClockOut);
int CY_Init(int8_t adr);
int SetPLL2freq(int8_t adr, int Freq);
int SetPLL3freq(int8_t adr, int Freq);
int FAIMSclockSet(int8_t adr, int Freq);
void FAIMSphase(int harmonic, int phase);

#endif /* CLOCKGENERATOR_H_ */
