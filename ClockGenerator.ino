/*
 * ClockGenerator.c
 *
 * This file contains the driver for the Cypress CY22393 PLL clock generator.
 * 20MHz HC-49US crystal used, 30 ohms ESR, 7 pF shunt capacitance.
 *
 * Updated to disable the PLL before update. This is recomended in the data sheet to
 * prevent any out of bound PLL conditions.
 *
 * Created: 12/27/2011 7:26:02 PM
 *  Author: Gordon Anderson
 */
#include "stdlib.h"
#include "ClockGenerator.h"

CY22393_regs CYregs;

// PLL parameters
int16_t  P0;
int16_t  P;
int16_t  Q;
int16_t  Div;
int16_t  LF;

int Ref = 8000000;

void SetRef(int Freq)
{
  Ref = Freq;
}

int FindPQ(long ClockOut)
{
  int  Pt, Qt, Divider, D, MaxQ, AClockOut;
  int  Cerror, CerrorMin, A, B;

  // Set the reference
  //  Ref = 8000000;
  //  MaxQ = Ref/40000;
  MaxQ = 200;
//  if(ClockOut > 5000000) MaxQ = 75;
  // Determine starting divider
  Divider = ((Ref / MaxQ) * 1600) / ClockOut;
  if (Divider > 117) Divider = 117;
  if (Divider < 11) Divider = 11; 
  // Loop through 20 Divider values
  CerrorMin = ClockOut;
  for (D = Divider - 10; D <= Divider + 10; D++)
  {
    // Loop through Qt values
    A = (ClockOut / 1000) * D;
    B = Ref / 1000;
    for (Qt = 2; Qt <= MaxQ; Qt++)
    {
      // Calculate Pt
      Pt = (A * Qt) / B;
      if(Pt > 1600) Pt = 1600;
      // Now calculate Actual ClockOut
      AClockOut = ((Ref / Qt) * Pt) / D;
      Cerror = labs(AClockOut - ClockOut);
      if (Cerror < CerrorMin)
      {
        CerrorMin = Cerror;
        Q = Qt;
        P = Pt;
        Div = D;
        if (Cerror < 50) Cerror = 0;
      }
      if (Cerror == 0) break;
    }
    if (Cerror == 0) break;
  }
  //serial->println(P);
  //serial->println(Q);
  P0 = P & 1;
  P = ((P - P0) / 2) - 3;
  Q = Q - 2;
  LF = 0;
  if (P > 231) LF = 1;
  if (P > 626) LF = 2;
  if (P > 834) LF = 3;
  if (P > 1043) LF = 4;
  //  P = ((P - P0) / 2) - 3;
  //serial->println(P);
  //serial->println(Q);
  //serial->println(LF);
  //serial->println(Div);
  return (CerrorMin);
}

int CY_Init(int8_t adr)
{
  int iStat;

  //
  // Initialize the data structure
  //
  CYregs.RegAdd = 0x08;
  CYregs.Osc = 0b10010101;		// Crystal, 7PF, 30 ohms
  // Set clock A and C to PLL2
  //   Clk*_fs[2:0] = 100
  CYregs.ClkA_Div_DS0 = 0b00000000;
  CYregs.ClkA_Div_DS1 = 0b00000000;
  CYregs.ClkC_Div = 0b00000000;
  // Set clock B and D to PLL3
  //   Clk*_fs[2:0] = 110
  CYregs.ClkB_Div_DS0 = 0b00000000;
  CYregs.ClkB_Div_DS1 = 0b00000000;
  CYregs.ClkD_Div = 0b00000000;
  CYregs.ClkABCD_FS = 0b11101110;	// FS[2:1]
  // Set output duty cycle, enables, clkE_div
  CYregs.Clk_ACadj = 0b01010101;
  // Set the output drive level
  CYregs.Clk_DCadj = 0b01010101;
  // PLL2
  CYregs.PLL2_Q = 0;
  CYregs.PLL2_P = 0;
  CYregs.PLL2_Misc = 0b01000000;
  // PLL3
  CYregs.PLL3_Q = 0;
  CYregs.PLL3_P = 0;
  CYregs.PLL3_Misc = 0b01000000;
  //
  // Send to the device using TWI
  //
  Wire.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;
  for (int i = 0; i < sizeof(CY22393_regs); i++)
  {
    Wire.write(bvals[i]);
  }
  if ((iStat = Wire.endTransmission()) != 0) return (iStat);
  return (0);
}

// If state = true then PLL2 will be enabled, else disabled.
void SetPLL2enable(int8_t adr, bool state)
{
  if(state)
  {
     Wire.beginTransmission(adr);
     Wire.write(0x08);
     Wire.write(CYregs.ClkA_Div_DS0);
     Wire.write(CYregs.ClkA_Div_DS1);
     Wire.endTransmission();
  }
  else
  {
     Wire.beginTransmission(adr);
     Wire.write(0x08);
     Wire.write(0x00);
     Wire.write(0x00);
     Wire.endTransmission();
  }
}

int SetPLL2freq(int8_t adr, int Freq)
{
  int iStat;

  //
  // Setup PLL2 parameters
  //
  FindPQ(Freq);			// Determine the PLL values
  CYregs.PLL2_P = P & 0xFF;
  CYregs.PLL2_Misc = (CYregs.PLL2_Misc & 0xC0) | (LF << 3) | (P0 << 2) | (P >> 8);
  CYregs.PLL2_Q = Q;
  CYregs.ClkA_Div_DS0 = (CYregs.ClkA_Div_DS0 & 0x80) | (Div & 0x7F);
  CYregs.ClkA_Div_DS1 = (CYregs.ClkA_Div_DS1 & 0x80) | (Div & 0x7F);
  CYregs.ClkC_Div = (CYregs.ClkC_Div & 0x80) | (Div & 0x7F);
  // Disable PLL2
  CYregs.PLL2_Misc &= ~0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x13);
  Wire.write(CYregs.PLL2_Misc);
  Wire.endTransmission();
  //
  // Send data structure to the device using TWI
  //
  Wire.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;

  for (int i = 0; i < sizeof(CY22393_regs); i++)
  {
    Wire.write(bvals[i]);
  }
  if ((iStat = Wire.endTransmission()) != 0) return (iStat);
  // Enable PLL2
  CYregs.PLL2_Misc |= 0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x13);
  Wire.write(CYregs.PLL2_Misc);
  Wire.endTransmission();
  return (0);
}

// If state = true then PLL2 will be enabled, else disabled.
void SetPLL3enable(int8_t adr, bool state)
{
  if(state)
  {
     Wire.beginTransmission(adr);
     Wire.write(0x0A);
     Wire.write(CYregs.ClkB_Div_DS0);
     Wire.write(CYregs.ClkB_Div_DS1);
     Wire.endTransmission();
  }
  else
  {
     Wire.beginTransmission(adr);
     Wire.write(0x0A);
     Wire.write(0x00);
     Wire.write(0x00);
     Wire.endTransmission();
  }
}

int SetPLL3freq(int8_t adr, int Freq)
{
  int iStat;

  //
  // Setup PLL3 parameters
  //
  FindPQ(Freq);			// Determine the PLL values
  CYregs.PLL3_P = P & 0xFF;
  CYregs.PLL3_Misc = (CYregs.PLL3_Misc & 0xC0) | (LF << 3) | (P0 << 2) | (P >> 8);
  CYregs.PLL3_Q = Q;
  CYregs.ClkB_Div_DS0 = (CYregs.ClkB_Div_DS0 & 0x80) | (Div & 0x7F);
  CYregs.ClkB_Div_DS1 = (CYregs.ClkB_Div_DS1 & 0x80) | (Div & 0x7F);
  CYregs.ClkD_Div = (CYregs.ClkD_Div & 0x80) | (Div & 0x7F);
  // Disable PLL3
  CYregs.PLL3_Misc &= ~0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x16);
  Wire.write(CYregs.PLL3_Misc);
  Wire.endTransmission();
  //
  // Send to the device using TWI
  //
  Wire.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;
  for (int i = 0; i < sizeof(CY22393_regs); i++)
  {
    Wire.write(bvals[i]);
  }
  if ((iStat = Wire.endTransmission(true)) != 0) return (iStat);
  // Disable PLL3
  CYregs.PLL3_Misc |= 0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x16);
  Wire.write(CYregs.PLL3_Misc);
  Wire.endTransmission();
  return (0);
}

// This function sets the clock frequeancy for the FAIMS rev 2 controller.
// Clock is set to 20 times the Freq value passed and the divers are set as follows
// Output B = Fundemental, div by 120
// Output C = Second harmonic, div by 60
// Output D = Forth harmonic, div by 30
// Output A = Fifth harmonic, div by 24
// Output E = unused
int FAIMSclockSet(int8_t adr, int Freq)
{
  float PTdivQT,error;
  int   PT,QT,Q,P,PO,BestPT,BestQT;
  int iStat;

  CYregs.PLL2_Misc &= ~0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x13);
  Wire.write(CYregs.PLL2_Misc);
  Wire.endTransmission();
  CYregs.RegAdd = 0x08;
  CYregs.Osc = 0b10010101;    // Crystal, 7PF, 30 ohms
  // Set the dividers
  CYregs.ClkA_Div_DS0 = 24;
  CYregs.ClkA_Div_DS1 = 24;
  CYregs.ClkB_Div_DS0 = 120;
  CYregs.ClkB_Div_DS1 = 120;
  CYregs.ClkC_Div = 60;
  CYregs.ClkD_Div = 30;
  CYregs.Clk_ACadj = 1;  // Set E out divider to 4
  // Set the clock sources to PLL1
  CYregs.ClkABCD_FS = 0b10101010;
  // Set output duty cycle, enables, clkE_div
  CYregs.Clk_ACadj |= 0b01010100;
  // Set the output drive level
  CYregs.Clk_DCadj = 0b00000000;
  // Set the frequency for the PLL, set them all to 20 * Freq.
  // Nominal Freq = 1mHz, Ref osc 20MHz = Ref * PT/QT
  PTdivQT = (float)(Freq * 120) / (float)Ref;
  error = 10000;
  for(QT = 1; QT <= 256; QT++)
  {
    PT = PTdivQT * QT;
    if(PT < 2048 )
    {
      if(error > abs(PTdivQT - (float)PT/(float)QT))
      {
        error > abs(PTdivQT - (float)PT/(float)QT);
        BestPT = PT;
        BestQT = QT;
      }
    }
  }
  PT = BestPT;
  QT = BestQT;
  //serial->println(Freq);
  //serial->println(PT);
  //serial->println(QT);
  // Calculate P,P0, and Q
  // PT =  (2 * (P+3)) + PO
  // QT = Q + 2
  Q = QT - 2;
  // If PT is odd then PO = 1;
  if((PT & 1) != 0) PO = 1;
  else PO = 0;
  P = (PT - PO)/2 - 3;
  LF = 0;
  
  if (P > 231) LF = 1;
  if (P > 626) LF = 2;
  if (P > 834) LF = 3;
  if (P > 1043) LF = 4;
  LF = 1;
  // Disable PLL2 and PLL3
  CYregs.PLL2_P = P & 0xFF;
  CYregs.PLL2_Misc = (0x00) | (LF << 3) | (PO << 2) | (P >> 8);
  CYregs.PLL2_Q = Q;
  CYregs.PLL3_P = P & 0xFF;
  CYregs.PLL3_Misc = (0x00) | (LF << 3) | (PO << 2) | (P >> 8);
  CYregs.PLL3_Q = Q;
  // Send the data to the chip
  Wire.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;
  for (int i = 0; i < sizeof(CY22393_regs); i++) Wire.write(bvals[i]);
  if ((iStat = Wire.endTransmission(true)) != 0) return (iStat);  
  // Set the PLL1 regs, use PLL2 in struct
  CYregs.PLL2_Misc |= 0x40;
  Wire.beginTransmission(adr);
  Wire.write(0x13);
  Wire.write(CYregs.PLL2_Misc);
  Wire.endTransmission();
  return(0);
}

// For harmonic, 1,2,4,5 set phase to 0 or 180
void FAIMSphase(int harmonic, int phase)
{
  byte a;
  byte adr = 0x69;
  
  if(harmonic == 1)  // output B
  {
    a = 120;
    if(phase == 180) a |= 0x80;
    Wire.beginTransmission(adr);
    Wire.write(0x0a);
    Wire.write(a);
    Wire.write(a);
    Wire.endTransmission();
  }
  else if(harmonic == 2)  // output C
  {
    a = 60;
    if(phase == 180) a |= 0x80;
    Wire.beginTransmission(adr);
    Wire.write(0x0c);
    Wire.write(a);
    Wire.endTransmission();    
  }
  else if(harmonic == 4)  // output D
  {
    a = 30;
    if(phase == 180) a |= 0x80;
    Wire.beginTransmission(adr);
    Wire.write(0x0d);
    Wire.write(a);
    Wire.endTransmission();        
  }
  else if(harmonic == 5)  // output A
  {
    a = 24;
    if(phase == 180) a |= 0x80;
    Wire.beginTransmission(adr);
    Wire.write(0x08);
    Wire.write(a);
    Wire.write(a);
    Wire.endTransmission();    
  }
}
