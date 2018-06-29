//
// FS7140.c
//
// This file supports the FS7140 PLL clock generator. This clock chip is used on
// the rev 3.0 Twave module.
//
// Gordon Anderson
//
#include "FS7140.h"
#include "Hardware.h"

FS7140 fs7140;

void FindScaler(int target, int *Np1, int *Np2, int *Np3)
{
  int error;
  int i, j, k;

  error = 10000;
  for (i = 1; i <= 12; i++)
  {
    for (j = 1; j <= 12; j++)
    {
      for (k = 1; k <= 8; k *= 2)
      {
        if (abs(target - (i * j * k)) < error)
        {
          *Np1 = i;
          *Np2 = j;
          *Np3 = k;
          error = abs(target - (i * j * k));
          if (error == 0) return;
        }
      }
    }
  }
}

void FS7140setup(int addr, int Fout)
{
  int   Starget;
  int   Np1, Np2, Np3, NF, NR;
  int   i, j, error;
  float f, NFovrNR;

  NF = NR = -1;
  Starget = 44000000 / Fout;
  FindScaler(Starget, &Np1, &Np2, &Np3);
  NFovrNR = (float)(Fout * Np1 * Np2 * Np3) / (float)Fref;
  error = Fout;
  // Try all divisors to find the best match
  for (i = 2; i < 100; i++)
  {
    j = NFovrNR * i;
    if (j < 12) j = 12;
    if (j > 16383) j = 16383;
    f = (float)j / (float)i * (float)Fref / (float)(Np1 * Np2 * Np3);
    if (abs(Fout - f) < error)
    {
      error = abs(Fout - f);
      NF = j;
      NR = i;
    }
  }
  // Fill LC and LR arrays
  float kfVCO = 520000000; // Hz/Volt
  float CP[4] = {0.000002, 0.0000045, 0.000011, 0.0000225};
  float LR[4] = {400000, 133000, 30000, 12000};
  float LC[2] = {0.000000000185, 0.0000000005};
  float C2 = 0.0000000000295;
  // Now calculate the loop filter parameters and 3 dB Bandwidth
  float fPD = Fref / NR;
  int selLC = 0;
  float ideal_LR = 1 / (2 * PI * 0.025 * fPD * LC[selLC]);
  float ideal_CP = (2 * PI / ideal_LR) * (0.1 * fPD / kfVCO) * NF;
  int selLR = 3;
  for (i = 1; i < 4; i++)
  {
    float geoMean = sqrt(LR[i] * LR[i - 1]);
    if (geoMean < ideal_LR )
    {
      selLR = i - 1;
      break;
    }
  }
  int selCP = 3;
  for (i = 1; i < 4; i++)
  {
    float geoMean = sqrt(CP[i] * CP[i - 1]);
    if (geoMean > ideal_CP)
    {
      selCP = i - 1;
      break;
    }
  }
  // Now fill the PLL data structure
  Np3 /= 2;
  if(Np3 > 3) Np3=3;
  fs7140.FSbyte0  = NR & 0xFF;
  fs7140.FSbyte1  = ((NR >> 8) & 0xF) | ((Np3 << 6) & 0xC0);
  fs7140.FSbyte2  = (((Np2 - 1) << 4) & 0xF0) | ((Np1 - 1) & 0x0F);
  fs7140.FSbyte3  = NF & 0xFF;
  fs7140.FSbyte4  = ((NF >> 8)  & 0x3F) | 0x80;
  fs7140.FSbyte5  = (selCP & 0x03) | ((selLR & 0x03) << 4) | ((selLC & 0x01) << 6);
  fs7140.FSbyte6  = 0;
  fs7140.FSbyte7  = 0;
  // Send this data structure to the device unless the address is 0, if its 0 then just exit
  if (addr == 0) return;
  // Now send the struct to the hardware
  Wire.beginTransmission(addr);
  byte *bvals = (byte *)&fs7140;
  Wire.write(0);
  for (i = 0; i < sizeof(FS7140); i++)
  {
    Wire.write(bvals[i]);
  }
  Wire.endTransmission();
}






