#ifndef FS7140_H_
#define FS7140_H_

// 20 MHz PLL clock
#define Fref     20000000  // Reference frequency

typedef struct
{
   uint8_t   FSbyte0;
   uint8_t   FSbyte1;
   uint8_t   FSbyte2;
   uint8_t   FSbyte3;
   uint8_t   FSbyte4;
   uint8_t   FSbyte5;
   uint8_t   FSbyte6;
   uint8_t   FSbyte7;
} FS7140;

// prototypes
void FindScaler(int target, int *Np1,int *Np2,int *Np3) ;
void FS7140setup(int addr, int Fout);

#endif
