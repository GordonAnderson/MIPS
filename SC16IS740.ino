/*

The SC16IS740 is a slave I2C-bus/SPI interface to a single-channel high performance UART. 
It offers data rates up to 5 Mbit/s and guarantees low operating and sleeping current. 

The SC16IS740/750/760’s internal register set is backward-compatible with the widely used 
and widely popular 16C450. This allows the software to be easily written or ported from 
another platform. The SC16IS740/750/760 also provides additional advanced features such 
as auto hardware and software flow control, automatic RS-485 support, and software reset. 
This allows the software to reset the UART at any moment, independent of the hardware 
reset signal.

Address selection on FragInterface PCB

JP1-1    JP1-2     Address, hex
  -        -         0x48 (default)
  -        X         0x49
  X        -         0x4C
  X        X         0x4D

-  = no jumper
X = jumper installed

*/
#include "SC16IS740.h"
#include "Wire.h"

#if FAIMSFBcode

TwoWire *twi;

void TWIwrite(uint8_t addr, uint8_t reg, uint8_t dt)
{
  twi->beginTransmission(addr);
  twi->write(reg << 3);
  twi->write(dt);
  twi->endTransmission();
}

int TWIread(uint8_t addr, uint8_t reg)
{
  twi->beginTransmission(addr);
  twi->write(reg << 3);
  twi->endTransmission();
  twi->requestFrom(addr, (uint8_t)1);
  return(twi->read());
}

void setSC16IS740wire(TwoWire *wire) 
{
   twi = wire;
}

void init_SC16IS740(uint8_t addr) 
{
   TWIwrite (addr, IER, 0x00); // Disable all interrupts
   TWIwrite (addr, LCR, 0x80); // 0x80 to program baud rate
   TWIwrite (addr, DLL, 0x30); // 0x30=19.2K, 0x08 =115.2K with X1=14.7456 MHz
   TWIwrite (addr, DLH, 0x00); // divisor = 0x0008 for 115200 bps
   TWIwrite (addr, LCR, 0xBF); // access EFR register
   TWIwrite (addr, EFR, 0x10); // enable enhanced registers
   TWIwrite (addr, LCR, 0x03); // 8 data bit, 1 stop bit, no parity
   TWIwrite (addr, FCR, 0x06); // reset TXFIFO, reset RXFIFO, non FIFO mode
   delay(1);
   TWIwrite (addr, FCR, 0x01); // enable FIFO mode
}

int  readSC16IS740char(uint8_t addr)
{
   int i;
   
   if ((i = TWIread(addr, LSR)) == -1) return(-1);
   if (i & 0x01)
   {
      i = TWIread (addr, RHR);
      return(i);
   }
   return(-1);
}

int  writeSC16IS740char(uint8_t addr, uint8_t data)
{
   int i;
   
   if ((i = TWIread(addr, LSR)) == -1) return(-1);
   if (i & 0x20)
   {
      TWIwrite (addr, THR, data);
      return(0);
   }
   return(-1);
}

#endif
