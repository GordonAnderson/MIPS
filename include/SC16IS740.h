#ifndef SC16IS740_H
#define SC16IS740_H

#if FAIMSFBcode

#include "Arduino.h"
#include <WIRE.h>

#define  RHR				0x00
#define  THR				0x00
#define  IER				0x01
#define  FCR				0x02
#define  IIR				0x02
#define  LCR				0x03
#define  MCR				0x04
#define  LSR				0x05
#define  TCR				0x06
#define  TLR				0x07
#define  TXLVL			0x08
#define  RXLVL			0x09
#define  IODir			0x0A
#define  IOState		0x0B
#define  IOControl	0x0E
#define  EFCR				0x0F
#define  DLL        0x00
#define  DLH        0x01
#define  EFR        0x02

void setSC16IS740wire(TwoWire *wire);
void init_SC16IS740(uint8_t addr);
int  readSC16IS740char(uint8_t addr);
int  writeSC16IS740char(uint8_t addr, uint8_t data);

#endif  

#endif
