#ifndef DIO_H_
#define DIO_H_

#include <Arduino.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

// Macros
#define DOrefresh  DigitalOut(MIPSconfigData.DOmsb, MIPSconfigData.DOlsb)

// Prototypes
void DIO_init(void);
void DIO_loop(void);
void GDIO_Serial(char *CH);
void SDIO_Set_Image(char chan,char val);
void SDIO_Serial(char *CH, char *State);

#endif




