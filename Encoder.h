/*
  Encoder.h - Encoder header file, definition of methods and attributes.

  This class supports a rotary encoder with intergral push button.

  There can only be one of these objects created at any one time,
  most variables are static becuse this object includes interrupt
  service routines.

  Created by Gordon Anderson, November, 2013.
*/
#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"
#include <inttypes.h>

class Encoder
{
    friend void encoderISR(void);
    friend void encoderPBISR(void);
  protected:
    static int ENC_A;
    static int ENC_B;
    static int ENC_PB;
    static int *ValuePtr;
    static int MaxValue;
    static int MinValue;
    static short int encStep;
  public:
    // Needs to be public, because the handlers are outside class:
    static void (*callbackChange)();
    static void (*callbackPushButton)();

    Encoder(void);
    Encoder start(int A, int B, int PB);
    Encoder stop(void);
    Encoder attachInterruptChange(void (*isr)());
    Encoder attachInterruptPushButton(void (*isr)());
    Encoder setValue(int *Value);
    Encoder setMinimum(int minimum);
    Encoder setMaximum(int maximum);
    Encoder setStepSize(short int Step);
};
#endif





