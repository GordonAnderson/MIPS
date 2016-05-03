/*
  Encoder.h - Implementation of Encoder defined in Encoder.h.

  This class supports a rotary encoder with intergral push button.

  There can only be one of these objects created at any one time,
  most variables are static becuse this object includes interrupt
  service routines.

  Created by Gordon Anderson, November, 2013.
*/
#include "Encoder.h"
#include "AtomicBlock.h"

int Encoder::ENC_A;
int Encoder::ENC_B;
int Encoder::ENC_PB;
int *Encoder::ValuePtr;
int Encoder::MaxValue;
int Encoder::MinValue;
short int Encoder::encStep;

void (*Encoder::callbackChange)() = NULL;
void (*Encoder::callbackPushButton)() = NULL;

void encoderISR(void);
void encoderPBISR(void);

// The constructor of the class PPM
Encoder::Encoder(void)
{
  ValuePtr = NULL;
  MaxValue = 10000;
  MinValue = -10000;
  encStep = 1;
  ENC_PB = -1;
}

Encoder Encoder::start(int A, int B, int PB)
{
  ValuePtr = NULL;
  ENC_A = A;
  ENC_B = B;
  ENC_PB = PB;
  // Setup the io pins for the encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(ENC_A, encoderISR, CHANGE);
  attachInterrupt(ENC_B, encoderISR, CHANGE);
  if (ENC_PB > 0)
  {
    pinMode(ENC_PB, INPUT);
    attachInterrupt(ENC_PB, encoderPBISR, FALLING);
  }
  return *this;
}

Encoder Encoder::stop(void)
{
  ValuePtr = NULL;
  detachInterrupt(ENC_A);
  detachInterrupt(ENC_B);
  if (ENC_PB > 0)
  {
    detachInterrupt(ENC_PB);
  }
  return *this;
}

Encoder Encoder::attachInterruptChange(void (*isr)())
{
  callbackChange = isr;
  return *this;
}

Encoder Encoder::attachInterruptPushButton(void (*isr)())
{
  if (ENC_PB > 0)
  {
    callbackPushButton = isr;
  }
  return *this;
}

Encoder Encoder::setValue(int *Value)
{
  ValuePtr = Value;
  return *this;
}

Encoder Encoder::setStepSize(short int Step)
{
  encStep = Step;
  return *this;
}


/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */
void encoderISR(void)
{
  static unsigned int pt = 0;
  static int multiplier = 1;
  static int lastValue;
  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encchg;       //encoder change
  static const int8_t enc_states[] =
  {0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //encoder lookup table
//  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0}; //encoder lookup table
  // Try adding additional states to see if it helps the performance. Also monitor
  // the actual bit status when we are sitting on a detent.  
//    {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table

  AtomicBlock< Atomic_RestoreState > a_Block;
  // The encoder has 24 detents, 24 pulses per revolution
  //
  // bits
  // abcd,  a & c are phase A
  //  ab are previous states
  //  cd are current state
  //
  // forward count on state
  //  1101
  //
  // reverse count on state
  //  1110
  old_AB <<= 2; //remember previous state
  old_AB |= (((digitalRead(Encoder::ENC_A) & 1) << 1) | (digitalRead(Encoder::ENC_B) & 1));
  encchg = pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
  /* post "Navigation forward/reverse" event */
  if ( encchg != 0 )
  {
    // The code inside the following if block is responsible for increasing the step
    // size if the encoder is rapidly changed
    if (Encoder::ValuePtr != NULL)
    {
      if (pt == 0)
      {
        pt = millis();
        lastValue = *Encoder::ValuePtr;
      }
      else if (abs(*Encoder::ValuePtr - lastValue) > 10)
      {
        if (abs(millis() - pt) < 600) multiplier *= 10;
        else multiplier /= 10;
        if (multiplier < 1) multiplier = 1;
        if (multiplier > 100) multiplier = 100;
        
        multiplier=1;  // disable
        
        lastValue = *Encoder::ValuePtr;
        pt = millis();
      }
      *Encoder::ValuePtr += encchg * Encoder::encStep * multiplier;
      if (*Encoder::ValuePtr > Encoder::MaxValue) *Encoder::ValuePtr = Encoder::MaxValue;
      if (*Encoder::ValuePtr < Encoder::MinValue) *Encoder::ValuePtr = Encoder::MinValue;
    }
    if (Encoder::callbackChange != NULL) Encoder::callbackChange();
  }
}

void encoderPBISR(void)
{
  static unsigned int pt;

  if (abs(millis() - pt) > 300)
  {
    if (Encoder::callbackPushButton != NULL) Encoder::callbackPushButton();
  }
  pt = millis();
}




