#include "SerialBuffer.h"


SerialBuffer::SerialBuffer()
{
}

SerialBuffer::~SerialBuffer()
{
}

void SerialBuffer::clear()
{
   tail = head = sbsize = 0;
}

void SerialBuffer::begin()
{
   tail = head = sbsize = 0;
   wire = NULL;
}

void SerialBuffer::begin(TwoWire *twi, uint8_t add)
{
   tail = head = sbsize = 0;
   wire = twi;
   twiadd = add;
}

size_t SerialBuffer::write(uint8_t by)
{
   if(sbsize > SB_SIZE/2) flush();
   if(sbsize == SB_SIZE) return(0);  // Full!
   buf[head++] = by;
   if(head >= SB_SIZE) head = 0;
   noInterrupts();
   sbsize++;
   interrupts();
   return(1);
}

size_t SerialBuffer::write(const uint8_t *ch, size_t sz)
{
   if(sbsize > SB_SIZE/2) flush();
   if(sbsize == SB_SIZE) return(0);  // Full!
   // Insert characters at head pointer
   int num = 0;
   for(size_t i=0; i < sz; i++)
   {
      buf[head++] = ch[i];
      num++;
      if(head >= SB_SIZE) head = 0;
      noInterrupts();
      sbsize++;
      if(sbsize == SB_SIZE) break;
      interrupts();
   }
   interrupts();
   return(num);
}

int SerialBuffer::available(void)
{
   return(sbsize);
}

int SerialBuffer::read(void)
{
   if(sbsize == 0) return(-1);	// empty
   int i = buf[tail++];
   if(tail >= SB_SIZE) tail = 0;
   noInterrupts();
   sbsize--;
   interrupts();
   return(i);
}

int SerialBuffer::peek(void)
{
	return(0);
}

void SerialBuffer::flush(void)
{
   // If wire channel is defined and the buffer is not
   // empty then send chars
   if((wire != NULL) && (available() > 0))
   {
      wire->beginTransmission(twiadd);
      for(int i=0;i<30;i++)
      {
         if(available() > 0) wire->write(read());
         else break;
      }
      wire->endTransmission();
   }
}

