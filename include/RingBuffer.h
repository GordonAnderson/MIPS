// This file is a shim that provides the Arduino SAM framework's class RingBuffer
// definition. Without it, the GAACE library's RingBuffer.h (defining class ringBuffer,
// lowercase) shadows the SAM framework's RingBuffer.h, causing USBAPI.h to fail with
// "'RingBuffer' does not name a type".
//
// The GAACE library's own source files are compiled with GAACE/src first in their
// include path, so they still find their own RingBuffer.h and class ringBuffer.
#ifndef _RING_BUFFER_
#define _RING_BUFFER_

#include <stdint.h>

#define SERIAL_BUFFER_SIZE 128

class RingBuffer
{
  public:
    volatile uint8_t _aucBuffer[SERIAL_BUFFER_SIZE] ;
    volatile int _iHead ;
    volatile int _iTail ;

  public:
    RingBuffer( void ) ;
    void store_char( uint8_t c ) ;
} ;

#endif /* _RING_BUFFER_ */
