#include <Arduino.h>
#include "MemoryFix.h"
#include "syscalls.h"

#include <stdio.h>
#include <stdarg.h>
#include "sam.h"
#if defined (  __GNUC__  ) /* GCC CS3 */
  #include <sys/types.h>
  #include <sys/stat.h>
#endif
#include <cstdlib>

// Helper macro to mark unused parameters and prevent compiler warnings.
// Appends _UNUSED to the variable name to prevent accidentally using them.
#ifdef __GNUC__
#  define UNUSED(x) x ## _UNUSED __attribute__((__unused__))
#else
#  define UNUSED(x) x ## _UNUSED
#endif

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

#undef errno
extern int errno ;
extern int  _end ;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
extern void _exit( int status ) ;
extern void _kill( int pid, int sig ) ;

// The following code replaces the _sbrk function
// in the syscalls_sam3.c file. This function fixes the allocation problems
// with malloc, it will now return NULL when out of memory.
// syscalls_sam3.c must be edited and the name of _sbrk changed or you will get
// linker errors.
//
// Note; the due implementation of new and delete are very simple, basically
// they are malloc and free and new does return NULL on allocation error if the
// _sbrk function is update as shown below.

#define ENOMEM 12
size_t __malloc_margin = 4 * (size_t)1024;
extern caddr_t _sbrk (int incr) {
    static const unsigned char *       heap      = (unsigned char *)&_end ;
    const unsigned char * const prev_heap = heap;

    if (incr > 0) {
        // Extra checks for stack growing


        if (incr >= 96 * (size_t)1024) {
            // The Due only has 96K of ram,
            // so any request for 96K or greater
            // is a failure out of the gate.
            errno = ENOMEM;
            return (caddr_t)-1;
        }

        unsigned char *stack_ptr;

        asm volatile(
            "mov %[sp_out], sp"
            : [sp_out] "=r" (stack_ptr)
            :
        );

        if (stack_ptr < heap) {
            // We're already in real trouble.
            abort();
        }

        if (stack_ptr - heap  <  incr + __malloc_margin) {
            // Not enough memory considering safety margin.
            errno = ENOMEM;
            return (caddr_t)-1;
        }
    }

    heap      += incr ;
    return (caddr_t) prev_heap ;
}

// For ARM Cortex-M3 boards like Arduino Due
extern "C" char* sbrk(int incr);
int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

