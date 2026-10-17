//
// File: TWIext
//
// This file contains TWI extentions used by the MIPS application. These extnsions allow sending and received various
// data type to and from MIPS modules that used a TWI interface. Specifically the ARB, FAMISfb, and rev 6.0 RF driver.
// Additional function support acquiring the TWI interface and releasing to allow use in ISRs. The TWI interface is not
// interrupt safe. The Acquire and Release logic will queue requests made when the TWI system is in use and these
// queued requests will be performed when the release call is made.
//
// Software TWI routines are included and were developed to support the analog devices AD7998, this device would not
// funcnction with the TWI drivers in the Arduino libraries.
//
// Gordon Anderson
//
#include <Arduino.h>
#include <Wire.h>
#include "TWIext.h"
#include "Variants.h"

#include "AtomicBlock.h"
int  SelectedBoard(void);
void SelectBoard(int8_t Board);

int WireDefaultSpeed  = 100000;
int Wire1DefaultSpeed = 100000;

bool TWItest(int8_t adr)
{
  Wire.beginTransmission(adr);
  if(Wire.endTransmission() == 0) return true;
  return false;
}

// The following code sopports the TWI interface by implementing the protocol in
// software, i.e. bit banging. This provides maximum flexibility but is only
// used when the Wire function will not work.

// TWI bus reset function.
// The bus recovery procedure is as follows
//  1.) Master tries to assert logic 1 on SDA line
//  2.) Master still sees a logig 0 then generate a clock pulse on SLC (1,0,1 transistion)
//  3.) Master examines SDA. If SDA = 0 goto step 2 if SDA = 1 goto step 4
//  4.) Generate a STOP condition
void TWI_RESET(void)
{
  // Do reset procedure for both board addresses
  int brd = SelectedBoard();
  for(int b=0; b<2;b++)
  {
    SelectBoard(b);
    for(int i=0;i<2;i++)
    {
      TWI_START();
      TWI_STOP();
      TWI_SDA_IN;
      TWI_SCL_OUT;
      for(int j = 0; j < 10; j++)
      {
        TWI_SCL_HI;
        TWI_SCL_LOW;
        delayMicroseconds(5);
        TWI_SCL_HI;
        delayMicroseconds(5);
        if(TWI_SDA_data == HIGH) break;
      }
      TWI_STOP();
      TWI_STOP();
    }
  }
  SelectBoard(brd);
}

// Issue a start condition
void TWI_START(void)
{
  TWI_SCL_OUT;
  TWI_SDA_OUT;
  TWI_SCL_HI;
  TWI_SDA_HI;
  TWI_SDA_LOW;
  TWI_SCL_LOW;
}

// Issue a stop condition
void TWI_STOP(void)
{
  TWI_SDA_OUT;
  TWI_SDA_LOW;
  TWI_SDA_LOW;
  TWI_SCL_HI;
  TWI_SDA_HI;
}

// Write a byte to the TWI bus, this function returns true if acked and false if naked
bool TWI_WRITE(int8_t val)
{
  int8_t Response;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  TWI_SDA_OUT;
  for (int i = 0; i < 8; i++)
  {
    delayMicroseconds(5);
    if ((val & 0x80) != 0) TWI_SDA_HI;
    else TWI_SDA_LOW;
    val = val << 1;
    TWI_SCL_HI;
    delayMicroseconds(5);
    TWI_SCL_LOW;
  }
  // Now read the ACK or NAK from the device
  TWI_SDA_IN;
  TWI_SCL_HI;
  Response = TWI_SDA_data;
  TWI_SCL_LOW;
  if (Response == HIGH) return (false);
  return (true);
}

// Reads and returns a byte from the TWI interface, if reply is true than ACK is sent to device,
// if reply is false the NAK is sent.
int8_t TWI_READ(bool Reply)
{
  int8_t val;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  val = 0;
  TWI_SDA_IN;
  for (int i = 0; i < 8; i++)
  {
    val = val << 1;
    TWI_SCL_HI;
    delayMicroseconds(5);
    if (TWI_SDA_data == HIGH) val |= 1;
    TWI_SCL_LOW;
    delayMicroseconds(5);
  }
  // Now write the ACK or NAK to the device
  TWI_SDA_OUT;
  if (Reply == HIGH) TWI_SDA_HI;
  else TWI_SDA_LOW;
  TWI_SCL_HI;
  TWI_SCL_LOW;
  TWI_SDA_IN;
  return (val);
}

// Called when a TWI error is detected. This function will reset the TWI interface and reinit the
// wire driver
void TWIerror(void)
{
  TWI_RESET();
  Wire.begin();
}

void TWIreset(void)
{
  Wire.end();
  TWI_RESET();
  Wire.begin();
  Wire.setClock(WireDefaultSpeed);
  SendACK;
}

//
// Routines used to allow TWI use in ISR. The TWI queue functions allow you to
// queue function with parameters.
//
TWIqueueEntry TWIq[MaxQueued] = {{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL},{Empty,NULL}};

bool TWIbusy=false;
// This functoin acquires the TWI interface.
// Returns false if it was busy.
bool AcquireTWI(void)
{
 // AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy) return(false);
  TWIbusy = true;
  return(true);
}

// This routine releases the TWI interface and if there are any queued functions they are called
// and then the queued pointer it set to NULL. This allows an ISR to use this function
// and queue up its io if the TWI interface is busy.
void ReleaseTWI(void)
{
  static bool busy = false;

  if(busy) return;
  busy = true;
  //AtomicBlock< Atomic_RestoreState > a_Block;
  if(TWIbusy)
  {
    int b=SelectedBoard();
    for(int i=0;i<MaxQueued;i++)
    {
      if(TWIq[i].pointers.funcVoidVoid != NULL) 
      {
        if(TWIq[i].Type == VoidVoid) TWIq[i].pointers.funcVoidVoid();
        else if(TWIq[i].Type == VoidIntIntFloat) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Float1);
        else if(TWIq[i].Type == VoidIntFloat) TWIq[i].pointers.funcIntFloat(TWIq[i].Int1,TWIq[i].Float1);
        else if(TWIq[i].Type == VoidIntInt) TWIq[i].pointers.funcIntInt(TWIq[i].Int1,TWIq[i].Int2);
        else if(TWIq[i].Type == VoidIntIntBool) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Bool1);
        else if(TWIq[i].Type == VoidIntIntWord) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Word1);
        else if(TWIq[i].Type == VoidIntIntByte) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,(byte)TWIq[i].Word1);
        TWIq[i].pointers.funcVoidVoid = NULL;
        TWIq[i].Type = Empty;
      }
    }
    SelectBoard(b);
    TWIbusy=false;
  }
  busy=false;
}

// This queues up a function to call when the current TWI operation finishes.
void TWIqueue(void (*TWIfunction)(void))
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcVoidVoid == NULL)
  {
     TWIq[i].pointers.funcVoidVoid = TWIfunction;
     TWIq[i].Type = VoidVoid;
     break;
  }
}

void TWIqueue(void (*TWIfunction)(int,float),int arg1,float arg2)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntFloat == NULL)
  {
     TWIq[i].pointers.funcIntFloat = TWIfunction;
     TWIq[i].Type   = VoidIntFloat;
     TWIq[i].Int1   = arg1;
     TWIq[i].Float1 = arg2;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int),int arg1,int arg2)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntInt == NULL)
  {
     TWIq[i].pointers.funcIntInt = TWIfunction;
     TWIq[i].Type   = VoidIntInt;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,float),int arg1,int arg2,float arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntFloat == NULL)
  {
     TWIq[i].pointers.funcIntIntFloat = TWIfunction;
     TWIq[i].Type   = VoidIntIntFloat;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Float1 = arg3;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,bool),int arg1,int arg2,bool arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntBool == NULL)
  {
     TWIq[i].pointers.funcIntIntBool = TWIfunction;
     TWIq[i].Type   = VoidIntIntBool;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Bool1  = arg3;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,byte),int arg1,int arg2,byte arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntBool == NULL)
  {
     TWIq[i].pointers.funcIntIntByte = TWIfunction;
     TWIq[i].Type   = VoidIntIntByte;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Word1  = arg3;
     break;
  }  
}

void TWIqueue(void (*TWIfunction)(int,int,uint16_t),int arg1,int arg2,uint16_t arg3)
{
  for(int i=0;i<MaxQueued;i++) if(TWIq[i].pointers.funcIntIntWord == NULL)
  {
     TWIq[i].pointers.funcIntIntWord = TWIfunction;
     TWIq[i].Type   = VoidIntIntWord;
     TWIq[i].Int1   = arg1;
     TWIq[i].Int2   = arg2;
     TWIq[i].Word1  = arg3;
     break;
  }  
}


// The TWIstart function will start a TWI send cycle. This function returns the board address
// that was active when called, this value is -1 if the requested board address matches the
// current address.
int TWIstart(uint8_t add, int board, int cmd)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(cb == board) return -1;
  return cb;  
}
// This function will close a TWI send cycle. The value board is the value returned by the TWIstart function.
void TWIend(uint8_t add, int board)
{
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(board != -1) SelectBoard(board);
  ReleaseTWI();  
}

// TWI command routines used to send and received various data types to and from MIPS modules.

void TWIcmd(uint8_t add, int board, int cmd)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
}

void TWIcmd(uint8_t add, int board, int ch, int cmd)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(ch != -1) Wire.write(ch);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
}

void TWIsetBool(uint8_t add, int board, int cmd, bool flag)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(flag);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
}

void TWIsetByte(uint8_t add, int board, int cmd, byte bval)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(bval);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;  // 11/17/22
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
}

void TWIsetWord(uint8_t add, int board, int cmd, uint16_t wval)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(wval & 0xFF);
  Wire.write((wval >> 8) & 0xFF);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
}

void TWIset16bitInt(uint8_t add, int board, int cmd, int ival)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  uint8_t *b = (uint8_t *)&ival;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();  
}

void TWIset24bitInt(uint8_t add, int board, int cmd, int ival)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  uint8_t *b = (uint8_t *)&ival;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();  
}

void TWIsetInt(uint8_t add, int board, int cmd, int ival)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  uint8_t *b = (uint8_t *)&ival;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();  
}

void TWIsetFloat(uint8_t add, int board, int cmd, float fval)
{
  AcquireTWI();
  int cb=SelectedBoard();
  //SelectedBoard();
  SelectBoard(board);
  uint8_t *b = (uint8_t *)&fval;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();  
}

void TWIsetFloat(uint8_t add, int board, int ch, int cmd, float fval)
{
  AcquireTWI();
  int cb=SelectedBoard();
  //SelectedBoard();
  SelectBoard(board);
  uint8_t *b = (uint8_t *)&fval;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(ch != -1) Wire.write(ch);
  Wire.write(b[0]);
  Wire.write(b[1]);
  Wire.write(b[2]);
  Wire.write(b[3]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();  
}

bool TWIreadFloat(uint8_t add, int board,int ch,int cmd, float *value)
{
  byte *b;
  int  i=0;

  AcquireTWI();
  b = (byte *)value;
  int cb=SelectedBoard();
  SelectBoard(board);
  //AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(ch !=-1) Wire.write(ch);
  if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)4);
  while (Wire.available()) b[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if((fpclassify(*value) != FP_NORMAL) && (fpclassify(*value) != FP_ZERO))
  {
    *value = 0;
    return false;
  }
  if(i==4) return true;
  return false;
}

bool TWIreadFloat(uint8_t add, int board,int cmd, float *value)
{
  return TWIreadFloat(add,board,-1,cmd,value);
}

bool TWIread32bitInt(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  AcquireTWI();
  b = (byte *)value;
  int cb=SelectedBoard();
  SelectBoard(board);
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb);ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)4);
  while (Wire.available()) b[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==4) return true;
  return false;
}

bool TWIread24bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  AcquireTWI();
  *value = 0;
  b = (byte *)value;
  int cb=SelectedBoard();
  SelectBoard(board);
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)3);
  while (Wire.available()) b[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==3) return true;
  return false;
}

bool TWIread16bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  AcquireTWI();
  *value = 0;
  b = (byte *)value;
  int cb=SelectedBoard();
  SelectBoard(board);
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)2);
  while (Wire.available()) b[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==2) return true;
  return false;
}

// If cmd == -1 then only perform the read
bool TWIread8bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  AcquireTWI();
  *value = 0;
  b = (byte *)value;
  int cb=SelectedBoard();
  SelectBoard(board);
  AtomicBlock< Atomic_RestoreState > a_Block;
  if( cmd != -1)
  {
    Wire.beginTransmission(add);
    Wire.write(cmd);
    if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}
  }
  Wire.requestFrom((uint8_t)add, (uint8_t)1);
  while (Wire.available()) b[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==1) return true;
  return false;
}

bool TWIreadBlock(uint8_t add, int board,int cmd, void *ptr, int numbytes)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  if(cmd != 0)
  {
    Wire.beginTransmission(add);
    Wire.write(cmd);
    if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}  
  }
  uint8_t  *bptr;
  int i = 0;
  bptr = (uint8_t *)ptr;
  Wire.requestFrom((uint8_t)add, (uint8_t)numbytes);
  while (Wire.available()) bptr[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==numbytes) return true;
  return false;
}

bool TWIreadBlock(uint8_t add, int board,int ch, int cmd, void *ptr, int numbytes)
{
  AcquireTWI();
  int cb=SelectedBoard();
  SelectBoard(board);
  if(cmd != 0)
  {
    Wire.beginTransmission(add);
    Wire.write(cmd);
    Wire.write(ch);
    if(Wire.endTransmission() !=0) {if(cb != board) SelectBoard(cb); ReleaseTWI(); return false;}  
  }
  uint8_t  *bptr;
  int i = 0;
  bptr = (uint8_t *)ptr;
  Wire.requestFrom((uint8_t)add, (uint8_t)numbytes);
  while (Wire.available()) bptr[i++] = Wire.read();
  if(cb != board) SelectBoard(cb);
  ReleaseTWI();
  if(i==numbytes) return true;
  return false;
}

void setTWIspeed(int ch, int speed)
{
  if((ch<0) || (ch > 1)) BADARG;
  if(ch == 0)
  {
    WireDefaultSpeed = speed;
    Wire.setClock(speed);
  }
  else
  {
    Wire1DefaultSpeed = speed;
    Wire1.setClock(speed);
  }
}
void getTWIspeed(int ch)
{
  if((ch<0) || (ch > 1)) BADARG;
  SendACKonly;
  if(ch == 0) serial->println(WireDefaultSpeed);
  else serial->println(Wire1DefaultSpeed);
}

//
// TWIreadLine
//
// Reads a reply line from a TWI slave and stops the bus transaction as soon as the line terminator
// is seen, so the transaction takes only as long as the reply. Wire.requestFrom() cannot do this, it
// must be told the byte count up front and clocks every byte requested (32 bytes is about 3.3mS at
// 100KHz even if the reply is just "\n"). This function drives the SAM3X TWI hardware directly using
// the same twi.c calls that Wire.requestFrom() uses, but decides on the fly when to send STOP.
//
// Arguments:
//   twi    - WIRE_INTERFACE (Wire, TWI1) or WIRE1_INTERFACE (Wire1, TWI0). The port must already be
//            configured as master, i.e. WireX.begin() has been called.
//   addr   - 7 bit slave address
//   buf    - receives the reply, always null terminated
//   maxlen - size of buf. At most min(maxlen-1, TWI_RL_MAXBYTES) reply bytes are stored
//
// Returns:
//   >0  number of bytes stored in buf. A complete reply ends with '\n'. If the count is
//       min(maxlen-1,TWI_RL_MAXBYTES) with no '\n' the reply was truncated.
//    0  slave ACKed its address but had nothing queued (first byte was 0xFF). Slave replies never
//       start with 0xFF; an idle slave pads with 0xFF. Caller should wait a bit and poll again.
//   -1  address NAK (no device), or a byte timed out (stuck slave / clock stretch). The bus is
//       released with a STOP before returning.
//
// Notes:
//   - STOP is written after the byte that ends the reply has been read. The hardware sends STOP
//     at the end of the byte in flight, so one extra byte may be clocked and is discarded. The
//     drain loop handles zero, one or more extra bytes (if an ISR delayed the STOP write).
//   - The TWI status register is cleared on read, so all status bits are accumulated in 'sr'.
//   - This function does not take the TWI lock (AcquireTWI) or select a board; the caller does.
//   - Not ISR safe, same as the Arduino Wire library.
//
// UPDATE AND TEST PLAN
//   Status: written, NOT yet called. TWI1scmd() (Serial.cpp) and TWIscmd() still use requestFrom(32).
//
//   Before using:
//    1. Confirm the project does not override Wire.cpp / twi.c (build uses -Wl,--allow-multiple-definition).
//    2. Check no ISR or timer callback calls Wire1 code. Wire (TWI1) ISR use is protected by
//       AcquireTWI/ReleaseTWI/TWIqueue, Wire1 has no such protection. Files using Wire1: ARB, DCbias,
//       DCBcurrent, FAIMSfb, DMSDMSMB, Hardware, ESI, HVPSinterface, Table, ethernet, Serial.
//    3. Baseline: add temporary micros() timing to TWI1scmd (send start, first '\n' received) and
//       record times for a few commands, including an unused address (expect ~250mS timeout).
//    4. Add an endTransmission() != 0 check to TWI1scmd, reply "?\n" and return, so a missing
//       device fails in ~1mS instead of 250mS. Independent of this function.
//
//   Bench tests (logic analyzer or scope on SDA1/SCL1, Wire1 = pins 70/71):
//    a. Slave that replies "\n": expect START, addr+R, ~2-3 bytes, NACK, STOP. Note whether the
//       hardware clocks an extra byte after the '\n' (it may NAK the '\n' byte itself).
//    b. Slave that replies a 20 byte string + '\n': same, stop right after the '\n'.
//    c. Slave not ready (all 0xFF): returns 0 after 2 bytes at most.
//    d. Address with no device: returns -1, bus idle (SDA/SCL high) afterwards. Verify the bus is
//       released without needing the STOP that the NAK path waits on; if not, add TWI_Stop().
//    e. Reply longer than 31 bytes with no '\n': truncated at the cap, STOP sent, bus released.
//    f. Slave that stretches the clock longer than TWI_RL_BYTE_TIMEOUT_US: returns -1 and bus recovers.
//    g. 400KHz (STWI) and 100KHz. At 400KHz the STOP write window is ~22uS per byte; check with
//       interrupts active (ADC DMA, timers) that the extra-byte count stays small.
//
//   Integration:
//    1. TWI1scmd(): replace the requestFrom(32) loop with TWIreadLine(WIRE1_INTERFACE,...) retrying
//       on 0 (with ~1mS delay) until the existing 250mS limit; send buf to host; -1 -> "?\n".
//    2. Compare timings against the baseline from step 3.
//    3. TWIscmd(): same change with WIRE_INTERFACE, keeping SelectBoard/AcquireTWI/ReleaseTWI.
//    4. Update MIPScommands.txt only if the command behaviour (error replies) changes.
//
#define TWI_RL_MAXBYTES        32
#define TWI_RL_BYTE_TIMEOUT_US 5000

int TWIreadLine(Twi *twi, uint8_t addr, char *buf, int maxlen)
{
  uint32_t sr = 0;
  int      n = 0;
  int      cap;
  int      result;
  bool     stopSent = false;
  uint8_t  b;
  uint32_t t0;

  if((buf == NULL) || (maxlen < 2)) return -1;
  buf[0] = 0;
  cap = maxlen - 1;
  if(cap > TWI_RL_MAXBYTES) cap = TWI_RL_MAXBYTES;
  result = -1;

  TWI_StartRead(twi, addr, 0, 0);
  t0 = micros();
  while(true)
  {
    sr |= twi->TWI_SR;
    if(sr & TWI_SR_RXRDY)
    {
      b = TWI_ReadByte(twi);
      sr &= ~TWI_SR_RXRDY;
      t0 = micros();
      if(stopSent) continue;          // extra byte clocked before STOP took effect, discard it
      buf[n++] = (char)b;
      buf[n] = 0;
      if((n == 1) && (b == 0xFF))     // Slave has nothing queued
      {
        n = 0;
        buf[0] = 0;
        result = 0;
        TWI_Stop(twi);
        stopSent = true;
      }
      else if((b == '\n') || (n >= cap))
      {
        result = n;
        TWI_Stop(twi);
        stopSent = true;
      }
      continue;
    }
    if(stopSent)
    {
      // Wait for the transfer to complete, extra bytes were discarded above
      if(sr & TWI_SR_TXCOMP) return result;
    }
    else if(sr & TWI_SR_NACK)
    {
      // Address NAK, no device. Wait for the hardware to finish and release the bus.
      t0 = micros();
      while(!(twi->TWI_SR & TWI_SR_TXCOMP))
      {
        if((micros() - t0) > TWI_RL_BYTE_TIMEOUT_US) break;
      }
      buf[0] = 0;
      return -1;
    }
    if((micros() - t0) > TWI_RL_BYTE_TIMEOUT_US)
    {
      // Timeout, release the bus and report failure
      if(!stopSent) TWI_Stop(twi);
      t0 = micros();
      while(!(twi->TWI_SR & TWI_SR_TXCOMP))
      {
        if((micros() - t0) > TWI_RL_BYTE_TIMEOUT_US) break;
      }
      return (stopSent) ? result : -1;
    }
  }
}

// Wire1 helpers for firmware that talks to a binary TWI device directly, i.e. the QUAD scan reading
// an IsolatedElectrometer. Both return false on any NAK or short read.
//
// TWI1sendCmd writes the single command byte cmd, no data.
// TWI1readCmd writes the single command byte cmd, then reads n bytes into buf.
bool TWI1sendCmd(uint8_t add, uint8_t cmd)
{
  Wire1.beginTransmission(add);
  Wire1.write(cmd);
  return (Wire1.endTransmission() == 0);
}

bool TWI1readCmd(uint8_t add, uint8_t cmd, uint8_t *buf, int n)
{
  if(!TWI1sendCmd(add, cmd)) return false;
  if(Wire1.requestFrom((int)add, n) != n) return false;
  for(int i=0;i<n;i++) buf[i] = Wire1.read();
  return true;
}

// The following codes support the GTWI and STWI commands
//
// Add generic TWI commands allowing MIPS controller to talk to TWI attached modules or
// devices.
//  STWI,TWI hex command string,value
//  GTWI,TWI hex command string
//    The TWI command hex string has the following format:
//    AABB..BBTT
//    AA defines the Wire channel to use and the board address if we are using the Wire port.
//    The LS 4 bits define the wire channel, 0=Wire, 1=Wire1
//    The MS 4 bits define the board address, 0 to 1. This only applies to the Wire port
//    BB..BB is the hex TWI command array of bytes, the first by is the TWI device address.
//    TT defines the variable type, the MS 4 bits define the number of bytes to read and the
//    LS 4 bits define the data type:
//        0 = nothing, no variable
//        1 = bool
//        2 = int
//        3 = char
//        4 = byte
//        5 = word
//        6 = float
// In MIPS host app the custom control class supports the TWI command strings. The
// TWI command format is: GTWI_"TWI hex command string". 

/**
 * Converts a single hex character to its integer value.
 */
int8_t hexCharToInt(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/**
 * Converts a hex string to a byte array.
 * Handles odd-length strings by assuming a leading zero.
 * * @param hexStr: The input string (e.g., "32F")
 * @param output: The byte array to fill
 * @return: The number of bytes written to the array
 */
int hexStringToBytes(const char* hexStr, uint8_t* output) {
    size_t len = strlen(hexStr);
    int strIdx = 0;
    int outIdx = 0;

    // Handle odd length by processing the first character alone
    if (len % 2 != 0) {
        output[outIdx++] = hexCharToInt(hexStr[strIdx++]);
    }

    // Process the remaining characters in pairs
    while (hexStr[strIdx] != '\0') {
        uint8_t high = hexCharToInt(hexStr[strIdx++]);
        uint8_t low  = hexCharToInt(hexStr[strIdx++]);
        output[outIdx++] = (high << 4) | low;
    }

    return outIdx;
}

void stwi(char *cmdString, char *value)
{
  int byteCount = (strlen(cmdString) + 1) / 2;
  uint8_t buffer[byteCount],bptr[4];
  TwoWire *wire;

  hexStringToBytes(cmdString, buffer);
  // Select the wire interface
  if((buffer[0]&0x0F) == 0) 
  {
    wire = &Wire;
    // Select the board address
    if((buffer[0]&0xF0) == 0x00) SelectBoard(0);
    else if((buffer[0]&0xF0) == 0x10) SelectBoard(1);
    else BADARG;
  }
  else if((buffer[0]&0x0F) == 1) wire = &Wire1;
  else BADARG;
  // Address the TWI device and send the bytes
  wire->beginTransmission(buffer[1]);
  for(int i=2;i<byteCount-1;i++) wire->write(buffer[i]);
  // Determine the number of bytes we need to send to the device
  int num = (buffer[byteCount-1] & 0xF0) >> 4;
  if(num==0)
  {
    wire->endTransmission();
    SendACK;
    return;
  }
  // Use the type value to send data to the device
  if((buffer[byteCount-1] & 0x0F) == 1)  // bool
  {
    if(strcmp(value,"TRUE")==0) wire->write(true);
    else if(strcmp(value,"FALSE")==0) wire->write(false);
  }
  else if((buffer[byteCount-1] & 0x0F) == 2)  // int
  {
    sscanf(value,"%d",(int *)bptr);
    for(int i=0;i<4;i++) wire->write(bptr[i]);
  }
  else if((buffer[byteCount-1] & 0x0F) == 3)  // char
  {
    wire->write(value[0]);
  }
  else if((buffer[byteCount-1] & 0x0F) == 4)  // byte
  {
    sscanf(value,"%u",(int *)bptr);
    wire->write(bptr[0]);
  }
  else if((buffer[byteCount-1] & 0x0F) == 5)  // word
  {
    sscanf(value,"%u",(int *)bptr);
    wire->write(bptr[0]);
    wire->write(bptr[1]);
  }
  else if((buffer[byteCount-1] & 0x0F) == 6)  // float
  {
    sscanf(value,"%f",(float *)bptr);
    for(int i=0;i<4;i++) wire->write(bptr[i]);
  } 
  wire->endTransmission();
  SendACK;
}

// Get TWI command processor. The string defines the TWI command bytes and the expected return format if
// the command returns data.
// 01208346
// Hex sting characters
// 012345...67
// 0  = 0 or 1 for board select 0 or 1
// 1  = 0 for wire, 1 for wire1
// 23 = TWI device address
// 45 = command bytes
// 67 = expected return format
//      6 = number of bytes
//      7 = type, 1=bool,2=int,3=char,4=byte,5=word,6=float
//
void gtwi(char *cmdString)
{
  int byteCount = (strlen(cmdString) + 1) / 2;
  uint8_t buffer[byteCount],bptr[16];   // The read count is a 4 bit field, up to 15 bytes
  TwoWire *wire;

  hexStringToBytes(cmdString, buffer);
  // Select the wire interface
  if((buffer[0]&0x0F) == 0) 
  {
    wire = &Wire;
    // Select the board address
    if((buffer[0]&0xF0) == 0x00) SelectBoard(0);
    else if((buffer[0]&0xF0) == 0x10) SelectBoard(1);
    else BADARG;
  }
  else if((buffer[0]&0x0F) == 1) wire = &Wire1;
  else BADARG;
  // Address the TWI device and send the bytes
  wire->beginTransmission(buffer[1]);
  for(int i=2;i<byteCount-1;i++) wire->write(buffer[i]);
  wire->endTransmission();
  // Determine the number of bytes we need to read from the device
  int num = (buffer[byteCount-1] & 0xF0) >> 4;
  if(num==0)
  {
    SendACKonly;
    serial->println("");
    return;
  }
  // Read from device
  wire->requestFrom(buffer[1], (uint8_t)num);
  int i = 0;
  memset(bptr, 0, sizeof(bptr));
  while (wire->available() && (i < (int)sizeof(bptr))) bptr[i++] = wire->read();
  // Use the type value to print the data read from the device
  if((buffer[byteCount-1] & 0x0F) == 1)  // bool
  {
    SendACKonly;
    if(bptr[0]) serial->println("TRUE");
    else serial->println("FALSE");
  }
  else if((buffer[byteCount-1] & 0x0F) == 2)  // int
  {
    SendACKonly;
    serial->println(*(int *)bptr);
  }
  else if((buffer[byteCount-1] & 0x0F) == 3)  // char
  {
    SendACKonly;
    serial->println(*(char *)bptr);
  }
  else if((buffer[byteCount-1] & 0x0F) == 4)  // byte
  {
    SendACKonly;
    serial->println(*(uint8_t *)bptr);
  }
  else if((buffer[byteCount-1] & 0x0F) == 5)  // word
  {
    SendACKonly;
    serial->println(*(uint16_t *)bptr);
  }
  else if((buffer[byteCount-1] & 0x0F) == 6)  // float
  {
    SendACKonly;
    serial->println(*(float *)bptr);
  }
  else BADARG;
}
