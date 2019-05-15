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
#include "TWIext.h"

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
  // Do resect procedure for both board addresses
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
      for(int i = 0; i < 10; i++)
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
    if ((val & 0x80) != 0) TWI_SDA_HI;
    else TWI_SDA_LOW;
    val = val << 1;
    TWI_SCL_HI;
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
  int8_t val, r;

//  AtomicBlock< Atomic_RestoreState > a_Block;
  val = 0;
  TWI_SDA_IN;
  for (int i = 0; i < 8; i++)
  {
    val = val << 1;
    TWI_SCL_HI;
    if (TWI_SDA_data == HIGH) val |= 1;
    TWI_SCL_LOW;
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
  TWI_RESET();
  Wire.begin();
  Wire.setClock(100000);
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
    for(int i=0;i<MaxQueued;i++)
    {
      if(TWIq[i].pointers.funcVoidVoid != NULL) 
      {
        if(TWIq[i].Type == VoidVoid) TWIq[i].pointers.funcVoidVoid();
        else if(TWIq[i].Type == VoidIntIntFloat) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Float1);
        else if(TWIq[i].Type == VoidIntIntBool) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Bool1);
        else if(TWIq[i].Type == VoidIntIntWord) TWIq[i].pointers.funcIntIntFloat(TWIq[i].Int1,TWIq[i].Int2,TWIq[i].Word1);
        TWIq[i].pointers.funcVoidVoid = NULL;
        TWIq[i].Type = Empty;
      }
    }
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
  SelectBoard(board);
  AcquireTWI();
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(bval);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
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
  SelectBoard(board);
  AcquireTWI();
  uint8_t *b = (uint8_t *)&ival;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  Wire.write(b[0]);
  Wire.write(b[1]);
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    Wire.endTransmission();
  }
  ReleaseTWI();  
}

void TWIsetInt(uint8_t add, int board, int cmd, int ival)
{
  SelectBoard(board);
  AcquireTWI();
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
  ReleaseTWI();  
}

void TWIsetFloat(uint8_t add, int board, int cmd, float fval)
{
  AcquireTWI();
  int cb=SelectedBoard();
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

bool TWIreadFloat(uint8_t add, int board,int cmd, float *value)
{
  byte *b;
  int  i=0;

  b = (byte *)value;
  SelectBoard(board);
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)4);
  while (Wire.available()) b[i++] = Wire.read();
  ReleaseTWI();
  if(fpclassify(*value) != FP_NORMAL)
  {
    *value = 0;
    return false;
  }
  if(i==4) return true;
  return false;
}

bool TWIread32bitInt(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  b = (byte *)value;
  SelectBoard(board);
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)4);
  while (Wire.available()) b[i++] = Wire.read();
  ReleaseTWI();
  if(i==4) return true;
  return false;
}

bool TWIread24bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  *value = 0;
  b = (byte *)value;
  SelectBoard(board);
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)3);
  while (Wire.available()) b[i++] = Wire.read();
  ReleaseTWI();
  if(i==3) return true;
  return false;
}

bool TWIread16bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  *value = 0;
  b = (byte *)value;
  SelectBoard(board);
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)2);
  while (Wire.available()) b[i++] = Wire.read();
  ReleaseTWI();
  if(i==2) return true;
  return false;
}

bool TWIread8bitUnsigned(uint8_t add, int board,int cmd, int *value)
{
  byte *b;
  int  i=0;

  *value = 0;
  b = (byte *)value;
  SelectBoard(board);
  AcquireTWI();
  AtomicBlock< Atomic_RestoreState > a_Block;
  Wire.beginTransmission(add);
  Wire.write(cmd);
  if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}
  Wire.requestFrom((uint8_t)add, (uint8_t)1);
  while (Wire.available()) b[i++] = Wire.read();
  ReleaseTWI();
  if(i==1) return true;
  return false;
}

bool TWIreadBlock(uint8_t add, int board,int cmd, void *ptr, int numbytes)
{
  SelectBoard(board);
  if(cmd != 0)
  {
    Wire.beginTransmission(add);
    Wire.write(cmd);
    if(Wire.endTransmission() !=0) {ReleaseTWI(); return false;}  
  }
  uint8_t  *bptr;
  int i = 0;
  bptr = (uint8_t *)ptr;
  Wire.requestFrom((uint8_t)add, (uint8_t)numbytes);
  while (Wire.available()) bptr[i++] = Wire.read();
  ReleaseTWI();
  if(i==numbytes) return true;
  return false;
}
