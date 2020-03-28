#ifndef TWIext_h
#define TWIext_h

#define MaxQueued 10

enum TWIcbType
{
  Empty,
  VoidVoid,
  VoidIntIntByte,
  VoidIntFloat,
  VoidIntIntFloat,
  VoidIntIntBool,
  VoidIntIntWord
};

union TWIfunctions
{
  void  (*funcVoidVoid)(void);
  void  (*funcIntIntByte)(int, int, byte);
  void  (*funcIntFloat)(int, float);
  void  (*funcIntIntFloat)(int, int, float);
  void  (*funcIntIntBool)(int, int, bool);
  void  (*funcIntIntWord)(int, int, uint16_t);
};

typedef struct 
{
  TWIcbType             Type;
  union   TWIfunctions  pointers;
  int                   Int1,Int2;
  float                 Float1;
  bool                  Bool1;
  uint16_t              Word1;
} TWIqueueEntry;

#define TWI_SCL_OUT            pinMode(TWI_SCL,OUTPUT)
#define TWI_SDA_OUT            pinMode(TWI_SDA,OUTPUT)
#define TWI_SDA_IN             pinMode(TWI_SDA,INPUT)
#define TWI_SCL_HI             digitalWrite(TWI_SCL,HIGH)
#define TWI_SCL_LOW            digitalWrite(TWI_SCL,LOW)
#define TWI_SDA_HI             digitalWrite(TWI_SDA,HIGH)
#define TWI_SDA_LOW            digitalWrite(TWI_SDA,LOW)
#define TWI_SDA_data           digitalRead(TWI_SDA)

// Function prototypes for software TWI interface
void   TWI_RESET(void);
void   TWI_START(void);
void   TWI_STOP(void);
bool   TWI_WRITE(int8_t val);
int8_t TWI_READ(bool Reply);
void   TWIerror(void);
void   TWIreset(void);

// Function prototypes for TWI acquire / release and function queqing system.
void TWI_RESET(void);
bool AcquireTWI(void);
void ReleaseTWI(void);
void TWIqueue(void (*TWIfunction)());
void TWIqueue(void (*TWIfunction)(int,int,float),int arg1,float arg3);
void TWIqueue(void (*TWIfunction)(int,int,float),int arg1,int arg2,float arg3);
void TWIqueue(void (*TWIfunction)(int,int,bool),int arg1,int arg2,bool arg3);
void TWIqueue(void (*TWIfunction)(int,int,byte),int arg1,int arg2,byte arg3);
void TWIqueue(void (*TWIfunction)(int,int,uint16_t),int arg1,int arg2,uint16_t arg3);

int TWIstart(uint8_t add, int board, int cmd);
// These macros will send various data types using the TWI port
#define  TWIBYTE(b)  {Wire.write(b);}
#define  TWIBOOL(b)  {Wire.write(b);}
#define  TWI16BIT(w) {Wire.write(w & 0xFF); Wire.write((w >> 8) & 0xFF);}
#define  TWI24BIT(w) {Wire.write(w & 0xFF); Wire.write((w >> 8) & 0xFF); Wire.write((w >> 16) & 0xFF);}
#define  TWI32BIT(w) {Wire.write(w & 0xFF); Wire.write((w >> 8) & 0xFF); Wire.write((w >> 16) & 0xFF); Wire.write((w >> 24) & 0xFF);}
//#define  F2INT(f)    (*(int *)&f;)
//#define  TWIFLOAT(w) {Wire.write(F2INT((w)) & 0xFF); Wire.write((F2INT((w)) >> 8) & 0xFF); Wire.write((F2INT((w)) >> 16) & 0xFF); Wire.write((F2INT((w)) >> 24) & 0xFF);}
#define  TWIFLOAT(w) {Wire.write((w) & 0xFF); Wire.write(((w) >> 8) & 0xFF); Wire.write(((w) >> 16) & 0xFF); Wire.write(((w) >> 24) & 0xFF);}
void TWIend(uint8_t add, int board);

// Function prototypes for TWI data types read and write functions
void TWIcmd(uint8_t add, int board, int cmd);
void TWIsetBool(uint8_t add, int board, int cmd, bool flag);
void TWIsetByte(uint8_t add, int board, int cmd, byte bval);
void TWIsetWord(uint8_t add, int board, int cmd, uint16_t wval);
void TWIset16bitInt(uint8_t add, int board, int cmd, int ival);
void TWIsetInt(uint8_t add, int board, int cmd, int ival);
void TWIsetFloat(uint8_t add, int board, int cmd, float fval);
bool TWIreadFloat(uint8_t add, int board,int cmd, float *value);
bool TWIread32bitInt(uint8_t add, int board,int cmd, int *value);
bool TWIread24bitUnsigned(uint8_t add, int board,int cmd, int *value);
bool TWIread16bitUnsigned(uint8_t add, int board,int cmd, int *value);
bool TWIread8bitUnsigned(uint8_t add, int board,int cmd, int *value);
bool TWIreadBlock(uint8_t add, int board,int cmd, void *ptr, int numbytes);

#endif
