/*
 * Serial.h
 *
 * Created: 10/16/2014
 *  Author: Gordon Anderson
 */
#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif


//extern Serial_ *serial;
extern Stream *serial;

extern bool SerialMute;

extern bool LEDoverride;
extern int  LEDstate;

//extern HardwareSerial *serial;

// Ring buffer size
#define RB_BUF_SIZE		4096

extern char *SelectedACKonlyString;

#define SendNAK {if(!SerialMute) serial->write("\x15?\n\r");}
#define SendACK {if(!SerialMute) serial->write("\x06\n\r");}
//#define SendACKonly {if(!SerialMute) serial->write("\x06");}
#define SendACKonly {if(!SerialMute) serial->write(SelectedACKonlyString);}
#define SendERR {if(!SerialMute) serial->write("\x15?\n\r");}
#define SendBSY {if(!SerialMute) serial->write("\x15?\n\r");}

// The serial receiver uses Xon and Xoff to control input data from the source
#define XON   0x11
#define XOFF  0x13
#define EOF   0x1A
#define ACK   0x06
#define NAK   0x15

typedef struct
{
  char  Buffer[RB_BUF_SIZE];
  int   Tail;
  int   Head;
  int   Count;
  int   Commands;
} Ring_Buffer;

enum CmdTypes
{
  CMDstr,		        // Sends a string
  CMDint,			      // Sends an int
  CMDfloat,			    // Sends an float
  CMDbool,          // Sends or receives a bool, TRUE or FALSE
  CMDfunction,		  // Calls a function with 0,1,or 2 int args
  CMDfunctionStr,		// Calls a function with pointer to str arg
  CMDfun2int1flt,		// Calls a function with 2 int args followed by 1 float arg
  CMDna
};

enum PCstates
{
  PCcmd,			// Looking for a command token
  PCarg1,			// Looking for int arg1
  PCarg2,			// Looking for int arg2
  PCarg3,			// Looking for int arg3
  PCargStr,			// Looking for string arg
  PCend,
  PCna
};

union functions
{
  char  *charPtr;
  int   *intPtr;
  float *floatPtr;
  bool  *boolPtr;
  void  (*funcVoid)();
  void  (*func1int)(int);
  void  (*func2int)(int, int);
  void  (*func1str)(char *);
  void  (*func2str)(char *, char *);
  void  (*func2int1flt)(int, int, float);
};

typedef struct
{
  const char  		*Cmd;
  enum	CmdTypes	Type;
  int   		NumArgs;
  union functions pointers;
} Commands;

extern Ring_Buffer  RB;
extern char Version[];

// Function prototypes
void SetThreadEnable(char *, char *);
void ListThreads(void);
void SerialInit(void);
char *GetToken(bool ReturnComma);
int  ProcessCommand(void);
void GetNumChans(char *cmd);
void RB_Init(Ring_Buffer *);
int RB_Size(Ring_Buffer *);
char RB_Put(Ring_Buffer *, char);
char RB_Get(Ring_Buffer *);
char RB_Next(Ring_Buffer *);
int RB_Commands(Ring_Buffer *);
void PutCh(char ch);
void MacroRecord(char *filename);
void MacroStop(void);
char *MacroBuildList(char *current);
void MacroList(void);
void MacroDelete(char *filename);
void MacroPlay(char *filename, bool optional = false);
void Mute(char *cmd);
void GetCommands(void);
void DelayCommand(int dtime);

// Prototypes for external functions called
void SAVEparms(void);
void RFnumber(void);
void DCbiasSet(char *chan, char *value);
void DCbiasRead(int chan);
void DCbiasReadV(int chan);
void DCbiasSetFloat(char *chan, char *value);
void DCbiasReadFloat(int chan);
void DCbiasReadMin(int chan);
void DCbiasReadMax(int chan);
void DCbiasNumber(void);

#endif /* SERIAL_H_ */



