#include "DCBswitch.h"
#include "Arduino.h"
#include <Wire.h>
#include "Hardware.h"

#if DCBswitchCode

int           NumberOfDCBSmodules=0;
DCBswitch     *DCBswitchData[2]   = {NULL,NULL};
DCBswitchRB   *DCBswitchrb[2]     = {NULL,NULL};
Thread        DCBswitchThread     = Thread();

const Commands  DCBSCmdArray[] = {
  {"SDCBEE",CMDfunctionStr    ,2,(char *)setDCBSeepromEnable},
  {"SDCBS",CMDfunctionStr     ,2,(char *)setDCBSvoltage},
  {"GDCBS",CMDfunction        ,1,(char *)getDCBSvoltage},
  {"GDCBSV",CMDfunction       ,1,(char *)getDCBSreadback},
  {"SDCBSPWR",CMDfunctionStr  ,2,(char *)setDCBSpwr},                  // Set DC power supply, ON or OFF
  {"GDCBSPWR",CMDfunction     ,1,(char *)getDCBSpwr},                  // Return DC power supply status, ON or OFF
  {"SDCBSIO",CMDfunctionLine  ,0,(char *)setDCBSdout},                 // Set digital output,A|B|C|D,0|1
  {"GDCBSIO",CMDfunctionStr   ,2,(char *)getDCBSdin},                  // Return digital input states,A|B|C|D|Q|R|S|T
  {"SDCBSSW",CMDfunctionStr   ,2,(char *)setDCBSsw},                   // Set FET state,switch#,0|1
  {"GDCBSSW",CMDfunction      ,1,(char *)getDCBSsw},                   // Return FET state,switch#
  {"PDCBSSW",CMDfunction      ,2,(char *)pulseDCBSsw},                 // Pulse FET switch,switch#,width in uS
  {"SDCBSTRGSW",CMDfunctionStr,2,(char *)setDCBStrigger},              // Set FET switch follow input,switch#,0|Q|R|S|T
  {"GDCBSTRGSW",CMDfunction   ,1,(char *)getDCBStrigger},              // Return FET switch follow input,switch#
  {"SDCBSRBTST",CMDfunctionStr,2,(char *)setDCBSrbTest},               // Enable read back testing if TRUE
  {"GDCBSRBTST",CMDfunction   ,1,(char *)getDCBSrbTest},               // Enable read back testing if TRUE
  // Pulse generator command
  {"SPGENA",CMDfunctionStr    ,2,(char *)setPGena},                    // Set Pulse Generator enable,Ch,TRUE|FALSE
  {"GPGENA",CMDfunction       ,1,(char *)getPGena},                    // Return Pulse Generator enable,Ch
  {"SPGRET",CMDfunctionStr    ,2,(char *)setPGretrig},                 // Set Pulse Generator re-trigger,Ch,TRUE|FALSE
  {"GPGRET",CMDfunction       ,1,(char *)getPGretrig},                 // Return Pulse Generator re-trigger,Ch
  {"SPGARMT",CMDfunctionStr   ,2,(char *)setPGarmT},                   // Set Pulse Generator arm input,Ch,0|Q|R|S|T
  {"GPGARMT",CMDfunction      ,1,(char *)getPGarmT},                   // Return Pulse Generator arm input,Ch
  {"SPGARML",CMDfunctionStr   ,2,(char *)setPGarmL},                   // Set Pulse Generator arm level,Ch,CHANGE|RISING|FALLING
  {"GPGARML",CMDfunction      ,1,(char *)getPGarmL},                   // Return Pulse Generator arm level,Ch
  {"SPGTRG",CMDfunctionStr    ,2,(char *)setPGtrig},                   // Set Pulse Generator trigger input,Ch,0|Q|R|S|T
  {"GPGTRG",CMDfunction       ,1,(char *)getPGtrig},                   // Return Pulse Generator trigger input,Ch
  {"SPGTRGL",CMDfunctionStr   ,2,(char *)setPGtrigL},                  // Set Pulse Generator trigger level,Ch,CHANGE|RISING|FALLING
  {"GPGTRGL",CMDfunction      ,1,(char *)getPGtrigL},                  // Return Pulse Generator trigger level,Ch
  {"SPGSKIP",CMDfunction      ,2,(char *)setPGskip},                   // Set Pulse Generator trigger skip count,Ch,Count
  {"GPGSKIP",CMDfunction      ,1,(char *)getPGskip},                   // Return Pulse Generator trigger skip count,Ch
  {"SPGDLY",CMDfunction       ,2,(char *)setPGdly},                    // Set Pulse Generator trigger delay in uS,Ch,Delay
  {"GPGDLY",CMDfunction       ,1,(char *)getPGdly},                    // Return Pulse Generator trigger delay in uS,Ch
  {"SPGWDTH",CMDfunction      ,2,(char *)setPGwdth},                   // Set Pulse Generator pulse width in uS in uS,Ch,Width
  {"GPGWDTH",CMDfunction      ,1,(char *)getPGwdth},                   // Return Pulse Generator pulse width in uS in uS,Ch
  {"SPGTRGO",CMDfunctionStr   ,2,(char *)setPGtrgOut},                 // Set Pulse Generator trigger output,Ch,0|A|B|D|D
  {"GPGTRGO",CMDfunction      ,1,(char *)getPGtrgOut},                 // Return Pulse Generator trigger output,Ch
  {"SPGTRGF",CMDfunctionStr   ,2,(char *)setPGtrhFET},                 // Set Pulse Generator FET switch output,Ch,0|1|2
  {"GPGTRGF",CMDfunction      ,1,(char *)getPGtrhFET},                 // Return Pulse Generator FET switch output,Ch
  {"SPGOUTCH",CMDfunction     ,2,(char *)setPGoutCh},                  // Set Pulse Generator DC bias output,Ch,0|1|2|3|4
  {"GPGOUTCH",CMDfunction     ,1,(char *)getPGoutCh},                  // Return Pulse Generator DC bias output,Ch
  {"SPGPV",CMDfunctionStr     ,2,(char *)setPGvoltage},                // Set Pulse Generator pulse voltage,Ch,Voltage
  {"GPGPV",CMDfunction        ,1,(char *)getPGvoltage},                // Return Pulse Generator pulse voltage,Ch
  {"SPGNUM",CMDfunction       ,2,(char *)setPGnum},                    // Set Pulse Generator number of pulses,Ch,Count
  {"GPGNUM",CMDfunction       ,1,(char *)getPGnum},                    // Return Pulse Generator number of pulses,Ch
  {"SPGPER",CMDfunction       ,2,(char *)setPGperiod},                 // Set Pulse Generator period in uS,Ch,Value
  {"GPGPER",CMDfunction       ,1,(char *)getPGperiod},                 // Return Pulse Generator period in uS,Ch 
  // Ramp commands
//  {"SRPENA",CMDfunction       ,1,(char *)setRPena},                    // Set ramp generator enable, TRUE or FALSE
//  {"GRPENA",CMDfunction       ,1,(char *)getRPena},                    // Return ramp generator enable, TRUE or FALSE
//  {"SRPTRG",CMDfunctionStr    ,2,(char *)setRPtrig},                   // Set Ramp Generator trigger input,Ch,0|Q|R|S|T
//  {"GRPTRG",CMDfunction       ,1,(char *)getRPtrig},                   // Return Ramp Generator trigger input,Ch
//  {"SRPTRGL",CMDfunctionStr   ,2,(char *)setRPtrigL},                  // Set Ramp Generator trigger level,Ch,CHANGE|RISING|FALLING
//  {"GRPTRGL",CMDfunction      ,1,(char *)getRPtrigL},                  // Return Ramp Generator trigger level,Ch
//  {"SRPDCBC",CMDfunctionStr   ,2,(char *)setRPDCBchan},                // Set ramp channel DC bias output number,ramp ch, DCB ch
//  {"GRPDCBC",CMDfunction      ,1,(char *)getRPDCBchan},                // Return ramp channel DC bias output number,ramp ch, DCB ch
//  {"SRPCHENA",CMDfunctionSTR  ,2,(char *)setRPchENA},                  // Set ramp channel enable, ramp ch
//  {"GRPCHENA",CMDfunction     ,1,(char *)getRPchENA},                  // Return ramp channel enable, ramp cn
//  {"SRAMP",CMDfunctionLine    ,0,(char *)setRPramp},                   // Set ramp channel data, ramp ch, time, value....
//  {"GRAMP",CMDfunctionLine    ,0,(char *)getRPramp},                   // Return ramp channel data, ramp ch, time, value....
//  {"SRPPER",CMDfunctionStr    ,1,(char *)setRPperiod},                 // Set ramp period, ms
//  {"GRPPER",CMDfunction       ,0,(char *)getRPperiod},                 // Return ramp period, mS
//  {"SRPCYC",CMDfunction       ,1,(char *)setRPcycl},                   // Set ramp number of cycles
//  {"GRPCYC",CMDfunction       ,0,(char *)getRPcycl},                   // Return ramp number of cycles
  {0},
};

CommandList DCBSCmdList = { (Commands *)DCBSCmdArray, NULL };

void DCBswitch_init(int8_t Board, int8_t addr)
{
  // Allocate the data structure
  if(DCBswitchData[Board] != NULL) return;
  DCBswitchData[Board] = new DCBswitch;
  DCBswitchData[Board]->TWIadd = addr;
  DCBswitchrb[Board] = new DCBswitchRB;
  // Read the data structure from the DCBswitch module. Note, this module uses the enhanced
  // EEPROM read function
  RestoreDCBswitch(Board);
  if(NumberOfDCBSmodules == 0)
  {
    // Here if this is the first module
    // Add the commands to the command processor
    AddToCommandList(&DCBSCmdList);
    // Add processing loop
    // Configure Threads
    DCBswitchThread.setName("DCBswitchCtrl");
    DCBswitchThread.onRun(DCBswitchCtrl_loop);
    DCBswitchThread.setInterval(100);
    // Add threads to the controller
    control.add(&DCBswitchThread);
  }
  NumberOfDCBSmodules++;
}

void DCBswitchCtrl_loop(void)
{
  DCBswitchRB rb;

  for(int brd=0;brd<2;brd++)
  {
    if(DCBswitchrb[brd] != NULL)
    {
      if(TWIreadBlock(DCBswitchData[brd]->TWIadd, brd, 0x80 | GET_DCBS_RBTST, (void *)&rb, sizeof(DCBswitchRB)))
      {
        for(int i=0;i<4;i++) if((fpclassify(rb.rb[i]) == FP_NORMAL) || (fpclassify(rb.rb[i]) == FP_ZERO)) DCBswitchrb[brd]->rb[i] = rb.rb[i];
      }
      else
      {
        // Here with TWI error so reset the TWI interface
        TWIerror();
      }
    }
  }
}

bool RestoreDCBswitch(int8_t brd)
{
  DCBswitch dcbs;
  
  if(DCBswitchData[brd] == NULL) return false;
  SelectBoard(brd);
  if(ReadEEPROMext(&dcbs, DCBswitchData[brd]->TWIadd, 0, sizeof(DCBswitch)) == 0)
  {
    if(strcmp(dcbs.Name,"DCBswitch") == 0)
    {
      // Here if the name matches so copy the data to the targe data structure
      *DCBswitchData[brd] = dcbs;
      return true;
    } 
    else return false;
  }
  return false;
}

// Host command functions

// Convertes module number, 1 or 2, into board address 0 or 1.
int DCBSmod2brd(int mod)
{
  if((mod == 1) && (DCBswitchData[0] != NULL)) return 0;
  if((mod == 1) && (DCBswitchData[1] != NULL)) return 1;
  if((mod == 2) && (DCBswitchData[0] != NULL) && (DCBswitchData[1] != NULL)) return 1;
  return -1;
}

int DCBSmod2brd(char *module)
{
  String token = module;
  return DCBSmod2brd(token.toInt());
}

// Accepts the channel number (int) 1-number of channel and the number of channels per
// module and returns the board number of -1 if error. 
// Send host nak message on error
int getDCBSbrd(int ch, int chPmodule)
{
  int maxCh = 0;
  if(DCBswitchData[0] != NULL) maxCh += chPmodule;
  if(DCBswitchData[1] != NULL) maxCh += chPmodule;
  if((ch < 1) || (ch > maxCh)) { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
  if((ch <= chPmodule) && (DCBswitchData[0] != NULL)) return 0;
  return 1;
}

// Accepts the channel number (char *) 1-number of channel and the number of channels per
// module and returns the board number of -1 if error. 
// Send host nak message on error
int getDCBSbrd(char *chan, int chPmodule)
{
  String token = chan;
  int ch = token.toInt();
  return getDCBSbrd(ch, chPmodule);
}

// Converts the input channel number (int) 1-number of channels to a channel index
// No error testing
int getDCBSch(int ch, int chPmodule)
{
  return((ch-1) & (chPmodule-1));
}

// Converts the input channel number (char *) 1-number of channels to a channel index
// No error testing
int getDCBSch(char *chan, int chPmodule)
{
  String token = chan;
  int ch = token.toInt();
  return((ch-1) & (chPmodule-1));
}

// This function sends a TWI message to the DCBswitch module to update memory in the 
// data structure. 
bool writeDCBSdata(uint8_t brd, uint32_t address, uint8_t size)
{
  SelectBoard(0);
  if(WriteEEPROMext((void *)address, DCBswitchData[brd]->TWIadd, address - (uint32_t)DCBswitchData[brd], size)) return true;
  return false;
}

void setDCBSvalue(int brd, char *value, bool *val)
{
  String token = value;
  if((brd < 0) || ((token != "TRUE") && (token != "FALSE"))) BADARG;
  if(token == "TRUE") *val = true;
  else *val = false;
  writeDCBSdata(brd,(uint32_t)val,sizeof(bool));
  SendACK;
}
void setDCBSvalue(int brd, char *value, char *val, char *options)
{
  if(brd < 0) BADARG;
  char c;
  if(!rangeCheck(value, &c, options, true)) return;
  if(c=='0') c = 0;
  *val = c;
  writeDCBSdata(brd,(uint32_t)val,sizeof(char));
  SendACK;
}
void setDCBSvalue(int brd, int ival, int *val, int ll, int ul)
{
  if((brd < 0) || (ival < ll) || (ival > ul)) BADARG;
  *val = ival;
  writeDCBSdata(brd,(uint32_t)val,sizeof(int));
  SendACK;
}
void setDCBSvalue(int brd, char *value, int *val, int ll, int ul)
{
  String token = value;
  setDCBSvalue(brd, token.toInt(), val, ll, ul);
}
void setDCBSvalue(int brd, char *value, float *val, float ll, float ul)
{
  String token = value;
  float fval = token.toFloat();
  if((brd < 0) || (fval < ll) || (fval > ul)) BADARG;
  *val = fval;
  writeDCBSdata(brd,(uint32_t)val,sizeof(float));
  SendACK;
}
void setDCBSvalue(int brd, char *value, bool *val, int twicmd)
{
  String token = value;
  if((brd < 0) || ((token != "TRUE") && (token != "FALSE"))) BADARG;
  if(token == "TRUE") *val = true;
  else *val = false;
  TWIsetBool(DCBswitchData[brd]->TWIadd, brd, 0x80 | twicmd, *val);
  SendACK;
}
void getDCBSvalue(int brd, bool *value, int twicmd)
{
  int val;
  if(TWIread8bitUnsigned(DCBswitchData[brd]->TWIadd, brd, twicmd, &val))
  {
    SendACKonly;
    *value = val;
    if(SerialMute) return;
    if(val) serial->println("TRUE"); 
    else serial->println("FALSE");
    return;
  }
  BADARG;
}

void setDCBSeepromEnable(char *board, char *value)
{
  String token = board;
  int brd = token.toInt();
  bool bval;
  if((brd < 0) || (brd > 1)) BADARG;
  if(DCBswitchData[brd]==NULL) BADARG;
  token = value;
  if(token == "TRUE") bval = true;
  else if(token == "FALSE") bval = false;
  else BADARG;
  TWIsetBool(DCBswitchData[brd]->TWIadd, brd, 0x80 | TWI_DCBS_EEPROM, bval);
  SendACK;
}
void setDCBSvoltage(char *chan, char *value) {int brd = getDCBSbrd(chan,4); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->DCBV[getDCBSch(chan,4)],-250.0, 250.0);}
void getDCBSvoltage(int ch) {int brd = getDCBSbrd(ch,4); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->DCBV[getDCBSch(ch,4)]);}
void getDCBSreadback(int ch) {int brd = getDCBSbrd(ch,4); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchrb[0]->rb[getDCBSch(ch,4)]);}
void setDCBSpwr(char *ch, char *value) {int brd; if((brd = getDCBSbrd(ch,4)) == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->pwrEnable,0x80 | SET_DCBS_PWR);}
void getDCBSpwr(int ch) {int brd; if((brd = getDCBSbrd(ch,4)) == -1) return; getDCBSvalue(brd,&DCBswitchData[brd]->pwrEnable,0x80 | GET_DCBS_PWR);}

// Parameters are in the input buffer.
// Module number, 1 or 2
// DIO channel, A|B|C|D
// State, 0 or 1
void setDCBSdout(void) 
{
  int   brd,mod,state;
  char  c;

  while(true)
  {
    if(!valueFromCommandLine(&mod, 1, 2))    break;
    if(!valueFromCommandLine(&c, "ABCD")) break;
    if(!valueFromCommandLine(&state, 0, 1))  break;
    if((brd = DCBSmod2brd(mod)) == -1) break;
    uint16_t wval = c | (state << 8);
    TWIsetWord(DCBswitchData[brd]->TWIadd, brd, 0x80 | SET_DCBS_SDOUT, wval);
    SendACK;
    return;
  }
  BADARG;
}
void getDCBSdin(char *module, char *dioCH) 
{
  while(true)
  {
    int brd; if((brd = DCBSmod2brd(module)) == -1) break;
    char c;  if(!rangeCheck(dioCH, &c, "ABCDQRST", false)) break;
    TWIsetByte(DCBswitchData[brd]->TWIadd, brd, 0x80 | GET_DCBS_SDIN, c);
    int value; if(!TWIread8bitUnsigned(DCBswitchData[brd]->TWIadd, brd, -1, &value)) break;
    if((value != 0) && (value != 1)) break;
    SendACKonly; if(SerialMute) return; serial->println(value);
    return;
  }
  BADARG;
}

void setDCBSsw(char *ch, char *value) 
{
  int brd; if((brd = getDCBSbrd(ch,2)) == -1) return; 
  int val; if(!rangeCheck(value, &val, 0, 1, true)) return;
  val = (val << 8) & 0xFF00;
  val |= getDCBSch(ch,2) + 1;
  TWIsetWord(DCBswitchData[brd]->TWIadd, brd, 0x80 | SET_DCBS_FETSW, val);
  SendACK;
}

void getDCBSsw(int ch)
{
  int brd; if((brd = getDCBSbrd(ch,2)) == -1) return; 
  TWIsetByte(DCBswitchData[brd]->TWIadd, brd, 0x80 | GET_DCBS_FETSW, getDCBSch(ch,2)+1);
  int value; if(!TWIread8bitUnsigned(DCBswitchData[brd]->TWIadd, brd, -1, &value)) BADARG;
  if((value != 0) && (value != 1)) BADARG;
  SendACKonly; if(SerialMute) return; serial->println(value);
}

void pulseDCBSsw(char *ch, char *value)
{
  int brd; if((brd = getDCBSbrd(ch,2)) == -1) return; 
  int val; if(!rangeCheck(value, &val, 1, 10000, true)) return;
  val << 8 & 0xFFFF00 | getDCBSch(ch,2);
  TWIset24bitInt(DCBswitchData[brd]->TWIadd, brd, 0x80 | SET_DCBS_FETPULSE, val);
  SendACK;
}

void setDCBStrigger(char *ch, char *dioCH)
{
  int brd; if((brd = getDCBSbrd(ch,2)) == -1) return; 
  char c;  if(!rangeCheck(dioCH, &c, "0QRST", true)) return;
  uint16_t wval = (getDCBSch(ch,2) + 1) | (c << 8);
  TWIsetWord(DCBswitchData[brd]->TWIadd, brd, 0x80 | SET_DCBS_SWFOL, wval);
  SendACK;
}

void getDCBStrigger(int ch)
{
  int brd; if((brd = getDCBSbrd(ch,2)) == -1) return; 
  TWIsetByte(DCBswitchData[brd]->TWIadd, brd, 0x80 | GET_DCBS_SWFOL, getDCBSch(ch,2)+1);
  int value; if(!TWIread8bitUnsigned(DCBswitchData[brd]->TWIadd, brd, -1, &value)) BADARG;
  if(value == 0) value = '0';
  char c[2]; c[0] = value; c[1] = 0;
  char res;  if(!rangeCheck(c, &res, "0QRST", true)) return;
  SendACKonly; if(SerialMute) return; serial->println(res);
}

void setDCBSrbTest(char *ch, char *value) {int brd; if((brd = getDCBSbrd(ch,4)) == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->rbTest,0x80 | SET_DCBS_RBTST);}
void getDCBSrbTest(int ch) {int brd; if((brd = getDCBSbrd(ch,4)) == -1) return; getDCBSvalue(brd,&DCBswitchData[brd]->rbTest,0x80 | GET_DCBS_RBTST);}

void reportIOport(char c)
{
   if(SerialMute) return;
   if(c == 0) serial->println('0');
   else serial->println(c);
}

// Pulse generator commands
void setPGena(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].enable);}
void getPGena(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; if(SerialMute) return; if(DCBswitchData[brd]->PG[getDCBSch(ch,2)].enable) serial->println("TRUE"); else serial->println("FALSE");}
void setPGretrig(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].reTrig);}
void getPGretrig(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; if(SerialMute) return; if(DCBswitchData[brd]->PG[getDCBSch(ch,2)].reTrig) serial->println("TRUE"); else serial->println("FALSE");}
void setPGarmT(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].armTrig,"0QRST");}
void getPGarmT(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; reportIOport(DCBswitchData[brd]->PG[getDCBSch(ch,2)].armTrig);}

// Note, the level enums don't seem to match the teensy!
// Teensy values
// CHANGE  = 4
// RISING  = 3
// FALLING = 2
int level2int(char *value)
{
  String token = value;
  if(token == "CHANGE")       return 4;
  else if(token == "RISING")  return 3;
  else if(token == "FALLING") return 2;
  else { SetErrorCode(ERR_BADARG); SendNAK; return -1;}
}
void reportLevel(int lvl)
{
  char *rep = NULL;
  if(lvl == 4) rep = "CHANGE";
  else if(lvl == 3) rep = "RISING";
  else if(lvl == 2) rep = "FALING";
  if(SerialMute) return;
  if(rep == NULL) BADARG;
  serial->println(rep);
}

void setPGarmL(char *ch, char *value){int brd = getDCBSbrd(ch,2); int lvl = level2int(value); if((brd == -1) || (lvl == -1)) return; setDCBSvalue(brd,lvl,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].armLevel,0,10);}
void getPGarmL(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; reportLevel(DCBswitchData[brd]->PG[getDCBSch(ch,2)].armLevel);}
void setPGtrig(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].trig,"0QRST");}
void getPGtrig(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; reportIOport(DCBswitchData[brd]->PG[getDCBSch(ch,2)].trig);}
void setPGtrigL(char *ch, char *value){int brd = getDCBSbrd(ch,2); int lvl = level2int(value); if((brd == -1) || (lvl == -1)) return; setDCBSvalue(brd,lvl,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigLevel,0,10);}
void getPGtrigL(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; reportLevel(DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigLevel);}
void setPGskip(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].skip,0, 10000);}
void getPGskip(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].skip);}
void setPGdly(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].delay,0, 10000000);}
void getPGdly(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].delay);}
void setPGwdth(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].width,0, 10000000);}
void getPGwdth(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].width);}
void setPGtrgOut(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigOut,"0ABCD");}
void getPGtrgOut(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; reportIOport(DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigOut);}
void setPGtrhFET(char *ch, char *value) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigFET,"012");}
void getPGtrhFET(int ch) {int brd = getDCBSbrd(ch,2); if(brd == -1) return; SendACKonly; reportIOport(DCBswitchData[brd]->PG[getDCBSch(ch,2)].trigFET);}
void setPGoutCh(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].outputCh,0, 4);}
void getPGoutCh(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].outputCh);}
void setPGvoltage(char *chan, char *value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].pulseV,-250.0, 250.0);}
void getPGvoltage(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].pulseV);}
void setPGnum(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].numPulse,0, 1000000);}
void getPGnum(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].numPulse);}
void setPGperiod(int chan, int value) {int brd = getDCBSbrd(chan,2); if(brd == -1) return; setDCBSvalue(brd,value,&DCBswitchData[brd]->PG[getDCBSch(chan,2)].period,0, 10000000);}
void getPGperiod(int chan){int brd = getDCBSbrd(chan,2); if(brd == -1) return; SendACKonly; if(!SerialMute) serial->println(DCBswitchData[brd]->PG[getDCBSch(chan,2)].period);}

// Overloaded set of functions to check the range of user entered data. 
// If the value is in range and the target passed by reference is not NULL
// then its updated. Returns false on error. If report it true then
// the NAK message is sent to host.
bool rangeCheck(char *value, bool *target, bool report)
{
  String token = value;
  if((token == "TRUE") || (token == "FALSE"))
  {
    if(target != NULL)
    {
      if(token == "TRUE") *target = true;
      else *target = false;
    }
    return true;
  }
  if(report) { SetErrorCode(ERR_BADARG); SendNAK;}
  return false;
}

bool rangeCheck(int value, int *target, int ll, int ul, bool report)
{
  if((value >= ll) && (value <= ul))
  {
    if(target != NULL) *target = value;
    return true;
  }
  if(report) { SetErrorCode(ERR_BADARG); SendNAK;}
  return false;
}

bool rangeCheck(char *value, int *target, int ll, int ul, bool report)
{
  String token = value;
  return rangeCheck(token.toInt(), target, ll, ul, report);
}

bool rangeCheck(float value, float *target, float ll, float ul, bool report)
{
  if((value >= ll) && (value <= ul))
  {
    if(target != NULL) *target = value;
    return true;
  }
  if(report) { SetErrorCode(ERR_BADARG); SendNAK;}
  return false;
}

bool rangeCheck(char *value, float *target, float ll, float ul, bool report)
{
  String token = value;
  return rangeCheck(token.toFloat(), target, ll, ul, report);
}

bool rangeCheck(char *value, char **target, char *options, bool report)
{
  String token = value;
  token.trim();
  if(options != NULL)
  {
    if(strstr(options, token.c_str()) == NULL)
    {
      if(report) {SetErrorCode(ERR_BADARG); SendNAK;}
      return false;
    }
  }
  if(target != NULL) *target = value;
  return true;
}

bool rangeCheck(char *value, char *target, char *options, bool report)
{
  String token = value;
  token.trim();
  if(options != NULL)
  {
    if(strstr(options, token.c_str()) == NULL)
    {
      if(report) {SetErrorCode(ERR_BADARG); SendNAK;}
      return false;
    }
  }
  if(target != NULL) *target = token.c_str()[0];
  return true;
}

#endif

