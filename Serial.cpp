/*
 * serial.c
 *
 * This file contains the code for the serial IO hardware including the ring buffer
 * code and the serial command processor.
 *
 * Developed by: Gordon Anderson
 */
#include "Arduino.h"
#include "SD.h"
#include "string.h"
#include "Serial.h"
#include "Errors.h"
#include "Menu.h"
#include "Dialog.h"
#include "DIO.h"
#include "Table.h"
#include "Hardware.h"
#include "DCbias.h"
#include "ESI.h"
#include "Twave.h"
#include "FAIMS.h"
#include "Filament.h"
#include "WiFi.h"
#include "Variants.h"
#include <ThreadController.h>

extern ThreadController control;

//Serial_ *serial = &SerialUSB;
Stream *serial = &SerialUSB;
bool SerialMute = false;
//HardwareSerial *serial = &Serial;

// Variables used for Macro recording
extern bool SDcardPresent;
File MacroFile;
bool Recording = false;

#define MaxToken 20

char Token[MaxToken];
char Sarg1[MaxToken];
char Sarg2[MaxToken];
unsigned char Tptr;

int ErrorCode = 0;   // Last communication error that was logged

Ring_Buffer  RB;     // Receive ring buffer

// ACK only string, two options. Need a comma when in the echo mode
char *ACKonlyString1 = "\x06";
char *ACKonlyString2 = ",\x06";
char *SelectedACKonlyString = ACKonlyString1;

bool echoMode = false;

Commands  CmdArray[] = 	{
  // General commands
  {"GVER",  CMDstr, 0, (char *)Version},	               // Report version
  {"GERR",  CMDint, 0, (char *)&ErrorCode},              // Report the last error code
  {"GNAME", CMDstr, 0, (char *)MIPSconfigData.Name},	   // Report MIPS system name
  {"SNAME", CMDstr, 1, (char *)MIPSconfigData.Name},	   // Set MIPS system name
  {"RESET", CMDfunction, 0, (char *)Software_Reset},     // Reset the Due
  {"SAVE",  CMDfunction, 0, (char *)SAVEparms},	         // Save MIPS configuration data to default.cfg on CD card
  {"GCHAN", CMDfunctionStr, 1, (char *)GetNumChans},     // Report number for the selected system
  {"MUTE",  CMDfunctionStr, 1, (char *)Mute},            // Turns on and off the serial response from the MIPS system
  {"ECHO",  CMDbool, 1, (char *)&echoMode},              // Turns on and off the serial echo mode where the command is echoed to host, TRUE or FALSE
  {"TRIGOUT", CMDfunctionStr, 1, (char *)(static_cast<void (*)(char *)>(&TriggerOut))},    // Generates output trigger on rev 2 and higher controllers
                                                         // supports, HIGH,LOW,PULSE
  {"DELAY", CMDfunction, 1, (char *)DelayCommand},       // Generates a delay in milliseconds. This is used by the macro functions
                                                         // to define delays in voltage ramp up etc.
  {"GCMDS", CMDfunction, 0, (char *)GetCommands},	 // Send a list of all commands
  {"GAENA", CMDbool, 0, (char *)&MIPSconfigData.UseAnalog}, // Print the UseAnalog flag, true or false
  {"SAENA", CMDbool, 1, (char *)&MIPSconfigData.UseAnalog}, // Sets the UseAnalog flag, true or false
  {"THREADS", CMDfunction, 0, (char *)ListThreads},         // List all threads, there IDs, and there last runtimes
  {"STHRDENA", CMDfunctionStr, 2, (char *)SetThreadEnable}, // Set thread enable to true or false
  {"SDEVADD", CMDfunctionStr, 2, (char *)DefineDeviceAddress}, // Set device board and address
  {"RDEV", CMDfunction, 1, (char *)ReportAD7998},              // Read the ADC channel value
  {"TBLTSKENA", CMDbool, 1, (char *)&TasksEnabled},            // Enables tasks in table mode, true or false
  {"ADC", CMDfunction, 1, (char *)ADCread},                    // Read and report ADC channel. Valid range 0 through 3
  {"LEDOVRD", CMDbool, 1, (char *)&LEDoverride},               // Override the LED operation is true, always false on startup
  {"LED",  CMDint, 1, (char *)&LEDstate},                      // Define the LEDs state you are looking for.
  {"DSPOFF", CMDbool, 1, (char *)&DisableDisplay},             // Print the UseAnalog flag, true or false
  // Clock generation functions
  {"GWIDTH",  CMDint, 0, (char *)&PulseWidth},                // Report the pulse width in microseconds
  {"SWIDTH",  CMDint, 1, (char *)&PulseWidth},                // Set the pulse width in microseconds
  {"GFREQ",  CMDint, 0, (char *)&PulseFreq},                  // Report the pulse frequency in Hz
  {"SFREQ",  CMDint, 1, (char *)&PulseFreq},                  // Set the pulse frequency in Hz
  {"BURST",  CMDfunction, 1, (char *)&GenerateBurst},              // Generates a frequencyy burst on trig out line
  // DC bias module commands
  {"SDCB", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&DCbiasSet))},      // Set voltage value
  {"GDCB", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasRead))},// Get Voltage value requested
  {"GDCBV", CMDfunction, 1, (char *)DCbiasReadV},        // Get Voltage actual voltage value
  {"SDCBOF", CMDfunctionStr, 2, (char *)DCbiasSetFloat}, // Set float voltage for selected board
  {"GDCBOF", CMDfunction, 1, (char *)DCbiasReadFloat},   // Read float voltage for selected board
  {"GDCMIN", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasReadMin))},// Read float voltage for selected board
  {"GDCMAX", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasReadMax))},// Read float voltage for selected board
  {"SDCPWR", CMDfunctionStr, 1, (char *)DCbiasPowerSet}, // Sets the DC bias power, on or off
  {"GDCPWR", CMDfunction, 0, (char *)DCbiasPower},       // Get the DC bias power, on or off
  {"GDCBALL", CMDfunction, 0, (char *)DCbiasReportAllSetpoints},   // Returns the DC bias voltage setpoints for all channels in the system
  {"GDCBALLV", CMDfunction, 0, (char *)DCbiasReportAllValues},     // Returns the DC bias voltage readback values for all channels in the system
  {"SDCBCHNS", CMDfunction, 2, (char *)DCbiasSetNumBoardChans},    // Sets the number of channels on a DCbias board. Used for setup only.
  {"SDCBONEOFF", CMDbool, 1, (char *)&DCbDarray[0].UseOneOffset},  // TRUE to enable use of one offset
  {"DCBOFFRBENA", CMDbool, 1, (char *)&DCbDarray[0].OffsetReadback},  // TRUE to enable use of offset readback
  // RF generator module commands
  {"SRFFRQ", CMDfunction, 2, (char *)RFfreq},		 // Set RF frequency
  {"SRFVLT", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&RFvoltage))},	 // Set RF output voltage
  {"SRFDRV", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&RFdrive))},        // Set RF drive level
  {"GRFFRQ", CMDfunction, 1, (char *)RFfreqReport},	 // Report RF frequency
  {"GRFPPVP", CMDfunction, 1, (char *)RFvoltageReportP}, // Report RF output voltage, positive phase
  {"GRFPPVN", CMDfunction, 1, (char *)RFvoltageReportN}, // Report RF output voltage, negative phase
  {"GRFDRV", CMDfunction, 1, (char *)RFdriveReport},     // Report RF drive level in percentage
  {"GRFVLT", CMDfunction, 1, (char *)RFvoltageReport},   // Report RF output voltage setpoint
  {"GRFPWR", CMDfunction, 1, (char *)RFheadPower},       // Report RF head power draw
  {"GRFALL", CMDfunction, 0, (char *)RFreportAll},       // Reports Freq, RFVpp + and - for each RF channel in system
  // DIO commands
  {"SDIO", CMDfunctionStr, 2, (char *)SDIO_Serial},	 // Set DIO output bit
  {"GDIO", CMDfunctionStr, 1, (char *)GDIO_Serial},	 // Get DIO output bit
  {"RPT", CMDfunctionStr, 2, (char *)DIOreport},     // Report an input state change
  {"MIRROR", CMDfunctionStr, 2, (char *)DIOmirror},  // Mirror an input to an output
  // ESI module commands
  {"SHV", CMDfunctionStr, 2, (char *)SetESIchannel},     // Set channel high voltage
  {"GHV", CMDfunction, 1, (char *)GetESIchannel},        // Returns the high voltage setpoint
  {"GHVV", CMDfunction, 1, (char *)GetESIchannelV},      // Returns the actual high voltage output
  {"GHVI", CMDfunction, 1, (char *)GetESIchannelI},      // Returns the output current in mA
  {"GHVMAX", CMDfunction, 1, (char *)GetESIchannelMax},  // Returns the maximum high voltage outut value
  // Table commands, tables enable pulse sequence generation
  {"STBLDAT", CMDfunction, 0, (char *)ParseTableCommand}, // Read the HVPS voltage table
  {"STBLCLK", CMDfunctionStr, 1, (char *)SetTableCLK},	  // Clock mode, EXT or INT
  {"STBLTRG", CMDfunctionStr, 1, (char *)SetTableTRG},	  // Trigger mode, EXT or SW
  {"TBLABRT", CMDfunction, 0, (char *)SetTableAbort},	    // Abort table operation
  {"SMOD", CMDfunctionStr, 1, (char *)SetTableMode},	    // Set the table mode
  {"TBLSTRT", CMDfunction, 0, (char *)SWTableTrg},        // Set the software trigger
  {"TBLSTOP", CMDfunction, 0, (char *)StopTable},         // Stops a running table
  {"GTBLFRQ", CMDfunction, 0, (char *)TableFreq},	        // Returns the current internal clock frequency
  {"STBLNUM", CMDfunction, 1, (char *)SetTableNumber},    // Set the active table number
  {"GTBLNUM", CMDfunction, 0, (char *)GetTableNumber},    // Get the active table number
  {"STBLADV", CMDfunctionStr, 1, (char *)SetTableAdvance},// Set the table advance status, ON or OFF
  {"GTBLADV", CMDfunction, 0, (char *)GetTableAdvance},   // Get the table advance status, ON or OFF
  {"STBLVLT", CMDfun2int1flt, 3, (char *)SetTableEntryValue}, // Set a value in a loaded table
  {"GTBLVLT", CMDfunction, 2, (char *)GetTableEntryValue},// Get a value from a loaded table
  {"STBLCNT", CMDfun2int1flt, 3, (char *)SetTableEntryCount}, // Set a count value in a loaded table
  {"STBLDLY", CMDint, 1, (char *)&InterTableDelay}, // Defines the inter table delay in milli seconds
  // Macro commands
  {"MRECORD", CMDfunctionStr, 1, (char *) MacroRecord},    // Turn on macro recording into the filename argument
  {"MSTOP", CMDfunction, 0, (char *) MacroStop},           // Stop macro recording and close the file
  {"MPLAY", CMDfunctionStr, 1, (char *) MacroPlay},        // Play a macro file
  {"MLIST", CMDfunction, 0, (char *) MacroList},           // Send a list of macro files
  {"MDELETE", CMDfunctionStr, 1, (char *) MacroDelete},    // Delete a macro file
  // TWAVE commands
  {"GTWF",  CMDfunction, 1, (char *)sendTWAVEfrequency},       // Report the TWAVE frequency
  {"STWF",  CMDfunction, 2, (char *)setTWAVEfrequency},        // Set the TWAVE frequency
  {"GTWPV", CMDfunction, 1, (char *)sendTWAVEpulseVoltage},    // Report the TWAVE pulse voltage
  {"STWPV", CMDfunctionStr, 2, (char *)setTWAVEpulseVoltage},  // Set the TWAVE pulse voltage
  {"GTWG1V", CMDfunction, 1, (char *)sendTWAVEguard1Voltage},  // Report the TWAVE Guard 1 voltage
  {"STWG1V", CMDfunctionStr, 2, (char *)setTWAVEguard1Voltage},// Set the TWAVE Guard 1 voltage
  {"GTWG2V", CMDfunction, 1, (char *)sendTWAVEguard2Voltage},  // Report the TWAVE Guard 2 voltage
  {"STWG2V", CMDfunctionStr, 2, (char *)setTWAVEguard2Voltage},// Set the TWAVE Guard 2 voltage
  {"GTWSEQ", CMDfunction, 1, (char *)sendTWAVEsequence},       // Report the TWAVE sequence
  {"STWSEQ", CMDfunctionStr, 2, (char *)setTWAVEsequence},     // Set the TWAVE sequence
  {"GTWDIR", CMDfunction, 1, (char *)getTWAVEdir},             // Report the TWAVE waveform direction, FWD or REV
  {"STWDIR", CMDfunctionStr, 2, (char *)setTWAVEdir},          // Set the TWAVE waveform direction, FWD or REV
  // Twave compressor commands
  {"STWCTBL", CMDlongStr, 100, (char *)TwaveCompressorTable},  // Twave compressor table definition setting command
  {"GTWCTBL", CMDstr, 0, (char *)TwaveCompressorTable},        // Twave compressor table definition reporting command
  {"GTWCMODE",CMDstr, 0, (char *)Cmode},                       // Report Twave compressor mode
  {"STWCMODE",CMDfunctionStr, 1, (char *)SetTWCmode},          // Set Twave compressor mode
  {"GTWCORDER",CMDfunction, 0, (char *)GetTWCorder},           // Report Twave compressor order
  {"STWCORDER",CMDfunction, 1, (char *)SetTWCorder},           // Set Twave compressor order
  {"GTWCTD",CMDfloat, 0, (char *)&TDarray[0].Tdelay},          // Report Twave compressor trigger delay in mS
  {"STWCTD",CMDfunctionStr, 1, (char *)SetTWCtriggerDelay},    // Set Twave compressor trigger delay in mS
  {"GTWCTC",CMDfloat, 0, (char *)&TDarray[0].Tcompress},       // Report Twave compressor compress time in mS
  {"STWCTC",CMDfunctionStr, 1, (char *)SetTWCcompressTime},    // Set Twave compressor compress time in mS
  {"GTWCTN",CMDfloat, 0, (char *)&TDarray[0].Tnormal},         // Report Twave compressor normal time in mS
  {"STWCTN",CMDfunctionStr, 1, (char *)SetTWCnormalTime},      // Set Twave compressor normal time in mS
  {"GTWCTNC",CMDfloat, 0, (char *)&TDarray[0].TnoC},           // Report Twave compressor non compress time in mS
  {"STWCTNC",CMDfunctionStr, 1, (char *)SetTWCnoncompressTime},// Set Twave compressor non compress time in mS
  {"TWCTRG",CMDfunction, 0, (char *)TWCtrigger},               // Force a Twave compressor trigger
  {"GTWCSW",CMDstr, 0, (char *)CswitchState},                  // Report Twave compressor Switch state
  {"STWCSW",CMDfunctionStr, 1, (char *)SetTWCswitch},          // Set Twave compressor Switch state
  // Twave configuration commands  
  {"STWCCLK", CMDbool, 1, (char *)&TDarray[0].UseCommonClock}, // Flag to indicate common clock mode for two Twave modules.
  {"STWCMP", CMDbool, 1, (char *)&TDarray[0].CompressorEnabled}, // Flag to indicate Twave compressor mode is enabled.
  // FAIMS commands
  {"SRFHPCAL", CMDfunctionStr, 2, (char *)FAIMSsetRFharPcal},  // Set FAIMS RF harmonic positive peak readback calibration
  {"SRFHNCAL", CMDfunctionStr, 2, (char *)FAIMSsetRFharNcal},  // Set FAIMS RF harmonic negative peak readback calibration
  // Filament commands
  {"GFLENA", CMDfunction, 1, (char *)GetFilamentEnable},             // Get filament ON/OFF status
  {"SFLENA", CMDfunctionStr, 2, (char *)SetFilamentEnable},          // Set filament ON/OFF status
  {"GFLI", CMDfunction, 1, (char *)GetFilamentCurrent},              // Get filament channel current (setpoint)
  {"GFLAI", CMDfunction, 1, (char *)GetFilamentActualCurrent},       // Get filament channel actual current
  {"SFLI", CMDfunctionStr, 2, (char *)SetFilamentCurrent},           // Set filament channel current
  {"GFLSV", CMDfunction, 1, (char *)GetFilamentSupplyVoltage},       // Get filament supply voltage (setpoint)
  {"GFLASV", CMDfunction, 1, (char *)GetFilamentActualSupplyVoltage},// Get the actual supply side voltage
  {"SFLSV", CMDfunctionStr, 2, (char *)SetFilamentSupplyVoltage},    // Set filament supply voltage
  {"GFLV", CMDfunction, 1, (char *)GetFilamentVoltage},              // Get filament voltage (actual)
  {"GFLPWR", CMDfunction, 1, (char *)GetFilamentPower},              // Get filament power (actual) 
  {"GFLRT", CMDfunction, 1, (char *)GetCurrentRampRate},             // Get filament current ramp rate in amps per second
  {"SFLRT", CMDfunctionStr, 2, (char *)SetCurrentRampRate},          // Set filament current ramp rate in amps per second
  {"GFLP1", CMDfunction, 1, (char *)GetFilamentCycleCurrent1},       // Get filament cycle current 1 (setpoint)
  {"SFLP1", CMDfunctionStr, 2, (char *)SetFilamentCycleCurrent1},    // Set filament cycle current 1 (setpoint)
  {"GFLP2", CMDfunction, 1, (char *)GetFilamentCycleCurrent2},       // Get filament cycle current 2 (setpoint)
  {"SFLP2", CMDfunctionStr, 2, (char *)SetFilamentCycleCurrent2},    // Set filament cycle current 2 (setpoint)
  {"GFLCY", CMDfunction, 1, (char *)GetFilamentCycleCount},          // Get filament cycle count
  {"SFLCY", CMDfunctionStr, 2, (char *)SetFilamentCycleCount},       // Set filament cycle count
  {"GFLENAR", CMDfunction, 1, (char *)GetFilamentStatus},            // Get filament cycle status, OFF, or the number of cycles remaining
  {"SFLENAR", CMDfunctionStr, 2, (char *)SetFilamentStatus},         // Set filament cycle status, ON or OFF
  {"RFLPARMS", CMDfunction, 2, (char *)SetFilamentReporting},        // Sets a filament channel reporting rate, 0 = off. Rate is in seconds
  // WiFi commands
  {"GHOST",  CMDstr, 0, (char *)wifidata.Host},                      // Report this MIPS box host name
  {"GSSID",  CMDstr, 0, (char *)wifidata.ssid},                      // Report the WiFi SSID to connect to
  {"GPSWD",  CMDstr, 0, (char *)wifidata.password},                  // Report the WiFi network password
  {"SHOST",  CMDfunctionStr, 1, (char *)SetHost},                    // Set this MIPS box host name
  {"SSSID",  CMDfunctionStr, 1, (char *)SetSSID},                    // Set the WiFi SSID to connect to
  {"SPSWD",  CMDfunctionStr, 1, (char *)SetPassword},                // Set the WiFi network password
  {"SWIFIENA",  CMDbool, 1, (char *)&MIPSconfigData.UseWiFi},              // Set the WiFi enable flag
  // End of table marker
  {0},
};

// This function lists all the current threads and there current state.
void ListThreads(void)
{
  int    i = 0;
  Thread *t;

  // Loop through all the threads and report the Thread name, ID, Interval, enabled state, and last run time
  SendACKonly;
  serial->println("Thread name,ID,Interval,Enabled,Run time");
  while (1)
  {
    t = control.get(i++);
    if (t == NULL) break;
    serial->print(t->getName()); serial->print(", ");
    serial->print(t->getID()); serial->print(", ");
    serial->print(t->getInterval()); serial->print(", ");
    if (t->enabled) serial->print("Enabled,");
    else serial->print("Disabled,");
    serial->println(t->runTimeMs());
  }
}

void SetThreadEnable(char *name, char *state)
{
  Thread *t;

  if ((strcmp(state, "TRUE") !=0) && (strcmp(state, "FALSE") != 0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // Find thread by name
  t = control.get(name);
  if (t == NULL)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if (strcmp(state, "TRUE") == 0) t->enabled = true;
  else t->enabled = false;
}

// Sends a list of all commands
void GetCommands(void)
{
  int  i;

  SendACKonly;
  // Loop through the commands array and send all the command tokens
  for (i = 0; CmdArray[i].Cmd != 0; i++)
  {
    serial->println((char *)CmdArray[i].Cmd);
  }
}

// Delay command, delay is in millisecs
void DelayCommand(int dtime)
{
  delay(dtime);
  SendACK;
}

// Turns on and off responses from the MIPS system
void Mute(char *cmd)
{
  if (strcmp(cmd, "ON") == 0)
  {
    SerialMute = true;
    SendACK;
    return;
  }
  else if (strcmp(cmd, "OFF") == 0)
  {
    SerialMute = false;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Generic get number of channels function, calls the proper routine based on parameter entered.
// RF = Number of RF channels
// DCB = number of DC bias channels
// ESI = number of HV ESI supplies
// FAIMS = Number of FAIMS drivers
// TWAVE = Number of TWAVE drivers
// FIL = Number of filiment channels
void GetNumChans(char *cmd)
{
  if (strcmp(cmd, "RF") == 0) RFnumber();
  else if (strcmp(cmd, "DCB") == 0) DCbiasNumber();
  else if (strcmp(cmd, "ESI") == 0) ESInumberOfChannels();
  else if (strcmp(cmd, "TWAVE") == 0) TWAVEnumberOfChannels();
  else if (strcmp(cmd, "FAIMS") == 0) FAIMSnumberOfChannels();
  else if (strcmp(cmd, "FIL") == 0) FilamentChannels();
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
}

void SerialInit(void)
{
  //  Serial.begin(9600);
  //Serial_ *serial = &SerialUSB;
  SerialUSB.begin(0);
  //  serial->println("Initializing....");
  RB_Init(&RB);
}

// This function builds a token string from the characters passed.
void Char2Token(char ch)
{
  Token[Tptr++] = ch;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
}

// This function reads the serial input ring buffer and returns a pointer to a ascii token.
// Tokens are comma delimited. Commands strings end with a semicolon or a \n.
// The returned token pointer points to a standard C null terminated string.
// This function does not block. If there is nothing in the input buffer null is returned.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  // Exit if the input buffer is empty
  ch = RB_Next(&RB);
  if (ch == 0xFF) return NULL;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;

  if ((ch == '\n') || (ch == ';') || (ch == ':') || (ch == ',') || (ch == ']') || (ch == '['))
  {
    if (Tptr != 0) ch = 0;
    else
    {
      Char2Token(RB_Get(&RB));
      ch = 0;
    }
  }
  else RB_Get(&RB);
  // Place the character in the input buffer and advance pointer
  Char2Token(ch);
  if (ch == 0)
  {
    Tptr = 0;
    if ((Token[0] == ',') && !ReturnComma) return NULL;
    return Token;
  }
  return NULL;
}

void ExecuteCommand(Commands *cmd, int arg1, int arg2, char *args1, char *args2, float farg1)
{
  if(echoMode) SelectedACKonlyString = ACKonlyString2;
  else SelectedACKonlyString = ACKonlyString1;
  switch (cmd->Type)
  {
    case CMDbool:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute)
        {
          if (*(cmd->pointers.boolPtr)) serial->println("TRUE");
          else serial->println("FALSE");
        }
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        if((strcmp(args1,"TRUE") == 0) || (strcmp(args1,"FALSE") == 0))
        {
          if(strcmp(args1,"TRUE") == 0) *(cmd->pointers.boolPtr) = true;
          else *(cmd->pointers.boolPtr) = false;
          SendACK;
          break;
        }
        SetErrorCode(ERR_BADARG);
        SendNAK;
      }
      break;
    case CMDstr:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(cmd->pointers.charPtr);
        break;
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
          strcpy(cmd->pointers.charPtr,args1);
          SendACK;
          break;
      }
      break;
    case CMDint:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
         SendACKonly;
         if (!SerialMute) serial->println(*(cmd->pointers.intPtr));
         break;
      }
      if (cmd->NumArgs == 1) 
      {
          *(cmd->pointers.intPtr) = arg1;
          SendACK;
          break;
      }
    case CMDfloat:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
          SendACKonly;
          if (!SerialMute) serial->println(*(cmd->pointers.floatPtr));
      }
      if (cmd->NumArgs == 1) 
      {
          *(cmd->pointers.floatPtr) = farg1;
          SendACK;
          break;
      }
      break;
    case CMDfunction:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;
    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;
    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;
    default:
      SendNAK;
      break;
  }
}

// This function processes serial commands.
// This function does not block and return -1 if there was nothing to do.
int ProcessCommand(void)
{
  char   *Token,ch;
  int    i;
  static int   arg1, arg2;
  static float farg1;
  static enum  PCstates state;
  static int   CmdNum;
  static char  delimiter=0;
  // The following variables are used for the long string reading mode
  static char *lstrptr = NULL;
  static int  lstrindex;
  static bool lstrmode = false;
  static int lstrmax;

  if(lstrmode)
  {
    ch = RB_Get(&RB);
    if(ch == 0xFF) return(0);
    if(ch == ',') return(0);
    if(ch == '\r') return(0);
    if(ch == '\n')
    {
      lstrptr[lstrindex++] = 0;
      lstrmode = false;
      return(0);
    }
    lstrptr[lstrindex++] = ch;
    return(0);
  }
  Token = GetToken(false);
  if (Token == NULL) return (-1);
  if (Token[0] == 0) return (-1);
  if((echoMode) && (!SerialMute))
  {
    if (strcmp(Token, "\n") != 0) 
    {
      if(delimiter!=0) serial->write(delimiter);
      serial->print(Token);
    }
    if (strcmp(Token, "\n") == 0) delimiter=0;
    else delimiter = ',';
  }
  switch (state)
  {
    case PCcmd:
      if (strcmp(Token, ";") == 0) break;
      if (strcmp(Token, "\n") == 0) break;
      CmdNum = -1;
      // Look for command in command table
      for (i = 0; CmdArray[i].Cmd != 0; i++) if (strcmp(Token, CmdArray[i].Cmd) == 0) 
      {
        CmdNum = i;
        break;
      }
      if (CmdNum == -1)
      {
        SetErrorCode(ERR_BADCMD);
        SendNAK;
        break;
      }
      // If this is a long string read command type then init the vaiable to support saving the
      // string directly to the provided pointer and exit. This function must not block
      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr = CmdArray[i].pointers.charPtr;
        lstrindex = 0;
        lstrmax = CmdArray[i].NumArgs;
        lstrmode = true;
        break;
      }
      if (CmdArray[i].NumArgs > 0) state = PCarg1;
      else state = PCend;
      break;
    case PCarg1:
      Sarg1[0]=0;
      sscanf(Token, "%d", &arg1);
      sscanf(Token, "%s", Sarg1);
      sscanf(Token, "%f", &farg1);
      if (CmdArray[CmdNum].NumArgs > 1) state = PCarg2;
      else state = PCend;
      break;
    case PCarg2:
      Sarg2[0]=0;
      sscanf(Token, "%d", &arg2);
      sscanf(Token, "%s", Sarg2);
      if (CmdArray[CmdNum].NumArgs > 2) state = PCarg3;
      else state = PCend;
      break;
    case PCarg3:
      sscanf(Token, "%f", &farg1);
      state = PCend;
      break;
    case PCend:
      if ((strcmp(Token, "\n") != 0) && (strcmp(Token, ";") != 0))
      {
        state = PCcmd;
        SendNAK;
        break;
      }
      i = CmdNum;
      CmdNum = -1;
      state = PCcmd;
      ExecuteCommand(&CmdArray[i], arg1, arg2, Sarg1, Sarg2, farg1);
      break;
    default:
      state = PCcmd;
      break;
  }
  return (0);
}

void RB_Init(Ring_Buffer *rb)
{
  rb->Head = 0;
  rb->Tail = 0;
  rb->Count = 0;
  rb->Commands = 0;
}

int RB_Size(Ring_Buffer *rb)
{
  return (rb->Count);
}

int RB_Commands(Ring_Buffer *rb)
{
  return (rb->Commands);
}

// Put character in ring buffer, return 0xFF if buffer is full and can't take a character.
// Return 0 if character is processed.
char RB_Put(Ring_Buffer *rb, char ch)
{
  if (rb->Count >= RB_BUF_SIZE) return (0xFF);
  rb->Buffer[rb->Tail] = ch;
  if (rb->Tail++ >= RB_BUF_SIZE - 1) rb->Tail = 0;
  rb->Count++;
  if (ch == ';') rb->Commands++;
  if (ch == '\r') rb->Commands++;
  if (ch == '\n') rb->Commands++;
  return (0);
}

// Get character from ring buffer, return NULL if empty.
char RB_Get(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0)
  {
    rb->Commands = 0;  // This has to be true if the buffer is empty...
    return (0xFF);
  }
  ch = rb->Buffer[rb->Head];
  if (rb->Head++ >= RB_BUF_SIZE - 1) rb->Head = 0;
  rb->Count--;
  // Map \r to \n
  //  if(Recording) MacroFile.write(ch);
  if (ch == '\r') ch = '\n';
  if (ch == ';') rb->Commands--;
  if (ch == '\n') rb->Commands--;
  if (rb->Commands < 0) rb->Commands = 0;
  return (ch);
}

// Return the next character in the ring buffer but do not remove it, return NULL if empty.
char RB_Next(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0) return (0xFF);
  ch = rb->Buffer[rb->Head];
  if (ch == '\r') ch = '\n';
  return (ch);
}

void PutCh(char ch)
{
  RB_Put(&RB, ch);
}

//
// The following function support the macro capibility.
//

// This function will open a file for recording a macro. This will result in all received chararcters from
// the host computer being saved in the macro file. If the file is already present it will be appended to.
void MacroRecord(char *filename)
{
  char fname[30];
  char ch;
  char *TK;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  // NAK if file name is too long
  if (strlen(filename) > 20)
  {
    SetErrorCode(ERR_FILENAMETOOLONG);
    SendNAK;
    return;
  }
  sprintf(fname, "%s.mac", filename);
  // Open the file
  if (!(MacroFile = SD.open(fname, FILE_WRITE)))
  {
    // Error opening file
    SetErrorCode(ERR_CANTOPENFILE);
    SendNAK;
    return;
  }
  // Set the Macro record flag
  Recording = true;
  // Fall into a loop and record all received characters until the command
  // MSTOP<CR> is received.
  while (1)
  {
    if (serial->available() > 0)
    {
      ch = serial->read();
      MacroFile.write(ch);
      // Put the character in the input ring buffer then pull tokens
      // looking for MSTOP
      RB_Put(&RB, ch);
      while (RB_Size(&RB) != 0)  if ((TK = GetToken(false)) != NULL) break;
      if (TK != NULL)
      {
        if (strcmp(TK, "MSTOP") == 0)
        {
          Recording = false;
          break;
        }
      }
    }
  }
  MacroFile.close();
  SendACK;
}

// Turn off macro recording. This function does nothing but will be called by the command
// processor.
void MacroStop(void)
{
}

// This function returns a pointer to a c style string that contains a comma seperated
// list of macro file names. This is used by the UI list function to allow the user to
// select a macro file.
// The extension is stripped from the file name.
char *MacroBuildList(char *current)
{
  File root, entry;
  String FileName;
  static String MacroList;

  if (!SDcardPresent) return NULL;
  SD.begin(_sdcs);
  root = SD.open("/", FILE_READ);
  root.rewindDirectory();
  MacroList = current;
  MacroList.toUpperCase();
  if (MacroList != "NONE") MacroList += ",NONE";
  while (true)
  {
    if (!(entry = root.openNextFile())) break;
    FileName = entry.name();
    FileName.toUpperCase();
    if ((FileName.endsWith(".MAC")) && (!FileName.startsWith("_")) && (!FileName.startsWith(".")))
    {
      MacroList += ",";
      MacroList += entry.name();
      MacroList.remove(MacroList.lastIndexOf(".MAC"));
    }
  }
  root.close();
  return (char *)MacroList.c_str();
}

// List of macro files on MIPS system
void MacroList(void)
{
  File root, entry;
  String FileName;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  root = SD.open("/", FILE_READ);
  SendACKonly;
  root.rewindDirectory();
  while (true)
  {
    if (!(entry = root.openNextFile())) break;
    FileName = entry.name();
    FileName.toUpperCase();
    if ((FileName.endsWith(".MAC")) && (!FileName.startsWith("_")) && (!FileName.startsWith(".")))
    {
      FileName.remove(FileName.lastIndexOf(".MAC"));
      if (!SerialMute) serial->println(FileName);
    }
  }
  if (!SerialMute) serial->println("End of macro file list.");
  root.close();
}

// Delete a macro file
void MacroDelete(char *filename)
{
  char fname[30];

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  // NAK if file name is too long
  if (strlen(filename) > 20)
  {
    SetErrorCode(ERR_FILENAMETOOLONG);
    SendNAK;
    return;
  }
  sprintf(fname, "%s.mac", filename);
  if (!SD.remove(fname))
  {
    SetErrorCode(ERR_CANTDELETEFILE);
    SendNAK;
    return;
  }
  SendACK;
}

// Ths function plays a macro. This is done by opening
// the macro file and placing all its contestents in the serial
// input ring buffer.
// All the normal serial responses are muted by setting the serial
// object to NULL.
// The last entry placed in the ring buffer is a command to unmute the
// serial response.
void MacroPlay(char *filename, bool silent)
{
  char fname[30];
  char *mute = "MUTE,ON\n";
  char *unmute = "MUTE,OFF\n";
  File mfile;
  int  i;

  if (!SDcardPresent)
  {
    if (!silent)
    {
      SetErrorCode(ERR_NOSDCARD);
      SendNAK;
    }
    return;
  }
  SD.begin(_sdcs);
  // NAK if file name is too long
  if (strlen(filename) > 20)
  {
    if (!silent)
    {
      SetErrorCode(ERR_FILENAMETOOLONG);
      SendNAK;
    }
    return;
  }
  sprintf(fname, "%s.mac", filename);
  // Open the file
  if (!(mfile = SD.open(fname, FILE_READ)))
  {
    // Error opening file
    if (!silent)
    {
      SetErrorCode(ERR_CANTOPENFILE);
      SendNAK;
    }
    return;
  }
  if (!silent) SendACK;
  // Mute the serial response
  for (i = 0; i < strlen(mute); i++) RB_Put(&RB, mute[i]);
  // Fill the ring buffer with the macro file contents
  while (true)
  {
    if ((i = mfile.read()) == -1) break;
    if (RB_Put(&RB, i) == 0xFF)
    {
      ProcessCommand();
      RB_Put(&RB, i);
    }
    //    serial->write(i);
  }
  mfile.close();
  // Unmute the serial system
  for (i = 0; i < strlen(unmute); i++) RB_Put(&RB, unmute[i]);
}






