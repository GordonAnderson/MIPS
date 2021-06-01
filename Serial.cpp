/*
 * serial.c
 *
 * This file contains the code for the serial IO hardware including the ring buffer
 * code and the serial command processor.
 *
 * Developed by: Gordon Anderson
 */
#include "Arduino.h"
#include <Wire.h>
#include "SD.h"
#include "Adafruit_ILI9340.h"
#include "string.h"
#include "Serial.h"
#include "Errors.h"
#include "Menu.h"
#include "Dialog.h"
#include "DIO.h"
//#include "Table.h"
#include "Hardware.h"
#include "ClockGenerator.h"
#include "DCbias.h"
#include "DCbiasList.h"
#include "ESI.h"
#include "RFamp.h"
#include "Twave.h"
#include "FAIMS.h"
#include "Filament.h"
#include "WiFi.h"
#include "ethernet.h"
#include "arb.h"
#include "adcdrv.h"
#include "FAIMSfb.h"
#include "TWIext.h"
#include <ThreadController.h>
#include "Variants.h"
#include "Table.h"
#include "SC16IS740.h"
#include "AtomicBlock.h"

extern ThreadController control;

Stream *serial = &SerialUSB;
//MIPSstream *mipsstream = &SerialUSB;
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

char *OnMessage = "I'm on!";

bool echoMode = false;

const Commands  CmdArray[] = 	{
// General commands
  {"NOP",  CMDfunction, 0, NULL},                        // Do nothing
  {"GVER",  CMDstr, 0, (char *)Version},                 // Report version
  {"ISPWR",  CMDstr, 0, (char *)OnMessage},              // Report power status
  {"SUPPLIES", CMDfunction, 0, (char *)ReportSupplies},  // Report main supply voltages
  {"V12", CMDfunction, 0, (char *)ReportV12},            // Report 12 volt supply
  {"V24", CMDfunction, 0, (char *)ReportV24},            // Report 24 volt supply
  {"CUR", CMDfunction, 0, (char *)ReportCur},            // Report input current, in amps
  {"PWR", CMDfunction, 0, (char *)ReportPower},          // Report input power in watts
  {"UPTIME", CMDfunction, 0, (char *)UpTime},            // Report uptime in mins
  {"GERR",  CMDint, 0, (char *)&ErrorCode},              // Report the last error code
  {"GNAME", CMDstr, 0, (char *)MIPSconfigData.Name},	   // Report MIPS system name
  {"SNAME", CMDstr, 1, (char *)MIPSconfigData.Name},     // Set MIPS system name
  {"SSERWD",CMDint, 1, (char *)&SerialWatchDog},         // Set serial watch dog time in seconds, 0 to negative to disable
  {"GSERWD",  CMDint, 0, (char *)&SerialWatchDog},       // Return serial watch dog time in seconds
  {"UUID", CMDfunction, 0, (char *)ReportUniqueID},      // Reports the microcontrollers unique ID, 128 bits, hex format  
  {"BIMAGE", CMDstr, 1, (char *)MIPSconfigData.BootImage}, // Set MIPS boot image
  {"ABOUT", CMDfunction, 0, (char *)About},              // Report about this MIPS system
  {"SMREV", CMDfunctionLine, 0, (char *)SetModuleRev},   // Set module rev level
  {"RESET", CMDfunction, 0, (char *)Software_Reset},     // Reset the Due  
  {"STATUS", CMDfunction, 0, (char *)RebootStatus},      // Reports the last reboot status and time in millisec from boot
  {"SAVE",  CMDfunction, 0, (char *)SAVEparms},	         // Save MIPS configuration data to default.cfg on SD card
  {"GCHAN", CMDfunctionStr, 1, (char *)GetNumChans},     // Report number for the selected system
  {"MUTE",  CMDfunctionStr, 1, (char *)Mute},            // Turns on and off the serial response from the MIPS system
  {"ECHO",  CMDbool, 1, (char *)&echoMode},              // Turns on and off the serial echo mode where the command is echoed to host, TRUE or FALSE
  {"TRIGOUT", CMDfunctionStr, 1, (char *)(static_cast<void (*)(char *)>(&TriggerOut))},    // Generates output trigger on rev 2 and higher controllers
                                                         // supports, HIGH,LOW,PULSE
  {"AUXOUT",  CMDfunctionStr, 1, (char *)AuxOut},        // Generates output trigger on Aux output, supports, HIGH,LOW,PULSE                                                         
  {"DELAY", CMDfunction, 1, (char *)DelayCommand},       // Generates a delay in milliseconds. This is used by the macro functions
                                                         // to define delays in voltage ramp up etc.
  {"GCMDS", CMDfunction, 0, (char *)GetCommands},	       // Send a list of all commands
  {"GAENA", CMDbool, 0, (char *)&MIPSconfigData.UseAnalog}, // Print the UseAnalog flag, true or false
  {"SAENA", CMDbool, 1, (char *)&MIPSconfigData.UseAnalog}, // Sets the UseAnalog flag, true or false
  {"THREADS", CMDfunction, 0, (char *)ListThreads},         // List all threads, there IDs, and there last runtimes
  {"STHRDENA", CMDfunctionStr, 2, (char *)SetThreadEnable}, // Set thread enable to true or false
  {"SDEVADD", CMDfunctionStr, 2, (char *)DefineDeviceAddress}, // Set device board and address
  {"RDEV", CMDfunction, 1, (char *)ReportAD7998},              // Read the ADC channel value, AD7998 device
  {"RDEV2", CMDfunction, 1, (char *)ReportAD7994},             // Read the ADC channel value, AD7994 device
  {"TBLTSKENA", CMDbool, 1, (char *)&TasksEnabled},            // Enables tasks in table mode, true or false
  {"LEDOVRD", CMDbool, 1, (char *)&LEDoverride},               // Override the LED operation is true, always false on startup
  {"LED",  CMDint, 1, (char *)&LEDstate},                      // Define the LEDs state you are looking for.
  {"DSPOFF", CMDbool, 1, (char *)&DisableDisplay},             // Print the UseAnalog flag, true or false
  {"STRTDLY", CMDint, 1, (char *)&MIPSconfigData.StartupDelay},// Startup delay in seconds
  {"CHKIMAGE",  CMDfunctionStr, 1, (char *)CheckImage},        // Reports the image file status
  {"LOADIMAGE",  CMDfunctionStr, 1, (char *)LoadImage},        // Loads an image file to the display
  {"SSERIALNAV", CMDbool, 1, (char *)&EnableSerialNavigation}, // Set flag to TRUE to enable UI navigation from the host interface
  {"TRACE", CMDfunction, 0, (char *)TraceEnable},              // Enable the trace function 
  {"DIR", CMDfunction, 0, (char *)ListFiles},                  // List all file in the SD card
  {"DEL", CMDfunctionStr, 1, (char *)DeleteFile},              // Delete file on the SD card
  {"GET", CMDfunctionStr, 1, (char *)GetFile},                 // Dump file contents, hex, from SD card file
  {"PUT", CMDfunctionStr, 2, (char *)PutFile},                 // Create file on SD card from host interface
  {"SAVEMOD", CMDfunctionLine, 0, (char *)EEPROMtoSD},         // Save module EEPROM to SD file. Filename,Board (A or B),Add (hex)
  {"LOADMOD", CMDfunctionLine, 0, (char *)SDtoEEPROM},         // Load module EEPROM from SD file. Filename,Board (A or B),Add (hex)
  {"SAVEALL", CMDfunction, 0, (char *)SaveAlltoSD},            // Saves all the modules EEPROM data to the SD card
  {"LOADALL", CMDfunction, 0, (char *)LoadAllfromSD},          // Loads all the modules EEPROM data from the SD card
  {"GETEEPROM", CMDfunctionStr, 2, (char *)EEPROMtoSerial},    // Sends the selected EEPROM data to the host, Board( A or B), Add (Hex)
  {"PUTEEPROM", CMDfunctionStr, 2, (char *)SerialtoEEPROM},    // Receives data from the host and writes to EEPROM, Board( A or B), Add (Hex)
  {"SSPND", CMDbool, 1, (char *)&Suspend},                     // Suspend all tasks, suspends real time control, TRUE or FALSE
  {"GSPND", CMDbool, 0, (char *)&Suspend},                     // Returns suspend status, TRUE or FALSE
  {"CPUTEMP", CMDfunction, 0, (char *)CPUtemp},                // Returns the CPU temp in degrees C, not an accurate reading 
  {"TWITALK", CMDfunction, 2, (char *)TWItalk},                // Redirect the serial communications through a board and TWI address passed
  {"TWI1TALK", CMDfunction, 2, (char *)TWI1talk},              // Redirect the serial communications through a board and TWI address passed
  {"SSER1ENA", CMDbool, 1, (char *)&MIPSconfigData.Ser1ena},   // Set the Serial1 port enable, TRUE = general use
  {"GSER1ENA", CMDbool, 0, (char *)&MIPSconfigData.Ser1ena},   // Get the Serial1 port enable status
  {"SETADDRESS", CMDfunctionStr, 1, (char *)&SetMemAddress},   // Set memory address to read or write, hex
  {"WRITE", CMDfunctionStr, 2, (char *)&WriteMemory},          // Write to memory, BYTE,WORD,DWORD,INT,FLOAT
  {"READ", CMDfunctionStr, 1, (char *)&ReadMemory},            // Read from memory, BYTE,WORD,DWORD,INT,FLOAT
  {"FORMAT", CMDfunctionLine, 0, (char *)&FormatEEPROM},       // Format the EEPROM, args example,A 0x50,RFdrvA R1  
  {"SAVEM", CMDfunctionStr, 1, (char *)&SaveModule},           // Save the defined module to EEPROM  
  {"TWIRESET", CMDfunction, 0, (char *)TWIreset},              // Resets the TWI interface
  {"TWIERROR",  CMDint, 0, (char *)&TWIfails},                 // Reports the numner of detected TWI failures 
  {"STWIHDW", CMDbool, 1, (char *)&MIPSconfigData.TWIhardware},   // If true then the TWI hardware interface is used for ADC read functions
  {"GTWIHDW", CMDbool, 0, (char *)&MIPSconfigData.TWIhardware},   // Returns the current status.
  {"STBLRETRIG", CMDbool, 1, (char *)&MIPSconfigData.TableRetrig},// Set the table retriggerable flag, TRUE=retriggerable.
  {"GTBLRETRIG", CMDbool, 0, (char *)&MIPSconfigData.TableRetrig},// Returns the table retriggerable flag, TRUE=retriggerable.
  {"SSETECHO", CMDbool, 1, (char *)&Serial1Echo},                 // Set the SerialUSB to Serial1 echo, TRUE enables echo.
  {"GSERECHO", CMDbool, 0, (char *)&Serial1Echo},                 // Return the SerialUSB to Serial1 echo, TRUE enables echo.
  {"POWER", CMDfunction, 0, (char *)PowerControl},                // Cycle MIPS system power
  {"SUBRDSEL", CMDbool, 1, (char *)&MIPSconfigData.UseBRDSEL},    // Set to true to enable the use of board select, default
  {"GUBRDSEL", CMDbool, 0, (char *)&MIPSconfigData.UseBRDSEL},    // Returns the state of the UseBRDSEL flag
  {"USBPWR", CMDfunctionStr, 1, (char *)&USBpower},               // Enables or disables USB powering controller. TRUE = enable. Note if false
                                                                  // and power is lost the system will reboot to being powered by USB                                                                 
  {"DREAD", CMDfunction, 1, (char *)Dread},                       // Read an arduino pin, returns HIGH or LOW
  {"DWRITE", CMDfunctionStr, 2, (char *)Dwrite},                  // Write an arduino pin, Set HIGH or LOW or PULSE or SPLUSE
  {"DSET", CMDfunctionStr, 2, (char *)Dset},                      // Sets an arduino pin, INPUT, PULLUP, OUTPUT  
  {"DEBUG", CMDfunction, 1, (char *)Debug},                       // Debug function, as needed
  {"TWICMD", CMDfunctionLine, 0, (char *)&TWIscmd},               // Sends a command to the addressed TWI (Wire) address, TWICMD,brd,add,string. 
                                                                  // add is decimal, string is \n terminated
  {"TWI1CMD", CMDfunctionLine, 0, (char *)&TWI1scmd},             // Sends a command to the addressed TWI1 (Wire1) address, TWICMD,add,string.
                                                                  // add is decimal, string is \n terminated
  {"STRPLVL", CMDfloat, 1, (char *)&MIPSconfigData.VerrorThreshold},// Set the DCbias power supply readback error trip point in percentage of FS
  {"GTRPLVL", CMDfloat, 0, (char *)&MIPSconfigData.VerrorThreshold},// Returns the DCbias power supply readback error trip point in percentage of FS
// Log commands
  {"SLOGENA", CMDbool, 1, (char *)&logdata.enabled},                // Enables logging is TRUE, disable if FALSE
  {"GLOGENA", CMDbool, 0, (char *)&logdata.enabled},                // Returns the log enable status
  {"LOGREP", CMDfunction, 0, (char *)ReportLogEntry},               // Reports the current log entries
// Real time clock functions. Note, no battery backup so the clock reset every time the hardware boots.
  {"STIME", CMDfunctionLine, 0, (char *)&SetTime},                  // Sets the current time, 00:00:00 format
  {"GTIME", CMDfunction, 0, (char *)&GetTime},                      // Returns the current time, 00:00:00 format
  {"SDATE", CMDfunctionLine, 0, (char *)&SetDate},                  // Sets the current date, dd/mm/yyyy format
  {"GDATE", CMDfunction, 0, (char *)&GetDate},                      // Returns the current date, dd/mm/yyyy format
// Level detection module change enable
  {"ENALDET", CMDfunctionStr, 1, (char *)SetLevelDetChangeReport},  // Enables the level detection module to report a detected change to the host.
                                                                    // This function requires the module address in hex.
// Clock generation functions
  {"GWIDTH",  CMDint, 0, (char *)&PulseWidth},                // Report the pulse width in microseconds
  {"SWIDTH",  CMDint, 1, (char *)&PulseWidth},                // Set the pulse width in microseconds
  {"GFREQ",  CMDint, 0, (char *)&PulseFreq},                  // Report the pulse frequency in Hz
  {"SFREQ",  CMDint, 1, (char *)&PulseFreq},                  // Set the pulse frequency in Hz
  {"BURST",  CMDfunction, 1, (char *)&GenerateBurst},         // Generates a frequencyy burst on trig out line
  {"GMAXFTRIG",  CMDint, 0, (char *)&TrigMax},                // Report the maximum frequency for trig out, 0 = nolimit
  {"SMAXFTRIG",  CMDint, 1, (char *)&TrigMax},                // Sets the maximum frequency for trig out, 0 = nolimit
  {"GMAXFATRIG",  CMDint, 0, (char *)&AuxTrigMax},            // Report the maximum frequency for aux trig out, 0 = nolimit
  {"SMAXFATRIG",  CMDint, 1, (char *)&AuxTrigMax},            // Sets the maximum frequency for aux trig out, 0 = nolimit
// Delay trigger functions. This supports delayed trigger and retriggering of supported modules
  {"SDTRIGINP", CMDfunctionStr, 2, (char *)SetDelayTrigInput},// Set delay trigger input (Q-X) and level, POS or NEG
  {"SDTRIGDLY",  CMDint, 1, (char *)&DtrigDelay},             // Set trigger delay in uS
  {"GDTRIGDLY",  CMDint, 0, (char *)&DtrigDelay},             // Returns trigger delay in uS
  {"SDTRIGPRD",  CMDint, 1, (char *)&DtrigPeriod},            // Set trigger delay repeat period in uS
  {"GDTRIGPRD",  CMDint, 0, (char *)&DtrigPeriod},            // Returns trigger delay repeat period in uS
  {"SDTRIGRPT",  CMDint, 1, (char *)&DtrigNumber},            // Defines the number of trigger repeats, 0 = forever
  {"GDTRIGRPT",  CMDint, 0, (char *)&DtrigNumber},            // Returns the number of trigger repeats, 0 = forever
  {"SDTRIGMOD", CMDfunctionStr, 1, (char *)SetDelayTrigModule},// Defines the delay trigger module, ARB, ADC, SWEEP, AUXTRIG
  {"SDTRIGENA", CMDfunctionStr, 1, (char *)SetDelayTrigEnable},// TRUE enables the trigger FALSE disables
  {"GDTRIGENA", CMDbool, 0, (char *)&DtrigEnable},             // Returns the trigger delay enable status
// ADC functions
  {"ADC", CMDfunction, 1, (char *)ADCread},                    // Read and report ADC channel. Valid range 0 through 3
  {"ADCGAIN", CMDfunction, 2, (char *)ADRsetGain},             // Set ADC channel gain. Channel range 0 to 3, gains are 1,2, or 4
  {"ADCCHG", CMDfunction, 2, (char *)ADCchangeDet},            // Enable ADC change detection. Channel range 0 to 3, threshold 0 to 100
  {"ADCCPRM", CMDfunction, 2, (char *)SetADCchangeParms},      // Set ADC change filter parameters
  {"RADCCHG", CMDfunctionStr, 1, (char *)MonitorADCchange},    // This function will report ADC value changes to the host, the ADC value is multiplied by gain
  {"ADCINIT", CMDfunction, 0, (char *)ADCprep},                // ADC vector recording setup
  {"ADCTRIG", CMDfunction, 0, (char *)ADCsoftTrigger},         // ADC vector recording software trigger
  {"ADCABORT", CMDfunction, 0, (char *)ADCabort},              // ADC system abort
  {"SADCCHAN", CMDint, 1, (char *)&ADCchannel},                // Set ADC channel number
  {"GADCCHAN", CMDint, 0, (char *)&ADCchannel},                // Get ADC channel number
  {"SADCSAMPS", CMDint, 1, (char *)&ADCnumsamples},            // Set ADC number of samples
  {"GADCSAMPS", CMDint, 0, (char *)&ADCnumsamples},            // Get ADC number of samples
  {"SADCVECTS", CMDint, 1, (char *)&ADCvectors},               // Set ADC number of vectors
  {"GADCVECTS", CMDint, 0, (char *)&ADCvectors},               // Get ADC number of vectors
  {"SADCRATE", CMDint, 1, (char *)&ADCrate},                   // Set ADC sample rate in Hz
  {"GADCRATE", CMDint, 0, (char *)&ADCrate},                   // Get ADC sample rate in Hz
// DC bias module commands
  {"SDCB", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&DCbiasSet))},      // Set voltage value
  {"GDCB", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasRead))},// Get Voltage value requested
  {"GDCBV", CMDfunction, 1, (char *)DCbiasReadV},                     // Get Voltage actual voltage value
  {"SDCBOF", CMDfunctionStr, 2, (char *)DCbiasSetFloat},              // Set float voltage for selected board
  {"GDCBOF", CMDfunction, 1, (char *)DCbiasReadFloat},                // Read float voltage for selected board
  {"GDCMIN", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasReadMin))}, // Read float voltage for selected board
  {"GDCMAX", CMDfunction, 1, (char *)(static_cast<void (*)(int)>(&DCbiasReadMax))}, // Read float voltage for selected board
  {"SDCPWR", CMDfunctionStr, 1, (char *)DCbiasPowerSet},              // Sets the DC bias power, on or off
  {"GDCPWR", CMDfunction, 0, (char *)DCbiasPower},                    // Get the DC bias power, on or off
  {"SDCBALL", CMDfunctionLine, 0, (char *)SetAllDCbiasChannels},      // Sets the DCbias setpoints for all outputs
  {"GDCBALL", CMDfunction, 0, (char *)DCbiasReportAllSetpoints},      // Returns the DC bias voltage setpoints for all channels in the system
  {"GDCBALLV", CMDfunction, 0, (char *)DCbiasReportAllValues},        // Returns the DC bias voltage readback values for all channels in the system
  {"SDCBDELTA", CMDfunctionStr, 1, (char *)DCbiasDelta},              // Set all DC bias channels by a delta value
  {"SDCBCHNS", CMDfunction, 2, (char *)DCbiasSetNumBoardChans},       // Sets the number of channels on a DCbias board. Used for setup only.
  {"SDCBONEOFF", CMDfunctionStr, 1, (char *)DCbiasUseOneOffset},      // TRUE to enable use of one offset
  {"DCBOFFRBENA", CMDfunctionStr, 1, (char *)DCbiasOffsetReadback},   // TRUE to enable use of offset readback
  {"SDCBOFFENA", CMDfunctionStr, 2, (char *)DCbiasOffsetable},        // Set the DC bias channels offsetable flag, setup command
  {"SDCBTEST", CMDbool, 1, (char *)&DCbiasTestEnable},                // Set to FALSE to disable readback testing
  {"SDCBADCADD", CMDfunction, 2, (char *)SetDCbiasADCtwiADD},         // Set a modules ADC TWI address, radix 10
  {"SDCBDACADD", CMDfunction, 2, (char *)SetDCbiasDACtwiADD},         // Set a modules DAC TWI address, radix 10
  {"SDCBARST", CMDbool, 1, (char *)&AutoReset},                       // Set to TRUE to enable power supply auto reset
  {"SDCBRNG", CMDfunction, 2, (char *)&SetDCbiasRange},               // Set the range for the DC bias board
  {"SDCBLMT", CMDfunction, 2, (char *)&SetDCbiasLimit},               // Set the limit for the DC bias board
  {"SDCBEXT", CMDfunction, 1, (char *)&SetDCbiasExtended},            // Set the DCbias board for extended addressing, factory command
  {"RDCBSPLY", CMDfunction, 1, (char *)&ReportDCbiasSuppplies},       // Report the DCbias board supply voltages. Requires AD5593 for this
                                                                      // function to work.
  {"SDCBUPDATE", CMDbool, 1, (char *)&DCbiasUpdate},                  // Set to TRUE to force all DC bias channels to update
  {"GDCBCAL", CMDfunction, 1, (char *)DCbiasCalParms},                // This command will return the selected channels calibration parameters
  {"SDCCALM", CMDfunctionStr, 2, (char *)DCbiasCalsetM},              // Set the DC bias channels cal parameter M
  {"SDCCALB", CMDfunctionStr, 2, (char *)DCbiasCalsetB},              // Set the DC bias channels cal parameter B
//DC bias module offset control
  {"SDCBOFOF", CMDfunctionStr, 2, (char *)SetDCbiasOffOff},           // Set the DC bias global offset, will apply to all channels on module
  {"GDCBOFOF", CMDfunction, 1, (char *)GetDCbiasOffOff},              // Returns the DC bias global offset
  {"SDCBCHOF", CMDfunctionStr, 2, (char *)SetDCbiasCHOff},            // Set the DC bias channel offset, will apply to all enabled channels on the module
  {"GDCBCHOF", CMDfunction, 1, (char *)GetDCbiasCHOff},               // Returns the DC bias channel offset
  {"SDCBCHMK", CMDfunctionStr, 2, (char *)SetDCbiasCHMK},             // Set the DC bias channel offset mask, value in hex, defines channels to apply offset
  {"GDCBCHMK", CMDfunction, 1, (char *)GetDCbiasCHMK},                // Returns the DC bias channel offset mask, in hex
  {"DCBCADCOF", CMDfunctionStr, 2, (char *)SetADCoffsetAdjust},       // Connect the ADC change detector to the OffsetOffset on select board
  {"DCBCADCCO", CMDfunctionStr, 2, (char *)SetADCchannelAdjust},      // Connect the ADC change detector to the Channel Offset on select board
  {"SDCBCADCP", CMDfunctionStr, 2, (char *)SetADCgainPol},            // Sets the ADC gain polarity control DIO, Q thru X or NA
  {"GDCBCADCP", CMDfunction, 1, (char *)GetADCgainPol},               // Returns the ADC gain polarity control DIO

  {"DCBCLDOF", CMDfunctionStr, 2, (char *)SetLevelDetOffsetAdjust},   // Connect the Level Detect module to the OffsetOffset on selected board
  {"DCBCLDCH", CMDfunctionStr, 2, (char *)SetLevelDetChOffsetAdjust}, // Connect the Level Detect module to the Channel offset on selected board
// DC bias module profile commands
  {"SDCBPRO", CMDfunctionLine, 0, (char *)SetDCbiasProfile},          // Sets a DC bias profile
  {"GDCBPRO", CMDfunction, 1, (char *)GetDCbiasProfile},              // Reports the select DC bias profile
  {"ADCBPRO", CMDfunction, 1, (char *)SetApplyDCbiasProfile},         // Applies the select DC bias profile
  {"CDCBPRO", CMDfunction, 1, (char *)SetDCbiasProfileFromCurrent},   // Copy the current DC bias values to the select profile
  {"TDCBPRO", CMDfunctionLine, 0, (char *)SetDCbiasProfileToggle},    // Enables toggeling between two profiles with user defined dwell time, mS
  {"TDCBSTP", CMDfunction, 0, (char *)StopProfileToggle},             // Stop the profile toggeling
// DC bias list functions, supports DMA high speed transfer
  {"DSTATE", CMDfunctionLine, 0, (char *)DefineState},                // Define a state, name,ch,val....
  {"SSTATE", CMDfunctionStr, 1, (char *)SetState},                    // Sets the DCbias channels to the values defined in named state
  {"LSTATES", CMDfunction, 0, (char *)ListStateNames},                // List all the defined state names
  {"RSTATE", CMDfunctionStr, 1, (char *)RemoveState},                 // Remove a state from the linked list
  {"RSTATES", CMDfunction, 0, (char *)RemoveStates},                  // Clear the full linked list of states
  {"GSTATE", CMDfunctionStr, 1, (char *)IsState},                     // Returns true if named state is in list, else false
  {"DSEGMENT", CMDfunctionLine, 0, (char *)DefineSegment},            // Defines a segment with the following arguments: name, length, next, repeat count
  {"ADDSEGTP", CMDfunctionLine, 0, (char *)AddSegmentTimePoint},      // Adds a time point to a segment, arguments: name,count, state1, state 2... (variable number of states)
  {"ADDSEGTRG", CMDfunctionLine, 0, (char *)AddSegmentTrigger},       // Adds a trigger point to a segment, arguments: name,count,port,level
  {"ADDSEGSTRG", CMDfunctionLine, 0, (char *)AddSegmentStartTrigger}, // Defines the start trigger for a segment, arguments: name,source,level
  {"LSEGMENTS", CMDfunction, 0, (char *)ListSegments},                // List all the defined segments
  {"RSEGMENT", CMDfunctionStr, 1, (char *)RemoveSegment},             // Remove a segment from the linked list
  {"RSEGMENTS", CMDfunction, 0, (char *)RemoveSegments},              // Clear the full linked list of segments
  {"PSEGMENTS", CMDfunction, 0, (char *)PlaySegments},                // Execute the segment list
  {"SABORT", CMDfunction, 0, (char *)AbortSegments},                  // Abort an executing the segment list
  {"TSEGMENT", CMDfunction, 0, (char *)SoftTriggerSegment},           // Software trigger the segment if ready
  {"RCURSEG", CMDfunction, 0, (char *)ReportCurrentSegment},          // Report the current segment
  {"FRCTRIG", CMDfunction, 0, (char *)ForceTrigger},                  // Force a trigger of the current segment
// DC bias channel pulse commands
  {"SDCBPCH", CMDint, 1, (char *)&DCbiasPchan},                       // Set DC bias pulse channel
  {"GDCBPCH", CMDint, 0, (char *)&DCbiasPchan},                       // Get DC bias pulse channel
  {"SDCBPV", CMDfloat, 1, (char *)&DCbiasPvoltage},                   // Set DC bias pulse voltage, in volts
  {"GDCBPV", CMDfloat, 0, (char *)&DCbiasPvoltage},                   // Get DC bias pulse voltage, in volts
  {"SDCBPD", CMDint, 1, (char *)&DCbiasPdelay},                       // Set DC bias pulse delay, in uS
  {"GDCBPD", CMDint, 0, (char *)&DCbiasPdelay},                       // Get DC bias pulse delay, in uS
  {"SDCBPW", CMDint, 1, (char *)&DCbiasPwidth},                       // Set DC bias pulse width, in uS
  {"GDCBPW", CMDint, 0, (char *)&DCbiasPwidth},                       // Get DC bias pulse width, in uS
  {"SDCBPT", CMDfunctionStr, 1, (char *)SetDCbiasPtrigger},           // Set DC bias pulse trigger source, Q-X or t
  {"GDCBPT", CMDfunction, 0, (char *)GetDCbiasPtrigger},              // Get DC bias pulse trigger source, Q-X or t
  {"SDCBPENA", CMDfunctionStr, 1, (char *)SetDCbiasPena},             // Set DC bias pulse enable, TRUE or FALSE
  {"GDCBPENA", CMDbool, 0, (char *)&DCbiasPena},                      // Get DC bias pulse enable, TRUE or FALSE
// DCbias serial calibration functions
  {"CDCBCH", CMDfunction, 1, (char *)CalDCbiasChannel},               // Calibrate the requested DC bias channel
  {"CDCBCHS", CMDfunction, 0, (char *)CalDCbiasChannels},             // Calibrate all DC bias channel in order
  {"CDCBOFF", CMDfunction, 1, (char *)CalDCbiasOffset},               // Calibrate all DC bias offset for channel number
// RF generator module commands
  {"SRFFRQ", CMDfunction, 2, (char *)RFfreq},		 // Set RF frequency
  {"SRFVLT", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&RFvoltage))},	 // Set RF output voltage
  {"SRFDRV", CMDfunctionStr, 2, (char *)(static_cast<void (*)(char *, char *)>(&RFdrive))},    // Set RF drive level
  {"GRFFRQ", CMDfunction, 1, (char *)RFfreqReport},	     // Report RF frequency
  {"GRFPPVP", CMDfunction, 1, (char *)RFvoltageReportP}, // Report RF output voltage, positive phase
  {"GRFPPVN", CMDfunction, 1, (char *)RFvoltageReportN}, // Report RF output voltage, negative phase
  {"GRFDRV", CMDfunction, 1, (char *)RFdriveReport},     // Report RF drive level in percentage
  {"GRFVLT", CMDfunction, 1, (char *)RFvoltageReport},   // Report RF output voltage setpoint
  {"GRFPWR", CMDfunction, 1, (char *)RFheadPower},       // Report RF head power draw
  {"GRFMODE", CMDfunction, 1, (char *)RFmodeReport},     // Report RF mode (MANUAL | AUTO) for the selected channel
  {"SRFMODE", CMDfunctionStr, 2, (char *)RFmodeSet},     // Sets the RF mode (MANUAL | AUTO) for the selected channel
  {"GRFALL", CMDfunction, 0, (char *)RFreportAll},       // Reports Freq, RFVpp + and - for each RF channel in system
  {"TUNERFCH", CMDfunction, 1, (char *)RFautoTune},      // Auto tune the select RF channel
  {"RETUNERFCH", CMDfunction, 1, (char *)RFautoRetune},  // Auto retune the select RF channel, start and current freq and drive
  {"SRFCAL", CMDfunctionLine, 1, (char *)RFcalParms},    // Sets the RF calibration parameters, channel,slope,intercept  
  {"RFCALP", CMDfunctionStr, 2, (char *)RFcalP},         // Adjust the calibration for a RF+ channel, channel,actual level in Vp-p, enter negative to set defaults
  {"RFCALN", CMDfunctionStr, 2, (char *)RFcalN},         // Adjust the calibration for a RF- channel, channel,actual level in Vp-p, enter negative to set defaults
  {"SRFPL", CMDfunction, 2, (char *)SetRFpwrLimit},      // Sets the RF power limit for the given channel, in watts
  {"GRFPL", CMDfunction, 1, (char *)GetRFpwrLimit},      // Returns the RF power limit for the given channel, in watts
// RF amplifier / QUAD commands
  {"SRFAENA", CMDfunctionStr, 2, (char *)RFAsetENA},         // Sets the RF system enable mode, ON or OFF
  {"GRFAENA", CMDfunction, 1, (char *)RFAgetENA},            // Returns the RF system enable mode, ON or OFF
  {"SRFAFREQ", CMDfunction, 2, (char *)RFAsetFreq},          // Sets the RF system frequency
  {"GRFAFREQ", CMDfunction, 1, (char *)RFAgetFreq},          // Returns the RF frequency
  {"SRFAK", CMDfunctionStr, 2, (char *)RFAsetK},             // Sets the resolving DC voltage ratio
  {"GRFAK", CMDfunction, 1, (char *)RFAgetK},                // Returns the resolving DC voltage ratio
  {"SRFAMOD", CMDfunctionStr, 2, (char *)RFAsetMode},        // Sets the RF system to open or closed loop
  {"GRFAMOD", CMDfunction, 1, (char *)RFAgetMode},           // Returns the RF mode state, open or closed
  {"SRFADRV", CMDfunctionStr, 2, (char *)RFAsetDrive},       // Sets the RF drive level to 0 to 100 %
  {"GRFADRV", CMDfunction, 1, (char *)RFAgetDrive},          // Returns the RF drive level
  {"SRFALEV", CMDfunctionStr, 2, (char *)RFAsetLevel},       // Sets the RF output voltage setpoint
  {"GRFALEV", CMDfunction, 1, (char *)RFAgetLevel},          // Returns the RF output voltage setpoint
  {"GRFAVPPA", CMDfunction, 1, (char *)RFAgetVPPA},          // Returns the RF output A actual level
  {"GFRAVPPB", CMDfunction, 1, (char *)RFAgetVPPB},          // Returns the RF output B actual level
  {"GRFAPWR", CMDfunction, 1, (char *)RFAgetPWR},            // Returns the RF amp RF forward pwr
  {"SRFARNG", CMDfunctionStr, 2, (char *)RFAsetRange},       // Sets the QUAD RF maximum RF level
  {"GRFARNG", CMDfunction, 1, (char *)RFAgetRange},          // Returns the QUAD RF maximum RF level
  {"SRFAPB", CMDfunctionStr, 2, (char *)RFAsetPoleBias},     // Sets the pole bias DC
  {"GRFAPB", CMDfunction, 1, (char *)RFAgetPoleBias},        // Returns the pole bias DC
  {"SRFARDC", CMDfunctionStr, 2, (char *)RFAsetResolvingDC}, // Sets the resolving DC + and - voltages using DC bias channels 1 and 2
  {"GRFARDC", CMDfunction, 1, (char *)RFAgetResolvingDC},    // Returns the resolving DC + and - voltages using DC bias channels 1 and 2
  {"SRFAR0", CMDfunctionStr, 2, (char *)RFAsetRo},           // Sets the Ro value in mm
  {"GRFAR0", CMDfunction, 1, (char *)RFAgetRo},              // Returns the Ro value in mm
  {"SRFAMZ", CMDfunctionStr, 2, (char *)RFAsetMZ},           // Sets the m/z in amu
  {"GRFAMZ", CMDfunction, 1, (char *)RFAgetMZ},              // Returns the m/z in amu
  {"SRFARES", CMDfunctionStr, 2, (char *)RFAsetRes},         // Sets the resolution in AMU
  {"GRFARES", CMDfunction, 1, (char *)RFAgetRes},            // Returns the resolution in AMU
  {"RFAQUPDATE", CMDfunction, 1, (char *)RFAupdateQUAD},     // Updates the QUAD parameters
  {"SRFAGAIN", CMDfunctionStr, 2, (char *)RFAsetGain},       // Sets RF head level control gain, HIGH or LOW
  {"GRFAGAIN", CMDfunction, 1, (char *)RFAreturnGain},       // Returns RF head level control gain, HIGH or LOW
  {"RRFAAMP", CMDfunction, 1, (char *)RFAreport},            // Reports RF amplifier parameters
  {"SRFADCCH", CMDfunction, 2, (char *)RFAsetDCBchan},       // Sets the RF quad dc bias channel used for resolving DC
  {"GRFADCCH", CMDfunction, 1, (char *)RFAgetDCBchan},       // Reports the RF quad dc bias channel used for resolving DC
// DIO module commands
  {"SDIO", CMDfunctionStr, 2, (char *)SDIO_Serial},	        // Set DIO output bit
  {"GDIO", CMDfunctionStr, 1, (char *)GDIO_Serial},	        // Get DIO output bit
  {"RPT", CMDfunctionStr, 2, (char *)DIOreport},            // Report an input state change
  {"MIRROR", CMDfunctionStr, 2, (char *)DIOmirror},         // Mirror an input to an output
  {"MDIO", CMDfunctionStr, 2, (char *)DIOmonitor},          // Monitors a digital input for a state change
  {"RDIO", CMDfunctionStr, 1, (char *)DIOchangeReport},     // Returns true if a state change was detected, else return false
  {"SDIINV", CMDbyte, 1, (char *)&MIPSconfigData.DIinvert}, // Sets digital input inversion mask, radix 10
  {"GDIINV", CMDbyte, 0, (char *)&MIPSconfigData.DIinvert}, // Returns digital input inversion mask, radix 10
// ESI module commands
  {"SHV", CMDfunctionStr, 2, (char *)SetESIchannel},        // Set channel high voltage
  {"GHV", CMDfunction, 1, (char *)GetESIchannel},           // Returns the high voltage setpoint
  {"GHVV", CMDfunction, 1, (char *)GetESIchannelV},         // Returns the actual high voltage output
  {"GHVI", CMDfunction, 1, (char *)GetESIchannelI},         // Returns the output current in mA
  {"GHVMAX", CMDfunction, 1, (char *)GetESIchannelMax},     // Returns the maximum high voltage outut value
  {"GHVMIN", CMDfunction, 1, (char *)GetESIchannelMin},     // Returns the minimum high voltage outut value
  {"SHVEGHNA", CMDfunction, 1, (char *)SetESIchannelEnable},  // Enables a selected channel
  {"SHVDIS", CMDfunction, 1, (char *)SetESIchannelDisable}, // Disables a selected channel
  {"GHVSTATUS", CMDfunction, 1, (char *)GetESIstatus},      // Returns the selected channel's status, ON or OFF
  {"SHVPSUP", CMDfunction, 2, (char *)SetESImodulePos},     // Sets a  modules positive supply voltage
  {"SHVNSUP", CMDfunction, 2, (char *)SetESImoduleNeg},     // Sets a  modules negative supply voltage
  {"GHVITST", CMDbool, 0, (char *)&ESIcurrentTest},         // Returns TRUE is current testing is enabled, else FALSE
  {"SHVITST", CMDbool, 1, (char *)&ESIcurrentTest},         // Set to TRUE to enable ESI current testing, FALSE to disable
  {"SHVENAGT", CMDfunctionStr, 2, (char *)SetESIgateEnable},// Enables a selected module gate option. This will enable all modules
                                                            // in the system for any enabled module. Factory setup command.
                                                            // This should only be enabled for firmware rev 4.0.
                                                            // TRUE to enable FALSE to disable
  {"SHVRAMP", CMDfunction, 2, (char *)SetESIramp},          // Set ESI voltage ramp rate, this will effect both channels on a module.
                                                            // Units are V/0.1s
  {"GHVRAMP", CMDfunction, 1, (char *)GetESIramp},          // Returns ESI voltage ramp rate for the selected module
                                                            
  
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
  {"GTBLVLT", CMDfunction, 2, (char *)GetTableEntryValue},    // Get a value from a loaded table
  {"STBLCNT", CMDfun2int1flt, 3, (char *)SetTableEntryCount}, // Set a count value in a loaded table
  {"STBLDLY", CMDint, 1, (char *)&InterTableDelay},        // Defines the inter table delay in milli seconds
  {"SOFTLDAC",CMDbool, 1, (char *)&softLDAC},              // TRUE or FALSE, set to TRUE to force the used of software LDAC
  {"GTBLREPLY",CMDbool, 0, (char *)&TableResponse},        // Returns TRUE or FALSE state of enable table response flag
  {"STBLREPLY",CMDbool, 1, (char *)&TableResponse},        // TRUE or FALSE, set to TRUE enable table response messages (default)
  {"TBLRPT",CMDfunction,1, (char *)ReportTable},           // Report table as hex bytes
  {"STBLTSKS",CMDbool, 1, (char *)&TblTasks},              // Set to TRUE to enable tasks in table mode based on avalible time, FALSE is default
  {"GTBLTSKS",CMDbool, 0, (char *)&TblTasks},              // Returns the TblTasks flag status
  {"SEXTFREQ",CMDint, 1, (char *)&ExtFreq},                // Sets the external frequency used for the table clock, needed for TblTasks capability, 0 by default
  {"GEXTFREQ",CMDint, 0, (char *)&ExtFreq},                // Returns the ExtFreq value, in Hz
  #if TABLE2code
  {"TBLCHK",CMDfunction, 0, (char *)TableCheck},           // The function tests a tbale for timing violations and prints the results
  {"STBLRMPENA", CMDfunctionStr, 2, (char *)EnableRamp},   // Enable the table ramp mode and set ramp ISR frequency
  // ADC change triggering of table
  {"STPADJ",CMDfunction,2, (char *)SelectTPforAdjust},     // Select table time point for adjustment on ADC change detection
  {"SADJRNG",CMDfunction,2, (char *)DefineAdjustRange},    // Define the time point valid range for ADC
  {"SADCMZCAL", CMDfunctionStr, 2, (char *)SetADCtoMZcal}, // Set ADC to m/z calibration parameters, m and b
  {"SMZTARG", CMDfunctionLine, 1, (char *)DefineMZtarget}, // Define target mz. parms index, mx, coount
  {"TRGTBLADC",CMDfunction, 0, (char *)TableTrigOnADC},    // Enables triggering on ADC change 
  {"TRIGCHG", CMDfunctionStr, 1, (char *) TriggerOnChange},// This command enabled the change detection module and triggers
                                                           // The table on detected change, TWIaddress in hex is passed to 
                                                           // init the change module.
  {"STBLDLT",CMDint, 1, (char *)&TimeDelta},               // Sets the TimeDelta values that is added to all flaged time point counts
  {"GTBLDLT",CMDint, 0, (char *)&TimeDelta},               // Returns the TimeDelta values that is added to all flaged time point counts
  {"GTBLSTA",CMDfunction, 0, (char *)&GetTableStatus},     // Returns the table status
  #endif
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
  {"STBLRBT", CMDfunctionStr, 1, (char *)SetTWenableTest},     // Set the readback test enable flag, TRUE or FALSE
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
// Twave and ARB frequency/voltage sweep commands
  {"STWSSTRT",CMDfunction, 2, (char *)SetStartFreqTWSW},       // Set the TWAVE sweep start frequency
  {"GTWSSTRT",CMDfunction, 1, (char *)GetStartFreqTWSW},       // Return the TWAVE sweep start frequency
  {"STWSSTP",CMDfunction, 2, (char *)SetStopFreqTWSW},         // Set the TWAVE sweep stop frequency
  {"GTWSSTP",CMDfunction, 1, (char *)GetStopFreqTWSW},         // Return the TWAVE sweep stop frequency
  {"STWSSTRTV",CMDfunctionStr, 2, (char *)SetStartVoltageTWSW},// Set the TWAVE sweep start voltage
  {"GTWSSTRTV",CMDfunction, 1, (char *)GetStartVoltageTWSW},   // Return the TWAVE sweep start voltage
  {"STWSSTPV",CMDfunctionStr, 2, (char *)SetStopVoltageTWSW},  // Set the TWAVE sweep stop voltage
  {"GTWSSTPV",CMDfunction, 1, (char *)GetStopVoltageTWSW},     // Return the TWAVE sweep stop voltage
  {"STWSTM",CMDfunctionStr, 2, (char *)SetSweepTimeTWSW},      // Set the TWAVE sweep time
  {"GTWSTM",CMDfunction, 1, (char *)GetSweepTimeTWSW},         // Return the TWAVE sweep time
  // There are two ARB sweep systems, the following commands start the sweep functions controlled
  // from the MIPS system. These work on both ARB and TWAVE system and support only channel 1 and 2.
  // The new ARB (version 1.14 and later) support ARB module sweeping and you can use all 4 ARB board.
  // The above setup commands work for both system. The following 3 commands are only the old system.
  // Look in the ARB secton for the new sweep start/stop/status commands.
  {"STWSGO",CMDfunction, 1, (char *)StartSweepTWSW},           // Start the sweep
  {"STWSHLT",CMDfunction, 1, (char *)StopSweepTWSW},           // Stop the sweep
  {"GTWSTA",CMDfunction, 1, (char *)GetStatusTWSW},            // Return the TWAVE sweep status
// Twave configuration commands  
  {"STWCCLK", CMDbool, 1, (char *)&TDarray[0].UseCommonClock},   // Flag to indicate common clock mode for two Twave modules.
  {"STWCMP", CMDbool, 1, (char *)&TDarray[0].CompressorEnabled}, // Flag to indicate Twave compressor mode is enabled.
#if FAIMScode
// FAIMS  General FAIMS commands
  {"SFMENA", CMDbool, 1, (char *)&faims.Enable},                // Set the FAIMS enable flag, TRUE enables waveform generation
  {"GFMENA", CMDbool, 0, (char *)&faims.Enable},                // Returns the FAIMS enable flag
  {"SFMDRV", CMDfunctionStr, 1, (char *)FAIMSsetDrive},         // Sets FAIMS drive level in percent
  {"GFMDRV", CMDfloat, 0, (char *)&faims.Drv},                  // Returns FAIMS drive level in percent
  {"GFMPWR", CMDfloat, 0, (char *)&TotalPower},                 // Returns FAIMS total power in watts
  {"GFMPV", CMDfloat, 0, (char *)&KVoutP},                      // Returns FAIMS positive peak output voltage
  {"GFMNV", CMDfloat, 0, (char *)&KVoutN},                      // Returns FAIMS negative peak output voltage
  {"SFMLOCK", CMDfunctionStr, 1, (char *)FAIMSsetLock},         // Sets FAIMS output level lock mode
  {"GFMLOCK", CMDbool, 0, (char *)&Lock},                       // Returns FAIMS output level lock mode
  {"SFMSP", CMDfunctionStr, 1, (char *)FAIMSsetLockSP},         // Sets FAIMS output level lock setpoint
  {"GFMSP", CMDfloat, 0, (char *)&LockSetpoint},                // Returns FAIMS output level lock setpoint
  {"SFMTUNE", CMDfunction, 0, (char *)FAIMSrequestAutoTune},    // Set auto tune request flag
  {"SFMTABRT", CMDfunction, 0, (char *)FAIMSautoTuneAbort},     // Set auto tune abort flag
  {"GFMTSTAT", CMDstr, 0, (char *)TuneState},                   // Returns the auto tune state string
  {"SFMTPOS",CMDbool, 1, (char *)&TunePos},                     // Set the positive tune mode if TRUE, if FALSE tune for neg peak
  {"GFMTPOS",CMDbool, 0, (char *)&TunePos},                     // Return positive peak mode
  {"SFMTDRV",CMDint, 1, (char *)&TuneDrive},                    // Set the tune mode drive level
  {"GFMTDRV",CMDint, 0, (char *)&TuneDrive},                    // Return the tune mode drive level
// FAIMS DC CV and Bias commands
  {"SFMCV", CMDfunctionStr, 1, (char *)FAIMSsetCV},                // Sets FAIMS DC CV voltage setpoint
  {"GFMCV", CMDfloat, 0, (char *)&faims.DCcv.VoltageSetpoint},     // Returns FAIMS DC CV voltage setpoint
  {"GFMCVA", CMDfloat, 0, (char *)&DCcvRB},                        // Returns FAIMS DC CV voltage actual
  {"SFMBIAS", CMDfunctionStr, 1, (char *)FAIMSsetBIAS},            // Sets FAIMS DC Bias voltage setpoint
  {"GFMBIAS", CMDfloat, 0, (char *)&faims.DCbias.VoltageSetpoint}, // Returns FAIMS DC Bias voltage setpoint
  {"GFMBIASA", CMDfloat, 0, (char *)&DCbiasRB},                    // Returns FAIMS DC Bias voltage actual
  {"SFMOFF", CMDfunctionStr, 1, (char *)FAIMSsetOffset},           // Sets FAIMS DC offset voltage setpoint
  {"GFMOFF", CMDfloat, 0, (char *)&faims.DCoffset.VoltageSetpoint},// Returns FAIMS DC offset voltage setpoint
  {"GFMOFFA", CMDfloat, 0, (char *)&DCoffsetRB},                   // Returns FAIMS DC offset voltage actual
// FAIMS CV Scan commands
  {"SFMCVSTART", CMDfunctionStr, 1, (char *)FAIMSsetCVstart},      // Sets FAIMS DC CV scan start voltage
  {"GFMCVSTART", CMDfloat, 0, (char *)&faims.CVstart},             // Returns FAIMS DC CV scan start voltage
  {"SFMCVEND", CMDfunctionStr, 1, (char *)FAIMSsetCVend},          // Sets FAIMS DC CV scan end voltage
  {"GFMCVEND", CMDfloat, 0, (char *)&faims.CVend},                 // Returns FAIMS DC CV scan end voltage
  {"SFMDUR", CMDfunctionStr, 1, (char *)FAIMSsetDuration},         // Sets FAIMS DC CV scan duration in seconds
  {"GFMDUR", CMDfloat, 0, (char *)&faims.Duration},                // Returns FAIMS DC CV scan duration in seconds
  {"SFMLOOPS", CMDfunctionStr, 1, (char *)FAIMSsetLoops},          // Sets FAIMS DC CV scan loops
  {"GFMLOOPS", CMDint, 0, (char *)&Loops},                         // Returns FAIMS DC CV scan loops
  {"SFMSTRTLIN", CMDbool, 1, (char *)&FAIMSscan},                  // Sets FAIMS DC CV linear scan flag
  {"GFMSTRTLIN", CMDbool, 0, (char *)&FAIMSscan},                  // Returns FAIMS DC CV linear scan flag
  {"SFMSTPTM", CMDfunctionStr, 1, (char *)FAIMSsetStepTime},       // Sets FAIMS DC CV scan step duration
  {"GFMSTPTM", CMDint, 0, (char *)&faims.StepDuration},            // Returns FAIMS DC CV scan step duration
  {"SFMSTEPS", CMDfunctionStr, 1, (char *)FAIMSsetSteps},          // Sets FAIMS DC CV step scan number of steps
  {"GFMSTEPS", CMDint, 0, (char *)&faims.Steps},                   // Returns FAIMS DC CV step scan number of steps
  {"SFMSTRTSTP", CMDbool, 1, (char *)&FAIMSstepScan},              // Sets FAIMS DC CV step scan flag
  {"GFMSTRTSTP", CMDbool, 0, (char *)&FAIMSstepScan},              // Returns FAIMS DC CV step scan flag
// FAIMS configuration / calibration commands
  {"SRFHPCAL", CMDfunctionStr, 2, (char *)FAIMSsetRFharPcal},      // Set FAIMS RF harmonic positive peak readback calibration
  {"SRFHNCAL", CMDfunctionStr, 2, (char *)FAIMSsetRFharNcal},      // Set FAIMS RF harmonic negative peak readback calibration
  {"SARCDIS",CMDbool, 1, (char *)&DiableArcDetect},                // TRUE or FALSE, set to TRUE disable arc detection
  {"FMISCUR",CMDbool, 0, (char *)&CurtianFound},                   // Returns TRUE is Curtian supply was detected
  {"SFMCCUR",CMDbool, 1, (char *)&CurtianCtrl},                    // If TRUE allows enable faims to enable curtian supply
  {"GFMCCUR",CMDbool, 0, (char *)&CurtianCtrl},                    // Returns curtian control flag
  {"SFMMDIS",CMDbool, 1, (char *)&ArcMessAutoDismiss},             // Set the message auto dismiss is TRUE
  {"GFMMDIS",CMDbool, 0, (char *)&ArcMessAutoDismiss},             // Return the message auto dismiss flag 
  {"SFARCR",CMDint, 1, (char *)&FMnumTries},                       // Set the number of arc retry attempts
  {"GFARCR",CMDint, 0, (char *)&FMnumTries},                       // Return the number of arc retry attempts 
#endif
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
  {"GFLDIR", CMDfunction, 1, (char *)GetCerrentDirection},           // Get filament current direcrtion,rev 2
  {"SFLDIR", CMDfunctionStr, 2, (char *)SetCurrentDirection},        // Set filament current direction, rev 2
  {"GFLP1", CMDfunction, 1, (char *)GetFilamentCycleCurrent1},       // Get filament cycle current 1 (setpoint)
  {"SFLP1", CMDfunctionStr, 2, (char *)SetFilamentCycleCurrent1},    // Set filament cycle current 1 (setpoint)
  {"GFLP2", CMDfunction, 1, (char *)GetFilamentCycleCurrent2},       // Get filament cycle current 2 (setpoint)
  {"SFLP2", CMDfunctionStr, 2, (char *)SetFilamentCycleCurrent2},    // Set filament cycle current 2 (setpoint)
  {"GFLCY", CMDfunction, 1, (char *)GetFilamentCycleCount},          // Get filament cycle count
  {"SFLCY", CMDfunctionStr, 2, (char *)SetFilamentCycleCount},       // Set filament cycle count
  {"GFLENAR", CMDfunction, 1, (char *)GetFilamentStatus},            // Get filament cycle status, OFF, or the number of cycles remaining
  {"SFLENAR", CMDfunctionStr, 2, (char *)SetFilamentStatus},         // Set filament cycle status, ON or OFF
  {"RFLPARMS", CMDfunction, 2, (char *)SetFilamentReporting},        // Sets a filament channel reporting rate, 0 = off. Rate is in seconds
  {"GFLSRES", CMDint, 0, (char *)&FDarray[0].iSense},                // Returns the bias current sense resistor value
  {"SFLSRES", CMDint, 1, (char *)&FDarray[0].iSense},                // Sets the bias current sense resistor value
  {"GFLECUR", CMDfunction, 0, (char *)ReportBiasCurrent},            // Returns the filament emission current 
  {"SFLSWD",  CMDbool, 1, (char *)&FLserialWD},                      // Set serial watch dog timer mode
  {"CALFIL", CMDfunction, 0, (char *)CalibrateFilament},             // Calibration procedure for rev 4 filament module 
// WiFi commands
  {"GHOST",  CMDstr, 0, (char *)wifidata.Host},                      // Report this MIPS box host name
  {"GSSID",  CMDstr, 0, (char *)wifidata.ssid},                      // Report the WiFi SSID to connect to
  {"GPSWD",  CMDstr, 0, (char *)wifidata.password},                  // Report the WiFi network password
  {"SHOST",  CMDfunctionStr, 1, (char *)SetHost},                    // Set this MIPS box host name
  {"SSSID",  CMDfunctionStr, 1, (char *)SetSSID},                    // Set the WiFi SSID to connect to
  {"SPSWD",  CMDfunctionStr, 1, (char *)SetPassword},                // Set the WiFi network password
  {"SWIFIENA",  CMDbool, 1, (char *)&MIPSconfigData.UseWiFi},        // Set the WiFi enable flag
  {"SWIFISP",  CMDint, 1, (char *)&wifidata.SerialPort},             // Set the WiFi serial port
// Ethernet commands
  {"ENTEST", CMDfunction, 0, (char *)Ethernet_test},                 // Tests the eternet adapter connection
  {"GEIP", CMDfunction, 0, (char *)ReportEIP},                       // Report the ethernet adapter IP address
  {"SEIP", CMDfunctionStr, 1, (char *)SetEIP},                       // Set the ethernet adapter IP address
  {"GESNIP", CMDfunction, 0, (char *)ReportSNEIP},                   // Report the ethernet adapter Subnet IP address
  {"SESNIP", CMDfunctionStr, 1, (char *)SetSNEIP},                   // Set the ethernet adapter Subnet IP address
  {"GEPORT", CMDfunction, 0, (char *)ReportEport},                   // Report the ethernet adapter port number
  {"SEPORT", CMDfunction, 1, (char *)SetEport},                      // Set the ethernet adapter port number
  {"GEGATE", CMDfunction, 0, (char *)ReportEGATE},                   // Report the ethernet adapter gateway IP address
  {"SEGATE", CMDfunctionStr, 1, (char *)SetEGATE},                   // Set the ethernet adapter gateway IP address
  {"SENTTWI", CMDbool, 1, (char *)&MIPSconfigData.EnetUseTWI},       // Sets the TWI interface flag for enet interface, TRUE=use TWI
  {"GENTTWI", CMDbool, 0, (char *)&MIPSconfigData.EnetUseTWI},       // Returns the enet TWI flag
// ARB general commands
  {"SARBMODE", CMDfunctionStr, 2, (char *)SetARBMode},               // Sets the ARB mode
  {"GARBMODE", CMDfunction, 1, (char *)GetARBMode},                  // Reports the ARB mode
  {"SWFREQ", CMDfunction, 2, (char *)SetWFfreq},                     // Sets waveform frequency, 0 to 40000Hz
  {"GWFREQ", CMDfunction, 1, (char *)GetWFfreq},                     // Returns the waveform frequency, 0 to 40000Hz
  {"SWFVRNG", CMDfunctionStr, 2, (char *)SetWFrange},                // Sets waveform voltage range, rev 2.0
  {"GWFVRNG", CMDfunction, 1, (char *)GetWFrange},
  {"SWFVRAMP", CMDfunctionStr, 2, (char *)SetWFramp},                // Sets waveform voltage chanel ramp rate in V/s
  {"GWFVRAMP", CMDfunction, 1, (char *)GetWFramp},
  {"SWFVOFF", CMDfunctionStr, 2, (char *)SetWFoffsetV},              // Sets waveform offset voltage, rev 2.0
  {"GWFVOFF", CMDfunction, 1, (char *)GetWFoffsetV},
  {"SWFVAUX", CMDfunctionStr, 2, (char *)SetWFaux},                  // Sets waveform aux voltage, rev 2.0
  {"GWFVAUX", CMDfunction, 1, (char *)GetWFaux},
  {"SWFDIS", CMDfunction, 1, (char *)SetWFdisable},                  // Stops waveform generation
  {"SWFENA", CMDfunction, 1, (char *)SetWFenable},                   // Starts waveform generation 
  {"SWFDIR", CMDfunctionStr, 2, (char *)SetARBdirection},            // Sets the waveform direction, FWD or REV
  {"GWFDIR", CMDfunction, 1, (char *)GetARBdirection},               // Returns the waveform direction, FWD or REV
  {"SWFARB", CMDfunctionLine, 0, (char *)(static_cast<void (*)(void)>(SetARBwaveform))},            // Sets an arbitrary waveform
  {"GWFARB", CMDfunction, 1, (char *)GetARBwaveform},                // Returns an arbitrary waveform
  {"SWFTYP", CMDfunctionStr, 2, (char *)SetARBwfType},               // Sets the arbitrary waveform type
  {"GWFTYP", CMDfunction, 1, (char *)GetARBwfType},                  // Returns the arbitrary waveform type
  {"SARBOFFA", CMDfunctionStr, 2, (char *)SetARBoffsetBoardA},       // For a dual output board ARB channel this commands sets the board A offset
  {"GARBOFFA", CMDfunction, 1, (char *)GetARBoffsetBoardA},          // For a dual output board ARB channel this commands returns the board A offset
  {"SARBOFFB", CMDfunctionStr, 2, (char *)SetARBoffsetBoardB},       // For a dual output board ARB channel this commands sets the board B offset
  {"GARBOFFB", CMDfunction, 1, (char *)GetARBoffsetBoardB},          // For a dual output board ARB channel this commands returns the board B offset  
  {"ARBSYNC", CMDfunction, 0, (char *)ARBmoduleSync},                // Issues a software sync, note, the modules have to be configured for external sync
                                                                     // for this function to work    
// ARB conventional ARB mode commands
  {"SARBBUF", CMDfunction, 2, (char *)SetARBbufferLength},           // Sets ARB buffer length
  {"GARBBUF", CMDfunction, 1, (char *)GetARBbufferLength},           // Reports ARB buffer length
  {"SARBNUM", CMDfunction, 2, (char *)SetARBbufferNum},              // Sets number of ARB buffer repeats per trigger 
  {"GARBNUM", CMDfunction, 1, (char *)GetARBbufferNum},              // Reports number of ARB buffer repeats per trigger
  {"SARBCHS", CMDfunctionStr, 2, (char *)SetARBchns},                // Sets all ARB channels in the full buffer to a defined value  
  {"SARBCH", CMDfunctionLine, 0, (char *)SetARBchannel},             // Sets a defined ARB channel in the full buffer to a defined value  
  {"SACHRNG", CMDfunctionLine, 0, (char *)SetARBchanRange},          // Sets an ARB channel to a value over a defined range
  {"SARBSINE", CMDfunctionLine, 0, (char *)SetARBsine},              // Sets a specific ARB channel to one sine wave cycle at select starting phase
                                                                     // parameters; module, channel, phase
// ARB compressor commands
  {"SARBCTBL", CMDlongStr, 100, (char *)TwaveCompressorTable},       // Twave compressor table definition setting command
  {"GARBCTBL", CMDstr, 0, (char *)TwaveCompressorTable},             // Twave compressor table definition reporting command
  {"GARBCMODE",CMDstr, 0, (char *)Cmode},                            // Report Twave compressor mode
  {"SARBCMODE",CMDfunctionStr, 1, (char *)SetARBCmode},              // Set Twave compressor mode
  {"GARBCORDER",CMDfunction, 0, (char *)GetARBCorder},               // Report Twave compressor order
  {"SARBCORDER",CMDfunction, 1, (char *)SetARBCorder},               // Set Twave compressor order
  {"GARBCTD",CMDfunction, 0, (char *)GetARBCtriggerDelay},           // Report Twave compressor trigger delay in mS
  {"SARBCTD",CMDfunctionStr, 1, (char *)SetARBCtriggerDelay},        // Set Twave compressor trigger delay in mS
  {"GARBCTC",CMDfunction, 0, (char *)GetARBCcompressTime},           // Report Twave compressor compress time in mS
  {"SARBCTC",CMDfunctionStr, 1, (char *)SetARBCcompressTime},        // Set Twave compressor compress time in mS
  {"GARBCTN",CMDfunction, 0, (char *)GetARBCnormalTime},             // Report Twave compressor normal time in mS
  {"SARBCTN",CMDfunctionStr, 1, (char *)SetARBCnormalTime},          // Set Twave compressor normal time in mS
  {"GARBCTNC",CMDfunction, 0, (char *)GetARBCnoncompressTime},       // Report Twave compressor non compress time in mS
  {"SARBCTNC",CMDfunctionStr, 1, (char *)SetARBCnoncompressTime},    // Set Twave compressor non compress time in mS
  {"TARBTRG",CMDfunction, 0, (char *)ARBCtrigger},                   // Force a Twave compressor trigger
  {"GARBCSW",CMDstr, 0, (char *)CswitchState},                       // Report Twave compressor Switch state
  {"SARBCSW",CMDfunctionStr, 1, (char *)SetARBCswitch},              // Set Twave compressor Switch state
// ARB alternate waveform commands  
  {"SALTTRG", CMDfunctionStr, 2, (char *)SetARBaltTrgInp},           // Defines the alternate waveform external trigger input on MIPS, Input Q thru W or NA to disable
  {"GALTTRG", CMDfunction, 1, (char *)GetARBaltTrgInp},              // Returns the alternate waveform external trigger input on MIPS, Input Q thru W or NA to disable

  {"SALTENA", CMDfunctionStr, 2, (char *)SetARBaltEna},              // Enables alternate waveform for selected module, TRUE or FALSE
  {"GALTENA", CMDfunction, 1, (char *)GetARBaltEna},                 // Returns alternate waveform enable status for selected module
  {"SALTHWD", CMDfunctionStr, 2, (char *)SetARBaltTrg},              // Enables alternate waveform harware triggering for selected module. TRUE or FALSE
  {"GALTHWD", CMDfunction, 1, (char *)GetARBaltTrg},                 // Returns alternate waveform harware triggering status for selected module
  {"SALTTMODE",CMDfunctionStr, 2, (char *)SetAltTrigMode},           // Sets the alternate trigger mode for selected module, LEVEL, POS, NEG
  {"GALTTMODE",CMDfunction, 1, (char *)GetAltTrigMode},              // Return the alternate trigger mode for the selected module

  {"SALTFVAL",CMDfun2int1str, 3, (char *)SetFixedValue},             // Sets the alternate waveform fixed value in percent for selected module, -100 to 100. first argument is index 0 to 7
  {"GALTFVAL",CMDfunction, 2, (char *)GetFixedValue},                // Returns the alternate waveform fixed value in percent for selected module, -100 to 100. first argument is index 0 to 7

  {"SALTWFM",CMDfunctionStr, 2, (char *)SetAltWaveFrm},              // Sets the alternate waveform type for the selected module, COMP (default),REV,ARB,FIX
  {"GALTWFM",CMDfunction, 1, (char *)GetAltWaveFrm},                 // Returns the alternate waveform type for the selected module, COMP (default),REV,ARB,FIX
 
  {"SALTDLY", CMDfunctionStr,2, (char *)SetARBaltTrgDly},            // Sets the alternate waveform trigger delay for pos or neg edge trigging only, in mS, for selected module
  {"GALTDLY", CMDfunction,1, (char *)GetARBaltTrgDly},               // Returns the alternate waveform trigger delay for pos or neg edge trigging only, in mS, for selected module
  {"SALTPLY", CMDfunctionStr,2, (char *)SetARBaltTrgDur},            // Sets the alternate waveform play or apply time for pos or neg edge trigging only, in mS, for selected module
  {"GALTPLY", CMDfunction,1, (char *)GetARBaltTrgDur},               // Returns the alternate waveform play or apply time for pos or neg edge trigging only, in mS, for selected module
  
  {"SALTRENA", CMDfunctionStr, 2, (char *)SetARBaltRngEna},          // Enables alternate waveform range for selected module. TRUE or FALSE
  {"GALTRENA", CMDfunction, 1, (char *)GetARBaltRngEna},             // Returns alternate waveform range enable flag for selected module
  {"SALTRNG",  CMDfunctionStr,2, (char *)SetARBaltRng},              // Sets the alternate waveform range in volts, for selected module
  {"GALTRNG",  CMDfunction,1, (char *)GetARBaltRng},                 // Returns the alternate waveform range in volts, for selected module


  {"CARBADLY",  CMDfunctionStr,2, (char *)ARBdelayOnChange},         // Enables level detector to change delay time in mS for the selected ARB modules
                                                                     // parameters, TWI address in hex of the level detector (hex), ARB mask that defines the
                                                                     // ARB modules to change (hex)
  {"CARBADUR",  CMDfunctionStr,2, (char *)ARBdurationOnChange},      // Enables level detector to change duration time in mS for the selected ARB modules
                                                                     // parameters, TWI address in hex of the level detector (hex), ARB mask that defines the
                                                                     // ARB modules to change (hex)
// ARB commands for application of reverse direction aux voltage change
  {"SARBREVA", CMDfunctionStr,2, (char *)SetARBrevAuxV},             // Sets the reverse direction ARB AUX output voltage, for selected module
  {"CLRARBRV", CMDfunction,21, (char *)ClearARBrevAuxV},             // Clears the reverse direction ARB AUX output voltage application, for selected module
// ARB sweep commands, ARB module based
  {"SARBSGO",CMDfunction, 1, (char *)ARBstartSweep},                 // Start the sweep
  {"SARBSHLT",CMDfunction, 1, (char *)ARBstopSweep},                 // Stop the sweep
  {"GARBSTA",CMDfunction, 1, (char *)GetARBsweepStatus},             // Return the TWAVE sweep status
// ARB configuration commands  
  {"SARBCCLK", CMDfunctionStr, 2, (char *)SetARBUseCommonClock},     // Flag to indicate common clock mode for the given ARB module.
  {"SARBCMP", CMDfunctionStr, 1, (char *)SetARBCompressorEnabled},   // Flag to indicate ARB compressor mode is enabled.
  {"SARBCOFF", CMDfunctionStr, 1, (char *)SetARBcommonOffset},       // Flag for all ARB channels use a common offset.
  {"SARBADD",CMDfunction, 2, (char *)SetARBtwiADD},                  // Set the TWI address (base 10) for the ARB module.
  {"GARBADD",CMDfunction, 1, (char *)GetARBtwiADD},                  // Return the TWI address (base 10) for the ARB module.
  {"SARBDBRD",CMDfunctionStr, 2, (char *)SetARBDualBoard},           // Sets module dual board flag to true or false.
  {"GARBVER",CMDfunction, 1, (char *)GetARBversion},                 // Returns the ARB module version number
  {"GARBPPP",CMDfunction, 1, (char *)GetARBppp},                     // Returns the ARB module points per period, 8 to 32
  {"SARBPPP",CMDfunction, 2, (char *)SetARBppp},                     // Sets the ARB module points per period, 8 to 32
  {"SARBEXT",CMDfunctionStr, 2, (char *)SetARBext},                  // Sets the selected ARB external clock source, MIPS or EXT
// ARB advanced setup commands
  {"SARBCPEX",CMDfunctionStr, 2, (char *)SetARBcompExt},             // Sets the ARB compress line hardware control, TRUE enables
  {"SARBHISR",CMDfunctionStr, 2, (char *)SetARBhwdISR},              // Sets the ARB compress line process to ISR if true
  {"SARBSYNLN",CMDfunction, 2, (char *)SetARBsyncLine},              // Sets the ARB sync line to use, 1 or 2, 1 = default
  {"SARBCMPLN",CMDfunction, 2, (char *)SetARBcompLine},              // Sets the ARB comp line to use, 1 or 2, 2 = default
// DAC module commands
  {"SDACV", CMDfunctionStr, 2, (char *)SetDACValue},                 // Sets the named DAC channel's value  
  {"GDACV", CMDfunctionStr, 1, (char *)GetDACValue},                 // Returns the named DAC channel's value  
  {"SDACMAX", CMDfunctionStr, 2, (char *)SetDACMax},                 // Sets the named DAC channel's maximum  
  {"GDACMAX", CMDfunctionStr, 1, (char *)GetDACMax},                 // Returns the named DAC channel's maximum  
  {"SDACMIN", CMDfunctionStr, 2, (char *)SetDACMin},                 // Sets the named DAC channel's minimum  
  {"GDACMIN", CMDfunctionStr, 1, (char *)GetDACMin},                 // Returns the named DAC channel's minimum    
  {"SDACUN", CMDfunctionStr, 2, (char *)SetDACUnits},                // Sets the named DAC channel's units  
  {"GDACUN", CMDfunctionStr, 1, (char *)GetDACUnits},                // Returns the named DAC channel's units  
  {"SDACNM", CMDfun2int1str, 3, (char *)SetDACName},                 // Sets the DAC channel name defined by module and channel number 
  {"GDACMN", CMDfunction, 2, (char *)GetDACName},                    // Returns the DAC channel name defined by module and channel number 
#if FAIMSFBcode
// FAIMSFB commands
  {"SFBENA",  CMDfunctionStr, 2, (char *)SetFAIMSfbEnable},          // Set module number to TRUE to enable FAIMS
  {"GFBENA",  CMDfunction, 1, (char *)ReturnFAIMSfbEnable},          // Returns enable status for selected module
  {"SFBMODE", CMDfunctionStr, 2, (char *)SetFAIMSfbMode},            // Set module number to TRUE to enable closed loop control of Vrf
  {"GFBMODE", CMDfunction, 1, (char *)ReturnFAIMSfbMode},            // Returns mode control for selected module
  {"SFBFREQ", CMDfunction, 2, (char *)SetFAIMSfbFreq},               // Set FAIMS frequency, in Hz for seletced module
  {"GFBFREQ", CMDfunction, 1, (char *)ReturnFAIMSfbFreq},            // Returns frequency for the selected module
  {"SFBDUTY", CMDfunction, 2, (char *)SetFAIMSfbDuty},               // Set FAIMS duty cycle in percent for seletced module
  {"GFBDUTY", CMDfunction, 1, (char *)ReturnFAIMSfbDuty},            // Returns duty cycle for selected module
  {"SFBDRV", CMDfunctionStr,2, (char *)SetFAIMSfbDrive},             // Set FAIMS drive level in percent for the selected module
  {"GFBDRV", CMDfunction,1, (char *)ReturnFAIMSfbDrive},             // Returns drive level for the selected module
  {"GDRVV", CMDfunction,  1, (char *)ReturnFAIMSfbDriveV},           // Returns voltage level into drive FET, V for the selected module
  {"GDRVI", CMDfunction,  1, (char *)ReturnFAIMSfbDriveI},           // Returns current into drive FET, mA for the selected module
  {"SVRF", CMDfunctionStr, 2, (char *)SetFAIMSfbVrf},                // Sets the drive level setpoint to achieve the desired voltage for the selected module
  {"GVRF", CMDfunction,  1, (char *)ReturnFAIMSfbVrf},               // Returns the Vrf peak voltage for the selected module
  {"GVRFV", CMDfunction,  1, (char *)ReturnFAIMSfbVrfV},             // Returns the Vrf peak voltage readback for the selected module
  {"GPWR", CMDfunction,  1, (char *)ReturnFAIMSfbPWR},               // Returns the power in watts for the selected module
  {"SVRFN", CMDfunctionStr, 2, (char *)SetFAIMSfbVrfNow},            // Sets the drive level setpoint achieve the desired voltage for the selected module
                                                                     // and adjusts the drive level to reach the setpoint
  {"SVRFT", CMDfunctionStr, 2, (char *)SetFAIMSfbVrfTable},          // Sets the drive level setpoint achieve the desired voltage for the selected module
                                                                     // and adjusts the drive level to reach the setpoint using the drive level table
  {"GENVRFTBL", CMDfunction, 1, (char *)GenerateVrfTable},           // This function will generate 
  // FAIMSFB Limits
  {"SFBMAXDRV",CMDfunctionStr,2, (char *)SetFAIMSfbMaxDrive},        // Set the maximum drive level allowed for the selected module
  {"GFBMAXDRV",CMDfunction, 1,(char *)ReturnFAIMSfbMaxDrive},        // Returns the maximum drive level for the selected module
  {"SFBMAXPWR",CMDfunctionStr,2, (char *)SetFAIMSfbMaxPower},        // Set the maximum power allowed for the selected module
  {"GFBMAXPWR",CMDfunction,1,(char *)ReturnFAIMSfbMaxPower},         // Returns the maximum power limit for the selected module
  // FAIMSFB DC bias commands
  {"SCV", CMDfunctionStr,  2, (char *)SetFAIMSfbCV},                 // Set the current CV for the seletced module
  {"GCV", CMDfunction,  1, (char *)ReturnFAIMSfbCV},                 // Return the current CV for the seletced module
  {"GCVV", CMDfunction, 1, (char *)ReturnFAIMSfbCVrb},               // Return the CV readback value for the seletced module
  {"SBIAS", CMDfunctionStr,  2, (char *)SetFAIMSfbBIAS},             // Set the current BIAS for the seletced module
  {"GBIAS", CMDfunction,  1, (char *)ReturnFAIMSfbBIAS},             // Return the current BIAS for the seletced module
  {"GBIASV", CMDfunction,  1, (char *)ReturnFAIMSfbBIASrb},          // Set the current BIAS for the seletced module 
  // FAIMSFB scanning commands
  {"SFBCVSTRT",  CMDfunctionStr, 2, (char *)SetFAIMSfbCVstart},      // set the CV scan start voltage for the seletced module 
  {"GFBCVSTRT",  CMDfunction, 1, (char *)ReturnFAIMSfbCVstart},      // returns the CV scan start voltage for the seletced module 
  {"SFBCVEND",   CMDfunctionStr, 2, (char *)SetFAIMSfbCVend},        // set the CV scan end voltage for the seletced module 
  {"GFBCVEND",   CMDfunction, 1, (char *)ReturnFAIMSfbCVend},        // returns the CV scan end voltage for the seletced module 
  {"SFBVRFSTRT", CMDfunctionStr, 2, (char *)SetFAIMSfbVRFstart},     // set the Vrf scan start voltage for the seletced module 
  {"GFBVRFSTRT", CMDfunction, 1, (char *)ReturnFAIMSfbVRFstart},     // returns the Vrf scan start voltage for the seletced module 
  {"SFBVRFEND",  CMDfunctionStr, 2, (char *)SetFAIMSfbVRFend},       // set the Vrf scan end voltage for the seletced module 
  {"GFBVRFEND",  CMDfunction, 1, (char *)ReturnFAIMSfbVRFend},       // returns the Vrf scan end voltage for the seletced module 
  {"SFBSTEPDUR", CMDfunction, 2, (char *)SetFAIMSfbStepDuration},    // set the scan step duration in mS for the seletced module 
  {"GFBSTEPDUR", CMDfunction, 1, (char *)ReturnFAIMSfbStepDuration}, // returns the scan step duration in mS for the seletced module 
  {"SFBNUMSTP",  CMDfunction, 2, (char *)SetFAIMSfbSteps},           // set the scan number of steps for the seletced module 
  {"GFBNUMSTP",  CMDfunction, 1, (char *)ReturnFAIMSfbSteps},        // returns the scan number of steps for the seletced module 
  {"FBSCNSTRT",  CMDfunction, 1, (char *)InitFAIMSfbScan},           // Start the scan for the seletced module 
  {"FBSCNSTP",  CMDfunction, 1, (char *)StopFAIMSfbScan},            // Stop a scn that is in process for the seletced module 
  {"SFBADCSMP",  CMDint, 1, (char *)&NumSamples},                    // Set the number of adc sample to average, 1 to 16 
  {"GFBADCSMP",  CMDint, 0, (char *)&NumSamples},                    // Returns the number of adc sample to average, 1 to 16 
  // Electrometer commands
  {"SELTMTRENA",  CMDfunctionStr, 1, (char *)SetEMRTenable},         // Set the electrometer enabled flag, TRUE or FALSE 
  {"GELTMTRENA",  CMDfunction, 0, (char *)ReturnEMRTenable},         // Return the electrometer enable flag
  {"GELTMTRPOS",  CMDfunction, 0, (char *)ReturnEMRTpos},            // Return the electrometer positive channel
  {"GELTMTRNEG",  CMDfunction, 0, (char *)ReturnEMRTneg},            // Return the electrometer negative channel
  {"SELTMTRPOSOFF",  CMDfunctionStr, 1, (char *)SetEMRTposOff},      // Set the electrometer positive channel offset
  {"GELTMTRPOSOFF",  CMDfunction, 0, (char *)ReturnEMRTposOff},      // Returns the electrometer positive channel offset
  {"SELTMTRNEGOFF",  CMDfunctionStr, 1, (char *)SetEMRTnegOff},      // Set the electrometer negative channel offset
  {"GELTMTRNEGOFF",  CMDfunction, 0, (char *)ReturnEMRTnegOff},      // Returns the electrometer negative channel offset
  {"SELTMTRPOSZERO",  CMDfunctionStr, 1, (char *)SetEMRTposZero},    // Set the electrometer positive channel zero
  {"GELTMTRPOSZERO",  CMDfunction, 0, (char *)ReturnEMRTposZero},    // Returns the electrometer positive channel zero
  {"SELTMTRNEGZERO",  CMDfunctionStr, 1, (char *)SetEMRTnegZero},    // Set the electrometer negative channel zero
  {"GELTMTRNEGZERO",  CMDfunction, 0, (char *)ReturnEMRTnegZero},    // Returns the electrometer negative channel zero
  {"ELTMTRZERO",  CMDfunction, 0, (char *)SetEMRTzero},              // Execute the electrometer zero procedure
#endif
// End of table marker
  {0},
};

void Debug(int function)
{
   serial->println(TableStatus);
}

// Real time clock functions
void GetTime(void)
{
  char buf[3];

  SendACKonly;
  if(SerialMute) return; 
  sprintf(buf,"%.2d:", rtc.getHours());
  serial->print(buf);
  sprintf(buf,"%.2d:", rtc.getMinutes());
  serial->print(buf);
  sprintf(buf,"%.2d", rtc.getSeconds());
  serial->println(buf);
}

void SetTime(void)
{
  char     *tkn;
  String   arg;
  uint8_t  h,m,s;

  while(true)
  {
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     h = arg.toInt();
     if(h >= 24) break;
     if((tkn = TokenFromCommandLine(':')) == NULL) break;
     arg = tkn;
     m = arg.toInt();
     if(m >= 60) break;
     if((tkn = TokenFromCommandLine(':')) == NULL) break;
     arg = tkn;
     s = arg.toInt();
     if(s >= 60) break;
     rtc.setTime(h, m, s);
     SendACK;
     return;
  }
  BADARG;
}

void GetDate(void)
{
  char buf[5];

  SendACKonly;
  if(SerialMute) return; 
  sprintf(buf,"%.2d/", rtc.getDay());
  serial->print(buf);
  sprintf(buf,"%.2d/", rtc.getMonth());
  serial->print(buf);
  sprintf(buf,"%.4d", rtc.getYear());
  serial->println(buf);  
}

void SetDate(void)
{
  char     *tkn;
  String   arg;
  uint8_t  d,m;
  uint16_t y;

  while(true)
  {
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     d = arg.toInt();
     if(d > 31) break;
     if((tkn = TokenFromCommandLine('/')) == NULL) break;
     arg = tkn;
     m = arg.toInt();
     if((m==0) || (m>12)) break;
     if((tkn = TokenFromCommandLine('/')) == NULL) break;
     arg = tkn;
     y = arg.toInt();
     if(y<1970) break;
     rtc.setDate(d, m, y);
     SendACK;
     return;
  }
  BADARG;
}

// end real time clock functions

// This function will set a command to the TWI primary port. This function then looks
// for a response for up to 250mS. After a \n is received this function will exit. The
// the received characters and sent to the selected host port.
// On call it is assumed the full command line is in the input buffer.
void TWIscmd(void)
{
  
  char        *Token;
  char        ch;
  uint8_t     addr;
  uint8_t     brd;
  String      sToken;

  while(true)
  {
    // Get address and send TWIcmd to the device on Wire1
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    brd = sToken.toInt();
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    addr = sToken.toInt();
    GetToken(true);
    // Send the command to TWI device
    SelectBoard(brd);
    AcquireTWI();
    Wire.beginTransmission(addr);
    Wire.write(TWI_CMD);
    // Send string and limit length to 32 bytes
    for(int i=0;i<30;i++)
    {
      if(i==29) 
      {
        Wire.write('\n');
        break;
      }
      ch = RB_Get(&RB);
      if(ch == 0xFF) break;
      Wire.write(ch);
      if(ch == '\n') break;
    }
    Wire.endTransmission();
    // Read reply and send to host
    unsigned long tm = millis();
    while(true)
    {
      int j = Wire.requestFrom((int)addr, 32);
      for(int i=0;i<j;i++)
      {
        ch = Wire.read();
        if(ch != 0xFF) serial->write(ch);
        if(ch == '\n') 
        {
          ReleaseTWI();
          return;
        }
      }
      if((tm+250) < millis()) break;
    }
  }
  ReleaseTWI();
}

// This function will set a command to the TWI port 1, the secondary port. This function then looks
// for a response for up to 250mS. After a \n is received this function will exit. The
// the received characters and sent to the selected host port.
// On call it is assumed the full command line is in the input buffer.
void TWI1scmd(void)
{
  char        *Token;
  char        ch;
  uint8_t     addr;
  String      sToken;
  static bool inited = false;

  if(!inited)
  {
    Wire1.begin();
    Wire1.setClock(100000);
    inited = true;
  }
  while(true)
  {
    // Get address and send TWIcmd to the device on Wire1
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    addr = sToken.toInt();
    GetToken(true);
    // Send the command to TWI device
    Wire1.beginTransmission(addr);
    Wire1.write(TWI_CMD);
    // Send string and limit length to 32 bytes
    for(int i=0;i<30;i++)
    {
      if(i==29) 
      {
        Wire1.write('\n');
        break;
      }
      ch = RB_Get(&RB);
      if(ch == 0xFF) break;
      Wire1.write(ch);
      if(ch == '\n') break;
    }
    Wire1.endTransmission();
    // Read reply and send to host
    unsigned long tm = millis();
    while(true)
    {
      int j = Wire1.requestFrom((int)addr, 32);
      for(int i=0;i<j;i++)
      {
        ch = Wire1.read();
        if(ch != 0xFF) serial->write(ch);
        if(ch == '\n') return;
      }
      if((tm+250) < millis()) break;
    }
  }
}

// This function will read a DUE pin
void Dread(int pin)
{
  SendACKonly;
  if(digitalRead(pin) == HIGH) serial->println("HIGH");
  else serial->println("LOW");
}

void Dwrite(char *pin, char *state)
{
  String  sToken;
  int     p;

  sToken = pin;
  p = sToken.toInt();
  sToken = state;
  if(sToken == "HIGH") digitalWrite(p,HIGH);
  else if(sToken == "LOW") digitalWrite(p,LOW);
  else if(sToken == "PULSE")
  {
    if(digitalRead(p) == LOW) {digitalWrite(p,HIGH); digitalWrite(p,LOW);}
    else {digitalWrite(p,LOW); digitalWrite(p,HIGH);}
  }
  else if(sToken == "SPULSE")
  {
    if(digitalRead(p) == LOW) {digitalWrite(p,HIGH); delay(1); digitalWrite(p,LOW);}
    else {digitalWrite(p,LOW);  delay(1); digitalWrite(p,HIGH);}    
  }
  else BADARG;
  SendACK;
}

void Dset(char *pin, char *mode)
{
  String  sToken;
  int     p;

  sToken = pin;
  p = sToken.toInt();
  sToken = mode;
  if(sToken == "INPUT") pinMode(p, INPUT);
  else if(sToken == "PULLUP") pinMode(p, INPUT_PULLUP);
  else if(sToken == "OUTPUT") pinMode(p, OUTPUT);
  else BADARG;
  SendACK;
}

// This function does control the Arduino USB power pin but given the way the hardware is
// desiged USB power will always power the processor reguardless of this setting.
void USBpower(char *bval)
{
  String  sToken;

  sToken = bval;

  pinMode(85,OUTPUT); 
  if(sToken == "TRUE") 
  {
    //UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_VBUSPO;
    UOTGHS->UOTGHS_CTRL &= ~0x2100;
    UOTGHS->UOTGHS_CTRL |= 0x2000;
    digitalWrite(85,HIGH);
  }
  else if(sToken == "FALSE") 
  {
    //UOTGHS->UOTGHS_CTRL &= ~UOTGHS_CTRL_VBUSPO;
    UOTGHS->UOTGHS_CTRL &= ~0x2100;
    UOTGHS->UOTGHS_CTRL |= 0x0000;
    digitalWrite(85,LOW);
  }
  else BADARG;
  SendACK;
}

void UpTime(void)
{
  SendACKonly;
  serial->print("System has been up for at least: ");
  float uptime = (float)millis() / 60000;
  serial->print(uptime);
  serial->println(" minutes");
}

uint32_t MemoryAddress;

void SetMemAddress(char *address)
{
  sscanf(address,"%x",&MemoryAddress);
  serial->println(MemoryAddress,16);    
}

void WriteMemory(char *type, char *val)
{
  String sType;

  sType = type;
  if(sType == "BYTE") sscanf(val,"%x",(uint8_t *)MemoryAddress);
  else if(sType == "WORD") sscanf(val,"%x",(uint16_t *)MemoryAddress);
  else if(sType == "DWORD") sscanf(val,"%x",(uint32_t *)MemoryAddress);
  else if(sType == "INT") sscanf(val,"%d",(int *)MemoryAddress);
  else if(sType == "FLOAT") sscanf(val,"%f",(float *)MemoryAddress);
}

void ReadMemory(char *type)
{
  String sType;

  sType = type;
  if(sType == "BYTE") serial->println(*(uint8_t *)MemoryAddress,16);    
  else if(sType == "WORD") serial->println(*(uint16_t *)MemoryAddress,16);    
  else if(sType == "DWORD") serial->println(*(uint32_t *)MemoryAddress,16);    
  else if(sType == "INT") serial->println(*(int *)MemoryAddress,16);    
  else if(sType == "FLOAT") serial->println(*(float *)MemoryAddress,16);    
}

void twitalk(TwoWire *wire, int brd, int TWIadd)
{
  wire->begin();
  wire->setClock(100000);
  serial->println("Redirecting the serial host commands through the");
  delay(100);
  serial->println("TWI port for the selected  device. Press escape");
  delay(100);
  serial->println("to exit this mode. If you don't know what your");
  delay(100);
  serial->println("doing, do not use this function!");
  delay(100);
  // Select the board and send the TWIadd the communications enable command
  SelectBoard(brd);
  AcquireTWI();
  wire->beginTransmission(TWIadd);
  wire->write(TWI_ARB_SERIAL);
  wire->endTransmission();
  delay(100);
  // Echo all communications through the TWI port
  while(1)
  {
    WDT_Restart(WDT);
    wire->requestFrom(TWIadd, 32);
    for(int j=0;j<32;j++)
    {
      int i = wire->read();
      if(i == -1) break;
      if(i == 27)
      {
        // Exit per slave command
         ReleaseTWI();
         serial->println(" TWI redirection terminated by slave!");
         return;
      }
      if((i != 120) && (i != 255)) serial->write((char) i);   
    }
    while (serial->available() > 0)
    {
      char c = serial->read();
      wire->beginTransmission(TWIadd);
      wire->write(c);
      wire->endTransmission();
      if(c == 27)
      {
         ReleaseTWI();
         serial->println("Exiting TWI redirection.");
         return;
      }
    }
  }
  ReleaseTWI();
  serial->println("Exiting TWI redirection.");
}

void TWItalk(int brd, int TWIadd)
{
  twitalk(&Wire, brd, TWIadd);
}

void TWI1talk(int brd, int TWIadd)
{
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  twitalk(&Wire1, brd, TWIadd);
}

void CheckImage(char *filename)
{
  SendACKonly;
  bmpReport(filename);
}

void LoadImage(char *filename)
{
  bool DD;
  
  DD = DisableDisplay;
  DisableDisplay = false;
  tft.disableDisplay(DisableDisplay);
  if(bmpDraw(filename, 0, 0))
  {
    SendACK;
  }
  else
  {
    SetErrorCode(ERR_BMPERROR);
    SendNAK;
  }
  DisableDisplay = DD;
  tft.disableDisplay(DisableDisplay);
}

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

// This function saves the selected modules data to EEPROM.
void SaveModule(char *Module)
{
  if (strcmp(Module, "RF") == 0) SaveRF2EEPROM();
  else if (strcmp(Module, "DCB") == 0) SaveDCB2EEPROM();
  else if (strcmp(Module, "ESI") == 0) SaveESI2EEPROM();
  #if FAIMScode
  else if (strcmp(Module, "FAIMS") == 0) SaveFAIMS2EEPROM();
  #endif
  else if (strcmp(Module, "TWAVE") == 0) SaveTWAVE2EEPROM();
  else if (strcmp(Module, "ARB") == 0) SaveARB2EEPROM();
  else if (strcmp(Module, "FIL") == 0) SaveFIL2EEPROM();
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }  
}

// Generic get number of channels function, calls the proper routine based on parameter entered.
// RF = Number of RF channels
// DCB = number of DC bias channels
// ESI = number of HV ESI supplies
// FAIMS = Number of FAIMS drivers
// TWAVE = Number of TWAVE drivers
// FIL = Number of filiment channels
// ARB = Number of ARB channels
// DIO = Number of digital IO channels
// DI = Number of digital input channels
// DO = Number of digital output channels
void GetNumChans(char *cmd)
{
  if (strcmp(cmd, "RF") == 0) RFnumber();
  else if (strcmp(cmd, "DCB") == 0) DCbiasNumber();
  else if (strcmp(cmd, "ESI") == 0) ESInumberOfChannels();
  else if (strcmp(cmd, "TWAVE") == 0) TWAVEnumberOfChannels();
  #if FAIMScode
  else if (strcmp(cmd, "FAIMS") == 0) FAIMSnumberOfChannels();
  #endif
  else if (strcmp(cmd, "FIL") == 0) FilamentChannels();
  else if (strcmp(cmd, "ARB") == 0) ReportARBchannels();
  else if (strcmp(cmd, "DAC") == 0) ReportDACchannels();
  else if (strcmp(cmd, "DIO") == 0)
  {
    SendACKonly;
    if(!SerialMute) serial->println(24);
  }
  else if (strcmp(cmd, "DI") == 0)
  {
    SendACKonly;
    if(!SerialMute) serial->println(8);
  }
  else if (strcmp(cmd, "DO") == 0)
  {
    SendACKonly;
    if(!SerialMute) serial->println(16);
  }
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
}

__attribute__ ((section (".ramfunc")))
void _EEFC_ReadUniqueID( unsigned int * pdwUniqueID )
{
    unsigned int status ;
  
    /* Send the Start Read unique Identifier command (STUI) by writing the Flash Command Register with the STUI command.*/
    EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_STUI;
    do
    {
        status = EFC1->EEFC_FSR ;
    } while ( (status & EEFC_FSR_FRDY) == EEFC_FSR_FRDY ) ;

    /* The Unique Identifier is located in the first 128 bits of the Flash memory mapping. So, at the address 0x400000-0x400003. */
    pdwUniqueID[0] = *(uint32_t *)IFLASH1_ADDR;
    pdwUniqueID[1] = *(uint32_t *)(IFLASH1_ADDR + 4);
    pdwUniqueID[2] = *(uint32_t *)(IFLASH1_ADDR + 8);
    pdwUniqueID[3] = *(uint32_t *)(IFLASH1_ADDR + 12);

    /* To stop the Unique Identifier mode, the user needs to send the Stop Read unique Identifier
       command (SPUI) by writing the Flash Command Register with the SPUI command. */
    EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_SPUI ;

    /* When the Stop read Unique Unique Identifier command (SPUI) has been performed, the
       FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises. */
    do
    {
        status = EFC1->EEFC_FSR ;
    } while ( (status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY ) ;
}

void ReportUniqueID(void)
{
   unsigned int adwUniqueID[4]; 

   {
      AtomicBlock< Atomic_RestoreState > a_Block;
      _EEFC_ReadUniqueID( adwUniqueID );
   }
   SendACKonly;
   serial->print("ID: ");
   for (byte b = 0 ; b < 4 ; b++)
   {
      serial->print ((unsigned int) adwUniqueID[b], HEX);
   }
   serial->println ("");
}

void SerialInit(void)
{
  #ifdef EnableSerial
  if((!MIPSconfigData.UseWiFi) || (wifidata.SerialPort!=0)) Serial.begin(SerialBAUD);
  #endif
  //Serial_ *serial = &SerialUSB;
  SerialUSB.begin(0);
//  SerialUSB.printSetEOL("kk");
//  mipsstream = &SerialUSB;
  //  serial->println("Initializing....");
  RB_Init(&RB);
}

// This function builds a token string from the characters passed.
void Char2Token(char ch)
{
  if(ch == 0x08)
  {
     if(Tptr > 0) Tptr--;
     return;
  }
  Token[Tptr++] = ch;
//  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
  Token[MaxToken - 1] = 0;  // Make sure its null terminated
}

// This function reads the serial input ring buffer and returns a pointer to a ascii token.
// Tokens are comma delimited. Commands strings end with a semicolon or a \n.
// The returned token pointer points to a standard C null terminated string.
// This function does not block. If there is nothing in the input buffer null is returned.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  // Exit if the input buffer is empty
  while(1)
  {
    ch = RB_Next(&RB);
    if (ch == 0xFF) return NULL;
    if (Tptr >= MaxToken) Tptr = MaxToken - 1;
    if ((ch == '\n') || (ch == ';') || (ch == ':') || (ch == ',') || (ch == ']') || (ch == '[') || (ch == '/'))
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
  }
}

// Returns the last token returned for GetToken call
char *LastToken(void)
{
  return Token;
}

char *TokenFromCommandLine(char expectedDel)
{
  char *tkn;
  
  tkn = GetToken(true);
  if(tkn == NULL) return NULL;
  if(tkn[0] != expectedDel)
  {
    // Flush to end of line and return NULL
    return NULL;
  }
  return GetToken(true);
}

char  *UserInput(char *message)
{
  char *tkn;
  
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  serial->print(message);
  // Wait for a line to be detected in the ring buffer
  while(RB.Commands == 0) ReadAllSerial();
  // Read the token
  tkn = GetToken(true);
  if(tkn != NULL) if(tkn[0] == '\n') tkn = NULL;
  return tkn;
}

int   UserInputInt(char *message)
{
  char   *tkn;
  String arg;

  tkn = UserInput(message);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toInt();
}

float UserInputFloat(char *message)
{
  char   *tkn;
  String arg;

  tkn = UserInput(message);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toFloat();
}

void ExecuteCommand(const Commands *cmd, int arg1, int arg2, char *args1, char *args2, float farg1)
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
          strcpy((char *)cmd->pointers.charPtr,args1);
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
    case CMDbyte:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
         SendACKonly;
         if (!SerialMute) serial->println(*(cmd->pointers.bytePtr));
         break;
      }
      if (cmd->NumArgs == 1) 
      {
          *(cmd->pointers.bytePtr) = arg1;
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
      if(cmd->pointers.funcVoid == NULL) break;
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      else if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      else if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;
    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      else if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      else if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;
    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;
    case CMDfun2int1str:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1str(arg1, arg2, args1);
      break;
    default:
      SendNAK;
      break;
  }
}

// This function processes serial commands.
// This function does not block and returns -1 if there was nothing to do.
int ProcessCommand(void)
{
  char          *Token,ch;
  int           i;
  static int    arg1, arg2;
  static float  farg1;
  static enum   PCstates state;
  static int    CmdNum;
  static char   delimiter=0;
  static String EchoString = "";
  // The following variables are used for the long string reading mode
  static char   *lstrptr = NULL;
  static int    lstrindex;
  static bool   lstrmode = false;
  static int    lstrmax;

  // Wait for line in ringbuffer
  if(state == PCargLine)
  {
    if(RB.Commands <= 0) return -1;
    CmdArray[CmdNum].pointers.funcVoid();
    state = PCcmd;
    return(0);
  }
  if(lstrmode)
  {
    ch = RB_Get(&RB);
    if(ch == 0xFF) return(-1);
    if(ch == ',')  return(0);
    if(ch == '\r') return(0);
    if(ch == '\n')
    {
      lstrptr[lstrindex++] = 0;
    if(lstrindex >= lstrmax) lstrindex = lstrmax - 1;
      lstrmode = false;
      SendACK;
      return(-1);                 // Changed from 0 on 12/6/18, signals nothing to do
    }
    lstrptr[lstrindex++] = ch;
    if(lstrindex >= lstrmax) lstrindex = lstrmax - 1;
    return(0);
  }
  Token = GetToken(false);
  if (Token == NULL) return (-1);
  if (Token[0] == 0) return (-1);
  if((echoMode) && (!SerialMute))
  {
    if (strcmp(Token, "\n") != 0) 
    {
      if(delimiter!=0) EchoString += delimiter;
      EchoString += Token;
    }
    if (strcmp(Token, "\n") == 0)
    {
      delimiter=0;
      serial->print(EchoString);
      serial->flush();  // Added 11/16/2020, caused system to crash on UUID command without it!
      EchoString = "";
    }
    else EchoString += ',';
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
      // If the type CMDfunctionLine then we will wait for a full line in the ring buffer
      // before we call the function. Function has not args and must pull tokens from ring buffer.
      if (CmdArray[i].Type == CMDfunctionLine)
      {
        state = PCargLine;
        break;
      }
      // If this is a long string read command type then init the vaiable to support saving the
      // string directly to the provided pointer and exit. This function must not block
      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr = (char *)CmdArray[i].pointers.charPtr;
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
      sscanf(Token, "%s", Sarg1);
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

int GetLine(Ring_Buffer *rb,char *cbuf,int maxlen)
{
  int i;
  char c;
  
  cbuf[0] = 0;
  if(RB_Commands(rb) <= 0) return(0);
  while(true)
  {
    c = RB_Get(rb);
    if(c==0xFF) break;
    if(c == ';') break;
    if(c == '\n') break;
    cbuf[i++] = c;
    if(i >= maxlen) i--;
  }
  cbuf[i] = 0;
  return(i);
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
  if((int)ch == 255) return;  // Never put a null in the buffer!
  RB_Put(&RB, ch);
}

//
// The following function support the macro capibility.
//

// This function will open a file for recording a macro. This will result in all received chararcters from
// the host computer being saved in the macro file. If the file is already present it will be appended to.
void MacroRecord(char *filename)
{
  char   fname[30];
  char   ch;
  char   *TK;
  String cmd;

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
  // Put all chars that in in the input ring buffer and write to the file
  while((ch = RB_Get(&RB)) != 0xFF) 
  {
    if(ch == '\n')
    {
      if(cmd == "MSTOP")
      {
        MacroFile.write(ch);
        MacroFile.close();
        SendACK;
        return;  
      }
      cmd = "";
    }
    else cmd += ch;
    MacroFile.write(ch);
    WDT_Restart(WDT);
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
      while (RB_Size(&RB) != 0)
      {
        if ((TK = GetToken(false)) != NULL) break;
        WDT_Restart(WDT);
      }
      if (TK != NULL)
      {
        if (strcmp(TK, "MSTOP") == 0)
        {
          Recording = false;
          break;
        }
      }
      WDT_Restart(WDT);
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
