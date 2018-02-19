//
//  Errors.h
//  AMPS
//
//  Created by Gordon Anderson on 9/7/14.
//
// This fine defines all the error codes that can be set by the AMPS system.
// These error codes are defined when a serial command is received and
// can not be processed for some reason. Olny the last error is saved in the
// variable ErrorCode. The controlling computer and issue the GERR command to
// Read the error code. The error code is never cleared, that last error is
// always avalible.
//
#ifndef MIPS_Errors_h
#define MIPS_Errors_h

extern int ErrorCode;

#define SetErrorCode(x) ErrorCode=x

// List of error codes and there descriptions

// This group of error codes are sharded between MIPS and AMPS. MIPS
// does not use all of these codes.
#define ERR_BADCMD                  1       // invalid command
#define ERR_BADARG                  2       // invalid argument
#define ERR_LOCALREADY              3       // System already in LOC mode
#define ERR_TBLALREADY              4       // System already in TBL mode
#define ERR_NOTBLLOADED             5       // No tables have been loaded
#define ERR_NOTBLMODE               6       // System is not in table mode
#define ERR_TBLNOTREADY             7       // Table not ready
#define ERR_TOKENTIMEOUT            8       // Timed out waiting for token
#define ERR_EXPECTEDCOLON           9       // Expected to see a :
#define ERR_TBLTOOBIG               10      // Table too big!
#define ERR_CHLOWORBRD              11      // Channel number too low or board not present
#define ERR_CHHIORBRD               12      // Channel number too high or board not present
#define ERR_CHHIGH                  13      // Requested channel number too high
#define ERR_BRDLOWORBRD             14      // Board number too low or board not present
#define ERR_BRDHIORBRD              15      // Board number too high or board not present
#define ERR_BRDHIGH                 16      // Board number too high
#define ERR_BRDNOTSUPPORT           17      // Command not support on this board rev
#define ERR_BADBAUD                 18      // Invalid baud rate requsted
#define ERR_EXPECTEDCOMMA           19      // Expected comma
#define ERR_NESTINGTOODEEP          20      // Table nesting too deep
#define ERR_MISSINGOPENBRACKET      21      // Table ] without coresponding [
#define ERR_INVALIDCHAN             22      // Invalid channel request
#define ERR_DIOHARDWARENOTPRESENT   23      // DIO hardware not found
#define ERR_TEMPRANGE               24      // Temp range error
#define ERR_ESIHVOUTOFRANGE         25      // ESI HV out of range
#define ERR_TEMPCONTROLLOOPGAIN     26      // Temp control loop gain range error
#define ERR_NOTLOCMODE              27      // System is not in local mode
#define ERR_WRONGTRGMODE            28      // Wrong trigger mode
#define ERR_CANTFINDENTRY           29      // Can't locate the requested table entry

// These error codes are specific to MIPS
#define ERR_VALUERANGE              101     // Requested value is out of range
#define ERR_NOSDCARD                102     // No SD card present in system
#define ERR_CANTCREATEFILE          103     // Can't create the file on SD card
#define ERR_FILENAMETOOLONG         104     // Filename defined is too long
#define ERR_CANTOPENFILE            105     // Can't open file
#define ERR_CANTDELETEFILE          106     // Can't delete file
#define ERR_NOTSUPPORTINREV         107     // Not supported in this MIPS controller revision
#define ERR_WIFICONNECTED           108     // Not supported while WiFi is connected
#define ERR_NOETHERNET              109     // No ethernet adapter detected
#define ERR_ETHERNETCOMM            110     // Ethernet adapter communication error
#define ERR_EEPROMWRITE             110     // Error writing to EEPROM on module
#define ERR_NOTOFFSETABLE           111     // DC bias module is not offsetable
#define ERR_INTERNAL                112     // Internal error such as can't allocate resource
#define ERR_BMPERROR                113     // BMP file error
#define ERR_TUNEINPROCESS           114     // Auto tune is already in process
#define ERR_NOARB                   115     // No ARB module in system
#define ERR_CANTALLOCATE            116     // Can't allocate needed memory
#define ERR_PROFILENOTDEFINED       117     // DC bias profile not defined
#define ERR_NAMEINLIST              118     // Name already in linked list
#define ERR_NAMENOTFOUND            119     // Name already in linked list
#define ERR_EEPROMREAD              120     // Error Reading from EEPROM on module
#define ERR_READINGSD               121     // Error Reading data from SD card
#define ERR_NOTSUPPORTED            122     // Not supported by hardware
#define ERR_NOTINMANMODE            123     // Auto tune error, not in manual mode
#define ERR_ADCNOTAVALIABLE         124     // ADC interface in use and not avaliable at this time
#define ERR_ADCALREARYSETUP         125     // ADC interface is already setup
#define ERR_ADCNOTSETUP             126     // ADC interface is not setup
#endif






