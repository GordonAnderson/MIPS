//
// Hardware variants for the MIPS controller.
//
// This file contains all the posible board data structues and there default values. This allows a user
// to setup a board to initial conditions.
//
//
#ifndef Variants_h
#define Variants_h
#include "DCbias.h"
#include "RFdriver.h"
#include "Twave.h"
#include "FAIMS.h"
#include "ESI.h"
#include "Filament.h"
#include "ARB.h"
#include "WiFi.h"
#include "HOFAIMS.h"

//#define TestMode
//#define TestFilament
//#define TestTwave

#define EnableSerial
#define SerialBAUD    9600

// Processor timer assignment to various MIPS modules.
// If timer 6 is used then you can not have a RF driver at
// board address of 0, or jumper position A. Timer 6 output
// is the same pin used for RF drive channel 1 power control.
#define    TMR_TrigOut  3       // Used to generate freq source on trigger output line
#define    TMR_TwaveClk 7       // Used for the Twave clock in compressor mode
//#define    TMR_TwaveClk 6     // Used for the Twave clock in compressor mode
#define    TMR_TwaveCmp 4       // Used for the Twave multi pass compressor table
#define    TMR_Table    8       // Used for the pulse sequence tables, must be 8 because of hardware needs
#define    TMR_Profiles Timer5  // Used for the profile toggeling function
#define    TMR_servos   4       // Used by the HOFAIMS module to drive the servos
#define    TMR_ARBclock 6       // Used for common clock generation for ARB
#define    TMR_DelayedTrigger 1 // Used by the delayed trigger capability

// Table mode software clock input pin, use S (DI2), default
#define    SoftClockDIO DI2

// MIPS system level confirguration data structure
typedef struct
{
  int16_t  Size;
  char     Name[20];          // Holds the MIPS box name
  int      Rev;               // Holds the hardware rev level
  uint8_t  DOlsb;             // Digitial output image registers, LSB
  uint8_t  DOmsb;             // Digitial output image registers, MSB
  int      StartupDelay;      // Startup delay in seconds
  bool     StartupHold;       // If this flag is true then MIPS will hold for button press before startup
  bool     PowerEnable;       // Enables the power supply for the DCbias board if true 
  float    VerrorThreshold;   // Trip threshold for the DCbias supply output monitor
  char     StartupMacro[20];  // This macro is played when the system powers up
  bool     UseAnalog;         // If true then the analog module is tested for on power up
  bool     UseWiFi;           // If true then the WiFi module is tested for on power up
  char     BootImage[20];     // Defines a image (bmp) to load at boot up, if found and loaded then the display
                              // is diabled.
} MIPSconfigStruct;

extern MIPSconfigStruct MIPSconfigData;

extern TwaveData Twave_Rev1;

extern TwaveData Twave_Rev2;

extern TwaveData Twave_Rev3;

extern RFdriverData  RFDD_A_Rev_1;
                             
extern RFdriverData  RFDD_B_Rev_1;

extern DCbiasData  DCbD_250_Rev_1;
                            
extern DCbiasData  DCbD_50_Rev_1;

extern FAIMSdata  FAIMS_Rev_1;

extern ESIdata  ESI_Rev_1;

extern FilamentData FILAMENT_Rev_1;

extern ARBdata  ARB_Rev_1;

extern WiFiData  WiFi_Rev_1;

extern HOFAIMSdata  HOFAIMS_Rev_1;

// List of all posible board addresses. These addresses are those of the EEPROM on the modules
extern char *BoardAddressList;

// List of board names used to allow user to select a board by name for inital setup of re-init
extern char *BoardVariantsNames;

// List of variant board default data structure pointers with a one to one corespondence to list of board names
extern void *BoardVariants[];

#endif





