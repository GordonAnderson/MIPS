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

//#define TestMode
//#define TestFilament
//#define TestTwave

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

// List of all posible board addresses. These addresses are those of the EEPROM on the modules
extern char *BoardAddressList;

// List of board names used to allow user to select a board by name for inital setup of re-init
extern char *BoardVariantsNames;

// List of variant board default data structure pointers with a one to one corespondence to list of board names
extern void *BoardVariants[];

#endif



