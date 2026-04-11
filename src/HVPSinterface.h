#pragma once
#include "Variants.h"
#if HVPSinterface

#define   DeviceID_HVPSI  0x81

// TWI commands and constants
#define TWI_HVPSI_SERIAL          0x27      // This command enables the TWI port to process serial commands
#define TWI_HVPSI_CMD             0x7F      // This command sends a ascii string to the serial command processor
                                            // and returns the response through the TWI interface
#define TWI_HVPSI_READ_AVALIBLE   0x82      // Returns the number of bytes available in output buffer, 16 bit unsigned int
#define TWI_HVPSI_READ_ID         0xF0      // Returns the device ID byte
// Application specific commands
#define TWI_HVPSI_SET_Power       0x01      // Sets the power, bool, true = on
#define TWI_HVPSI_SET_Voltage     0x03      // Sets the Voltage, float
#define TWI_HVPSI_SET_Pol         0x04      // Sets the polarity, bool, true = pos
#define TWI_HVPSI_SET_Range       0x05      // Sets the Voltage range, float
#define TWI_HVPSI_SET_MaxV        0x06      // Sets the maximum Voltage limit, float
#define TWI_HVPSI_SET_MaxI        0x07      // Sets the maximum Current limit, float, mA
#define TWI_HVPSI_SET_Save        0x08      // Save parameters
#define TWI_HVPSI_SET_Restore     0x09      // Restore parameters
#define TWI_HVPSI_SET_Reversable  0x0C      // Sets the reversable flag, bool
#define TWI_HVPSI_SET_CurEnable   0x0D      // Sets the current monitor enable flag, bool

#define TWI_HVPSI_GET_Power       0x81      // Returns the power status, bool, true = on
#define TWI_HVPSI_GET_Voltage     0x83      // Returns the Voltage, float
#define TWI_HVPSI_GET_Pol         0x84      // Returns the polarity status, bool, true = pos
#define TWI_HVPSI_GET_Range       0x85      // Returns the Voltage range, float
#define TWI_HVPSI_GET_MaxV        0x86      // Returns the maximum Voltage limit, float
#define TWI_HVPSI_GET_MaxI        0x87      // Returns the maximum Current limit, float, mA
#define TWI_HVPSI_GET_Reversable  0x8C      // Returns the reversable flag, bool
#define TWI_HVPSI_GET_CurEnable   0x8D      // Returns the current monitor enable flag, bool

// Readbacks
#define TWI_HVPSI_GET_Vrb         0x88      // Returns the Voltage readback, float
#define TWI_HVPSI_GET_Irb         0x89      // Returns the Current readback, float
#define TWI_HVPSI_GET_Prb         0x8A      // Returns the Polarity readback, bool, true = positive

typedef struct
{
  // General parameters
  bool          power;                  // True = on
  bool          polarity;               // True = positive
  float         setpoint;               // Desired voltage
  float         maxVoltage;             // Programable voltage limit
  float         maxCurrent;             // Programable current limit
  // Setup commands
  float         range;
  bool          reversable;
  bool          curMonEnabled;
} HVPSinterfaceData;

void HVPSinterface_init(void);

#endif