//
// Variants.cpp
//
// This file defines all the hardware variants and rev levels.
//
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include "DIO.h"
#include "Hardware.h"
#include "Serial.h"
#include "Errors.h"
#include "Menu.h"
#include "Dialog.h"
#include "Variants.h"

MIPSconfigStruct MIPSconfigData = {sizeof(MIPSconfigStruct),"MIPS",1,0,0,10,false,true,1.0,"",false};

TwaveData Twave_Rev1 = {sizeof(TwaveData),"Twave",1,1000000,03,0x10,0x20,0x27,0x69,
                        20.0, 0, 76.53, 905, 0, 1, 0,
                         0.0, 1, 76.53, 0, 1, 1, 0,
                        10.0, 2, 76.53, 905, 2, 1, 0,
                        10.0, 3, 76.53, 874, 3, 1, 0,
                        };

TwaveData Twave_Rev2 = {sizeof(TwaveData),"Twave",2,100000,03,0x10,0x20,0x27,0x69,
                        20.0, 0, 520, 0, 0, 520, 0,
                         0.0, 1, 520, 0, 1, 520, 0,
                        10.0, 2, 520, 0, 2, 520, 0,
                        10.0, 3, 520, 0, 3, 520, 0,
                        true,
                        };

RFdriverData  RFDD_A_Rev_1 = {sizeof(RFdriverData),"RFdriver", 1, 2, 0x20, 0x69, 0x50,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 5, 4, 32, 320, 5, 32, 320, 0, 2383.09, 0, 1, 2621.4, 0,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 7, 6, 32, 320, 7, 32, 320, 2, 2383.09, 0, 3, 2621.4, 0,
                              0,0,
                              -1,-1,
                             };
                             
RFdriverData  RFDD_B_Rev_1 = {sizeof(RFdriverData),"RFdriver", 1, 2, 0x20, 0x69, 0x50,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 6, 4, 32, 320, 5, 32, 320, 0, 2383.09, 0, 1, 2621.4, 0,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 8, 6, 32, 320, 7, 32, 320, 2, 2383.09, 0, 3, 2621.4, 0,
                              0,0,
                              -1,-1,
                             };

DCbiasData  DCbD_250_Rev_1 = {sizeof(DCbiasData),"DCbias", 1, 8, 250, -250, true, 2, 0x23, 0x18, 0x52,
                              0, 0, 113.7, 32767, 0, 113.7, 32767,        // Ch 1
                              0, 1, 113.7, 32767, 1, 113.7, 32767,        // Ch 2
                              0, 2, 113.7, 32767, 2, 113.7, 32767,        // Ch 3
                              0, 3, 113.7, 32767, 3, 113.7, 32767,        // Ch 4
                              0, 4, 113.7, 32767, 4, 113.7, 32767,        // Ch 5
                              0, 5, 113.7, 32767, 5, 113.7, 32767,        // Ch 6
                              0, 6, 113.7, 32767, 6, 113.7, 32767,        // Ch 7
                              0, 7, 113.7, 32767, 7, 113.7, 32767,        // Ch 8
                              0, 0, -113.7, 32767, 0, 0, 0,               // Offset control, DAC only
                             };

DCbiasData  DCbD_750_Rev_1 = {sizeof(DCbiasData),"DCbias", 1, 8, 750, -750, true, 2, 0x23, 0x18, 0x52,
                              0, 0, -26, 32767, 0, 26, 32767,        // Ch 1
                              0, 1, -26, 32767, 1, 26, 32767,        // Ch 2
                              0, 2, -26, 32767, 2, 26, 32767,        // Ch 3
                              0, 3, -26, 32767, 3, 26, 32767,        // Ch 4
                              0, 4, -26, 32767, 4, 26, 32767,        // Ch 5
                              0, 5, -26, 32767, 5, 26, 32767,        // Ch 6
                              0, 6, -26, 32767, 6, 26, 32767,        // Ch 7
                              0, 7, -26, 32767, 7, 26, 32767,        // Ch 8
                              0, 0, 26, 32767, 0, 0, 0,               // Offset control, DAC only
                             };
          
DCbiasData  DCbD_50_Rev_1 = {sizeof(DCbiasData),"DCbias", 1, 8, 50, -50, true, 2, 0x23, 0x18, 0x52,
                             0, 0, 609.48, 32767, 0, 609.48, 32767,        // Ch 1
                             0, 1, 609.48, 32767, 1, 609.48, 32767,        // Ch 2
                             0, 2, 609.48, 32767, 2, 609.48, 32767,        // Ch 3
                             0, 3, 609.48, 32767, 3, 609.48, 32767,        // Ch 4
                             0, 4, 609.48, 32767, 4, 609.48, 32767,        // Ch 5
                             0, 5, 609.48, 32767, 5, 609.48, 32767,        // Ch 6
                             0, 6, 609.48, 32767, 6, 609.48, 32767,        // Ch 7
                             0, 7, 609.48, 32767, 7, 609.48, 32767,        // Ch 8
                             0, 0, -609.48, 32767, 0, 0, 0,               // Offset control, DAC only
                            };
                            
FAIMSdata  FAIMS_Rev_1 = {sizeof(FAIMSdata),"FAIMS", 1, 1000000, 25.0, false, 4, 180, 75.2, 21.5, 100.0, 60.0,0.5,
                          0, 2, -113.7, 32767, 5, 113.7, 32767,            // DC offset
                          0, 0, 113.7, 32767, 3, 113.7, 32767,             // DC bias
                          0, 1, 113.7, 32767, 4, 113.7, 32767,             // DC CV
                          0,200,100,
                          5,30.0,true,50.0,0,2383.09,0,1,2621.4,0,         // Drive 1
                          7,30.0,true,50.0,2,2383.09,0,3,2621.4,0,         // Drive 2
                          6,25.0,true,50.0,4,2383.09,0,5,2621.4,0,         // Drive 3
                          0,100,0,                                         // RFpri
                          2,6720,13088,                                    // RFharP
                          1,16640,12530,                                   // RFharN
                          0x50,0x20,0x24,0x18,0x69,0x27,0x23,0x40,
                          false,0.001,0.01,10,
                          1,
                          };
                         
ESIdata  ESI_Rev_1 =   { sizeof(ESIdata),"ESI", 1,
                         0.0,6000,0.0,      // ESI channel 0
                             0,10.9225,0,   //   DAC voltage control
                             1,10.9225,0,   //   ADC voltage readback
                             0,13106,0,   //   ADC current readback, mA
                         0.0,-6000,0.0,     // ESI channel 1
                             1,-10.9225,0,  //   DAC voltage control
                             3,-10.9225,0,  //   ADC voltage readback
                             2,13106,0,   //   ADC current readback, mA
                         0x24,0x1F,0x52,
                       };

FilamentData FILAMENT_Rev_1 = {sizeof(FilamentData),"Filament",1,2,
                               0,5.0,false,0.1,FmodeI,10,14, 0,-3868,21206, 1,5392,13960, 0,13156,-137, 1,13050,-96, 2,2722,6421,
                               0,5.0,false,0.1,FmodeI,10,48, 2,-3868,21206, 3,5392,13960, 3,13156,-137, 4,13050,-96, 5,2722,6421,
                               0x24,0x12,0x52
                              };

ARBdata  ARB_Rev_1 = {sizeof(ARBdata),"ARB", 1, false, 1, false, 100, 0, true,20000,25,0,32,ARB_SIN,
                      1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
                      0x32,0x50,
                     };

WiFiData  WiFi_Rev_1 = {sizeof(WiFiData),"WiFi",1,WS_IDLE,"MIPSnet","Linksys20476","","",0};


// List of all posible board addresses. These addresses are those of the EEPROM on the modules
char *BoardAddressList = "A 0x50,A 0x52,A 0x54,A 0x56,B 0x50,B 0x52,B 0x54,B 0x56";
// List of board names used to allow user to select a board by name for inital setup or re-init
char *BoardVariantsNames = "RFdrvA R1,RFdrvB R1,DC250V R1,DC750V R1,DC50V  R1,Twave R1,Twave R2,FAIMS R1,ESI  R1,FIL R1,ARB R1";
// List of variant board default data structure pointers with a one to one corespondence to list of board names
void *BoardVariants[] = {(void *)&RFDD_A_Rev_1,(void *)&RFDD_B_Rev_1,(void *)&DCbD_250_Rev_1,(void *)&DCbD_750_Rev_1,(void *)&DCbD_50_Rev_1,(void *)&Twave_Rev1,(void *)&Twave_Rev2,(void *)&FAIMS_Rev_1,(void *)&ESI_Rev_1,(void *)&FILAMENT_Rev_1,(void *)&ARB_Rev_1};


