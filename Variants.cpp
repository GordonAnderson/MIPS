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

uint32_t TWIfails = 0;
bool     Serial1Echo = false;

MIPSconfigStruct MIPSconfigData = {sizeof(MIPSconfigStruct),"MIPS",2,0,0,10,false,true,1.0,"",false,false,"",5,false,0xA55AE99E,false,false,false,false,0,0,0,0,true};

TwaveData Twave_Rev1 = {sizeof(TwaveData),"Twave",1,1000000,03,0x10,0x20,0x27,0x69,
                        20.0, 0, 76.53, 905, 0, 1, 0,
                         0.0, 1, 76.53, 0, 1, 1, 0,
                        10.0, 2, 76.53, 905, 2, 1, 0,
                        10.0, 3, 76.53, 874, 3, 1, 0,
                        true,0,-1,0,-1,
                        false,
                        false,2,10,1,1.0,1.0,1.0,1.0,0,-1,0,-1,
                        0,-1,false,
                        };

TwaveData Twave_Rev2 = {sizeof(TwaveData),"Twave",2,100000,03,0x10,0x20,0x27,0x69,
                        20.0, 0, 520, 0, 0, 520, 0,
                         0.0, 1, 520, 0, 1, 520, 0,
                        10.0, 2, 520, 0, 2, 520, 0,
                        10.0, 3, 520, 0, 3, 520, 0,
                        true,0,-1,0,-1,
                        false,
                        false,2,10,1,1.0,1.0,1.0,1.0,0,-1,0,-1,
                        0,-1,false,
                        0,1,0.0
                        };
                        
TwaveData Twave_Rev3 = {sizeof(TwaveData),"Twave",3,100000,03,0x1F,0x24,0x27,0x58,
                        20.0, 0, 520, 0, 0, 520, 0,
                         0.0, 1, 520, 0, 1, 520, 0,
                        10.0, 2, 520, 0, 2, 520, 0,
                        10.0, 3, 520, 0, 3, 520, 0,
                        true,0,-1,0,-1,
                        false,
                        false,2,10,1,1.0,1.0,1.0,1.0,0,-1,0,-1,
                        0,-1,false,
                        0,1,0.0
                        };

TwaveData Twave_Rev5 = {sizeof(TwaveData),"Twave",5,100000,03,0x1F,0x24,0x27,0x58,
                        20.0, 0, 100, 0, 0, 100, 0,
                         0.0, 1, 100, 0, 1, 100, 0,
                        10.0, 2, 100, 0, 2, 100, 0,
                        10.0, 3, 100, 0, 3, 100, 0,
                        true,0,-1,0,-1,
                        false,
                        false,2,10,1,1.0,1.0,1.0,1.0,0,-1,0,-1,
                        0,-1,false,
                        0,1,0.0
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

// Rev 2 uses the new linear tech level sensors and thus needs a different engineering unit conversion.
RFdriverData  RFDD_A_Rev_2 = {sizeof(RFdriverData),"RFdriver", 2, 2, 0x20, 0x69, 0x50,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 5, 4, 32, 320, 5, 32, 320, 0, 2383.09, 0, 1, 2621.4, 0,
                              1000000, 0, 0, RF_MANUAL, 50.0, 20.0, 7, 6, 32, 320, 7, 32, 320, 2, 2383.09, 0, 3, 2621.4, 0,
                              0,0,
                              -1,-1,
                             };
                             
RFdriverData  RFDD_B_Rev_2 = {sizeof(RFdriverData),"RFdriver", 2, 2, 0x20, 0x69, 0x50,
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
                              0, 0, -113.7, 32767, 1, -113.7, 32767,      // Offset control, DAC only unless using AD5593
                              false,false,
                             };

DCbiasData  DCbD_250_Rev_2 = {sizeof(DCbiasData),"DCbias", 2, 8, 250, -250, true, 0, 0x20, 0x1A, 0x54,
                              0, 0, 113.7, 32767, 0, 113.7, 32767,        // Ch 1
                              0, 1, 113.7, 32767, 1, 113.7, 32767,        // Ch 2
                              0, 2, 113.7, 32767, 2, 113.7, 32767,        // Ch 3
                              0, 3, 113.7, 32767, 3, 113.7, 32767,        // Ch 4
                              0, 4, 113.7, 32767, 4, 113.7, 32767,        // Ch 5
                              0, 5, 113.7, 32767, 5, 113.7, 32767,        // Ch 6
                              0, 6, 113.7, 32767, 6, 113.7, 32767,        // Ch 7
                              0, 7, 113.7, 32767, 7, 113.7, 32767,        // Ch 8
                              0, 0, -113.7, 32767, 1, -113.7, 32767,      // Offset control, DAC only unless using AD5593
                              false,false,
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
                              0, 0, 26, 32767, 1, 26, 32767,         // Offset control, DAC only unless using AD5593
                              false,false,
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
                             0, 0, -609.48, 32767, 1, -609.48, 32767,      // Offset control, DAC only unless using AD5593
                             false,false,
                            };

DCbiasData  DCbD_60_Rev_1 = {sizeof(DCbiasData),"DCbias", 1, 8, 60, -60, true, 2, 0x23, 0x18, 0x52,
                             0, 0, 540, 32767, 0, 540, 32767,        // Ch 1
                             0, 1, 540, 32767, 1, 540, 32767,        // Ch 2
                             0, 2, 540, 32767, 2, 540, 32767,        // Ch 3
                             0, 3, 540, 32767, 3, 540, 32767,        // Ch 4
                             0, 4, 540, 32767, 4, 540, 32767,        // Ch 5
                             0, 5, 540, 32767, 5, 540, 32767,        // Ch 6
                             0, 6, 540, 32767, 6, 540, 32767,        // Ch 7
                             0, 7, 540, 32767, 7, 540, 32767,        // Ch 8
                             0, 0, -540, 32767, 1, -540, 32767,      // Offset control, DAC only unless using AD5593
                             false,false,
                            };
// Supports the single board filament controller for e-msion. remaps the DAC channels for board routing,
// also uses the AD5593 ADC/DAC                            
DCbiasData  DCbD_60_Rev_3 = {sizeof(DCbiasData),"DCbias", 3, 8, 60, -60, true, 2, 0x23, 0x11, 0x52,
                             0, 1, 540, 32767, 0, 540, 32767,        // Ch 1
                             0, 3, 540, 32767, 1, 540, 32767,        // Ch 2
                             0, 5, 540, 32767, 2, 540, 32767,        // Ch 3
                             0, 7, 540, 32767, 3, 540, 32767,        // Ch 4
                             0, 0, 540, 32767, 4, 540, 32767,        // Ch 5
                             0, 2, 540, 32767, 5, 540, 32767,        // Ch 6
                             0, 4, 540, 32767, 6, 540, 32767,        // Ch 7
                             0, 6, 540, 32767, 7, 540, 32767,        // Ch 8
                             0, 0, -540, 32767, 1, -540, 32767,      // Offset control, DAC only unless using AD5593
                             false,false,
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
                          50.0,
                          25,
                          200,
                          'R',RISING
                          };
                         
ESIdata  ESI_Rev_1 =   { sizeof(ESIdata),"ESI", 1,
                         false,0.0,6000,6000,0.1,  // ESI channel 0
                             0,10.9225,0,            //   DAC voltage control
                             1,10.9225,0,            //   ADC voltage readback
                             0,13106,0,              //   ADC current readback, mA
                         false,0.0,-6000,-6000,0.1, // ESI channel 1
                             1,-10.9225,0,           //   DAC voltage control
                             3,-10.9225,0,           //   ADC voltage readback
                             2,13106,0,              //   ADC current readback, mA
                         0x24,0x1F,0x52,
                       };

FilamentData FILAMENT_Rev_1 = {sizeof(FilamentData),"Filament",1,2,
                               0,5.0,false,0.1,FmodeI,10,14, 0,-3868,21206, 1,5392,13960, 0,13156,-137, 1,13050,-96, 2,2722,6421,
                               0,5.0,false,0.1,FmodeI,10,48, 2,-3868,21206, 3,5392,13960, 3,13156,-137, 4,13050,-96, 5,2722,6421,
                               0x24,0x12,0x52,
                               false,1.0,4.0,20,
                               false,1.0,4.0,20,
                               0,true,
                               7,2500,32767,
                              };
                              
// Supports the single board filament controller for e-msion. only one filament channel and no reversal
FilamentData FILAMENT_Rev_4 = {sizeof(FilamentData),"Filament",4,1,
                               0,5.0,false,0.1,FmodeI,10, 0, 1,-3868,21206, 4,5392,13960, 2,13156,-137, 3,13050,-96, 5,2722,6421,
                               0,5.0,false,0.1,FmodeI,10, 0, 1,-3868,21206, 4,5392,13960, 2,13156,-137, 3,13050,-96, 5,2722,6421,
                               0x10,0x10,0x52,
                               false,1.0,4.0,20,
                               false,1.0,4.0,20,
                               0,true,
                               6,2500,32767,
                              };

ARBdata  ARB_Rev_1 = {sizeof(ARBdata),"ARB", 1, false, 1, false, 100, 0, true,20000,25,0,32,ARB_SIN,
                      1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
                      0x40,0x50,
                      "TWAVE",25.0,'R',RISING,1000,1,0,-1,
                      false,false,2,1,10.0,10.0,10.0,10.0,
                      0,-1,0,-1,
                      false,
                      false,0,0,
                      0,1,
                      0,
                      20000,20000,25,25,1,
                      0,-1
                     };

HOFAIMSdata  HOFAIMS_Rev_1 = {sizeof(HOFAIMSdata),"HOFAIMS", 1, false, 0, 25.0, 0, 20, 50,
                              0,2621,0,                                         // Vmon
                              1,2621,0,                                         // Imod
                              2,6656,10000,                                     // RFmon
                              0x50,0x24,
                              2                                                // SPI address
                              };

DACdata DAC_Rev1 = {sizeof(DACdata),"DAC",1,8,3,4,0x54,
                    0,-10,10,"CH1","V",0,0,3276, 32767,
                    0,-10,10,"CH2","V",0,1,3276, 32767,
                    0,-10,10,"CH3","V",0,2,3276, 32767,
                    0,-10,10,"CH4","V",0,3,3276, 32767,
                    0,-10,10,"CH5","V",0,4,3276, 32767,
                    0,-10,10,"CH6","V",0,5,3276, 32767,
                    0,-10,10,"CH7","V",0,6,3276, 32767,
                    0,0,10,"CH8","V",0,7,6553, 0
                   };

RFAdata RFA_Rev1 = {sizeof(RFAdata),"RFamp",1,false,1000000,0,0,true,false,3,4,0x56,0x1F,0x24,
                    0,650,0,
                    1,3.47,0,
                    2,650,0,
                    3,650,0,
                    0,2126,0,
                    1,12957,0,
                    2,650,0,
                    3,1043,0,
                    4,48372,0,
                    5,650,0,
                    6,5.26,-156.71,
                    7,3.6614,-147.14,
                    4000,4.5,600,
                    0.0,0.0,0.0,
                    200,
                    6,5.26,-156.71,
                    7,3.6614,-147.14,                    
                    1,3.47,0,
                    5.958257713248639
                   };
                   
#ifdef TestMode
WiFiData  WiFi_Rev_1 = {sizeof(WiFiData),"WiFi",1,WS_AP,"MIPSnet","MIPS","MIPS1234","",0,true,1};
#else
WiFiData  WiFi_Rev_1 = {sizeof(WiFiData),"WiFi",1,WS_IDLE,"MIPSnet","MIPS","MIPS1234","",0,false,1};
#endif


// List of all posible board addresses. These addresses are those of the EEPROM on the modules
char *BoardAddressList = "A 0x50,A 0x52,A 0x54,A 0x56,B 0x50,B 0x52,B 0x54,B 0x56";
// List of board names used to allow user to select a board by name for inital setup or re-init
char *BoardVariantsNames = "RFdrvA R1,RFdrvB R1,RFdrvA R2,RFdrvB R2,DC250V R1,DC250V R2,DC750V R1,DC50V  R1,DC60V  R1,DC60V  R3,Twave R1,Twave R2,Twave R3,Twave R5,FAIMS R1,ESI  R1,FIL R1,FIL R4,ARB R1,HOFAIMS,DAC R1,RFamp R1";
// List of variant board default data structure pointers with a one to one corespondence to list of board names
void *BoardVariants[] = {(void *)&RFDD_A_Rev_1,(void *)&RFDD_B_Rev_1,(void *)&RFDD_A_Rev_2,(void *)&RFDD_B_Rev_2,(void *)&DCbD_250_Rev_1,(void *)&DCbD_250_Rev_2,(void *)&DCbD_750_Rev_1,(void *)&DCbD_50_Rev_1,(void *)&DCbD_60_Rev_1,(void *)&DCbD_60_Rev_3,(void *)&Twave_Rev1,(void *)&Twave_Rev2,(void *)&Twave_Rev3,(void *)&Twave_Rev5,(void *)&FAIMS_Rev_1,(void *)&ESI_Rev_1,(void *)&FILAMENT_Rev_1,(void *)&FILAMENT_Rev_4,(void *)&ARB_Rev_1,(void *)&HOFAIMS_Rev_1,(void *)&DAC_Rev1,(void *)&RFA_Rev1};
