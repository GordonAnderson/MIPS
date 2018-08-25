//
// RFamp
//
//  This file provides the drivers used for the RF linear amplifier module designed to drive the 40 watt RF amplifier.
//  This module has 2 DC DAC outputs to be used to Quad DC bias with the addition of amplifiers.
//
//  Some facts reguarding Q voltages
//  - DC is a fixed ratio of RF voltage, 11.916 times less that Vp-p for max res
//  - The voltage delta per AMU is fixed over the m/z range
//  - The DC voltage delta per AMU is fixed over the m/z range
//  - RF Vo-p = 0.000000072226*Freq*Freq*MZ*(Ro/1000)^2
//  - RF Vp-p = 2 * Vo-p
//  - DC V = =0.000000012122*Freq*Freq*MZ*(Ro/1000)^2
//  - DC pos on one set of poles, neg on the other
//
// Features:
//  - RF reference source
//  - Level readback and display
//  - Closed loop control
//      - 2 modes, open loop and closed loop
//  - RF amplifier monitoring
//      - DC voltage in
//      - DC current in
//      - Temp
//      - RF current
//      - RF voltage
//      - VSWR
//  - This module calculates and displays
//      - RF amp power in
//      - RF amp power out
//      - RF amp reflected power
//
// Calibration menu functions
//  - Pos Vpp
//  - Neg Vpp
//  - Setpoint
//  - DC outputs (2)
//  - Power cal, use 50 ohm load and set voltage for 40 watts then calculate all needed values
//
// To do:
//    - Add the calibration functions
//    - Add DC control
//    - Add scaning control
//    - Add serial commands
//
#include "RFAMP.h"
#include "Variants.h"
#include "Hardware.h"
#include "Table.h"
#include "Errors.h"

//MIPS Threads
Thread RFAthread  = Thread();

#define RFAD    RFAarray[SelectedRFAboard]
#define DDSref  20000000

int      NumberOfRFAchannels = 0;
int      SelectedRFAboard=0;                                    // Active board, 0 or 1. This is also the index into the data array for RF amp
int      RFAmodule = 1;                                         // Selected RFamp module
bool     RFAupdate = true;                                      // Force all values to update
bool     QUADupdateFlag = false;                                // This is a flag set by the UI to cause the QUAD parameters to be recalculated
uint16_t RFACPLDimage[2] = {(1<<RFAcpldSTATUS) | (1<<RFAcpldRANGE),(1<<RFAcpldSTATUS) | (1<<RFAcpldRANGE)}; // CPLD control work image

RFAdata  *RFAarray[2] = {NULL,NULL};       // Pointer to RF amp data arrays, one for each module. Allocated at init time
RFAdata  rfad;                             // Copy of the selected channels data, must be static used in menu structs

RFAstate *RFAstates[2] = {NULL,NULL};

extern DialogBoxEntry RFAdialogEntriesPage2[];
extern DialogBoxEntry RFAdialogEntriesPage3[];
extern DialogBoxEntry RFAdialogEntriesCal[];
extern DialogBoxEntry RFAdialogEntriesQUAD[];
extern DialogBoxEntry RFAdialogEntriesCal2[];

DialogBoxEntry RFAdialogEntryDrive    = {"Drive         "     , 1, 4, D_FLOAT    , 0, 100, 0.1, 17, false, "%5.1f", &rfad.Drive, NULL, NULL};
DialogBoxEntry RFAdialogEntrySetPoint = {"SetPoint, Vp-p"     , 1, 4, D_FLOAT    , 0, 4000,  1, 17, false, "%5.0f", &rfad.SetPoint, NULL, NULL};

DialogBoxEntry RFAdialogEntriesPage1[] = {
  {"Module"             , 1, 0, D_INT      , 1, 1, 1, 20, false, "%2d", &RFAmodule, NULL, RFAmoduleSelected},
  {"Enable"             , 1, 1, D_ONOFF    , 0, 1, 1, 19, false, NULL, &rfad.Enabled, NULL, NULL},
  {"Mode"               , 1, 2, D_OPENCLOSE, 0, 1, 1, 17, false, NULL, &rfad.Mode, NULL, NULL},
  {"Frequency"          , 1, 3, D_INT      , 500000, 5000000, 1000, 15, false, "%7d", &rfad.Freq, NULL, NULL},
  RFAdialogEntryDrive,
  {"RF+ Vp-p"           , 1, 6, D_FLOAT    , 0, 0, 0, 17, true, "%5.0f", NULL, NULL, NULL},
  {"RF- Vp-p"           , 1, 7, D_FLOAT    , 0, 0, 0, 17, true, "%5.0f", NULL, NULL, NULL},
  {"RF power,W"         , 1, 8, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"Next page"          , 1, 9, D_PAGE     , 0, 0, 0, 0 , false, NULL, RFAdialogEntriesPage2, NULL, NULL},
  {"Return to main menu", 1, 10,D_MENU     , 0, 0, 0, 0 , false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry RFAdialogEntriesPage2[] = {
  {"QUAD parameters"    , 1, 1, D_PAGE, 0, 0, 0, 0, false, NULL, RFAdialogEntriesQUAD, NULL, NULL},
  {"RF amp parameters"  , 1, 2, D_PAGE, 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage3, NULL, NULL},
  {"DC bias outputs"    , 1, 3, D_PAGE, 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {"Invert"             , 1, 4, D_YESNO, 0, 1, 1, 19, false, NULL, &rfad.Invert, NULL, NULL},
  {"Calibration"        , 1, 5, D_PAGE, 0, 0, 0, 0, false, NULL, RFAdialogEntriesCal, LowRangeCalPrep, NULL},
  {"Save settings"      , 1, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFAsettings, NULL},
  {"Restore settings"   , 1, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFAsettings, NULL},
  {"First page"         , 1,10, D_PAGE, 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage1, NULL, NULL},
  {NULL},
};

DialogBoxEntry RFAdialogEntriesQUAD[] = {
  {"Ro, mm"             , 1, 1, D_FLOAT    , 1, 10, 0.001, 17, false, "%5.3f", &rfad.Ro, NULL, NULL},
  {"ResolvingDC, V"     , 1, 2, D_FLOAT    , -400, 400, 1, 17, false, "%5.1f", &rfad.ResolvingDC, NULL, NULL},
  {"Pole Bias, V"       , 1, 3, D_FLOAT    , -400, 400, 1, 17, false, "%5.0f", &rfad.PoleBias, NULL, NULL},
  {"Resolution, AMU"    , 1, 4, D_FLOAT    , 0, 100, 1, 19, false, "%3.0f", &rfad.Res, NULL, NULL},
  {"m/z"                , 1, 5, D_FLOAT    , 100, 2000, 1, 17, false, "%5.0f", &rfad.mz, NULL, NULL},
  {"Update"             , 1, 6, D_FUNCTION , 0, 0, 0, 0, false, NULL, NULL, SetUpdateFlag, NULL},
  {"Previous page"      , 1,10, D_PAGE     , 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage2, NULL, NULL},
  {NULL},
};

// Order need to remain constant
DialogBoxEntry RFAdialogEntriesPage3[] = {
  {"DC voltage, V"      , 1, 1, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"DC current, A"      , 1, 2, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"DC pwr in, W"       , 1, 3, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"Heatsink temp, C"   , 1, 4, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"RF voltage, Vrms"   , 1, 5, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"RF current, Irms"   , 1, 6, D_FLOAT    , 0, 0, 0, 17, true, "%5.2f", NULL, NULL, NULL},
  {"RF fwd pwr,W"       , 1, 7, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"RF rev pwr,W"       , 1, 8, D_FLOAT    , 0, 0, 0, 17, true, "%5.1f", NULL, NULL, NULL},
  {"SWR"                , 1, 9, D_FLOAT    , 0, 0, 0, 17, true, "%5.0f", NULL, NULL, NULL},
  {"Previous page"      , 1,10, D_PAGE     , 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage2, NULL, NULL},
  {NULL},
};

// ADC raw values for calibration function
int VpP1adc,VpP2adc,VnP1adc,VnP2adc;
int VpP1adcLR,VpP2adcLR,VnP1adcLR,VnP2adcLR;
// Setpoint calibration values for calibration function
float SP1,SP2, SP1V, SP2V;
float SP1LR,SP2LR, SP1VLR, SP2VLR;

char RFcalP1[10];
char RFcalP2[10];

DialogBoxEntry RFAdialogEntriesCal[] = {
  {"Range thres"        , 1, 1, D_FLOAT , 0, 500, 1, 17, false, "%5.0f", &rfad.RangeThreshold, NULL, UpdateLowRangeMenu},
  {"Cal LR V+"          , 1, 2, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {RFcalP1              , 1, 3, D_FLOAT , 0, 100, 0.01, 17, false, "%5.2f", &rfad.Drive, RFdrvCalPrep, RFpP1readLR},
  {RFcalP2              , 1, 4, D_FLOAT , 0, 100, 0.01, 17, false, "%5.2f", &rfad.Drive, RFdrvCalPrep, RFpP2readLR},
  {"Cal LR V-"          , 1, 5, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {RFcalP1              , 1, 6, D_FLOAT , 0, 100, 0.01, 17, false, "%5.2f", &rfad.Drive, RFdrvCalPrep, RFnP1readLR},
  {RFcalP2              , 1, 7, D_FLOAT , 0, 100, 0.01, 17, false, "%5.2f", &rfad.Drive, RFdrvCalPrep, RFnP2readLR},
  {"Cal LR SP"          , 1, 8, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Point 1"           , 1, 9, D_FLOAT , 0, 4000, 1, 17, false, "%5.0f", &rfad.SetPoint, RFdrvCalPrep, RFspP1readLR},
  {" Point 2"           , 1,10, D_FLOAT , 0, 4000, 1, 17, false, "%5.0f", &rfad.SetPoint, RFdrvCalPrep, RFspP2readLR},
  {"Next"               , 1,11, D_PAGE  , 0, 0, 0, 0, false, NULL, RFAdialogEntriesCal2, HighRangeCalPrep, NULL},
  {NULL},
};

DialogBoxEntry RFAdialogEntriesCal2[] = {
  {"Calibrate V+"       , 1, 1, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {RFcalP1              , 1, 2, D_FLOAT , 0, 100, 0.1, 17, false, "%5.1f", &rfad.Drive, RFdrvCalPrep, RFpP1read},
  {RFcalP2              , 1, 3, D_FLOAT , 0, 100, 0.1, 17, false, "%5.1f", &rfad.Drive, RFdrvCalPrep, RFpP2read},
  {"Calibrate V-"       , 1, 4, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {RFcalP1              , 1, 5, D_FLOAT , 0, 100, 0.1, 17, false, "%5.1f", &rfad.Drive, RFdrvCalPrep, RFnP1read},
  {RFcalP2              , 1, 6, D_FLOAT , 0, 100, 0.1, 17, false, "%5.1f", &rfad.Drive, RFdrvCalPrep, RFnP2read},
  {"Calibrate SP"       , 1, 7, D_TITLE , 0, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Point 1"           , 1, 8, D_FLOAT , 0, 4000, 1, 17, false, "%5.0f", &rfad.SetPoint, RFdrvCalPrep, RFspP1read},
  {" Point 2"           , 1, 9, D_FLOAT , 0, 4000, 1, 17, false, "%5.0f", &rfad.SetPoint, RFdrvCalPrep, RFspP2read},
  {"Calibrate"          , 1,10, D_PAGE  , 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage2, RFampCalibrate, NULL},
  {"Abort"              , 1,11, D_PAGE  , 0, 0, 0, 0, false, NULL, RFAdialogEntriesPage2, NULL, NULL},
  {NULL},
};

DialogBox RFAdialog = {
  {
    "RF amp parameters",
    ILI9340_BLACK,ILI9340_WHITE,2,0, 0,300, 220,B_DOUBLE,12
  },
  M_SCROLLING,0,0,false,RFAdialogEntriesPage1
};

MenuEntry MERFAmonitor = {" RF amp module", M_DIALOG,0,0,0,NULL,&RFAdialog,NULL,NULL};

void UpdateLowRangeMenu(void)
{
  LowRangeCalPrep();
  DialogBoxDisplay(&RFAdialog);
}

void LowRangeCalPrep(void)
{
  SetRangeLow(SelectedRFAboard);
  // Set the UI range message text
  sprintf(RFcalP1," %4.0f V", rfad.RangeThreshold * .25);
  sprintf(RFcalP2," %4.0f V", rfad.RangeThreshold * .75);
}

void HighRangeCalPrep(void)
{
  SetRangeHigh(SelectedRFAboard);
  // Set the UI range message text
  sprintf(RFcalP1," %4.0f V", rfad.FullScale * .25);
  sprintf(RFcalP2," %4.0f V", rfad.FullScale * .75);
}

float GetAverageV(void)
{
  float V =  Counts2Value(AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLA, 100),&RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLA]);
  V += Counts2Value(AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLB, 100),&RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLB]);
  return(V/2.0);
}
uint16_t CaledFlag = 0;
void RFdrvCalPrep(void) { RFAD->Drive = rfad.Drive = RFAD->SetPoint = rfad.SetPoint = 0; }
void RFpP1read(void)    { SelectBoard(SelectedRFAboard); VpP1adc   = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLA, 100); CaledFlag |= 1; }
void RFpP2read(void)    { SelectBoard(SelectedRFAboard); VpP2adc   = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLA, 100); CaledFlag |= 2; }
void RFnP1read(void)    { SelectBoard(SelectedRFAboard); VnP1adc   = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLB, 100); CaledFlag |= 4; }
void RFnP2read(void)    { SelectBoard(SelectedRFAboard); VnP2adc   = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLB, 100); CaledFlag |= 8; }
void RFpP1readLR(void)  { SelectBoard(SelectedRFAboard); VpP1adcLR = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLA, 100); CaledFlag |= 64; }
void RFpP2readLR(void)  { SelectBoard(SelectedRFAboard); VpP2adcLR = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLA, 100); CaledFlag |= 128; }
void RFnP1readLR(void)  { SelectBoard(SelectedRFAboard); VnP1adcLR = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLB, 100); CaledFlag |= 256; }
void RFnP2readLR(void)  { SelectBoard(SelectedRFAboard); VnP2adcLR = AD7998(RFAarray[SelectedRFAboard]->ADCadr, RFAadcRFLB, 100); CaledFlag |= 512; }
void RFspP1read(void)   { SelectBoard(SelectedRFAboard); SP1 = rfad.SetPoint; SP1V = GetAverageV(); CaledFlag |= 16; }
void RFspP2read(void)   { SelectBoard(SelectedRFAboard); SP2 = rfad.SetPoint; SP2V = GetAverageV(); CaledFlag |= 32; }
void RFspP1readLR(void) { SelectBoard(SelectedRFAboard); SP1LR = rfad.SetPoint; SP1VLR = GetAverageV(); CaledFlag |= 1024; }
void RFspP2readLR(void) { SelectBoard(SelectedRFAboard); SP2LR = rfad.SetPoint; SP2VLR = GetAverageV(); CaledFlag |= 2048; }
void RFampCalibrate(void)
{
  RFAD->Drive = rfad.Drive = 0;
  RFAD->SetPoint = rfad.SetPoint = 0;

  if((CaledFlag & 0x03) == 0x03)
  {
     RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLA].m = (float)(VpP2adc - VpP1adc) / (RFAD->FullScale * 0.5);
     RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLA].b = (float)VpP1adc - (RFAD->FullScale * 0.25) * RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLA].m;
     CaledFlag &= ~0x03;
  }
  if((CaledFlag & 0x0C) == 0x0C)
  {
     RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLB].m = (float)(VnP2adc - VnP1adc) / (RFAD->FullScale * 0.5);
     RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLB].b = (float)VnP1adc - (RFAD->FullScale * 0.25) * RFAarray[SelectedRFAboard]->ADCchans[RFAadcRFLB].m;
     CaledFlag &= ~0x0C;
  }
  if((CaledFlag & 0x30) == 0x30)
  {
     int c1 = Value2Counts(SP1, &RFAarray[SelectedRFAboard]->DACchans[RFAdacSETPOINT]);
     int c2 = Value2Counts(SP2, &RFAarray[SelectedRFAboard]->DACchans[RFAdacSETPOINT]);
     // SP1 = (c1 - b)/m, SP2 = (c2 - b)/m
     RFAarray[SelectedRFAboard]->DACchans[RFAdacSETPOINT].m = (float)(c1 - c2)/(SP1V - SP2V);
     RFAarray[SelectedRFAboard]->DACchans[RFAdacSETPOINT].b = (float)c1 - SP1V * RFAarray[SelectedRFAboard]->DACchans[RFAdacSETPOINT].m;
     CaledFlag &= ~0x30;
  }
  // Low range calibration
  if((CaledFlag & 0xC0) == 0xC0)
  {
     RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLA-6].m = (float)(VpP2adcLR - VpP1adcLR) / (RFAD->RangeThreshold * 0.5);
     RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLA-6].b = (float)VpP1adcLR - (RFAD->RangeThreshold * 0.25) * RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLA-6].m;
     CaledFlag &= ~0xC0;
  }
  if((CaledFlag & 0x300) == 0x300)
  {
     RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLB-6].m = (float)(VnP2adcLR - VnP1adcLR) / (RFAD->RangeThreshold * 0.5);
     RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLB-6].b = (float)VnP1adcLR - (RFAD->RangeThreshold * 0.25) * RFAarray[SelectedRFAboard]->ADCchansLR[RFAadcRFLB-6].m;
     CaledFlag &= ~0x300;
  }
  if((CaledFlag & 0xC00) == 0xC00)
  {
     int c1 = Value2Counts(SP1LR, &RFAarray[SelectedRFAboard]->DACchansLR);
     int c2 = Value2Counts(SP2LR, &RFAarray[SelectedRFAboard]->DACchansLR);
     // SP1 = (c1 - b)/m, SP2 = (c2 - b)/m
     RFAarray[SelectedRFAboard]->DACchansLR.m = (float)(c1 - c2)/(SP1VLR - SP2VLR);
     RFAarray[SelectedRFAboard]->DACchansLR.b = (float)c1 - SP1V * RFAarray[SelectedRFAboard]->DACchansLR.m;
     CaledFlag &= ~0xC00;
  }
}

void SetRangeHigh(int brd)
{
   RFACPLDimage[brd] |= 1<<RFAcpldRANGE;
   SendFRAcpldCommand(RFAarray[brd]->CPLDspi, RFACPLDimage[brd]);  
}

void SetRangeLow(int brd)
{
   RFACPLDimage[brd] &= ~(1<<RFAcpldRANGE);
   SendFRAcpldCommand(RFAarray[brd]->CPLDspi, RFACPLDimage[brd]);  
}

// Set the QUADupdate flag to cause QUAD parameters to be recalcualted and update
// the parameters.
void SetUpdateFlag(void)
{
  QUADupdateFlag = true;
}

// Calculate the RF and DC parameters for the selected QUAD
void QUADupdate(int brd)
{
  double c = (float)RFAarray[brd]->Freq * (float)RFAarray[brd]->Freq * RFAarray[brd]->mz * (RFAarray[brd]->Ro/1000) * (RFAarray[brd]->Ro/1000);
  RFAarray[brd]->SetPoint = (0.000000072226 * c) * 2;
  float DCV = 0.000000012122 * c;
  DCV *= ((RFAarray[brd]->mz - RFAarray[brd]->Res) / RFAarray[brd]->mz);
  RFAarray[brd]->ResolvingDC = DCV;  
}

// Called when user user selects mode (open or closed) number and/or the UI updates
void RFAmodeSelected(void)
{
  if(rfad.Mode)
  {
    // Open loop
    RFAdialogEntriesPage1[4] = RFAdialogEntryDrive;
  }
  else
  {
    // Closed loop
    RFAdialogEntriesPage1[4] = RFAdialogEntrySetPoint;
  }
  // If this page of the dialog is displayed update
  if(ActiveDialog->Entry == RFAdialogEntriesPage1)
  {
    if (RFAdialogEntriesPage1[4].Name != NULL) DisplayDialogEntryNames(&RFAdialog.w, &RFAdialogEntriesPage1[4], false);
  }  
}

// Called when user user selects a module number and/or the UI updates
void RFAmoduleSelected(void)
{
  // Convert module number to Selected board and define selected board
  if(RFAmodule == 1)
  {
    if(RFAarray[0] != 0) SelectedRFAboard = 0;
    else if(RFAarray[1] != 0) SelectedRFAboard = 1;
  }
  if(RFAmodule == 2)
  {
    if((RFAarray[0] != 0) && (RFAarray[1] != 0)) SelectedRFAboard = 1;
  }
  rfad = *RFAD;
  // Setup menu based on loop mode
  RFAmodeSelected();
  // Update the page is displayed
  if(ActiveDialog->Entry == RFAdialogEntriesPage1) DialogBoxDisplay(&RFAdialog);
}

void SaveRFAsettings(void)
{
  bool Success = true;
  
  if(RFAarray[SelectedRFAboard] != NULL)
  {
    SelectBoard(SelectedRFAboard);
    if(WriteEEPROM(RFAarray[SelectedRFAboard], RFAarray[SelectedRFAboard]->EEPROMadr, 0, sizeof(RFAdata)) != 0) Success = false;
  } 
  if(Success)
  {
    DisplayMessage("Parameters Saved!",2000);
  }
  else DisplayMessage("Error saving!",2000);
}

void RestoreRFAsettings(void)
{
  RestoreDACsettings(false);
}

void RestoreRFAsettings(bool NoDisplay)
{
  int b;
  RFAdata rfa_data;
  bool Success=true;
  bool Corrupted=false;
  
  b = SelectedDACboard;
  if(RFAarray[b] == NULL) return;
  SelectBoard(b);
  if(ReadEEPROM(&rfa_data, RFAarray[b]->EEPROMadr, 0, sizeof(RFAdata)) == 0)
  {
     if(strcmp(rfa_data.Name,RFAarray[b]->Name) == 0)
     {
       // Here if the name matches so copy the data to the operating data structure
       rfa_data.EEPROMadr = RFAarray[b]->EEPROMadr;
       memcpy(RFAarray[b], &rfa_data, sizeof(RFAdata));
     } 
     else Corrupted=true;
  }
  else Success=false;
  rfad = *RFAarray[SelectedDACboard];
  RFAmoduleSelected();
  if(NoDisplay) return;
  if(Corrupted) DisplayMessage("Corrupted EEPROM data!",2000);
  else if(!Success)  DisplayMessage("Unable to Restore!",2000);
  else DisplayMessage("Parameters Restored!",2000);
  if(ActiveDialog == &RFAdialog) DialogBoxDisplay(&RFAdialog);
}

void SetDDSfrequency(int8_t addr, uint32_t freq)
{
  float    fFREQ;
  uint32_t uFREQ;

  // Calculate the ferquency value
  fFREQ = (float)freq/(float)DDSref * 0xFFFFFFFF;
  uFREQ = fFREQ;
  // Write the value to DDS chip, analog devices AD9832
  SetAddress(addr);
  SPI.setDataMode(SPI_CS, SPI_MODE2);
  SPI.transfer(SPI_CS, 0xF8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0);

  SPI.transfer(SPI_CS, 0x33, SPI_CONTINUE);
  SPI.transfer(SPI_CS, (uFREQ >> 24) & 0xFF);
  SPI.transfer(SPI_CS, 0x22, SPI_CONTINUE);
  SPI.transfer(SPI_CS, (uFREQ >> 16) & 0xFF);
  SPI.transfer(SPI_CS, 0x31, SPI_CONTINUE);
  SPI.transfer(SPI_CS, (uFREQ >> 8) & 0xFF);
  SPI.transfer(SPI_CS, 0x20, SPI_CONTINUE);
  SPI.transfer(SPI_CS, uFREQ & 0xFF);

  SPI.transfer(SPI_CS, 0xC0, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0);
  SetAddress(0);
}

// This function sends a control word to the control CPLD
void SendFRAcpldCommand(int8_t addr, int16_t cmd)
{
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Set the SPI address
  SetAddress(addr);
  // Send data
  SPI.setDataMode(SPI_CS, SPI_MODE2);
  SPI.transfer(SPI_CS, cmd >> 8, SPI_CONTINUE);
  SPI.transfer(SPI_CS, cmd);
  // Pulse strobe and exit
  digitalWrite(RFAstrobe,HIGH);
  digitalWrite(RFAstrobe,LOW);
  SetAddress(0);  
}

// This function sets up the user interface data sructures based on the selected
// board, this is also the structure index.
// Uses: SelectedRFAboard and RFAstates
void SetupRFAentries(void)
{
  DialogBoxEntry *de;

  if(RFAstates[SelectedRFAboard] == NULL) return;
  // Setup pointers to monitored and calculated values in the states structure
  de = GetDialogEntries(RFAdialogEntriesPage1, "RF+ Vp-p");
  de[0].Value = (void *)&RFAstates[SelectedRFAboard]->RFVPpp;
  de[1].Value = (void *)&RFAstates[SelectedRFAboard]->RFVNpp;
  de[2].Value = (void *)&RFAstates[SelectedRFAboard]->RFpowerFWD;
  de = RFAdialogEntriesPage3;
  de[0].Value = (void *)&RFAstates[SelectedRFAboard]->DCV;
  de[1].Value = (void *)&RFAstates[SelectedRFAboard]->DCI;
  de[2].Value = (void *)&RFAstates[SelectedRFAboard]->DCpower;
  de[3].Value = (void *)&RFAstates[SelectedRFAboard]->Temp;
  de[4].Value = (void *)&RFAstates[SelectedRFAboard]->RFV;
  de[5].Value = (void *)&RFAstates[SelectedRFAboard]->RFI;
  de[6].Value = (void *)&RFAstates[SelectedRFAboard]->RFpowerFWD;
  de[7].Value = (void *)&RFAstates[SelectedRFAboard]->RFpowerREF;
  de[8].Value = (void *)&RFAstates[SelectedRFAboard]->SWR;
}

// This function is called at powerup to initiaize the RF amp modules(s).
void RFA_init(int8_t Board, int8_t addr)
{
  DialogBoxEntry *de = GetDialogEntries(RFAdialogEntriesPage1, "Module");
  
  // Allocate the module data structure based on board number passed
  RFAarray[Board] = new RFAdata;
  RFAstates[Board] = new RFAstate;
  // Init the structure variables
  RFAstates[Board]->ResolvingDC = 0.0;
  RFAstates[Board]->PoleBias = 0.0;  
  RFAstates[Board]->RFVPpp = 0;
  RFAstates[Board]->RFVNpp = 0;
  RFAstates[Board]->RFpowerFWD = 0;
  RFAstates[Board]->DCV = 0;
  RFAstates[Board]->DCI = 0;
  RFAstates[Board]->DCpower = 0;
  RFAstates[Board]->Temp = 0;
  RFAstates[Board]->RFV = 0;
  RFAstates[Board]->RFI = 0;
  RFAstates[Board]->RFpowerREF = 0;
  RFAstates[Board]->SWR = 0;
  RFAstates[Board]->ph = 0;
  SelectBoard(Board);
  // Fill the array with default data
  *RFAarray[Board] = RFA_Rev1;
  RFAarray[Board]->EEPROMadr = addr;
  // Restore parameters from module's memory
  if(NormalStartup) RestoreRFAsettings(true);
  // Set full scale limit
  RFAdialogEntrySetPoint.Max = RFAarray[Board]->FullScale;
  // Setup menu and start the thread
  if(NumberOfRFAchannels == 0)
  {
    pinMode(RFAstrobe, OUTPUT);
    digitalWrite(RFAstrobe, LOW);
    SelectedRFAboard = Board;
    SelectBoard(SelectedRFAboard);
    // Setup the user interface structures for this module
    SetupRFAentries();
    // Setup the menu
    AddMainMenuEntry(&MERFAmonitor);
    if(ActiveDialog == NULL) DialogBoxDisplay(&RFAdialog);
    // Configure Threads
    RFAthread.setName("RFamp");
    RFAthread.onRun(RFA_loop);
    RFAthread.setInterval(100);
    // Add threads to the controller
    control.add(&RFAthread);
  }
  NumberOfRFAchannels++;
  if(NumberOfRFAchannels > 1) de->Max = 2;
  SelectBoard(SelectedRFAboard);
}

// This is the RF amp processing loop, called by the task scheduler 10 times
// per second.
void RFA_loop(void)
{
  int      brd;
  uint16_t ADCvals[8];
  float    val;
  
  // If the user has changed any values upate the proper data structure
  if (ActiveDialog == &RFAdialog)
  {
    if(ActiveDialog->Changed)
    {
      *RFAD = rfad;
      RFAdialog.Changed = false;
    }
  }
  // Process all modules in system
  for(brd=0;brd<2;brd++)
  {
    if(RFAarray[brd] == NULL) continue;
    if(RFAstates[brd] == NULL) continue;
    SelectBoard(brd);
    if(QUADupdateFlag) QUADupdate(brd);
    // Look for any changed values and update
    if((RFAarray[brd]->Enabled != RFAstates[brd]->Enabled) || RFAupdate)
    {
      if(RFAarray[brd]->Enabled)
      {
        // Here if enabled so set the drive and SetPoint levels
        AD5625(RFAarray[brd]->DACadr,RFAdacDRIVE,Value2Counts(RFAarray[brd]->Drive,&RFAarray[brd]->DACchans[RFAdacDRIVE]),3);
        AD5625(RFAarray[brd]->DACadr,RFAdacSETPOINT,Value2Counts(RFAarray[brd]->SetPoint,&RFAarray[brd]->DACchans[RFAdacSETPOINT]),3);
      }
      else
      {
        // Here if disabled, set drive and setpoint outputs to 0
        AD5625(RFAarray[brd]->DACadr,RFAdacDRIVE,Value2Counts(0,&RFAarray[brd]->DACchans[RFAdacDRIVE]),3);
        AD5625(RFAarray[brd]->DACadr,RFAdacSETPOINT,Value2Counts(0,&RFAarray[brd]->DACchans[RFAdacSETPOINT]),3);
      }
      RFAstates[brd]->Enabled = RFAarray[brd]->Enabled;
    }
    if((RFAarray[brd]->Freq != RFAstates[brd]->Freq) || RFAupdate)
    {
      // Set the frequency
      SetDDSfrequency(RFAarray[brd]->DDSspi, RFAarray[brd]->Freq);
      RFAstates[brd]->Freq = RFAarray[brd]->Freq;
    }
    if((RFAarray[brd]->Drive != RFAstates[brd]->Drive) || RFAupdate)
    {
      // Set the drive level if enabled
      if(RFAarray[brd]->Enabled) AD5625(RFAarray[brd]->DACadr,RFAdacDRIVE,Value2Counts(RFAarray[brd]->Drive,&RFAarray[brd]->DACchans[RFAdacDRIVE]),3);
      RFAstates[brd]->Drive = RFAarray[brd]->Drive;
    }
    if((RFAarray[brd]->SetPoint != RFAstates[brd]->SetPoint) || RFAupdate)
    {
      // Set the voltage setpoint if enabled
      if(RFAarray[brd]->Enabled) AD5625(RFAarray[brd]->DACadr,RFAdacSETPOINT,Value2Counts(RFAarray[brd]->SetPoint,&RFAarray[brd]->DACchans[RFAdacSETPOINT]),3);
      RFAstates[brd]->SetPoint = RFAarray[brd]->SetPoint;
    }
    if((RFAarray[brd]->PoleBias != RFAstates[brd]->PoleBias) || RFAupdate)
    {
      // Set the PoleBias value
      int bd = DCbiasCH2Brd(0);
      if(bd != -1)
      {
        DCbDarray[bd]->DCoffset.VoltageSetpoint =  RFAarray[brd]->PoleBias;
        DCbDarray[bd]->DCCD[0].VoltageSetpoint  =  RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        DCbDarray[bd]->DCCD[1].VoltageSetpoint  = -RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        if(GetDCbiasBoard(1) == SelectedDCBoard) dcbd.DCoffset.VoltageSetpoint = RFAarray[brd]->PoleBias;
        if(GetDCbiasBoard(1) == SelectedDCBoard) dcbd.DCCD[0].VoltageSetpoint =  RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        if(GetDCbiasBoard(2) == SelectedDCBoard) dcbd.DCCD[1].VoltageSetpoint = -RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        DelayMonitoring();
      }
      RFAstates[brd]->PoleBias = RFAarray[brd]->PoleBias;
    }
    if((RFAarray[brd]->ResolvingDC != RFAstates[brd]->ResolvingDC) || RFAupdate)
    {
      // Set the ResolvingDC value
      int bd = DCbiasCH2Brd(0);
      if(bd != -1)
      {
        DCbDarray[bd]->DCCD[0].VoltageSetpoint =  RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        DCbDarray[bd]->DCCD[1].VoltageSetpoint = -RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        if(GetDCbiasBoard(1) == SelectedDCBoard) dcbd.DCCD[0].VoltageSetpoint =  RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        if(GetDCbiasBoard(2) == SelectedDCBoard) dcbd.DCCD[1].VoltageSetpoint = -RFAarray[brd]->ResolvingDC + RFAarray[brd]->PoleBias;
        DelayMonitoring();
      }
      RFAstates[brd]->ResolvingDC = RFAarray[brd]->ResolvingDC;
    }
    if((RFAarray[brd]->Mode != RFAstates[brd]->Mode) || RFAupdate)
    {
      // If this channel is selected for display then update the dialog box
      // to display proper selection, Drive or Setpoint
      if(brd == SelectedRFAboard)
      {
        rfad = *RFAD;
        RFAmodeSelected();
      }
      // Set the loop mode, Open or Closed
      if(RFAarray[brd]->Mode)
      {
        // Here is open loop mode is selected
        RFACPLDimage[brd] &= ~(1 << RFAcpldCLOSED); // Closed LED off
        RFACPLDimage[brd] &= ~(1 << RFAcpldDRVSP_SELECT); // Select the drive DAC control
      }
      else
      {
        // Here if closed loop mode is selected
        RFACPLDimage[brd] |= 1 << RFAcpldCLOSED; // Closed LED on
        RFACPLDimage[brd] |= 1 << RFAcpldDRVSP_SELECT; // Selext the setpoint DAC control
      }
      RFAstates[brd]->Mode = RFAarray[brd]->Mode;
    }
    if((RFAarray[brd]->Invert != RFAstates[brd]->Invert) || RFAupdate)
    {
      // Set the loop polarity
      if(!RFAarray[brd]->Mode) RFACPLDimage[brd] |= 1 << RFAcpldINVERT;
      else RFACPLDimage[brd] &= ~(1 << RFAcpldINVERT);      
      RFAstates[brd]->Invert = RFAarray[brd]->Invert;
    }
    if((RFACPLDimage[brd] != RFAstates[brd]->CPLDimage) || RFAupdate)
    {
      // Update the CPLD control word
      SendFRAcpldCommand(RFAarray[brd]->CPLDspi, RFACPLDimage[brd]);
      RFAstates[brd]->CPLDimage = RFACPLDimage[brd];
    }
    // Read the ADC and update all values
    if(AD7998(RFAarray[brd]->ADCadr, ADCvals)==0)
    {
      val = Counts2Value(ADCvals[RFAadcDCV],&RFAarray[brd]->ADCchans[RFAadcDCV]);
      RFAstates[brd]->DCV = Filter * val + (1-Filter) * RFAstates[brd]->DCV;
      val = Counts2Value(ADCvals[RFAadcDCI],&RFAarray[brd]->ADCchans[RFAadcDCI]);
      RFAstates[brd]->DCI = Filter * val + (1-Filter) * RFAstates[brd]->DCI;
      val = Counts2Value(ADCvals[RFAadcTEMP],&RFAarray[brd]->ADCchans[RFAadcTEMP]);
      RFAstates[brd]->Temp = Filter * val + (1-Filter) * RFAstates[brd]->Temp;
      val = Counts2Value(ADCvals[RFAadcRFV],&RFAarray[brd]->ADCchans[RFAadcRFV]);
      RFAstates[brd]->RFV = Filter * val + (1-Filter) * RFAstates[brd]->RFV;
      val = Counts2Value(ADCvals[RFAadcRFI],&RFAarray[brd]->ADCchans[RFAadcRFI]);
      RFAstates[brd]->RFI = Filter * val + (1-Filter) * RFAstates[brd]->RFI;
      if((RFACPLDimage[brd] & (1<<RFAcpldRANGE)) != 0)
      {
         val = Counts2Value(ADCvals[RFAadcRFLA],&RFAarray[brd]->ADCchans[RFAadcRFLA]);
         RFAstates[brd]->RFVPpp = Filter * val + (1-Filter) * RFAstates[brd]->RFVPpp;
         val = Counts2Value(ADCvals[RFAadcRFLB],&RFAarray[brd]->ADCchans[RFAadcRFLB]);
         RFAstates[brd]->RFVNpp = Filter * val + (1-Filter) * RFAstates[brd]->RFVNpp;
      }
      else
      {
         val = Counts2Value(ADCvals[RFAadcRFLA],&RFAarray[brd]->ADCchansLR[RFAadcRFLA-6]);
         RFAstates[brd]->RFVPpp = Filter * val + (1-Filter) * RFAstates[brd]->RFVPpp;
         val = Counts2Value(ADCvals[RFAadcRFLB],&RFAarray[brd]->ADCchansLR[RFAadcRFLB-6]);
         RFAstates[brd]->RFVNpp = Filter * val + (1-Filter) * RFAstates[brd]->RFVNpp;       
      }
      // Calculate the DC power input
      RFAstates[brd]->DCpower = RFAstates[brd]->DCV * RFAstates[brd]->DCI;
      // Calculate RFV powers and SWR
      // ph = arc cos((b^2 + a^2 - c^2)/2*a*b))
      // a = RFV dac counts
      // b = RFI dac counts
      // c = SWR dac counts
      // Forward power = RFV * RFI * cos(ph)
      // Reflected power = RFV * RFI - Forward power
      // T = srt(Reflected power / Forward power)
      // SWR = (1 + T)/(1 - T)
      float a = (float)ADCvals[RFAadcRFV]  / 10000.0;
      float b = (float)ADCvals[RFAadcRFI]  / 10000.0;
      float c = (float)ADCvals[RFAadcSWR]  / 10000.0;
      b *= 1.08;  // correction factor
      // Calibrate by using a 50 ohm load and setting power to a know level then have system calculate all the parameters
      if((2*a*b) == 0) a = 0;
      else a = (b*b + a*a - c*c)/(2*a*b);
      if(a > 1.0) a = 1.0;
      if(a < -1.0) a = -1.0;
      float ph = acos(a);
      RFAstates[brd]->ph = Filter * ph + (1-Filter) * RFAstates[brd]->ph;
      RFAstates[brd]->RFpowerFWD = RFAstates[brd]->RFV * RFAstates[brd]->RFI;
      RFAstates[brd]->RFpowerREF = RFAstates[brd]->RFpowerFWD * sin(RFAstates[brd]->ph);
      if(RFAstates[brd]->RFpowerFWD != 0.0)
      {
         float T = sqrt(RFAstates[brd]->RFpowerREF/RFAstates[brd]->RFpowerFWD);
         RFAstates[brd]->SWR = abs((1.0 + T)/(1.0 - T));
         if(RFAstates[brd]->SWR > 100) RFAstates[brd]->SWR = 100;
      }
    }
  }
  RFAupdate = false;
  QUADupdateFlag = false;
  SelectBoard(SelectedRFAboard);
  if ((ActiveDialog == &RFAdialog) && (ActiveDialog->Entry != RFAdialogEntriesCal) && (ActiveDialog->Entry != RFAdialogEntriesCal2)) RefreshAllDialogEntries(&RFAdialog);
  rfad = *RFAD;
}

//
// Host serial commands
// 

int RFAmodule2board(int Module)
{
  int b=-1;

  if(Module == 1)
  {
    if(RFAarray[0] != NULL) b = 0;
    if((RFAarray[0] == NULL) && (RFAarray[1] != NULL)) b = 1;
  }
  if((Module == 2) && (RFAarray[0] != NULL) && (RFAarray[1] != NULL)) b = 1;
  if(b != -1) return(b);
  SetErrorCode(ERR_BADCMD);
  SendNAK;
  return(b);
}

void RFAsetPoleBias(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;
  char   vstr[100];

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  RFAarray[b]->PoleBias = v;
}

void RFAgetPoleBias(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->PoleBias);  
}

void RFAsetResolvingDC(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;
  char   vstr[100];

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  RFAarray[b]->ResolvingDC = v;
}

void RFAgetResolvingDC(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->ResolvingDC);  
}

void RFAsetRange(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  if((v < 500) || (v > 5000))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;   
     return;
  }
  SendACK;
  RFAarray[b]->FullScale = v;
}

void RFAgetRange(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->FullScale);
}

// Set quad Ro in mm
void RFAsetRo(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  if((v < 1) || (v > 10))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;   
     return;
  }
  SendACK;
  RFAarray[b]->Ro = v; 
}

// Returns the current Ro value in cm
void RFAgetRo(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->Ro); 
}

// Set target m/z value. This function will calculate the RF and DC parameters based on 
// Ro,Res,RF freq
void RFAsetMZ(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  if((v < 100) || (v > 2000))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;   
     return;
  }
  SendACK;
  RFAarray[b]->mz = v;    
}

// Returns target M/Z value
void RFAgetMZ(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->mz);     
}

// sets resolution
void RFAsetRes(char *Module, char *value)
{
  String token;
  int    b,mod;
  float  v;

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  v = token.toFloat();
  if((v < 0) || (v > 200))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;   
     return;
  }
  SendACK;
  RFAarray[b]->Res = v;  
}

// Returns resolution
void RFAgetRes(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACKonly;
  if(!SerialMute) serial->println(RFAarray[b]->Res);   
}

void RFAupdateQUAD(int Module)
{
  int b;
  
  if((b = RFAmodule2board(Module)) == -1) return;
  SendACK;
  QUADupdate(b);
}

void RFAsetGain(char *Module, char *value)
{
  String token;
  int    mod,b;

  token = Module;
  mod = token.toInt();
  if((b = RFAmodule2board(mod)) == -1) return;
  token = value;
  if((token != "HIGH") && (token != "LOW"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
  }
  if(token == "HIGH") RFACPLDimage[b] |= 1<<RFAcpldRANGE;
  else RFACPLDimage[b] &= ~(1<<RFAcpldRANGE);
}




