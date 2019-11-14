//
// File: FAIMSfb
//
//  This file contains the MIPS functions supporting the FAIMSfb module. This is a FAIMS system using a fly-back
//  transformer design. This developed was driven by Gary Eiceman. 
//
//  MIPS controller EXT1 pin 13 (48) connects to FAIMSFB M0 pin A2. This MIPS pin is programmed to be an output
//  and used to signal scan step advance to the FAIMSFB module. 
//
//  Use the ARB common clock timer for the scan step timing generation. TMR_ARBclock, timer channel 6. This timer
//  will be programmed to generate a interrupt at the scan step duration period. This interrupt will toggle the
//  step line (48). 
//
//  Gordon Anderson
//  March 10, 2019
//

#include "FAIMSfb.h"
#include "Arduino.h"
#include <Wire.h>

#if FAIMSFBcode

MIPStimer *FAIMSfbScanClock = NULL;

//MIPS Threads
Thread FAIMSFBthread  = Thread();

#define FAIMSFB FAIMSFBarray[SelectedFAIMSFBboard]

FAIMSFBdata  *FAIMSFBarray[2]  = {NULL,NULL};
FAIMSFBstate *FAIMSFBstates[2] = {NULL,NULL};
ReadBacks    *FAIMSFBrb[2]     = {NULL,NULL};
FAIMSFBdata  faimsfb;  // This is the display / UI data structure
FAIMSFBvars  *faimsfbvars = NULL;

int NumberOfFAIMSFBchannels    =  0;
int SelectedFAIMSFBboard       =  0;    // Active board, 0 or 1 = A or B
int FAIMSFBmodule              =  1;
int CurrentFAIMSFBmodule       = -1;

extern DialogBoxEntry FAIMSFBentries[];
extern DialogBoxEntry FAIMSFBsettings[];
extern DialogBoxEntry FAIMSFBscan[];
extern DialogBoxEntry ElectrometerSettings[];

DialogBoxEntry FAIMSFBentries[] = {
  {" Module"             , 0, 1, D_INT  , 1, 1  , 1, 21, false, "%2d", &FAIMSFBmodule, NULL, NULL},
  {" Enable"             , 0, 2, D_ONOFF, 0, 1  , 1, 20, false, NULL, &faimsfb.Enable, NULL, NULL},
  {" Mode"               , 0, 3, D_ONOFF, 0, 1  , 1, 20, false, NULL, &faimsfb.Mode, NULL, NULL},
  {" Drive"              , 0, 4, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &faimsfb.Drive, NULL, NULL},
  {"        Request  Actual", 0, 5, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {" Vrf"                , 0, 6, D_FLOAT, 0, 2000, 10, 8, false, "%5.0f", &faimsfb.Vrf, NULL, NULL},
  {" CV"                 , 0, 7, D_FLOAT, -48, 48, 0.1, 8, false, "%5.1f", &faimsfb.CV, NULL, NULL},
  {""                    , 0, 6, D_FLOAT, 0, 2000, 10, 18, true, "%5.0f", NULL, NULL, NULL},
  {""                    , 0, 7, D_FLOAT, -48, 48, 0.1, 18, true, "%5.1f", NULL, NULL, NULL},
  {" Power"              , 0, 8, D_FLOAT, 0, 100, 1, 18, true, "%5.1f", NULL, NULL, NULL},
  {" Settings"           , 0, 9, D_PAGE,  0, 0  , 0, 0,  false, NULL, FAIMSFBsettings, NULL, NULL},
  {" Electrometer"       , 0,10, D_OFF,  0, 0  , 0, 0,  false, NULL, ElectrometerSettings, NULL, NULL},
  {" Return to main menu", 0,11, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry FAIMSFBsettings[] = {
  {" Scan"               , 0, 1, D_PAGE,  0, 0  , 0, 0,  false, NULL, FAIMSFBscan, NULL, NULL},
  {" Frequency"          , 0, 2, D_INT  , 250000, 2000000  , 1000, 16, false, "%6d", &faimsfb.Freq, NULL, NULL},
  {" Duty cycle"         , 0, 3, D_UINT8, 0, 90  , 1, 21, false, "%2d", &faimsfb.Duty, NULL, NULL},
  {" Max drive"          , 0, 4, D_FLOAT, 0, 100 , 1, 20, false, "%3.0f", &faimsfb.MaxDrive, NULL, NULL},
  {" Max power"          , 0, 5, D_FLOAT, 0, 50 , 1, 20, false, "%3.0f", &faimsfb.MaxPower, NULL, NULL},
  {" Save settings"      , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveFAIMSFBsettings, NULL},
  {" Restore settings"   , 0,10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorFAIMSFBsettings, NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, FAIMSFBentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry FAIMSFBscan[] = {
  {" Start CV"           , 0, 1, D_FLOAT, -48, 48 , 0.1, 18, false, "%5.1f", &faimsfb.CVstart, NULL, NULL},
  {" End CV"             , 0, 2, D_FLOAT, -48, 48 , 0.1, 18, false, "%5.1f", &faimsfb.CVend, NULL, NULL},
  {" Start Vrf"          , 0, 3, D_FLOAT, 0, 2000 , 10,  19, false, "%4.0f", &faimsfb.VRFstart, NULL, NULL},
  {" End Vrf"            , 0, 4, D_FLOAT, 0, 2000 , 10,  19, false, "%4.0f", &faimsfb.VRFend, NULL, NULL},
  {" Num steps"          , 0, 5, D_INT, 10, 2000 , 1,  19, false, "%4d", &faimsfb.Steps, NULL, NULL},
  {" Step duration"      , 0, 6, D_INT, 1, 2000 , 1,  19, false, "%4d", &faimsfb.StepDuration, NULL, NULL},
  {" Start scan"         , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, FAIMSfbScan, NULL},
  {" Abort scan"         , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, FAIMSfbScanAbort, NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, FAIMSFBentries, NULL, NULL},
  {NULL},
};

DialogBoxEntry ElectrometerSettings[] = {
  {" Pos current, pA"    , 0, 1, D_FLOAT, 0, 0 , 0, 17, true, "%6.2f", NULL, NULL, NULL},
  {" Neg current, pA"    , 0, 2, D_FLOAT, 0, 0 , 0, 17, true, "%6.2f", NULL, NULL, NULL},
  {" Pos offset,V"       , 0, 3, D_FLOAT, 0, 5.0 , 0.01, 18, false, "%5.3f", NULL, NULL, NULL},
  {" Neg offset,V"       , 0, 4, D_FLOAT, 0, 5.0 , 0.01, 18, false, "%5.3f", NULL, NULL, NULL},
  {" Zero"               , 0, 5, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, ZeroElectrometer, NULL},
  {" Return"             , 0,11, D_PAGE,  0, 0  , 0, 0,  false, NULL, FAIMSFBentries, NULL, NULL},
  {NULL},
};

DialogBox FAIMSFBdialog = {
  {
    "FAIMSFB control params",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0,false, FAIMSFBentries
};

MenuEntry MEFAIMSFBmodule = {" FAIMEFB module", M_DIALOG, 0, 0, 0, NULL, &FAIMSFBdialog, NULL, NULL};

// Update the dialog box and display
void SelectFAIMSFBmodule(bool paint = true)
{
  if(FAIMSFBmodule == 1)
  {
    if(FAIMSFBarray[0] != NULL) SelectedFAIMSFBboard = 0;
    else SelectedFAIMSFBboard = 1;
  }
  else SelectedFAIMSFBboard = FAIMSFBmodule - 1;
  SelectBoard(SelectedFAIMSFBboard);
  faimsfb = *FAIMSFB;
  FAIMSFBdialog.Selected = 0;
  FAIMSFBdialog.State = M_SCROLLING;
  // Repaint main menu
  if(paint) DialogBoxDisplay(&FAIMSFBdialog);
}

// Write the current board parameters to the EEPROM on the ARB board.
void SaveFAIMSFBsettings(void)
{
  SelectBoard(SelectedFAIMSFBboard);
  faimsfb.Size = sizeof(FAIMSFBdata);
  if (WriteEEPROM(&faimsfb, faimsfb.TWIadd, 0, sizeof(FAIMSFBdata)) == 0)
  {
    DisplayMessage("Parameters Saved!", 2000);
  }
  else DisplayMessage("Unable to Save!", 2000);
}

void RestorFAIMSFBsettings(bool NoDisplay)
{
  FAIMSFBdata fd;

  SelectBoard(SelectedFAIMSFBboard);
  if (ReadEEPROM(&fd, FAIMSFB->TWIadd, 0, sizeof(FAIMSFBdata)) == 0)
  {
    if (strcmp(fd.Name, "FAIMSfb") == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (fd.Size > sizeof(FAIMSFBdata)) fd.Size = sizeof(FAIMSFBdata);
      fd.TWIadd = FAIMSFB->TWIadd;
      memcpy(FAIMSFB, &fd, fd.Size);
      faimsfb = *FAIMSFB;
      if (!NoDisplay) DisplayMessage("Parameters Restored!", 2000);
    }
    else if (!NoDisplay) DisplayMessage("Corrupted EEPROM data!", 2000);
  }
  else if (!NoDisplay) DisplayMessage("Unable to Restore!", 2000);
}

void RestorFAIMSFBsettings(void)
{
  RestorFAIMSFBsettings(false);
}

// Called from the UI to start a scan, both channels are started
void FAIMSfbScan(void)
{
   InitFBScan(3);
}

// Called from the menu system to abort a scan in progress
void FAIMSfbScanAbort(void)
{
  StopFBScan(3);
}

void FAIMSfb_init(int8_t Board, int8_t addr)
{
  DialogBoxEntry *de;

  // Allocate the module data structure based on board number passed
  FAIMSFBarray[Board]  = new FAIMSFBdata;
  FAIMSFBstates[Board] = new FAIMSFBstate;
  FAIMSFBrb[Board] = new ReadBacks;
  if(faimsfbvars == NULL) faimsfbvars = new FAIMSFBvars;
  faimsfbvars->VRFrb = faimsfbvars->CVrb = faimsfbvars->PWR = 0.0;
  FAIMSFBstates[Board]->update = true;
  // Set active board to board being inited
  SelectedFAIMSFBboard = Board;
  SelectBoard(Board);
  FAIMSFB->TWIadd = addr;
  RestorFAIMSFBsettings(true);
  FAIMSFB->TWIadd = addr;
  if (NumberOfFAIMSFBchannels == 0)
  {
    // Here if this is the first FAIMSfb module
    // If the Electrometer is enabled then setup the menus
    // and init the electrometer module
    if(FAIMSFBarray[Board]->ElectrometerEnable)
    {
        // Setup the electrometer 
        ElectrometerAD5593init(FAIMSFBarray[Board]->ElectAdd);  // Init the hardware
        faimsfbvars->PosCurrent = faimsfbvars->NegCurrent = 0;
        faimsfbvars->PosOffset = FAIMSFBarray[Board]->ElectPosOffset;
        faimsfbvars->NegOffset = FAIMSFBarray[Board]->ElectNegOffset;
        // Write the zero values to the electrometer
        AD5593writeDACWire1(FAIMSFBarray[Board]->ElectAdd, FAIMSFBarray[Board]->ElectPosZeroCtrl.Chan, Value2Counts(FAIMSFBarray[Board]->ElectPosZero,&FAIMSFBarray[Board]->ElectPosZeroCtrl));
        AD5593writeDACWire1(FAIMSFBarray[Board]->ElectAdd, FAIMSFBarray[Board]->ElectNegZeroCtrl.Chan, Value2Counts(FAIMSFBarray[Board]->ElectNegZero,&FAIMSFBarray[Board]->ElectNegZeroCtrl));
        // Enable the electrometer menu option
        de = GetDialogEntries(FAIMSFBentries, "Electrometer");
        if(de != NULL) de->Type = D_PAGE;
        de = GetDialogEntries(ElectrometerSettings, "Pos current");
        if(de != NULL) 
        {
          de[0].Value = (void *)&(faimsfbvars->PosCurrent);
          de[1].Value = (void *)&(faimsfbvars->NegCurrent);
          de[2].Value = (void *)&(faimsfbvars->PosOffset);
          de[3].Value = (void *)&(faimsfbvars->NegOffset);
        }
    }
    de = GetDialogEntries(FAIMSFBentries, "CV");
    if(de != NULL) 
    {
      de[1].Value = (void *)&(faimsfbvars->VRFrb);
      de[2].Value = (void *)&(faimsfbvars->CVrb);
      de[3].Value = (void *)&(faimsfbvars->PWR);
    }
    faimsfb = *FAIMSFB;
    AddMainMenuEntry(&MEFAIMSFBmodule);
    if (ActiveDialog == NULL) ActiveDialog = &FAIMSFBdialog;
    // Configure Threads
    FAIMSFBthread.setName("FAIMSFB");
    FAIMSFBthread.onRun(FAIMSfb_loop);
    FAIMSFBthread.setInterval(100);
    // Add threads to the controller
    control.add(&FAIMSFBthread);
  }
  else
  {
    // Set back to first board
    SelectedARBboard = 0;
    SelectBoard(0);
    faimsfb = *FAIMSFB;
  }
  NumberOfFAIMSFBchannels++;
  de = GetDialogEntries(FAIMSFBentries, "Module");
  if(de != NULL) de->Max = NumberOfFAIMSFBchannels;
  if(FAIMSfbScanClock == NULL) FAIMSfbScanClock = new MIPStimer(FAIMSFB_ScanClock);
}

void FAIMSfb_loop(void)
{
  float fval;
  
  if (ActiveDialog == &FAIMSFBdialog)
  {
    if(ActiveDialog->Changed)
    {
      *FAIMSFB = faimsfb;
      FAIMSFB->ElectPosOffset = faimsfbvars->PosOffset;
      FAIMSFB->ElectNegOffset = faimsfbvars->NegOffset;
      ActiveDialog->Changed = false;
    }
    // If the mode has changed then update the display with the new mode
    faimsfb=*FAIMSFB;
    if(CurrentFAIMSFBmodule != FAIMSFBmodule)
    {
      SelectFAIMSFBmodule();
      CurrentFAIMSFBmodule = FAIMSFBmodule;
    }
  }
  // Process each FAIMSfb board. Look for changes in parameters and update as needed
  for(int b = 0; b < 2; b++)
  {
    if(FAIMSFBarray[b] != NULL)
    {
     SelectBoard(b);
     if(FAIMSFBrb[b] != NULL)
      {
        // Update the readback structure
        TWIreadBlock(FAIMSFBarray[b]->TWIadd | 0x20, b,TWI_FB_READ_READBACKS, (void *)FAIMSFBrb[b], sizeof(ReadBacks));
        if(b==SelectedFAIMSFBboard)
        {
           faimsfbvars->CVrb = FAIMSFBrb[b]->CV;
           faimsfbvars->VRFrb = FAIMSFBrb[b]->Vrf;
           faimsfbvars->PWR = FAIMSFBrb[b]->V * FAIMSFBrb[b]->I / 1000;
        }
        // If this is module 1 and electrometer is enabled then process electrometer changes
        if((b==FAIMSfbModule2Brd(1,false)) && (FAIMSFBarray[b]->ElectrometerEnable))
        {
           faimsfbvars->PosCurrent = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,4), &FAIMSFBarray[b]->ElectPosCtrl);
           faimsfbvars->NegCurrent = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,4), &FAIMSFBarray[b]->ElectNegCtrl);
           if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->PosOffset != faimsfbvars->PosOffset)) AD5593writeDACWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosOffsetCtrl.Chan, Value2Counts(FAIMSFBstates[b]->PosOffset = faimsfbvars->PosOffset,&FAIMSFBarray[b]->ElectPosOffsetCtrl));
           if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->NegOffset != faimsfbvars->NegOffset)) AD5593writeDACWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegOffsetCtrl.Chan, Value2Counts(FAIMSFBstates[b]->NegOffset = faimsfbvars->NegOffset,&FAIMSFBarray[b]->ElectNegOffsetCtrl));
        }
      }
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Enable) != FAIMSFBarray[b]->Enable)     TWIsetBool(FAIMSFBarray[b]->TWIadd  | 0x20, b, TWI_FB_SET_ENABLE, FAIMSFBstates[b]->Enable = FAIMSFBarray[b]->Enable);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Mode) != FAIMSFBarray[b]->Mode)         TWIsetBool(FAIMSFBarray[b]->TWIadd  | 0x20, b, TWI_FB_SET_MODE, FAIMSFBstates[b]->Mode = FAIMSFBarray[b]->Mode);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Drive) != FAIMSFBarray[b]->Drive)       TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_DRIVE, FAIMSFBstates[b]->Drive = FAIMSFBarray[b]->Drive);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->CV) != FAIMSFBarray[b]->CV)             TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_CV, FAIMSFBstates[b]->CV = FAIMSFBarray[b]->CV);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Vrf) != FAIMSFBarray[b]->Vrf)           TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_VRF, FAIMSFBstates[b]->Vrf = FAIMSFBarray[b]->Vrf);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Freq) != FAIMSFBarray[b]->Freq)         TWIsetInt(FAIMSFBarray[b]->TWIadd   | 0x20, b, TWI_FB_SET_FREQ, FAIMSFBstates[b]->Freq = FAIMSFBarray[b]->Freq);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Duty) != FAIMSFBarray[b]->Duty)         TWIsetByte(FAIMSFBarray[b]->TWIadd  | 0x20, b, TWI_FB_SET_DUTY, FAIMSFBstates[b]->Duty = FAIMSFBarray[b]->Duty);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->MaxDrive) != FAIMSFBarray[b]->MaxDrive) TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_MAXDRV, FAIMSFBstates[b]->MaxDrive = FAIMSFBarray[b]->MaxDrive);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->MaxPower) != FAIMSFBarray[b]->MaxPower) TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_MAXPWR, FAIMSFBstates[b]->MaxPower = FAIMSFBarray[b]->MaxPower);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->CVstart) != FAIMSFBarray[b]->CVstart)   TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_CV_START, FAIMSFBstates[b]->CVstart = FAIMSFBarray[b]->CVstart);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->CVend) != FAIMSFBarray[b]->CVend)       TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_CV_END, FAIMSFBstates[b]->CVend = FAIMSFBarray[b]->CVend);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->VRFstart) != FAIMSFBarray[b]->VRFstart) TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_VRF_START, FAIMSFBstates[b]->VRFstart = FAIMSFBarray[b]->VRFstart);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->VRFend) != FAIMSFBarray[b]->VRFend)     TWIsetFloat(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_VRF_END, FAIMSFBstates[b]->VRFend = FAIMSFBarray[b]->VRFend);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->Steps) != FAIMSFBarray[b]->Steps)       TWIsetInt(FAIMSFBarray[b]->TWIadd   | 0x20, b, TWI_FB_SET_STEPS, FAIMSFBstates[b]->Steps = FAIMSFBarray[b]->Steps);
      if(FAIMSFBstates[b]->update || (FAIMSFBstates[b]->StepDuration) != FAIMSFBarray[b]->StepDuration) TWIsetInt(FAIMSFBarray[b]->TWIadd | 0x20, b, TWI_FB_SET_DURATION, FAIMSFBstates[b]->StepDuration = FAIMSFBarray[b]->StepDuration);
      // Readback Drive, CV and Vrf in case the module changed the values
      if(TWIreadFloat(FAIMSFBarray[b]->TWIadd | 0x20, b,TWI_FB_READ_DRIVE, &fval)) FAIMSFBstates[b]->Drive = FAIMSFBarray[b]->Drive = fval;
      if(TWIreadFloat(FAIMSFBarray[b]->TWIadd | 0x20, b,TWI_FB_READ_VRF, &fval)) FAIMSFBstates[b]->Vrf = FAIMSFBarray[b]->Vrf = fval;
      if(TWIreadFloat(FAIMSFBarray[b]->TWIadd | 0x20, b,TWI_FB_READ_CV, &fval)) FAIMSFBstates[b]->CV = FAIMSFBarray[b]->CV = fval;
      FAIMSFBstates[b]->update = false;
    }
  }
  SelectBoard(SelectedFAIMSFBboard);
  if (ActiveDialog == &FAIMSFBdialog) RefreshAllDialogEntries(&FAIMSFBdialog);
  faimsfb = *FAIMSFB;
}

//
// Scanning functions and variables;
//
typedef struct
{
  bool      Updated;
  uint32_t  StartTime;
  uint32_t  TimeStamp;
  int       Point;
  int       steps;
  int       brd1;
  float     PosCurrent;
  float     NegCurrent;
  int       brd;          // Board index for scan parameters
  ScanPoint sp[2];        // Scan point struct for each module
} MIPSscanData;

volatile MIPSscanData  *msd = NULL;

// Scanning ISR
void FAIMSFBscanISR(void)
{
  msd->TimeStamp = millis();
  msd->Point++;
  int b = FAIMSfbModule2Brd(1);  // always use module 1 for electrometer settings
  // Read the electrometer here if its enabled
  if(FAIMSFBarray[b]->ElectrometerEnable)
  {
     msd->PosCurrent = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,4), &FAIMSFBarray[b]->ElectPosCtrl);
     msd->NegCurrent = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,4), &FAIMSFBarray[b]->ElectNegCtrl);
  }
  // Advance to next scan point
  digitalWrite(MIPSscanAdv,HIGH);
  digitalWrite(MIPSscanAdv,LOW);
  // Set flag that data is ready
  msd->Updated = true;
}

// This function starts a scan on the FAIMSFB modules. The module
// number can be 1 or 2 for module number 1 or 2 or if 3 is entered
// all modules found will be scanned. When 3 is entered the first module
// scan duration is used and the largest steps value is use.
//
// The modules are configured to use the external scan advance signal. The
// modules are also configured to return no report.
void InitFBScan(int module)
{
  int brd;
  int num;

  FAIMSfbScanClock->stop();
  if(msd == NULL) msd = new MIPSscanData;
  msd->Updated = false;
  msd->Point = 0;
  msd->steps = 0;
  // Set the scan advance pin to output and set low
  pinMode(MIPSscanAdv,OUTPUT);
  digitalWrite(MIPSscanAdv,LOW);
  for(int i=1;i<=2;i++)
  {
    if((brd=FAIMSfbModule2Brd(module & i, false)) != -1)
    {
      if(i==1) msd->brd1 = brd;
      if(FAIMSFBarray[brd]->Steps > msd->steps) msd->steps = FAIMSFBarray[brd]->Steps;
      // Setup FAIMS for external scan trigger
      TWIsetBool(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_EXTSTEP, true);
      TWIsetBool(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_SCNRPT, false);
      TWIcmd(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_STEPSTR);
    }
  }
  msd->StartTime = millis();
  // Setup the timer ISR
  FAIMSfbScanClock->attachInterrupt(FAIMSFBscanISR);
  FAIMSfbScanClock->setPeriod(FAIMSFBarray[msd->brd1]->StepDuration * 1000,8);
  // Start the timer
  FAIMSfbScanClock->start(-1, 8, false);
}

void ReportFBScan(void)
{
  // Wait for scan data and report
  while(1)
  {
    WDT_Restart(WDT);
    if(msd->Updated)
    {
       msd->Updated = false;
       serial->print(msd->Point); serial->print(",");
       serial->print(msd->TimeStamp - msd->StartTime);
       // Read the electrometer here if its enabled
       if(FAIMSFBarray[msd->brd1]->ElectrometerEnable)
       {
          serial->print(","); serial->print(msd->PosCurrent);
          serial->print(","); serial->print(msd->NegCurrent);
       }
       serial->println("");
       if(msd->Point >= msd->steps) 
       {
          FAIMSfbScanClock->stop();
          return;
       }
    }
  }
}

// Stops a scan in progress
void StopFBScan(int module)
{
  int brd;
    
  FAIMSfbScanClock->stop();
  for(int i=1;i<=2;i)
  {
    if((brd=FAIMSfbModule2Brd(module & i, false)) != -1)
    {
       if(FAIMSFBarray[brd] != NULL) TWIcmd(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_STEPSTP);
    }
  }
}

//
// Host FAIMSfb commands
//
// This function converts the module number into a board index, this board index
// is also the index into the modules data structure. There are two overloaded versions
// to accept the module as a string or an int
//int FAIMSfbModule2Brd(int module, bool response);
int FAIMSfbModule2Brd(char *module, bool response)
{
  String sToken;
  int    brd;

  sToken = module;
  brd = sToken.toInt();
  return(FAIMSfbModule2Brd(brd,response));
}
int FAIMSfbModule2Brd(int module, bool response)
{
  int brd = module-1;
  if((brd == 0) || (brd == 1))
  {
    if((brd == 0) && (FAIMSFBarray[0] != NULL)) return(0);
    if((brd == 0) && (FAIMSFBarray[1] != NULL)) return(1);
    if((brd == 1) && (FAIMSFBarray[1] != NULL)) return(1);
  }
  if(!response) return(-1);
  SetErrorCode(ERR_BADARG); SendNAK; return(-1);
}
// This function tests the int value to make sure its with in a valid range
bool IntCheck(int ival, int minVal, int maxVal)
{
  if((ival < minVal) || (ival > maxVal)) {SetErrorCode(ERR_BADARG); SendNAK; return(false);}
  return(true);
}
// This function convertes the string pointer into a float and tests its range
bool FloatCheck(char *fstr, float *rfloat,float minVal, float maxVal)
{
  String sToken;

   sToken = fstr;
   *rfloat = sToken.toFloat();
   if((*rfloat < minVal) || (*rfloat > maxVal)) {SetErrorCode(ERR_BADARG); SendNAK; return(false);}
   return(true);
}
// This is a generic function to print one of two values based on the module number,
// two overloaded function are provided, one for int vales and one for floats.
void ReturnFAIMSfb(int module,float fval1,float fval2)
{
  int brd;
  
  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(brd == 0) serial->println(fval1);  
  else serial->println(fval2);  
}
void ReturnFAIMSfb(int module,int ival1,int ival2)
{
  int brd;
  
  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(brd == 0) serial->println(ival1);  
  else  serial->println(ival2);  
}
// Generic functions to update one of two values based on the module input, two overloaded
// functions are provided one for floats and one for ints.
void SetFAIMSfb(char *module,char *val,float *fdst1,float *fdst2,float minVal, float maxVal)
{
  int   brd;
  float fval;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(!FloatCheck(val,&fval,minVal,maxVal)) return;
  if(brd == 0) *fdst1 = fval;
  else *fdst2 = fval;
  SendACK;
}
void SetFAIMSfb(int module,int ival,int *idst1,int *idst2,int minVal, int maxVal)
{
  int   brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(!IntCheck(ival,minVal,maxVal)) return;
  if(brd == 0) *idst1 = ival;
  else *idst2 = ival;
  SendACK;
}
//
// The following are function called by the command processor and use the generic
// functions defined above.
//
void SetFAIMSfbEnable(char *module, char *ena)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(strcmp(ena,"TRUE") == 0) FAIMSFBarray[brd]->Enable = true;
  else if(strcmp(ena,"FALSE") == 0) FAIMSFBarray[brd]->Enable = false;
  else { SetErrorCode(ERR_BADARG); SendNAK; return;}
  SendACK;
}
void ReturnFAIMSfbEnable(int module)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(FAIMSFBarray[brd]->Enable) serial->println("TRUE");
  else serial->println("FALSE");
}
void SetFAIMSfbMode(char *module, char *mode)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(strcmp(mode,"TRUE") == 0) FAIMSFBarray[brd]->Mode = true;
  else if(strcmp(mode,"FALSE") == 0) FAIMSFBarray[brd]->Mode = false;
  else { SetErrorCode(ERR_BADARG); SendNAK; return;}
  SendACK;  
}
void ReturnFAIMSfbMode(int module)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(FAIMSFBarray[brd]->Mode) serial->println("TRUE");
  else serial->println("FALSE");  
}
void SetFAIMSfbFreq(int module, int freq)
{
  SetFAIMSfb(module,freq,&FAIMSFBarray[0]->Freq,&FAIMSFBarray[1]->Freq,250000,2000000);
}
void ReturnFAIMSfbFreq(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Freq,FAIMSFBarray[1]->Freq);
}
void SetFAIMSfbDuty(int module, int duty)
{
  SetFAIMSfb(module,duty,&FAIMSFBarray[0]->Duty,&FAIMSFBarray[1]->Duty,10,90);
}
void ReturnFAIMSfbDuty(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Duty,FAIMSFBarray[1]->Duty);
}
void SetFAIMSfbDrive(char *module, char *drive)
{
  SetFAIMSfb(module,drive,&FAIMSFBarray[0]->Drive,&FAIMSFBarray[1]->Drive,0, 100);      
}
void ReturnFAIMSfbDrive(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Drive,FAIMSFBarray[1]->Drive);
}
void ReturnFAIMSfbDriveV(int module)
{
  ReturnFAIMSfb(module,FAIMSFBrb[0]->V,FAIMSFBrb[1]->V);
}
void ReturnFAIMSfbDriveI(int module)
{
  ReturnFAIMSfb(module,FAIMSFBrb[0]->I,FAIMSFBrb[1]->I);
}
void SetFAIMSfbVrf(char *module, char *Vrf)
{
   SetFAIMSfb(module,Vrf,&FAIMSFBarray[0]->Vrf,&FAIMSFBarray[1]->Vrf,100, 2000);      
}
void ReturnFAIMSfbVrf(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Vrf,FAIMSFBarray[1]->Vrf);
}
void ReturnFAIMSfbVrfV(int module)
{
  ReturnFAIMSfb(module,FAIMSFBrb[0]->Vrf,FAIMSFBrb[1]->Vrf);  
}
void ReturnFAIMSfbPWR(int module)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  float pwr = FAIMSFBrb[brd]->V * FAIMSFBrb[brd]->I / 1000.0;
  serial->println(pwr);         
}
void SetFAIMSfbMaxDrive(char *module, char *MaxDrv)
{
   SetFAIMSfb(module,MaxDrv,&FAIMSFBarray[0]->MaxDrive,&FAIMSFBarray[1]->MaxDrive,0, 100);      
}
void ReturnFAIMSfbMaxDrive(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->MaxDrive,FAIMSFBarray[1]->MaxDrive);
}
void SetFAIMSfbMaxPower(char *module, char *MaxPwr)
{
   SetFAIMSfb(module,MaxPwr,&FAIMSFBarray[0]->MaxPower,&FAIMSFBarray[1]->MaxPower,0, 40);      
}
void ReturnFAIMSfbMaxPower(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->MaxPower,FAIMSFBarray[1]->MaxPower);
}
void SetFAIMSfbCV(char *module, char *CV)
{
   SetFAIMSfb(module,CV,&FAIMSFBarray[0]->CV,&FAIMSFBarray[1]->CV,-48, 48);      
}
void ReturnFAIMSfbCV(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->CV,FAIMSFBarray[1]->CV);
}
void ReturnFAIMSfbCVrb(int module)
{
  ReturnFAIMSfb(module,FAIMSFBrb[0]->CV,FAIMSFBrb[1]->CV);
}
void SetFAIMSfbBIAS(char *module, char *BIAS)
{
   SetFAIMSfb(module,BIAS,&FAIMSFBarray[0]->Bias,&FAIMSFBarray[1]->Bias,-48, 48);      
}
void ReturnFAIMSfbBIAS(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Bias,FAIMSFBarray[1]->Bias);
}
void ReturnFAIMSfbBIASrb(int module)
{
  ReturnFAIMSfb(module,FAIMSFBrb[0]->BIAS,FAIMSFBrb[1]->BIAS);
}
void SetFAIMSfbCVstart(char *module, char *CVstart)
{
   SetFAIMSfb(module,CVstart,&FAIMSFBarray[0]->CVstart,&FAIMSFBarray[1]->CVstart,-48, 48);      
}
void ReturnFAIMSfbCVstart(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->CVstart,FAIMSFBarray[1]->CVstart); 
}
void SetFAIMSfbCVend(char *module, char *CVend)
{
   SetFAIMSfb(module,CVend,&FAIMSFBarray[0]->CVend,&FAIMSFBarray[1]->CVend,-48, 48);    
}
void ReturnFAIMSfbCVend(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->CVend,FAIMSFBarray[1]->CVend);   
}
void SetFAIMSfbVRFstart(char *module, char *VRFstart)
{
   SetFAIMSfb(module,VRFstart,&FAIMSFBarray[0]->VRFstart,&FAIMSFBarray[1]->VRFstart,100, 2000);  
}
void ReturnFAIMSfbVRFstart (int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->VRFstart,FAIMSFBarray[1]->VRFstart);   
}
void SetFAIMSfbVRFend(char *module, char *VRFend)
{
   SetFAIMSfb(module,VRFend,&FAIMSFBarray[0]->VRFend,&FAIMSFBarray[1]->VRFend,100, 2000);
}
void ReturnFAIMSfbVRFend(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->VRFend,FAIMSFBarray[1]->VRFend);     
}
void SetFAIMSfbStepDuration(int module, int StpDur)
{
  SetFAIMSfb(module,StpDur,&FAIMSFBarray[0]->StepDuration,&FAIMSFBarray[1]->StepDuration,1,1000);
}
void ReturnFAIMSfbStepDuration(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->StepDuration,FAIMSFBarray[1]->StepDuration);
}
void SetFAIMSfbSteps(int module, int Steps)
{
  SetFAIMSfb(module,Steps,&FAIMSFBarray[0]->Steps,&FAIMSFBarray[1]->Steps,10,10000);  
}
void ReturnFAIMSfbSteps(int module)
{
  ReturnFAIMSfb(module,FAIMSFBarray[0]->Steps,FAIMSFBarray[1]->Steps);       
}
void SetFAIMSfbVrfNow(char *module, char *Vrf)
{
  int   brd;
  float fval;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(!FloatCheck(Vrf,&fval,100,2000)) return;
  FAIMSFBarray[brd]->Vrf = fval;
  SendACK;
  // Send command to set the Vrf level
  TWIsetFloat(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_VRF_NOW, FAIMSFBstates[brd]->Vrf = FAIMSFBarray[brd]->Vrf);
}
void SetFAIMSfbVrfTable(char *module, char *Vrf)
{
  int   brd;
  float fval;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  if(!FloatCheck(Vrf,&fval,100,2000)) return;
  FAIMSFBarray[brd]->Vrf = fval;
  SendACK;
  // Send command to set the Vrf level
  TWIsetFloat(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_VRF_TABLE, FAIMSFBstates[brd]->Vrf = FAIMSFBarray[brd]->Vrf);  
}
void GenerateVrfTable(int module)
{
  int   brd;

  if((brd=FAIMSfbModule2Brd(module)) == -1) return;
  SendACK;
  TWIcmd(FAIMSFBarray[brd]->TWIadd | 0x20, brd, TWI_FB_SET_CAL);
}

// This function starts a scan on the FAIMSFB modules. The module
// number can be 1 or 2 for module number 1 or 2 or if 3 is entered
// all modules found will be scanned. When 3 is entered the first module
// scan duration is used and the largest steps value is use.
//
// The modules are configured to use the external scan advance signal. The
// modules are also configured to return no report.
void InitFAIMSfbScan(int module)
{
   InitFBScan(module);
   SendACK;
   delay(10);
   ReportFBScan();
}

// Stops a scan in progress
void StopFAIMSfbScan(int module)
{
   StopFBScan(module);
   SendACK;
}

//
// Electrometer interface routines. This electrometer connects to J1 on the MIPS controller and used the
// wire1 TWI interface on the ARM processor.
//

// Init the AD5593 (Analog and digital IO chip) for the Electrometer module. The following 
// setup requirements:
// CH0 = ADC in, positive output from electrometer
// CH1 = ADC in, negative output from electrometer
// CH2 = DAC out, positive zero set
// CH3 = DAC out, negative zero set
// CH4 = DAC out, positive offset
// CH5 = DAC out, negative offset
// Internal 2.5V reference with 0 to 2.5V range
// No pullups
void ElectrometerAD5593init(int8_t addr)
{
   Wire1.begin();
   Wire1.setClock(400000);

   MIPSconfigData.Ser1ena = false;
   pinMode(18, INPUT);
   pinMode(19, INPUT);
   // General purpose configuration, set range for 5 volts
   AD5593writeWire1(addr, 3, 0x0130);
   // Set ext reference
   AD5593writeWire1(addr, 11, 0x0200);
   // Set LDAC mode
   AD5593writeWire1(addr, 7, 0x0000);
   // Set DAC outputs channels
   AD5593writeWire1(addr, 5, 0x003C);
   // Set ADC input channels
   AD5593writeWire1(addr, 4, 0x0003);
   // Turn off all pulldowns
   AD5593writeWire1(addr, 6, 0x0000);

   // Set all DACs to zero
   AD5593writeDACWire1(addr, 2, 0);
   AD5593writeDACWire1(addr, 3, 0);
   AD5593writeDACWire1(addr, 4, 0);
   AD5593writeDACWire1(addr, 5, 0); 
}

// Adjusts the zero DAC output volts to set the electrometer near zero.
// Note that electrometer is clamped at zero so the algorith sets to near 
// zero. 
void ZeroElectrometer(void)
{
  int i;
  float Ival;
  int   b;
    
  // Determing the board number that has an electrometer enabled
  if(FAIMSFBarray[1] != NULL) if(FAIMSFBarray[1]->ElectrometerEnable) b = 1;
  if(FAIMSFBarray[0] != NULL) if(FAIMSFBarray[0]->ElectrometerEnable) b = 0;
  // Read the positive channel current, average several readings and
  // adjust the zero channel to set to 1 to 5 range
  for(i=0;i<25;i++)
  {
     Ival = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,10), &FAIMSFBarray[b]->ElectPosCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,10), &FAIMSFBarray[b]->ElectPosCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,10), &FAIMSFBarray[b]->ElectPosCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosCtrl.Chan,10), &FAIMSFBarray[b]->ElectPosCtrl);
     Ival /= 4;
     //serial->println(Ival);
     if((Ival > 2) && (Ival < 10)) break;
     // Adjust zero voltage
     //serial->print("Zero val :");
     //serial->println(FAIMSFBarray[b]->ElectPosZero);
     FAIMSFBarray[b]->ElectPosZero += (Ival - 7) * - 0.01;
     if(FAIMSFBarray[b]->ElectPosZero < 0.0) FAIMSFBarray[b]->ElectPosZero = 0.0;
     if(FAIMSFBarray[b]->ElectPosZero > 5.0) FAIMSFBarray[b]->ElectPosZero = 5.0;
     AD5593writeDACWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectPosZeroCtrl.Chan, Value2Counts(FAIMSFBarray[b]->ElectPosZero,&FAIMSFBarray[b]->ElectPosZeroCtrl));
     delay(25);    
     WDT_Restart(WDT);
  }
 // Read the negative channel current, average several readings and
 // adjust the zero channel to set to 1 to 5 range
  for(i=0;i<25;i++)
  {
     Ival = Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,10), &FAIMSFBarray[b]->ElectNegCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,10), &FAIMSFBarray[b]->ElectNegCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,10), &FAIMSFBarray[b]->ElectNegCtrl);
     Ival += Counts2Value(AD5593readADCWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegCtrl.Chan,10), &FAIMSFBarray[b]->ElectNegCtrl);
     Ival /= 4;
     //serial->println(Ival);
     if((Ival > 2) && (Ival < 10)) break;
     // Adjust zero voltage
     //serial->print("Zero val :");
     //serial->println(FAIMSFBarray[b]->ElectPosZero);
     FAIMSFBarray[b]->ElectNegZero += (Ival - 7) * -0.01;
     if(FAIMSFBarray[b]->ElectNegZero < 0.0) FAIMSFBarray[b]->ElectNegZero = 0.0;
     if(FAIMSFBarray[b]->ElectNegZero > 5.0) FAIMSFBarray[b]->ElectNegZero = 5.0;
     AD5593writeDACWire1(FAIMSFBarray[b]->ElectAdd, FAIMSFBarray[b]->ElectNegZeroCtrl.Chan, Value2Counts(FAIMSFBarray[b]->ElectNegZero,&FAIMSFBarray[b]->ElectNegZeroCtrl));
     delay(25);    
     WDT_Restart(WDT);
  }  
}

//
// Electrometer host commands
//
// ELTMTRZERO
int isElectrometer(void)
{
  int brd;
  
  if((brd=FAIMSfbModule2Brd(1)) == -1) return(-1);
  if(!FAIMSFBarray[brd]->ElectrometerEnable)
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return(-1);
  }
  return(brd);
}
void SetEMRTenable(char *ena)
{
  int brd;

  // Get module 1's data structure and set flag
  if((brd=FAIMSfbModule2Brd(1)) == -1) return;
  if(strcmp(ena,"TRUE") == 0) 
  {
    ElectrometerAD5593init(FAIMSFBarray[brd]->ElectAdd);
    FAIMSFBarray[brd]->ElectrometerEnable = true;
  }
  else if(strcmp(ena,"FALSE") == 0) FAIMSFBarray[brd]->ElectrometerEnable = false;
  else { SetErrorCode(ERR_BADARG); SendNAK; return;}
  faimsfb.ElectrometerEnable = FAIMSFBarray[brd]->ElectrometerEnable;
  SendACK;
}
void ReturnEMRTenable(void)
{
  int brd;

  if((brd=FAIMSfbModule2Brd(1)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(FAIMSFBarray[brd]->ElectrometerEnable) serial->println("TRUE");
  else serial->println("FALSE");
}
void ReturnEMRTpos(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(faimsfbvars->PosCurrent,4);  
}
void ReturnEMRTneg(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(faimsfbvars->NegCurrent,4); 
}
void SetEMRTposOff(char *val)
{
    if(isElectrometer() == -1) return;
    SetFAIMSfb("1",val,&faimsfbvars->PosOffset,&faimsfbvars->PosOffset,0,5.0);
}
void ReturnEMRTposOff(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(faimsfbvars->PosOffset,4); 
}
void SetEMRTnegOff(char *val)
{
    if(isElectrometer() == -1) return;
    SetFAIMSfb("1",val,&faimsfbvars->NegOffset,&faimsfbvars->NegOffset,0,5.0);
}
void ReturnEMRTnegOff(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(faimsfbvars->NegOffset,4); 
}

void SetEMRTposZero(char *val)
{
    int brd;
    
    if((brd=isElectrometer()) == -1) return;
    SetFAIMSfb("1",val,&FAIMSFBarray[brd]->ElectPosZero,&FAIMSFBarray[brd]->ElectPosZero,0,5.0);
    AD5593writeDACWire1(FAIMSFBarray[brd]->ElectAdd, FAIMSFBarray[brd]->ElectPosZeroCtrl.Chan, Value2Counts(FAIMSFBarray[brd]->ElectPosZero,&FAIMSFBarray[brd]->ElectPosZeroCtrl));
}
void ReturnEMRTposZero(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(FAIMSFBarray[brd]->ElectPosZero,4); 
}
void SetEMRTnegZero(char *val)
{
    int brd;
    
    if((brd=isElectrometer()) == -1) return;
    SetFAIMSfb("1",val,&FAIMSFBarray[brd]->ElectNegZero,&FAIMSFBarray[brd]->ElectNegZero,0,5.0);
    AD5593writeDACWire1(FAIMSFBarray[brd]->ElectAdd, FAIMSFBarray[brd]->ElectNegZeroCtrl.Chan, Value2Counts(FAIMSFBarray[brd]->ElectNegZero,&FAIMSFBarray[brd]->ElectNegZeroCtrl));
}
void ReturnEMRTnegZero(void)
{
  int brd;

  if((brd=isElectrometer()) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(FAIMSFBarray[brd]->ElectNegZero,4); 
}


void SetEMRTzero(void)
{
  SendACK;
  ZeroElectrometer();
}

#endif
