#if RFdriver2

RFdriverData *RFDDarray[4] = {NULL,NULL,NULL,NULL};
int  SelectedRFChan  = 0;    // Active channel
int  SelectedRFBoard = 0;    // Active board, 0 or 1 = A or B

RFDRVstate   *RFstate[4][2] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
RFreadBacks  *RFrb[4][2]    = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};

DialogBoxEntry RFdriverDialogEntriesPage1[] = {
  {" RF channel", 0, 1, D_INT, 1, 2, 1, 21, false, "%2d", &Channel, NULL, SelectChannel},
  {" Freq, Hz"  , 0, 2, D_INT, 500000, 5000000, 1000, 16, false, "%7d", &RFCD.Freq, NULL, NULL},
  {" Drive %"   , 0, 3, D_FLOAT, 0, 100, 0.1, 18, false, "%5.1f", &RFCD.DriveLevel, NULL, NULL},
  {" Vout Vpp"  , 0, 3, D_OFF, 0, 5000, 1, 18, false, "%5.0f", &RFCD.Setpoint, NULL, NULL},
  {" RF+ Vpp"   , 0, 5, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFpVpp, NULL, NULL},
  {" RF- Vpp"   , 0, 6, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.0f", &RFnVpp, NULL, NULL},
  {" Power, W"  , 0, 7, D_FLOAT, 0, 1000, 0.1, 18, true, "%5.1f", &Power, NULL, NULL},
  {" Next page" , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, RFdriverDialogEntriesPage2, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

char *ModeList = "MANUAL,AUTO";
char RFmode[8] = "MANUAL";

DialogBoxEntry RFdriverDialogEntriesPage2[] = {
  {" Drive limit, %", 0, 1, D_FLOAT, 0, 100, 1, 18, false, "%5.1f", &RFCD.MaxDrive, NULL, NULL},
  {" Power limit, W", 0, 2, D_FLOAT, 0, 50, 1, 18, false, "%5.1f", &RFCD.MaxPower, NULL, NULL},
  {" Mode"          , 0, 3, D_LIST, 0, 0, 7, 16, false, ModeList, RFmode, NULL, RFmodeChange},
  {" Gate input"    , 0, 4, D_DI, 0, 0, 5, 18, false, DIlist, RFgateDI, NULL, NULL},
  {" Gate level"    , 0, 5, D_DILEVEL, 0, 0, 5, 18, false, DILlist, RFgateTrig, NULL, NULL},
  {" Auto tune"     , 0, 6, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFat, NULL},
  {" Auto retune"   , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RFart, NULL},
  {" Save settings" , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveRFdriverSettings, NULL},
  {" Restore settings", 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreRFdriverSettings, NULL},
  {" First page"         , 0,10, D_PAGE, 0, 0, 0, 0, false, NULL, RFdriverDialogEntriesPage1, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox RFdriverDialog = {
  {"RF driver parameters", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0,0,false, RFdriverDialogEntriesPage1
};

MenuEntry MERFdriverMonitor = {" RF driver module", M_DIALOG, 0, 0, 0, NULL, &RFdriverDialog, NULL, NULL};

// This function uses the selected channel to determine the board number, 0 or 1.
// Returns -1 if error condition is detected
int8_t BoardFromSelectedChannel(int8_t SC)
{
  // if selected channel is 0 or 1 then find the first avalible board and return it
  if (SC <= 1)
  {
    if (RFDDarray[0] != NULL) return (0);
    if (RFDDarray[1] != NULL) return (1);
    return (-1);
  }
  if(SC > 7) return(-1);
  if(RFDDarray[SC << 1] == NULL) return(-1);
  return(SC << 1);
}

// This function is called at powerup to initiaize the RF driver.
// The board parameter (0 or 1) defines the board select parameter where this card was found.
// Up to two boards are supported. Board A, or 0, is always called first.
// If only one board is installed it can be board 0 or 1.
void RFdriver_init(int8_t Board, int8_t addr)
{
  DialogBox *sd;

  if(NumberOfRFChannels >= 4) Board += 2;
  // Allocate the RFdriver structures
  RFDDarray[Board]  = new RFdriverData;
  RFstate[Board][0] = new RFDRVstate;
  RFstate[Board][1] = new RFDRVstate;
  RFstate[Board][0]->update = RFstate[Board][0]->update = false;
  RFrb[Board][0]    = new RFreadBacks;
  RFrb[Board][1]    = new RFreadBacks;
  // Create the digital input gate objects
  DIh[Board][0] = new DIhandler;
  DIh[Board][1] = new DIhandler;
  // Set active board to board being inited
  SelectedRFBoard = Board;
  SelectBoard(Board);
  // If normal startup load the EEPROM parameters from the RF driver card.
  sd = ActiveDialog;
  ActiveDialog = NULL;
  if(NumberOfRFChannels > 0)
  {
    Channel = 3;
    SelectedRFChan = Channel - 1;
  }
  RFDDarray[Board].EEPROMadr = addr;
  if (NormalStartup) RestoreRFdriverSettings(true);
  // Define the initial selected channel as 0 and setup
  Channel = 1;
  SelectChannel();
  ActiveDialog = sd;
  // Setup the menu if this is the very first call to this init function
  if (NumberOfRFChannels == 0)
  {
    AddMainMenuEntry(&MERFdriverMonitor);
    if (ActiveDialog == NULL) DialogBoxDisplay(&RFdriverDialog);
    // Configure Threads
    RFdriverThread.setName("RFdriver");
    RFdriverThread.onRun(RFdriver_loop);
    RFdriverThread.setInterval(100);
    // Add threads to the controller
    control.add(&RFdriverThread);
  }
  NumberOfRFChannels += 2;  // Always add two channels for each board
  // Set the maximum number of channels in the selection menu
  RFdriverDialogEntriesPage1[0].Max = NumberOfRFChannels;
}

void RFdriver_loop(void)
{
  float fval;
  
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  if(ActiveDialog == &RFdriverDialog)
  {
    if(ActiveDialog->Changed) 
    {
      // This stores any changes back to the selected channels data structure
      RFDD.RFCD[SelectedRFChan & 1] = RFCD;    
      // Set the Drive level limit in the UI menu
      RFdriverDialogEntriesPage1[2].Max = RFCD.MaxDrive;
      ActiveDialog->Changed = false;
    }
  }
  // Process each RF driver board. Look for changes in parameters and update as needed
  for(int b = 0; b < 4; b++)
  {
    if(RFDDarray[b] != NULL)
    {
      SelectBoard(b);

      for(int c = 0; c < 2; c++)   // process each channel on the selected board
      {
        TWIsetByte(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_CHAN, c+1);  // Select this channel on the selected board
        // Update the readback structure
        if(RFrb[b] != NULL)
        {
          TWIreadBlock(RFDDarray[b]->EEPROMadr | 0x20, b,TWI_RF_READ_READBACKS, (void *)RFrb[b][c], sizeof(RFreadBacks));
          if(b == SelectedRFBoard) && (c == (SelectedRFChan & 1)))
          {
          }
        }
        if(RFstate[b][c]->update || (RFstate[b][c]->DriveLevel != RFDDarray[b].RFCD[c]->DriveLevel)) TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_DRIVE, RFstate[b][c]->DriveLevel = RFDDarray[b].RFCD[c]->DriveLevel);
        if(RFstate[b][c]->update || (RFstate[b][c]->Freq != RFDDarray[b].RFCD[c]->Freq)) TWIsetInt(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_FREQ, RFstate[b][c]->Freq = RFDDarray[b].RFCD[c]->Freq);
        if(RFstate[b][c]->update || (RFstate[b][c]->SetPoint != RFDDarray[b].RFCD[c]->Setpoint)) TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_VRF, RFstate[b][c]->Setpoint = RFDDarray[b].RFCD[c]->Setpoint);
        if(RFstate[b][c]->update || (RFstate[b][c]->MaxDrive != RFDDarray[b].RFCD[c]->MaxDrive)) TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_MAXDRV, RFstate[b][c]->MaxDrive = RFDDarray[b].RFCD[c]->MaxDrive);
        if(RFstate[b][c]->update || (RFstate[b][c]->MaxPower != RFDDarray[b].RFCD[c]->MaxPower)) TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_MAXPWR, RFstate[b][c]->MaxPower = RFDDarray[b].RFCD[c]->MaxPower);
        if(RFstate[b][c]->update || (RFstate[b][c]->RFmode != RFDDarray[b].RFCD[c]->RFmode))
        {
          if(RFmode == RF_AUTO) TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_MODE, true);
          else TWIsetFloat(RFDDarray[b]->EEPROMadr | 0x20, b, TWI_RF_SET_MODE, false);
          RFstate[b][c]->RFmode = RFDDarray[b].RFCD[c]->RFmode;
        }
        // Readback Drive in case the module changed the value
       if(TWIreadFloat(FAIMSFBarray[b]->TWIadd | 0x20, b,TWI_RF_READ_DRIVE, &fval)) RFstate[b][c]->DriveLevel = RFDDarray[b].RFCD[c]->DriveLevel = fval;
       RFstate[b][c]->update = false;      
      }
    }
  }
  // Reselect the active channel's board
  SelectedRFBoard = BoardFromSelectedChannel(SelectedRFChan);
  SelectBoard(SelectedRFBoard);
  // Read the ADC monitor values for the selected channel.
  RFpVpp = RFrb[SelectedRFBoard][SelectedRFChan & 1].RFP;
  RFnVpp = RFrb[SelectedRFBoard][SelectedRFChan & 1].RFN;
  Power = RFrb[SelectedRFBoard][SelectedRFChan & 1].PWR;
  if (ActiveDialog == &RFdriverDialog) RefreshAllDialogEntries(&RFdriverDialog);
  RFCD = RFDD.RFCD[SelectedRFChan & 1]; 
}

#endif
