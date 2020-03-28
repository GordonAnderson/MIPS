/*
 * WiFi
 *
 * This file contains the interface to an Adafruit ESP8266 Hazzah WiFi module. This module needs to run MIPSnet. This module
 * acts as a WiFi to serial repeater and is connected to the Due serial port. The serial communications speed is 115200 baud.
 * When connected to a network you can use telnet to connect to MIPS and issue any of the MIPS commands.
 *
 * Currently the only way to set the parameters is to use the host interface on the native USB connection.
 * 
 * To do list:
 *  1.) Add disconnect function, done
 *  2.) Only allow changes to parameters when disconnected
 *  3.) Fix the restore to disconnect then restore and connect if enabled
 *  4.) Add serial commands, some added
 *
 * Gordon Anderson
 * November 28, 2015
 *
 */
#include "WiFi.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

//MIPS Threads
Thread WiFiThread  = Thread();

WiFiData wifidata = WiFi_Rev_1;

Stream *WiFiSerial = &Serial1;

char *StartModeList = "IDLE,CONNECT,AP";
char STM[20] = "IDLE";

// From wl_status_t, 0 through 6
char *WiFiStatusList = "IDLE,NOssid,SCANED,CONNECTED,FAILED,LOST,DISCON,AP";
char WFS[20] = "IDLE";

extern DialogBoxEntry WiFiEntriesPage2[];

DialogBoxEntry WiFiEntriesPage1[] = {
  {" Start mode"           , 0, 1, D_LIST , 0, 1, 10, 13, false, StartModeList, STM, NULL, SetStartMode},
  {" Host"                 , 0, 2, D_STRING, 0, 15, 0, 8, true, "%15s", wifidata.Host, NULL, NULL},
  {" SSID"                 , 0, 3, D_STRING, 0, 15, 1, 8, true, "%15s", wifidata.ssid, NULL, NULL},
  {" Pswd"                 , 0, 4, D_STRING, 0, 15, 1, 8, true, "%15s", wifidata.password, NULL, NULL},
  {" IP"                   , 0, 5, D_STRING, 0, 15, 1, 8, true, "%15s", wifidata.IP, NULL, NULL},
  {" Status"               , 0, 6, D_LIST,   0, 0, 10, 13,  true, WiFiStatusList, WFS, NULL, NULL},
  {" Press to connect"     , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, Connect, NULL},
  {" Start as Access Point", 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, AccessPoint, NULL},
  {" Press to disconnect"  , 0, 9, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, Disconnect, NULL},
  {" Next page"            , 0,10, D_PAGE,  0, 0  , 0, 0,  false, NULL, WiFiEntriesPage2, NULL, NULL},
  {" Return to main menu"  , 0,11, D_MENU,  0, 0  , 0, 0,  false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry WiFiEntriesPage2[] = {
  {" Save settings"      , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveWiFiSettings, NULL},
  {" Restore settings"   , 0, 8, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestorWiFiSettings, NULL},
  {" First page"         , 0, 9, D_PAGE, 0, 0, 0, 0, false, NULL, WiFiEntriesPage1, NULL, NULL},
  {" Return to main menu", 0, 10, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox WiFiDialog = {
  {
    "WiFi control parameters",
    ILI9340_BLACK, ILI9340_WHITE,
    2, 0, 0, 300, 220, B_DOUBLE, 12
  },
  M_SCROLLING, 0, 0,false, WiFiEntriesPage1
};

MenuEntry MEWiFiModule = {" WiFi module", M_DIALOG, 0, 0, 0, NULL, &WiFiDialog, NULL, NULL};

// Sets the data structure when the user changes the selection
void SetStartMode(void)
{
  wifidata.WFstartMode = (WiFiStartMode)(FindInList(StartModeList, STM) - 1);
}

// WiFi setting are saved to a file oon the SD card. This file cotains the WiFi data structure contents
// at the time save was selected.
void SaveWiFiSettings(void)
{
  File file;
  
  // Test SD present flag, exit and NAK if no card or it failed to init
  if(!SDcardPresent)
  {
    DisplayMessage("No SD card found!", 2000);
    return;
  }
  SD.begin(_sdcs);
  // Remove the existing wifi.cfg file
  SD.remove("wifi.cfg");
  // Open file and write config structure to disk
  if(!(file=SD.open("wifi.cfg",FILE_WRITE)))
  {
    DisplayMessage("Unable to save!", 2000);
    return;
  }
  wifidata.Size = sizeof(WiFiData);
  file.write((byte *)&wifidata,sizeof(WiFiData));
  file.close();
  DisplayMessage("Parameters Saved!", 2000);
  return;
}

void RestorWiFiSettings(void)
{
  RestorWiFiSettings(true);
}

void RestorWiFiSettings(bool Display)
{
  File file;
  WiFiData wfd;
  int  i,fVal;
  byte *b;

  // Don't restore if we are connected
  if(strcmp(WFS,"CONNECTED")==0)
  {
    if(Display) DisplayMessage("Disconnect first!", 2000);
    return;
  }    
  // Test SD present flag
  if(!SDcardPresent)
  {
    if(Display) DisplayMessage("No SD card found!", 2000);
    return;
  }
  SD.begin(_sdcs);
  // Open the file
  if(!(file=SD.open("wifi.cfg",FILE_READ)))
  {
    if(Display) DisplayMessage("Unable to Restore!", 2000);
    return;
  }
  // read the data
  b = (byte *)&wfd;
  for(i=0;i<sizeof(WiFiData);i++)
  {
    if((fVal = file.read()) == -1) break;
    b[i] = fVal;
  }
  file.close();
  // Copy to MIPS config struct
  wfd.Status = wifidata.Status; 
  memcpy(&wifidata, &wfd, wfd.Size);
  strcpy(STM, GetEntry(StartModeList, wifidata.WFstartMode+1));
  if(Display) DisplayMessage("Parameters Restored!", 2000);
  strcpy(wifidata.IP,"");  // Clear IP, makes no sense to restore it!
  return;
}

// Break any existing WiFi connections
void Disconnect(void)
{
  String res;
  
  WiFiSerial->println("StoP");
  WiFiSerial->flush();
  delay(100);
  WiFiSerial->println("DISCONNECT");
  WiFiSerial->flush();
  delay(100);
  while (WiFiSerial->available()) WiFiSerial->read();
  WiFiSerial->println("RESET");
  WiFiSerial->flush();
  delay(500);
  while (WiFiSerial->available()) WiFiSerial->read();  
  WiFiSerial->println("STATUS");
  WiFiSerial->flush();
  delay(100);
  res = WiFiSerial->readStringUntil('\n');
  strcpy(WFS, GetEntry(WiFiStatusList, res.toInt() + 1));
  strcpy(wifidata.IP,"");
}

// The function will place the system in Access Point mode and allow connection to the interface
void AccessPoint(void)
{
  String res;

  DisplayMessage("Access Point setup");
  Disconnect();
  // Send host name
  WiFiSerial->print("HOST,");
  WiFiSerial->println(wifidata.Host);
  // Send SSID
  WiFiSerial->print("SSID,");
  WiFiSerial->println(wifidata.ssid);
  // Send password
  WiFiSerial->print("PSWD,");
  WiFiSerial->println(wifidata.password);
  delay(100);
  while (WiFiSerial->available()) WiFiSerial->read();
  WiFiSerial->flush();
  // Set AP mode
  WiFiSerial->println("AP");
  delay(2000);
  while (WiFiSerial->available()) WiFiSerial->read();
  WiFiSerial->flush();
  while (WiFiSerial->available()) WiFiSerial->read();
  // Get the IP address from the interface
  WiFiSerial->println("IP");
  res = WiFiSerial->readStringUntil('\n');
  strcpy(wifidata.IP, res.c_str());
  WiFiSerial->println("MDNS");
  delay(100);
  WiFiSerial->flush();
  WiFiSerial->println("REPEAT");
  while (WiFiSerial->available()) WiFiSerial->read();
  WiFiSerial->flush();
  delay(500);
  while (WiFiSerial->available()) WiFiSerial->read();
  strcpy(WFS, GetEntry(WiFiStatusList, 8));
  DismissMessage();
}

// This function tries to connect to the wireless network. Any current connection is first terminated.
void Connect(void)
{
  int i;
  String res;

  DisplayMessage("Connecting to WiFi");
  Disconnect();
  // Send host name
  WiFiSerial->print("HOST,");
  WiFiSerial->println(wifidata.Host);
  // Send SSID
  WiFiSerial->print("SSID,");
  WiFiSerial->println(wifidata.ssid);
  // Send password
  WiFiSerial->print("PSWD,");
  WiFiSerial->println(wifidata.password);
  WiFiSerial->flush();
  delay(100);
  // Connect
  WiFiSerial->println("CONNECT");
  WiFiSerial->flush();
  delay(1000);
  while (WiFiSerial->available()) WiFiSerial->read();
  // Loop looking for a connect indication. Try for a few seconds then give up
  for (i = 0; i < 20; i++)
  {
    WDT_Restart(WDT);
    while (WiFiSerial->available()) WiFiSerial->read();
    // Send get status command
    WiFiSerial->println("STATUS");
    WiFiSerial->flush();
    res = WiFiSerial->readStringUntil('\n');
    if (res.toInt() == 3)
    {
      strcpy(WFS, GetEntry(WiFiStatusList, res.toInt() + 1));
      // Get the IP address from the interface
      WiFiSerial->println("IP");
      res = WiFiSerial->readStringUntil('\n');
      strcpy(wifidata.IP, res.c_str());
      WiFiSerial->println("MDNS");
      WiFiSerial->flush();
      delay(100);
      while (WiFiSerial->available()) WiFiSerial->read();
      WiFiSerial->println("REPEAT");
      WiFiSerial->flush();
      delay(500);
      while (WiFiSerial->available()) WiFiSerial->read();
      DismissMessage();
      return;
    }
    delay(500);
  }
  strcpy(wifidata.IP, "");
  strcpy(WFS, GetEntry(WiFiStatusList, res.toInt() + 1));
  DismissMessage();
}

// This function is called at powerup to test for and initiaize the WiFi interface.
void WiFi_init(void)
{
  String res;

//SaveWiFiSettings();
  RestorWiFiSettings(false);
//  if(!wifidata.Enable) return;   // Exit if disabled

//MIPSconfigData.UseWiFi=true;
//wifidata.SerialPort = 1;
//if(MIPSconfigData.UseWiFi) && (wifidata.SerialPort==0)

  if(!MIPSconfigData.UseWiFi) return;
  // Setup ther serial port.
  if(wifidata.SerialPort == 1)
  {
     Serial1.begin(115200);  
     WiFiSerial = &Serial1;     
  }
  else
  {
     Serial.begin(115200);  
     WiFiSerial = &Serial;  
  }
  WiFiSerial->flush();
  // Issue a stop and disconnect command in case the module in already connected
  Disconnect();
  // Test for the module using the GVER command and exit if not found.
  WiFiSerial->println("GVER");
  WiFiSerial->flush();                             // Send all data
  delay(10);
  WiFiSerial->setTimeout(100);
  res = WiFiSerial->readStringUntil('\n');
  if (!res.startsWith("MIPSnet Version ")) 
  {
    DisplayMessage("WiFi setup ERROR!", 2000);
  }
//  RestorWiFiSettings(false);
  // If found and a SSID is defined try to connect and setup the interface, only if enabled
  if (wifidata.WFstartMode == WS_CONNECT) if (strlen(wifidata.ssid) > 0) Connect();
  if (wifidata.WFstartMode == WS_AP) if (strlen(wifidata.ssid) > 0) AccessPoint();
  // Setup module's task and add to main menu.
  AddMainMenuEntry(&MEWiFiModule);
  if (ActiveDialog == NULL) DialogBoxDisplay(&WiFiDialog);
  // Configure Threads
  WiFiThread.setName("WiFi");
  WiFiThread.onRun(WiFi_loop);
  WiFiThread.setInterval(100);
  // Add threads to the controller
  control.add(&WiFiThread);
}

void WiFi_loop(void)
{
  if (ActiveDialog->Entry == WiFiEntriesPage1) RefreshAllDialogEntries(&WiFiDialog);
}

/*
 * Serial command proccessing routines.
 */

// Returns true if WiFi is connected and send a NAK
bool isConnected(void)
{
  if(strcmp(WFS,"CONNECTED")==0)
  {
    SetErrorCode(ERR_WIFICONNECTED);
    SendNAK;
    return true;
  }  
  return false;
}

void SetHost(char *host)
{
  if(isConnected()) return;
  strcpy(wifidata.Host,host);
  SendACK;
}

void SetSSID(char *ssid)
{
  if(isConnected()) return;
  strcpy(wifidata.ssid,ssid);   
  SendACK;
}

void SetPassword(char *pswd)
{
  if(isConnected()) return;
  // If "none" is entered then the password is cleared
  if(strcmp(pswd,"none") == 0) wifidata.password[0]=0;
  else strcpy(wifidata.password,pswd);     
  SendACK;
}
