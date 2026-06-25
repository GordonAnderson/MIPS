/*
 * WiFi
 *
 * This file contains the interface to an Adafruit ESP8266 Hazzah WiFi module. This module needs to run MIPSnet. This module
 * acts as a WiFi to serial repeater and is connected to the Due serial port. The serial communications speed is 115200 baud.
 * When connected to a network you can use telnet to connect to MIPS and issue any of the MIPS commands.
 *
 * Adafruit HUZZAH ESP8266 Breakout, Product ID: 2471
 * 
 * Parameters are set using the host interface on the native USB connection.
 *
 * Gordon Anderson
 * Created, November 28, 2015
 *
 */
#include <Arduino.h>
#include "WiFi.h"
#include "Variants.h"
#include "Hardware.h"
#include "Errors.h"

#include <Thread.h>
#include <ThreadController.h>

extern Menu MainMenu;
extern bool NormalStartup;
extern bool SDcardPresent;
extern ThreadController control;

char *GetEntry(const char *list, int num);
void AddMainMenuEntry(MenuEntry *me);

// Forward declarations for functions used in static initializers
void SetStartMode(void);
void Connect(void);
void AccessPoint(void);
void Disconnect(void);
void SaveWiFiSettings(void);
void RestorWiFiSettings(void);
void RestorWiFiSettings(bool Display);
void WiFi_loop(void);

//MIPS Threads
Thread WiFiThread  = Thread();

WiFiData wifidata = WiFi_Rev_1;

Stream *WiFiSerial = &Serial1;

const char *StartModeList = "IDLE,CONNECT,AP";
char STM[20] = "IDLE";

// From wl_status_t, 0 through 6
const char *WiFiStatusList = "IDLE,NOssid,SCANED,CONNECTED,FAILED,LOST,DISCON,AP";
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

// The RestorWiFiSettings function restores Wi-Fi configuration settings from a 
// file named "wifi.cfg" on an SD card, provided that the device is not currently 
// connected to a Wi-Fi network and the SD card is present. If successful, 
// it updates the global Wi-Fi data structure and displays a message indicating 
// the restoration status.
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
  for(i=0;i<(int)sizeof(WiFiData);i++)
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

// The Disconnect function is responsible for disconnecting a Wi-Fi connection by 
// sending a series of commands to a WiFiSerial interface, flushing the output, 
// and reading the status response. It also resets the Wi-Fi data and clears the 
// IP address stored in wifidata.
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

// The AccessPoint function sets up a Wi-Fi access point by sending configuration 
// data such as the host name, SSID, and password to a serial interface, and retrieves 
// the IP address assigned to the access point. It also manages the serial 
// communication by flushing and reading available data to ensure proper setup 
// and operation.
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

// The Connect function attempts to establish a connection to a wireless network by 
// first disconnecting any existing connection, sending the host name, SSID, and 
// password to the WiFi module, and then checking for a successful connection 
// status. If successful, it retrieves the IP address; otherwise, it clears the IP 
// information and updates the connection status.
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

// The WiFi_init function initializes the WiFi interface at power-up by checking the 
// configuration settings, setting up the appropriate serial port, and attempting to 
// connect to a WiFi network or set up an access point based on the defined SSID. 
// It also configures a thread for handling WiFi operations and adds a menu entry 
// for the WiFi module in the main application.
void WiFi_init(void)
{
  String res;

  RestorWiFiSettings(false);

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

// The WiFi_loop function is responsible for managing the Wi-Fi interface in an 
// application, specifically refreshing all dialog entries in the WiFiDialog if 
// the current active dialog entry is WiFiEntriesPage1. This function is likely 
// called repeatedly in a loop to ensure the user interface remains responsive 
// and up-to-date with the current Wi-Fi status.
void WiFi_loop(void)
{
  if (ActiveDialog->Entry == WiFiEntriesPage1) RefreshAllDialogEntries(&WiFiDialog);
}

/*
 * Serial command proccessing routines.
 */

// The isConnected function checks if the WiFi status is "CONNECTED" and, if so, 
// sets an error code and sends a NAK (Negative Acknowledgment), returning true. 
// If the WiFi is not connected, it returns false
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

// The SetHost function sets the host address for a Wi-Fi connection by copying the 
// provided host string into the wifidata.Host variable, but only if the system is 
// not currently connected. After updating the host, it sends an acknowledgment 
// signal.
void SetHost(char *host)
{
  if(isConnected()) return;
  strcpy(wifidata.Host,host);
  SendACK;
}

// The SetSSID function sets the SSID (Service Set Identifier) for a Wi-Fi connection, 
// copying the provided ssid string into the wifidata.ssid variable only if the device 
// is not currently connected. After updating the SSID, it sends an acknowledgment 
// signal.
void SetSSID(char *ssid)
{
  if(isConnected()) return;
  strcpy(wifidata.ssid,ssid);   
  SendACK;
}

// The SetPassword function sets the password for a Wi-Fi data structure, clearing 
// it if the input is "none" and copying the provided password into the structure 
// otherwise. It also checks if the system is connected before proceeding with the 
// password update.
void SetPassword(char *pswd)
{
  if(isConnected()) return;
  // If "none" is entered then the password is cleared
  if(strcmp(pswd,"none") == 0) wifidata.password[0]=0;
  else strcpy(wifidata.password,pswd);     
  SendACK;
}
