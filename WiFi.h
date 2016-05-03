#ifndef WIFI_H_
#define WIFI_H_

enum WiFiStartMode
{
  WS_IDLE,
  WS_CONNECT,
  WS_AP
};

// Struct for the WiFi interface
typedef struct
{
  int16_t        Size;              // This data structures size in bytes
  char           Name[20];          // Holds the module name, "WiFi"
  int8_t         Rev;               // Holds the interface revision number
  WiFiStartMode  WFstartMode;       // Turns the interface on and off
  // WiFi connection parameters
  char           Host[20];          // Holds the module name of the MIPS server, default is MIPSnet
  char           ssid[20];          // Wireless network name
  char           password[20];      // Wireless network password
  char           IP[20];            // Hold the actual IP, system uses DHCP only for IP determination
  int            Status;            // Connection status
  bool           Enable;            // If false the WiFi interface is ignored, this is the default
} WiFiData;


extern WiFiData wifidata;

// Prototypes
void SetHost(char *host);
void SetSSID(char *ssid);
void SetPassword(char *pswd);

#endif




