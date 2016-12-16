/*
  // This code will enable the ethernet to rs232 adapter
  // Send 0x55 0xBB to read configuration from module
typedef struct
{
   uint8_t    Dest_IP[4];
   uint16_t   Dest_Port;
   uint8_t    Mod_IP[4];
   uint16_t   Mod_Port;
   uint8_t    Gate_IP[4];   
   uint8_t    WorkMode;
   uint8_t    Baud[3];
   uint8_t    SerialMode;
   uint8_t    Reserved;
   uint8_t    CheckSum;   
} EConfig;
*/
/*  
  delay(5000);
  Serial1.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  delay(100);
  Serial1.write(0x55);
  Serial1.write(0xBB);
  delay(1000);
  while(Serial1.available() > 0)
  {
    SerialUSB.println(Serial1.read());
  }
//  uint8_t  einit[24] = {0x55,0xAA,0xC9,0x00,0xA8,0xC0,0x2A,0x20,0x07,0x00,0xA8,0xC0,0x8C,0x4E,0xC9,0x00,0xA8,0xC0,0x03,0x00,0xC2,0x01,0x03,0xBE};
//  for(int i=0;i<24;i++) Serial1.write(einit[i]);
  delay(100);
  digitalWrite(13,HIGH);
  Serial1.begin(115200);
*/  

#include "ethernet.h"

EConfig eConfig;
bool EthernetPresent = false;

// This function assumes the serial point has been initalized. The command 0x55 0xBB is sent and the
// configuration data is read back. True is returned if success else false is returned.
bool EloadConfig(EConfig *ec)
{
  EConfig  EC;
  int i;
  unsigned int mtime;
  uint8_t lastC,ch,*ptr,chksum;

  // Lower the config bit
  pinMode(ECONFIG, OUTPUT);
  digitalWrite(ECONFIG,LOW);
  delay(100);
  // Send the command, 0x55 0xBB
  Serial1.write(0x55);
  Serial1.write(0xBB);
  delay(100);
  // Read back 0x55 0xBB to detect start of data
  mtime = millis();
  lastC = 0;
  while(true)
  {
    if(millis() > mtime + 500) return false;
    if(Serial1.available() > 0) 
    {
      ch = Serial1.read();
      if((lastC == 0x55) && (ch == 0xBB)) break;
      lastC = ch;
    }
  }
  // Read data into local structure
  ptr = (uint8_t *)&EC;
  mtime = millis();
  for(i=0;i<sizeof(EConfig);i++)
  {
    while(true)
    {
      if(Serial1.available() > 0) break;
      if(millis() > mtime + 500) return false;
    }
    ptr[i] = Serial1.read();
  }
  // Verify the checksum
  chksum = 0;
  ptr = (uint8_t *)&EC;
  for(i=0;i<sizeof(EConfig)-1;i++) chksum += ptr[i];
  if(chksum != EC.CheckSum) return false;
  // Move data to passed in pointer
  memcpy((void *)ec,(void *)&EC,sizeof(EConfig));
  // Raise the config bit and exit
  digitalWrite(ECONFIG,HIGH);
  return true;
}

// This function sends the configuration data to the ethernet adapter. Returns true with no errors.
bool EsendConfig(EConfig *ec)
{
  int i;
  unsigned int mtime;
  uint8_t *ptr,ch;

  if(!EthernetPresent) return false;
  // Calculate the checksum and update the data structure
  ptr = (uint8_t *)ec;
  ec->CheckSum = 0;
  for(i=0;i<sizeof(EConfig)-1;i++) ec->CheckSum += ptr[i];
  // Put system in config mode
  pinMode(ECONFIG, OUTPUT);
  digitalWrite(ECONFIG,LOW);
  delay(1000);
  // Send data to ethernet adapter
  //Serial1.flush();
  Serial1.write(0x55);
  Serial1.write(0xAA);
  for(i=0;i<sizeof(EConfig);i++)  Serial1.write(ptr[i]);
  // We should have a 'K' in the serial buffer so find it!
  mtime = millis();
  while(true)
  {
    if(Serial1.available() > 0)
    {
      ch = Serial1.read();
      if(ch == 'K') 
      {
        digitalWrite(ECONFIG,HIGH);
        return true;
      }
    }
    if(millis() > mtime+1000) break;
  }
  digitalWrite(ECONFIG,HIGH);
  return false;
}

void PrintIP(uint8_t *ip)
{
  serial->print(ip[3]);
  serial->print(".");
  serial->print(ip[2]);
  serial->print(".");
  serial->print(ip[1]);
  serial->print(".");
  serial->print(ip[0]);
}

bool ScanIP(char *ips, uint8_t *ip)
{
  String sval;
  uint8_t IP[4];
  int count;
  int i;

  sval = "";
  count = 0;
  for(i=0;i<strlen(ips);i++)
  {
    if(ips[i] == '.')
    {
      if(count <= 3) IP[count] = sval.toInt();
      count++;
      sval = "";
    }
    else sval += ips[i];
  }
  if(count != 3) return false;
  IP[count] = sval.toInt();
  // Move the data to the passed in pointer
  for(i=0;i<4;i++) ip[3-i] = IP[i];
}

void Ethernet_init(void)
{
  // Exit if WiFi is enabled
  if(MIPSconfigData.UseWiFi == true) return;
  // See if we can find a ethernet adapter
  Serial1.begin(9600);
  EthernetPresent = EloadConfig(&eConfig);
  digitalWrite(ECONFIG,HIGH);
  if(EthernetPresent)
  {
    Serial1.begin(115200);
  }
}

void ProcessEthernet(void)
{
  if(!EthernetPresent) return;
  while (Serial1.available() > 0) 
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
}

//
// Below are the host commands for the ethernet adapter.
//
// GEIP
// SEIP
// GEPORT
// SEPORT
// GEGATE
// SEGATE
//

bool isEthernetPresent(void)
{
  if(!EthernetPresent)
  {
    SetErrorCode(ERR_NOETHERNET);
    SendNAK;
    return false;
  }
  return true;
}

bool UpdateEthernetAdapter(void)
{
  bool bStatus;

  if(!isEthernetPresent()) return false;
  Serial1.begin(9600);
  eConfig.WorkMode = eConfig.Reserved = 3;
  bStatus = EsendConfig(&eConfig);
  Serial1.begin(115200);
  if(bStatus)
  {
     SendACK;
     return bStatus;      
  }
  SetErrorCode(ERR_ETHERNETCOMM);
  SendNAK;
  return bStatus;
}

void ReportEIP(void)
{
  if(!isEthernetPresent()) return;
  SendACKonly;
  if(!SerialMute) PrintIP(eConfig.Mod_IP);
  if(!SerialMute) serial->println("");
}

void SetEIP(char *ips)
{
  if(!isEthernetPresent()) return;
  if(ScanIP(ips, eConfig.Mod_IP)) 
  {
    UpdateEthernetAdapter();
  }
  else 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
  }
}

void ReportEport(void)
{
  if(!isEthernetPresent()) return;
  SendACKonly;
  if(!SerialMute) serial->println(eConfig.Mod_Port);
}

void SetEport(int port)
{
  if(!isEthernetPresent()) return;
  eConfig.Mod_Port = port;
  UpdateEthernetAdapter();
}

void ReportEGATE(void)
{
  if(!isEthernetPresent()) return;
  SendACKonly;
  if(!SerialMute) PrintIP(eConfig.Gate_IP);
  if(!SerialMute) serial->println("");
}

void SetEGATE(char *ips)
{
  if(!isEthernetPresent()) return;
  if(ScanIP(ips, eConfig.Gate_IP)) 
  {
    UpdateEthernetAdapter();
  }
  else 
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
  }
}



