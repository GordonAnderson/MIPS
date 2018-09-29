//
// File: Ethernet
//
// This file contains the driver for the USR-TCP232-T V2 Ethernet to RS232 module.
// This code will enable the ethernet to rs232 adapter if found and the set the enabled
// flag telling MIPS the module is present.
//
// Sept 2018, this function was updated to support communicating through the Wire1 twi
// interface to the ethernet module. This requires a twi 2 rs232 interface to connect the
// MIPS wire port to the ethernet interface. This is custom hardware and uses a Adafruit Trinket M0
// to perform the translation. This was done to resolve a MIPS controller disign bug that
// did not provide the needed signals at the interface.
//
// Notes:
//   Send 0x55 0xBB to read configuration from module
//   MIPSconfigData.UseWiFi = true flags the use of the TWI interface
//
// Gordon Anderson
//

#include "ethernet.h"
#include <SerialBuffer.h>

EConfig eConfig;
bool EthernetPresent = false;

SerialBuffer enet_sb;

// This function assumes the serial port has been initalized. The command 0x55 0xBB is sent and the
// configuration data is read back. True is returned if success else false is returned.
bool EloadConfig(EConfig *ec, bool report)
{
  EConfig  EC;
  int i;
  unsigned int mtime;
  uint8_t lastC,ch,*ptr,chksum;

  if(MIPSconfigData.EnetUseTWI)
  { 
    Wire1.beginTransmission(TWI_ENET_ADD);
    Wire1.write(TWI_GET_CFG);
    Wire1.endTransmission();
    delay(2000);
    ptr = (uint8_t *)&EC;
    for(i=0;i<sizeof(EConfig);i++) 
    {
      Wire1.requestFrom(TWI_ENET_ADD, 1);
      if(Wire1.available() >= 1) ptr[i] = Wire1.read();
      SerialUSB.println(ptr[i]);
    }
    // Verify the checksum
    chksum = 0;
    ptr = (uint8_t *)&EC;
    for(i=0;i<sizeof(EConfig)-1;i++) chksum += ptr[i];
    if(chksum != EC.CheckSum) return false;
    // Move data to passed in pointer
    memcpy((void *)ec,(void *)&EC,sizeof(EConfig));
    return true;
  }
  // Lower the config bit
  pinMode(ECONFIG, OUTPUT);
  digitalWrite(ECONFIG,LOW);
  delay(100);
  // Send the command, 0x55 0xBC
  Serial1.write(0x55);
  Serial1.write(0xBC);
  delay(100);
  // Read back 0x55 0xBC to detect start of data
  mtime = millis();
  lastC = 0;
  if(report) serial->println("looking for 0x55, 0xBC");
  while(true)
  {
    if(millis() > mtime + 500) return false;
    if(Serial1.available() > 0) 
    {
      ch = Serial1.read();
      if(report) serial->println(ch, 16);
      if((lastC == 0x55) && (ch == 0xBC)) break;
      lastC = ch;
    }
  }
  // Read data into local structure
  ptr = (uint8_t *)&EC;
  mtime = millis();
  if(report) serial->println("Reading config data");
  for(i=0;i<sizeof(EConfig);i++)
  {
    while(true)
    {
      if(Serial1.available() > 0) break;
      if(millis() > mtime + 500) return false;
    }
    ptr[i] = Serial1.read();
    if(report) serial->println(ptr[i], 16);
  }
  // Verify the checksum
  chksum = 0;
  ptr = (uint8_t *)&EC;
  for(i=0;i<sizeof(EConfig)-1;i++) chksum += ptr[i];
  if(chksum != EC.CheckSum) { if(report) serial->println("Checksum error"); return false; }
  // Move data to passed in pointer
  memcpy((void *)ec,(void *)&EC,sizeof(EConfig));
  // Raise the config bit and exit
  digitalWrite(ECONFIG,HIGH);
  if(report) serial->println("No errors");
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
  for(i=0;i<sizeof(EConfig)-2;i++) ec->CheckSum += ptr[i];
  if(MIPSconfigData.EnetUseTWI)
  { 
    ptr = (uint8_t *)ec;
    Wire1.beginTransmission(TWI_ENET_ADD);
    Wire1.write(TWI_SET_CFG);
    for(i=0;i<sizeof(EConfig);i++) Wire1.write(ptr[i]);
    Wire1.endTransmission();
    return true;
  }
  // Put system in config mode
  pinMode(ECONFIG, OUTPUT);
  digitalWrite(ECONFIG,LOW);
  delay(1000);
  // Send data to ethernet adapter
  //Serial1.flush();
  Serial1.write(0x55);
  Serial1.write(0xBA);  //was AA
  for(i=0;i<(sizeof(EConfig)-2);i++)  Serial1.write(ptr[i]);
  Serial1.write(ec->CheckSum);
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
  if(MIPSconfigData.Ser1ena)
  {
    Serial1.begin(115200);
    return;
  }
  // Exit if WiFi is enabled
  if(MIPSconfigData.UseWiFi == true) return;
  // See if we can find a ethernet adapter
  if(MIPSconfigData.EnetUseTWI)
  { 
    EthernetPresent = false;
    // init the TWI port
    Wire1.begin();
    Wire1.setClock(1000000);
    // See if the TWI enet interface is present
    Wire1.beginTransmission(TWI_ENET_ADD);
    Wire1.write(TWI_GET_PRESENT);
    Wire1.endTransmission();
    Wire1.requestFrom(TWI_ENET_ADD, 1);
    if(Wire1.available() > 0)
    {
       EthernetPresent = Wire1.read();
       EloadConfig(&eConfig);
    }
    enet_sb.begin(&Wire1,TWI_ENET_ADD);
    return;
  }
  Serial1.begin(9600);
  EthernetPresent = EloadConfig(&eConfig);
  digitalWrite(ECONFIG,HIGH);
  if(EthernetPresent)
  {
    Serial1.begin(115200);
  }
}

void Ethernet_test(void)
{
  if(MIPSconfigData.EnetUseTWI)
  {
     serial->println("Not supported with TWI interface.");
     return;
  }
  // See if we can find a ethernet adapter
  Serial1.begin(9600);
  EthernetPresent = EloadConfig(&eConfig,true);
  digitalWrite(ECONFIG,HIGH);
  if(EthernetPresent)
  {
    Serial1.begin(115200);
  }  
}

void ProcessEthernet(void)
{
  uint8_t b;
  
  if((!EthernetPresent) && (!MIPSconfigData.Ser1ena)) return;
  if(MIPSconfigData.EnetUseTWI)
  {
    // Send any chars to enet interface
    enet_sb.flush();
    // If TWI data is available read into processing buffer
    Wire1.requestFrom(TWI_ENET_ADD, 1);
    while (Wire1.available() > 0)
    {
      b = Wire1.read();
      if(b == 255) break;
      serial = &enet_sb;
      PutCh(b);
    }
    return;
  }
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
  if(MIPSconfigData.EnetUseTWI) return EsendConfig(&eConfig);
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

void ReportSNEIP(void)
{
  if(!isEthernetPresent()) return;
  SendACKonly;
  if(!SerialMute) PrintIP(eConfig.SubnetMask);
  if(!SerialMute) serial->println("");
}

void SetSNEIP(char *ips)
{
  if(!isEthernetPresent()) return;
  if(ScanIP(ips, eConfig.SubnetMask)) 
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





