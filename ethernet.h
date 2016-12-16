#ifndef ETHERNET_H_
#define ETHERNET_H_

extern bool EthernetPresent;

typedef struct
{
   uint8_t    Dest_IP[4];
   uint16_t   Dest_Port;
   uint8_t    Mod_IP[4];
   uint16_t   Mod_Port;
   uint8_t    Gate_IP[4];   
   uint8_t    WorkMode;
   uint8_t    Baud[3];
   uint8_t    Reserved;
   uint8_t    CheckSum;   
} EConfig;

#define ECONFIG  13   // Configuration line, pull low to enter config mode

// Prototypes
bool EloadConfig(EConfig *ec);
void Ethernet_init(void);
void ProcessEthernet(void);
bool ScanIP(char *ips, uint8_t *ip);
void PrintIP(uint8_t *ip);
bool EsendConfig(EConfig *ec);

// Host command prototypes
bool isEthernetPresent(void);
bool UpdateEthernetAdapter(void);
void ReportEIP(void);
void SetEIP(char *ips);
void ReportEport(void);
void SetEport(int port);
void ReportEGATE(void);
void SetEGATE(char *ips);

#endif



