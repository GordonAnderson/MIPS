#ifndef ETHERNET_H_
#define ETHERNET_H_

// If used with the TWI2ENET adapter the use the following parameters
#define TWI_ENET_ADD      0x42
#define TWI_SET_CFG       0x80
#define TWI_GET_CFG       0x81
#define TWI_GET_PRESENT   0x82

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
   uint8_t    IndependentID[3];
   uint8_t    SubnetMask[4];
   uint8_t    FirmwareVer;
   uint8_t    CheckSum;   
} EConfig;

#define ECONFIG  13   // Configuration line, pull low to enter config mode

// Prototypes
bool EloadConfig(EConfig *ec, bool report = false);
void Ethernet_init(void);
void Ethernet_test(void);
void ProcessEthernet(void);
bool ScanIP(char *ips, uint8_t *ip);
void PrintIP(uint8_t *ip);
bool EsendConfig(EConfig *ec);

// Host command prototypes
bool isEthernetPresent(void);
bool UpdateEthernetAdapter(void);
void ReportEIP(void);
void SetEIP(char *ips);
void ReportSNEIP(void);
void SetSNEIP(char *ips);
void ReportEport(void);
void SetEport(int port);
void ReportEGATE(void);
void SetEGATE(char *ips);

#endif
