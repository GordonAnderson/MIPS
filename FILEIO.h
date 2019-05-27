#ifndef FILEIO_H_
#define FILEIO_H_

bool bmpDraw(char *filename, uint8_t x, uint8_t y);
void bmpReport(char *filename);

// SD file io commands
void ListFiles(void);
void DeleteFile(char *FileName);
void GetFile(char *FileName);
void PutFile(char * FileName,char *Fsize);
void EEPROMtoSD(void);
void SDtoEEPROM(void);
void SaveAlltoSD(void);
void LoadAllfromSD(void);
void EEPROMtoSerial(char *brd, char *add);
void SerialtoEEPROM(char *brd, char *add);

#endif
