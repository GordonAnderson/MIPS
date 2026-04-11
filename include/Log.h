#ifndef LOG_H_
#define LOG_H_

#define MAXLOG  5

extern DialogBoxEntry MIPSlogEntries[];
extern DialogBox MIPSlog;

typedef struct
{
  bool       enabled=true;
  uint32_t   LogTime[MAXLOG];
  char const *Mess[MAXLOG];
  int8_t     head=0;
  int8_t     tail=0;
  int8_t     count=0;
} LogData;

extern LogData logdata;

void DisplayLog(void);
void LogMessage(char const *mess);
bool LogGetMessage(uint32_t *logTime, char **mess);
void ReportLogEntry(void);

#endif
