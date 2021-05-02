#include "Variants.h"
#include "Log.h"

LogData logdata;

DialogBoxEntry MIPSlogEntries[] = {
  {"   Return to main menu  ", 0, 11, D_DIALOG, 0, 0, 0, 0, false, NULL, &MIPSconfig, NULL, NULL},
  {NULL},
};

DialogBox MIPSlog = {
  {"Event log", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MIPSlogEntries
};


void DisplayLog(void)
{
  uint32_t   logTime;
  char *mess, str[20];

  for(int i=0;i<5;i++)
  {
    if(!LogGetMessage(&logTime, &mess)) return;
    PrintDialog(&MIPSlog, 1, i*2, "Time :");
    sprintf(str,"%u",logTime);
    PrintDialog(&MIPSlog, 8, i*2, str);
    PrintDialog(&MIPSlog, 1, i*2 + 1, mess);
  }
}

void LogMessage(char const *mess)
{
  if(!logdata.enabled) return;
  if(logdata.count >= MAXLOG) LogGetMessage(NULL,NULL);
  logdata.LogTime[logdata.tail] = rtc.unixtime();
  logdata.Mess[logdata.tail++] = mess;
  if(logdata.tail >= MAXLOG) logdata.tail = 0;
  logdata.count++;
}

bool LogGetMessage(uint32_t *logTime, char **mess)
{
  if(logdata.count == 0) return false;
  if(logTime != NULL) *logTime = logdata.LogTime[logdata.head];
  if(mess != NULL) *mess = (char *)logdata.Mess[logdata.head];
  *mess = (char *)logdata.Mess[logdata.head];
  logdata.head++;
  if(logdata.head >= MAXLOG) logdata.head = 0;
  logdata.count--;
  return true;
}

void ReportLogEntry(void)
{
  uint32_t   logTime;
  char *mess;

  SendACKonly;
  if(LogGetMessage(&logTime, &mess))
  {
    serial->print(logTime);
    serial->print(",");
    serial->println(mess);
  }
  serial->println("");
}
