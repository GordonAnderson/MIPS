//
//  Table.h
//  MIPS
//
//  Created by Gordon Anderson on 9/4/14.
//
//

#ifndef Table_h
#define Table_h

// Table byte indexes into key locations
#define IndexTableName    0
#define IndexRepeatCount  1
#define IndexMaxCount     3
#define IndexNumentries   5

// Custom data types a enums
enum TriggerModes
{
	SW,     // Software
	EDGE,   // External any edge
	POS,    // External positive edge
  NEG     // External negative edge
};

enum ClockModes
{
	EXT,     // External
	MCK2,    // Internal, options are MCK / (2 or 8 or 32 or 128)
  MCK8,
  MCK32,
  MCK128
};

enum TableModes
{
    TBL,
    LOC
};

// Table data structures for the three sections of a table:
//  -- Header
//  -- Entry header
//  -- Entry
// Table is packed:
//  Table header
//  Entry header, one for each entry
//  Entry, one for each entry
typedef struct
{
    char TableName;
    int  RepeatCount;
    int  MaxCount;
    char NumEntries;
} TableHeader;

typedef struct
{
    int  Count;
    char NumChans;
} TableEntryHeader;

typedef struct
{
    char Chan;
    int  Value;
} TableEntry;

// Table nesting stack
#define     MaxNesting  5
typedef struct
{
    volatile char        Ptr;
    volatile TableHeader *Table[MaxNesting];
    volatile int         Count[MaxNesting];
} NestingStack;

extern enum TableModes  TableMode;
extern bool TasksEnabled;
extern int  InterTableDelay;
extern bool ValueChange;
extern volatile bool softLDAC;

// ProcessEntry return codes
#define PEprocessed 1
#define PENewNamedTable 2
#define PENewTable 3
#define PEEndTables 4
#define PEerror 0

void SetImageRegs(void);
void ProcessSerial(void);
// Function prototypes

// Serial IO routines
char *NextToken(void);
bool ExpectColon(void);
bool Token2int(int *val);
bool Token2float(float *fval);
void SetTableMode(char *cmd);
void SetTableCLK(char *cmd);
void SetTableTRG(char *cmd);
void SetTableAbort(void);
void SWTableTrg(void);
void TableFreq(void);
void StopTable(void);
void ParseTableCommand(void);
int  ParseEntry(int Count, char *TK);
void ReportTable(int count);
void SetTableNumber(int tblnum);
void GetTableNumber(void);
void GetTableAdvance(void);
void SetTableAdvance(char *cmd);
void GetTableEntryValue(int Count, int Chan);
void SetTableEntryValue(int Count, int Chan, float fval);
void SetTableEntryCount(int Count, int Chan, float NewCount);

// Real time processing routines
void UserSetAbort(void);
void ProcessTables(void);
void SetupTimer(void);
void StartTimer(void);
void StopTimer(void);
bool AdvanceTablePointer(void);
void AdvanceEntryPointer(void);
void SetupNextEntry(void);

// Call backs
void Dummy_ISR(void);
void Trigger_ISR(void);
void RAmatch_Handler(void);
void RCmatch_Handler(void);

#endif




