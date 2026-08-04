//
// QUADscan
//
// Firmware resident m/z scan support for the RF QUAD module, and the mass calibration
// table the scan applies. Rev 3 modules only.
//
// Phase 2 implements the calibration table: structure, interpolation, host commands, and
// persistence in a separate EEPROM region. The scan engine itself follows in Phase 3.
//
// Gordon Anderson
//
#include "QUADscan.h"
#include "Variants.h"
#include "Hardware.h"
#include "Serial.h"
#include "Errors.h"

// Allocated at init time, and only for Rev 3 modules. A NULL entry means either the
// module is not Rev 3 or no QUAD module is present on that board, so the calibration
// table costs 8 bytes of BSS rather than 248 on a system that does not use it. This
// follows the same pattern as RFAarray/RFAstates in RFamp.cpp.
QUADcal *QUADcalTable[2] = {NULL,NULL};

// Compile time guards. The failure mode this prevents is a calibration that quietly stops
// surviving power cycles, which would otherwise be diagnosed as flaky hardware.
static_assert(sizeof(RFAdata) <= QUADcalEEPROMOFFSET,
              "RFAdata has grown into the QUAD cal table EEPROM region");
static_assert(QUADcalEEPROMOFFSET + sizeof(QUADcalRecord) <= QUADcalEEPROMSIZE,
              "QUAD cal record does not fit in the module EEPROM");
// WriteEEPROM writes fixed 16 byte chunks from the supplied address, and an AT24C04 page
// write wraps within its 16 byte page. An unaligned offset therefore corrupts the record
// while still reporting success.
static_assert((QUADcalEEPROMOFFSET % 16) == 0,
              "QUAD cal EEPROM offset must be 16 byte page aligned, see QUADscan.h");

// CRC16, CCITT polynomial. Used only to tell a valid record from an uninitialised or
// corrupt EEPROM region, so the specific polynomial does not matter as long as it is
// stable across firmware versions.
static uint16_t QUADcalCRC(const uint8_t *data, int len)
{
  uint16_t crc = 0xFFFF;

  for(int i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for(int b = 0; b < 8; b++)
    {
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// Returns the board index for a module number, or -1 with the NAK already sent. Also
// rejects any module that is not Rev 3, since the whole feature is Rev 3 only.
static int QUADcalBoard(int Module)
{
  int b;

  if((b = RFAmodule2board(Module)) == -1) return -1;
  // A NULL table means the module is not Rev 3, or allocation failed at init. Either way
  // the calibration commands are not available for it.
  if(QUADcalTable[b] == NULL)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return -1;
  }
  return b;
}

static int QUADcalBoardStr(char *Module)
{
  String token = Module;
  return QUADcalBoard(token.toInt());
}

// ---------------------------------------------------------------------------------------
// Interpolation
// ---------------------------------------------------------------------------------------

// Linear interpolation of y as a function of x, using the table's Actual[] as x and the
// supplied array as y. Extrapolates from the first or last segment outside the table.
static float QUADcalInterp(QUADcal *cal, const float *y, float x)
{
  int n = cal->NumPoints;

  if(n <= 0) return 0;
  if(n == 1) return y[0];
  // Below the table, extrapolate using the first segment
  if(x <= cal->Actual[0])
  {
    float span = cal->Actual[1] - cal->Actual[0];
    if(span == 0) return y[0];
    return y[0] + (x - cal->Actual[0]) * (y[1] - y[0]) / span;
  }
  // Above the table, extrapolate using the last segment
  if(x >= cal->Actual[n-1])
  {
    float span = cal->Actual[n-1] - cal->Actual[n-2];
    if(span == 0) return y[n-1];
    return y[n-1] + (x - cal->Actual[n-1]) * (y[n-1] - y[n-2]) / span;
  }
  // Inside the table, find the bracketing segment
  for(int i = 0; i < n-1; i++)
  {
    if((x >= cal->Actual[i]) && (x <= cal->Actual[i+1]))
    {
      float span = cal->Actual[i+1] - cal->Actual[i];
      if(span == 0) return y[i];
      return y[i] + (x - cal->Actual[i]) * (y[i+1] - y[i]) / span;
    }
  }
  return y[n-1];
}

void QUADcalApply(int brd, float actual, float *commanded, float *delta)
{
  QUADcal *cal = QUADcalTable[brd];

  if((cal == NULL) || (!cal->Enabled) || (cal->NumPoints <= 0))
  {
    *commanded = actual;
    *delta     = 0;
    return;
  }
  if(cal->NumPoints == 1)
  {
    // A single point can only supply a constant offset, there is no slope to work with
    *commanded = actual + (cal->Measured[0] - cal->Actual[0]);
    *delta     = cal->Delta[0];
  }
  else
  {
    *commanded = QUADcalInterp(cal, cal->Measured, actual);
    *delta     = QUADcalInterp(cal, cal->Delta, actual);
  }
  // Extrapolation of the commanded m/z is unbounded, so keep it positive. The delta is
  // bounded downstream by the 18 bit DAC clamp in Set_18bitDAC.
  if(*commanded < 1.0) *commanded = 1.0;
}

bool QUADcalRangeOK(int brd, float startMZ, float stopMZ)
{
  QUADcal *cal = QUADcalTable[brd];
  float    lo, hi, span, margin;

  if((cal == NULL) || (!cal->Enabled) || (cal->NumPoints <= 1)) return true;
  lo   = cal->Actual[0];
  hi   = cal->Actual[cal->NumPoints-1];
  span = hi - lo;
  if(span <= 0) return true;
  margin = span * QUADcalEXTRAP;
  if((startMZ < (lo - margin)) || (startMZ > (hi + margin))) return false;
  if((stopMZ  < (lo - margin)) || (stopMZ  > (hi + margin))) return false;
  return true;
}

// ---------------------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------------------

void QUADcalSave(int Module)
{
  int           b;
  QUADcalRecord rec;

  if((b = QUADcalBoard(Module)) == -1) return;
  memset(&rec, 0, sizeof(QUADcalRecord));
  rec.Signature = QUADcalSIG;
  rec.Size      = sizeof(QUADcalRecord);
  rec.Version   = QUADcalVERSION;
  rec.NumPoints = QUADcalTable[b]->NumPoints;
  rec.Enabled   = QUADcalTable[b]->Enabled;
  for(int i = 0; i < QUADcalMAX; i++)
  {
    rec.Actual[i]   = QUADcalTable[b]->Actual[i];
    rec.Measured[i] = QUADcalTable[b]->Measured[i];
    rec.Delta[i]    = QUADcalTable[b]->Delta[i];
  }
  rec.CRC = QUADcalCRC((uint8_t *)&rec, sizeof(QUADcalRecord) - sizeof(uint16_t));
  if(WriteEEPROM(&rec, RFAarray[b]->EEPROMadr, QUADcalEEPROMOFFSET, sizeof(QUADcalRecord)) == 0)
  {
    SendACK;
    return;
  }
  SetErrorCode(ERR_EEPROMWRITE);
  SendNAK;
}

// Load the record for a board. Returns true if a valid record was found. Does not send an
// ACK or NAK; used both by the host command and by QUADcalInit.
static bool QUADcalLoad(int b)
{
  QUADcalRecord rec;
  uint16_t      crc;

  if(ReadEEPROM(&rec, RFAarray[b]->EEPROMadr, QUADcalEEPROMOFFSET, sizeof(QUADcalRecord)) != 0) return false;
  if(rec.Signature != QUADcalSIG) return false;
  if(rec.Size != (int16_t)sizeof(QUADcalRecord)) return false;
  if(rec.Version != QUADcalVERSION) return false;
  crc = QUADcalCRC((uint8_t *)&rec, sizeof(QUADcalRecord) - sizeof(uint16_t));
  if(crc != rec.CRC) return false;
  // Range check what came out of EEPROM before trusting it
  if((rec.NumPoints < 0) || (rec.NumPoints > QUADcalMAX)) return false;
  QUADcalTable[b]->NumPoints = rec.NumPoints;
  QUADcalTable[b]->Enabled   = rec.Enabled;
  for(int i = 0; i < QUADcalMAX; i++)
  {
    QUADcalTable[b]->Actual[i]   = rec.Actual[i];
    QUADcalTable[b]->Measured[i] = rec.Measured[i];
    QUADcalTable[b]->Delta[i]    = rec.Delta[i];
  }
  return true;
}

void QUADcalRestore(int Module)
{
  int b;

  if((b = QUADcalBoard(Module)) == -1) return;
  if(QUADcalLoad(b))
  {
    SendACK;
    return;
  }
  SetErrorCode(ERR_EEPROMREAD);
  SendNAK;
}

void QUADcalInit(int brd)
{
  if((brd < 0) || (brd > 1)) return;
  if(RFAarray[brd] == NULL) return;
  // Rev 3 only. Nothing is allocated for other revisions, so a system with no Rev 3 QUAD
  // module carries only the two NULL pointers.
  if(RFAarray[brd]->Rev != 3) return;
  if(QUADcalTable[brd] == NULL) QUADcalTable[brd] = new QUADcal;
  if(QUADcalTable[brd] == NULL) return;
  memset(QUADcalTable[brd], 0, sizeof(QUADcal));
  // A module that has never been calibrated simply has no record; that is not an error
  QUADcalLoad(brd);
}

// ---------------------------------------------------------------------------------------
// Host commands
// ---------------------------------------------------------------------------------------

void QUADcalSetEnable(char *Module, char *value)
{
  int    b;
  String token;

  if((b = QUADcalBoardStr(Module)) == -1) return;
  token = value;
  if(token == "TRUE") QUADcalTable[b]->Enabled = true;
  else if(token == "FALSE") QUADcalTable[b]->Enabled = false;
  else BADARG;
  SendACK;
}

void QUADcalGetEnable(int Module)
{
  int b;

  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(QUADcalTable[b]->Enabled) serial->println("TRUE");
  else serial->println("FALSE");
}

void QUADcalSetNum(int Module, int num)
{
  int b;

  if((b = QUADcalBoard(Module)) == -1) return;
  if((num < 0) || (num > QUADcalMAX)) BADARG;
  // Reducing the count leaves the higher rows in place but unused. Setting 0 clears the
  // table outright so a stale calibration cannot be resurrected by raising the count.
  if(num == 0) memset(QUADcalTable[b], 0, sizeof(QUADcal));
  else QUADcalTable[b]->NumPoints = num;
  SendACK;
}

void QUADcalGetNum(int Module)
{
  int b;

  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(QUADcalTable[b]->NumPoints);
}

// SQCPNT,module,index,actual,measured,delta
//
// Index is 1 based to match the row numbering the operator sees in the calibration
// procedure. Actual[] must stay strictly ascending: the manual tells the user to enter
// rows in ascending order, but a mis-ordered table interpolates silently wrong, so this
// verifies rather than trusts.
void QUADcalSetPoint(void)
{
  int   module,index,b,i;
  float actual,measured,delta;

  while(true)
  {
    if(!valueFromCommandLine(&module,1,2)) break;
    if(!valueFromCommandLine(&index,1,QUADcalMAX)) break;
    if(!valueFromCommandLine(&actual,1,100000)) break;
    if(!valueFromCommandLine(&measured,1,100000)) break;
    if(!valueFromCommandLine(&delta,-100,100)) break;
    if((b = QUADcalBoard(module)) == -1) return;
    i = index - 1;
    if(i >= QUADcalMAX) break;
    // Verify ordering against the neighbouring rows that are in use
    if((i > 0) && (i < QUADcalTable[b]->NumPoints) && (actual <= QUADcalTable[b]->Actual[i-1])) break;
    if((i < (QUADcalTable[b]->NumPoints - 1)) && (actual >= QUADcalTable[b]->Actual[i+1])) break;
    QUADcalTable[b]->Actual[i]   = actual;
    QUADcalTable[b]->Measured[i] = measured;
    QUADcalTable[b]->Delta[i]    = delta;
    // Growing the table one row at a time is the normal calibration workflow
    if(index > QUADcalTable[b]->NumPoints) QUADcalTable[b]->NumPoints = index;
    SendACK;
    return;
  }
  BADARG;
}

void QUADcalGetPoint(int Module, int index)
{
  int b,i;

  if((b = QUADcalBoard(Module)) == -1) return;
  if((index < 1) || (index > QUADcalMAX)) BADARG;
  i = index - 1;
  SendACKonly;
  if(SerialMute) return;
  serial->print(QUADcalTable[b]->Actual[i],4);   serial->print(",");
  serial->print(QUADcalTable[b]->Measured[i],4); serial->print(",");
  serial->println(QUADcalTable[b]->Delta[i],4);
}

void QUADcalReport(int Module)
{
  int b;

  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->print("QUAD mass calibration, module ");
  serial->print(Module);
  serial->print(", ");
  if(QUADcalTable[b]->Enabled) serial->println("enabled");
  else serial->println("disabled");
  if(QUADcalTable[b]->NumPoints <= 0)
  {
    serial->println("  Table is empty");
    return;
  }
  serial->println("  Row      Actual    Measured       Delta");
  for(int i = 0; i < QUADcalTable[b]->NumPoints; i++)
  {
    serial->print("  ");
    if((i+1) < 10) serial->print(" ");
    serial->print(i+1);
    serial->print("  ");
    serial->print(QUADcalTable[b]->Actual[i],4);   serial->print("  ");
    serial->print(QUADcalTable[b]->Measured[i],4); serial->print("  ");
    serial->println(QUADcalTable[b]->Delta[i],4);
  }
}

// Command table
Commands QUADscanCmdArray[] = {
  {"SQCENA",  CMDfunctionStr, 2, (char *)QUADcalSetEnable},   // Set the QUAD mass cal table enable, module,TRUE|FALSE
  {"GQCENA",  CMDfunction, 1, (char *)QUADcalGetEnable},      // Return the QUAD mass cal table enable flag
  {"SQCNUM",  CMDfunction, 2, (char *)QUADcalSetNum},         // Set number of cal points, module,num. 0 clears the table
  {"GQCNUM",  CMDfunction, 1, (char *)QUADcalGetNum},         // Return number of cal points
  {"SQCPNT",  CMDfunctionLine, 0, (char *)QUADcalSetPoint},   // Set a cal row, module,index,actual,measured,delta
  {"GQCPNT",  CMDfunction, 2, (char *)QUADcalGetPoint},       // Return a cal row, module,index
  {"RQCAL",   CMDfunction, 1, (char *)QUADcalReport},         // Report the whole cal table
  {"SAVEQCAL",   CMDfunction, 1, (char *)QUADcalSave},        // Save the cal table to the module EEPROM
  {"RESTOREQCAL",CMDfunction, 1, (char *)QUADcalRestore},     // Restore the cal table from the module EEPROM
  {0},
};

CommandList QUADscanCmdList = { (Commands *)QUADscanCmdArray, NULL };
