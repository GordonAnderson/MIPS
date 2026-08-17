//
// QUADscan
//
// Firmware resident m/z scan support for the RF QUAD module, and the mass calibration
// table the scan applies. Works on any module revision; see the note in QUADscan.h.
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
#include "ADCdrv.h"
#include <stddef.h>

// Number of bytes the CRC covers: everything up to but NOT including the CRC field.
//
// This must be offsetof(), not sizeof(record) - sizeof(CRC). QUADcalRecord has two bytes of
// trailing padding after CRC, so the subtraction gives 134 rather than 132 and includes the
// CRC field's own bytes in the checksum. At save time memset has zeroed them, so the CRC is
// computed over zeros; on read back those bytes hold the stored CRC, the recomputed value
// differs, and verification fails every single time.
#define QUADcalCRCLEN  offsetof(QUADcalRecord, CRC)

// Defined later in the scan engine section, used before their definitions
static void QUADscanParmsInit(int brd);
static int  QUADscanPointCount(int b);

// Allocated at init time for any board that has an RF QUAD module present. A NULL entry
// means no module is present on that board, so the calibration table costs 8 bytes of BSS
// rather than 248 on a system that does not use it. This follows the same pattern as
// RFAarray/RFAstates in RFamp.cpp.
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

// Returns the board index for a module number, or -1 with the NAK already sent.
static int QUADcalBoard(int Module)
{
  int b;

  if((b = RFAmodule2board(Module)) == -1) return -1;
  // A NULL table means no QUAD module is present on this board, or allocation failed at
  // init. Either way the calibration and scan commands are not available for it.
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
  rec.CRC = QUADcalCRC((uint8_t *)&rec, QUADcalCRCLEN);
  // The module EEPROMs share TWI addresses between the two board slots; the hardware board
  // select line is what disambiguates them. Without this the write lands on whichever board
  // happens to be selected. Same requirement as SaveRFAsettings/RestoreRFAsettings.
  SelectBoard(b);
  if(WriteEEPROM(&rec, RFAarray[b]->EEPROMadr, QUADcalEEPROMOFFSET, sizeof(QUADcalRecord)) == 0)
  {
    // Read back and verify. WriteEEPROM returns success as long as the device ACKs, which
    // it will do even when the data does not land where intended, so an ACK from the write
    // alone is not evidence that anything was stored.
    QUADcalRecord chk;
    if(ReadEEPROM(&chk, RFAarray[b]->EEPROMadr, QUADcalEEPROMOFFSET, sizeof(QUADcalRecord)) == 0)
    {
      if((chk.Signature == QUADcalSIG) &&
         (chk.CRC == QUADcalCRC((uint8_t *)&chk, QUADcalCRCLEN)))
      {
        SendACK;
        return;
      }
    }
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

  // Board select is required before any module EEPROM access, see QUADcalSave
  SelectBoard(b);
  if(ReadEEPROM(&rec, RFAarray[b]->EEPROMadr, QUADcalEEPROMOFFSET, sizeof(QUADcalRecord)) != 0) return false;
  if(rec.Signature != QUADcalSIG) return false;
  if(rec.Size != (int16_t)sizeof(QUADcalRecord)) return false;
  if(rec.Version != QUADcalVERSION) return false;
  crc = QUADcalCRC((uint8_t *)&rec, QUADcalCRCLEN);
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
  // Any revision: the calibration table and scan parameters are m/z arithmetic only, with
  // no dependency on which DAC hardware RFAsetScanPoint ends up driving for this module.
  if(QUADcalTable[brd] == NULL) QUADcalTable[brd] = new QUADcal;
  if(QUADcalTable[brd] == NULL) return;
  memset(QUADcalTable[brd], 0, sizeof(QUADcal));
  QUADscanParmsInit(brd);
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


// ---------------------------------------------------------------------------------------
// Scan parameter commands
// ---------------------------------------------------------------------------------------

static bool QUADscanFloatArg(char *value, float *f, float ll, float ul)
{
  String token = value;
  float  v = token.toFloat();

  if((v < ll) || (v > ul)) return false;
  *f = v;
  return true;
}

void QUADscanSetStart(char *Module, char *value)
{
  int b;
  if((b = QUADcalBoardStr(Module)) == -1) return;
  if(!QUADscanFloatArg(value, &QUADscanP[b]->StartMZ, 1, 100000)) BADARG;
  SendACK;
}
void QUADscanGetStart(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(QUADscanP[b]->StartMZ,4);
}
void QUADscanSetStop(char *Module, char *value)
{
  int b;
  if((b = QUADcalBoardStr(Module)) == -1) return;
  if(!QUADscanFloatArg(value, &QUADscanP[b]->StopMZ, 1, 100000)) BADARG;
  SendACK;
}
void QUADscanGetStop(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(QUADscanP[b]->StopMZ,4);
}
void QUADscanSetStep(char *Module, char *value)
{
  int b;
  if((b = QUADcalBoardStr(Module)) == -1) return;
  if(!QUADscanFloatArg(value, &QUADscanP[b]->StepMZ, -10000, 10000)) BADARG;
  if(QUADscanP[b]->StepMZ == 0) BADARG;
  SendACK;
}
void QUADscanGetStep(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(QUADscanP[b]->StepMZ,4);
}
void QUADscanSetDwell(int Module, int value)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  if((value < 0) || (value > 1000)) BADARG;
  QUADscanP[b]->Dwell = value;
  SendACK;
}
void QUADscanGetDwell(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(QUADscanP[b]->Dwell);
}
void QUADscanSetNumScans(int Module, int value)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  if((value < 1) || (value > 10000)) BADARG;
  QUADscanP[b]->NumScans = value;
  SendACK;
}
void QUADscanGetNumScans(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  serial->println(QUADscanP[b]->NumScans);
}
void QUADscanSetTrig(char *Module, char *value)
{
  int    b;
  String token;
  if((b = QUADcalBoardStr(Module)) == -1) return;
  token = value;
  if(token == "TRUE") QUADscanP[b]->TrigOutEna = true;
  else if(token == "FALSE") QUADscanP[b]->TrigOutEna = false;
  else BADARG;
  SendACK;
}
void QUADscanGetTrig(int Module)
{
  int b;
  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  if(QUADscanP[b]->TrigOutEna) serial->println("TRUE");
  else serial->println("FALSE");
}

// Reports everything the scan will use, including the ADC settings. Those are global and
// are set by commands outside this module, so a user who changed ADCnumsamples for some
// other purpose would otherwise get a silently different acquisition time per point.
void QUADscanStatus(int Module)
{
  int b,n;

  if((b = QUADcalBoard(Module)) == -1) return;
  SendACKonly; if(SerialMute) return;
  n = QUADscanPointCount(b);
  serial->print("QUAD scan, module "); serial->println(Module);
  serial->print("  Start m/z      "); serial->println(QUADscanP[b]->StartMZ,4);
  serial->print("  Stop m/z       "); serial->println(QUADscanP[b]->StopMZ,4);
  serial->print("  Step m/z       "); serial->println(QUADscanP[b]->StepMZ,4);
  serial->print("  Dwell, mS      "); serial->println(QUADscanP[b]->Dwell);
  serial->print("  Num scans      "); serial->println(QUADscanP[b]->NumScans);
  serial->print("  Trigger output ");
  if(QUADscanP[b]->TrigOutEna) serial->println("TRUE"); else serial->println("FALSE");
  serial->print("  Points         ");
  if(n == -1) serial->println("INVALID, check start/stop/step");
  else if(n > QUADscanMAXPOINTS) { serial->print(n); serial->print(" EXCEEDS LIMIT OF "); serial->println(QUADscanMAXPOINTS); }
  else serial->println(n);
  serial->println("  ADC settings are global, set with the ADC commands:");
  serial->print("  ADC channel    "); serial->println(ADCchannel);
  serial->print("  ADC samples    "); serial->println(ADCnumsamples);
  serial->print("  ADC rate, Hz   "); serial->println(ADCrate);
  if(ADCrate > 0)
  {
    float acq = (float)ADCnumsamples * 1000.0 / (float)ADCrate;   // mS per point
    serial->print("  Acquire, mS    "); serial->println(acq,3);
    if(n > 0)
    {
      serial->print("  Est scan, S    ");
      serial->println(((acq + QUADscanP[b]->Dwell) * n * QUADscanP[b]->NumScans) / 1000.0,2);
    }
  }
  serial->print("  Cal table      ");
  if((QUADcalTable[b]->Enabled) && (QUADcalTable[b]->NumPoints > 0))
  {
    serial->print("enabled, "); serial->print(QUADcalTable[b]->NumPoints); serial->println(" points");
    serial->print("  Cal m/z range  ");
    serial->print(QUADcalTable[b]->Actual[0],2); serial->print(" to ");
    serial->println(QUADcalTable[b]->Actual[QUADcalTable[b]->NumPoints-1],2);
    // Show what the table does at the scan endpoints, so a wrong table is visible before
    // a scan is run rather than after the spectrum comes out shifted.
    float c1,d1,c2,d2;
    QUADcalApply(b, QUADscanP[b]->StartMZ, &c1, &d1);
    QUADcalApply(b, QUADscanP[b]->StopMZ,  &c2, &d2);
    // Show the requested value alongside the commanded one, so extrapolation outside the
    // table is obvious rather than something the operator has to work out.
    serial->print("  Start  "); serial->print(QUADscanP[b]->StartMZ,2);
    serial->print(" -> cmd "); serial->print(c1,3);
    serial->print(", delta "); serial->print(d1,4);
    if((QUADscanP[b]->StartMZ < QUADcalTable[b]->Actual[0]) ||
       (QUADscanP[b]->StartMZ > QUADcalTable[b]->Actual[QUADcalTable[b]->NumPoints-1]))
         serial->println("  (extrapolated)");
    else serial->println("");
    serial->print("  Stop   "); serial->print(QUADscanP[b]->StopMZ,2);
    serial->print(" -> cmd "); serial->print(c2,3);
    serial->print(", delta "); serial->print(d2,4);
    if((QUADscanP[b]->StopMZ < QUADcalTable[b]->Actual[0]) ||
       (QUADscanP[b]->StopMZ > QUADcalTable[b]->Actual[QUADcalTable[b]->NumPoints-1]))
         serial->println("  (extrapolated)");
    else serial->println("");
    if(!QUADcalRangeOK(b, QUADscanP[b]->StartMZ, QUADscanP[b]->StopMZ))
      serial->println("  WARNING: scan range too far outside the cal table, QSCAN will reject");
  }
  else serial->println("disabled");
  // State that QSCAN requires. A scan will not run without these and the status report is
  // where that should be visible, not in a GERR round trip after a failed QSCAN.
  serial->print("  Module enabled ");
  if(RFAarray[b]->Enabled) serial->println("TRUE"); else serial->println("FALSE  <- QSCAN will reject");
  serial->print("  Main power     ");
  if(MIPSconfigData.PowerEnable) serial->println("ON"); else serial->println("OFF    <- QSCAN will reject");
  // Rev 3 always passes this; Rev 1/2 needs DCBchan (SRFADCCH) pointed at a present DC
  // bias board or resolving DC never moves during the scan.
  serial->print("  Resolving DC   ");
  if(RFAarray[b]->Rev == 3) serial->println("on board, rev 3");
  else if(RFAresolvingDCReady(b)) { serial->print("DC bias chan "); serial->println(RFAarray[b]->DCBchan); }
  else serial->println("NOT CONFIGURED  <- QSCAN will reject, set with SRFADCCH");
}

// Command table
Commands QUADscanCmdArray[] = {
// Start of command block
//
// QUAD mass calibration table commands. Work on any QUAD module revision; all of these
// NAK with error 2 if no QUAD module is present on the selected board. Module is 1 or 2.
//
  {"SQCENA",  CMDfunctionStr, 2, (char *)QUADcalSetEnable},   // Set the QUAD mass cal table enable, module,TRUE|FALSE
  {"GQCENA",  CMDfunction, 1, (char *)QUADcalGetEnable},      // Return the QUAD mass cal table enable flag
  {"SQCNUM",  CMDfunction, 2, (char *)QUADcalSetNum},         // Set number of cal points, module,num. Range 0 to 10,
                                                             // 0 clears the table
  {"GQCNUM",  CMDfunction, 1, (char *)QUADcalGetNum},         // Return number of cal points
  {"SQCPNT",  CMDfunctionLine, 0, (char *)QUADcalSetPoint},   // Set a cal row, module,index,actual,measured,delta.
                                                             // Index is 1 based, 1 to 10. Actual and measured are m/z,
                                                             // 1 to 100000. Delta is the resolving DC offset in volts,
                                                             // -100 to 100. Actual must stay strictly ascending with
                                                             // index or the command NAKs
  {"GQCPNT",  CMDfunction, 2, (char *)QUADcalGetPoint},       // Return a cal row, module,index. Returns
                                                             // actual,measured,delta
  {"RQCAL",   CMDfunction, 1, (char *)QUADcalReport},         // Report the whole cal table in readable form
  {"SAVEQCAL",   CMDfunction, 1, (char *)QUADcalSave},        // Save the cal table to the module EEPROM. The table is
                                                             // not included in the SAVE command
  {"RESTOREQCAL",CMDfunction, 1, (char *)QUADcalRestore},     // Restore the cal table from the module EEPROM
//
// QUAD scan commands. The scan parameters below are RAM only and are not saved by SAVE or
// SAVEQCAL; they return to their defaults on reset.
//
  {"SQSSTRT", CMDfunctionStr, 2, (char *)QUADscanSetStart},   // Set the scan start m/z, module,mz. Range 1 to 100000,
                                                             // default 50
  {"GQSSTRT", CMDfunction, 1, (char *)QUADscanGetStart},      // Return the scan start m/z
  {"SQSSTOP", CMDfunctionStr, 2, (char *)QUADscanSetStop},    // Set the scan stop m/z, module,mz. Range 1 to 100000,
                                                             // default 500
  {"GQSSTOP", CMDfunction, 1, (char *)QUADscanGetStop},       // Return the scan stop m/z
  {"SQSSTEP", CMDfunctionStr, 2, (char *)QUADscanSetStep},    // Set the scan step size in m/z, module,step. Range
                                                             // -10000 to 10000 excluding 0, default 1. The sign must
                                                             // move start towards stop, so a descending scan needs a
                                                             // negative step
  {"GQSSTEP", CMDfunction, 1, (char *)QUADscanGetStep},       // Return the scan step size
  {"SQSDWELL",CMDfunction, 2, (char *)QUADscanSetDwell},      // Set the per point settle time, module,mS. Range 0 to
                                                             // 1000, default 2. This is the settling window before
                                                             // acquisition starts, and the streaming of the previous
                                                             // point happens inside it. Total time per point is the
                                                             // dwell plus the ADC acquisition time
  {"GQSDWELL",CMDfunction, 1, (char *)QUADscanGetDwell},      // Return the per point settle time in mS
  {"SQSNUM",  CMDfunction, 2, (char *)QUADscanSetNumScans},   // Set the number of scans, module,num. Range 1 to 10000,
                                                             // default 1
  {"GQSNUM",  CMDfunction, 1, (char *)QUADscanGetNumScans},   // Return the number of scans
  {"SQSTRIG", CMDfunctionStr, 2, (char *)QUADscanSetTrig},    // Drive TRGOUT during a scan, module,TRUE|FALSE.
                                                             // Default FALSE. Intended for scope verification of the
                                                             // per point timing
  {"GQSTRIG", CMDfunction, 1, (char *)QUADscanGetTrig},       // Return the TRGOUT during scan flag
  {"QSCANSTAT",CMDfunction, 1, (char *)QUADscanStatus},       // Report the scan parameters, the global ADC settings the
                                                             // scan will use, the computed point count and estimated
                                                             // scan time, and what the cal table does at the scan
                                                             // endpoints. Run this before QSCAN
  {"QSCAN",   CMDfunction, 1, (char *)QUADscanGo},            // Run the scan and stream the result as binary. This
                                                             // command blocks: no other command is processed until the
                                                             // scan finishes. Send ESC, 0x1B, to abort. See the QUAD
                                                             // scan section of the manual for the stream format
// End of table marker
  {0},
};

CommandList QUADscanCmdList = { (Commands *)QUADscanCmdArray, NULL };

// =======================================================================================
// Scan engine
//
// The scan blocks. RFA_loop does not run for its duration, which removes any possibility
// of the 10Hz loop's change detection fighting the scan's own DAC writes. In exchange the
// loop must pet the watchdog itself and poll for the abort character, since the command
// processor is not running either.
//
// Per point timing is deterministic because the USB write for point N-1 is overlapped with
// the physical settling of point N. This is the same pipelining idea already used by the
// host driven RFAACQ path, moved inside the firmware.
// =======================================================================================

QUADscanParms *QUADscanP[2] = {NULL,NULL};

// Allocated alongside the calibration table, for any present module. Called from QUADcalInit.
static void QUADscanParmsInit(int brd)
{
  if(QUADscanP[brd] == NULL) QUADscanP[brd] = new QUADscanParms;
  if(QUADscanP[brd] == NULL) return;
  QUADscanP[brd]->StartMZ    = 50;
  QUADscanP[brd]->StopMZ     = 500;
  QUADscanP[brd]->StepMZ     = 1;
  QUADscanP[brd]->Dwell      = 2;
  QUADscanP[brd]->NumScans   = 1;
  QUADscanP[brd]->TrigOutEna = false;
}

// Acquire one spectrum point. Returns false on ADC timeout or failure; the caller aborts
// the scan rather than emitting a bad point.
static bool QUADscanAcquire(int *sum)
{
  if(!ADCrbTrigger()) return false;
  return ADCfindSum(sum);
}

static void QUADscanSendHeader(int numPoints, int scanNum, bool last)
{
  serial->write(0x55); serial->write(0x55); serial->write(0x55); serial->write(0x55);
  serial->write(QUADscanHDR);
  serial->write((byte)( numPoints        & 0xFF));
  serial->write((byte)((numPoints >>  8) & 0xFF));
  serial->write((byte)((numPoints >> 16) & 0xFF));
  serial->write((byte)( scanNum       & 0xFF));
  serial->write((byte)((scanNum >> 8) & 0xFF));
  serial->write(last ? 0xFF : 0x00);
}

static void QUADscanSendTrailer(bool aborted)
{
  serial->write(0xAE); serial->write(0xAE); serial->write(0xAE); serial->write(0xAE);
  serial->write(aborted ? QUADscanABORTED : QUADscanTRAILER);
}

// Fixed width, little endian. Fixed width matters: variable length ASCII would make the
// per point write time depend on the magnitude of the value, reintroducing exactly the
// timing jitter this feature exists to remove.
//
// The raw sum is sent rather than an average. Averaging ADCnumsamples samples buys
// resolution beyond the ADC's 12 bits; returning a 12 bit average would discard it. The
// host knows ADCnumsamples and can divide if it wants to.
static void QUADscanSendPoint(int sum)
{
  serial->write((byte)( (uint32_t)sum        & 0xFF));
  serial->write((byte)(((uint32_t)sum >>  8) & 0xFF));
  serial->write((byte)(((uint32_t)sum >> 16) & 0xFF));
  serial->write((byte)(((uint32_t)sum >> 24) & 0xFF));
}

// Poll for the abort character. The command processor is not running during a scan, so
// anything else that arrives is pushed into the normal input ring buffer to be handled
// after the scan. Discarding it, as an earlier version did, silently ate any command typed
// while a scan was running.
static bool QUADscanAbortRequested(void)
{
  bool abort = false;

  while(serial->available())
  {
    char c = serial->read();
    if(c == QUADscanABORTCHAR) abort = true;
    else PutCh(c);
  }
  return abort;
}

// Compute the point count for the current parameters, or -1 if they are not valid.
static int QUADscanPointCount(int b)
{
  QUADscanParms *sp = QUADscanP[b];
  float span;
  int   n;

  if(sp == NULL) return -1;
  if(sp->StepMZ == 0) return -1;
  span = sp->StopMZ - sp->StartMZ;
  if(span == 0) return 1;
  // The step must move from start towards stop
  if((span > 0) && (sp->StepMZ < 0)) return -1;
  if((span < 0) && (sp->StepMZ > 0)) return -1;
  n = (int)(span / sp->StepMZ) + 1;
  if(n < 1) return -1;
  return n;
}

void QUADscanGo(int Module)
{
  int   b,numPoints,sum = 0,err;
  float cmdMZ,delta;
  bool  aborted = false;
  QUADscanParms *sp;

  if((b = QUADcalBoard(Module)) == -1) return;
  sp = QUADscanP[b];
  if(sp == NULL) BADARG;
  // Validate everything before any frame header is emitted, so the host never sees a
  // truncated or headerless stream.
  // Distinct error codes so a failed QSCAN explains itself through GERR rather than
  // collapsing six different conditions into one BADARG.
  if(!RFAarray[b]->Enabled)       { SetErrorCode(ERR_QUADNOTENABLED);    SendNAK; return; }
  if(!MIPSconfigData.PowerEnable) { SetErrorCode(ERR_QUADPOWEROFF);      SendNAK; return; }
  // Rev 3 always passes; Rev 1/2 routes resolving DC through DCBchan and would otherwise
  // silently run the whole scan without ever moving it, see RFAresolvingDCReady.
  if(!RFAresolvingDCReady(b))     { SetErrorCode(ERR_QUADNODCBCHAN);     SendNAK; return; }
  if((numPoints = QUADscanPointCount(b)) == -1)
                                  { SetErrorCode(ERR_QUADBADRANGE);      SendNAK; return; }
  if(numPoints > QUADscanMAXPOINTS)
                                  { SetErrorCode(ERR_QUADTOOMANYPOINTS); SendNAK; return; }
  if(!QUADcalRangeOK(b, sp->StartMZ, sp->StopMZ))
                                  { SetErrorCode(ERR_QUADOUTSIDECAL);    SendNAK; return; }
  if(sp->NumScans < 1)            { SetErrorCode(ERR_QUADBADRANGE);      SendNAK; return; }
  // Claim the ADC. ADCvectors must cover every point of every scan: ADC_Handler tears the
  // ADC down and calls ReleaseADC once ADCvectorNum reaches it, so leaving it at the
  // default of 1 would dismantle the ADC after the very first spectrum point.
  ADCvectors = (numPoints * sp->NumScans) + 2;
  if((err = ADCrbSetup()) != 0)
  {
    SetErrorCode(err);
    SendNAK;
    return;
  }
  SendACKonly;
  DisplayMessage("Scanning");
  for(int scan = 0; (scan < sp->NumScans) && !aborted; scan++)
  {
    QUADscanSendHeader(numPoints, scan, (scan == (sp->NumScans - 1)));
    // Prime the pipeline: set the first point and acquire it before the loop, so that
    // inside the loop the USB write for the previous point overlaps this point's settling.
    QUADcalApply(b, sp->StartMZ, &cmdMZ, &delta);
    RFAsetScanPoint(b, cmdMZ, delta);
    if(sp->TrigOutEna) SetTRGOUT;
    if(sp->Dwell > 0) delay(sp->Dwell);
    // A failed priming acquisition must not skip the trailer. This used to break straight
    // out of the scan loop, past QUADscanSendTrailer below, so an ADC timeout on the very
    // first point of a scan left the host holding a header, no points, and nothing to
    // terminate on: neither the promised byte count nor a trailer ever arrived and only a
    // host side idle timeout recovered it. Every header now gets a trailer.
    if(!QUADscanAcquire(&sum)) aborted = true;
    if(!aborted)
    {
      if(sp->TrigOutEna) ResetTRGOUT;
      for(int i = 1; i < numPoints; i++)
      {
        WDT_Restart(WDT);
        if(QUADscanAbortRequested()) { aborted = true; break; }
        uint32_t t0 = micros();
        if(sp->TrigOutEna) SetTRGOUT;
        // Set this point, which starts the physical settling. The calibration table maps the
        // requested (true) m/z to the m/z that must be commanded, and supplies the resolving
        // DC offset for that mass. With the table disabled or empty this is a pass through:
        // cmdMZ = requested, delta = 0.
        QUADcalApply(b, sp->StartMZ + (float)i * sp->StepMZ, &cmdMZ, &delta);
        RFAsetScanPoint(b, cmdMZ, delta);
        // Stream the previous point while this one settles
        QUADscanSendPoint(sum);
        // Wait out whatever is left of the dwell
        while((micros() - t0) < (uint32_t)(sp->Dwell * 1000)) ;
        if(!QUADscanAcquire(&sum)) { aborted = true; break; }
        if(sp->TrigOutEna) ResetTRGOUT;
      }
      if(!aborted) QUADscanSendPoint(sum);      // The final point of this scan
    }
    QUADscanSendTrailer(aborted);
  }
  if(sp->TrigOutEna) ResetTRGOUT;
  ADCstop();
  DismissMessage();
  // The scan moved m/z, the setpoint and the pole DACs. Let the loop resynchronise from
  // the module structure rather than leaving the hardware wherever the scan finished.
  RFAupdate = true;
}
