#ifndef QUADSCAN_H_
#define QUADSCAN_H_

#include "RFamp.h"
#include "Serial.h"

//
// QUADscan supports the firmware resident m/z scan for the RF QUAD module, and the mass
// calibration table that scan applies.
//
// Works on any revision. The calibration table and scan parameters are pure m/z arithmetic
// with no dependency on which DAC hardware actually drives the quad, and RFAsetScanPoint
// already dispatches to the right hardware path per module (see setRFADAC/SetResolvingDC
// in RFamp.cpp). The one Rev specific requirement is that a Rev 1/2 module needs its
// resolving DC bias channel (DCBchan, set with SRFADCCH) configured before QSCAN will run;
// see RFAresolvingDCReady. Rev 3 has no such requirement, its resolving DC DACs are on
// board. See the EEPROM note below for why the calibration table lives outside RFAdata.
//
// Mass calibration
// ----------------
// The table holds up to QUADcalMAX rows, each recording:
//    Actual    the true m/z
//    Measured  the m/z that had to be commanded to put the peak at Actual
//    Delta     the resolving DC offset that gave the best peak shape at that m/z
//
// Two independent piecewise linear interpolations are performed, both keyed on Actual:
//    Actual -> Measured   gives the m/z to command
//    Actual -> Delta      gives the resolving DC offset to apply
//
// Outside the calibrated range both are extrapolated from the slope of the first or last
// segment. A scan whose range falls more than QUADcalEXTRAP times the table span outside
// the table is rejected rather than silently extrapolated.
//

#define QUADcalMAX          10        // Maximum calibration table points
#define QUADcalEXTRAP       2.0       // Reject a scan beyond this multiple of the table span

// This structure is the in RAM working copy. It is deliberately NOT a member of RFAdata.
//
// RFAdata is exactly 352 bytes and is written to the module EEPROM at offset 0. Adding the
// calibration table to it would grow sizeof(RFAdata) for every module including Rev 1. A
// Rev 1 field unit running this firmware would then write the larger structure to its
// module EEPROM, and if that module were later moved into a system running older firmware,
// the unguarded memcpy in that firmware's RestoreRFAsettings would overflow. Bug B10 is
// fixed going forward but the firmware already deployed cannot be retrofitted. Keeping the
// table in a separate EEPROM region leaves sizeof(RFAdata) untouched.
typedef struct
{
  int8_t   NumPoints;                 // 0 to QUADcalMAX
  bool     Enabled;                   // Apply the table during a scan
  float    Actual[QUADcalMAX];        // True m/z
  float    Measured[QUADcalMAX];      // m/z that produced the peak at Actual
  float    Delta[QUADcalMAX];         // Resolving DC offset, volts
} QUADcal;

// On EEPROM representation. The fields are flattened rather than nesting QUADcal so there
// is only one round of alignment padding. Verified to be 136 bytes.
//
// Module EEPROM is an AT24CS04, 512 bytes total. RFAdata occupies 0..351.
//
// THE OFFSET MUST BE A MULTIPLE OF 16. The AT24C04 has a 16 byte page, and a page write
// that starts part way into a page wraps around to the start of that page instead of
// continuing into the next one. WriteEEPROM in Hardware.cpp writes in fixed 16 byte
// chunks from the supplied address, so an unaligned offset silently scrambles the record.
// The write still ACKs and returns success; the corruption only shows up as a CRC failure
// on the next read. This cost a debugging session, hence the static_assert below.
//
// 368 is the highest 16 byte aligned offset that still fits the record: 368 + 136 = 504,
// inside the 512 byte device, leaving 16 bytes of growth room for RFAdata. Chunks land at
// 368, 384 ... 480, then a final 8 bytes at 496, all page aligned.
//
// 368..503 also sits entirely within the upper 256 byte half, so the record never crosses
// the boundary that ReadEEPROM/WriteEEPROM handle via the device address bit.
#define QUADcalEEPROMOFFSET 368
#define QUADcalSIG          0x51434C31UL    // "QCL1"
#define QUADcalVERSION      1
#define QUADcalEEPROMSIZE   512

typedef struct
{
  uint32_t Signature;                 // QUADcalSIG
  int16_t  Size;                      // sizeof(QUADcalRecord)
  int8_t   Version;                   // QUADcalVERSION
  int8_t   NumPoints;
  bool     Enabled;
  float    Actual[QUADcalMAX];
  float    Measured[QUADcalMAX];
  float    Delta[QUADcalMAX];
  uint16_t CRC;                       // CRC16 over everything above
} QUADcalRecord;

// Allocated at init time for any present RF QUAD module; NULL otherwise. See QUADcalInit.
// Scan parameters. RAM only, not persisted: the EEPROM record region has only 8 bytes
// spare (368..503 used of 368..511) and these need 24, so persisting them would require
// relocating or shrinking the calibration record. Defaults are applied at init.
typedef struct
{
  float StartMZ;
  float StopMZ;
  float StepMZ;
  int   Dwell;                 // Settle time per point, mS
  int   NumScans;              // Scans per QSCAN command
  bool  TrigOutEna;            // Drive TRGOUT during the scan for timing verification
} QUADscanParms;

#define QUADscanMAXPOINTS   2000

// Streaming frame markers. Same convention as ADCtrigger in ADCdrv.cpp so the host parser
// pattern is familiar.
#define QUADscanHDR         0xAA
#define QUADscanTRAILER     0xEA      // Normal completion
#define QUADscanABORTED     0xEB      // Scan was aborted
#define QUADscanABORTCHAR   0x1B      // ESC on the serial port aborts a running scan

extern QUADcal *QUADcalTable[2];
extern QUADscanParms *QUADscanP[2];
extern CommandList QUADscanCmdList;

// Prototypes

// Called from RFA_init for each board. Zeros the working copy and loads the EEPROM record
// if a valid record is present.
void QUADcalInit(int brd);

// Apply the calibration table. Returns the m/z to command and the resolving DC offset.
// With the table disabled or empty, commanded = actual and delta = 0.
void QUADcalApply(int brd, float actual, float *commanded, float *delta);

// True if the requested scan range is close enough to the calibrated range to be trusted.
bool QUADcalRangeOK(int brd, float startMZ, float stopMZ);

// Host interface
void QUADcalSetEnable(char *Module, char *value);
void QUADcalGetEnable(int Module);
void QUADcalSetNum(int Module, int num);
void QUADcalGetNum(int Module);
void QUADcalSetPoint(void);
void QUADcalGetPoint(int Module, int index);
void QUADcalReport(int Module);
void QUADcalSave(int Module);
void QUADcalRestore(int Module);

// Scan engine
void QUADscanSetStart(char *Module, char *value);
void QUADscanGetStart(int Module);
void QUADscanSetStop(char *Module, char *value);
void QUADscanGetStop(int Module);
void QUADscanSetStep(char *Module, char *value);
void QUADscanGetStep(int Module);
void QUADscanSetDwell(int Module, int value);
void QUADscanGetDwell(int Module);
void QUADscanSetNumScans(int Module, int value);
void QUADscanGetNumScans(int Module);
void QUADscanSetTrig(char *Module, char *value);
void QUADscanGetTrig(int Module);
void QUADscanStatus(int Module);
void QUADscanGo(int Module);

#endif
