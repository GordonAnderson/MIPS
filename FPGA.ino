#include "Variants.h"

//

//MIPS Threads
Thread FPGAthread  = Thread();

int fpgaBoard;
FPGAmodule *fpgaM   = NULL;
FPGAdata   *fpga    = NULL;
float      *ADCvals = NULL;

const Commands FPGACmdArray[] = {
  {"SFPGACFG",  CMDfunctionStr, 1, (char *)setFPGAconfig},           // Set the FPGA config word, value is in hex
  {"GFPGACFG",  CMDfunction,    0, (char *)getFPGAconfig},           // Returns the FPGA config word, value is in hex
  {"SFPGABC",   CMDfunctionStr, 1, (char *)setFPGAbusControl},       // Set FPGA bus control mode, TRUE = fpga in control
  {"GFPGABC",   CMDfunction,    0, (char *)getFPGAbusControl},       // Returns FPGA bus control mode, TRUE = fpga in control
  {"SFPGAP",    CMDfunctionStr, 1, (char *)setFPGAprocess},          // Set FPGA process mode, TRUE = fpga processes ADC data and outputs DAC data
  {"GFPGAP",    CMDfunction,    0, (char *)getFPGAprocess},          // Returns FPGA process mode, TRUE = fpga processes ADC data and outputs DAC data
  {"SFPGASCN",  CMDfunctionStr, 1, (char *)setFPGAadcUpdate},        // Set FPGA ADC scan mode, TRUE = fpga scans ADC data
  {"GFPGASCN",  CMDfunction,    0, (char *)getFPGAadcUpdate},        // Returns FPGA ADC scan mode, TRUE = fpga scans ADC data
  {"SFPGAADD",  CMDfunction,    1, (char *)setFPGAaddress},          // Set the FPGA DAC address, value is 0 thru 7
  {"GFPGAADD",  CMDfunction,    0, (char *)getFPGAaddress},          // Returns the FPGA DAC address, value is 0 thru 7
  {"SFPGAARY",  CMDfunctionLine,0, (char *)setFPGAarrayValue},       // Sets an array element, (M,B,ADC,DAC), index (1 thru 8), value
  {"GFPGAARY",  CMDfunctionStr, 2, (char *)getFPGAarrayValue},       // Returns an array element, (M,B,ADC,DAC), index (1 thru 8)
  {"SADCMSK",   CMDfunctionStr, 1, (char *)setADCmask},              // Set the ADC mask, hex. When a bit is set the channel is not updated
  {"GADCMSK",   CMDfunction,    0, (char *)getADCmask},              // Return the ADC mask, hex. When a bit is set the channel is not updated
  {"SDACMSK",   CMDfunctionStr, 1, (char *)setDACmask},              // Set the DAC mask, hex. When a bit is set the channel is not updated
  {"GDACMSK",   CMDfunction,    0, (char *)getDACmask},              // Return the DAC mask, hex. When a bit is set the channel is not updated
  {"RADCALL",   CMDfunction,    0, (char *)reportAllADC},            // Report all 8 ADC channels
  {"GADCCH",   CMDfunction,     1, (char *)reportADC},               // Report an ADC channel, 1 thru 8
  {"SAVEFPGA",   CMDfunction,   0, (char *)saveFPGA},                // Save setting to EEPROM
  {"RSTRFPGA",   CMDfunction,   0, (char *)restoreFPGA},             // Restore settings from FPGA
  {"FPGACAL",   CMDfunction,    1, (char *)calADCchan},              // Calibrate ADC channel
  {"FPGACALR",   CMDfunction,   1, (char *)calDACreflect},           // Calibrate ADC to DAC reflector, first DAC channel to use, must 
                                                                     // 1 or 9
  {0}
};

CommandList FPGACmdList = {(Commands *)FPGACmdArray, NULL };

void FPGAinit(void)
{
  int cfg = 0;

  if(fpga == NULL) return;
  cfg = fpga->cfg;
  writeFPGAcfg;
  writeFPGAadcMSK;
  writeFPGAdacMSK;
  writem;
  writeb;
  writeADC;
  writeDAC;
  fpga->cfg = cfg;
  writeFPGAcfg;
}

void readFPGAdacDisplay(void)
{
  readDAC;
  for(int i=0;i<8;i++) serial->println(fpga->dac[i]);
}

bool writeFPGAarrayElement(FPGA_array array, int index, int val)
{
  int offset = (int)array;
  uint8_t *ptr;

  switch(array)
  {
    case FPGA_M:
      fpga->m[index] = val;
      ptr = (uint8_t *)&fpga->m[index];
      break;
    case FPGA_B:
      fpga->b[index] = val;
      ptr = (uint8_t *)&fpga->b[index];
      break;
    case FPGA_ADC:
      fpga->adc[index] = val;
      ptr = (uint8_t *)&fpga->adc[index];
      break;
    case FPGA_DAC:
      fpga->dac[index] = val;
      ptr = (uint8_t *)&fpga->adc[index];
      break;
    default:
      return false;
      break;
  }
  writeFPGA(fpga, offset + 2 * index, 2, ptr);
  return true;
}

bool readFPGAarrayElement(FPGA_array array, int index, int *val)
{
  int offset = (int)array;
  uint8_t *ptr;

  switch(array)
  {
    case FPGA_M:
      ptr = (uint8_t *)&fpga->m[index];
      break;
    case FPGA_B:
      ptr = (uint8_t *)&fpga->b[index];
      break;
    case FPGA_ADC:
      ptr = (uint8_t *)&fpga->adc[index];
      break;
    case FPGA_DAC:
      ptr = (uint8_t *)&fpga->adc[index];
      break;
    default:
      return false;
      break;
  }
  readFPGA(fpga, offset + index * 2, 2, ptr);
  *val = (ptr[1] << 8) | ptr[0];
  return true;
}

int readADCaverage(int chan, int num)
{
  int val,average=0;

  for(int i=0;i<num;i++)
  {
    readFPGAarrayElement(FPGA_ADC,chan,&val);
    average += val;
    delay(5);
  }
  return(average/num);
}

void writeFPGA(FPGAdata *fd, int offset, int numBytes, uint8_t *ptr)
{
  if(fd == NULL) return;
  // Select the board address
  SelectBoard(0);
  // Set SPI mode
  SPI.setDataMode(SPI_CS, SPI_MODE0);
  // Set the address bits
  SetAddress(fd->boardAdd);
  // Send the FPGA address
  SPI.transfer(SPI_CS, (uint8_t)(fd->baseAdd + offset) , SPI_CONTINUE);
  // Write the data
  for(int i=0;i<numBytes;i++)
  {
    if(i == (numBytes-1)) SPI.transfer(SPI_CS, ptr[i], SPI_LAST);
    else SPI.transfer(SPI_CS, ptr[i], SPI_CONTINUE);
  }
  // Restore mode
  SPI.setDataMode(SPI_CS, SPI_MODE1);
}

void readFPGA(FPGAdata *fd, int offset, int numBytes, uint8_t *ptr)
{
  if(fd == NULL) return;
  // Select the board address
  SelectBoard(0);
  // Set SPI mode
  SPI.setDataMode(SPI_CS, SPI_MODE0);
  // Set the address bits
  SetAddress(fd->boardAdd);
  // Send the FPGA address
  SPI.transfer(SPI_CS, (uint8_t)(fd->baseAdd + offset) | 0x80, SPI_CONTINUE);
  // Need two dummy reads, not sure why?
  SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
  SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
  // Read the data
  for(int i=0;i<numBytes;i++)
  {
    if(i == (numBytes-1)) ptr[i] = SPI.transfer(SPI_CS, 0, SPI_LAST);
    else ptr[i] = SPI.transfer(SPI_CS, 0, SPI_CONTINUE);
  }
  // Restore mode
  SPI.setDataMode(SPI_CS, SPI_MODE1);
}

// Write the current board parameters to the EEPROM on the FPGA board.
bool SaveFPGAsettings(void)
{
  SelectBoard(fpgaBoard);
  fpgaM->Size = sizeof(FPGAmodule);
  if (WriteEEPROM(fpgaM, fpgaM->TWIadd, 0, sizeof(FPGAmodule)) == 0) return true;
  return false;
}

bool RestoreFPGAsettings(void)
{
  FPGAmodule fm;
//  bool SaveEnableFlag;

  SelectBoard(fpgaBoard);
  if (ReadEEPROM(&fm, fpgaM->TWIadd, 0, sizeof(FPGAmodule)) == 0)
  {
    if (strcmp(fm.Name, fpgaM->Name) == 0)
    {
      // Here if the name matches so copy the data to the operating data structure
      if (fm.Size > sizeof(FPGAmodule)) fm.Size = sizeof(FPGAmodule);
      fm.TWIadd = fpgaM->TWIadd;
      memcpy(fpgaM, &fm, fm.Size);
      return true;
    }
  }
  return false;
}

// This function is called at powerup to initiaize the FPGA board.
void FPGA_init(int8_t Board, int8_t addr)
{
  if(fpgaM != NULL) return;     // Only one module is supported
  fpgaM = new FPGAmodule;
  fpga  = new FPGAdata;
  fpga = &fpgaM->fpga;
  ADCvals = new float[8];
  *fpgaM = FPGA_Rev1;           // Set defaults
  fpgaBoard = Board;
  SelectBoard(Board);
  fpgaM->TWIadd = addr;
  if(NormalStartup) RestoreFPGAsettings();
  // Init the FPGA
  FPGAinit();
  // Schedule the FPGA processing loop
  // Configure Threads
  FPGAthread.setName("FPGA");
  FPGAthread.onRun(FPGA_loop);
  FPGAthread.setInterval(100);
  // Add threads to the controller
  control.add(&FPGAthread);
  // Add FPGA command to command processor
  AddToCommandList(&FPGACmdList);
}

// FPGA processing loop
void FPGA_loop(void)
{
  if((fpga->cfg & (1 << RUNADC)) != 0)
  {
    // Read all 8 ADC channels
    readADC;
    // Calibrate and fill the ADC voltage array
    for(int i=0;i<8;i++) ADCvals[i] = Counts2Value(fpga->adc[i], &fpgaM->adc[i]);
  }
}

//
// Host command processing functions
//

bool setVariable(int *v,int value, int LL, int UL)
{
  if((value<LL)||(value>UL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  *v = value;
  return true;
}

bool setVariable(bool *v,char *value)
{
  float   d;
  String  T=value;

  if(T == "TRUE") *v = true;
  else if(T == "FALSE") *v = false;
  else
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  SendACK;
  return true;
}

void setFPGAconfig(char *val) { int ival; sscanf(val,"%x", &ival); fpga->cfg = ival; writeFPGAcfg; SendACK; }
void getFPGAconfig(void) { readFPGAcfg; SendACKonly; if(!SerialMute) serial->println(fpga->cfg,HEX); }
void setFPGAbusControl(char *val)
{
  bool result;
  int b = DCbiasCH2Brd(fpgaM->DCBfirst-1);
  if(setVariable(&result,val))
  {
    if(result) 
    {
      setBUSCTRL(fpga->cfg,1);
      if(b!=-1) DCBtstMask[b] = ~fpga->DACmask;  // Mask the channels we will be controlling from ADCs
    }
    else 
    {
      setBUSCTRL(fpga->cfg,0); 
      DCbiasUpdate=true;              // Update all DCBias channels
      if(b!=-1) DCBtstMask[b] = 0;    // Update DCBias test mask
    }
    writeFPGAcfg;
  }
}
void getFPGAbusControl(void)
{
  SendACKonly;
  if(SerialMute) return;
  readFPGAcfg;
  if(((1 << BUSCTRL) & fpga->cfg) != 0) serial->println("TRUE");
  else serial->println("FALSE");
}
void setFPGAprocess(char *val)
{
  bool result;
  if(setVariable(&result,val))
  {
    if(result) setRUNCAL(fpga->cfg,1)
    else setRUNCAL(fpga->cfg,0)
    writeFPGAcfg;
  }
}
void getFPGAprocess(void)
{
  SendACKonly;
  if(SerialMute) return;
  readFPGAcfg;
  if(((1 << RUNCAL) & fpga->cfg) != 0) serial->println("TRUE");
  else serial->println("FALSE");
}

void setFPGAadcUpdate(char *val)
{
  bool result;
  if(setVariable(&result,val))
  {
    if(result) setRUNADC(fpga->cfg,1)
    else setRUNADC(fpga->cfg,0)
    writeFPGAcfg;
  }
}
void getFPGAadcUpdate(void)
{
  SendACKonly;
  if(SerialMute) return;
  readFPGAcfg;
  if(((1 << RUNADC) & fpga->cfg) != 0) serial->println("TRUE");
  else serial->println("FALSE");
}


void setFPGAaddress(int val)
{
  int result;
  if(setVariable(&result,val,0,7))
  {
    readFPGAcfg;
    setADDR(fpga->cfg,result);
    writeFPGAcfg;
  }
}
void getFPGAaddress(void)
{
  SendACKonly;
  if(SerialMute) return;
  readFPGAcfg;
  serial->println((fpga->cfg >> ADDR0bit) & 0x07);
}

// Arguments are in ring buffer.
// Array, index, value
// Array = M,B,ADC,DAC
void setFPGAarrayValue(void)
{
  char        *tkn;
  String      arg;
  FPGA_array  array;
  int         index,val;

  while(true)
  {
     // Read and validate the parameters
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     if(arg == "M") array = FPGA_M;
     else if(arg == "B") array = FPGA_B;
     else if(arg == "ADC") array = FPGA_ADC;
     else if(arg == "DAC") array = FPGA_DAC;
     else break;
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     index = arg.toInt();
     if((index < 1) || (index >8)) break;
     index--;
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     val = arg.toInt();
     writeFPGAarrayElement(array,index,val);
     SendACK;
     return;
  }
  // Error exit
  BADARG;
}

void getFPGAarrayValue(char *arry, char *indx)
{
  FPGA_array  array;
  String      arg;
  int         index,val;

  while(1)
  {
    arg = arry;
    if(arg == "M") array = FPGA_M;
    else if(arg == "B") array = FPGA_B;
    else if(arg == "ADC") array = FPGA_ADC;
    else if(arg == "DAC") array = FPGA_DAC;
    else break;
    arg = indx;
    index = arg.toInt();
    if((index < 1) || (index >8)) break;
    index--;
    readFPGAarrayElement(array,index,&val);
    SendACKonly;
    if(!SerialMute) serial->println(val);
    return;
  }
  // Error exit
  BADARG;
}

void setDACmask(char *val) {int i; sscanf(val,"%x",&i); fpga->DACmask = i; writeFPGAdacMSK; SendACK;}
void getDACmask(void) {SendACKonly; readFPGAdacMSK; if(!SerialMute) serial->println(fpga->ADCmask,HEX);}
void setADCmask(char *val) {int i; sscanf(val,"%x",&i); fpga->ADCmask = i; writeFPGAadcMSK; SendACK;}
void getADCmask(void) {SendACKonly; readFPGAadcMSK; if(!SerialMute) serial->println(fpga->ADCmask,HEX);}

void reportAllADC(void)
{
  SendACKonly;
  if((fpga->cfg & (1 << RUNADC)) == 0) return;
  if(SerialMute) return;
  for(int i=0;i<8;i++) serial->println(ADCvals[i]);
}
void reportADC(int chan)
{
  if((chan<1) || (chan>8)) BADARG;
  SendACKonly;
  if(!SerialMute) serial->println(ADCvals[chan-1]);
}
void calADCchan(int chan)
{
  if((chan<1) || (chan>8)) BADARG;
  chan--;
  // Apply voltage 1 and read the ADC channels raw data
  float val1 = UserInputFloat("Apply voltage 1 and enter value : ", ReadAllSerial);
  int adc1   = readADCaverage(chan, 10);
  // Apply voltage 2 and read the ADC channels raw data
  float val2 = UserInputFloat("Apply voltage 2 and enter value : ", ReadAllSerial);
  int adc2   = readADCaverage(chan, 10);
  // Calculate and apply calibration parameters
  fpgaM->adc[chan].m = ((float)adc1 - (float)adc2) / (val1 - val2);
  fpgaM->adc[chan].b = (float)adc1 - fpgaM->adc[chan].m * val1;
  serial->println(fpgaM->adc[chan].m);
  serial->println(fpgaM->adc[chan].b);
}

void calDACreflect(int firstDCBch)
{
  float adc1,adc2,dac1,dac2;
  float m,b;

  if((firstDCBch != 1) && (firstDCBch != 9)) BADARG;
  int chan = firstDCBch-1;
  fpgaM->DCBfirst = firstDCBch;
  for(int i=0;i<8;i++)
  {
    dac1 = DCbiasValue2Counts(chan + i,0);
    dac2 = DCbiasValue2Counts(chan + i,125);
    adc1 = Value2Counts(0, &fpgaM->adc[i]);
    adc2 = Value2Counts(5, &fpgaM->adc[i]);
    // dac = (adc * m) / 1024 + b
    //m = (adc2 - adc1)  * 1024.0 / (dac2 - dac1);
    m = (dac2 - dac1)  * 1024.0 / (adc2 - adc1);
    b = dac2 - adc2 * m / 1024.0;
    fpga->m[i] = (m + 0.5);
    fpga->b[i] = (b + 0.5);
    fpga->b[i] = (int)dac1 - ((int)adc1 * fpga->m[i])/1024;
  }
  writem;
  writeb;
}

void saveFPGA(void) {if(SaveFPGAsettings()) {SendACK;} else BADARG;}
void restoreFPGA(void) {if(RestoreFPGAsettings()) {SendACK;} else BADARG;}
