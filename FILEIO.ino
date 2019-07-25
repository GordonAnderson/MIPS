//
// BMP image loading functions
//

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

// MIPS display is 320 x 240 pixels in size.
// This function requires the bmp file to be 24 bit
// depth and use no compression.

#define BUFFPIXEL 20

// Returns false if any error is detected
bool bmpDraw(char *filename, uint8_t x, uint8_t y) 
{
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0;

  if((x >= tft.width()) || (y >= tft.height())) return(false);

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL)
  {
    return(false);
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) 
  { // BMP signature
    read32(bmpFile);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    // Read DIB header
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) 
    { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) 
      { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;
        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) 
        {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) 
        { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) 
          { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          for (col=0; col<w; col++) 
          { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) 
            { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }
            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }
  bmpFile.close();
  if(!goodBmp) return(false);
  return(true);
}

// This function reports details of the bmp immage file as well
// defining the MIPS display requirements.

void bmpReport(char *filename) 
{
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  boolean  goodBmp = false;       // Set to true on valid header parse


  serial->println();
  serial->println("MIPS display details:");
  serial->println("  display size: 320 x 240");
  serial->println("  color graphics");
  serial->println("  bmp file must have 24 bit depth");
  serial->println("  and use no compression");

  
  serial->println();
  serial->print("Image file '");
  serial->print(filename);
  serial->println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL)
  {
    serial->print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) 
  { // BMP signature
    serial->print("File size: "); serial->println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    serial->print("Image Offset: "); serial->println(bmpImageoffset, DEC);
    // Read DIB header
    serial->print("Header size: "); serial->println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) 
    { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      serial->print("Bit Depth: "); serial->println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) 
      { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        serial->print("Image size: ");
        serial->print(bmpWidth);
        serial->print('x');
        serial->println(bmpHeight);
        serial->println("BMP format is valid.");
      } // end goodBmp
    }
  }
  bmpFile.close();
  if(!goodBmp) serial->println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) 
{
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) 
{
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

//
// End of BMP functions
//

// The following function support reading and writting the EEPROM module memory to
// the SD card or to the MIPS host app using the USB interface.

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Compute CRC one byte at a time
void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// This function reads the EEPROM contents from the selected module and writes
// the data to a file on the SD card. If the file is present it will be overwritten.
// Returns 0 is operation completes with no errors.
int SaveEEPROMtoSD(char *FileName, uint8_t board, uint8_t EEPROMadd)
{
  uint8_t buf[512];
  File file;
    
  // Read the EEPROM
  SelectBoard(board);
  if(ReadEEPROM(buf, EEPROMadd, 0, 512) != 0) return ERR_EEPROMREAD;
  // Write the data to SD card
  // Test SD present flag, exit and NAK if no card or it failed to init
  if(!SDcardPresent) return ERR_NOSDCARD;
  AtomicBlock< Atomic_RestoreState > a_Block;
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(FileName);
  // Open file and write config structure to disk
  if(!(file=SD.open(FileName,FILE_WRITE))) return ERR_CANTCREATEFILE;
  file.write((byte *)buf,512);
  file.close();
  return(0);
}

// This function writes the EEPROM contents on the selected module from the
// the selected file on the SD card. 
// Returns 0 is operation completes with no errors.
int LoadEEPROMfromSD(char *FileName, uint8_t board, uint8_t EEPROMadd)
{
  uint8_t buf[512];
  int     i,fVal;
  File    file;
    
  // Read file from SD card
  if(!SDcardPresent) return ERR_NOSDCARD;
  {
     AtomicBlock< Atomic_RestoreState > a_Block;
     SD.begin(_sdcs);
     // Open the file
     if(!(file=SD.open(FileName,FILE_READ))) return ERR_CANTOPENFILE;
     // read the data
     for(i=0;i<512;i++)
     {
       if((fVal = file.read()) == -1) break;
       buf[i] = fVal;
     }
     file.close();
  }
  if(i != 512) return ERR_READINGSD;
  // Write data to EEPROM
  SelectBoard(board);  
  if(WriteEEPROM(buf, EEPROMadd, 0, 512) != 0) return ERR_EEPROMWRITE;
  return 0;
}

// This function saves the EEPROM data to the SD card.
// This function is called by the serial command processor with the parameters
// in the ring buffer. This routine expects the following:
// FileName
// Module address, A or B
// EEPROM TWI address, hex
void EEPROMtoSD(void)
{
   char   *Token,Module;
   String sToken,FileName;
   int    add,err,brd;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     FileName = Token;
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     Module = toupper(Token[0]);
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sscanf(Token,"%x",&add);
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the Module range
     if((Module != 'A') && (Module != 'B')) break;
     // Now we can call the function!
     brd = 0;
     if(Module == 'B') brd = 1; 
     if((err = SaveEEPROMtoSD((char *)FileName.c_str(), brd, add)) != 0)
     {
       SetErrorCode(err);
       SendNAK;
       return;
     }
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

// This function restores the EEPROM data from the SD card.
// This function is called by the serial command processor with the parameters
// in the ring buffer. This routine expects the following:
// FileName
// Module address, A or B
// EEPROM TWI address, hex
void SDtoEEPROM(void)
{
   char   *Token,Module;
   String sToken,FileName;
   int    add,err,brd;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     FileName = Token;
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     Module = toupper(Token[0]);
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sscanf(Token,"%x",&add);
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the Module range
     if((Module != 'A') && (Module != 'B')) break;
     // Now we can call the function!
     brd = 0;
     if(Module == 'B') brd = 1; 
     if((err = LoadEEPROMfromSD((char *)FileName.c_str(), brd, add)) != 0)
     {
       SetErrorCode(err);
       SendNAK;
       return;
     }
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;  
}

// Lists all files found on the SD card
void ListFiles(void)
{
  File root, entry;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState > a_Block;
    SD.begin(_sdcs);
    root = SD.open("/", FILE_READ);
    root.rewindDirectory();
  }
  SendACKonly;
  while (true)
  {
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      entry = root.openNextFile();
    }
    if (!entry) break;
    serial->print(entry.name());
    serial->print(", ");
    serial->println(entry.size());
  }
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      root.close();  
    }
}

File root;
int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

void rm(File dir, String tempPath) {
  while(true) {
    WDT_Restart(WDT);
    File entry =  dir.openNextFile();
    String localPath;

    serial->println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if( SD.rmdir( folderBuf ) )
        {
          serial->print("Deleted folder ");
          serial->println(folderBuf);
          FolderDeleteCount++;
        } 
        else
        {
          serial->print("Unable to delete folder ");
          serial->println(folderBuf);
          FailCount++;
        }
      } 
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if( SD.remove( charBuf ) )
        {
          serial->print("Deleted ");
          serial->println(localPath);
          DeletedCount++;
        } 
        else
        {
          serial->print("Failed to delete ");
          serial->println(localPath);
          FailCount++;
        }

      }
    } 
    else {
      // break out of recursion
      break;
    }
  }
}

void DeleteFile(char *FileName)
{
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  if(strcmp(FileName,"*") == 0)
  {
    // Delete all files on SD card

    root = SD.open("/");
    delay(2000);
    WDT_Restart(WDT);

    rm(root, rootpath);
    SendACK;
    return;
  }
  AtomicBlock< Atomic_RestoreState > a_Block;
  // Remove the existing default.cfg file
  SD.remove(FileName); 
  SendACK; 
}

// This function will send the selected file to the USB port. The file is assumed to
// be binary and its contents are converted to hex and send. after the ACK is sent.
// After the ACK, the files size is sent as an ascii string with a EOL termination, then
// the data block is sent as ascii hex followed by a EOL, and finally the 8 bit CRC is
// sent as a byte followed by a EOL.
void GetFile(char *FileName)
{
  byte     b,crc=0;
  int      i,fVal,fsize;
  File     file;
  char     sbuf[3];
  uint32_t start;
    
  // Open the file and read its size
  // Read file from SD card
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.begin(_sdcs);
    // Open the file
    if(!(file=SD.open(FileName,FILE_READ)))
    {
      SetErrorCode(ERR_CANTOPENFILE);
      SendNAK;
      return;
    }
    SendACK;
    serial->println(fsize = file.size());
  }
  // read the data
  for(i=0; i<fsize; i++)
  {
    {
      AtomicBlock< Atomic_RestoreState >   a_Block;
      fVal = file.read();
    }
    if(fVal == -1) break;
    b = fVal;
    ComputeCRCbyte(&crc,b);
    sprintf(sbuf,"%02x",b);
    serial->print(sbuf);
    if((i>0) && ((i%1024) == 0))
    {
      // Halt and wait for "Next" request. Timeout after
      // 10 sec with no ation and exit
      start = millis();
      while(RB.Commands == 0)
      {
         if(millis() > start + 10000)
         {
            AtomicBlock< Atomic_RestoreState >   a_Block;
            serial->println("\nFile sending to host timedout!");
            file.close();
            return;
         }
         ReadAllSerial();
      }
      GetToken(false);
      GetToken(false);
    }
  }
  serial->println("");
  serial->println(crc);
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
  }
  serial->print(FileName);
  serial->println(" file send to host!");
}

// The function will receive a file from the USB connected host. The file must be sent
// in hex and use the following format:
// First the file name and file size, in bytes (decimal) are sent. If the file can
// be created an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct the file will be saved.
// If the file is already present it will be overwitten.
void PutFile(char * FileName,char *Fsize)
{
  File   file;
  String sToken;
  int    numBytes,val,tcrc;
  char   c,buf[3],*Token;
  byte   b,crc=0;
  uint32_t start;
  
  if(!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.begin(_sdcs);
    // Remove the existing file
    SD.remove(FileName);
    // Open file and write config structure to disk
    if(!(file=SD.open(FileName,FILE_WRITE)))
    {
      SetErrorCode(ERR_CANTCREATEFILE);
      SendNAK;
      return;
    }
  }
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    b = val;
    {
      AtomicBlock< Atomic_RestoreState >   a_Block;
      file.write(b);
    }
    ComputeCRCbyte(&crc,b);
    WDT_Restart(WDT);
    if((i>0) && (numBytes > 512) && (((i+1)%512)==0)) serial->println("Next");
  }
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->print(FileName);
       serial->println(" file received from host and saved.");
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    SD.remove(FileName);
  }
  return;
TimeoutExit:
  {
    AtomicBlock< Atomic_RestoreState >   a_Block;
    file.close();
    SD.remove(FileName);
  }
  serial->println("\nFile receive from host timedout!");
  return;
}

// This function will save all the module's EEPROM data to files on the SD card.
// All modules found are saved with the following naming convention:
// Name_board_add where
// Name = first three chars for name found in EEPROM
// board = A or B
// add = hex address of EEPROM
void SaveAlltoSD(void)
{
  char    filename[10];
  uint8_t addr;
  char    signature[20];
  int     err;

  // Loop through all the addresses looking for signatures
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 20) == 0)
    {
      // Build file name
      for(int i=0; i<3; i++) filename[i] = signature[2+i];
      filename[3] = '_';
      filename[4] = 'A';
      filename[5] = '_';
      sprintf(&filename[6],"%02x",addr);
      // Save to SD
      serial->print("Saving: ");
      serial->println(filename);
      err = SaveEEPROMtoSD(filename, 0, addr);
      if(err != 0)
      {
        SetErrorCode(err);
        SendNAK;
        return;   
      }
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 20) == 0)
    {
      // Build file name
      for(int i=0; i<3; i++) filename[i] = signature[2+i];
      filename[3] = '_';
      filename[4] = 'B';
      filename[5] = '_';
      sprintf(&filename[6],"%02x",addr);
      // Save to SD
      serial->print("Saving: ");
      serial->println(filename);
      err = SaveEEPROMtoSD(filename, 1, addr);    
      if(err != 0)
      {
        SetErrorCode(err);
        SendNAK;
        return;   
      }
    }
  }
  SendACK;
}

// This function will search the SD card for files with the following format:
// Name_board_add where
// Name = first three chars for name found in EEPROM
// board = A or B
// add = hex address of EEPROM
// If found and the size is 512 then the files will be loaded into module EEPROM.
void LoadAllfromSD(void)
{
  File root, entry;
  int  board, addr;

  if (!SDcardPresent)
  {
    SetErrorCode(ERR_NOSDCARD);
    SendNAK;
    return;
  }
  SD.begin(_sdcs);
  root = SD.open("/", FILE_READ);
  SendACKonly;
  root.rewindDirectory();
  while (true)
  {
    if (!(entry = root.openNextFile())) break;
    // Test the name to see if its a valid filename
    if((strlen(entry.name()) == 8) && (entry.name()[3] == '_') && (entry.name()[5] == '_') && (entry.size() == 512))
    {
      if((toupper(entry.name()[4]) == 'A') || (toupper(entry.name()[4]) == 'B'))
      {
        board = 0;
        if(toupper(entry.name()[4]) == 'B') board = 1;
        sscanf(&entry.name()[6],"%x",&addr);
        serial->print("Writing: ");
        serial->println(entry.name());
        //entry.close();
        int err = LoadEEPROMfromSD(entry.name(), board, addr);    
        if(err != 0)
        {
          serial->print("Error writting EEPROM: ");
          serial->println(err);
        }
      }
    }
  }
  root.close();  
  SendACK;
}

// This function will send the selected EEPROM contents to the host using the
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
// brd is A or B
// add is 0x50, 0x52, 0x54, or 0x56
void EEPROMtoSerial(char *brd, char *add)
{
  char sbuf[3];
  int  addr,board;
  byte buf[512];

  sscanf(add,"%x",&addr);
  // Check the inputs and exit if error
  if(((toupper(brd[0]) != 'A') && (toupper(brd[0] != 'B')) || (addr < 0x50) || (addr > 0x56) || ((addr & 1) !=0)))
  {
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;       
  }
  board = 0;
  if(toupper(brd[0]) == 'B') board = 1;
  // Read the EEPROM data
  SelectBoard(board);
  if(ReadEEPROM(buf, addr, 0, 512) != 0) 
  {
    SetErrorCode(ERR_EEPROMREAD);
    SendNAK;
    return;
  }
  SendACK;
  // Send the filesize
  serial->println(512);
  // Send the data as hex
  for(int i=0; i<512; i++)
  {
    sprintf(sbuf,"%02x",buf[i]);
    serial->print(sbuf);
  }
  serial->println("");
  // Send the CRC then exit
  serial->println(ComputeCRC(buf,512));
}

// This function will receive the selected EEPROM contents from the host using the
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
// brd is A or B
// add is 0x50, 0x52, 0x54, or 0x56
void SerialtoEEPROM(char *brd, char *add)
{
  char sbuf[3],*Token,c;
  int  addr,board,numBytes,val,crc;
  byte buf[512];
  uint32_t start;

  sscanf(add,"%x",&addr);
  // Check the inputs and exit if error
  if(((toupper(brd[0]) != 'A') && (toupper(brd[0] != 'B')) || (addr < 0x50) || (addr > 0x56) || ((addr & 1) !=0)))
  {
      SetErrorCode(ERR_BADARG);
      SendNAK;
      return;       
  }
  board = 0;
  if(toupper(brd[0]) == 'B') board = 1;
  SendACK;
  start = millis();
  // Receive the number of bytes
  while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
  sscanf(Token,"%d",&numBytes); 
  GetToken(true); // Get the \n and toss
  // Read the data block
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sbuf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sbuf[1] = c;
    sbuf[2] = 0;
    sscanf(sbuf,"%x",&val);
    buf[i & 511] = val;
  }
  start = millis();
  // Now we should see an EOL, \n
  while((c = RB_Get(&RB)) == 0xFF) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
  if(c == '\n')
  {
    // Get CRC and test
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    sscanf(Token,"%d",&crc);
    while((Token = GetToken(true)) == NULL) { ReadAllSerial(); if(millis() > start + 10000) goto TimeoutS2E; }
    if((Token[0] == '\n') && (crc == ComputeCRC(buf,512)))
    {
      // Write the EEPROM buffer
      SelectBoard(board);  
      if(WriteEEPROM(buf, addr, 0, 512) == 0)
      {
        serial->println("EPROM data written!");
        return;
      }
    }
  }
  serial->println("Unable to write to EEPROM!");
  return;
TimeoutS2E:
  serial->println("\nEEPROM data receive from host timedout!");
  return;
}
