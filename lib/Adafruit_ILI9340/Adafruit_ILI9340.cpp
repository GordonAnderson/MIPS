/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ILI9340.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include "AtomicBlock.h"
#include <SPI.h>

#if defined(__SAM3X8E__)
#include <include/pio.h>
  #define SET_BIT(port, bitMask) (port)->PIO_SODR |= (bitMask)
  #define CLEAR_BIT(port, bitMask) (port)->PIO_CODR |= (bitMask)
#endif
#ifdef __AVR__
  #define SET_BIT(port, bitMask) *(port) |= (bitMask)
  #define CLEAR_BIT(port, bitMask) *(port) &= ~(bitMask)
#endif

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t mosi,
				   uint8_t sclk, uint8_t rst, uint8_t miso) : Adafruit_GFX(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
  
  DisplayType = ILI9340_DSP;


}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst) : Adafruit_GFX(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
  
  DisplayType = ILI9340_DSP;
}

void Adafruit_ILI9340::SetDisplayType(uint8_t dsp_type) 
{ 
   DisplayType = dsp_type;  
}

inline uint8_t Adafruit_ILI9340::spiwrite(uint8_t c) {
  if(disable) return(c);

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

AtomicBlock< Atomic_RestoreState > a_Block;
  CLEAR_BIT(csport, cspinmask);
  if (hwSPI) {
#ifdef __AVR__
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
#endif
#if defined(__SAM3X8E__)
    static uint32_t ch = SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS));
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | c;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	c = SPI0->SPI_RDR; 
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
        //digitalWrite(_mosi, HIGH); 
        SET_BIT(mosiport, mosipinmask);
      } else {
        //digitalWrite(_mosi, LOW); 
        CLEAR_BIT(mosiport, mosipinmask);
      }
      //digitalWrite(_sclk, HIGH);
      SET_BIT(clkport, clkpinmask);
      //digitalWrite(_sclk, LOW);
      CLEAR_BIT(clkport, clkpinmask);
    }
  }
  SET_BIT(csport, cspinmask);
  return(c);
}

inline void Adafruit_ILI9340::spiwrite16(uint16_t w) 
{
  uint8_t c;
  
  if(disable) return;
  
  AtomicBlock< Atomic_RestoreState > a_Block;
  CLEAR_BIT(csport, cspinmask);
  static uint32_t ch = SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS));
  while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
  SPI0->SPI_TDR = ch | (w >> 8);
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
  c = SPI0->SPI_RDR; 

  while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
  SPI0->SPI_TDR = ch | (w & 0xFF);
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
  c = SPI0->SPI_RDR; 
  
  SET_BIT(csport, cspinmask);
}



void Adafruit_ILI9340::writecommand(uint8_t c) {
//  if(disable) return;
  CLEAR_BIT(dcport, dcpinmask);
  CLEAR_BIT(clkport, clkpinmask);
  spiwrite(c);
}


void Adafruit_ILI9340::writedata(uint8_t c) {
//  if(disable) return;
  SET_BIT(dcport,  dcpinmask);
  CLEAR_BIT(clkport, clkpinmask);
  spiwrite(c);
} 

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9340::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  if(disable) return;
  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

void Adafruit_ILI9340::disableDisplay(bool state)
{
   disable = state;
}

static const uint8_t ILI9340_regValues[] PROGMEM = 
{
  21,
  0xEF,3,0x03,0x80,0x02,
  0xCF,3,0x00,0XC1,0X30,
  0xED,4,0x64,0x03,0X12,0X81,
  0xE8,3,0x85,0x00,0x78,
  0xCB,5,0x39,0x2C,0x00,0x34,0x02,
  0xF7,1,0x20,
  0xEA,2,0x00,0x00,
  ILI9340_PWCTR1,1,0x23,				//Power control 
  ILI9340_PWCTR2,1,0x10,    			//Power control, SAP[2:0];BT[3:0]
  ILI9340_VMCTR1,2,0x3e,0x28,    		//VCM control 
  ILI9340_VMCTR2,1,0x86,    			//VCM control2 
  ILI9340_MADCTL,1,ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR,   // Memory Access Control
  ILI9340_PIXFMT,1,0x55,
  ILI9340_FRMCTR1,2,0x00,0x18,
  ILI9340_DFUNCTR,3,0x08,0x82,0x27,  	// Display Function Control 
  0xF2,1,0x00,    		 				// 3Gamma Function Disable
  ILI9340_GAMMASET,1,0x01,    			//Gamma curve selected 
  //Set Gamma 
  ILI9340_GMCTRP1,15,0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00,
  //Set Gamma 
  ILI9340_GMCTRN1,15,0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F,
  ILI9340_SLPOUT,0 | DELAY,  			//Exit Sleep 
  120, 		
  ILI9340_DISPON,0,    //Display on   
//  0x53,1,0x2C,
//  0x51,1,0x00
};

void Adafruit_ILI9340::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
  if (DisplayType == HX8347_DSP) 
  {
     HX8347setAddrWindow(x0,y0,x1,y1);
     return;
  }  
#if defined(__SAM3X8E__)
//  if(disable) return;
  writecommand(ILI9340_CASET); // Column addr set
  SET_BIT(dcport,  dcpinmask);
  CLEAR_BIT(clkport, clkpinmask);
  spiwrite(x0 >> 8);
  spiwrite(x0 & 0xFF);
  spiwrite(x1 >> 8);
  spiwrite(x1 & 0xFF);
  
  writecommand(ILI9340_PASET); // Row addr set
  SET_BIT(dcport,  dcpinmask);
  CLEAR_BIT(clkport, clkpinmask);
  spiwrite(y0>>8);
  spiwrite(y0);
  spiwrite(y1>>8);
  spiwrite(y1);

  writecommand(ILI9340_RAMWR); // write to RAM
  return;
#endif

  writecommand(ILI9340_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9340_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9340_RAMWR); // write to RAM
}


void Adafruit_ILI9340::pushColor(uint16_t color) {
  SET_BIT(dcport, dcpinmask);
  spiwrite16(color);
}

void Adafruit_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);
  SET_BIT(dcport, dcpinmask);
  spiwrite16(color);
}


void Adafruit_ILI9340::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SET_BIT(dcport, dcpinmask);
  while (h--) {
    spiwrite16(color);
  }
}


void Adafruit_ILI9340::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  SET_BIT(dcport, dcpinmask);
  while (w--) {
    spiwrite16(color);
  }
}

void Adafruit_ILI9340::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9340::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SET_BIT(dcport, dcpinmask);
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite16(color);
    }
  }
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9340::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Adafruit_ILI9340::setRotation(uint8_t m) {

  if (DisplayType == HX8347_DSP) 
  {
     HX8347setRotation(m);
     return;
  }  
  writecommand(ILI9340_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
     break;
   case 1:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  case 2:
    writedata(ILI9340_MADCTL_MY | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
    break;
   case 3:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_MY | ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  }
}


void Adafruit_ILI9340::invertDisplay(boolean i) 
{
  if (DisplayType == HX8347_DSP) 
  {
     HX8347invertDisplay(i);
     return;
  }  
  writecommand(i ? ILI9340_INVON : ILI9340_INVOFF);
}


////////// stuff not actively being used, but kept for posterity


uint8_t Adafruit_ILI9340::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#ifdef __AVR__
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
#endif
#if defined(__SAM3X8E__)
    r = SPI.transfer(0x00);
#endif
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

 uint8_t Adafruit_ILI9340::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
 
/*
 uint8_t Adafruit_ILI9340::readcommand8(uint8_t c) {
//   digitalWrite(_dc, LOW);
//   digitalWrite(_sclk, LOW);
//   digitalWrite(_cs, LOW);
   CLEAR_BIT(dcport, dcpinmask);
   CLEAR_BIT(clkport, clkpinmask);
   CLEAR_BIT(csport, cspinmask);
   spiwrite(c);
 
//   digitalWrite(_dc, HIGH);
   SET_BIT(dcport, dcpinmask);
//   uint8_t r = spiread();
   uint8_t r = spiwrite(0);
//   digitalWrite(_cs, HIGH);
   SET_BIT(csport, cspinmask);
   return r;
}
*/

uint8_t Adafruit_ILI9340::readcommand8(uint8_t c)
{ 
    CLEAR_BIT(dcport, dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    CLEAR_BIT(csport, cspinmask);
    static uint32_t ch = SPI_PCS(BOARD_PIN_TO_SPI_CHANNEL(BOARD_SPI_DEFAULT_SS));
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | c;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	c = SPI0->SPI_RDR; 



    SET_BIT(dcport, dcpinmask);
    
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | 0x00;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	uint8_t r = SPI0->SPI_RDR; 

	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | 0x00;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	r = SPI0->SPI_RDR; 

	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | 0x00;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	r = SPI0->SPI_RDR; 
	
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | 0x00;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	r = SPI0->SPI_RDR; 
	
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
	SPI0->SPI_TDR = ch | 0x00;
	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
	r = SPI0->SPI_RDR; 
//	r = SPI0->SPI_RDR;
//	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
//	SPI0->SPI_TDR = ch | 0xFF;
//	while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
//    r = SPI0->SPI_RDR; 
    SET_BIT(csport, cspinmask);
    return r;
}
 
/*

 uint16_t Adafruit_ILI9340::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ILI9340::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 
 
 return r;
 }
 
 */

// 
// Functions that support the HX8347D display
//

#define HX8347G_COLADDRSTART_HI    0x02
#define HX8347G_COLADDRSTART_LO    0x03
#define HX8347G_COLADDREND_HI      0x04
#define HX8347G_COLADDREND_LO      0x05
#define HX8347G_ROWADDRSTART_HI    0x06
#define HX8347G_ROWADDRSTART_LO    0x07
#define HX8347G_ROWADDREND_HI      0x08
#define HX8347G_ROWADDREND_LO      0x09
#define HX8347G_MEMACCESS          0x16
#define HX8347G_MEMWRITE           0x22
#define HX8347G_MEMREAD            0x22

 static const uint8_t HX8347G_regValues[] PROGMEM = {
    60,
	0xEA,1,0x00, //PTBA[15:8]
	0xEB,1,0x20, //PTBA[7:0]
	0xEC,1,0x0C, //STBA[15:8]
	0xED,1,0xC4, //STBA[7:0]
	0xE8,1,0x38, //OPON[7:0]
	0xE9,1,0x10, //OPON1[7:0]
	0xF1,1,0x01, //OTPS1B
	0xF2,1,0x10, //GEN
	//Gamma 2.2 Setting
	0x40,1,0x01, //
	0x41,1,0x00, //
	0x42,1,0x00, //
	0x43,1,0x10, //
	0x44,1,0x0E, //
	0x45,1,0x24, //
	0x46,1,0x04, //
	0x47,1,0x50, //
	0x48,1,0x02, //
	0x49,1,0x13, //
	0x4A,1,0x19, //
	0x4B,1,0x19, //
	0x4C,1,0x16, //
	0x50,1,0x1B, //
	0x51,1,0x31, //
	0x52,1,0x2F, //
	0x53,1,0x3F, //
	0x54,1,0x3F, //
	0x55,1,0x3E, //
	0x56,1,0x2F, //
	0x57,1,0x7B, //
	0x58,1,0x09, //
	0x59,1,0x06, //
	0x5A,1,0x06, //
	0x5B,1,0x0C, //
	0x5C,1,0x1D, //
	0x5D,1,0xCC, //
	//Power Voltage Setting
	0x1B,1,0x1B, //VRH=4.65V
	0x1A,1,0x01, //BT (VGH~15V,VGL~-10V,DDVDH~5V)
	0x24,1,0x2F, //VMH(VCOM High voltage ~3.2V)
	0x25,1,0x57, //VML(VCOM Low voltage -1.2V)
//	****VCOM offset**
	0x23,1,0x88, //for Flicker adjust //can reload from OTP
	//Power on Setting
	0x18,1,0x34, //I/P_RADJ,N/P_RADJ, Normal mode 60Hz
	0x19,1,0x01, //OSC_EN='1', start Osc
	0x01,1,0x00, //DP_STB='0', out deep sleep
	0x1F,1 | DELAY,0x88,// GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
	5,
	0x1F,1 | DELAY,0x80,// GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
	5,
	0x1F,1 | DELAY,0x90,// GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
	5,
	0x1F,1 | DELAY,0xD0,// GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
	5,
	//262k/65k color selection
	0x17,1,0x05, //default 0x06 262k color // 0x05 65k color
	//SET PANEL
	0x36,1,0x00, //SS_P, GS_P,REV_P,BGR_P
	//Display ON Setting
	0x28,1 | DELAY,0x38, //GON=1, DTE=1, D=1000
	40,
	0x28,1,0x3F, //GON=1, DTE=1, D=1100
	0x16,1,0x18,
	//Set GRAM Area
	0x02,1,0x00,
	0x03,1,0x00, //Column Start
	0x04,1,0x00,
	0x05,1,0xEF, //Column End
	0x06,1,0x00,
	0x07,1,0x00, //Row Start
	0x08,1,0x01,
	0x09,1,0x3F, //Row End
};

void Adafruit_ILI9340::HX8347setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    writecommand(HX8347G_COLADDRSTART_HI);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(x >> 8);
    writecommand(HX8347G_COLADDRSTART_HI+1);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(x & 0xFF);
    writecommand(HX8347G_COLADDREND_HI);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(x1 >> 8);
    writecommand(HX8347G_COLADDREND_HI+1);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(x1 & 0xFF);
    
    writecommand(HX8347G_ROWADDRSTART_HI);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(y >> 8);
    writecommand(HX8347G_ROWADDRSTART_HI+1);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(y & 0xFF);
    writecommand(HX8347G_ROWADDREND_HI);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(y1 >> 8);
    writecommand(HX8347G_ROWADDREND_HI+1);
    SET_BIT(dcport,  dcpinmask);
    CLEAR_BIT(clkport, clkpinmask);
    spiwrite(y1 & 0xFF);

    writecommand(HX8347G_MEMWRITE);
}

void Adafruit_ILI9340::HX8347setRotation(uint8_t r)
{
    uint16_t mac = 0x0800;
    Adafruit_GFX::setRotation(r & 3);
    // rotation = r % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            mac = 0x0800;   // BGR=1
            break;
        case 1:
            mac = 0x6800;   //MY=0, MX=1, MV=0, ML=0, BGR=1
            break;
        case 2:
            mac = 0xD800;   //MY=1, MX=1, MV=0, ML=1, BGR=1
            break;
        case 3:
            mac = 0xB800;   //MY=1, MX=0, MV=1, ML=1, BGR=1
            break;
    }
    writecommand(HX8347G_MEMACCESS);
    writedata(mac >> 8);
}

void Adafruit_ILI9340::HX8347invertDisplay(bool i)
{
    uint8_t val = 0x00;
	if (i) val |= 2;
	else val &= ~2;
    writecommand(0x36);
    writedata(val);
}

void Adafruit_ILI9340::begin(void) {

  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
#ifdef __AVR__
  csport    = portOutputRegister(digitalPinToPort(_cs));
  dcport    = portOutputRegister(digitalPinToPort(_dc));
#endif
#if defined(__SAM3X8E__)
  csport    = digitalPinToPort(_cs);
  dcport    = digitalPinToPort(_dc);
#endif
  cspinmask = digitalPinToBitMask(_cs);
  dcpinmask = digitalPinToBitMask(_dc);

  disable = false;
  if(hwSPI) { // Using hardware SPI
    SPI.begin();
#ifdef __AVR__
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#endif
#if defined(__SAM3X8E__)
    SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
#endif    SPI.setBitOrder(MSBFIRST);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
#ifdef __AVR__
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
#endif
#if defined(__SAM3X8E__)
    clkport     = digitalPinToPort(_sclk);
    mosiport    = digitalPinToPort(_mosi);
#endif
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosipinmask = digitalPinToBitMask(_mosi);
    CLEAR_BIT(clkport, clkpinmask);
    CLEAR_BIT(mosiport, mosipinmask);
  }

  // toggle RST low to reset

  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(150);
  
  // Issue the init sequence
  
  if (DisplayType == ILI9340_DSP) commandList((uint8_t *) ILI9340_regValues);
  if (DisplayType == HX8347_DSP) commandList((uint8_t *) HX8347G_regValues);
}



