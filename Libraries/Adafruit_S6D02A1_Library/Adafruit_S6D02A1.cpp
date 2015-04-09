/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
// tweeked for LCD which uses a Samsung  S6D02A1 controller Jan 2014 RH

#include "Adafruit_S6D02A1.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}


// Constructor when using software SPI.  All output pins are configurable.
Adafruit_S6D02A1::Adafruit_S6D02A1(uint8_t cs, uint8_t rs, uint8_t sid,
 uint8_t sclk, uint8_t rst) : Adafruit_GFX(S6D02A1_TFTWIDTH, S6D02A1_TFTHEIGHT)
{
  _cs   = cs;
  _rs   = rs;
  _sid  = sid;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_S6D02A1::Adafruit_S6D02A1(uint8_t cs, uint8_t rs, uint8_t rst) :
    Adafruit_GFX(S6D02A1_TFTWIDTH, S6D02A1_TFTHEIGHT) {
  _cs   = cs;
  _rs   = rs;
  _rst  = rst;
  hwSPI = true;
  _sid  = _sclk = 0;
}

#if defined(CORE_TEENSY) && !defined(__AVR__)
#define __AVR__
#endif

#ifdef __AVR__
inline void Adafruit_S6D02A1::spiwrite(uint8_t c) {

  //Serial.println(c, HEX);

  if (hwSPI) {
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) *dataport |=  datapinmask;
      else        *dataport &= ~datapinmask;
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
    }
  }
}


void Adafruit_S6D02A1::writecommand(uint8_t c) {
  *rsport &= ~rspinmask;
  *csport &= ~cspinmask;

  //Serial.print("C ");
  spiwrite(c);

  *csport |= cspinmask;
}


void Adafruit_S6D02A1::writedata(uint8_t c) {
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
    
  //Serial.print("D ");
  spiwrite(c);

  *csport |= cspinmask;
} 
#endif //#ifdef __AVR__

#if defined(__SAM3X8E__)
inline void Adafruit_S6D02A1::spiwrite(uint8_t c) {
  
  //Serial.println(c, HEX);
  
  if (hwSPI) {
    SPI.transfer(c);
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) dataport->PIO_SODR |= datapinmask;
      else        dataport->PIO_CODR |= datapinmask;
      clkport->PIO_SODR |= clkpinmask;
      clkport->PIO_CODR |= clkpinmask;
    }
  }
}


void Adafruit_S6D02A1::writecommand(uint8_t c) {
  rsport->PIO_CODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
  
  //Serial.print("C ");
  spiwrite(c);
  
  csport->PIO_SODR  |=  cspinmask;
}


void Adafruit_S6D02A1::writedata(uint8_t c) {
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
  
  //Serial.print("D ");
  spiwrite(c);
  
  csport->PIO_SODR  |=  cspinmask;
} 
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    S6D02A1_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    S6D02A1_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    S6D02A1_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    S6D02A1_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    S6D02A1_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    S6D02A1_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    S6D02A1_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    S6D02A1_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    S6D02A1_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    S6D02A1_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    S6D02A1_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    S6D02A1_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    S6D02A1_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    S6D02A1_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    S6D02A1_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    S6D02A1_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    S6D02A1_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    S6D02A1_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    S6D02A1_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    S6D02A1_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    S6D02A1_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    S6D02A1_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    S6D02A1_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    S6D02A1_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    S6D02A1_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    S6D02A1_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    S6D02A1_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    S6D02A1_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    S6D02A1_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    S6D02A1_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    S6D02A1_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    S6D02A1_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    S6D02A1_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    S6D02A1_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    S6D02A1_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    S6D02A1_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    S6D02A1_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    S6D02A1_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    S6D02A1_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    S6D02A1_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    S6D02A1_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_S6D02A1::commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

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


// Initialization code common to both 'B' and 'R' type displays
void Adafruit_S6D02A1::commonInit(const uint8_t *cmdList) {
  colstart  = rowstart = 0; // May be overridden in init func

  pinMode(_rs, OUTPUT);
  pinMode(_cs, OUTPUT);
#ifdef __AVR__
  csport    = portOutputRegister(digitalPinToPort(_cs));
  rsport    = portOutputRegister(digitalPinToPort(_rs));
#endif
#if defined(__SAM3X8E__)
  csport    = digitalPinToPort(_cs);
  rsport    = digitalPinToPort(_rs);
#endif
  cspinmask = digitalPinToBitMask(_cs);
  rspinmask = digitalPinToBitMask(_rs);

  if(hwSPI) { // Using hardware SPI
    SPI.begin();
#ifdef __AVR__
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
    //Due defaults to 4mHz (clock divider setting of 21)
#endif
#if defined(__SAM3X8E__)
    SPI.setClockDivider(21); // 4 MHz
    //Due defaults to 4mHz (clock divider setting of 21), but we'll set it anyway 
#endif
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);
#ifdef __AVR__
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
#endif
#if defined(__SAM3X8E__)
    clkport     = digitalPinToPort(_sclk);
    dataport    = digitalPinToPort(_sid);
#endif
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
#ifdef __AVR__
    *clkport   &= ~clkpinmask;
    *dataport  &= ~datapinmask;
#endif
#if defined(__SAM3X8E__)
    clkport ->PIO_CODR  |=  clkpinmask; // Set control bits to LOW (idle)
    dataport->PIO_CODR  |=  datapinmask; // Signals are ACTIVE HIGH
#endif
  }

  // toggle RST low to reset; CS low so it'll listen to us
#ifdef __AVR__
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport ->PIO_CODR  |=  cspinmask; // Set control bits to LOW (idle)
#endif
  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

//  if(cmdList) commandList(cmdList);
// S6D02A1 init code copied from the 8051 sample code
		writecommand(0xf0);
		writedata(0x5a);
		writedata(0x5a);

		writecommand(0xfc);
		writedata(0x5a);
		writedata(0x5a);

		writecommand(0x26);
		writedata(0x01);

		writecommand(0xfa);
		writedata(0x02);
		writedata(0x1f);
		writedata(0x00);
		writedata(0x10);
		writedata(0x22);
		writedata(0x30);
		writedata(0x38);
		writedata(0x3A);
		writedata(0x3A);
		writedata(0x3A);
		writedata(0x3A);
		writedata(0x3A);
		writedata(0x3d);
		writedata(0x02);
		writedata(0x01);

		writecommand(0xfb);
		writedata(0x21);
		writedata(0x00);
		writedata(0x02);
		writedata(0x04);
		writedata(0x07);
		writedata(0x0a);
		writedata(0x0b);
		writedata(0x0c);
		writedata(0x0c);
		writedata(0x16);
		writedata(0x1e);
		writedata(0x30);
		writedata(0x3f);
		writedata(0x01);
		writedata(0x02);

		//////////////power setting sequence//////////
		writecommand(0xfd);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x17);
		writedata(0x10);
		writedata(0x00);
		writedata(0x01);
		writedata(0x01);
		writedata(0x00);
		writedata(0x1f);
		writedata(0x1f);

		writecommand(0xf4);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x3f);
		writedata(0x3f);
		writedata(0x07);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);
		//delay_ms(80);			   //ÐÂÔö

		writecommand(0xf5);
		writedata(0x00);
		writedata(0x70);//39
		writedata(0x66);//3a
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x6d);//38
		writedata(0x66);//38
		writedata(0x06);

		writecommand(0xf6);
		writedata(0x02);
		writedata(0x00);
		writedata(0x3f);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x02);
		writedata(0x00);
		writedata(0x06);
		writedata(0x01);
		writedata(0x00);

		writecommand(0xf2);
		writedata(0x00);
		writedata(0x01);//04
		writedata(0x03);
		writedata(0x08);
		writedata(0x08);
		writedata(0x04);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x01);
		writedata(0x00);
		writedata(0x00);
		writedata(0x04);
		writedata(0x08);
		writedata(0x08);

		writecommand(0xf8);
		writedata(0x11);//66

		writecommand(0xf7);
		writedata(0xc8);
		writedata(0x20);
		writedata(0x00);
		writedata(0x00);

		writecommand(0xf3);
		writedata(0x00);
		writedata(0x00);

		writecommand(0x11);
		delay(50);

		writecommand(0xf3);
		writedata(0x00);
		writedata(0x01);
		delay(50);
		writecommand(0xf3);
		writedata(0x00);
		writedata(0x03);
		delay(50);
		writecommand(0xf3);
		writedata(0x00);
		writedata(0x07);
		delay(50);
		writecommand(0xf3);
		writedata(0x00);
		writedata(0x0f);
		delay(50);

		writecommand(0xf4);
		writedata(0x00);
		writedata(0x04);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x3f);
		writedata(0x3f);
		writedata(0x07);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);
		delay(50);

		writecommand(0xf3);
		writedata(0x00);
		writedata(0x1f);
		delay(50);
		writecommand(0xf3);
		writedata(0x00);
		writedata(0x7f);
		delay(50);

		writecommand(0xf3);
		writedata(0x00);
		writedata(0xff);
		delay(50);

		writecommand(0xfd);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x17);
		writedata(0x10);
		writedata(0x00);
		writedata(0x00);
		writedata(0x01);
		writedata(0x00);
		writedata(0x16);
		writedata(0x16);

		writecommand(0xf4);
		writedata(0x00);
		writedata(0x09);
		writedata(0x00);
		writedata(0x00);
		writedata(0x00);
		writedata(0x3f);
		writedata(0x3f);
		writedata(0x07);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);
		writedata(0x3C);
		writedata(0x36);
		writedata(0x00);

		/////////////initializing sequence/////////////

		writecommand( 0x36);   
		writedata( 0x08); 

		writecommand(0x35);
		writedata(0x00);
		writecommand(0x3a);
		writedata(0x05);


		/////////////////gamma setting sequence/////////

		delay(150);	
		writecommand(0x29);
		writecommand(0x2c);

}


// Initialization for S6D02A1B screens
void Adafruit_S6D02A1::initB(void) {
  commonInit(Bcmd);
}


// Initialization for S6D02A1R screens (green or red tabs)
void Adafruit_S6D02A1::initR(uint8_t options) {
  commonInit(Rcmd1);
  if(options == INITR_GREENTAB) {
    commandList(Rcmd2green);
    colstart = 2;
    rowstart = 1;
  } else {
    // colstart, rowstart left at default '0' values
    commandList(Rcmd2red);
  }
  commandList(Rcmd3);

  // if black, change MADCTL color filter
  if (options == INITR_BLACKTAB) {
    writecommand(S6D02A1_MADCTL);
    writedata(0xC0);
  }

  tabcolor = options;
}


void Adafruit_S6D02A1::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  writecommand(S6D02A1_CASET); // Column addr set
  writedata(0x00);
  writedata(x0+colstart);     // XSTART 
  writedata(0x00);
  writedata(x1+colstart);     // XEND

  writecommand(S6D02A1_RASET); // Row addr set
  writedata(0x00);
  writedata(y0+rowstart);     // YSTART
  writedata(0x00);
  writedata(y1+rowstart);     // YEND

  writecommand(S6D02A1_RAMWR); // write to RAM
}


void Adafruit_S6D02A1::pushColor(uint16_t color) {
#ifdef __AVR__
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
#endif
  
  spiwrite(color >> 8);
  spiwrite(color);

#ifdef __AVR__
  *csport |= cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport->PIO_SODR  |=  cspinmask;
#endif
}

void Adafruit_S6D02A1::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

#ifdef __AVR__
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
#endif
  
  spiwrite(color >> 8);
  spiwrite(color);

#ifdef __AVR__
  *csport |= cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport->PIO_SODR  |=  cspinmask;
#endif
}


void Adafruit_S6D02A1::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;
#ifdef __AVR__
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
#endif
  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
#ifdef __AVR__
  *csport |= cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport->PIO_SODR  |=  cspinmask;
#endif
}


void Adafruit_S6D02A1::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
#ifdef __AVR__
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
#endif
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
#ifdef __AVR__
  *csport |= cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport->PIO_SODR  |=  cspinmask;
#endif
}



void Adafruit_S6D02A1::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void Adafruit_S6D02A1::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
#ifdef __AVR__
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
#endif
#if defined(__SAM3X8E__)
  rsport->PIO_SODR |=  rspinmask;
  csport->PIO_CODR  |=  cspinmask;
#endif
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }

#ifdef __AVR__
  *csport |= cspinmask;
#endif
#if defined(__SAM3X8E__)
  csport->PIO_SODR  |=  cspinmask;
#endif
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_S6D02A1::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_S6D02A1::setRotation(uint8_t m) {

  writecommand(S6D02A1_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     } else {
       writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
     }
     _width  = S6D02A1_TFTWIDTH;
     _height = S6D02A1_TFTHEIGHT;
     break;
   case 1:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
     } else {
       writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     }
     _width  = S6D02A1_TFTHEIGHT;
     _height = S6D02A1_TFTWIDTH;
     break;
  case 2:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_RGB);
     } else {
       writedata(MADCTL_BGR);
     }
     _width  = S6D02A1_TFTWIDTH;
     _height = S6D02A1_TFTHEIGHT;
    break;
   case 3:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     } else {
       writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
     }
     _width  = S6D02A1_TFTHEIGHT;
     _height = S6D02A1_TFTWIDTH;
     break;
  }
}


void Adafruit_S6D02A1::invertDisplay(boolean i) {
  writecommand(i ? S6D02A1_INVON : S6D02A1_INVOFF);
}


////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Adafruit_S6D02A1::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void Adafruit_S6D02A1::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Adafruit_S6D02A1::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Adafruit_S6D02A1::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Adafruit_S6D02A1::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
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
 
 uint32_t Adafruit_S6D02A1::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
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
