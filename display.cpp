// code for the TFT display

#include "display.h"
#include <Audio.h>
#include <Adafruit_GFX.h>        // LCD Core graphics library
//#include <Adafruit_QDTech.h>     // 1.8" TFT Module using Samsung S6D02A1 chip
#include <Adafruit_S6D02A1.h> // Hardware-specific library
//extern Adafruit_QDTech tft;
extern Adafruit_S6D02A1 tft;

extern AudioAnalyzeFFT256  myFFT;      // FFT for Spectrum Display

void setup_display(void) {
  
  // initialize the LCD display
//  tft.init();
  tft.initR(INITR_BLACKTAB);   // initialize a S6D02A1S chip, black tab
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  tft.setCursor(0, 115);
  tft.setTextColor(WHITE);
  tft.setTextWrap(true);
  tft.print("Teensy SDR 1.1");
  
  // Show mid screen tune position
  tft.drawFastVLine(80, 0,60,RED);
}

// draw the spectrum display
// this version draws 1/10 of the spectrum per call but we run it 10x the speed
// this allows other stuff to run without blocking for so long

void show_spectrum(void) {
  static int startx=0, endx;
  endx=startx+16;
  int scale=1;
//digitalWrite(DEBUG_PIN,1); // for timing measurements
  
  for (int16_t x=startx; x < endx; x+=2) 
  {
    int bar=abs(myFFT.output[x*8/10])/scale;
    if (bar >60) bar=60;
    if(x!=80)
    {
       tft.drawFastVLine(x, 60-bar,bar, GREEN);
       tft.drawFastVLine(x, 0, 60-bar, BLACK);    
    }
  }
  startx+=16;
  if(startx >=160) startx=0;
//digitalWrite(DEBUG_PIN,0); // 
}

/* old draw routine
// draw the spectrum display

void show_spectrum(void) {

  int scale=1;
  for (int16_t x=0; x < 160; x+=2) 
  {
    int bar=abs(myFFT.output[x*8/10])/scale;
    if (bar >60) bar=60;
    if(x!=80)
    {
       tft.drawFastVLine(x, 60-bar,bar, GREEN);
       tft.drawFastVLine(x, 0, 60-bar, BLACK);    
    }
  }
}
*/

void show_waterfall(void) {
  // experimental waterfall display for CW -
  // this should probably be on a faster timer since it needs to run as fast as possible to catch CW edges
  //  FFT bins are 22khz/128=171hz wide 
  // cw peak should be around 11.6khz - 
  static uint16_t waterfall[80];  // array for simple waterfall display
  static uint8_t w_index=0,w_avg;
  waterfall[w_index]=0;
  for (uint8_t y=66;y<67;++y)  // sum of bin powers near cursor - usb only
      waterfall[w_index]+=(uint8_t)(abs(myFFT.output[y])); // store bin power readings in circular buffer
  waterfall[w_index]|= (waterfall[w_index]<<5 |waterfall[w_index]<<11); // make it colorful
  int8_t p=w_index;
  for (uint8_t x=158;x>0;x-=2) {
    tft.fillRect(x,65,2,4,waterfall[p]);
    if (--p<0 ) p=79;
  }
  if (++w_index >=80) w_index=0; 
}

// indicate filter bandwidth on spectrum display
void show_bandwidth(int filtermode) { 
  tft.drawFastHLine(0,61,160, BLACK); // erase old indicator
  tft.drawFastHLine(0,62,160, BLACK); // erase old indicator 
  
  switch (filtermode)	{
    case LSB_NARROW:
      tft.drawFastHLine(72,61,6, RED);
      tft.drawFastHLine(72,62,6, RED);
    break;
    case LSB_WIDE:
      tft.drawFastHLine(61,61,20, RED);
      tft.drawFastHLine(61,62,20, RED);
    break;
    case USB_NARROW:
      tft.drawFastHLine(83,61,6, RED);
      tft.drawFastHLine(83,62,6, RED);
    break;
    case USB_WIDE:
      tft.drawFastHLine(80,61,20, RED);
      tft.drawFastHLine(80,62,20, RED);
    break;
  }
}  


// show radio mode
void show_radiomode(String mode) { 
  tft.fillRect(125, 85, 30, 7, BLACK); // erase old string
  tft.setTextColor(WHITE);
  tft.setCursor(125, 85);
  tft.print(mode);
}  

void show_band(String bandname) {  // show band
  tft.fillRect(100, 85, 19, 7, BLACK); // erase old string
  tft.setTextColor(WHITE);
  tft.setCursor(100, 85);
  tft.print(bandname);
}

// show frequency
void show_frequency(long int freq) { 
    char string[80];   // print format stuff
    sprintf(string,"%d.%03d.%03d",freq/1000000,(freq-freq/1000000*1000000)/1000,
          freq%1000 );
    tft.fillRect(100,115,100,120,BLACK);
    tft.setCursor(100, 115);
    tft.setTextColor(WHITE);
    tft.print(string); 
}    
