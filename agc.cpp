// soft AGC and S meter display by Loftur E. Jónasson TF3LJ/VE2LJX 6/14
//

#include <Metro.h>
#include <Audio.h>

#include <Adafruit_GFX.h>        // LCD Core graphics library
//#include <Adafruit_QDTech.h>     // 1.8" TFT Module using Samsung S6D02A1 chip
#include <Adafruit_S6D02A1.h> // Hardware-specific library

//extern Adafruit_QDTech tft;
extern Adafruit_S6D02A1 tft;

extern AudioMixer4     AGC;      // Summer (add inputs)
extern AudioPeak       AGCpeak;  // Measure Audio Peak for AGC use
extern AudioPeak       Smeter;   // Measure Audio Peak for S meter

Metro l_ms =   Metro(1);         // Set up a 1ms Metro
Metro lcd_upd2=Metro(100);       // Set up a Metro for LCD updates

int32_t       sample[10];        // A ringbuffer of samples (has to be larger than AGCattack)

double        AGCgain=1;         // Initial AGC gain. 1 = unity gain, 32768.0 max, 0.00004 min
#define       AGCMAX  10
const int32_t AGCnomVal = 10000; // Nominal Output (32768 max)
const int32_t AGCattack = 2;     // AGC Hang time (milliseconds) before reducing gain
const int32_t AGChang   = 30;   // AGC Hang time before increasing gain
const double  AGCslope  = 1.05;   // Relative gain change


//
// Automatic Gain Control function
//
void agc(void)
{
  static uint8_t i;
  static uint16_t hangtimer;
  uint8_t j,k;
  int32_t s_sample;  // Raw signal strength (max per 1ms)
  int32_t samp;      // AGC feedback loop sample strength (max per 1ms)
  int32_t temp;      // yeah, just a temp value
  double uv, dbuv, s;// microvolts, db-microvolts, s-units
  char string[80];   // print format stuff
  
  if (l_ms.check() == 1)
  {
    // Collect S-meter data
    s_sample = Smeter.Dpp(); // Highest sample within 1 millisecond
    Smeter.begin();          // Reset for next measurement

    // AGC: Collect current 1ms peak at output and feed to a ringbuffer
    sample[i++] = samp = AGCpeak.Dpp();
    AGCpeak.begin();
    if (i >= AGCattack) i=0;
    
    // Check if we need to reduce gain
    for(j=0,k=0;j<AGCattack;j++)
    {
      if (sample[j] > AGCnomVal) k++;
    }
    
    // We need to reduce gain
    if ((k == AGCattack) || ((k>0) && (hangtimer>=AGChang)))  // Activate AGCattack
    {
      // find largest value
      temp = 0;
      for(j=0;j<AGCattack;j++)
      {
        if (sample[j]> temp) temp = sample[j];
      }
      
      // Instant reduction to appropriate value
      AGCgain = AGCgain * AGCnomVal/temp;
      // Reset hang timer
      hangtimer = 0;
    }
    
    // Increment hangtimer while level is lower than nominal
    else if(samp < AGCnomVal) hangtimer++; 
      
    if (hangtimer >= AGChang)  // We need to ramp up the gain
    {
      AGCgain = AGCgain * AGCslope;
    }
    
    if (AGCgain > AGCMAX) AGCgain=AGCMAX; // limit the gain  
    
    AGC.gain(0,AGCgain);       // Adjust AGC gain
    
    //
    // Print stuff to LCD only once every 100ms
    //
    if (lcd_upd2.check() == 1) 
    {
      // Calculate S units. 50uV = S9
      uv = s_sample/30.0;        // microvolts, roughly calibrated
      dbuv = 20.0*log10(uv);
      s = 1.0 + (14.0 + dbuv)/6.0;
      if (s>9.0)
      {
        dbuv = dbuv - 34.0;
        s = 9.0;
      }
      else dbuv = 0;
      // Print S units
      tft.fillRect(10, 85, 30, 7,S6D02A1_BLACK);
      tft.setCursor(0, 85);
      if (dbuv == 0) sprintf(string,"S:%3.1f",s);
      else sprintf(string,"S:9+%02.0f",dbuv);
      tft.print(string);
 
      if(0)  // Debug stuff
      {
        // Print AGC loop parameters
        tft.fillRect(10, 105, 100, 7,S6D02A1_BLACK);
        tft.setCursor(0, 105);
        sprintf(string,"pk:%5lu g:%5.1f",samp, AGCgain);
        tft.print(string);
      }
    }
  }  
} 

