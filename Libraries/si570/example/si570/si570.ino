/*
 * Wire is only used from the Si570 module but we need to list it here so that
 * the Arduino environment knows we need it.
 */

#include <Wire.h>
#include "Si570.h"

#define SI570_I2C_ADDRESS 0x55
unsigned long frequency = 14200000;


Si570 *vfo;

void setup() {

  // Initialize the Serial port so that we can use it for debugging
  Serial.begin(115200);

  // The library automatically reads the factory calibration settings of your Si570
  // but it needs to know for what frequency it was calibrated for.
  // Looks like most HAM Si570 are calibrated for 56.320 Mhz.
  // If yours was calibrated for another frequency, you need to change that here
  vfo = new Si570(SI570_I2C_ADDRESS, 56320000);

  if (vfo->status == SI570_ERROR) {
    // The Si570 is unreachable. Show an error for 3 seconds and continue.
    printLine2("Si570 comm error");
    delay(3000);
  }

  // This will print some debugging info to the serial console.
  vfo->debugSi570();

  //set the initial frequency
  vfo->setFrequency(26150000L);


}

void loop(){

}


