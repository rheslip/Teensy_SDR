# Teensy_SDR  
Software defined radio using Teensy 3.1, Teensy Audio shield and Softrock  

This is a software defined radio using the Arduino compatible Teensy 3.1 and audio shield from PJRC. 
It takes advantage of the audio library supplied by PJRC to implement the audio signal processing chain.

Dependencies - most of these libraries are available from PJRC and can be installed with the Teensyduino package:  

Audio - Teensy Audio library V1.02,  
SD (not used but needed for the audio libraries to compile),  
SPI,  
Wire,  
Metro,  
Bounce,  
Encoder,  
SI5351 (github.com/NT7S/Si5351),  
Adafruit Graphics library (adafruit.com),  
Driver for your display - I used a Banggood knockoff of the AdaFruit 1.8" TFT  

Changelog:
Jan 2015 - first public release, RX only  
March 2015 - CW and SSB TX mode added, band swiching added, general cleanup of the code  
April 9 2015 - added the driver for the Banggood 1.8" display and a slightly modded SI5351 library for the Teensy which has no eeprom  
	- move these to your Arduino/libraries folder  
April 13 2015 - added wiring diagram  
Sept 19 2015 - merged Softrock Ensemble support and SI570 from GMTII   
	- I don't have an Ensemble so I can't test these patches  

NOTE: code compiles OK with Arduino 1.61 and Teensyduino 1.21 BUT sometimes image is not suppressed as evidenced
by signals moving both ways when you tune. Seems that if you recompile and reload once or twice its fine until
the next time you recompile and flash the code. Tool chain bug ???  

TODO:
- fix software AGC which no longer works after port to audio lib 1.02  
- S Meter needs improvement  
- UI improvements - configuration settings etc  
- clean up display code - remove direct x,y references to allow easier modifications  

project blog: rheslip.blogspot.com

