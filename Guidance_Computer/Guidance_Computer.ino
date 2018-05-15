/*
 ** SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - see variable SD_PIN (for MKRZero SD: SDCARD_SS_PIN)

 **Thermocouple MAX31855 Chip attached as follows:
 **
*/

#include <Stage.h>
#include <ThrottleLevel.h>
#include <PhantomConstants.h>
#include <PhantomModules.h>
#include <PhantomGuidance.h>
#include <PhantomUtils.h>

PhantomConstants constants;
PhantomModules modules = PhantomModules();

void setup() {
  
}

void loop() {  
}
