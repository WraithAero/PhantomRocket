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
#include <PhantomLib.h>

PhantomConstants constants;
PhantomLib lib = PhantomLib(constants);

void setup() {
  lib.initialize();
}

void loop() {  
  lib.guidanceLoop(lib.getPadCommand());
}
