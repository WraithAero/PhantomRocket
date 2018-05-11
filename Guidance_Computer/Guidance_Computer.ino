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

boolean baro_found;

void setup()
{
  
}

void loop() {
  switch (currentStage) {
    case On_Pad:
      if (!baro.begin()) {
        if (baro_found) {
          logPrint("Couldnt find barometer!", true);
          baro_found = false;
        }
        return;
      } else {
        baro_found = true;
      }
      stageRocket(getPadCommand());
      break;

    case Coast:
      if (baro.getAltitude() - initAlt <= CHUTE_DEPLOY_ALT) {
        stageRocket(Chute);
      }
      break;

    case Chute:
      deployChutes();
      break;
    case Abort:
      seperateStages();
      deployChutes();
      break;
  }
  
}

void signalPad() {
  pinMode(LAUNCHPAD_COM_PIN, OUTPUT);
  digitalWrite(LAUNCHPAD_COM_PIN, HIGH);
  int initMillis = millis();
  int currentMillis;
  do {
    currentMillis = millis();
  } while (currentMillis - initMillis < 90);
  digitalWrite(LAUNCHPAD_COM_PIN, LOW);
  pinMode(LAUNCHPAD_COM_PIN, INPUT);
}

boolean arrayEquals(int array_one[], int array_two[]) {
  boolean equal = true;

  if (sizeof(array_one) != sizeof(array_two)) {
    equal = false;
  }

  for (int i = 0; i < sizeof(array_one); i++) {
    if (array_one[i] != array_two[i]) {
      equal = false;
    }
  }
  return equal;
}

Stage getPadCommand() {
  Stage command;
  double initMillis = millis();
  double currentMillis;
  int counter = 0;
  int commandSequence[] = {0, 0, 0};
  if (digitalRead(LAUNCHPAD_COM_PIN) > 0) {
    do {
      currentMillis = millis();
      if (currentMillis - initMillis < counter + 1 * 110) {
        if (digitalRead(LAUNCHPAD_COM_PIN) > 0) {
          commandSequence[counter] = 1;
        } else {
          commandSequence[counter] = 0;
        }
        counter++;
      }
      signalPad();
    } while (commandSequence[2] == NULL_SEQUENCE[2] && currentMillis - initMillis < 500);

    if (currentMillis - initMillis > 500) {
      command = currentStage;
    }

    if (arrayEquals(commandSequence, LAUNCH_SEQUENCE)) {
      return Launch;
    } else if (arrayEquals(commandSequence, ABORT_SEQUENCE)) {
      return Abort;
    } else if (arrayEquals(commandSequence, FUEL_LOADING_SEQUENCE)) {
      return Loading_Fuel;
    } else {
      signalPad();
      delay(90);
      signalPad();
      delay(90);
      signalPad();
      command = getPadCommand();
    }
  } else {
    command = currentStage;
  }
  return command;
}

void logPrint(String message, boolean ln) {
  main_log = SD.open(logName, FILE_WRITE);
  if (ln) {
    Serial.println(message);
    main_log.println(message);
  } else {
    Serial.print(message);
    main_log.print(message);
  }
  main_log.close();
}

void printStage(Stage toPrint, boolean ln){
  main_log = SD.open(logName, FILE_WRITE);
  if (ln) {
    Serial.println(toPrint);
    main_log.println(toPrint);
  } else {
    Serial.print(toPrint);
    main_log.print(toPrint);
  }
  main_log.close();
}

