#include "Arduino.h"
#include "PhantomConstants.h"
#include "PhantomModules.h"
#include "Stage.h"

PhantomUtils::PhantomUtils(PhantomConstants constants, PhantomModules modules){
	_constants = constants;
	_modules = modules;
}

boolean PhantomUtils::arrayEquals(int array_one[], int array_two[]) {
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

Stage PhantomUtils::getPadCommand() {
  Stage command;
  double initMillis = millis();
  double currentMillis;
  int counter = 0;
  int commandSequence[] = {0, 0, 0};
  if (digitalRead(_constants.LAUNCHPAD_COM_PIN) > 0) {
    do {
      currentMillis = millis();
      if (currentMillis - initMillis < counter + 1 * 110) {
        if (digitalRead(_constants.LAUNCHPAD_COM_PIN) > 0) {
          commandSequence[counter] = 1;
        } else {
          commandSequence[counter] = 0;
        }
        counter++;
      }
      signalPad();
    } while (commandSequence[2] == _constants.NULL_SEQUENCE[2] && currentMillis - initMillis < 500);

    if (currentMillis - initMillis > 500) {
      command = currentStage;
    }

    if (arrayEquals(commandSequence, _constants.LAUNCH_SEQUENCE)) {
      return Launch;
    } else if (arrayEquals(commandSequence, _constants.ABORT_SEQUENCE)) {
      return Abort;
    } else if (arrayEquals(commandSequence, _constants.FUEL_LOADING_SEQUENCE)) {
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

void PhantomUtils::logPrint(String message, boolean ln) {
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

void PhantomUtils::stagePrint(Stage toPrint, boolean ln){
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

void PhantomUtils::dataPrint(int toPrint, boolean ln){
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
