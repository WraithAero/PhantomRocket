#include <Wire.h>
#include <SPI.h>
#include <SD.h>

void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

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
          counter++;
        } else {
          commandSequence[counter] = 0;
          counter++;
        }
      }
      signalPad();
    } while (commandSequence[2] != NULL_SEQUENCE[2] && currentMillis - initMillis < 500);

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
