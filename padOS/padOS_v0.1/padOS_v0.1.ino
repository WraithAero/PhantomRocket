#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Stage.h>

const int SD_PIN = 3;
const int ROCKET_COM_PIN = 4;

const String logName = "pad.log";

int LAUNCH_SEQUENCE[] = {1, 0, 1};
int ABORT_SEQUENCE[] = {0, 1, 0};
int FUEL_LOADING_SEQUENCE[] = {1, 1, 1};
int NULL_SEQUENCE[] = {0, 0, 0};

File pad_log;

void setup() {
  Serial.begin(9600);
  initializeModules();
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

void sendCommand(int sequence[]) {
  int initMillis = millis();
  int currentMillis;
  digitalWrite(ROCKET_COM_PIN, HIGH);
  delay(100);
  digitalWrite(ROCKET_COM_PIN, LOW);
  do {
    currentMillis = millis();
  } while (!readRocketCom() && currentMillis - initMillis < 500);
  for (int i = 0; i < sizeof(sequence); i++) {
    if (sequence[i] == 1) {
      digitalWrite(ROCKET_COM_PIN, HIGH);
      delay(100);
    } else {
      delay(100);
    }
  }
}

boolean readRocketCom() {
  pinMode(ROCKET_COM_PIN, INPUT);
  if (digitalRead(ROCKET_COM_PIN) > 0) {
    pinMode(ROCKET_COM_PIN, OUTPUT);
    return true;
  } else {
    pinMode(ROCKET_COM_PIN, OUTPUT);
    return false;
  }
}

Stage getRocketCommand() {
  Stage command;
  double initMillis = millis();
  double currentMillis;
  int counter = 0;
  int commandSequence[] = {0, 0, 0};
  if (digitalRead(ROCKET_COM_PIN) > 0) {
    do {
      currentMillis = millis();
      if (currentMillis - initMillis < counter + 1 * 110) {
        if (digitalRead(ROCKET_COM_PIN) > 0) {
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

void signalPad() {
  pinMode(ROCKET_COM_PIN, OUTPUT);
  digitalWrite(ROCKET_COM_PIN, HIGH);
  int initMillis = millis();
  int currentMillis;
  do {
    currentMillis = millis();
  } while (currentMillis - initMillis < 90);
  digitalWrite(ROCKET_COM_PIN, LOW);
  pinMode(ROCKET_COM_PIN, INPUT);
}

void logPrint(String message, boolean ln) {
  pad_log = SD.open(logName, FILE_WRITE);
  if (ln) {
    Serial.println(message);
    pad_log.println(message);
  } else {
    Serial.print(message);
    pad_log.print(message);
  }
  pad_log.close();
}


void initializeModules() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initializing flight computer...");
  Serial.println();

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  Serial.println();
  Serial.println("All further reports will be output both to");
  Serial.println("Serial and written to the SD file titled" + logName);
  Serial.println();

  logPrint("Initializing communications with Phantom I...", true);
}

