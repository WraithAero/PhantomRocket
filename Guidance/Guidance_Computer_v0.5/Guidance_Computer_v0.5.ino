/*
 ** SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - see variable SD_PIN (for MKRZero SD: SDCARD_SS_PIN)

 **Thermocouple MAX31855 Chip attached as follows:
 **
*/

#include <SD.h>
#include <SPI.h>
#include <Adafruit_MPL3115A2.h>
#include <Servo.h>
#include <PID_v1.h>
#include <CurieIMU.h>
#include <Wire.h>
#include <Stage.h>
#include <ThrottleLevel.h>
#include <Adafruit_MAX31855.h>

double pitchSP, rollSP, yawSP;
double pitchInput, rollInput, yawInput;

const int NORTH_PIN = 1;
const int EAST_PIN = 2;
const int SOUTH_PIN = 3;
const int WEST_PIN = 4;

const int FAR_FIN_VALUE = 180;
const int NEAR_FIN_VALUE = 0;
const int STANDARD_FIN_VALUE = 90;
const int FIN_DEGREES_PER_SECOND = 60;
const int CHUTE_DEPLOY_ALT = 1500;

const String logName = "main_control.log";

const int LAUNCHPAD_COM_PIN = 5;
const int PARACHUTE_ONE_PIN = 6;
const int PARACHUTE_TWO_PIN = 7;
const int SD_PIN = 8;
const int IGNITER_PIN = 9;
const int LAB_COM_PIN = 10;
const int LOX_VALVE_PIN = 11;
const int FUEL_VALVE_PIN = 12;
const int LOX_INPUT_VALVE_PIN = 13;
const int FUEL_INPUT_VALVE_PIN = 14;
const int NOZZLE_THERMOCOIL_PIN = 15;
const int CC_THERMOCOIL_PIN = 16;

int LAUNCH_SEQUENCE[] = {1, 0, 1};
int ABORT_SEQUENCE[] = {0, 1, 0};
int FUEL_LOADING_SEQUENCE[] = {1, 1, 1};
int NULL_SEQUENCE[] = {0, 0, 0};

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_MAX31855 nozzle_thermocoil = Adafruit_MAX31855(NOZZLE_THERMOCOIL_PIN);
Adafruit_MAX31855 cc_thermocoil = Adafruit_MAX31855(CC_THERMOCOIL_PIN);
double initAlt;

File main_log;

//Specify the links and initial tuning parameters
double pitchP = 2;
double pitchI = 5;
double pitchD = 3;
double rollP = 2;
double rollI = 5;
double rollD = 3;
double yawP = 2;
double yawI = 5;
double yawD = 3;
double pitchOutput;
double rollOutput;
double yawOutput;

boolean baro_found;

float gx, gy, gz;

Stage currentStage = On_Pad;

PID pitch(&pitchInput, &pitchOutput, &pitchSP, pitchP, pitchI, pitchD, DIRECT);
PID roll(&rollInput, &rollOutput, &rollSP, rollP, rollI, rollD, DIRECT);
PID yaw(&yawInput, &yawOutput, &yawSP, yawP, yawI, yawD, DIRECT);

Servo north;
Servo east;
Servo south;
Servo west;

int northAngle;
int eastAngle;
int southAngle;
int westAngle;

void setup()
{
  initializeModules();
  setInputs();

  initAlt = baro.getAltitude();

  //turn the PID on
  pitch.SetMode(AUTOMATIC);
  roll.SetMode(AUTOMATIC);
  yaw.SetMode(AUTOMATIC);
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
  }
  setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
  setInputs();

  compute();
  steer(pitchOutput, rollOutput, yawOutput);
}

void steer(double pitchAngle, double rollAngle, double yawAngle) {
  if (currentStage != Chute) {
    writeEast(pitchAngle + rollAngle / 2);
    writeWest(pitchAngle + rollAngle / 2);
    writeNorth(yawAngle + rollAngle / 2);
    writeSouth(yawAngle + rollAngle / 2);
  } else {
    writeEast(-(pitchAngle + rollAngle / 2));
    writeWest(-(pitchAngle + rollAngle / 2));
    writeNorth(-(yawAngle + rollAngle / 2));
    writeSouth(-(yawAngle + rollAngle / 2));
  }
}

void setAllFins(double angle) {
  writeNorth(angle);
  writeEast(angle);
  writeSouth(angle);
  writeWest(angle);
  delay((angle * FIN_DEGREES_PER_SECOND) * 1000);
}

void setSPs(double newPitchSP, double newRollSP, double newYawSP) {
  pitchSP = newPitchSP;
  rollSP = newRollSP;
  yawSP = newYawSP;
}

void compute() {
  pitch.Compute();
  roll.Compute();
  yaw.Compute();
}

void setInputs() {
  pitchInput = getPitch();
  rollInput = getRoll();
  yawInput = getYaw();
}

void setGyro() {
  CurieIMU.readGyroScaled(gx, gy, gz);
}

float getPitch() {
  setGyro();
  return gx;
}

float getRoll() {
  setGyro();
  return gy;
}

float getYaw() {
  setGyro();
  return gz;
}

double getAltitude() {
  return baro.getAltitude() - initAlt;
}

double getOptimalPitch() {
  double p;
  if (currentStage != Coast) {
    p = 90;
  } else {
    p = getPitch() + .05;
  }
  return p;
}
double getOptimalRoll() {
  return 0;
}
double getOptimalYaw() {
  return 90;
}
void stageRocket(Stage newStage) {
  currentStage = newStage;
  logPrint("Stage: ", false);
  logPrint(newStage);
}

void throttle(ThrottleLevel level) {
  switch (level) {
    case Off:
      digitalWrite(LOX_VALVE_PIN, LOW);
      digitalWrite(FUEL_VALVE_PIN, LOW);
      break;
    case Full:
      digitalWrite(LOX_VALVE_PIN, HIGH);
      digitalWrite(FUEL_VALVE_PIN, HIGH);
      break;
  }
}

void deployChutes() {
  digitalWrite(PARACHUTE_ONE_PIN, HIGH);
  digitalWrite(PARACHUTE_TWO_PIN, HIGH);
}

void openInputValves() {
  digitalWrite(LOX_INPUT_VALVE_PIN, HIGH);
  digitalWrite(FUEL_INPUT_VALVE_PIN, HIGH);
}

int getNorth() {
  return northAngle;
}
int getEast() {
  return eastAngle;
}
int getSouth() {
  return southAngle;
}
int getWest() {
  return westAngle;
}
int writeNorth(int angle) {
  north.write(angle);
  northAngle = angle;
}
int writeEast(int angle) {
  east.write(angle);
  eastAngle = angle;
}
int writeSouth(int angle) {
  south.write(angle);
  southAngle = angle;
}
int writeWest(int angle) {
  south.write(angle);
  westAngle = angle;
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

  logPrint("Contacting pad...", true);
  signalPad();
  logPrint("Waiting for pad response...", true);
  while (digitalRead(LAUNCHPAD_COM_PIN) < 0);
  logPrint("Received pad response", true);

  logPrint("Initializing gyro...", true);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  logPrint("Gyro initialization complete", true);
  logPrint("", true);

  logPrint("Initializing accelerometer...", true);
  CurieIMU.setAccelerometerRange(2);
  logPrint("Accelerometer initialization complete", true);

  logPrint("Initializing fin servos...", true);
  north.attach(NORTH_PIN);
  east.attach(EAST_PIN);
  south.attach(SOUTH_PIN);
  west.attach(WEST_PIN);
  setAllFins(STANDARD_FIN_VALUE);
  logPrint("Servo initialization complete. Testing fins...", true);
  setAllFins(NEAR_FIN_VALUE);
  setAllFins(FAR_FIN_VALUE);
  setAllFins(STANDARD_FIN_VALUE);
  logPrint("Fin Test Complete", true);
  logPrint("", true);

  logPrint("Initializing Barometer...", true);
  baro.begin();
  initAlt = baro.getAltitude();
  logPrint("Barometer initialization complete", true);
  logPrint("", true);

  logPrint("Initializing chutes...", true);
  logPrint("Contacting chute 1...", true);
  pinMode(PARACHUTE_ONE_PIN, OUTPUT);
  digitalWrite(PARACHUTE_ONE_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(PARACHUTE_ONE_PIN, LOW);
  pinMode(PARACHUTE_ONE_PIN, INPUT);
  while (digitalRead(PARACHUTE_ONE_PIN) == 0);
  logPrint("Chute 1 initialized", true);

  logPrint("Contacting chute 2...", true);
  pinMode(PARACHUTE_TWO_PIN, OUTPUT);
  digitalWrite(PARACHUTE_TWO_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(PARACHUTE_TWO_PIN, LOW);
  pinMode(PARACHUTE_TWO_PIN, INPUT);
  while (digitalRead(PARACHUTE_TWO_PIN) == 0);
  logPrint("Chute 2 initialized", true);
  logPrint("Chutes initialized", true);
  logPrint("", true);

  logPrint("Initializing igniter and valves", true);
  logPrint("Stand clear of rocket in case of mislight", true);
  pinMode(IGNITER_PIN, OUTPUT);
  digitalWrite(IGNITER_PIN, LOW);
  pinMode(LOX_VALVE_PIN, OUTPUT);
  pinMode(FUEL_VALVE_PIN, OUTPUT);
  pinMode(LOX_INPUT_VALVE_PIN, OUTPUT);
  pinMode(FUEL_INPUT_VALVE_PIN, OUTPUT);
  logPrint("Igniter and valves initialized", true);

  logPrint("Contacting lab", true);
  pinMode(LAB_COM_PIN, OUTPUT);
  digitalWrite(LAB_COM_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(LAB_COM_PIN, LOW);
  pinMode(LAB_COM_PIN, INPUT);
  while (digitalRead(LAB_COM_PIN) == 0);
  logPrint("Lab contact established", true);

  logPrint("All modules initialized", true);
  logPrint("Continue with launch procedure", true);
  logPrint("", true);
}

