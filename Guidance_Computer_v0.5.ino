/*
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - see variable SD_PIN (for MKRZero SD: SDCARD_SS_PIN)
*/


#include <SD.h>
#include <SPI.h>
#include <Adafruit_MPL3115A2.h>
#include <Servo.h>
#include <PID_v1.h>
#include <CurieIMU.h>
#include <Wire.h>

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

const int[] LAUNCH_SEQUENCE = {1, 0, 1};
const int[] ABORT_SEQUENCE = {0, 1, 0};
const int[] FUEL_LOADING_SEQUENCE = {1, 1, 1};

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
double altitude;
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

enum MoveType {
  Pitch, Roll, Yaw
};

enum Stage {
 On_Pad, Launch, Burn, Coast, Chute
};

enum CommandType {
  Com_Launch, Abort, Fuel_Load, None
};

enum ThrottleLevels {
  Off, Full
}

CommandType padCommand = CommandType.None;
Stage stage = Stage.On_Pad;

PID pitch(&pitchInput, &pitchOutput, &pitchSP, pitchP, pitchI, pitchD, DIRECT);
PID roll(&rollInput, &rollOutput, &rollSP, rollP, rollI, rollD, DIRECT);
PID yaw(&yawInput, &yawOutput, &yawSP, yawP, yawI, yawD, DIRECT);

Servo north;
Servo east;
Servo south;
Servo west;

void setup()
{
  initializeModules();
  setInputs();

  //turn the PID on
  pitch.SetMode(AUTOMATIC);
  roll.SetMode(AUTOMATIC);
  yaw.SetMode(AUTOMATIC);
}

void loop() {
  switch (stage) {
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
      padCommand = getPadCommand();
      switch (padCommand) {
        case Com_Launch:
          stageRocket(Launch);
          throttle(ThrottleLevel.High);
          digitalWrite(IGNITER_PIN, HIGH);
          break;

        case Abort:
          stageRocket(Abort);
          throttle(ThrottleLevel.Off);
          openInputValves();
          break;

        case Fuel_Load:
          throttle(ThrottleLevel.Off);
          openInputValves();
          signalPad();
          break;
      }
      break;

    case Launch:
      setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
      setInputs();

      compute();

      steer(pitchOutput, rollOutput, yawOutput);
      break;
    case Burn:
      setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
      setInputs();

      compute();

      steer(pitchOutput, Pitch);
      steer(rollOutput, Roll);
      steer(yawOutput, Yaw);
      break;

    case Coast:
      setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
      setInputs();

      compute();

      steer(pitchOutput, Pitch);
      steer(rollOutput, Roll);
      steer(yawOutput, Yaw);

      if(baro.getAltitude() - initAltitude <= CHUTE_DEPLOY_ALTITUDE){
        stageRocket(Chute);
      }
      break;

    case Chute:
      deployChutes();
      
    break;
  }
}

void steer(double pitchAngle, double rollAngle, double YawAngle) {
  east.write(pitchAngle + rollAngle / 2);
  west.write(pitchAngle + rollAngle / 2);
  north.write(yawAngle + rollAngle / 2);
  south.write(yawAngle + rollAngle / 2);
}

void setAllFins(double angle) {
  north.write(angle);
  east.write(angle);
  south.write(angle);
  west.write(angle);
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
  switch (stage) {
    case On_Pad:

      break;
  }
}
double getOptimalRoll() {
  double p;
  p =
  if (stage != Coast && stage != Chute) {
    return 90;
  } else if (stage = Coast) {

  }
}
double getOptimalYaw() {
  switch (stage) {
    case On_Pad:

      break;
  }
}
void stageRocket(Stage newStage) {
  stage = newStage();
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

void openInputValves() {
  digitalWrite(LOX_PURGE_VALVE, HIGH);
  digitalWrite(FUEL_PURGE_VALVE, HIGH);
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

CommandType getPadCommand() {
  CommandType command;
  double initMillis = millis();
  double currentMillis;
  int counter = 0;
  int[] commandSequence = new int[3];
  if (digitalRead(LAUNCHPAD_COM_PIN) > 0) {
    do {
      currentMillis = millis();
      if (digitalRead(LAUNCHPAD_COM_PIN) > 0 && currentMillis - initMillis < counter + 1 * 110) {
        commandSequence[counter] = 1;
        counter++;
      } else {
        commandSequence[counter] = 0;
        counter++;
      }
      signalPad();
    } while (commandSequence[2] != null && currentMillis - initMillis < 500);

    if (currentMillis - initMillis > 500) {
      return CommandType.None;
    }

    if (commandSequence = LAUNCH_SEQUENCE) {
      return CommandType.Launch;
    } else if (commandSequence = ABORT_SEQUENCE) {
      return CommandType.Abort;
    } else if (commandSequence = FUEL_LOADING_SEQUENCE) {
      return CommandType.Fuel_Load;
    } else {
      signalPad();
      delay(90);
      signalPad();
      delay(90);
      signalPad();
      command = getCommandSequence();
    }
  } else {
    command = CommandType.None;
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
  main_log = SD.close();
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

  logPrint("Initializing accelerometer...");
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
  while (digitalRead(PARACHUTE_ONE_PIN) = 0);
  logPrint("Chute 1 initialized", true);

  logPrint("Contacting chute 2...", true);
  pinMode(PARACHUTE_TWO_PIN, OUTPUT);
  digitalWrite(PARACHUTE_TWO_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(PARACHUTE_TWO_PIN, LOW);
  pinMode(PARACHUTE_TWO_PIN, INPUT);
  while (digitalRead(PARACHUTE_TWO_PIN) = 0);
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
  while (digitalRead(LAB_COM_PIN) = 0);
  logPrint("Lab contact established", true);

  logPrint("All modules initialized", true);
  logPrint("Continue with launch procedure", true);
  logPrint("", true);
}

