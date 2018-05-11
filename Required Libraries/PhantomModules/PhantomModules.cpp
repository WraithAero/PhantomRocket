#include "Arduino.h"
#include "PhantomModules.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "CurieIMU.h"
#include "Servo.h"
#include "Adafruit_MAX31855.h"
#include "Adafruit_MPL3115A2.h"

PhantomModules::PhantomModules(PhantomConstants constants, PhantomUtils utils){
  _constants = constants;
  _utils = utils;
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initializing flight computer...");
  Serial.println();

  Serial.print("Initializing SD card...");

  if (!SD.begin(_constants.SD_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  Serial.println();
  Serial.println("All further reports will be output both to");
  Serial.println("Serial and written to the SD file titled" + _constants.logName);
  Serial.println();

  logPrint("Contacting pad...", true);
  signalPad();
  logPrint("Waiting for pad response...", true);
  while (digitalRead(LAUNCHPAD_COM_PIN) == 0);
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
  north.attach(_constants.NORTH_PIN);
  east.attach(_constants.EAST_PIN);
  south.attach(_constants.SOUTH_PIN);
  west.attach(_constants._constants.WEST_PIN);
  setAllFins(_constants.STANDARD_FIN_VALUE);
  logPrint("Servo initialization complete. Testing fins...", true);
  setAllFins(_constants.NEAR_FIN_VALUE);
  setAllFins(_constants.FAR_FIN_VALUE);
  setAllFins(_constants.STANDARD_FIN_VALUE);
  logPrint("Fin Test Complete", true);
  logPrint("", true);

  logPrint("Initializing Barometer...", true);
  baro.begin();
  initAlt = baro.getAltitude();
  logPrint("Barometer initialization complete", true);
  logPrint("", true);

  logPrint("Initializing chutes...", true);
  logPrint("Contacting chute 1...", true);
  pinMode(_constants.PARACHUTE_PIN, OUTPUT);
  digitalWrite(_constants.PARACHUTE_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.PARACHUTE_PIN, LOW);
  pinMode(_constants.PARACHUTE_PIN, INPUT);
  while (digitalRead(_constants.PARACHUTE_PIN) == 0);
  logPrint("Chute 1 initialized", true);

  logPrint("Initializing igniter and valves", true);
  logPrint("Stand clear of rocket in case of mislight", true);
  pinMode(_constants.IGNITER_PIN, OUTPUT);
  digitalWrite(_constants.IGNITER_PIN, LOW);
  pinMode(_constants.LOX_VALVE_PIN, OUTPUT);
  pinMode(_constants.FUEL_VALVE_PIN, OUTPUT);
  pinMode(_constants.LOX_INPUT_VALVE_PIN, OUTPUT);
  pinMode(_constants.FUEL_INPUT_VALVE_PIN, OUTPUT);
  logPrint("Igniter and valves initialized", true);

  logPrint("Contacting lab", true);
  pinMode(_constants.LAB_COM_PIN, OUTPUT);
  digitalWrite(_constants.LAB_COM_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.LAB_COM_PIN, LOW);
  pinMode(_constants.LAB_COM_PIN, INPUT);
  while (digitalRead(_constants.LAB_COM_PIN) == 0);
  logPrint("Lab contact established", true);

  logPrint("All modules initialized", true);
  logPrint("Continue with launch procedure", true);
  logPrint("", true);
}

double PhantomModules::getNorth(){
	return northAngle;
}

double PhantomModules::getEast(){
	return eastAngle;
}

double PhantomModules::getSouth(){
	return southAngle;
}

double PhantomModules::getWest(){
	return westAngle;
}

void PhantomModules::deployChutes(){
	digitalWrite(_constants._constants._constants.PARACHUTE_PIN, HIGH);
}

void PhantomModules::openInputValves(){
	digitalWrite(_constants.LOX_INPUT_VALVE_PIN, HIGH);
	digitalWrite(_constants.FUEL_INPUT_VALVE_PIN, HIGH);
}

double PhantomModules::getOptimalPitch() {
  double p;
  if (currentStage != Coast) {
    p = 90;
  } else {
    p = getPitch() + .05;
  }
  return p;
}
double PhantomModules::getOptimalRoll() {
  return 0;
}
double PhantomModules::getOptimalYaw() {
  return 90;
}

double getCCTemp(){
  return cc_thermocoil.get();
}

double getAltitude() {
  return baro.getAltitude() - initAlt;
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
