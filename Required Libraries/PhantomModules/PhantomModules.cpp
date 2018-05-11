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

  _utils.logPrint("Contacting pad...", true);
  signalPad();
  _utils.logPrint("Waiting for pad response...", true);
  while (digitalRead(_constants.LAUNCHPAND_COM_PIN) == 0);
  _utils.logPrint("Received pad response", true);

  _utils.logPrint("Initializing gyro...", true);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  _utils.logPrint("Gyro initialization complete", true);
  _utils.logPrint("", true);

  _utils.logPrint("Initializing accelerometer...", true);
  CurieIMU.setAccelerometerRange(2);
  _utils.logPrint("Accelerometer initialization complete", true);

  _utils.logPrint("Initializing fin servos...", true);
  north.attach(_constants.NORTH_PIN);
  east.attach(_constants.EAST_PIN);
  south.attach(_constants.SOUTH_PIN);
  west.attach(_constants._constants.WEST_PIN);
  setAllFins(_constants.STANDARD_FIN_VALUE);
  _utils.logPrint("Servo initialization complete. Testing fins...", true);
  setAllFins(_constants.NEAR_FIN_VALUE);
  setAllFins(_constants.FAR_FIN_VALUE);
  setAllFins(_constants.STANDARD_FIN_VALUE);
  _utils.logPrint("Fin Test Complete", true);
  _utils.logPrint("", true);

  _utils.logPrint("Initializing Barometer...", true);
  baro.begin();
  initAlt = baro.getAltitude();
  _utils.logPrint("Barometer initialization complete", true);
  _utils.logPrint("", true);

  _utils.logPrint("Initializing chutes...", true);
  _utils.logPrint("Contacting chute 1...", true);
  pinMode(_constants.PARACHUTE_PIN, OUTPUT);
  digitalWrite(_constants.PARACHUTE_PIN, HIGH);
  _utils.logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.PARACHUTE_PIN, LOW);
  pinMode(_constants.PARACHUTE_PIN, INPUT);
  while (digitalRead(_constants.PARACHUTE_PIN) == 0);
  _utils.logPrint("Chute 1 initialized", true);

  _utils.logPrint("Initializing igniter and valves", true);
  _utils.logPrint("Stand clear of rocket in case of mislight", true);
  pinMode(_constants.IGNITER_PIN, OUTPUT);
  digitalWrite(_constants.IGNITER_PIN, LOW);
  pinMode(_constants.LOX_VALVE_PIN, OUTPUT);
  pinMode(_constants.FUEL_VALVE_PIN, OUTPUT);
  pinMode(_constants.LOX_INPUT_VALVE_PIN, OUTPUT);
  pinMode(_constants.FUEL_INPUT_VALVE_PIN, OUTPUT);
  _utils.logPrint("Igniter and valves initialized", true);

  _utils.logPrint("Contacting lab", true);
  pinMode(_constants.LAB_COM_PIN, OUTPUT);
  digitalWrite(_constants.LAB_COM_PIN, HIGH);
  _utils.logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.LAB_COM_PIN, LOW);
  pinMode(_constants.LAB_COM_PIN, INPUT);
  while (digitalRead(_constants.LAB_COM_PIN) == 0);
  _utils.logPrint("Lab contact established", true);

  _utils.logPrint("All modules initialized", true);
  _utils.logPrint("Continue with launch procedure", true);
  _utils.logPrint("", true);
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

