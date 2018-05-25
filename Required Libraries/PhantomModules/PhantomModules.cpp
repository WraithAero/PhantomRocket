#include "Arduino.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "PhantomGuidance.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "CurieIMU.h"
#include "Servo.h"
#include "Adafruit_MAX31855.h"
#include "Adafruit_MPL3115A2.h"
//Imports

PhantomModules::PhantomModules(PhantomConstants constants, PhantomUtils utils, PhantomGuidance guidance){ //Constructor
  _constants = constants;
  _utils = utils;
  _guidance = guidance //Imports instances of classes
  Serial.begin(9600); //Establishes serial connection
  while (!Serial); //Waits for serial connection to be established
  Serial.println("Initializing flight computer...");
  Serial.println();

  Serial.print("Initializing SD card...");

  if (!SD.begin(_constants.SD_PIN)) { //If SD is not responding
    Serial.println("initialization failed!");
    while (true); //Ends program
  }
  Serial.println("initialization done.");
  Serial.println();
  Serial.print("All further reports will be output to both ");
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
  _guidance.north.attach(_constants.NORTH_PIN);
  _guidance.east.attach(_constants.EAST_PIN);
  _guidance.south.attach(_constants.SOUTH_PIN);
  _guidance.west.attach(_constants._constants.WEST_PIN);
  _guidance.setAllFins(_constants.STANDARD_FIN_VALUE);
  _utils.logPrint("Servo initialization complete. Testing fins...", true);
  _guidance.setAllFins(_constants.NEAR_FIN_VALUE);
  _guidance.setAllFins(_constants.FAR_FIN_VALUE);
  _guidance.setAllFins(_constants.STANDARD_FIN_VALUE);
  _utils.logPrint("Fin Test Complete", true);
  _utils.logPrint("", true);

  _utils.logPrint("Initializing Barometer...", true);
  baro.begin();
  initAlt = baro.getAltitude();
  _utils.logPrint("Barometer initialization complete", true);
  _utils.logPrint("", true);

  _utils.logPrint("Initializing chute...", true);
  _utils.logPrint("Contacting chute ...", true);
  pinMode(_constants.PARACHUTE_PIN, OUTPUT);
  digitalWrite(_constants.PARACHUTE_PIN, HIGH);
  _utils.logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.PARACHUTE_PIN, LOW);
  pinMode(_constants.PARACHUTE_PIN, INPUT);
  while (digitalRead(_constants.PARACHUTE_PIN) == 0);
  _utils.logPrint("Chute initialized", true);

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

void PhantomModules::deployChute(){
	digitalWrite(_constants.PARACHUTE_PIN, HIGH);
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
  return _constants.OPTIMAL_ROLL;
}
double PhantomModules::getOptimalYaw() {
  return _constants.OPTIMAL_YAW;
}
double PhantomModules::getOptimalCCPressure(){
	return _constants.OPTIMAL_CC_PRESSURE;
}

double PhantomModules::getCCTemperature(){
  return cc_thermocoil.get();
}

double PhantomModules::getNozzleTemperature(){
	return nozzle_thermocoil.get();
}

double PhantomModules::getAltitude(boolean absolute) {
  if(absolute){
	  return baro.getAltitude();
  } else {
	  return baro.getAltitude() - initAlt;
  }
}

void PhantomModules::setGyro() {
  CurieIMU.readGyroScaled(gx, gy, gz);
}

float PhantomModules::getPitch() {
  setGyro();
  return gx;
}

float PhantomModules::getRoll() {
  setGyro();
  return gy;
}

float PhantomModules::getYaw() {
  setGyro();
  return gz;
}

double PhantomModules::getLOXTankPressure(){
	return (analogRead(_constants.LOX_PRESSURE_PIN) * _constants.LOX_PRESSURE_CALIBRATION);
}

double PhantomModules::getFuelTankPressure(){
	return (analogRead(_constants.FUEL_PRESSURE_PIN) * _constants.FUEL_PRESSURE_CALIBRATION);
}

double PhantomModules::getCCPressure(){
	return (analogRead(_constants.CC_PRESSURE_PIN) * _constants.CC_PRESSURE_CALIBRATION);
}

void PhantomModules::signalPad(){
	pinMode(_constants.LAUNCHPAD_COM_PIN, OUTPUT);
	digitalWrite(_constants.LAUNCHPAD_COM_PIN, HIGH);
	int initMillis = millis();
	int currentMillis;
	do {
		currentMillis = millis();
	} while (currentMillis - initMillis < 90);
	digitalWrite(_constants.LAUNCHPAD_COM_PIN, LOW);
	pinMode(_constants.LAUNCHPAD_COM_PIN, INPUT);
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
		} while (commandSequence[2] == _constants.NULL_SEQUENCE[2] && currentMillis - initMillis < 500);
	
		if (currentMillis - initMillis > 500) {
			command = _guidance.currentStage;
		}
	
		if (arrayEquals(commandSequence, LAUNCH_SEQUENCE)) {
			command = Stage.Launch;
		} else if (arrayEquals(commandSequence, ABORT_SEQUENCE)) {
			command = Stage.Abort;
		} else if (arrayEquals(commandSequence, FUEL_LOADING_SEQUENCE)) {
			command = Stage.Loading_Fuel;
		} else {
			signalPad();
			delay(9);
			signalPad();
			delay(90);
			signalPad();
			command = getPadCommand();
		}
	} else {
		command = _guidance.currentStage;
	}
	return command;
}