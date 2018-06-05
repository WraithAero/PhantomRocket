#include "Arduino.h"
#include "PhantomConstants.h"
#include "ThrottleLevel.h"
#include "Stage.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "CurieIMU.h"
#include "Servo.h"
#include "Adafruit_MAX31855.h"
#include "Adafruit_MPL3115A2.h"

void PhantomLib::steer(double pitchAngle, double rollAngle, double yawAngle){
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

void PhantomLib::setAllFins(double angle){
	writeNorth(angle);
	writeEast(angle);
	writeSouth(angle);
	writeWest(angle);
}

void PhantomLib::setSPs(double newPitchSP, double newRollSP, double newYawSP){
	pitchSP = newPitchSP;
	rollSP = newRollSP;
	yawSP = newYawSP;
}

void PhantomLib::compute(){
	pitch.Compute();
	roll.Compute();
	yaw.Compute();
}

void PhantomLib::setInputs(){
	pitchInput = getPitch();
	rollInput = getRoll();
	yawInput = getYaw();
}

void PhantomLib::throttle(ThrottleLevel level) {
  switch (level) {
    case Off:
      digitalWrite(_constants.LOX_VALVE_PIN, LOW);
      digitalWrite(_constants.FUEL_VALVE_PIN, LOW);
      break;
    case Full:
      digitalWrite(_constants.LOX_VALVE_PIN, HIGH);
      digitalWrite(_constants.FUEL_VALVE_PIN, HIGH);
      break;
  }
}

void PhantomLib::writeNorth(double angle){
	north.write(angle);
}

void PhantomLib::writeEast(double angle){
	east.write(angle);
}

void PhantomLib::writeSouth(double angle){
	south.write(angle);
}

void PhantomLib::writeWest(double angle){
	west.write(angle);
}

double PhantomLib::getNorth(){
	return northAngle;
}

double PhantomLib::getEast(){
	return eastAngle;
}

double PhantomLib::getSouth(){
	return southAngle;
}

double PhantomLib::getWest(){
	return westAngle;
}

void PhantomLib::guidanceLoop(Stage newStage){
	if(currentStage != newStage){
		currentStage = newStage;
		logPrint("Stage: ", false);
		printStage(newStage, true);		
	}
	
	switch(newStage){
		case Abort:
			seperateStage();
			deployChute();
			break;
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
			break;
		case Loading_Fuel:
			openInputValves();
			if(!loaded){
				logPrint("Loading fuel...");
				loaded = true;
			}
			printTelemetry(true);
			break;
		case Launch:
			logPrint("", true);
			logPrint("||||||||||LAUNCH|||||||||||", true);
			logPrint("", true);
			
			throttle(ThrottleLevel.Full);
			delay(10);
			digitalWrite(_constants.IGNITER_PIN, HIGH);
			stageRocket(Stage.Burn);
			engine.SetMode(AUTOMATIC);
			executePIDs();
			printTelemetry(false);
			break;
		case Burn:
			executePID();
			printTelemetry(false);
			if(getLOXTankPressure <= LOX_EMPTY_PRESSURE && getFuelTankPressure < _constants.FUEL_EMPTY_PRESSURE){
				stageRocket(Stage.Coast);
			}
			break;
		case Coast:
			printTelemetry(false);
			if(getAltitude(false) <= _constants.CHUTE_DEPLOY_ALT){
				stageRocket(Stage.Chute);
			break;
		case Chute:
			printTelemetry(false);
			deployChute();
			break;
	}
}

void PhantomLib::stageRocket(Stage newStage){
	currentStage = newStage;
	logPrint("Staging to: ", false");
	stagePrint(newStage, true);
}

void PhantomLib::seperateStage(){
	digitalWrite(_constants.STAGE_SEP_PIN, HIGH);
}

void PhantomLib::executePIDs(){
	setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll(), getOptimalCCPressure());
	setInputs();

	compute();
	steer(pitchOutput, rollOutput, yawOutput);
}

void PhantomLib::printTelemetry(boolean ground){
	logPrint("LOX_Pressure: ", false);
	dataPrint(getLOXTankPressure, false);
	logPrint(" ", false);
	logPrint(_constants.PRESSURE_UNITS, true);
	
	logPrint("Fuel Pressure: ", false);
	dataPrint(getFuelTankPressure, false);
	logPrint(" ", false);
	logPrint(_constants.PRESSURE_UNITS, true);
	
	logPrint("", true);
	
	logPrint("LOX Tank Pressure: ", false);
	dataPrint(getLOXTankPressure, false);
	logPrint(" ", false);
	logPrint(_constants.PRESSURE_UNITS, true);
	
	logPrint("", true);
	
	logPrint("Fuel Tank Pressure: ", false);
	dataPrint(getFuelTankPressure, false);
	logPrint(" ", false);
	logPrint(_constants.PRESSURE_UNITS, true);
	
	logPrint("", true);
	
	if(!ground){
		logPrint("Relative Altitude: ", false);
		dataPrint(getAltitude(false), false);
		logPrint(" ", false);
		logPrint(_constants.HEIGHT_UNITS, true);
		
		logPrint("", true);
		
		logPrint("True Altitude: ", false);
		dataPrint(getAltitude(true), false);
		logPrint(" ", false);
		logPrint(_constants.HEIGHT_UNITS, true);
		
		logPrint("", true);
		
		logPrint("Comb. Chamber Temperature: ", false);
		dataPrint(getCCTemperature, false);
		logPrint(" ", false);
		logPrint(_constants.TEMP_UNITS, true);
		
		logPrint("", true);
		
		logPrint("Nozzle Temperature: ", false);
		dataPrint(getNozzleTemperature, false);
		logPrint(" ", false);
		logPrint(_constants.TEMP_UNITS, true);
		
		logPrint("", true);
		
		logPrint("Comb. Chamber Pressure: ", false);
		dataPrint(getCCPressure, false);
		logPrint(" ", false);
		logPrint(_constants.PRESSURE_UNITS, true);
	
		logPrint("", true);
	}
}
void PhantomLib::deployChute(){
	digitalWrite(_constants.PARACHUTE_PIN, HIGH);
}

void PhantomLib::openInputValves(){
	digitalWrite(_constants.LOX_INPUT_VALVE_PIN, HIGH);
	digitalWrite(_constants.FUEL_INPUT_VALVE_PIN, HIGH);
}

double PhantomLib::getOptimalPitch() {
  double p;
  if (currentStage != Coast) {
    p = 90;
  } else {
    p = getPitch() + .05;
  }
  return p;
}
double PhantomLib::getOptimalRoll() {
  return _constants.OPTIMAL_ROLL;
}
double PhantomLib::getOptimalYaw() {
  return _constants.OPTIMAL_YAW;
}
double PhantomLib::getOptimalCCPressure(){
	return _constants.OPTIMAL_CC_PRESSURE;
}

double PhantomLib::getCCTemperature(){
  return cc_thermocoil.get();
}

double PhantomLib::getNozzleTemperature(){
	return nozzle_thermocoil.get();
}

double PhantomLib::getAltitude(boolean absolute) {
  if(absolute){
	  return baro.getAltitude();
  } else {
	  return baro.getAltitude() - initAlt;
  }
}

void PhantomLib::setGyro() {
  CurieIMU.readGyroScaled(gx, gy, gz);
}

float PhantomLib::getPitch() {
  setGyro();
  return gx;
}

float PhantomLib::getRoll() {
  setGyro();
  return gy;
}

float PhantomLib::getYaw() {
  setGyro();
  return gz;
}

double PhantomLib::getLOXTankPressure(){
	return (analogRead(_constants.LOX_PRESSURE_PIN) * _constants.LOX_PRESSURE_CALIBRATION);
}

double PhantomLib::getFuelTankPressure(){
	return (analogRead(_constants.FUEL_PRESSURE_PIN) * _constants.FUEL_PRESSURE_CALIBRATION);
}

double PhantomLib::getCCPressure(){
	return (analogRead(_constants.CC_PRESSURE_PIN) * _constants.CC_PRESSURE_CALIBRATION);
}

void PhantomLib::signalPad(){
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
			command = currentStage;
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
		command = currentStage;
	}
	return command;
}

PhantomLib::initialize(){
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

  logPrint("Contacting pad...", true);
  signalPad();
  logPrint("Waiting for pad response...", true);
  while (digitalRead(_constants.LAUNCHPAND_COM_PIN) == 0);
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

  logPrint("Initializing chute...", true);
  logPrint("Contacting chute ...", true);
  pinMode(_constants.PARACHUTE_PIN, OUTPUT);
  digitalWrite(_constants.PARACHUTE_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(_constants.PARACHUTE_PIN, LOW);
  pinMode(_constants.PARACHUTE_PIN, INPUT);
  while (digitalRead(_constants.PARACHUTE_PIN) == 0);
  logPrint("Chute initialized", true);

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