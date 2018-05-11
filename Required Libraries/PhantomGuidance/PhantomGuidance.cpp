#include "Arduino.h"
#include "PhantomModules.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "Stage.h"
#include "ThrottleLevel.h"

void PhantomGuidance::PhantomGuidance(PhantomConstants constants, PhantomModules modules, PhantomUtils utils){
	_constants = constants;
	_modules = modules;
	_utils = utils;
	
	pitch.SetMode(AUTOMATIC);
	roll.SetMode(AUTOMATIC);
	yaw.SetMode(AUTOMATIC);
	
	setInputs();
}

void PhantomGuidance::steer(double pitchAngle, double rollAngle, double yawAngle){
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

void PhantomGuidance::setAllFins(double angle){
	writeNorth(angle);
	writeEast(angle);
	writeSouth(angle);
	writeWest(angle);
}

void PhantomGuidance::setSPs(double newPitchSP, double newRollSP, double newYawSP){
	pitchSP = newPitchSP;
	rollSP = newRollSP;
	yawSP = newYawSP;
}

void PhantomGuidance::compute(){
	pitch.Compute();
	roll.Compute();
	yaw.Compute();
}

void PhantomGuidance::setInputs(){
	pitchInput = _modules.getPitch();
	rollInput = _modules.getRoll();
	yawInput = _modules.getYaw();
}

void PhantomGuidance::throttle(ThrottleLevel level) {
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

void PhantomGuidance::writeNorth(double angle){
	north.write(angle);
}

void PhantomGuidance::writeEast(double angle){
	east.write(angle);
}

void PhantomGuidance::writeSouth(double angle){
	south.write(angle);
}

void PhantomGuidance::writeWest(double angle){
	west.write(angle);
}

double PhantomGuidance::getNorth(){
	return northAngle;
}

double PhantomGuidance::getEast(){
	return eastAngle;
}

double PhantomGuidance::getSouth(){
	return southAngle;
}

double PhantomGuidance::getWest(){
	return westAngle;
}

void PhantomGuidance::guidanceLoop(Stage newStage){
	if(currentStage != newStage){
		currentStage = newStage;	
	}
	_utils.logPrint("Stage: ", false);
	_utils.printStage(newStage, true);
	switch(newStage){
		case Abort:
			seperateStage();
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
			_modules.openInputValves();
			if(!loaded){
				_utils.logPrint("Loading fuel...");
				loaded = true;
			}
			printTelemetry(true);
			break;
		case Launch:
			throttle(ThrottleLevel.Full);
			delay(10);
			digitalWrite(_constants.IGNITER_PIN, HIGH);
			stageRocket(Stage.Burn);
			executePIDs();
			break;
		case Burn:
			executePID();
			if(_modules.getLOXTankPressure <= LOX_EMPTY_PRESSURE && _modules.getFuelTankPressure < _constants.FUEL_EMPTY_PRESSURE){
				stageRocket(Stage.Coast);
			}
			break;
	}
}

void PhantomGuidance::stageRocket(Stage newStage){
	currentStage = newStage;
	_utils.logPrint("Staging to: ", false");
	_utils.stagePrint(newStage, true);
}

void PhantomGuidance::seperateStage(){
	digitalWrite(_constants.STAGE_SEP_PIN, HIGH);
}

void PhantomGuidance::executePIDs(){
	setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
	setInputs();

	compute();
	steer(pitchOutput, rollOutput, yawOutput);
}

void PhantomGuidance::printTelemetry(boolean ground){
	_utils.logPrint("LOX_Pressure: ", false);
	_utils.dataPrint(_modules.getLOXTankPressure, false);
	_utils.logPrint(" ", false);
	_utils.logPrint(_constants.PRESSURE_UNITS, true);
	
	_utils.logPrint("Fuel Pressure: ", false);
	_utils.dataPrint(_modules.getFuelTankPressure, false);
	_utils.logPrint(" ", false);
	_utils.logPrint(_constants.PRESSURE_UNITS, true);
	
	_utils.logPrint("", true);
	
	_utils.logPrint("LOX Tank Pressure: ", false);
	_utils.dataPrint(_modules.getLOXTankPressure, false);
	_utils.logPrint(" ", false);
	_utils.logPrint(_constants.PRESSURE_UNITS, true);
	
	_utils.logPrint("", true);
	
	_utils.logPrint("Fuel Tank Pressure: ", false);
	_utils.dataPrint(_modules.getFuelTankPressure, false);
	_utils.logPrint(" ", false);
	_utils.logPrint(_constants.PRESSURE_UNITS, true);
	
	_utils.logPrint("", true);
	
	if(!ground){
		_utils.logPrint("Relative Altitude: ", false);
		_utils.dataPrint(_modules.getAltitude(false), false);
		_utils.logPrint(" ", false);
		_utils.logPrint(_constants.HEIGHT_UNITS, true);
		_utils.logPrint("", true);
		
		_utils.logPrint("True Altitude: ", false);
		_utils.dataPrint(_modules.getAltitude(true), false);
		_utils.logPrint(" ", false);
		_utils.logPrint(_constants.HEIGHT_UNITS, true);
		
		_utils.logPrint("", true);_utils.logPrint("Comb. Chamber Temperature: ", false);
		_utils.dataPrint(_modules.getCCTemperature, false);
		_utils.logPrint(" ", false);
		_utils.logPrint(_constants.TEMP_UNITS, true);
		
		_utils.logPrint("", true);
		
		_utils.logPrint("Nozzle Temperature: ", false);
		_utils.dataPrint(_modules.getNozzleTemperature, false);
		_utils.logPrint(" ", false);
		_utils.logPrint(_constants.TEMP_UNITS, true);
		
		_utils.logPrint("", true);
	}
}