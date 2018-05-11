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
			break;
		case Launch:
			throttle(ThrottleLevel.Full);
			delay(10);
			digitalWrite(_constants.IGNITER_PIN, HIGH);
			guidanceLoop(Stage.Burn);
			break;
		case Burn:
			executePID();
			if()
			break;
	}
}

void PhantomGuidance::seperateStage(){
	digitalWrite(_constants.STAGE_SEP_PIN, HIGH);
	stageRocket(Stage.Chute);
}

void PhantomGuidance::executePIDs(){
	setSPs(getOptimalPitch(), getOptimalYaw(), getOptimalRoll());
	setInputs();

	compute();
	steer(pitchOutput, rollOutput, yawOutput);
}