#ifndef PhantomGuidance_h
#define PhantomGuidance_h

#include "Arduino.h"
#include "ThrottleLevel.h"
#include "PhantomConstants.h"
#include "PhantomModules.h"

class PhantomGuidance {
	public:
	PhantomGuidance(PhantomConstants constants, PhantomModules modules);
	
	void steer(double pitchAngle, double rollAngle, double yawAngle);
	void setAllFins(double angle);
	
	void setSPs(double newPitchSP, double newRollSP, double newYawSP);
	void compute();
	void setInputs();
	
	void throttle(ThrottleLevel level);
	void openInputValves();
	
	void writeNorth(int angle);
	void writeEast(int angle);
	void writeSouth(int angle);
	void writeWest(int angle);
	
	void stageRocket();
	void seperateStage();
};

#endif