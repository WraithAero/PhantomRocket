#ifndef PhantomGuidance_h
#define PhantomGuidance_h

#include "Arduino.h"
#include "ThrottleLevel.h"
#include "PhantomConstants.h"
#include "PhantomModules.h"
#include "Stage.h"

class PhantomGuidance {
	public:
	PhantomGuidance(PhantomConstants constants, PhantomModules modules, PhantomUtils utils);
	
	void steer(double pitchAngle, double rollAngle, double yawAngle);
	void setAllFins(double angle);
	
	void setSPs(double newPitchSP, double newRollSP, double newYawSP);
	void compute();
	void setInputs();
	
	void throttle(ThrottleLevel level);
	
	void writeNorth(int angle);
	void writeEast(int angle);
	void writeSouth(int angle);
	void writeWest(int angle);
	
	double getNorth();
	double getEast();
	double getSouth();
	double getWest();
	
	void guidanceLoop(Stage newStage);
	void seperateStage();
	
	void stageRocket(Stage newStage);
	
	void executePIDs();
	
	double northAngle;
	double eastAngle;
	double southAngle;
	double westAngle;
	
	Servo north;
	Servo east;
	Servo south;
	Servo west;
	
	Stage currentStage = On_Pad;
	
	private:
	PhantomConstants _constants;
	PhantomModules _modules;
	PhantomUtils _utils;
	
	double pitchP = 2;
	double pitchI = 5;
	double pitchD = 3;
	
	double rollP = 2;
	double rollI = 5;
	double rollD = 3;
	
	double yawP = 2;
	double yawI = 5;
	double yawD = 3;
	
	double engineP = 2;
	double engineI = 5;
	double engineD = 3;
	
	double pitchOutput;
	double rollOutput;
	double yawOutput;
	double engineOutput;
	
	PID pitch(&pitchInput, &pitchOutput, &pitchSP, pitchP, pitchI, pitchD, DIRECT);
	PID roll(&rollInput, &rollOutput, &rollSP, rollP, rollI, rollD, DIRECT);
	PID yaw(&yawInput, &yawOutput, &yawSP, yawP, yawI, yawD, DIRECT);
	PID engine(&engineInput, &engineOutput, &engineSP, engineP, engineI, engineD, DIRECT);
	
	double pitchSP, rollSP, yawSP, engineSP;
	double pitchInput, rollInput, yawInput, engineInput;
	
	boolean loaded = false;
	
	void printTelemetry(boolean ground);
};

#endif