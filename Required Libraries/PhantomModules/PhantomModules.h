#ifndef PhantomModules_h
#define PhantomModules_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "Stage.h"

class PhantomModules {
	public:
	PhantomModules(PhantomConstants constants);
	
	int getNorth();
	int getEast();
	int getSouth();
	int getWest();
	
	void deployChutes();
	
	void openInputValves();
	
	double getOptimalPitch();
	double getOptimalRoll();
	double getOptimalYaw();
	
	double getCCTemp();
	double getAltitude();
	
	float getPitch();
	float getRoll();
	float getYaw();
	void setGyro();
	
	private:
	PhantomConstants _constants;
	PhantomUtils _utils;
};

#endif