#ifndef PhantomModules_h
#define PhantomModules_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "PhantomModules.h"
#include "PhantomGuidance.h"
#include "Stage.h"

class PhantomModules {
	public:
	PhantomModules(PhantomConstants constants, PhantomUtils _utils, PhantomModules _modules, PhantomGuidance _guidance);
	
	void deployChutes();
	
	void openInputValves();
	
	double getOptimalPitch();
	double getOptimalRoll();
	double getOptimalYaw();
	
	double getCCTemperature();
	double getNozzleTemperature();
	double getAltitude();
	
	float getPitch();
	float getRoll();
	float getYaw();
	void setGyro();
	
	double getLOXTankPressure();
	double getFuelTankPressure();
	
	private:
	PhantomConstants _constants;
	PhantomUtils _utils;
	
	Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
	Adafruit_MAX31855 nozzle_thermocoil = Adafruit_MAX31855(13, NOZZLE_THERMOCOIL_PIN);
	Adafruit_MAX31855 cc_thermocoil = Adafruit_MAX31855(13, CC_THERMOCOIL_PIN);
	double initAlt;
	
	float gx, gy, gz;

	File main_log;
	
	void signalPad();
	void getPadCommand();
};

#endif