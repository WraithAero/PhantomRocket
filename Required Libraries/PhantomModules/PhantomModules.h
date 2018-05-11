#ifndef PhantomModules_h
#define PhantomModules_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "Stage.h"

class PhantomModules {
	public:
	PhantomModules(PhantomConstants constants);
	
	
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
	
	Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
	Adafruit_MAX31855 nozzle_thermocoil = Adafruit_MAX31855(13, NOZZLE_THERMOCOIL_PIN);
	Adafruit_MAX31855 cc_thermocoil = Adafruit_MAX31855(13, CC_THERMOCOIL_PIN);
	double initAlt;
	
	float gx, gy, gz;

	File main_log;
};

#endif