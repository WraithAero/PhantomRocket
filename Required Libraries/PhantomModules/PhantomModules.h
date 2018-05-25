#ifndef PhantomModules_h
#define PhantomModules_h

#include "Arduino.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "PhantomGuidance.h"
#include "Stage.h"

class PhantomModules {
	public:
	PhantomModules(PhantomConstants constants, PhantomUtils utils, PhantomGuidance guidance); //Contructor
	
	void deployChutes(); //Self explanatory
	
	void openInputValves(); //For tank purge and tank fill
	
	double getOptimalPitch(); //Gets pitch for altitude
	double getOptimalRoll(); //Gets roll for altitude*
	double getOptimalYaw(); //Gets yaw for altitude*
	double getOptimalCCPressure(); //Gets optimal combustion chamber pressure*
	//*Returns a static number
	
	double getCCTemperature(); //Gets temperature of Comb. Chamber thermocoil
	double getNozzleTemperature(); //Gets temperature of nozzle thermocoil
	double getAltitude(); //Gets barometric altitude
	
	float getPitch(); //Gets pitch
	float getRoll(); //Gets roll
	float getYaw(); //Gets yaw
	void setGyro(); //Sends new calibration to gyro
	
	double getLOXTankPressure(); //Gets LOX Tank pressure using analog pressure sensor
	double getFuelTankPressure(); //Gets Fuel Tank pressure using analog pressure sensor
	double getCCPressure(); //Gets Comb. Chamber pressure using analog pressure sensor
	
	void getPadCommand(); //Gets command from pad
	
	private:
	PhantomConstants _constants; //Local constants instance
	PhantomUtils _utils; //Local utils instance
	PhantomGuidance _guidance; //Local guidance instance
	
	Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //Barometer instance
	Adafruit_MAX31855 nozzle_thermocoil = Adafruit_MAX31855(13, NOZZLE_THERMOCOIL_PIN); //Nozzle thermocoil instance
	Adafruit_MAX31855 cc_thermocoil = Adafruit_MAX31855(13, CC_THERMOCOIL_PIN); //Nozzle thermocoil instance
	double initAlt; //Initial altitude for relative altitude method
	
	float gx, gy, gz; //Gyro values
	
	void signalPad(); //Signals pad for Comm methods
};

#endif