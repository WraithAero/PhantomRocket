#ifndef PhantomLibraries_h
#define PhantomLibraries_h

#include "Arduino.h"
#include "Stage.h"
#include "PhantomConstants.h"
#include "ThrottleLevel.h"

class PhantomLibraries{

	public:
		PhantomLibraries(PhantomConstants constants); //Contructor
		
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
};
#endif