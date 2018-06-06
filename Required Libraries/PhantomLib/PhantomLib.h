#ifndef PhantomLib_h
#define PhantomLib_h

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
		
		boolean arrayEquals(int array_one[], int array_two[]);
		void logPrint(String message, boolean ln);
		void stagePrint(Stage toPrint, boolean ln);
		void dataPrint(int toPrint, boolean ln);
		
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
		
		boolean arrayEquals(int array_one[], int array_two[]);
		void logPrint(String message, boolean ln);
		void stagePrint(Stage toPrint, boolean ln);
		void dataPrint(int toPrint, boolean ln);
		
		private:
		File main_log; 
		
		Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //Barometer instance
		Adafruit_MAX31855 nozzle_thermocoil = Adafruit_MAX31855(13, NOZZLE_THERMOCOIL_PIN); //Nozzle thermocoil instance
		Adafruit_MAX31855 cc_thermocoil = Adafruit_MAX31855(13, CC_THERMOCOIL_PIN); //Nozzle thermocoil instance
		double initAlt; //Initial altitude for relative altitude method
		
		float gx, gy, gz; //Gyro values
		
		void signalPad(); //Signals pad for Comm methods
		
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
		
		PhantomConstants _constants;
};
#endif