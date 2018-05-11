#ifndef PhantomConstants_h
#define PhantomConstants_h

#include "Arduino.h"

class PhantomConstants {
	
public:
const int NORTH_PIN = 1;
const int EAST_PIN = 2;
const int SOUTH_PIN = 3;
const int WEST_PIN = 4;

const int FAR_FIN_VALUE = 180;
const int NEAR_FIN_VALUE = 0;
const int STANDARD_FIN_VALUE = 90;
const int FIN_DEGREES_PER_SECOND = 60;
const int CHUTE_DEPLOY_ALT = 1500;

const String logName = "main_control.log";

const int LAUNCHPAD_COM_PIN = 5;
const int PARACHUTE_PIN = 6;
const int STAGE_PIN = 7;
const int SD_PIN = 8;
const int IGNITER_PIN = 9;
const int LAB_COM_PIN = 10;
const int LOX_VALVE_PIN = 11;
const int FUEL_VALVE_PIN = 12;
const int LOX_INPUT_VALVE_PIN = 13;
const int FUEL_INPUT_VALVE_PIN = 14;
const int NOZZLE_THERMOCOIL_PIN = 15;
const int CC_THERMOCOIL_PIN = 16;
const int CLK_PIN = 17;
const int MOSI_PIN = 18;
const int MISO_PIN = 19;
const int STAGE_SEP_PIN = 20;

const int LOX_PRESSURE_PIN = A1;


int LAUNCH_SEQUENCE[] = {1, 0, 1};
int ABORT_SEQUENCE[] = {0, 1, 0};
int FUEL_LOADING_SEQUENCE[] = {1, 1, 1};
int NULL_SEQUENCE[] = {0, 0, 0};
}

#endif