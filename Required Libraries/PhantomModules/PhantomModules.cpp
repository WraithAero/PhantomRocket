#include "Arduino.h"
#include "PhantomModules.h"
#include "PhantomConstants.h"
#include "PhantomUtils.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "CurieIMU.h"
#include "Servo.h"
#include "Adafruit_MAX31855.h"
#include "Adafruit_MPL3115A2.h"

PhantomModules::PhantomModules(PhantomConstants constants, PhantomUtils utils){
  _constants = constants;
  _utils = utils;
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initializing flight computer...");
  Serial.println();

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  Serial.println();
  Serial.println("All further reports will be output both to");
  Serial.println("Serial and written to the SD file titled" + logName);
  Serial.println();

  logPrint("Contacting pad...", true);
  signalPad();
  logPrint("Waiting for pad response...", true);
  while (digitalRead(LAUNCHPAD_COM_PIN) == 0);
  logPrint("Received pad response", true);

  logPrint("Initializing gyro...", true);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  logPrint("Gyro initialization complete", true);
  logPrint("", true);

  logPrint("Initializing accelerometer...", true);
  CurieIMU.setAccelerometerRange(2);
  logPrint("Accelerometer initialization complete", true);

  logPrint("Initializing fin servos...", true);
  north.attach(NORTH_PIN);
  east.attach(EAST_PIN);
  south.attach(SOUTH_PIN);
  west.attach(WEST_PIN);
  setAllFins(STANDARD_FIN_VALUE);
  logPrint("Servo initialization complete. Testing fins...", true);
  setAllFins(NEAR_FIN_VALUE);
  setAllFins(FAR_FIN_VALUE);
  setAllFins(STANDARD_FIN_VALUE);
  logPrint("Fin Test Complete", true);
  logPrint("", true);

  logPrint("Initializing Barometer...", true);
  baro.begin();
  initAlt = baro.getAltitude();
  logPrint("Barometer initialization complete", true);
  logPrint("", true);

  logPrint("Initializing chutes...", true);
  logPrint("Contacting chute 1...", true);
  pinMode(PARACHUTE_ONE_PIN, OUTPUT);
  digitalWrite(PARACHUTE_ONE_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(PARACHUTE_ONE_PIN, LOW);
  pinMode(PARACHUTE_ONE_PIN, INPUT);
  while (digitalRead(PARACHUTE_ONE_PIN) == 0);
  logPrint("Chute 1 initialized", true);

  logPrint("Contacting chute 2...", true);
  pinMode(PARACHUTE_TWO_PIN, OUTPUT);
  digitalWrite(PARACHUTE_TWO_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(PARACHUTE_TWO_PIN, LOW);
  pinMode(PARACHUTE_TWO_PIN, INPUT);
  while (digitalRead(PARACHUTE_TWO_PIN) == 0);
  logPrint("Chute 2 initialized", true);
  logPrint("Chutes initialized", true);
  logPrint("", true);

  logPrint("Initializing igniter and valves", true);
  logPrint("Stand clear of rocket in case of mislight", true);
  pinMode(IGNITER_PIN, OUTPUT);
  digitalWrite(IGNITER_PIN, LOW);
  pinMode(LOX_VALVE_PIN, OUTPUT);
  pinMode(FUEL_VALVE_PIN, OUTPUT);
  pinMode(LOX_INPUT_VALVE_PIN, OUTPUT);
  pinMode(FUEL_INPUT_VALVE_PIN, OUTPUT);
  logPrint("Igniter and valves initialized", true);

  logPrint("Contacting lab", true);
  pinMode(LAB_COM_PIN, OUTPUT);
  digitalWrite(LAB_COM_PIN, HIGH);
  logPrint("Waiting for response...", true);
  delay(90);
  digitalWrite(LAB_COM_PIN, LOW);
  pinMode(LAB_COM_PIN, INPUT);
  while (digitalRead(LAB_COM_PIN) == 0);
  logPrint("Lab contact established", true);

  logPrint("All modules initialized", true);
  logPrint("Continue with launch procedure", true);
  logPrint("", true);
}