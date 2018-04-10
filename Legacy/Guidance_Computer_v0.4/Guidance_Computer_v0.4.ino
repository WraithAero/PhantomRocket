/*
 __________.__                   __                    ____ 
\______   \  |__ _____    _____/  |_  ____   _____   /_   |
 |     ___/  |  \\__  \  /    \   __\/  _ \ /     \   |   |
 |    |   |   Y  \/ __ \|   |  \  | (  <_> )  Y Y  \  |   |
 |____|   |___|  (____  /___|  /__|  \____/|__|_|  /  |___|
               \/     \/     \/                  \/        


  This software was created for the guidance computer
  driver for the Phantom I rocket. It uses 4 servos 
  and uses the below scheme for steering:
  0 = stowed at 90* vertical. (|)
  1 = deployed 120* at the top. (\)
  2 = deploted 60* at the top. (/)

  Pitch North:
   Fin-N: 0
   Fin-W: 1
   Fin-S: 0
   Fin-E: 2
   
  Pitch South:
   Fin-N: 0
   Fin-W: 2
   Fin-S: 0
   Fin-E: 1
  
  Yaw East:
    Fin-N: 2 
    Fin-W: 0
    Fin-S: 1
    Fin-E: 0

  Yaw West:
    Fin-N: 1
    Fin-W: 0
    Fin-S: 2
    Fin-E: 0
    
  

  Credits:
  Xavier Beech - Lead Dev
 */

#include <SD.h>
#include <BMI160.h>
#include <CurieIMU.h>
#include <Servo.h>

File log_file; //SD card var
File density_file; //SD card var for getting density
Servo Fin_North; //Fin control Servos
Servo Fin_West;
Servo Fin_South;
Servo Fin_East;
const long hour = 3600000; //Time stamp variables
const long minute = 60000;
const long second = 1000;
double air_speed = 0; //Air speed variable. derived from acceleration felt
float gx, gy, gz; //Gyro variables
const int ACTIVATION_PIN = 1; //Pin for Activation
const int LED_PIN = 13; //Fin for notification LED
const int NO = 0; //NO/YES variables. For use with writeToLog() function
const int YES = 1;
const double DEGREE_IN_RADIANS = 0.017453; //1 degree of arc in radians
const double LIFT_COEFFICIENT = 2*PI*DEGREE_IN_RADIANS; //Lift coefficient 
const int WIDTH = 3; //Fin width in inches
const int HEIGHT = 8; //Fin height in inches
const int WING_AREA = (WIDTH*HEIGHT)/2; //Area of tringular fin
const int DRY_MASS = 200; //Mass of rocket in Kg
const int FUEL_MASS = 50; 
int ax; //Acclerometer variables
int ay;
int az;
long timeNow; //Variable for speed calculation. Subtract timeThen and you get timePassed.
long timeThen;
long timePassed;
boolean activated = false; //Activated boolean variable. True tells arduino to go forward with guidance calculations. 
int exe_north; //Executable movement vars for each fin. Set by calculateFinMovements() function. 
int exe_south;
int exe_west;
int exe_east;
int prev_north;
int prev_south;
int prev_west;
int prev_east;


void reset_fins(){ //Set fins to original position        finished
  setFins_ALL(0);
  delay(400);
}

void led_blink(int reps,int delay_time){ //Blink LED for notification. Finished
  for(int counter = 0; (counter + 1) <= reps; reps += 1){
    digitalWrite(LED_PIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_PIN, LOW);
    delay(delay_time);
  }
}

void setFins_ALL(int fin_position){ //Set all fins to certain position. Used in roll and testing operations Finished
  if(fin_position > 180){
    fin_position -= 180;
  }
  Fin_North.write(fin_position);
  Fin_West.write(fin_position);
  Fin_South.write(fin_position);
  Fin_East.write(fin_position);
}

void testFins(){ //Function for ground testing fins. Finished, until decided which servo/gimbal will be used. Then adjust speeds
  setFins_ALL(120);
  delay(400);
  setFins_ALL(60);
  delay(400);
  reset_fins();
  led_blink(2,100);
}

void printTimeStamp(){ //For printing time stamps in log entries //Finished
  timeNow = millis();
  byte hours = timeNow / hour;                       
  byte minutes = (timeNow % hour) / minute ;         
  byte seconds = ((timeNow % hour) % minute) / second;
  if(hours < 10){
   log_file.println('0');
  }
  log_file.println(hours, DEC); 
  if(minutes < 10){
   log_file.println('0');
  }
  log_file.println(minutes, DEC); 
  if(seconds < 10){
   log_file.println('0');
  }
  log_file.print(seconds, DEC); 
  log_file.print("-----");
}

void getOrientation(){ //Gets orientation (in mGs) of A101. Needs to be edited to get actual orientation, not just g values, so placeholder for now.  
  ax = CurieIMU.readAccelerometer(X_AXIS);
  ay = CurieIMU.readAccelerometer(Y_AXIS);
  az = CurieIMU.readAccelerometer(Z_AXIS);
  
  ax = (ax/32768)*CurieIMU.getAccelerometerRange();
  ay = (ay/32768)*CurieIMU.getAccelerometerRange();
  az = (az/32768)*CurieIMU.getAccelerometerRange();
}

void getTravel(){ //Reads gyro for movement.
  CurieIMU.readGyroScaled(gx, gy, gz);
}

int getDensity(){
  density_file = SD.open("density.txt", FILE_READ);
  int density = density_file.read();
  density_file.close();
  return density;
}
int getLift(){
  double lift = .5 * LIFT_COEFFICIENT;
  lift *= getDensity();
  lift *= air_speed^2;
  lift /= 2;
  lift *= WING_AREA;
  return lift;
}

void calculateFinMovements(boolean stabilise, int x, int y, int z){  //Unfinished
  if(stabilise = true){
    x = 90;
    y = 0;
    z = 0;
  }
  getOrientation();
  x -= ax;
  y -= ay;
  z -= az;
    
}

void refreshSpeed(){ //For setting air_speed variable. 
  float temp_air_speed;
  timeNow = millis();
  timePassed = timeNow - timeThen;
  getOrientation();
  temp_air_speed = (ax - 1000);
  temp_air_speed = (temp_air_speed * 9.81);
  air_speed *= 1000;
  air_speed += (temp_air_speed * timePassed);
  air_speed /= 1000;  
  timeThen = timeNow;
}

void recordSpeed(){ //Logs speed. 
  log_file = SD.open("spd_log_file.txt", FILE_WRITE);
  printTimeStamp();
  log_file.print("Speed: ");
  log_file.println(air_speed);
  log_file.close();
}

void recordMovement(){ //Logs next planned manuever. Currently placeholder. 
  log_file = SD.open ("ortn_log_file.txt", FILE_WRITE);
  printTimeStamp();
  getOrientation();
  log_file.print("X: ");
  log_file.print(ax);
  log_file.print("   ");
  log_file.print("Y: ");
  log_file.print(ay);
  log_file.print("   ");
  log_file.print("Z: ");
  log_file.println(az);
  log_file.close();
}

void writeToLog(String message, int lnOrNot){ //Write to log file function
  log_file = SD.open("main_log_file.txt", FILE_WRITE);
  if(lnOrNot = NO){
    log_file.print(message);
  }
  else if(lnOrNot = YES){
    log_file.println(message);
  }
}

void executeMovement(){ //For executing exe_ variables. 
  calculateMovement();
  
}

void setup() {
  writeToLog("Attaching servos..", 0);
  Fin_North.attach(9); //Attaches servos to PWM pins
  Fin_West.attach(6);
  Fin_South.attach(5);
  Fin_East.attach(3);
  writeToLog("Servo attachment completed", 0);
  pinMode(LED_PIN, OUTPUT); //Sets LED Pin to output mode.
  pinMode(ACTIVATION_PIN, INPUT); //Sets activation pin to input mode. 
  CurieIMU.begin(); //Initializes gyros. 
  writeToLog("Initializing gyro...", 0);
  CurieIMU.setGyroRange(250);
  CurieIMU.setAccelerometerRange(8);
  CurieIMU.setAccelerometerRate(25);
  writeToLog("Gyros complete", 0);
  setFins_ALL(0);

  testFins(); //Tests fins. 
}

void loop() {
  double activationVoltage = digitalRead(ACTIVATION_PIN);
  if(activationVoltage > 0){
    activated = true;
  }
  if(activated = true){
    refreshSpeed();
    recordSpeed();
    calculateFinMovements(true, 90,0,0);
    recordMovement();
    executeMovement();    
  }
}
