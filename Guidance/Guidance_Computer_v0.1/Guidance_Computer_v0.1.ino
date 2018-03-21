#include <BMI160.h>
#include <CurieIMU.h>
#include <Servo.h>

/*
 __________.__                   __                    ____ 
\______   \  |__ _____    _____/  |_  ____   _____   /_   |
 |     ___/  |  \\__  \  /    \   __\/  _ \ /     \   |   |
 |    |   |   Y  \/ __ \|   |  \  | (  <_> )  Y Y  \  |   |
 |____|   |___|  (____  /___|  /__|  \____/|__|_|  /  |___|
               \/     \/     \/                  \/        


  This software was created for the guidance computer
  driver for the Phantom I rocket. It uses 4 servos (robotshop.com's HS-805BB)
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
  Xavier Beech - Primary Software Dev
 */
Servo Fin_North;
Servo Fin_West;
Servo Fin_South;
Servo Fin_East;
CurieIMU gyro;
float gx, gy, gz;
const int LED_PIN = 13;
int lastY = 0;
int lastX = 0;
int lastZ = 0;
int ax;
int ay;
int az;

void reset_fins(){
  setFins_ALL(0);
  delay(400);
}

void led_blink(int reps,int delay_time){
  for(int counter = 0; (counter + 1) <= reps; reps += 1){
    digitalWrite(LED_PIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_PIN, LOW);
    delay(delay_time);
  }
}

void setFins_ALL(int fin_position){
  if(fin_position > 180){
    fin_position -= 180;
  }
  Fin_North.write(fin_position);
  Fin_West.write(fin_position);
  Fin_South.write(fin_position);
  Fin_East.write(fin_position);
}

void testFins(){
  setFins_ALL(FIN_POSITION_ONE);
  delay(800);
  setFins_ALL(FIN_POSITION_TWO);
  delay(400);
  reset_fins();
  led_blink(2,100);
}

void getOrientation(){
  ax = gyro.readAccelerometer(X_AXIS);
  ay = gyro.readAccelerometer(Y_AXIS);
  az = gyro.readAccelerometer(Z_AXIS);
}

void getTravel(){
  gyro.readGyroScaled(gx, gy, gz);
}

void calculateFinMovements_STATIONARY(){
  
}

void setup() {
  Fin_North.attach(9);
  Fin_West.attach(6);
  Fin_South.attach(5);
  Fin_East.attach(3);
  pinMode(LED_PIN, OUTPUT);
  gyro.begin();
  gyro.setGyroRange(250);
  gyro.setAccelerometerRange(8);
  gyro.setAccelerometerRate(25);

  setFins_ALL(0);

  testFins();
}

void loop() {
  getOrientation();
  getTravel();
  calculateFinMovements_STATIONARY();

}
