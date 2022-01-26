// Description:
// This program is used to make the robot have 
// Authors: Nathan Lee   Shantao Cao
// Date: 1/16/2022
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <movingAvg.h>
#include <TimerOne.h>


const int rtStepPin = 50; //right stepper motor step pin
const int rtDirPin = 51;  // right stepper motor direction pin
const int ltStepPin = 52; //left stepper motor step pin
const int ltDirPin = 53;  //left stepper motor direction pin

#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states

#define stepperEnable 48    //stepper enable pin on stepStick
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define test_led 13 //test led to test interrupt heartbeat

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define qrtr_rot 100      //stepper motor quarter rotation
#define half_rot 200      //stepper motor half rotation
#define one_rotation  400//stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  800 //stepper motor 2 rotations
#define three_rotation 1200 //stepper rotation 3 rotations
#define max_accel     100000//maximum robot acceleration
#define robot_spd     250 //set robot speed
#define robot_spd_mid     125 //set robot speed
#define robot_spd_min     50 //set robot speed
#define max_spd       2500//maximum robot speed


int spdR = 0;
int spdL = 0;
int baseSpeed = 100;
int speedFilterFactor = 100; // filters anything smaller than 100 because int cut off
int speedGain = 100;

//int max_spd = 2000;
//int max_accel = 1000;

int photocellPin = 15;     // the cell and 10K pulldown are connected to a0
movingAvg photocellReadingL(20);     // the analog reading from the sensor divider
movingAvg photocellReadingR(20);     // the analog reading from the sensor divider

  

  

  //light homing
  int rightReading;
  int leftReading;
  int sumReading;
  int prevsumReading = 0;
  
  int nturns;
  int nsteps;
  
  boolean Light = false;
  boolean straight = false;
  boolean PL = false;
  boolean PR = false;
  boolean midlight = false;
  

void setup(void) {

  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  pinMode(redLED, OUTPUT);//set LED as output
  pinMode(grnLED, OUTPUT);//set LED as output
  pinMode(ylwLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);//set right motor speed
  stepperLeft.setSpeed(robot_spd);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED

  Timer1.initialize(1000);         // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updatereading);  // attaches updateIR() as a timer overflow interrupt
  
  Serial.begin(9600);
  photocellReadingL.begin();
  photocellReadingR.begin();
  
}
 
void loop(void) {
//  photocellReadingL.reading(analogRead(14));  
//  photocellReadingR.reading(analogRead(15));
//
//  int rightReading = photocellReadingR.getAvg();
//  int leftReading = photocellReadingL.getAvg();
//
//  spdR = 0;
//  spdL = 0;
//
//  // ???
//  if (rightReading > 220){
//    spdR = -(photocellReadingR.getAvg()-220)/speedFilterFactor;
//  }
//  if (leftReading > 320){
//    spdL = -(photocellReadingL.getAvg()-320)/speedFilterFactor;
//  }
//  
//  spdR = spdR*speedGain + baseSpeed;
//  spdL = spdL*speedGain + baseSpeed;
//
//  Serial.print(photocellReadingL.getAvg());
//  Serial.print(" \t ");
//  Serial.print(photocellReadingR.getAvg());
//  Serial.print(" \t ");
//  Serial.print(spdL);
//  Serial.print(" \t ");
//  Serial.println(spdR);

  //runAtSpeed();
  forward(qrtr_rot*1,robot_spd*1);
//  if (Light == false){
//     forward(qrtr_rot,robot_spd);
//    midlight = false;
//  }else if(Light == true){
//    stop();
//    delay(500);
//    for (int n = 0; n < 180; n++){
//      if(PL == true && midlight == false){
//        spin(2);
//        delay(100);
//        nturns = nturns+1;
//      }else if(PR == true && midlight == false){
//        spin(-2);
//        delay(100);
//        nturns = nturns-1;
//      }else if(midlight == true){
//        break;
//      }
//    }
//    stop();
//    delay(500);
////    for(int n = 0; n < 2000; n++){
////      if(irF == false){
////        forward(qrtr_rot,robot_spd);
////        nsteps = nsteps+1;
////      }else if(irF == true){
////        break;
////      }
////    }
//    stop();
//    delay(1000);
//    reverse(nsteps*qrtr_rot,robot_spd);
//    spin(nturns*2);
//    stop();
//    midlight = false;
//    Light = false;
//    PL = false;
//    PR = false;
//    nsteps = 0;
//    nturns = 0;
//  }
 
}


//void runAtSpeed () {
//  stepperRight.setSpeed(spdR);
//  stepperLeft.setSpeed(spdL);
//  while(stepperRight.runSpeed() || stepperLeft.runSpeed()){
//  }
//  
//}


void updatereading(){
  photocellReadingL.reading(analogRead(14));  
  photocellReadingR.reading(analogRead(15));

  rightReading = photocellReadingR.getAvg();
  leftReading = photocellReadingL.getAvg();
  sumReading = rightReading+leftReading;

  spdR = 0;
  spdL = 0;

  // ???
  if (rightReading > 140){
    PR = true;
    Light = true;
    //spdR = -(photocellReadingR.getAvg()-220)/speedFilterFactor;
  }
//  else{
//    PR = false;
//  }
  if (leftReading > 180){
    PL = true;
    Light = true;
    //spdL = -(photocellReadingL.getAvg()-320)/speedFilterFactor;
  }
//  else{
//    PR = false;
//  }
  if (sumReading-prevsumReading < -20){
    midlight = true;
  }
//  else{
//    midlight = false;
//  }
  
  
  //spdR = spdR*speedGain + baseSpeed;
  //spdL = spdL*speedGain + baseSpeed;

  Serial.print(rightReading);
  Serial.print(" \t ");
  Serial.print(leftReading);
  Serial.print(" \t ");
  Serial.print(spdL);
  Serial.print(" \t ");
  Serial.print(spdR);
  Serial.print(" \t ");
  Serial.print(sumReading);
  Serial.print(" \t ");
  Serial.print(Light);
  Serial.println(" \t ");

  prevsumReading = sumReading;
}
void runAtSpeed (int Rspd,int Lspd) {
  stepperRight.setSpeed(Rspd);
  stepperLeft.setSpeed(Lspd);
  stepperRight.runSpeed();
  stepperLeft.runSpeed(); 
  
}


// spin function from Lab1
// can spin to the specified angle
void spin(int angle){
  long stepsToTake = angle*5.1;//calculate steps to reach the angle
  
  if (angle > 0){
    stepperRight.move(stepsToTake);// set steps(right wheel require more distance to achieve right angle)
    stepperRight.setMaxSpeed(robot_spd);
    stepperLeft.move(-stepsToTake); // set steps(left wheel require more distance to achieve right angle)
    stepperLeft.setMaxSpeed(robot_spd);
    runAtSpeedToPosition();
    runToStop();//run until the robot reaches the target
  }else if (angle < 0){
    stepperRight.move(stepsToTake);// set steps(right wheel require more distance to achieve right angle)
    stepperRight.setMaxSpeed(robot_spd);
    stepperLeft.move(-stepsToTake); // set steps(left wheel require more distance to achieve right angle)
    stepperLeft.setMaxSpeed(robot_spd);
    runAtSpeedToPosition();
    runToStop();//run until the robot reaches the target
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }
}

//a blocking function to move the stepper to a specified distance with a certain speed
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

//start to move the stepper to a specified distance with a certain speed
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

//move forward with required speed and distance
void forward(int rot, int spd) {
//  long positions[2]; // Array of desired stepper positions
//  stepperRight.setMaxSpeed(spd);//set right motor speed
//  stepperLeft.setMaxSpeed(spd);//set left motor speed
//  positions[0] = stepperRight.currentPosition() + rot*1; //right motor absolute position
//  positions[1] = stepperLeft.currentPosition() + rot*1; //left motor absolute position
//  steppers.moveTo(positions);
  stepperRight.move(rot);//move one full rotation forward relative to current position
  stepperLeft.move(rot);//move one full rotation forward relative to current position
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd);//set left motor speed
  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the target

  //stepperRight.run();
  //stepperLeft.run();
  //steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
  //runToStop();
}
void reverse(int rot, int spd) {
    forward(-rot, spd);
}
