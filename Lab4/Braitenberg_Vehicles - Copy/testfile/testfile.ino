#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
//#include <NewPing.h> //include sonar library
#include <TimerOne.h>
#include <movingAvg.h> //including moving average library

const int rtStepPin = 50; //right stepper motor step pin
const int rtDirPin = 51;  // right stepper motor direction pin
const int ltStepPin = 52; //left stepper motor step pin
const int ltDirPin = 53;  //left stepper motor direction pin

#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define test_led 13 //test led to test interrupt heartbeat

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

#define baud_rate 9600//set serial communication baud rate
volatile boolean test_state; //variable to hold test led state for timer 
#define timer_int 120000 // 1/2 second (500000 us) period for timer interrupt
//changed timer interrupt new number

int photocellPin1 = 15;     // the cell and 10K pulldown are connected to a0
int photocellPin2 = 14;     // the cell and 10K pulldown are connected to a0
movingAvg photocellReadingL(15);     // the analog reading from the sensor divider
movingAvg photocellReadingR(15);     // the analog reading from the sensor divider

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

void setup() {
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

  Timer1.initialize(timer_int);         // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updatereading);  // attaches updateIR() as a timer overflow interrupt

   Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
 // digitalWrite(redLED,HIGH);
  delay(2500); //seconds before the robot moves
  photocellReadingL.begin();
  photocellReadingR.begin();
}

void loop() {
  //forward(qrtr_rot,robot_spd);
  //spin(-2);
  if (Light == false){
     forward(-qrtr_rot,robot_spd);
    midlight = false;
  }else if(Light == true){
    stop();
    delay(500);
    for (int n = 0; n < 180; n++){
      if(PL == true && midlight == false){
        spin(-2);
        delay(100);
        nturns = nturns-1;
      }else if(PR == true && midlight == false){
        spin(2);
        delay(100);
        nturns = nturns+1;
      }else if(midlight == true){
        break;
      }
    }
    stop();
    delay(500);
//    for(int n = 0; n < 2000; n++){
//      if(irF == false){
//        forward(-qrtr_rot,robot_spd);
//        nsteps = nsteps+1;
//      }else if(irF == true){
//        break;
//      }
//    }
    stop();
    delay(1000);
    reverse(-nsteps*qrtr_rot,robot_spd);
    spin(-nturns*1.5);
    stop();
    midlight = false;
    Light = false;
    PL = false;
    PR = false;
    nsteps = 0;
    nturns = 0;
  }
  //delay(500);

}

void updatereading(){
  rightReading= 0;
  leftReading=0;
  photocellReadingL.reading(analogRead(14));  
  photocellReadingR.reading(analogRead(15));

  rightReading = photocellReadingR.getAvg()-20;
  leftReading = photocellReadingL.getAvg();
  sumReading = rightReading+leftReading;


  if (rightReading > 240){
    PR = true;
    Light = true;
  }
//  else{
//    PR = false;
//  }
  if (leftReading > 280){
    PL = true;
    Light = true;
  }
//  else{
//    PR = false;
//  }
  if (sumReading-prevsumReading < -3){
    midlight = true;
  }
//  else{
//    midlight = false;
//  }
  

  Serial.print(rightReading);
  Serial.print(" \t ");
  Serial.print(leftReading);
  Serial.print(" \t ");
  Serial.print(sumReading);
  Serial.print(" \t ");
  Serial.print(Light);
  Serial.println(" \t ");

  prevsumReading = sumReading;
}

//void updatereading(){
//  photocellReadingL.reading(analogRead(14));  
//  photocellReadingR.reading(analogRead(15));
//
//  rightReading = photocellReadingR.getAvg();
//  leftReading = photocellReadingL.getAvg();
//  sumReading = rightReading+leftReading;
//
//
//
//  Serial.print(photocellReadingL.getAvg());
//  Serial.print(" \t ");
//  Serial.print(photocellReadingR.getAvg());
//  Serial.print(" \t ");
//  Serial.print(sumReading);
//  Serial.println(" \t ");
//
//}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

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


void spin(int angle){
  long stepsToTake = angle*5.1;//calculate steps to reach the angle
  
  if (angle > 0){
    stepperRight.move(stepsToTake);// set steps(right wheel require more distance to achieve right angle)
    stepperRight.setMaxSpeed(robot_spd);
    stepperLeft.move(-stepsToTake); // set steps(left wheel require more distance to achieve right angle)
    stepperLeft.setMaxSpeed(robot_spd);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the target
  }else if (angle < 0){
    stepperRight.move(stepsToTake);// set steps(right wheel require more distance to achieve right angle)
    stepperRight.setMaxSpeed(robot_spd);
    stepperLeft.move(-stepsToTake); // set steps(left wheel require more distance to achieve right angle)
    stepperLeft.setMaxSpeed(robot_spd);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the target
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }
}



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

void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}
