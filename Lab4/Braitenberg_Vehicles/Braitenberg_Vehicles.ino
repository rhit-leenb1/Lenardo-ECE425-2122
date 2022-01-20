// Description:
// This program is used to make the robot have 
// Authors: Nathan Lee   Shantao Cao
// Date: 1/16/2022
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <movingAvg.h>;
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


int spdR = 0;
int spdL = 0;
int baseSpeed = 200;
int speedFilterFactor = 100; // filters anything smaller than 100 because int cut off
int speedGain = 100;

int max_spd = 2000;
int max_accel = 1000;

int photocellPin = 15;     // the cell and 10K pulldown are connected to a0
movingAvg photocellReadingL(10);     // the analog reading from the sensor divider
movingAvg photocellReadingR(10);     // the analog reading from the sensor divider

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
  stepperRight.setSpeed(0);//set right motor speed
  stepperLeft.setSpeed(0);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED

  Timer1.initialize(1000);         // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(runAtSpeed);  // attaches updateIR() as a timer overflow interrupt
  
  Serial.begin(9600);
  photocellReadingL.begin();
  photocellReadingR.begin();
  
}
 
void loop(void) {
  photocellReadingL.reading(analogRead(14));  
  photocellReadingR.reading(analogRead(15));

  int rightReading = photocellReadingR.getAvg();
  int leftReading = photocellReadingL.getAvg();

  spdR = 0;
  spdL = 0;

  // ???
  if (rightReading > 220){
    spdR = -(photocellReadingR.getAvg()-220)/speedFilterFactor;
  }
  if (leftReading > 320){
    spdL = -(photocellReadingL.getAvg()-320)/speedFilterFactor;
  }
  
  spdR = spdR*speedGain + baseSpeed;
  spdL = spdL*speedGain + baseSpeed;

  Serial.print(photocellReadingL.getAvg());
  Serial.print(" \t ");
  Serial.print(photocellReadingR.getAvg());
  Serial.print(" \t ");
  Serial.print(spdL);
  Serial.print(" \t ");
  Serial.println(spdR);

 
}


void runAtSpeed () {
  stepperRight.setSpeed(spdR);
  stepperLeft.setSpeed(spdL);
  while(stepperRight.runSpeed() || stepperLeft.runSpeed()){
  }
  
}
