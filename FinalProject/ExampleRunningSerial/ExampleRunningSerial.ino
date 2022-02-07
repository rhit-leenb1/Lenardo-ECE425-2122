// Description:
// This program is used to make the robot have 
// Authors: Nathan Lee   Shantao Cao
// Date: 1/16/2022
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <movingAvg.h>
#include <TimerOne.h>
#include <basicMPU6050.h> 

// Create instance
basicMPU6050<> imu;

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

String readLine;
String stringCom;
uint16_t dist;

int spdR = 00;
int spdL = 00;
int baseSpeed = 200;
int speedFilterFactor = 100; // filters anything smaller than 100 because int cut off
int speedGain = 100;

int max_spd = 2000;
int max_accel = 1000;


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
  stepperRight.setSpeed(spdR);//set right motor speed
  stepperLeft.setSpeed(spdL);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  
  //Timer1.initialize(1000);         // initialize timer1, and set a timer_int second period
  //Timer1.attachInterrupt(updatereading);  // attaches updateIR() as a timer overflow interrupt
  
  Serial.begin(115200);

  
}
 
void loop() {
  updatereading();
  
  if (Serial.available()){  
        readLine = Serial.readString();
        Serial.println(readLine);
        stringCom = readLine.substring(0,1);
        dist = readLine.substring(1,readLine.length()).toInt();
        if (stringCom == "F"){
          forward(dist);
        }
        if (stringCom == "S"){
          forward(dist);
        }
        Serial.print(stringCom);
        Serial.print("\t");
        Serial.println(dist);
        Serial.println("");
        //delay(1000);
  }
  
}


void forward(int distance) {
  // inches
 
  long stepsToTake = 800*distance/(PI*3.375); //calculate how many steps to go to distance
  stepperRight.move(stepsToTake);//move one full rotation forward relative to current position
  stepperLeft.move(stepsToTake);//move one full rotation forward relative to current position
  stepperRight.setMaxSpeed(500);//set right motor speed
  stepperLeft.setMaxSpeed(500);//set left motor speed
  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the target
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

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

void updatereading(){
  // Update gyro calibration 
  imu.updateBias();
 
  //-- Scaled and calibrated output:
  // Accel
  Serial.print( imu.ax() );
  Serial.print( " " );
  Serial.print( imu.ay() );
  Serial.print( " " );
  Serial.print( imu.az() );
  Serial.print( "    " );
  
  // Gyro
  Serial.print( imu.gx() );
  Serial.print( " " );
  Serial.print( imu.gy() );
  Serial.print( " " );
  Serial.print( imu.gz() );
  Serial.println( "    " );  

}
