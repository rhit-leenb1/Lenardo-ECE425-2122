/*RobotTimer1SonarTimer2Interrupt.ino
  Author: Carlotta. A. Berry
  Date: December 13, 2018
  This program will test using a timer1 interrupt to update the IR datat and
  using the NewPing library with a timer2 interrupt to update the sonar data
  in order  to create an obstacle avoidance beavhior on the robot.

  Hardware Connections:
  Stepper Enable Pin 48
  Right Stepper Step Pin 46
  Right Stepper Direction Pin 53
  Left Stepper Step Pin 44
  Left Stepper Direction Pin 49

  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 46 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 44 - left stepper motor step pin
  digital pin 49 - left stepper motor direction pin

  Hardware Connections:
  Front IR    A8
  Back IR     A9
  Right IR    A10
  Left IR     A11
  Left Sonar  A1
  Right Sonar A2
  Pushbutton  A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>
#include "movingAvg.h" //including moving average library in same folder

//define stepper motor pin numbers connections
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

//define IR sensor connections
#define irFront A10    //front IR analog pin
#define irRear A9    //back IR analog pin
#define irRight A12   //right IR analog pin
#define irLeft A11   //left IR analog pin
#define button A15    //pushbutton 

///////////// NEW SONAR CLASSES FOR TIMER 2 INTERRUPT/////////////////
//define sonar sensor connections
#define snrLeft   A1   //front left sonar 
#define snrRight  A2  //front right sonar 
#define SONAR_NUM     2         // Number of sensors.
#define MAX_DISTANCE 200        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 125        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define FIRST_PING_START 50     // First ping starts at this time in ms, gives time for the Arduino to chill before starting.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(snrLeft, snrLeft, MAX_DISTANCE),//create an instance of the left sonar
  NewPing(snrRight, snrRight, MAX_DISTANCE),//create an instance of the right sonar
};
////////////////////////////////////////////////////////////////////

//define stepper motor constants
#define stepperEnable 48    //stepper enable pin on stepStick
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define qrtr_rot 100      //stepper motor quarter rotation
#define half_rot 200      //stepper motor half rotation
#define one_rotation  400//stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  800 //stepper motor 2 rotations
#define three_rotation 1200 //stepper rotation 3 rotations
#define max_accel     10000//maximum robot acceleration
#define robot_spd     0 //set robot speed
#define max_spd       250//maximum robot speed


#define irThresh    400 // The IR threshold for presence of an obstacle in ADC value
#define snrThresh   20  // The sonar threshold for presence of an obstacle in cm
#define minThresh   0   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move

#define minSensorDist 1 //Threshold for stopping
#define sensorDist 8    //Threshold for repulsive forces (shykid using ir on left,right,rear) [in]

const int irListSize = 10;
movingAvg irFrontList(irListSize);  //variable to holds list of last front IR reading
movingAvg irLeftList(irListSize);   //variable to holds list of last left IR reading
movingAvg irRearList(irListSize);   //variable to holds list of last rear IR reading
movingAvg irRightList(irListSize);   //variable to holds list of last right IR reading

int rightDirection = 1; //1 for going forward, -1 for going backwards
int leftDirection = -1;
boolean changeDirection = false;

int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int srLeftAvg;   //variable to hold average of left sonar current reading
int srRightAvg;  //variable to hold average or right sonar current reading

#define baud_rate     9600  //set serial communication baud rate
#define TIME          250   //pause time
#define timer_int     125000 //timer interrupt interval in microseconds (range 1 us to 8.3 s)


//sonar Interrupt variables
volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
volatile uint8_t test_state = 0;

//set wheel speeds
int leftSpeed = robot_spd;
int rightSpeed = robot_spd;

void setup() {
  //multipler sonar on timer 2 setup
  pingTimer[0] = millis() + FIRST_PING_START;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++)               // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  //stepper Motor set up
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

  
  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateLeonardo);  // attaches updateLeonardo() as a timer overflow interrupt

  //Moving Average Set Up
  irFrontList.begin();
  irLeftList.begin();
  irRightList.begin();
  irRearList.begin();
  
  //Set up serial communication
  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
 // digitalWrite(redLED,HIGH);
  delay(2500); //seconds before the robot moves
}

void loop() {
  updateSonar();
  updateCollisionDetection();
  digitalWrite(ylwLED,HIGH);
  setStepperSpeeds();
  //Serial.print(rightSpeed); Serial.print("\t");Serial.println(leftSpeed);
  //obsRoutine();
  //forward(qrtr_rot);
}

void setStepperSpeeds(){
  if (changeDirection){
    rightSpeed = rightSpeed * rightDirection;
    leftSpeed = leftSpeed * leftDirection;
    changeDirection = false;
  }
  stepperRight.setSpeed(rightSpeed);//set right motor speed
  stepperLeft.setSpeed(leftSpeed);//set left motor speed
  Serial.print(rightSpeed); Serial.print("\t");
  Serial.println(leftSpeed);
  stepperRight.runSpeed();
  stepperLeft.runSpeed();
}


/*This is a sample updateSensors() function, it is called from the timer 1 interrupt an the updateSonar uses the timer2

*/
void updateLeonardo() {
  updateSensors();
  updatePathingForces();
}

void updateCollisionDetection(){
  
}

void updatePathingForces(){
  checkLeftRight();
  if(irRearAvg <= sensorDist && irRearAvg > minSensorDist){
    rightSpeed = rightSpeed + (max_spd - rightSpeed)/sensorDist*(sensorDist-irRearAvg);
    leftSpeed = leftSpeed + (max_spd - leftSpeed)/sensorDist*(sensorDist-irRearAvg);
    rightDirection = 1;
    leftDirection = 1;
    changeDirection = true;
  }
  if (((srRightAvg < snrThresh && srRightAvg > minThresh) &&
       (srLeftAvg < snrThresh && srLeftAvg > minThresh)) ) {
    rightSpeed = robot_spd + (max_spd - robot_spd)/(snrThresh)*(snrThresh-srRightAvg);
    leftSpeed = robot_spd + (max_spd - robot_spd)/(snrThresh)*(snrThresh-srLeftAvg);

    changeDirection = true;
    rightDirection = -1;
    leftDirection = -1;
    
  }

  if(((srRightAvg < snrThresh && srRightAvg > minThresh) &&
       (srLeftAvg < snrThresh && srLeftAvg > minThresh)) &&
       (irRearAvg <= sensorDist && irRearAvg > minSensorDist) ){
    checkLeftRight();
  }

  if((irLeftAvg <= sensorDist && irLeftAvg > minSensorDist)&&(irRightAvg <= sensorDist && irRightAvg > minSensorDist)&&
     (irRearAvg <= sensorDist && irRearAvg > minSensorDist) && ((srRightAvg < snrThresh && srRightAvg > minThresh) &&
     (srLeftAvg < snrThresh && srLeftAvg > minThresh))){
    leftSpeed = 0;
    rightSpeed = 0;
  }

}

void checkLeftRight(){
  if(irLeftAvg <= sensorDist && irLeftAvg > minSensorDist){
    leftSpeed = robot_spd + (max_spd - robot_spd)/sensorDist*(sensorDist-irLeftAvg);
    rightSpeed = robot_spd + (max_spd - robot_spd)/sensorDist*(sensorDist-irLeftAvg);
    rightDirection = -1;
    leftDirection = 1;
    changeDirection = true;
  } else {
    leftSpeed = robot_spd;
  }
  if(irRightAvg <= sensorDist && irRightAvg > minSensorDist){
    rightSpeed = robot_spd + (max_spd - robot_spd)/sensorDist*(sensorDist-irRightAvg);
    leftSpeed = robot_spd + (max_spd - robot_spd)/sensorDist*(sensorDist-irLeftAvg);
    rightDirection = 1;
    changeDirection = true;
  } else {
    rightSpeed = robot_spd;
  }

}

/*This is a sample updateSensors() function, it is called from the timer 1 interrupt an the updateSonar uses the timer2

*/
void updateSensors() {
  updateIR();
  //updateSonar();
}

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/
void updateIR() {
  //get voltage reading from pin and compute moving average
  irFrontAvg = irFrontList.reading(analogRead(irFront));
  irRearAvg = irRearList.reading(analogRead(irRear));
  irLeftAvg = irLeftList.reading(analogRead(irLeft));
  irRightAvg = irRightList.reading(analogRead(irRight));

  // convert moving average voltage to distance (in)
  irFrontAvg = 2000/(irFrontAvg+50)-1.5;
  irRearAvg = 2000/(irRearAvg+50)-1.5;
  irLeftAvg = 2500/(irLeftAvg-32)-1.8;
  irRightAvg = 2500/(irRightAvg-32)-1.8;
  
  //  print IR data
//  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
//  Serial.print(irFrontAvg); Serial.print("\t");
//  Serial.print(irRearAvg); Serial.print("\t");
//  Serial.print(irLeftAvg); Serial.print("\t");
//  Serial.println(irRightAvg);
}


/*
  This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar() {
  test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    //    Serial.print("\t\t\t");
    //    Serial.print(millis());
    //    Serial.print("\t");
    //    Serial.print(pingTimer[i]);
    //    Serial.print("\t");
    //    Serial.println(PING_INTERVAL);
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        //oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        if (cm[0] > 0)
          srLeftAvg = cm[0];
        if (cm[1] > 0)
          srRightAvg = cm[1];
//       Serial.print("lt snr:\t");
//       Serial.print(srLeftAvg);
//        Serial.print(" cm ");
//        Serial.print("\trt snr:\t");
//        Serial.print(srRightAvg);
//        Serial.println(" cm");
      }
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

//This function writes to the sonar data if the ping is received
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//This function prints the sonar data once all sonars have been read
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(i);
    //Serial.print(" = ");
    //Serial.print(cm[i]);
    //Serial.print(" cm\t");
  }
  srLeftAvg = cm[0];
  srRightAvg = cm[1];
//    Serial.print("Left Sonar = ");
//    Serial.print(srLeftAvg);
//    Serial.print("\t\tRight Sonar = ");
//    Serial.print(srRightAvg);
//    Serial.println();
}
