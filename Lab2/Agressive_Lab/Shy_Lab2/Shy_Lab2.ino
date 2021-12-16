#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>
#include "movingAvg.h" //including moving average library

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

#define snrLeft   A1   //front left sonar 
#define snrRight  A2  //front right sonar 

#define SONAR_NUM     2         // Number of sensors.
#define MAX_DISTANCE 200        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 125        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define FIRST_PING_START 50     // First ping starts at this time in ms, gives time for the Arduino to chill before starting.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

int cmFL = 0;
int cmFR = 0;

NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

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

int spdL = 0;
int spdR = 0;
int dir = 1;

#define irThresh    6 // The IR threshold for presence of an obstacle in ADC value
#define snrThresh   18  // The sonar threshold for presence of an obstacle in cm
#define snrThreshmid   15  // The sonar midian threshold for presence of an obstacle in cm
#define snrThreshmin   10  // The sonar minimum threshold for presence of an obstacle in cm
#define minThresh   1   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

// IR
#define irFront A10    //front IR analog pin
#define irRear A9    //back IR analog pin
#define irRight A12   //right IR analog pin
#define irLeft A11   //left IR analog pin

const int irListSize = 10;
movingAvg irFrontList(irListSize);  //variable to holds list of last front IR reading
movingAvg irLeftList(irListSize);   //variable to holds list of last left IR reading
movingAvg irRearList(irListSize);   //variable to holds list of last rear IR reading
movingAvg irRightList(irListSize);   //variable to holds list of last right IR reading
//

int inirF = 0;
int inirB = 0;
int inirL = 0;
int inirR = 0;
int inirFAvg = 0;
int inirBAvg = 0;
int inirLAvg = 0;
int inirRAvg = 0;

//sonar Interrupt variables
volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
//volatile uint8_t test_state = 0;
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data
volatile boolean test_state; //variable to hold test led state for timer interrupt

#define goright   1  // Moving Right Motor in progress flag
#define goleft   2  // Moving Left Motor in progress flag
#define fwd       3
#define rev       4
#define collide   5
#define runAway   6
#define wander    7
#define botStop  0

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt
boolean obstacle = false;
boolean SonarL = false;
boolean SonarR = false;
boolean IrF = false;
boolean IrB = false;
boolean IrL = false;
boolean IrR = false;

int state = 0;
int randomstate = 0;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  pingTimer[0] = millis() + FIRST_PING_START;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++)               // Set the starting time for each sensor.
  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  
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
  Timer1.attachInterrupt(updateSensors);  // attaches updateIR() as a timer overflow interrupt
  //Moving Average Set Up
  irFrontList.begin();
  irLeftList.begin();
  irRightList.begin();
  irRearList.begin();

  //Timer3.initialize(timer_int);         // initialize timer1, and set a timer_int second period
  //Timer3.attachInterrupt(updateSensors);  // attaches updateIR() as a timer overflow interrupt

  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
 // digitalWrite(redLED,HIGH);
  delay(2500); //seconds before the robot moves
}

void loop() {
  //delay(1);
  //Shyfunction();
  if (obstacle == true){
    runatspeed();
    digitalWrite(grnLED, LOW);//turn off green LED
    digitalWrite(ylwLED, HIGH);//turn on yellow LED
  }else if(obstacle == false){
    //stop;
    randomwonder();
    digitalWrite(grnLED, HIGH);//turn on green LED
    digitalWrite(ylwLED, LOW);//turn on yellow LED
  }

}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

void runatspeed(){
  stepperRight.setSpeed(spdR);
  stepperLeft.setSpeed(spdL);
  stepperRight.runSpeed();
  stepperLeft.runSpeed();
}


void forward(int rot, int spd) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd);//set left motor speed
  //stepperRight.setSpeed(robot_spd);//set right motor speed
  //stepperLeft.setSpeed(robot_spd);//set left motor speed

  //stepperRight.setCurrentPosition(0);
  //stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot*1; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot*1; //left motor absolute position
  steppers.moveTo(positions);

  //stepperRight.run();
  //stepperLeft.run();
  //steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
  runToStop();
}
void turnleft(int rot, int spd) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd);//set left motor speed
  //stepperRight.setSpeed(robot_spd);//set right motor speed
  //stepperLeft.setSpeed(robot_spd);//set left motor speed

  //stepperRight.setCurrentPosition(0);
  //stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot*1; //right motor absolute position
  positions[1] = stepperLeft.currentPosition(); //left motor absolute position
  steppers.moveTo(positions);

  //stepperRight.run();
  //stepperLeft.run();
  //steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
  runToStop();
}

void turnright(int rot, int spd) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd);//set left motor speed
  //stepperRight.setSpeed(robot_spd);//set right motor speed
  //stepperLeft.setSpeed(robot_spd);//set left motor speed

  //stepperRight.setCurrentPosition(0);
  //stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition(); //right motor absolute position
  positions[1] = stepperLeft.currentPosition()+ rot*1; //left motor absolute position
  steppers.moveTo(positions);

  //stepperRight.run();
  //stepperLeft.run();
  //steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
  runToStop();
}

void reverse(int rot, int spd) {
    forward(-rot, spd);
}

void gotoangle(int angle){
  //Serial.println(angle);
  
  long stepsToTake = angle*9.4;//calculate steps to reach the angle
  
  if (angle > 0){
    stepperRight.move(stepsToTake*1.05);// set steps(right wheel require more distance to achieve right angle)
    stepperRight.setMaxSpeed(defaultRightWheelSpeed );
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the target
  }else if (angle < 0){
    stepperLeft.move(-stepsToTake*1.25); // set steps(left wheel require more distance to achieve right angle)
    stepperLeft.setMaxSpeed(defaultRightWheelSpeed);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the target
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }
}

void updateState() {
  if (IrR == true){
    Serial.println(" IrR True");
  }

  
  if (obstacle == false) { //no sensors triggered
    state = botStop;
    Serial.println("obstacle");
  }
  else if (obstacle == true) { //front sensors triggered
    if ((SonarL == true && SonarR == false)||(IrL == false && IrR == true && IrB == false )){
      state = goleft;//(SonarR == true && SonarL == false)||
      Serial.println("True");
    }else if((SonarL == true && SonarR == false)||(IrL == true && IrR == false && IrB == false)){
      state = goright;//(SonarL == true && SonarR == false)||
    }else if((IrL == false && IrR == false && SonarL == true && SonarR == true && IrB == false)
      ||(IrL == true && IrR == true && SonarL == true && SonarR == true && IrB == false)
      ||(IrL == true && IrR == true && SonarL == true && SonarR == true && IrB == false)
      ||){
      state = rev;//(SonarL == true && SonarR == true)||
    }else if((IrL == false && IrR == false && SonarL == false && SonarR == false && IrB == true)||(IrL == true && IrR == true && SonarL == false && SonarR == false && IrB == false)){
      state = fwd;
    }else if (IrL == true && IrR == true && SonarL == true && SonarR == true && IrB == true){
      state = botStop;
    }

    
  }//else{
   // state = botStop;
  //}
  
  //print flag byte
     // Serial.println("\trtSNR\tltSNR\tltIR\trtIR\trearIR\tftIR");
     // Serial.print("flag byte: ");
     // Serial.println(flag, BIN);
  //print state byte
     // Serial.println("\twander\trunAway\tcollide\treverse\tforward");
     // Serial.print("state byte: ");
     //Serial.println(state, BIN);
  Serial.println(state);
}

void updateSpeed(){
  spdL = 0;
  spdR = 0;

  if (SonarL == true || SonarR == true){
        dir = -1;
  }else{
        dir = 1;
  }
  if (obstacle == false) { //no sensors triggered
    spdL=0;
    spdR=0;
    Serial.println("obstacle");
  }else if (obstacle == true){
    if ((IrL == true && IrR == true && SonarL == true && SonarR == true && IrB == true)){
      spdL=0;
      spdR=0;
    }else if(SonarR == true && SonarL == true && IrB == true){
      spdL=-100;
      spdR=100;
    }else{
      
      if (IrR == true){
        spdR = spdR+(dir)*(6.5-inirR)*200;
      }
      if (IrL == true){
        spdL = spdL+dir*(6.5-inirR)*200;
      }
      if (IrB == true){
        spdL = spdL+dir*(6.1-inirB)*150;
        spdR = spdR+dir*(6.1-inirB)*150;
      }
      if (SonarL == true && IrB == false){
        spdL = spdL+dir*(19-cmFL)*50;
      }
      if (SonarR == true && IrB == false){
        spdR = spdR+dir*(19-cmFR)*50;
      }
      
      
    }
    
  }
  
}

//void Shyfunction(){
//  if (state == botStop){
//    Serial.println("robot stop1");
//    stop();
//  }else if (obstacle == true && state == rev){
//    
//    reverse(one_rotation, robot_spd);
//    delay(50);
//    Serial.println("robot reverse");
//    
//  }else if(obstacle == true && state == goleft){
//    turnleft(one_rotation, robot_spd);//*(6.5-inirL)/6);
//    delay(50);
//    Serial.println("robot left");
//  }else if(obstacle == true && state == goright){
//    turnright(one_rotation, robot_spd);//*(6.5-inirR)/6);
//    delay(50);
//    Serial.println("robot right");
//  }else if(obstacle == true && state == fwd){
//    forward(one_rotation, robot_spd);//*(6.5-inirB)/6);
//    delay(50);
//    Serial.println("robot reverse");
//      
//    }
//  else{
//    Serial.println("robot stop2");
//    stop();
//  }
//  
//}

void randomwonder(){
  if (randomstate == 0){
    forward(one_rotation, 250);
    randomstate = random(1,3);
  }else if(randomstate == 1){
    turnright(one_rotation, 250);
    randomstate = 0;
  }else if(randomstate == 2 ){
    turnleft( one_rotation, 250);
    randomstate = 0;
  }
}


void updateSensors() {
  test_state = !test_state;
  digitalWrite(test_led, test_state);
  state = 0;
  obstacle = false;
  updateIR();
  updateSonar2();
  
  //updateState();
  updateSpeed();
  //Serial.println(state);
}

void updateSonar2() {
  SonarL = false;
  SonarR = false;
  long left, right;
  //read right sonar
  pinMode(snrRight, OUTPUT);//set the PING pin as an output
  digitalWrite(snrRight, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrRight, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrRight, LOW);//set pin low first again
  pinMode(snrRight, INPUT);//set pin as input with duration as reception
  right = pulseIn(snrRight, HIGH)/57;//measures how long the pin is high
  cmFR = right;
  //read left sonar
  pinMode(snrLeft, OUTPUT);//set the PING pin as an output
  digitalWrite(snrLeft, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrLeft, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrLeft, LOW);//set pin low first again
  pinMode(snrLeft, INPUT);//set pin as input with duration as reception
  left = pulseIn(snrLeft, HIGH)/57;//measures how long the pin is high
  cmFL = left;
  //  print sonar data
  //    Serial.println("leftSNR\trightSNR");
  //    Serial.print(left); Serial.print("\t");
   //   Serial.println(right);
  if (right < snrThresh){
    //bitSet(flag, obFRight);//set the front right obstacle
    //Serial.println(right);
    SonarR = true;
    obstacle = true;
    }
  else{
    //bitClear(flag, obFRight);//clear the front right obstacle
    SonarR = false;
  }
  if (left < snrThresh){
    //bitSet(flag, obFLeft);//set the front left obstacle
    SonarL = true;
    obstacle = true;
  }
  else{
    //bitClear(flag, obFLeft);//clear the front left obstacle
    SonarL = false;
  }
}

void updateIR() {
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  IrF = false;
  IrB = false;
  IrL = false;
  IrR = false;
  
  inirFAvg = irFrontList.reading(analogRead(irFront));
  inirBAvg = irRearList.reading(analogRead(irRear));
  inirLAvg = irLeftList.reading(analogRead(irLeft));
  inirRAvg = irRightList.reading(analogRead(irRight));

  

  inirF = 2000/(inirFAvg+50)-1.5;
  inirB = 2000/(inirBAvg+50)-1.5;
  inirL = 2500/(inirLAvg-32)-1.8;
  inirR = 2500/(inirRAvg-32)-1.8;

  if (inirR<=0){
    inirR = 1000;
  }
  if (inirL<=0){
    inirL = 1000;
  }
  if (inirB<=0){
    inirB = 1000;
  }
  
  //  print IR data
      Serial.println("frontIR\tbackIR\tleftIR\trightIR");
      Serial.print(inirF); Serial.print("\t");
      Serial.print(inirB); Serial.print("\t");
      Serial.print(inirL); Serial.print("\t");
      Serial.println(inirR);

  //if (inirF <= irThresh){
  //  obstacle = true;
  //  IrF = true;
    //Serial.println("True");
 // }else{
    //IrF = false;
 //}
  if (inirB <= irThresh){
    obstacle = true;
    IrB = true;
  }else{
    IrB = false;
  }
  if (inirL <= irThresh){
    obstacle = true;
    IrL = true;
  }else{
    IrL = false;
  }
  if (inirR <= irThresh){
    obstacle = true;
    IrR = true;
    Serial.println("True");
  }else{
    IrR = false;
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
