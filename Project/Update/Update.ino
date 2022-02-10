//Wall following function with light homing subfunction
// Description:
// This program is used to make the robot have Follow-Wall behavior for Lab3. The PD control is used to control the robot speed and direction.
// Basic logic of this program is to use one state machine to change the behavior of the robot according to the sensor readings
// The robot will be able to maintain a distance between 4 and 6 inches from the wall (actual range is 5 to 7 inches to provide bigger turning space)
// The program is a modular program with different levels. Modifications is required for upper level functions.
// Light homing function is added into the upper level function
// The light function will start working when photosensor detects the light source
// The robot will point to the light source and go forward
// When the IR sensor detects obstacles, the robot will stop and return to homing starting point
// Normally the robot will be in wall following mode.
// Authors: Nathan Lee   Shantao Cao
// Date: 1/16/2022


#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>
#include <movingAvg.h> //including moving average library
#include <basicMPU6050.h> // including gyroscope

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

int cmFL = 0; //Left sonar reading in cm
int cmFR = 0; //Right sonar reading in cm

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

#define snrMin   13               // sonar minimum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMax   15               // sonar maximum threshold for wall (use a deadband of 4 to 6 inches)


#define irThresh    8 // The IR threshold for presence of an obstacle in ADC value
#define irMax    7      // IR max threshold
#define irMin    5      // IR min threshold
#define snrThresh   60  // The sonar threshold for presence of an obstacle in cm
#define snrThreshmid   15  // The sonar midian threshold for presence of an obstacle in cm
#define snrThreshmin   10  // The sonar minimum threshold for presence of an obstacle in cm
#define minThresh   1   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 115200//set serial communication baud rate

// IR
#define irFront A10    //front IR analog pin
#define irRear A9    //back IR analog pin
#define irRight A12   //right IR analog pin
#define irLeft A11   //left IR analog pin

const int irListSize = 3;           // averaging number
movingAvg irFrontList(irListSize);  //variable to holds list of last front IR reading
movingAvg irLeftList(irListSize);   //variable to holds list of last left IR reading
movingAvg irRearList(irListSize);   //variable to holds list of last rear IR reading
movingAvg irRightList(irListSize);   //variable to holds list of last right IR reading

//store front, back, left right, ir sensor value and average value 
float inirF = 0;
float inirB = 0;
float inirL = 0;
float inirR = 0;
float inirFAvg = 0;
float inirBAvg = 0;
float inirLAvg = 0;
float inirRAvg = 0;

volatile unsigned long last_detection = 0; //the last detection
volatile unsigned long last_stop = 0; // the last stop
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
//volatile uint8_t test_state = 0;
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data

volatile boolean test_state; //variable to hold test led state for timer interrupt

#define fright   1  // follow right wall
#define fleft   2  // follow left wall
#define fcenter       3 //follow center
#define endhall      4 //endhall
#define Insidecorner   5 //avoid inside corner
#define runAway   6 //avoid collision
#define outsidecorner  7 //outside corner
#define wander    0 //random wonder

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

boolean obstacle = false; //obstacle detected
boolean SonarL = false;   //left sonar detected
boolean SonarR = false;   //right sonar detected
boolean IrF = false;      //front ir detected
boolean IrB = false;      //back ir detected
boolean IrL = false;      //left ir detected
boolean IrR = false;      //right ir detected

boolean obL = false;      //obstacle on the left
boolean obR = false;      //obstacle on the right

int state = 0; //define state
int prevState = 0; //previous state

int randomstate = 0; //define random state

boolean turnedleft = false; //used to detect left turn
boolean tuenedright = false; // used to detect right turn

int Vlefts = 0;   //left wheel veloctiy (For sonar sensor and hallway state)
int Vrights = 0;  //right wheel velocity


float ls_curr;    //left sonar current reading
float li_curr;    //left ir current reading
float rs_curr;    //right sonar current reading
float ri_curr;    //right ir current reading

float ls_cerror;    //left sonar current error
float li_cerror;    //left ir current error
float rs_cerror;    //right sonar current error
float ri_cerror;    //right ir current error

float ls_perror;    //left sonar previous error
float li_perror;    //left ir previous error
float rs_perror;    //right sonar previous error
float ri_perror;    //right ir previous error

float ls_derror;  //left sonar delta error
float li_derror;  //left ir delta error
float rs_derror;  //right sonar delta error
float ri_derror;  //right ir current error
float left_derror;   //difference between left front and back sensor, this may be useful for adjusting the turn angle
float right_derror;  //difference between right front and back sensor, this may be useful for adjusting the turn angle

float derror;       //difference between left and right error to center robot in the hallway
float dserror;       //difference between left and right error to center robot in the hallway

int spdL = 0; //left wheel speed (wall following mode)
int spdR = 0; //right wheel speed (wall following mode)

int kp; //P value
int kd; //D value

int turns = 0; //how many turns turned in end of wall condition(we find out we just need one turn in the end)
int cturns = 0; //

boolean leftwall = false; //wall follow left
boolean rightwall = false; // wall follow right

//light homing 

#define phLeft   14   //front left sonar 
#define phRight  15  //front right sonar 

  int rightReading;   //right photosensor reading
  int leftReading;    //left photosensor reading
  int sumReading;     //sum of photosensor readings
  int prevsumReading = 0;   // record last sum reading

  int nturns;         // store number of turns
  int nsteps;         // store number of steps
  
  boolean Light = false;    //Light interrupt
//  boolean straight = false;  
  boolean PL = false;       // Photosensor left detected
  boolean PR = false;       // Photosensor right detected
  boolean midlight = false;   // point to the light detected

int photocellPin1 = 15;     // the cell and 10K pulldown are connected to a0
int photocellPin2 = 14;     // the cell and 10K pulldown are connected to a0
movingAvg photocellReadingL(10);     // the analog reading from the sensor divider
movingAvg photocellReadingR(10);     // the analog reading from the sensor divider

// follow the Topological Path
String Input = "";
int InputLength;
String Char;
int Output;

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
  photocellReadingL.begin();
  photocellReadingR.begin();

  //Timer3.initialize(timer_int);         // initialize timer1, and set a timer_int second period
  //Timer3.attachInterrupt(updateSensors);  // attaches updateIR() as a timer overflow interrupt

  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
  
  Serial.println(Input);
  InputLength = Input.length();
  Input.remove(0,1);
  Serial.println(Input);
 // digitalWrite(redLED,HIGH);
 //stepperRight.setAcceleration(100);
 //stepperLeft.setAcceleration(100);
 delay(2000);
}

// main loop decides the robot movement

void loop() {

  
  //if(Light == false){
//    if (state == wander){
//      //wander state use wander function
//      randomwonder();
//      //LED indecates for different states
//      digitalWrite(grnLED, HIGH);
//      digitalWrite(ylwLED, LOW);
//      digitalWrite(redLED, LOW);
//    }
//    if (state == fright || state == fleft){
//      //wall following state for both left and right wall conditon
//      //PD control is used to change the speed and correct the robot moving direction by using calculated speed data from ir sensor reading.
//      runAtSpeed(spdR,spdL);
//    } else if (state == fcenter){
//      //hallway following state
//      //PD control is used to change the speed and correct the robot moving direction by using calculated speed data from sonar sensor reading.
//      runAtSpeed(Vrights,Vlefts);
//      digitalWrite(grnLED, HIGH);
//      digitalWrite(ylwLED, HIGH);
//      digitalWrite(redLED, HIGH);
//    } else {
//      //stop the robot for bad state reading
//      stepperRight.stop();
//      stepperLeft.stop();
//    }
//  
//    
//    if (state == Insidecorner){
//      // inside conner state
//      // cover left and right wall conditions
//      // turn 90 degrees to avoid collision
//      if (prevState == fright){
//        //find the wall location
//        //spin(-45);
//        //forward(300,200);
//        spin(90);
//        forward(-300,200);
//      } else {
//        //spin(45);
//        //forward(200,200);
//        spin(-90);
//        forward(-200,200);
//      }
//      //should have the wall in the same side
//      //change back to original state
//      state = prevState;
//        
//    } else if (state == endhall){
//      //end of hall state should turn robot 180 degrees
//      digitalWrite(grnLED, HIGH);
//      digitalWrite(ylwLED, LOW);
//      digitalWrite(redLED, HIGH);
//      // turn robot 180 degrees
//      forward(800,200); // reverse to create more room
//      spin(180);
//      forward(-100,200);
//      state = prevState;
//    } else if (state == outsidecorner){
//      // outside corner state
//      digitalWrite(grnLED, LOW);
//      digitalWrite(ylwLED, LOW);
//      digitalWrite(redLED, LOW);
//      // first turn when the ir sensor lost the wall
//      if (prevState == fright){
//        spin(-95);
//      } else {
//        spin(95);
//      }
//      forward(-800,200);
//      //updateIR();
//      //check ir after the first turn. In still no wall, turn another 90 degrees and go forward to find the opposite side of the wall
//      if (IrL == false && prevState == fleft){
//        spin(95);
//        forward(-2400,200);
//      } else if (IrR == false && prevState == fright){
//        spin(-95);
//        forward(-2400,200);
//      }
//      
//      state = prevState; //return to last wall follow state if robot find the wall
//      // otherwise turn to random wondering
//    } else if(state == runAway){
//      // runAway state is used to avoid front obstacles in random wander state and follow the wall after the turn if possible
//      digitalWrite(grnLED, HIGH);
//      digitalWrite(ylwLED, LOW);
//      digitalWrite(redLED, HIGH);
//      stepperRight.stop();
//      stepperLeft.stop();
//      //provide space to turn
//      forward(-800,200);
//      spin(90);
//    }

    
//  }else if(Light == true){  //Light homing level
//    stop();                     //stop and turn on LEDs
//    digitalWrite(grnLED, LOW);
//    digitalWrite(ylwLED, LOW);
//    digitalWrite(redLED, LOW);
//    delay(500);
//    digitalWrite(ylwLED, HIGH);
//    //find turnning direction and turn to the light source
//    // record turns for return movement
//    if (PL == true){
//      for (int n = 0; n < 180; n++){
//        spin(-2);     //spin 2 degrees everytime
//        delay(100);
//        nturns = nturns-1;    // recored turns
//        if(midlight == true){   // break when point to the light
//          break;
//        }
//      }
//    }else if(PR == true){   //same as left 
//        for (int n = 0; n < 180; n++){
//        spin(2);
//        delay(100);
//        nturns = nturns-1;
//        if(midlight == true){
//          break;
//        }
//    }
//    }
////    for (int n = 0; n < 180; n++){
////      if(PL == true && midlight == false){
////        spin(-2);
////        delay(100);
////        nturns = nturns-1;
////      }else if(PR == true && midlight == false){
////        spin(2);
////        delay(100);
////        nturns = nturns+1;
////      }else if(midlight == true){
////        break;
////      }
////    }
//    stop();
//    delay(500);
//
//    // go forward and stop when detect obstacle
//    for(int n = 0; n < 2000; n++){
//      if(inirF > 4.5){
//        forward(-qrtr_rot,robot_spd);   // forward 1/4 revolution every time
//        nsteps = nsteps+1;              // record steps
//      }else if(inirF <= 4.5){
//        break;              // break when obstacle detected
//      }
//    }
//    //trun LEDs on 
//    digitalWrite(ylwLED, LOW);
//    digitalWrite(redLED, HIGH);
//    stop();
//    // point back to the starting point
//    delay(1000);
//    spin(180);
//    delay(100);
//    // go forward for n steps
//    reverse(nsteps*qrtr_rot,robot_spd);
//    // spin for n*2degrees
//    spin(nturns*1.75);
//    //spin back on track
//    spin(180);
//    stop();
//    //reset booleans, turns and steps
//    midlight = false;
//    Light = false;
//    PL = false;
//    PR = false;
//    nsteps = 0;
//    nturns = 0;
//    digitalWrite(ylwLED, LOW);
//    digitalWrite(redLED, LOW);
//    delay(1000);
//  }

// no wall following implemented

if (Serial.available()){ 
      Serial.println(Input);
      Input = Serial.readString();
      InputLength = Input.length();
      //Input.remove(0,1); 
  }

//if (InputLength>0){
//  if (Input.charAt(0)=='L'){
//    if((IrL == true && IrR == true && IrF == false)||(IrL == true && IrR == false && IrF == false)){
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      stop();
//      delay(1000);
//    }else if((IrL == true && IrR == false && IrF == true)){
//      delay(500);
//      spin(-90);
//      delay(500);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//    }else if((IrL == false && IrR == true && IrF == false)||(IrL == false && IrR == true && IrF == true)||(IrL == false && IrR == false && IrF == false)||(IrL == false && IrR == false && IrF == true)){
//      delay(500);
//      spin(88);
//      delay(500);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      Input.remove(0,1);
//      InputLength = Input.length();
//    }
//  }else if(Input.charAt(0)=='R'){
//    if((IrL == true && IrR == true && IrF == false)||(IrL == false && IrR == true && IrF == false)){
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      stop();
//      delay(1000);
//    }else if((IrL == true && IrR == false && IrF == false)||(IrL == true && IrR == false && IrF == true)||(IrL == false && IrR == false && IrF == false)||(IrL == false && IrR == false && IrF == true)){
//      delay(500);
//      spin(-90);
//      delay(500);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      Input.remove(0,1);
//      InputLength = Input.length();
//    }else if((IrL == false && IrR == true && IrF == true)){
//      delay(500);
//      spin(88);
//      delay(500);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//
//    }
//  }else if(Input.charAt(0)=='T'){
//    if((IrL == true && IrR == true && IrF == false)||(IrL == false && IrR == false && IrF == false)){
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      stop();
//      delay(1000);
//    }else if((IrL == true && IrR == false && IrF == false)){
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      stop();
//      delay(1000);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//    }else if((IrL == false && IrR == true && IrF == false)){
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//      stop();
//      delay(1000);
//      forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
//    }else if((IrL == false && IrR == true && IrF == true)||(IrL == true && IrR == false && IrF == true)||(IrL == true && IrR == true && IrF == true)||(IrL == false && IrR == false && IrF == true)){
//      stop();
//      Input.remove(0,1);
//      InputLength = Input.length();
//    }
//  }
//}else{
//  stop();
//}


if (InputLength>0){
  if (Input.charAt(0)=='L'){

      spin(88);
      stop();
      delay(1000);
      Input.remove(0,1);
      InputLength = Input.length();
  }else if(Input.charAt(0)=='R'){
      spin(-90);
      stop();
      delay(1000);
      Input.remove(0,1);
      InputLength = Input.length();
  }else if(Input.charAt(0)=='F'){
    forward(qrtr_rot*6.8*1.9,robot_spd);//adjust the constant to make the robot move one block);
    stop();
    delay(1000);
    Input.remove(0,1);
    InputLength = Input.length();
  }else if(Input.charAt(0)=='U'){
    spin(-180);
    stop();
    delay(1000);
    Input.remove(0,1);
    InputLength = Input.length();
  }else if((Input.charAt(0)=='T')||(Input.charAt(0)=='S')){
      stop();
      Input.remove(0,1);
      InputLength = Input.length();
  }else if(Input.charAt(0)=='C'){
      stop();
      Input.remove(0,1);
      InputLength = Input.length();
  }
}


  Serial.println(Input);
}

// random wander function
// random generate a number between 1 to 3, use the number to decide the moving behavior
void randomwonder(){
  // generate random states by using random number
  if (randomstate == 0){
    forward(-800, 250);
    randomstate = random(1,3);
  }else if(randomstate == 1){
    spin(15);
    randomstate = 0;
  }else if(randomstate == 2 ){
    spin(-15);
    randomstate = 0;
  }
}
//Stop function to stop motors
void stop() {
  stepperRight.stop();//stopright
  stepperLeft.stop();//stopleft
}


// reverse function to go backward
void reverse(int rot, int spd) {
    forward(-rot, spd);
}


// a no blocking function to control the wheel speed.
// can continuously change the speed and break by a blocking function
void runAtSpeed (int Rspd,int Lspd) {
  stepperRight.setSpeed(-Rspd);
  stepperLeft.setSpeed(-Lspd);
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


  stepperRight.move(-rot*0.99);//move one full rotation forward relative to current position
  stepperLeft.move(-rot*0.98);//move one full rotation forward relative to current position
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd*0.98);//set left motor speed

  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the target

  //stepperRight.run();
  //stepperLeft.run();
  //steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
  //runToStop();
}

//state update function
//update states according to current detection

void updateState() {
  prevState = state; // set state

  
  if (obstacle == false) { //no sensors triggered
    state = wander; //wonder state
  }  else if (obstacle == true) {
    if(IrR == true && IrL == false && IrF == false){ //only right triggered
      state = fright; //wall right
    }else if(IrL == true && IrR == false && IrF == false){ // only left triggered
      state = fleft; //wall left
    }else if(IrL == true && IrR == true && IrF == false){ //both left and right triggered
      state = fcenter; //in hallway
    }else if(((IrL == true && IrR == false && IrF == true)||(IrL == false && IrR == true && IrF == true)) && (prevState == fright || prevState == fleft)){ //front and left/right triggered
      state = Insidecorner;//inside corner
    }else if((IrL == true && IrR == true && IrF == true) && (prevState == fcenter)){ // all triggered
      state = endhall; // end of hall
    }else if(((IrL == false && IrR == false && IrF == false)||(IrL == false && IrR == false && IrF == false)) && (prevState == fright || prevState == fleft)){ //no detection after wall following
      state = outsidecorner; // outside corner
      obstacle = false;
    }else if(IrF == true && (IrL == false && IrR == false)){ // only front triggered
      state = runAway; //avoid collision
      obstacle = true;
    }
  }
//        Serial.print("IrL = \t"); Serial.print(IrL);
//        Serial.print("IrR = \t"); Serial.println(IrR);
//        Serial.print("IrF = \t"); Serial.print(IrF);
//        Serial.print("SonarR = \t"); Serial.println(SonarR);
//        Serial.print("SonarL = \t"); Serial.println(SonarL);
//
  printState();
}

//serial print state to check current state
void printState(){
  if (state == wander){
    Serial.println("wander");
  } else if (state == fright){
    Serial.println("fright");
  } else if (state ==  fleft){
    Serial.println("fleft");
  } else if (state == fcenter){
    Serial.println("fcenter");
  } else if (state == Insidecorner){
    Serial.println("Insidecorner");
  } else if (state == endhall){
    Serial.println("endhall");
  } else if (state == outsidecorner){
    Serial.println("outsidecorner");
  } else {
    Serial.println("Unknown state");
  }
}

//time interrupt
// update readings and speed in the background
// update ir, sonar, error, speed, state

void updateSensors() {
  test_state = !test_state;
  digitalWrite(test_led, test_state);
//  state = 0;
//  obstacle = false;
  updateIR(); //update ir
  updateSonar2(); //update sonar reading
//  updateLight();
//  center(); //speed for hall way state

  //state = fleft;
  
  updateError(); //calculate error 
  updateState(); //update state
  //prevState = fright;
  //state = Insidecorner;
  //updateSpeed();
//  wallP(); //speed for wall following state
}

//calculate speed for wall following state
// use ir sensor reading
// PD control included

void wallP(){
  spdL=200;// base left speed
  spdR=200;// base right speed
  kp=10; // P value
  kd=10; // D value

  if ((state == fleft)){ // left or right wall condition
    
    digitalWrite(grnLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    
    if ((li_cerror>0)){//&&(turns<=13)){ // too close condition
          spdL=spdL-kp*li_perror+kd*li_derror;  // PD for left wheel
          spdR=spdR+kp*li_perror+kd*li_derror;  // PD for right wheel
          turns=turns+1;    // count for turns to prevent overshoot
          // LED indecator
          digitalWrite(grnLED, LOW);
          digitalWrite(ylwLED, HIGH);
          digitalWrite(redLED, LOW);
    }else if((li_cerror<0)){//&&(turns<=13)){ // too close condition
          spdL=spdL-kp*1.1*li_perror+kd*li_derror;
          spdR=spdR+kp*1.1*li_perror+kd*li_derror;
          turns=turns+1;
          digitalWrite(grnLED, LOW);
          digitalWrite(ylwLED, LOW);
          digitalWrite(redLED, HIGH);
          
    }else if((li_cerror==0)){ // in side range condition
      spdL=spdL-kd*(li_curr-6)/2; // small PD adjustment to maintain the dynamics of robot
      turns = 0; //reset turns
      
    }

  }else if(state == fright){ // same logic expect for right wall condtion
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    if ((ri_cerror>0)){//&&(turns<=10)){
          spdR=spdR-kp*1.2*ri_perror+kd*ri_derror;
          spdL=spdL+kp*1.2*ri_perror+kd*ri_derror;
          turns=turns+1;
          digitalWrite(grnLED, LOW);
          digitalWrite(ylwLED, HIGH);
          digitalWrite(redLED, LOW);
    }else if((ri_cerror<0)){//&&(turns<=10)){
          spdR=spdR-kp*1.3*ri_perror+kd*ri_derror;
          spdL=spdL+kp*1.3*ri_perror+kd*ri_derror;
          turns=turns+1;
          digitalWrite(grnLED, LOW);
          digitalWrite(ylwLED, LOW);
          digitalWrite(redLED, HIGH);
    }else if((ri_cerror==0)){
      spdR=spdR-kd*(ri_curr-6)/2;
      turns = 0;
    }
  
  
  }
//  Serial.println(spdR);
//  Serial.println(spdL);
}

//calculate speed for hall following state
// use ir sensor to detect wall
// use sonar to control the speed (sonar angle can be adjusted)
// PD control included

void center(){
  Vlefts=200; // base left speed
  Vrights=200;// base right speed
  kp=4;       //P value
  kd=10;      //D value
    Vrights=Vrights+kp*dserror+kd*rs_derror;    // PD for left wheel
    Vlefts=Vlefts-kp*dserror+kd*rs_derror;      // PD for right wheel
//  if (dserror>=5 && cturns<=30){
//      spdR=spdR-kp*dserror+kd*ri_derror;
//    spdL=spdL+kp*dserror+kd*ri_derror;
//    cturns = cturns+1;
//  }else if(dserror<=-5 && cturns<=30){
//      spdR=spdR-kp*dserror+kd*ri_derror;
//    spdL=spdL+kp*dserror+kd*ri_derror;
//    cturns = cturns+1;
//  }else if(dserror>-5 && dserror<5){
//    cturns = 0;
//    spdR=spdR-kp*dserror*0.5;
//    spdL=spdL+kp*dserror*0.5;
//  }

//   Serial.println(Vrights);
//   Serial.println(Vlefts);

}

// update sonar reading and calculate basic error

void updateSonar2() {
  SonarL = false;
  SonarR = false;
  long left, right;
  //read right sonar
//  srRightAvg =  sonarRt.ping_in();//read right sonar in inches
//  delay(50);                      //delay 50 ms
//  srLeftAvg = sonarLt.ping_in();  //reaqd left sonar in inches
  
  
  pinMode(snrRight, OUTPUT);//set the PING pin as an output
  digitalWrite(snrRight, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrRight, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrRight, LOW);//set pin low first again
  pinMode(snrRight, INPUT);//set pin as input with duration as reception
  srRightAvg = pulseIn(snrRight, HIGH)/57;//measures how long the pin is high

  cmFR = srRightAvg; //take the reading
  //read left sonar
  pinMode(snrLeft, OUTPUT);//set the PING pin as an output
  digitalWrite(snrLeft, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrLeft, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrLeft, LOW);//set pin low first again
  pinMode(snrLeft, INPUT);//set pin as input with duration as reception
  srLeftAvg = pulseIn(snrLeft, HIGH)/57;//measures how long the pin is high

  
  cmFL = srLeftAvg; //take the reading
  //  print sonar data
//      Serial.println("leftSNR\trightSNR");
//      Serial.print(cmFL); Serial.print("\t");
//      Serial.println(cmFR);
  if (cmFR < snrThresh){ // test the distance to check whether in wall or hall condition
    //bitSet(flag, obFRight);//set the front right obstacle
    //Serial.println(right);
    SonarR = true;  // sonar right detection
    obstacle = true;  // obstacle ditection
    }
  else{
    //bitClear(flag, obFRight);//clear the front right obstacle
    SonarR = false;  //no sonar right detection
  }
  if (cmFL < snrThresh){
    //bitSet(flag, obFLeft);//set the front left obstacle
    SonarL = true; // sonar left detection
    obstacle = true;// obstacle ditection
  }
  else{
    //bitClear(flag, obFLeft);//clear the front left obstacle
    SonarL = false; // sonar right detection
  }

  rs_curr = srRightAvg;             //log current sensor reading [right sonar]
  if ((rs_curr > snrMax) || (rs_curr < snrMin))
    rs_cerror = rs_curr - snrMax;    //calculate current error (too far positive, too close negative)
  else
    rs_cerror = 0;                  //set error to zero if robot is in dead band
  rs_derror = rs_cerror - rs_perror; //calculate change in error
  rs_perror = rs_cerror;            //log current error as previous error [left sonar]

  ls_curr = srLeftAvg;                   //log current sensor reading [left sonar]
  if ((ls_curr > snrMax) || (ls_curr < snrMin))
    ls_cerror = ls_curr - snrMax;     //calculate current error
  else
    ls_cerror = 0;                  //error is zero if in deadband
  ls_derror = ls_cerror - ls_perror; //calculate change in error
  ls_perror = ls_cerror;                //log reading as previous error
}

// update IR reading and calculate basic error

void updateIR() {
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  IrF = false;
  IrB = false;
  IrL = false;
  IrR = false;
  // average ir readings to avoid bad reading for all direction
  
  inirFAvg = irFrontList.reading(analogRead(irRear)); 
  inirBAvg = irRearList.reading(analogRead(irFront));
  inirLAvg = irLeftList.reading(analogRead(irRight));
  inirRAvg = irRightList.reading(analogRead(irLeft));

  
  // calibrate ir reading for all direction
  inirF = 2000/(inirFAvg+50)-1.5;
  inirB = 2000/(inirBAvg+50)-1.5;
  inirL = 2500/(inirLAvg-32)-1.8;
  inirR = 2500/(inirRAvg-32)-1.8;
//
//  inirF = 2000/(analogRead(irFront)+50)-1.5;
//  inirB = 2000/(analogRead(irRear)+50)-1.5;
//  inirL = 2500/(analogRead(irLeft)-32)-1.8;
//  inirR = 2500/(analogRead(irRight)-32)-1.8;

  //clear false reading
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
//      Serial.println("frontIR\tbackIR\tleftIR\trightIR");
//      Serial.print(inirF); Serial.print("\t");
//      Serial.print(inirB); Serial.print("\t");
//      Serial.print(inirL); Serial.print("\t");
//      Serial.println(inirR);

// test the distance to check whether in wall, hallway, obstacle condition
  if (inirF <= irMax){
    obstacle = true;
    IrF = true;
    //Serial.println("True");
   }else{
    IrF = false;
 }
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

  }else{
    IrR = false;
  }

//       Serial.println("frontIR\tbackIR\tleftIR\trightIR");
//      Serial.print(IrF); Serial.print("\t");
//      Serial.print(IrB); Serial.print("\t");
//      Serial.print(IrL); Serial.print("\t");
//      Serial.println(IrR);
//
//  //calcuate error

    ri_curr = inirR;             //log current sensor reading [right IR]
  if ((ri_curr > irMax) | (ri_curr < irMin)){
    ri_cerror = irMax - ri_curr;  //calculate current error (too far positive, too close negative)
    //Serial.println("True");
  }else{
    ri_cerror = 0;                  //set error to zero if robot is in dead band
  }
  ri_derror = ri_cerror - ri_perror; //calculate change in error
  ri_perror = ri_cerror;            //log current error as previous error [left sonar]

  li_curr = inirL;                   //log current sensor reading [left sonar]
  if ((li_curr > irMax)){
    li_cerror = irMin - li_curr;   //calculate current error
  }else if((li_curr < irMin)){
    li_cerror = irMax - li_curr;
  } else{
    li_cerror = 0;                  //error is zero if in deadband
  }
  li_derror = li_cerror - li_perror; //calculate change in error
  li_perror = li_cerror;                //log reading as previous error

//  if (inirR > 0 && inirR < 1000) { //filter out garbage readings
//        Serial.print("right IR current = \t"); Serial.print(ri_curr);
//        Serial.print("\tright IR cerror = \t"); Serial.println(ri_cerror);
//        Serial.print("\tright IR derror = \t"); Serial.print(ri_derror);
//        Serial.print("\tright IR perror = \t"); Serial.println(ri_perror);
//  }
//
//  if (inirL > 0 && inirL < 1000) { //filter out garbage readings
//        Serial.print("left IR current = \t"); Serial.print(li_curr);
//        Serial.print("\tleft IR cerror = \t"); Serial.println(li_cerror);
//        Serial.print("\tleft IR derror = \t"); Serial.print(li_derror);
//        Serial.print("\tleft IR perror = \t"); Serial.println(li_perror);
//  }

  
}


// function to update the left and right error
//

void updateError() {
  left_derror = ls_cerror*0.39 - li_cerror; //difference between left front and back sensor, use threshold for robot mostly parallel to wall
  right_derror = rs_cerror*0.39  - ri_cerror; //difference between right front and back sensor, use threshold for robot mostly parallel to wall
  //derror = ls_cerror - rs_cerror;//use sonar data for difference error
  derror = li_cerror - ri_cerror; //use IR data for difference error
  dserror = -(ls_cerror - rs_cerror);
//    Serial.print("left derror\t"); Serial.print(left_derror);
//   Serial.print("derror\t"); Serial.print(dserror);
//    Serial.print("\right derror\t"); Serial.println(right_derror);
}

// function to update the left and right photosensors
void updateLight(){
  rightReading= 0;    //reset right photosensor reading
  leftReading=0;      //reset left photosensor reading
  photocellReadingL.reading(analogRead(phLeft));  //read left and right data and get averaged value
  photocellReadingR.reading(analogRead(phRight));

  rightReading = photocellReadingR.getAvg()-20; //calibrate readings
  leftReading = photocellReadingL.getAvg();
  sumReading = rightReading+leftReading;  //sum readings for light centering detection

//  check whether the light source is detected
  if (rightReading > 260){
    PR = true;
    Light = true;
  }
//  else{
//    PR = false;
//  }
  if (leftReading > 260){
    PL = true;
    Light = true;
  }
//  else{
//    PR = false;
//  }

// test whether the robot is directly point to the light
if (sumReading-prevsumReading < -3&&prevsumReading>500){
  if(rightReading-leftReading<3||rightReading-leftReading>-3){
      midlight = true;
  }

}

// send readings to serial output

  Serial.print(rightReading);
  Serial.print(" \t ");
  Serial.print(leftReading);
  Serial.print(" \t ");
  Serial.print(sumReading);
  Serial.print(" \t ");
  Serial.print(Light);
    Serial.print(" \t ");
  Serial.print(midlight);
  Serial.println(" \t ");

  //save old readings
  prevsumReading = sumReading;
}
