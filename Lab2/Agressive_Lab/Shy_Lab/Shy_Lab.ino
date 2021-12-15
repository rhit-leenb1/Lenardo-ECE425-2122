#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>

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
#define max_accel     10000//maximum robot acceleration
#define robot_spd     250 //set robot speed
#define robot_spd_mid     150 //set robot speed
#define robot_spd_min     50 //set robot speed
#define max_spd       2500//maximum robot speed

#define irThresh    400 // The IR threshold for presence of an obstacle in ADC value
#define snrThresh   18  // The sonar threshold for presence of an obstacle in cm
#define snrThreshmid   15  // The sonar midian threshold for presence of an obstacle in cm
#define snrThreshmin   10  // The sonar minimum threshold for presence of an obstacle in cm
#define minThresh   1   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

// IR

//

//sonar Interrupt variables
volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
//volatile uint8_t test_state = 0;
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data
volatile boolean test_state; //variable to hold test led state for timer interrupt

byte flag = 0;    // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int count; //count number of times collide has tripped
#define max_collide 250 //maximum number of collides before robot reverses

//state byte to hold robot motion and state data
byte state = 0;   //state to hold robot states and motor motion

//bit definitions for robot motion and state byte
#define movingR   1  // Moving Right Motor in progress flag
#define movingL   2  // Moving Left Motor in progress flag
#define fwd       3
#define rev       4
#define collide   5
#define runAway   6
#define wander    7
#define bot_stop    0

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

void setup() {
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

  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");
 // digitalWrite(redLED,HIGH);
  delay(2500); //seconds before the robot moves
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(100);
  Shyfunction();
 // updateSonar();

}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

void forward(int rot, int spd) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setMaxSpeed(spd);//set right motor speed
  stepperLeft.setMaxSpeed(spd);//set left motor speed
  //stepperRight.setSpeed(robot_spd);//set right motor speed
  //stepperLeft.setSpeed(robot_spd);//set left motor speed

  //stepperRight.setCurrentPosition(0);
  //stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot*10; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot*10; //left motor absolute position
  steppers.moveTo(positions);

  //stepperRight.run();
  //stepperLeft.run();
  steppers.runSpeedToPosition();
  //delay(100);
  //steppers.run(); //move forward with no blocking
}

void reverse(int rot, int spd) {
    forward(-rot, spd);
}

void updateState() {
  if (!(flag)) { //no sensors triggered
    bitSet(state, bot_stop); //set forward motion
    bitClear(state, collide);//clear collide state
    count--;//decrement collide counter
  }
  else if (bitRead(flag, obFRight)) { //front sensors triggered
    bitClear(state, bot_stop); //clear reverse motion
    bitSet(state, rev);//set collide state
  }
  
  //print flag byte
     // Serial.println("\trtSNR\tltSNR\tltIR\trtIR\trearIR\tftIR");
     // Serial.print("flag byte: ");
     // Serial.println(flag, BIN);
  //print state byte
     // Serial.println("\twander\trunAway\tcollide\treverse\tforward");
     // Serial.print("state byte: ");
     //Serial.println(state, BIN);
}

void Shyfunction() {
  if (bitRead(flag, obFRight) || bitRead(state, rev)) { //check for a collide state
    
    reverse(one_rotation, robot_spd);
    
    Serial.println("robot reverse");
  }
  else{
    Serial.println("robot forward");
    stop();
    //forward(one_rotation);//move forward as long as all sensors are clear
  }
}

void updateSensors() {
  test_state = !test_state;
  digitalWrite(test_led, test_state);
  flag = 0;       //clear all sensor flags
  state = 0;      //clear all state flags
  //updateIR();     //update IR readings and update flag variable and state machine
  //updateSonar();  //update Sonar readings and update flag variable and state machine
  updateSonar2(); //there are 2 ways to read sonar data, this is the 2nd option, use whichever one works best for your hardware
  updateState();  //update State Machine based upon sensor readings
  //delay(1000);     //added so that you can read the data on the serial monitor
}

void updateSonar2() {
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
    bitSet(flag, obFRight);//set the front right obstacle
    Serial.println(right);
    }
  else{
    bitClear(flag, obFRight);//clear the front right obstacle
  }
  if (left < snrThresh){
    bitSet(flag, obFLeft);//set the front left obstacle
  }
  else{
    bitClear(flag, obFLeft);//clear the front left obstacle
  }
}
