

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

#define snrMin   13               // sonar minimum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMax   15               // sonar maximum threshold for wall (use a deadband of 4 to 6 inches)


#define irThresh    13 // The IR threshold for presence of an obstacle in ADC value
#define irMax    7
#define irMin    5
#define snrThresh   60  // The sonar threshold for presence of an obstacle in cm
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

const int irListSize = 3;
movingAvg irFrontList(irListSize);  //variable to holds list of last front IR reading
movingAvg irLeftList(irListSize);   //variable to holds list of last left IR reading
movingAvg irRearList(irListSize);   //variable to holds list of last rear IR reading
movingAvg irRightList(irListSize);   //variable to holds list of last right IR reading

float inirF = 0;
float inirB = 0;
float inirL = 0;
float inirR = 0;
float inirFAvg = 0;
float inirBAvg = 0;
float inirLAvg = 0;
float inirRAvg = 0;

volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
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

#define timer_int 500000 // 1/2 second (500000 us) period for timer interrupt

boolean obstacle = false;
boolean SonarL = false;
boolean SonarR = false;
boolean IrF = false;
boolean IrB = false;
boolean IrL = false;
boolean IrR = false;

boolean obL = false;
boolean obR = false;

int state = 0; //define state

int randomstate = 0; //define random state

boolean turnedleft = false;
boolean tuenedright = false;


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

int spdL = 0;
int spdR = 0;

int kp;
int kd;

int turns = 0;

boolean leftwall = false;
boolean rightwall = false;

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
 //stepperRight.setAcceleration(100);
 //stepperLeft.setAcceleration(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  runAtSpeed();
  //Serial.println("run");
}

void runAtSpeed () {
  stepperRight.setSpeed(spdR);
  stepperLeft.setSpeed(spdL);
 stepperRight.runSpeed();
 stepperLeft.runSpeed(); 
  
}

void updateState() {

  if (obstacle == false) { //no sensors triggered
    state = wander;
    Serial.println("wander");
  }  else if (obstacle == true) {
    if(IrR == true && IrL == false && IrF == false && SonarR == true && SonarL == false){
      state = fright;
    }else if(IrL == true && IrR == false && IrF == false && SonarR == false && SonarL == true){
      state = fleft;
    }else if(IrL == true && IrR == true && IrF == false && SonarR == true && SonarL == true){
      state = fcenter;
    }else if((IrL == true && IrR == false && IrF == true && SonarR == false && SonarL == true)||(IrL == false && IrR == true && IrF == true && SonarR == true && SonarL == false)){
      state = Insidecorner;
    }else if(IrL == true && IrR == true && IrF == true && SonarR == true && SonarL == true){
      state = endhall;
    }else if((IrL == true && IrR == false && IrF == false && SonarR ==false && SonarL == false)||(IrL == false && IrR == true && IrF == false && SonarR == false && SonarL == false)){
      state = outsidecorner;
    }
  }
//        Serial.print("IrL = \t"); Serial.print(IrL);
//        Serial.print("IrR = \t"); Serial.println(IrR);
//        Serial.print("IrF = \t"); Serial.print(IrF);
//        Serial.print("SonarR = \t"); Serial.println(SonarR);
//        Serial.print("SonarL = \t"); Serial.println(SonarL);


  //Serial.println(state);
}

void wallfollow(){
  
}

void updateSensors() {
  test_state = !test_state;
  digitalWrite(test_led, test_state);
  state = 0;
  obstacle = false;
  updateIR();
  updateSonar2();

  //state = fleft;
  
  updateError();
  //updateState();
  state = fleft;
  //updateSpeed();
  wallP();

  Serial.println(state);
}

void wallP(){
  spdL=200;
  spdR=200;
  kp=10;
  kd=10;

  if ((state == fleft)){
    if ((li_cerror>0)&&(turns<=10)){
          spdL=spdL+kp*li_perror+kd*li_derror;
          spdR=spdR-kp*li_perror+kd*li_derror;
          turns=turns+1;
    }else if((li_cerror<0)&&(turns<=10)){
          spdL=spdL+kp*li_perror+kd*li_derror;
          spdR=spdR-kp*li_perror+kd*li_derror;
          turns=turns+1;
    }else if((li_cerror==0)){
      spdL=spdL+kd*(li_curr-6)/2;
      turns = 0;
      
    }

  }else if(state == fright){
    if ((ri_cerror>0)&&(turns<=10)){
          spdR=spdR+kp*ri_perror+kd*ri_derror;
          spdL=spdL-kp*ri_perror+kd*ri_derror;
          turns=turns+1;
    }else if((ri_cerror<0)&&(turns<=10)){
          spdR=spdR+kp*ri_perror+kd*ri_derror;
          spdL=spdL-kp*ri_perror+kd*ri_derror;
          turns=turns+1;
    }else if((li_cerror==0)){
      spdR=spdR+kd*(ri_curr-6)/2;
      turns = 0;
  }
  Serial.println(spdR);
  Serial.println(spdL);
  
}

void center(){
  spdL=200;
  spdR=200;
  kp=10;
  kd=10;
  spdR=spdR+kp*derror+kd*ri_derror;
  spdL=spdL-kp*derror+kd*ri_derror;
}

void inwall(){
  
}

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

  cmFR = srRightAvg;
  //read left sonar
  pinMode(snrLeft, OUTPUT);//set the PING pin as an output
  digitalWrite(snrLeft, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrLeft, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrLeft, LOW);//set pin low first again
  pinMode(snrLeft, INPUT);//set pin as input with duration as reception
  srLeftAvg = pulseIn(snrLeft, HIGH)/57;//measures how long the pin is high

  
  cmFL = srLeftAvg;
  //  print sonar data
      Serial.println("leftSNR\trightSNR");
      Serial.print(cmFL); Serial.print("\t");
      Serial.println(cmFR);
  if (cmFR < snrThresh){
    //bitSet(flag, obFRight);//set the front right obstacle
    //Serial.println(right);
    SonarR = true;
    obstacle = true;
    }
  else{
    //bitClear(flag, obFRight);//clear the front right obstacle
    SonarR = false;
  }
  if (cmFL < snrThresh){
    //bitSet(flag, obFLeft);//set the front left obstacle
    SonarL = true;
    obstacle = true;
  }
  else{
    //bitClear(flag, obFLeft);//clear the front left obstacle
    SonarL = false;
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
//
//  inirF = 2000/(analogRead(irFront)+50)-1.5;
//  inirB = 2000/(analogRead(irRear)+50)-1.5;
//  inirL = 2500/(analogRead(irLeft)-32)-1.8;
//  inirR = 2500/(analogRead(irRight)-32)-1.8;

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

//  if (inirL > 0 && inirL < 1000) { //filter out garbage readings
//        Serial.print("left IR current = \t"); Serial.print(li_curr);
//        Serial.print("\tleft IR cerror = \t"); Serial.println(li_cerror);
//        Serial.print("\tleft IR derror = \t"); Serial.print(li_derror);
//        Serial.print("\tleft IR perror = \t"); Serial.println(li_perror);
//  }

  
}

void updateError() {
  left_derror = ls_cerror*0.39 - li_cerror; //difference between left front and back sensor, use threshold for robot mostly parallel to wall
  right_derror = rs_cerror*0.39  - ri_cerror; //difference between right front and back sensor, use threshold for robot mostly parallel to wall
  //derror = ls_cerror - rs_cerror;//use sonar data for difference error
  derror = li_cerror - ri_cerror; //use IR data for difference error
    Serial.print("left derror\t"); Serial.print(left_derror);
    Serial.print("\right derror\t"); Serial.println(right_derror);
}
