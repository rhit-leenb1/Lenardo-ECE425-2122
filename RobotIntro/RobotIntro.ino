/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  digital pin 14 - red LED in series with 220 ohm resistor
  digital pin 15 - green LED in series with 220 ohm resistor
  digital pin 16 - yellow LED in series with 220 ohm resistor


  INSTALL THE LIBRARY
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//define pin numbers
const int rtStepPin = 50; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 51;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 52; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin (no change in pin for wireless)
const int stepTime = 500; //delay time between high and low on step pin

//define robot features
const int stepsPerRotation = 800;               //800 steps make one full wheel rotation
const float wheelDiameter = 3.375;                  //wheel diameter in inches
const int defaultRightWheelSpeed = 500;         // default speed for the right Wheel (speeds has been tested from Lab1)
const int defaultLeftWheelSpeed = 500;          // default speed for the left Wheel (speeds has been tested from Lab1)
const float dleft = 4.25;
const float dright = 4.25;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define enableLED 13        //stepper enabled LED
#define redLED 5           //red LED for displaying states
#define grnLED 6         //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define pauseTime 2500 //time before robot moves

void setup()
{
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED


  stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  Serial.begin(9600); //start serial communication at 9600 baud rate for debugging
}

void loop()
{
  //uncomment each function one at a time to see what the code does
 // move1();//call move back and forth function
 //move2();//call move back and forth function with AccelStepper library functions
 // move3();//call move back and forth function with MultiStepper library functions
  //move4(); //move to target position with 2 different speeds
  //move5(); //move continuously with 2 different speeds

//  forward(4);
//  stop();
//  reverse(204);
//  stop();
//  pivot(2);
//  spin(2);
//  turn(2);

  //moveCircle(36,1);
  //delay(5000);
  //moveFigure8(36);

  //gotoangle(90);
  //delay(1000);
  //gotoangle(-90);
  delay(5000);
  //gotogoal(-8,8);
  moveSquare(24);
  delay(1000);
}




/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
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

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void pivot(int direction) {
  int distance = PI*dleft;
  
  int stepsToTake = stepsPerRotation*distance/(PI*wheelDiameter);
  int directionUnitV = direction / abs(direction);

  if (directionUnitV>0){
    stepperRight.move(stepsToTake * directionUnitV);
    stepperRight.setSpeed(defaultRightWheelSpeed * directionUnitV);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the  
  }else if(directionUnitV<0){
    stepperLeft.move(stepsToTake * directionUnitV);
    stepperLeft.setSpeed(defaultRightWheelSpeed * directionUnitV);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the  
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }
}

/*
 * The spin() function uses the AccelStepper library
 * The robot will spin about its center from the given angle

 * move() is a library function for relative movement to set a target position
   setSpeed() is a library function for setting motor speed
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking [used in runAtSpee
  
 * runToStop() is a written function to execute runSpeedToPosition() as a blocking
   runAtSpeedToPosition() is a written function that uses constant speed to achieve target positon for both steppers, no blocking
*/
void spin(int direction) {
  int distance = PI*dleft;
  
  int stepsToTake = stepsPerRotation*distance/(PI*wheelDiameter); //calculate how many steps to go to distance
  int directionUnitV = direction / abs(direction);
  
  stepperRight.move(stepsToTake * directionUnitV);//move one full rotation forward relative to current position
  stepperLeft.move(-stepsToTake * directionUnitV);//move one full rotation forward relative to current position
  stepperRight.setSpeed(defaultRightWheelSpeed * directionUnitV);//set right motor speed
  stepperLeft.setSpeed(-defaultLeftWheelSpeed * directionUnitV);//set left motor speed
  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the  
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void turn(int direction) {
  int distance = PI*dleft;
  
  int stepsToTake = stepsPerRotation*distance/(PI*wheelDiameter); //calculate how many steps to go to distance
  int directionUnitV = direction / abs(direction);

  if (directionUnitV>0){
    stepperRight.move(stepsToTake * directionUnitV);//move one full rotation forward relative to current position
    stepperLeft.move(stepsToTake * directionUnitV*0.5);//move one full rotation forward relative to current position
    stepperRight.setMaxSpeed(defaultRightWheelSpeed * directionUnitV);//set right motor speed
    stepperLeft.setMaxSpeed(defaultLeftWheelSpeed * directionUnitV*0.5);//set left motor speed
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the  
  }else if(directionUnitV<0){
    stepperRight.move(stepsToTake * directionUnitV*0.5);//move one full rotation forward relative to current position
    stepperLeft.move(stepsToTake * directionUnitV);//move one full rotation forward relative to current position
    stepperRight.setMaxSpeed(defaultRightWheelSpeed * directionUnitV*0.5);//set right motor speed
    stepperLeft.setMaxSpeed(defaultLeftWheelSpeed * directionUnitV);//set left motor speed
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }

  
}





/*
 * The forward() function uses the AccelStepper library
 * The robot will move forward from the given distance in inches
 * The robot speed is set to 100
 * Maximum distance = 204 inches
 
 * move() is a library function for relative movement to set a target position
   setSpeed() is a library function for setting motor speed
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking [used in runAtSpee
  
 * runToStop() is a written function to execute runSpeedToPosition() as a blocking
   runAtSpeedToPosition() is a written function that uses constant speed to achieve target positon for both steppers, no blocking

 
 * Refer to move2() for more details
 */

void forward(int distance) {
  // inches
  int stepsToTake = stepsPerRotation*distance/(PI*wheelDiameter); //calculate how many steps to go to distance
  stepperRight.move(stepsToTake);//move one full rotation forward relative to current position
  stepperLeft.move(stepsToTake);//move one full rotation forward relative to current position
  stepperRight.setMaxSpeed(defaultRightWheelSpeed*0.9012);//set right motor speed
  stepperLeft.setMaxSpeed(defaultLeftWheelSpeed);//set left motor speed
  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the
}



/*
 * The reverse() function uses the AccelStepper library
 * The robot will move backwards from the given distance in inches
 * Maximum distance = 204 inches

 * forward() is a written function to more robot forward set distance
  
 * Refer to forward() for more details
*/
void reverse(int distance) {
  forward(-distance);
}


/*
 * The stop() function uses the AccelStepper library
 * The robot will stop all movement when called

 * setSpeed() is a library function for setting motor speed
   stop() is a library function stops motor with set speed and acceleration
   
*/
void stop() {
  stepperRight.setSpeed(0);//set right motor speed
  stepperLeft.setSpeed(0);//set left motor speed

  stepperRight.stop(); //stops right motor using set speeds and acceleration
  stepperLeft.stop(); //stops left motor using set speeds and acceleration
}




/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void moveCircle(int diam, int dir) {
  //inches
   
  int v = defaultRightWheelSpeed;
  float r = diam/2;
  float w = v/r;

  float innerDis = (r-dleft)*PI*2;
  float outerDis = (r+dright)*PI*2;
  
  float innersteps = stepsPerRotation*innerDis/(PI*wheelDiameter)*.87;
  float outersteps = stepsPerRotation*outerDis/(PI*wheelDiameter)*.87;
  
  if (dir > 0){
    int vinner = w*(r-dleft);
    int vouter = w*(r+dright);
    stepperLeft.setMaxSpeed(vinner*0.96);
    stepperRight.setMaxSpeed(vouter);
    stepperRight.move(outersteps);//move one full rotation forward relative to current position
    stepperLeft.move(innersteps*0.97);//move one full rotation forward relative to current position
  }else if(dir < 0){
    int vinner = w*(r-dright);
    int vouter = w*(r+dleft);
    stepperRight.setMaxSpeed(vinner*0.96);
    stepperLeft.setMaxSpeed(vouter);
    stepperRight.move(innersteps*1.07);//move one full rotation forward relative to current position
    stepperLeft.move(outersteps*1.1);//move one full rotation forward relative to current position
  }

  
  
  runAtSpeedToPosition(); //run both stepper to set position
  runToStop();//run until the robot reaches the


}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam) {
  moveCircle(diam,1);
  moveCircle(diam,-1);
}

void gotoangle(int angle){
  long stepsToTake = angle*10;
  
  if (angle > 0){
    stepperRight.move(stepsToTake*1.05);
    stepperRight.setMaxSpeed(defaultRightWheelSpeed );
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the  
  }else if (angle < 0){
    stepperLeft.move(-stepsToTake*1.25);
    stepperLeft.setMaxSpeed(defaultRightWheelSpeed);
    runAtSpeedToPosition(); //run both stepper to set position
    runToStop();//run until the robot reaches the  
  }else{
    stepperRight.stop();
    stepperLeft.stop();
  }
}

void gotogoal(int x, int y){
  float theta = atan2(y,x);
  int l = sqrt(sq(x)+sq(y));

  gotoangle(theta*180/PI);
  forward(l);

}

void moveSquare(int side){
  forward(side);
  delay(500);
  gotoangle(90);
  forward(side);
  delay(500);
  gotoangle(90);
  forward(side);
  delay(500);
  gotoangle(90);
  forward(side);
  delay(500);
  gotoangle(90);
}

























/*  Given move code from intro
 */

 

//   The move1() function will move the robot forward one full rotation and backwared on
//   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
//   step. This function uses setting the step pins high and low with delays to move. The speed is set by
//   the length of the delay.
//*/
//void move1() {
//  digitalWrite(redLED, HIGH);//turn on red LED
//  digitalWrite(grnLED, LOW);//turn off green LED
//  digitalWrite(ylwLED, LOW);//turn off yellow LED
//  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
//  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
//  // Makes 800 pulses for making one full cycle rotation
//  for (int x = 0; x < 800; x++) {
//    digitalWrite(rtStepPin, HIGH);
//    digitalWrite(ltStepPin, HIGH);
//    delayMicroseconds(stepTime);
//    digitalWrite(rtStepPin, LOW);
//    digitalWrite(ltStepPin, LOW);
//    delayMicroseconds(stepTime);
//  }
//  delay(1000); // One second delay
//  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
//  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
//  // Makes 800 pulses for making one full cycle rotation
//  for (int x = 0; x < 800; x++) {
//    digitalWrite(rtStepPin, HIGH);
//    digitalWrite(ltStepPin, HIGH);
//    delayMicroseconds(stepTime);
//    digitalWrite(rtStepPin, LOW);
//    digitalWrite(ltStepPin, LOW);
//    delayMicroseconds(stepTime);
//  }
//  delay(1000); // One second delay
//}
//
///*
//   The move2() function will use AccelStepper library functions to move the robot
//   move() is a library function for relative movement to set a target position
//   moveTo() is a library function for absolute movement to set a target position
//   stop() is a library function that causes the stepper to stop as quickly as possible
//   run() is a library function that uses accel and decel to achieve target position, no blocking
//   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
//   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
//   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
//   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
//*/
//void move2() {
//  digitalWrite(redLED, LOW);//turn off red LED
//  digitalWrite(grnLED, HIGH);//turn on green LED
//  digitalWrite(ylwLED, LOW);//turn off yellow LED
//  stepperRight.moveTo(800);//move one full rotation forward relative to current position
//  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
//  stepperRight.setSpeed(100);//set right motor speed
//  stepperLeft.setSpeed(100);//set left motor speed
//  stepperRight.runSpeedToPosition();//move right motor
//  stepperLeft.runSpeedToPosition();//move left motor
//  runToStop();//run until the robot reaches the target
//  delay(1000); // One second delay
//  stepperRight.moveTo(0);//move one full rotation backward relative to current position
//  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
//  stepperRight.setSpeed(1000);//set right motor speed
//  stepperLeft.setSpeed(1000);//set left motor speed
//  stepperRight.runSpeedToPosition();//move right motor
//  stepperLeft.runSpeedToPosition();//move left motor
//  runToStop();//run until the robot reaches the target
//  delay(1000); // One second delay
//}
//
///*
//   The move3() function will use the MultiStepper() class to move both motors at once
//   move() is a library function for relative movement to set a target position
//   moveTo() is a library function for absolute movement to set a target position
//   stop() is a library function that causes the stepper to stop as quickly as possible
//   run() is a library function that uses accel and decel to achieve target position, no blocking
//   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
//   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
//   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
//   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
//*/
//void move3() {
//  digitalWrite(redLED, LOW);//turn off red LED
//  digitalWrite(grnLED, LOW);//turn off green LED
//  digitalWrite(ylwLED, HIGH);//turn on yellow LED
//  long positions[2]; // Array of desired stepper positions
//  positions[0] = 800;//right motor absolute position
//  positions[1] = 800;//left motor absolute position
//  steppers.moveTo(positions);
//  steppers.runSpeedToPosition(); // Blocks until all are in position
//  delay(1000);//wait one second
//  // Move to a different coordinate
//  positions[0] = 0;//right motor absolute position
//  positions[1] = 0;//left motor absolute position
//  steppers.moveTo(positions);
//  steppers.runSpeedToPosition(); // Blocks until all are in position
//  delay(1000);//wait one second
//}
//
///*this function will move to target at 2 different speeds*/
//void move4() {
//
//  long positions[2]; // Array of desired stepper positions
//  int leftPos = 5000;//right motor absolute position
//  int rightPos = 1000;//left motor absolute position
//  int leftSpd = 5000;//right motor speed
//  int rightSpd = 1000; //left motor speed
//
//  digitalWrite(redLED, HIGH);//turn on red LED
//  digitalWrite(grnLED, HIGH);//turn on green LED
//  digitalWrite(ylwLED, LOW);//turn off yellow LED
//
//  //Uncomment the next 4 lines for absolute movement
//  //stepperLeft.setCurrentPosition(0);//set left wheel position to zero
//  //stepperRight.setCurrentPosition(0);//set right wheel position to zero
//  //stepperLeft.moveTo(leftPos);//move left wheel to absolute position
//  //stepperRight.moveTo(rightPos);//move right wheel to absolute position
//
//  //Unomment the next 2 lines for relative movement
//  stepperLeft.move(leftPos);//move left wheel to relative position
//  stepperRight.move(rightPos);//move right wheel to relative position
//
//  stepperLeft.setSpeed(leftSpd);//set left motor speed
//  stepperRight.setSpeed(rightSpd);//set right motor speed
//  runAtSpeedToPosition();//run at speed to target position
//}
//
///*This function will move continuously at 2 different speeds*/
//void move5() {
//  digitalWrite(redLED, LOW);//turn off red LED
//  digitalWrite(grnLED, HIGH);//turn on green LED
//  digitalWrite(ylwLED, HIGH);//turn on yellow LED
//  int leftSpd = 5000;//right motor speed
//  int rightSpd = 1000; //left motor speed
//  stepperLeft.setSpeed(leftSpd);//set left motor speed
//  stepperRight.setSpeed(rightSpd);//set right motor speed
//  runAtSpeed();
//}
