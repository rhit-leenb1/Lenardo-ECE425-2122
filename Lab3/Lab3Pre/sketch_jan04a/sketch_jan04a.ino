#include <Servo.h>
Servo leftservo;  
Servo rightservo;  
const int pingPin = 5; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
int t;
int nowall;
void setup() {
  nowall = 0;
  leftservo.attach(9);  
  rightservo.attach(10);
   //set up the Serial
  Serial.begin(9600);
  //setupt the pin modes  
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  leftservo.write(90);
  rightservo.write(90);
}
void loop() {
  long duration;  
  //clear the ping pin
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  //send the 10 microsecond trigger
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  //get the pulse duration in microseconds
  duration = pulseIn(echoPin, HIGH);
  Serial.println(duration);
  if (duration<=6300 && duration >=5700){
    nowall = 0;
    forward();
  }else if(duration>6300 && duration<=10000){
    nowall = 0;
    t = (duration-5800)/1.5;
    turnleft();
    delay(t);
    forward();
    delay(500);
    turnright();
    delay(t/1.2);
    stop();
  }else if(duration<5700 && duration >=0){
    nowall = 0;
    t = (6100-duration)/1.5;
    turnright();
    delay(t);
    forward();
    delay(500);
    turnleft();
    delay(t/1.2);
    stop();
  }else if(duration>10000){
    if (nowall == 0){
      turnright();
      delay(2500);
      forward();
      delay(1000);
      stop();
      nowall = 1;
    }else if(nowall >= 1){
      turnleft();
      delay(2500);
      forward();
      delay(500);
      turnright();
      delay(2500);
      //forward();
      //delay(500);
      //turnright();
      //delay(3000);
      stop();
    }

  }
  /*
    TASK: The coins are around 110 cm away from the top wall.
    Use the ultrasonic sensor data to navigate the robot in order
    to collect the coins.
  */
  delay(50); 
  stop(); 
}
void forward(){
  leftservo.write(170);
  rightservo.write(-180);
}

void turnleft(){
  leftservo.write(-180);
  rightservo.write(-180);
  //delay(t);
}

void turnright(){
  leftservo.write(180);
  rightservo.write(180);
}

void stop(){
  leftservo.write(90);
  rightservo.write(90);
}
