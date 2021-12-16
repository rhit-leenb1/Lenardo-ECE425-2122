/*DigitalSonar.ino
  4 pin HRC - SR04 sesnro
  trigger and echo pin tied together on pin 12
  data pin 7
  VCC 5V
  GND GND
*/

#include <NewPing.h>
#include <SharpIR.h>
const int PING_PIN = 8;
const int PING_PIN_2 = 9;// Arduino pin for both trig and echo tied together on 4 pin [data pin]

#define irFront   A8    //front IR analog pin
#define irRear    A9    //back IR analog pin
#define irRight   A10   //right IR analog pin
#define irLeft    A11   //left IR analog pin

#define snrLeft   A1   //front left sonar 
#define snrRight  A2  //front right sonar 
NewPing sonar(snrRight,snrRight);//(PING_PIN, PING_PIN);
NewPing sonar2(snrLeft,snrLeft);//(PING_PIN_2, PING_PIN_2);
SharpIR 

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay
  //unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int uS1 = sonar.ping_cm(); //Ping in inches
  //unsigned int uS2 = sonar.ping_cm();//Ping in cm
  Serial.print("4-pin:\t");
  //Serial.print(uS2);
  //Serial.print(uS / US_ROUNDTRIP_CM); // convert time into distance
  //Serial.println(" cm");
  //Serial.print(uS / US_ROUNDTRIP_CM * 0.393); // convert time into distance
  Serial.print(uS1);
  Serial.print(" cm");
  unsigned int uS2 = sonar2.ping_cm();
  Serial.print("4-pin:\t");
  Serial.print(uS2);
  Serial.println(" cm");
}
