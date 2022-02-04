/*
  AUTHOR: Hazim Bitar (techbitar)
  DATE: Aug 29, 2013
  LICENSE: Public domain (use at your own risk)
  CONTACT: techbitar at gmail dot com (techbitar.com)
  Receives from the hardware serial, sends to software serial.
  Receives from software serial, sends to hardware serial.

  Hardware serial for all boards RX 0 TX 1

  Arduino Uno use TX to pin 2, RX to pin 3 for software serial

  The circuit:
  RX is digital pin 11 (connect to TX of other device)
  TX is digital pin 10 (connect to RX of other device)

  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
*/


#include <SoftwareSerial.h>
#define BTTX 0
#define BTRX 1
#define switchPin 7
SoftwareSerial SerialBT(BTTX, BTRX);

void setup()
{
  pinMode(switchPin, INPUT_PULLUP);
  SerialBT.begin(57600);
  SerialBT.println("Bluetooth connection established");
}

void loop()
{
  delay(300);
  if (SerialBT.available())
  {
    String msg = SerialBT.readString();
    if (msg == "on\r\n")
    {
      digitalWrite(LED_BUILTIN, HIGH);
      SerialBT.println("LED is ON");
      SerialBT.println(digitalRead(switchPin));
    }
    else if (msg == "off\r\n")
    {
      digitalWrite(LED_BUILTIN, LOW);
      SerialBT.println("LED is OFF");
      SerialBT.println(digitalRead(switchPin));
    }
  }
}
