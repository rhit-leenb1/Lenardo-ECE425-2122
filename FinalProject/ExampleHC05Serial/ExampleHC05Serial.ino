//Connect to HC05 BlueTooth Module (Name: 'LEO', code: '1234')
//Connect Arduino Mega 2560 at COM Port 9
//Upload code
//Connect Serial port to 9 

String BT_input;                          // to store input character received via BT.
int LED = 5;                            // device to control

char onChar = 65;
char offChar = 66;

void setup()  
{  
  Serial.begin(115200);                      //set baud rate of module (115200 on com9)
  pinMode(LED, OUTPUT);
  while (!Serial) 
  {
     // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() 

 { 
  if (Serial.available())
    {   
        BT_input = Serial.readString();   // read input string from bluetooth 

        uint8_t dataArray[BT_input.length()];
        BT_input.toCharArray(dataArray, BT_input.length());
        
        for (int i = 0; i < BT_input.length();i++){
          if (dataArray[i]==65)                
          {
            digitalWrite(LED, HIGH);
            Serial.println("LED is ON");
          }
          else if (dataArray[i]==66)
          {
            digitalWrite(LED, LOW);
            Serial.println("LED is OFF");
          }
          else 
          {     
            //Serial.println("Send 'A' to get LED ON");
            //Serial.println("Send 'B' to get LED OFF");
          }
          Serial.println("");
          delay(1000);
        }
        Serial.print("Recieved String ");
        Serial.println(BT_input);
    }
 
}
