
int Map[4][4] = {{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
bool iszero;
int Dir = 1;
String Input;
int InputLength;

int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//start serial communication in order to debug the software while coding
  Serial.println("Timer Interrupt to Update Sensors......");

}

void loop() {
  // put your main code here, to run repeatedly:
//  delay(2000);
//  randomMap();
//  printArray(Map);
//  if (checknegone()==true){
//      Serial.println("x");
//  }
//  UpdateDirection(-1);
//  Serial.println(Dir);
//
//  while(x = 5){
//    x=x+1;
//    delay(2000);
//   Serial.println(x);
//  }
    if (Serial.available()) {
    Input = Serial.readString();
    InputLength = Input.length();
    if (Input = 'M'){
      printArray(Map);
      x=1;
      while (x<10){
        x=x+1;
        Map[3][3]=x;
        delay(5000);
        printArray(Map);
      }
      Serial.println('E');
    }
    }

}

void printArray( const int a[][ 4 ] ) {
   // loop through array's rows
   for ( int i = 0; i < 4; ++i ) {
      // loop through columns of current row
      for ( int j = 0; j < 4; ++j ){
              Serial.print (a[ i ][ j ] );
      Serial.print (" " ) ; // start new line of output
      }

 } 
 Serial.println("\r");
}

void randomMap(){
  int xrand = 2;//random(0,4);
  int yrand = 3;//random(0,4);
  Map[xrand][yrand] = random(0,3);
}

bool checknegone() {
   for ( int i = 0; i < 4; ++i ) {
      // loop through columns of current row
      for ( int j = 0; j < 4; ++j ){
        if(Map[ i ][ j ]==0){
                    return true;
          //break;
        }
      }
 }
  return false;
}

void UpdateDirection(int turndir){
  Dir = Dir+turndir;
  if (Dir>4){
    Dir = Dir-4;
  }else if(Dir<1){
    Dir = Dir+4;
  }
}
