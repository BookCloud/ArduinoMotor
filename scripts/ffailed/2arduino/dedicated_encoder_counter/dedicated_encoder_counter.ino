
//encoder tick count variables
int encoderLB = 2;
int encoderLA = 3;
int encoderRB = 19;
int encoderRA = 18;
volatile int lastEncoded = 0;
volatile long encoderValue;
volatile int lastEncoded2 = 0;
volatile long encoderValue2;
volatile long encoderNewValue;
volatile long encoderNewValue2;

//communication variables
char buf[99];
String message;
String message2;


void setup()
{

  Serial.begin(9600);
  Serial3.begin(57600);
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE); //left     //when i used encoderLB and encoderLA, does not interrupt
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE); //left
  attachInterrupt(digitalPinToInterrupt(encoderRB), updateEncoder2, CHANGE); //right                  //uno only has 2 interrupt pins 2 & 3
  attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoder2, CHANGE); //right                  //mega has 6 interrupt pins 2 & 3 & 21 & 20 & 19 & 18

  }


unsigned long tim = 0;
  
void loop()
{  
/*
  if(millis() - tim >= 1000){
   
    
      
     Serial.print("left is ");
    Serial.println(encoderValue);
        Serial.print("right is ");

    Serial.println(encoderValue2);

    
    
    tim = millis();}*/
}


void serialEvent3(){

  message = String(encoderValue); //add left encoder value
    message2 = String(encoderValue2);;  //add right encoder value

  message.concat(",");    //add delimiter
  message.concat(message2);

  message.toCharArray(buf, 99);
  Serial3.write(buf);    //write data to mega

  Serial3.read();    //clear serial buffer
}



void updateEncoder() {
  int MSB = digitalRead(encoderLB); //MSB = most significant bit
  int LSB = digitalRead(encoderLA); //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue = encoderValue + 1.;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue = encoderValue - 1;
  }
  //17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
  lastEncoded = encoded; //store this value for next time
}


void updateEncoder2() {
  int MSB = digitalRead(encoderRB); //MSB = most significant bit
  int LSB = digitalRead(encoderRA); //LSB = least significant bit
  int encoded2 = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue2 = encoderValue2 + 1;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue2 = encoderValue2 - 1;
  }
  //17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
  lastEncoded2 = encoded2; //store this value for next time
}
