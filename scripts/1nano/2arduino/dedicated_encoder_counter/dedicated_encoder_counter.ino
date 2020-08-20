//encoder tick count variables
int encoderLB = 2;
int encoderLA = 3;
int encoderRB = 19;
int encoderRA = 18;
volatile int lastEncoded = 0;
volatile long encoderValue = -10000;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 9999;
volatile long encoderNewValue;
volatile long encoderNewValue2;

unsigned long lastTime = 0;
const int timeDelay = 10;

void setup()
{

  //Serial.begin(9600);
  Serial1.begin(1000000);
  
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
  if(millis() - lastTime >= timeDelay){
    
    sendData();
    lastTime = millis();
  }
}


void sendData(){

  Serial1.write(0x3C); //start marker "<"
  
    int8_t MSB1 = encoderValue >> 8;  //send most significant bit
    int8_t LSB1 = encoderValue;   //send least significant bit
    Serial1.write(MSB1);
    Serial1.write(LSB1);


    int8_t MSB2 = encoderValue2 >> 8;
    int8_t LSB2 = encoderValue2;
    Serial1.write(MSB2);
    Serial1.write(LSB2);
  
  Serial1.write(0x3E); //end marker ">"
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
