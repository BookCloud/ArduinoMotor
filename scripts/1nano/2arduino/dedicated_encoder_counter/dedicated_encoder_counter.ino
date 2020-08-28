//encoder tick count variables
int encoderLB = 2;
int encoderLA = 3;
int encoderRB = 4;
int encoderRA = 5;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
volatile long encoderNewValue;
volatile long encoderNewValue2;

unsigned long lastTime = 0;
const int timeDelay = 30;

void setup()
{

  //Serial.begin(115200);
  Serial1.begin(115200);
  
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE); //left   
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE); //left
  attachInterrupt(digitalPinToInterrupt(encoderRB), updateEncoder2, CHANGE); //right           
  attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoder2, CHANGE); //right                 

  }


unsigned long tim = 0;
  
void loop()
{  
  if(millis() - lastTime >= timeDelay){
    
    sendData();
    //Serial.print(encoderValue);
    //Serial.print(" ");
    //Serial.println(encoderValue2);
    lastTime = millis();
  }
}


void sendData(){

    Serial1.write(0x3C); //start marker "<"

    //splits encoderValue into 4 bytes
    int8_t byte1Enc1 = encoderValue;
    int8_t byte2Enc1 = encoderValue >> 8;
    int8_t byte3Enc1 = encoderValue >> 16;
    int8_t byte4Enc1 = encoderValue >> 24;
    //send the 4 bytes to mega
    Serial1.write(byte1Enc1);
    Serial1.write(byte2Enc1);
    Serial1.write(byte3Enc1);
    Serial1.write(byte4Enc1);

    //splits encoderValue2 into 4 bytes
    int8_t byte1Enc2 = encoderValue2;
    int8_t byte2Enc2 = encoderValue2 >> 8;
    int8_t byte3Enc2 = encoderValue2 >> 16;
    int8_t byte4Enc2 = encoderValue2 >> 24;
    //send the 4 bytes to mega
    Serial1.write(byte1Enc2);
    Serial1.write(byte2Enc2);
    Serial1.write(byte3Enc2);
    Serial1.write(byte4Enc2);
  
    Serial1.write(0x3E); //end marker ">"

  //Serial.println(encoderValue);
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
