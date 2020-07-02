#include <Tone.h>


int left_spd=26;
int left_dir=30;
int right_spd=46;
int right_dir=42;
int encoderLB=18; 
int encoderLA=19; 
int encoderRB=20; 
int encoderRA=21;
int freq=1000;

//Encoder myEnc(encoderLA, encoderLB);
int c = 0;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

Tone leftm;
Tone rightm;


void setup()
{
  leftm.begin(left_spd);
  rightm.begin(right_spd);
  
  Serial.begin(9600);

  pinMode(left_spd, OUTPUT);
  pinMode(left_dir, OUTPUT);
  pinMode(right_spd, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);

   
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE); //left     //when i used encoderLB and encoderLA, does not interrupt 
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE); //left     
  attachInterrupt(digitalPinToInterrupt(encoderRB), updateEncoder2, CHANGE); //right                  //uno only has 2 interrupt pins 2 & 3
  attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoder2, CHANGE); //right                  //mega has 6 interrupt pins 2 & 3 & 21 & 20 & 19 & 18 

  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,LOW);
  leftm.play(freq);
  rightm.play(freq);
  }
long old = -999;
void loop()
{  
  /*long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    ///Serial.println(newPosition);
  }
  if(newPosition >= 17000){
    m.stop();
  }
  

  if(old != encoderValue){
    old = encoderValue;
    Serial.println(encoderValue);
  
  
   //Serial.println(encoderValue2);
  }*/

  if(encoderValue >= 130769){
    leftm.stop();
    rightm.stop();
  }

  

  
}

void updateEncoder(){
int MSB = digitalRead(encoderLB); //MSB = most significant bit
int LSB = digitalRead(encoderLA); //LSB = least significant bit
int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue= encoderValue+1.;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue= encoderValue-1;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded = encoded; //store this value for next time
}


void updateEncoder2(){
int MSB = digitalRead(encoderRB); //MSB = most significant bit
int LSB = digitalRead(encoderRA); //LSB = least significant bit
int encoded2 = (MSB << 1) |LSB; //converting the 2 pin value to single number
int sum = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue= encoderValue2+1;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue= encoderValue2-1;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded2 = encoded2; //store this value for next time
}
