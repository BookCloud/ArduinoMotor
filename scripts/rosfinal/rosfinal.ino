#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Tone.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Tone leftm;
Tone rightm;

unsigned long dtime;

int left_spd=26;
int left_dir=30;
int right_spd=46;
int right_dir=42;
int encoderLB=18; 
int encoderLA=19; 
int encoderRB=20; 
int encoderRA=21;
int freq=2000;
int f = 1000;

float linear = 0.0;
float angular = 0.0;
float spd = 0.0;

char state = 's';
bool forward=false;
bool reverse=false;
bool left=false;
bool right=false;
bool Stop=false;
 
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

void messageCb( const geometry_msgs::Twist &cmd_msg) {
angular=cmd_msg.angular.z;
linear=cmd_msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb );
std_msgs::Float32 lencoderData;
ros::Publisher Lencoder("Lencoder", &lencoderData);
std_msgs::Float32 rencoderData;
ros::Publisher Rencoder("Rencoder", &rencoderData);

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
  
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.advertise(Lencoder);
  nh.advertise(Rencoder);
}


long oldEncoder = -999;
long oldEncoder2 = -999;
void loop(){

  
  
  if ( angular == 0 && linear == 0 ) {stopf();}
  else if ( angular < -0 ) {leftf();}
  else if ( angular > 0 )  {;rightf();} 
  else if ( linear < 0.0 ) { reversef();}
  else if ( linear > 0.0 ) { forwardf();}


  if(oldEncoder != encoderValue){
    oldEncoder = encoderValue;
    lencoderData.data = encoderValue;
    Lencoder.publish( &lencoderData );
  }
   if(oldEncoder2 != encoderValue2){
    oldEncoder2 = encoderValue2;
    rencoderData.data = encoderValue2;
    Rencoder.publish( &rencoderData );
  }
  nh.spinOnce();
  
   
}



void forwardf()
{
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,LOW);
  spd = linear * freq;
  leftm.play(f);rightm.play(f);
  state = 'f';
}

void reversef()
{
  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,HIGH);
  leftm.play(f);rightm.play(f);
  state = 'b';
  }

void leftf()
{
  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,LOW);
  spd = linear * freq;
  leftm.play(f);rightm.play(f);
  state = 'l';
  }

void rightf()
{
  digitalWrite(left_dir,LOW);
  digitalWrite(right_dir,HIGH);
  spd = linear * freq;
  leftm.play(f);rightm.play(f);
  state = 'r';
  }

void stopf()
{
  leftm.stop();
  rightm.stop();
  state = 's';
}


void updateEncoder(){
int MSB = digitalRead(encoderLB); //MSB = most significant bit
int LSB = digitalRead(encoderLA); //LSB = least significant bit
int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue++;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue--;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded = encoded; //store this value for next time
}


void updateEncoder2(){
int MSB = digitalRead(encoderRB); //MSB = most significant bit
int LSB = digitalRead(encoderRA); //LSB = least significant bit
int encoded2 = (MSB << 1) |LSB; //converting the 2 pin value to single number
int sum = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue++;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue--;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded2 = encoded2; //store this value for next time
}
