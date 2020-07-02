#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Tone.h>
#include <std_msgs/Float32.h>


ros::NodeHandle nh; //start ros node

Tone leftm;
Tone rightm;

//Hardware measurements
const float Dbase = 0.3; //distance between wheels in metres
const float dbase = Dbase/2;
const float wheelD = 0.165; //diameter of wheel
const float wheelR = wheelD/2;
const int ppr = 800; //pulse per revolution of motor
const int sum_enc = 16000; //pulse per rev of encoder


float angular_vel = 0.0;
float linear_vel = 0.0;
float Vright = 0.0; //Velocity of right wheel
float Vleft = 0.0; //Velocity of left wheel
int freq_left = 0;
int freq_right = 0;

//Arduino pins
const int left_motor_pin = 1;
const int right_motor_pin = 2;
const int encoderLB = 18;
const int encoderLA = 19;
const int encoder RB = 20;
const int encoder RA = 21;

//encoder interupt variables 
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

//ROS receive cmd_vel data
void messageCb( const geometry_msgs::Twist &cmd_msg) {
angular_vel=cmd_msg.angular.z;
linear_vel=cmd_msg.linear.x;
unsigned long StartTime = millis();
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb );


void setup() {
  Serial.begin(57600);

  leftm.begin(left_motor_pin);
  rightm.begin(right_motor_pin);
    
  nh.initNode();
  nh.subscribe(sub_vel);
  
}

void loop() {
  //Calculate velocity of each wheel
  Vleft = linear_vel + angular_vel * dbase;
  Vright = linear_vel - angular_vel * dbase;
  //Calculate pwm to send to each motor
  freq_left = ppr * Vleft /(2*PI*wheelR);
  freq_right = ppr * Vright /(2*PI*wheelR);

  
  leftm.play(freq_left);
  rightm.play(freq_right);
  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;
  if(ElaspedTime >= 1000){
    Vleft = 0.0;
    Vright = 0.0;
  }
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
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue2++;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue2--;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded2 = encoded2; //store this value for next time
}
