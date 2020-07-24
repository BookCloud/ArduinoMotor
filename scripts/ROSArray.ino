#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ModbusMaster.h>  //Library for using ModbusMaster
#include <std_msgs/String.h>



ros::NodeHandle nh; //start ros node

//Hardware measurements
const float baseDistance = 0.3; //distance between wheels in metres
const float baseHalf = baseDistance/2;
const float wheelDiameter = 0.165; //diameter of wheel
const float wheelRadius = wheelDiameter/2;
const float distPerRev = 0.5307; //dist per rev of wheel
const float sumEnc = 4096*4; //total pulses per rev of encoder
const float rateEnc = (2*PI*wheelRadius) / sumEnc; 

float angularVel = 0.0;
float linearVel = 0.0;
float rightVel = 0.0; //Velocity of right wheel
float leftVel = 0.0; //Velocity of left wheel
float rightRpm = 0.0;
float leftRpm = 0.0;

//Motor variables , Common the DE and RE_NEG of BOTH motors
#define MAX485_DE      5                 //put for RSE pin
#define MAX485_RE_NEG  4
#define MAX485_DE2      6                //put for RSE pin
#define MAX485_RE_NEG2  5
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
float result;
float result2;

//Odom variables
float leftEncInc;
float rightEncInc;
float leftEncNew;
float rightEncNew;
volatile long leftEncOld = 0;
volatile long rightEncOld = 0;
float lengthError;
float leftOverflowCount;
float rightOverflowCount;
float leftNewTicksCount;
float rightNewTicksCount;
float leftOldTicksCount = 0;
float rightOldTicksCount = 0;
float leftCountDifference;
float rightCountDifference;

float ang_z = 0.0;
float ang_z_error;
double Xw = 0.0;
double Yw = 0.0;
float delta_d;

unsigned long pressedTime;
unsigned long currentTime;


//encoder tick count variables
int encoderLB=2; 
int encoderLA=3; 
int encoderRB=21; 
int encoderRA=20;
volatile int lastEncoded = 0;
volatile long encoderValue;
volatile int lastEncoded2 = 0;
volatile long encoderValue2;
volatile long encoderNewValue;
volatile long encoderNewValue2;

char str[99];
char linear_val[15];
char ang_val[15];
char leftEnc[15];
char rightEnc[15];
char rateEncoder[15];
char baseDist[15];

//ROS receive cmd_vel data subscriber
void messageCb( const geometry_msgs::Twist &cmd_msg){
angularVel = cmd_msg.angular.z;
linearVel = cmd_msg.linear.x;
pressedTime = millis();
}

std_msgs::String ROSData;
ros::Subscriber<geometry_msgs::
Twist> sub_vel("cmd_vel", &messageCb );
ros::Publisher ROSData_pub("ROSData", &ROSData);

//object node for class ModbusMaster
ModbusMaster node;
ModbusMaster node2; 

void preTransmission()            //Function for setting stste of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);             
  digitalWrite(MAX485_DE, 1);
  digitalWrite(MAX485_RE_NEG2, 1);             
  digitalWrite(MAX485_DE2, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
}


void setup() {

 //setup ros stuff
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.advertise(ROSData_pub);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE_NEG2, OUTPUT);
  pinMode(MAX485_DE2, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
  Serial.begin(57600);
  Serial1.begin(9600);
  Serial3.begin(9600);             //Default Baud Rate of motor as 115200


  node.begin(2, Serial1);            //Slave ID as 4, serialport 3
  node2.begin(4, Serial3);
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node2.preTransmission(preTransmission); //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);
  node2.postTransmission(postTransmission);
  node.writeSingleRegister(0x2032,3); // operating mode, velocity mode
  node.writeSingleRegister(0x2031,8); // control, enable motor
  node.writeSingleRegister(0x2005,1);
  node2.writeSingleRegister(0x2032,3); // operating mode, velocity mode
  node2.writeSingleRegister(0x2031,8); // control, enable motor
  node2.writeSingleRegister(0x2005,1);
  
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE); //left     //when i used encoderLB and encoderLA, does not interrupt 
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE); //left     
  attachInterrupt(digitalPinToInterrupt(encoderRB), updateEncoder2, CHANGE); //right                  //uno only has 2 interrupt pins 2 & 3
  attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoder2, CHANGE); //right                  //mega has 6 interrupt pins 2 & 3 & 21 & 20 & 19 & 18 
  

   
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Calculate velocity of each wheel
  leftVel = linearVel + (angularVel * baseHalf);
  rightVel = linearVel - (angularVel * baseHalf);
  //Calculate pwm to send to each motor
  leftRpm = leftVel * (60/distPerRev);
  rightRpm = rightVel * (60/distPerRev);
  //rpm_left = -20;
  //rpm_right = -20;

  
  node.writeSingleRegister(0x203A,leftRpm * -1); //target speed, rpm (negative)
  node2.writeSingleRegister(0x203A,rightRpm); //target speed, rpm

 //calculate Odom();
  String s = dtostrf(linearVel, 1, 5, linear_val); // float to string
  String s2 = dtostrf(angularVel, 1, 5, ang_val);
  String s3 = dtostrf(encoderValue, 1, 1, leftEnc);
  String s4 = dtostrf(encoderValue2, 1, 1, rightEnc);
  String s5 = dtostrf(rateEnc, 1, 5, rateEncoder); 
  String s6 = dtostrf(baseDistance, 1, 5, baseDist);

  strcpy (str,linear_val); 
  strcat (str,", ");
  strcat (str,ang_val);
  strcat (str,", ");
  strcat (str,leftEnc);
  strcat (str,", ");
  strcat (str,rightEnc);
  strcat (str,", ");
  strcat (str,rateEncoder);
  strcat (str,", ");
  strcat (str,baseDist);
  puts (str);

  ROSData.data = str;
  ROSData_pub.publish(&ROSData);
  nh.spinOnce();
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
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue2= encoderValue2+1;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue2= encoderValue2-1;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded2 = encoded2; //store this value for next time
}


void calOdom(){
  //calculate encoder increments
  encoderNewValue = encoderValue;
  encoderNewValue2 = encoderValue2;
  leftEncInc = encoderNewValue - leftEncOld;
  rightEncInc = encoderNewValue2 - rightEncOld; 
  //calculate odom
  lengthError = (rightEncInc - leftEncInc) * rateEnc;
  ang_z_error = lengthError/(baseDistance);
  ang_z = ang_z + ang_z_error;

  delta_d = ((leftEncInc + rightEncInc)/2) * rateEnc;
  Xw = Xw + (delta_d * cos(ang_z));
  Yw = Yw + (delta_d * sin(ang_z));
  //reset encoder values
  leftEncOld = encoderValue;
  rightEncOld = encoderValue2;

}
