#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ModbusMaster.h>  //Library for using ModbusMaster

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
volatile long leftEncInc;
volatile long rightEncInc;
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

char base_link[] = "/base_link";
char odom[] = "/odom";
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


//ROS receive cmd_vel data subscriber
void messageCb( const geometry_msgs::Twist &cmd_msg){
angularVel = cmd_msg.angular.z;
linearVel = cmd_msg.linear.x;
pressedTime = millis();
}


ros::NodeHandle nh; //start ros node
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb );
std_msgs::Float32 leftEncRos;
ros::Publisher leftEncPub("encL", &leftEncRos);
std_msgs::Float32 rightEncRos;
ros::Publisher rightEncPub("encR", &rightEncRos);
nav_msgs::Odometry odomData;
ros::Publisher odomPub("odom", &odomData);

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
  Serial.begin(57600);
  Serial1.begin(9600);
  Serial3.begin(9600);             //Default Baud Rate of motor as 115200

  //setup ros stuff
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.advertise(rightEncPub);
  nh.advertise(leftEncPub);
  nh.advertise(odomPub);
  broadcaster.init(nh);

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
  
  currentTime = millis();
  if((currentTime - pressedTime) >= 500){
    angularVel = 0;
    linearVel = 0;
  }
  node.writeSingleRegister(0x203A,leftRpm); //target speed, rpm (negative)
  node2.writeSingleRegister(0x203A,rightRpm * -1); //target speed, rpm


  calOdom();

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

  //convert angle to quaternion
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(ang_z);
  //publish transform over tf
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = Xw;
  t.transform.translation.y = Yw;
  t.transform.rotation = odom_quat;
  t.header.stamp = nh.now();
  //send transform
  broadcaster.sendTransform(t);

  //publish odom message
  odomData.header.stamp = nh.now();
  odomData.header.frame_id = odom;

  //set the position
  odomData.pose.pose.position.x = Xw;
  odomData.pose.pose.position.y = Yw;
  odomData.pose.pose.position.z = 0.0;
  odomData.pose.pose.orientation = odom_quat;

  //set the velocity
  odomData.child_frame_id = "base_link";
  odomData.twist.twist.linear.x = linearVel;
  odomData.twist.twist.linear.y = 0;
  odomData.twist.twist.angular.z = angularVel;

  //publish the message
  odomPub.publish( &odomData);

  rightEncRos.data = 420;
  rightEncPub.publish( &rightEncRos);
  leftEncRos.data = 69;
  leftEncPub.publish( &leftEncRos);

}
