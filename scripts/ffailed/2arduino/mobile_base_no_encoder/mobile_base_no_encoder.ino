#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ModbusMaster.h>  //Library for using ModbusMaster
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh; //start ros node

//Hardware measurements
const float baseDistance = 0.3; //distance between wheels in metres
const float baseHalf = baseDistance / 2;
const float wheelDiameter = 0.165; //diameter of wheel
const float wheelRadius = wheelDiameter / 2;
const float distPerRev = wheelDiameter*PI; //dist per rev of wheel
const int encResolution = 4096; //resolution of encoder
const float sumEnc = encResolution * 4; //total pulses per rev of encoder
const float rateEnc = (2 * PI*wheelRadius) / sumEnc;

float angularVel = 0.0;
float linearVel = 0.0;
float rightVel = 0.0; //Velocity of right wheel
float leftVel = 0.0; //Velocity of left wheel
float rightRpm = 0.0;
float leftRpm = 0.0;

const int stopInterval = 500; //Time taken for motors to stop when no more cmd_vel is received
unsigned long pressedTime;  //Time when teleop is pressed

//Motor variables , Common the DE and RE_NEG of BOTH motors
#define MAX485_DE      5                 //put for RSE pin
#define MAX485_RE_NEG  4
#define MAX485_DE2      6                //put for RSE pin
#define MAX485_RE_NEG2  7
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
float result;
float result2;



// Initialise variables to send data to ROS as string
char str[99];
char linear_val[15];  //linear velocity from cmd_vel topic
char ang_val[15];     //angular velocity from cmd_vel topic
char leftEnc[15];     //left encoder count
char rightEnc[15];    //right encoder count
char rateEncoder[15]; //rate endcoder
char baseDist[15];    //distance between the wheels 

//ROS receive cmd_vel data subscriber
void messageCb( const geometry_msgs::Twist &cmd_msg) {
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


//Variables for communication to Nano
char nanoData[32];  //store encoder data from nano
char *encoderArray[99]; //split data to left and right encoder
char *ptr = NULL;
unsigned long encoderTime = 0;  //time when retrieving data
unsigned long encoderInterval = 50; //interval of retrievals (ms)

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
  nh.subscribe(sub_vel);  //subscribe to cmd_vel topic
  nh.advertise(ROSData_pub);  //publishes data for odometry

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  //pinMode(MAX485_RE_NEG2, OUTPUT);
  //pinMode(MAX485_DE2, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
  Serial.begin(57600);
  
  Serial1.begin(9600);
  Serial3.begin(9600);             //Default Baud Rate of motor as 115200

  Serial2.begin(57600); //to communicate with encoder arduino
  
  node.begin(2, Serial3);            //Slave ID as 2, serialport 3
  node2.begin(4, Serial1);          //Slave ID as 4, serialport 1
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node2.preTransmission(preTransmission); //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);
  node2.postTransmission(postTransmission);
  node.writeSingleRegister(0x2032, 3); // operating mode, velocity mode
  node.writeSingleRegister(0x2031, 8); // control, enable motor
  node.writeSingleRegister(0x2005, 1);
  node2.writeSingleRegister(0x2032, 3); // operating mode, velocity mode
  node2.writeSingleRegister(0x2031, 8); // control, enable motor
  node2.writeSingleRegister(0x2005, 1);


}

void loop() {
  //Calculate velocity of each wheel
  leftVel = linearVel + (angularVel * baseHalf);
  rightVel = linearVel - (angularVel * baseHalf);
  //Calculate pwm to send to each motor
  leftRpm = leftVel * (60 / distPerRev);
  rightRpm = rightVel * (60 / distPerRev);

  node.writeSingleRegister(0x203A, leftRpm * -1); //target speed, rpm (negative) //motor is reversed physically
  node2.writeSingleRegister(0x203A, rightRpm); //target speed, rpm



  // stops motor if no commands are received
  if ((millis() - pressedTime) >= stopInterval){
    angularVel = 0;
    linearVel = 0;
  }

  if(millis() - encoderTime >= encoderInterval){
  //Serial2.write(1);
  sendOdom();
  encoderTime = millis();}



  nh.spinOnce();  
}

  


//code to recieve encoder counts from Nano
void serialEvent2(){
  
  Serial2.readBytes(nanoData, 30);

  byte index = 0;
  ptr = strtok(nanoData, ",");  // takes a list of delimiters
  while(ptr != NULL)
  {
      encoderArray[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");  // takes a list of delimiters
  }
    
    String(encoderArray[0]).toCharArray(leftEnc, 15);
    String(encoderArray[1]).toCharArray(rightEnc, 15);
   // Serial.println(nanoData);
    sendOdom();
}


void sendOdom(){
    //send values required to calculate odom to ros as string
  String s = dtostrf(linearVel, 1, 5, linear_val); // float to string
  String s2 = dtostrf(angularVel, 1, 5, ang_val);
  String s5 = dtostrf(rateEnc, 1, 5, rateEncoder);
  String s6 = dtostrf(baseDistance, 1, 5, baseDist);

 
  strcpy (str, linear_val);
  strcat (str, ", ");
  /*strcat (str, ang_val);
  strcat (str, ", ");
  
  strcat (str, leftEnc);
  strcat (str, ", ");
  strcat (str, rightEnc);
  strcat (str, ", ");
  
  strcat (str, rateEncoder);
  strcat (str, ", ");
  strcat (str, baseDist);*/
  puts (str);

  
  Serial.println(str);
  ROSData.data = str;
  ROSData_pub.publish(&ROSData);
}
