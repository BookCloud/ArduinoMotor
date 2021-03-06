#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <ros/time.h>
#include <ModbusMaster.h>  //Library for using ModbusMaster
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

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

//object node for class ModbusMaster
ModbusMaster node;
ModbusMaster node2;


//Variables for communication to Nano
const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;
boolean start = false;


//ultrasonic sensor variables
char sonarFrameId[] = "/sonar_ranger";
unsigned long ultrasonicTimer = 0;
const int trigPin = 9;
const int echoPin = 10;
float sensorReading = 0;
int duration;


//setup ros publishers and subscribers
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb );

std_msgs::String ROSData;         //publish to ROSData
//std_msgs::Int32 ROSData;        //try out different message typ
//geometry_msgs::Point ROSData;        

ros::Publisher ROSData_pub("ROSData", &ROSData);

sensor_msgs::Range sonar_msg;     //publish to sonar
ros::Publisher pub_sonar( "sonar1" , &sonar_msg);


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
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_vel);  //subscribe to cmd_vel topic
  nh.advertise(ROSData_pub);  //publishes data for odometry
  //nh.advertise(pub_sonar); //publishes ultrasonic sensor data

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
  //Serial.begin(115200);
  Serial1.begin(9600);
  Serial3.begin(9600);             //Default Baud Rate of motor is 115200

  Serial2.begin(115200); //to communicate with encoder arduino
  
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

  //setup ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  sensor_msgs::Range::ULTRASOUND;
  sonar_msg.header.frame_id = sonarFrameId;
  sonar_msg.field_of_view = (10.0 / 180.0) * 3.14;
  sonar_msg.min_range = 0.0;
  sonar_msg.max_range = 10.0;

}


void loop() {
  

  movement();


  // stops motor if no commands are received
  if ((millis() - pressedTime) >= stopInterval){
    angularVel = 0;
    linearVel = 0;
  }

  //receive encoder data from nano and publish all data to calculate odom
  receiveEncoder();

  //periodically calculate and publish ultrasonic sensor value
  //ultraman();
  
  nh.spinOnce();

}

  
void movement(){
  //Calculate velocity of each wheel
  leftVel = linearVel + (angularVel * baseHalf);
  rightVel = linearVel - (angularVel * baseHalf);
  //Calculate pwm to send to each motor
  leftRpm = leftVel * (60 / distPerRev);
  rightRpm = rightVel * (60 / distPerRev);
  //send rpm to motor
  node.writeSingleRegister(0x203A, leftRpm * -1); //target speed, rpm (negative) //motor is reversed physically
  node2.writeSingleRegister(0x203A, rightRpm); //target speed, rpm
}


//code to recieve encoder counts from Nano
void receiveEncoder(){
  clearBuffer(); //clear serial buffer to receive the latest data
  recvBytesWithStartEndMarkers(); //receive data
  processNewData(); //process data
}


void clearBuffer(){
  //while something in serial buffer, read and throw away the data
  while(Serial2.available() > 0){
    Serial2.read();   //do nothing with it
  }
}


void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;
    
    //wait for newest data to arrive
    while(Serial2.available() <= 0){
      int wait = 0;  //do nothing here
    }
    
    // start reading
    while (Serial2.available() > 0 && newData == false) {
        rb = Serial2.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
}


void processNewData() {
  //  sort out the seperate bytes (HARDCODED!!!)  first 4bytes belong to the first integer sent by nano
  if(newData == true){
      int32_t x = constructData(receivedBytes[0], receivedBytes[1], receivedBytes[2], receivedBytes[3]);
      int32_t y = constructData(receivedBytes[4], receivedBytes[5], receivedBytes[6], receivedBytes[7]);
 
    sendOdom(x, y); //pass values to publish to ROS
    newData = false;
  }
}


//combine the four 8bits back into a 32bit integer
int32_t constructData(uint32_t byte1, uint32_t byte2, uint32_t byte3, uint32_t byte4){
  
  int32_t bit16 = (byte2 << 8) | byte1;
  int32_t bit24 = (byte3 << 16) | bit16;
  int32_t bit32 = (byte4 << 24) | bit24;
  return bit32;
}


void sendOdom(int32_t leftEncoderData, int32_t rightEncoderData){
  //send values required to calculate odom to ros as string
  String s = dtostrf(linearVel, 1, 5, linear_val); // float to string
  String s2 = dtostrf(angularVel, 1, 5, ang_val);
  String s3 = dtostrf(leftEncoderData, 1, 1, leftEnc);
  String s4 = dtostrf(rightEncoderData, 1, 1, rightEnc);
  String s5 = dtostrf(rateEnc, 1, 5, rateEncoder);
  String s6 = dtostrf(baseDistance, 1, 5, baseDist);

 
  strcpy (str, linear_val);
  strcat (str, ", ");
  strcat (str, ang_val);
  strcat (str, ", ");
  strcat (str, leftEnc);
  strcat (str, ", ");
  strcat (str, rightEnc);
  strcat (str, ", ");
  strcat (str, rateEncoder);
  strcat (str, ", ");
  strcat (str, baseDist);
  puts (str);
  
  ROSData.data = str;
  ROSData_pub.publish(&ROSData);
  nh.spinOnce();  

}

void ultraman(){
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  if ((millis() - ultrasonicTimer) > 50 )
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    duration = pulseIn(echoPin, HIGH);
    sensorReading = duration * 0.342 / 2000;
    //sensoReading = getDistance;
    sonar_msg.range = sensorReading;
    //Serial.println(sensoReading);
    sonar_msg.header.stamp = nh.now();
    pub_sonar.publish(&sonar_msg);

    //Serial.println(sensorReading);
    ultrasonicTimer = millis(); //publish once a second
  }
}
