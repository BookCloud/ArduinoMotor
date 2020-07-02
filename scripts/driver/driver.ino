#include <ModbusMaster.h>               //Library for using ModbusMaster
#include <ros.h>
#include <geometry_msgs/Twist.h>


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


#define MAX485_DE      3
#define MAX485_RE_NEG  2
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

const int encoderLB = 18;
const int encoderLA = 19;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

boolean newData = false;
String spd;
int pos;
int in =0;
ModbusMaster node;                    //object node for class ModbusMaster
ModbusMaster node2; 
void preTransmission()            //Function for setting stste of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);             
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist>sub_vel("cmd_vel", &messageCB);

void messageCb( const geometry_msgs::Twist &cmd_msg) {
angular_vel=cmd_msg.angular.z;
linear_vel=cmd_msg.linear.x;
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.begin(9600);
  
  Serial3.begin(115200);             //Baud Rate as 115200

  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE);
 

  node.begin(4, Serial3);            //Slave ID as 4, serialport 3
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);

  
  node.writeSingleRegister(0x2032,3); // operating mode, velocity mode
  node.writeSingleRegister(0x2031,8); // control, enable motor


  nh.initNode();
  nh.subscribe(sub_vel);

}


void loop()
{
  Vleft = linear_vel + angular_vel * dbase;
  Vright = linear_vel - angular_vel * dbase;
  //Calculate pwm to send to each motor
  freq_left = ppr * Vleft /(2*PI*wheelR);
  freq_right = ppr * Vright /(2*PI*wheelR);

  node.writeSingleRegister(0x203A,Vleft); //target speed, rpm
    //recvWithEndMarker();
    //showNewData();

   //uint16_t result;
   //result = node.readHoldingRegisters(0x200D,1);
   //Serial.println(result);
  // Serial.println(encoderValue);//node.getResponseBuffer(0));
   //Serial.println(node.getResponseBuffer(0));
  

   //if (encoderValue >= 16384){

   // node.writeSingleRegister(0x203A,0);
   //}

   
 //  delay(500);
   
    


  //delay(100);
  //node.writeSingleRegister(0x2031,7); //control, stop
  //delay(500);//Writes value to 0x40000 holding register
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
