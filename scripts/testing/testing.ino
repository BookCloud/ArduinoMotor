#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ModbusMaster.h>  //Library for using ModbusMaster

ros::NodeHandle nh; //start ros node

//Hardware measurements
const float Dbase = 0.3; //distance between wheels in metres
const float dbase = Dbase/2;
const float wheelD = 0.165; //diameter of wheel
const float wheelR = wheelD/2;
const float dpr = 0.5307; //dist per rev of wheel
const int sum_enc = 4096; //total pulses per rev of encoder
const float rate_enc = (2*PI*wheelR) / sum_enc;

//float angular_vel = 0.0;
//float linear_vel = 0.0;
float Vright = 0.0; //Velocity of right wheel
float Vleft = 0.0; //Velocity of left wheel
float rpm_left = 0;
float rpm_right = 0;

//Motor variables , Common the DE and RE_NEG of BOTH motors
#define MAX485_DE      3                 //put for RSE pin
#define MAX485_RE_NEG  2
#define MAX485_DE2      7                //put for RSE pin
#define MAX485_RE_NEG2  6
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
uint16_t result;
uint16_t result2;

//Odom variables
float inc_encL;
float enc_newL = 0;
float enc_oldL = 0;
float inc_encR;
float enc_newR = 0;
float enc_oldR = 0;
float length_error;

float ang_z = 0.0;
float ang_z_error;
float Xw = 0.0;
float Yw = 0.0;
float delta_d;


ModbusMaster node;                    //object node for class ModbusMaster
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

  cli();//stop interrupts

//set timer4 interrupt at 1Hz
 TCCR4A = 0;// set entire TCCR1A register to 0
 TCCR4B = 0;// same for TCCR1B
 TCNT4  = 0;//initialize counter value to 0
 // set compare match register for 1hz increments
 OCR4A = 15625 ;// = (16*10^6) / (1*1024) - 1 (must be <65536)
 // turn on CTC mode
 TCCR4B |= (1 << WGM12);
 // Set CS12 and CS10 bits for 1024 prescaler
 TCCR4B |= (1 << CS12) | (1 << CS10);  
 // enable timer compare interrupt
 TIMSK4 |= (1 << OCIE4A);

  sei();//allow interrupts
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE_NEG2, OUTPUT);
  pinMode(MAX485_DE2, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
  Serial.begin(9600);
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
  


}



ISR(TIMER4_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)


  Left_Wheel_enc();
  Right_Wheel_enc();
  Calculate_Odom();
}

unsigned long timeElapsed;
unsigned long oldTime = 0;
float angular_vel = 0.0;
float linear_vel = 1.0;

void loop() {
  //Calculate velocity of each wheel
  Vleft = linear_vel + (angular_vel * dbase);
  Vright = linear_vel - (angular_vel * dbase);
  //Calculate pwm to send to each motor
  rpm_left = Vleft * (60/dpr);
  rpm_right = Vright * (60/dpr);

  node.writeSingleRegister(0x203A,rpm_right); //target speed, rpm (negative)
  node2.writeSingleRegister(0x203A,rpm_left*-1); //target speed, rpm

  result = node.readHoldingRegisters(0x202A,2);
  result2 = node2.readHoldingRegisters(0x202A,2);

  Serial.println("Left encoder value: ",inc_encL);
  Serial.println("Right encoderer value: "inc_encR);
  Serial.println("Xw encoderer value: ",Xw);
  Serial.println("Yw encoderer value: ",Yw);
  
  

  unsigned long currentTime = millis();
  timeElapsed = currentTime - oldTime;
  if(timeElapsed >= 1000)
  { 
    linear_vel = 0;
    angular_vel = 0;
  }
  
  
  
}



int overflow_count_L;
int count_newticks_L;
int count_oldticks_L;
int difference_L;

void Left_Wheel_enc()
{
//Reads the register 0 to determine the changes in direction
overflow_count_L = 65535 - node2.getResponseBuffer(0);
//Reads the register 1 to determine the change in distance travelled
enc_newL = node2.getResponseBuffer(1);

//Left
  count_newticks_L = overflow_count_L;
  difference_L = count_newticks_L - count_oldticks_L;
  count_oldticks_L = count_newticks_L;

//Moving forward
  if (difference_L > 0 || difference_L == (-65535))
  {
    inc_encL = enc_newL - enc_oldR + 65535;  //add 65535 to increase the distance count
  }

//Moving reverse
  else if (difference_L < 0 || difference_L == 65535)
  {
    inc_encL = enc_newL - enc_oldL - 65535;  //minus 65535 to decrease the distance count
  }
//Stationary Position condition
  else
  {
    inc_encL = enc_newL - enc_oldL;
  }
//Puts back the new count in to the old
enc_oldL = enc_newL;
}


int overflow_count_R;
int count_newticks_R;
int count_oldticks_R;
int difference_R;

void Right_Wheel_enc()
{
overflow_count_R = node.getResponseBuffer(0);
enc_newR = node.getResponseBuffer(1);

//Right
  count_newticks_R = overflow_count_R;
  difference_R = count_newticks_R - count_oldticks_R;
  count_oldticks_R = count_newticks_R;


//moving forward
  if (difference_R > 0 || difference_R == (-65535))
  { 
    inc_encR = enc_newR - enc_oldR;
  }

//moving reverse
  else if (difference_R < 0 || difference_R == 65535)
  {
    inc_encR = enc_newR - enc_oldR;  
  }
  else
  {
    inc_encR = enc_newR - enc_oldR;
  }
  enc_oldR = enc_newR; 

}

void Calculate_Odom()
{

   length_error = (inc_encR - inc_encL) * rate_enc;

   ang_z_error = length_error/(Dbase);
   ang_z = ang_z + ang_z_error;

   delta_d = ((inc_encL + inc_encR)/2) * rate_enc;

   Xw = Xw + (delta_d * cos(ang_z));
   Yw = Yw + (delta_d * sin(ang_z));

}
