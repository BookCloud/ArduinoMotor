#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Tone.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


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

char base_link[] = "/base_link";
char odom[] = "/odom";


//ROS receive cmd_vel data
void messageCb( const geometry_msgs::Twist &cmd_msg) {
angular_vel=cmd_msg.angular.z;
linear_vel=cmd_msg.linear.x;
unsigned long StartTime = millis();
}

ros::NodeHandle nh; //start ros node
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb );
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


void setup() {
  Serial.begin(57600);

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
  
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);
    
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoderR, RISING);
 
  leftm.begin(left_motor_pin);
  rightm.begin(right_motor_pin);
    
  nh.initNode();
  nh.subscribe(sub_vel);

}

ISR(TIMER4_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
   inc_encL = enc_newL - enc_oldL;
   inc_encR = enc_newR - enc_oldR;

   length_error = (inc_encR - inc_encL) * rate_enc;

   ang_z_error = length_error/(D);
   ang_z = ang_z + ang_z_error;

   delta_d = ((inc_encL + inc_encR)/2) * rate_enc;

   Xw = Xw + (delta_d * cos(ang_z));
   Yw = Yw + (delta_d * sin(ang_z));

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = Xw;
  t.transform.translation.y = Yw;
  
  t.transform.rotation = tf::createQuaternionFromYaw(ang_z);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  

   enc_oldL = enc_newL;
   enc_oldR = enc_newR;
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

  if(encoderValue >= 17000){
    leftm.stop();
    rightm.stop();
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
