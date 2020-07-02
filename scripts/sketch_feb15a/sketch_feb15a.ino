char blueToothVal;           //value sent over via bluetooth
char lastValue;              //stores last state of device (on/off)


#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher led("led", &str_msg);

void setup()
{
 nh.initNode();
 nh.advertise(led);
 pinMode(13,OUTPUT);
 pinMode(12,INPUT);
}
 
 
void loop()
{
  if(Serial.available()>0)
  {//if there is data being recieved
    blueToothVal=Serial.read(); //read it
  }
  if (blueToothVal=='n')
  {//if value from bluetooth serial is n
    digitalWrite(13,HIGH);            //switch on LED
    if (lastValue!='n')
      Serial.println(F("LED is on")); //print LED is on
    lastValue=blueToothVal;
  }
  else if (blueToothVal=='f')
  {//if value from bluetooth serial is n
    digitalWrite(13,LOW);             //turn off LED
    if (lastValue!='f')
      Serial.println(F("LED is off")); //print LED is on
    lastValue=blueToothVal;
  }

int state = digitalRead(12);
  if(state==HIGH){
  str_msg.data = 1;
  led.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
  }
  delay(1000);
}
