#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void cmdvel_cb( const geometry_msgs::Twist& cmd_msg){
  serial.println(cmd_msg);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdvel_cb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
