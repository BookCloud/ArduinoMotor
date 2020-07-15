#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh; //start ros node
nav_msgs::Odometry odomData;
ros::Publisher odomPub("odom", &odomData);

void setup() {
  // put your setup code here, to run once:
    nh.initNode();

  nh.advertise(odomPub);

}

void loop() {
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(0);

  // put your main code here, to run repeatedly:
  //publish odom message
  odomData.header.stamp = nh.now();
  odomData.header.frame_id = "odom";

  //set the position
  odomData.pose.pose.position.x = 0;
  odomData.pose.pose.position.y = 0;
  odomData.pose.pose.position.z = 0.0;
  odomData.pose.pose.orientation = odom_quat;

  //set the velocity
  odomData.child_frame_id = "base_link";
  odomData.twist.twist.linear.x = 0;
  odomData.twist.twist.linear.y = 0;
  odomData.twist.twist.angular.z = 0;

  //publish the message
  odomPub.publish( &odomData);
  nh.spinOnce();
  //delay(10);
}
