#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String

class Odometry_Publisher:
    def __init__(self):
        self.arduino_sub = rospy.Subscriber("/ROSData", String, self.arduino_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.leftEncOld = 0
        self.rightEncOld = 0
        self.last_time = rospy.Time.now()

        rospy.Rate(1).sleep()
        rospy.spin()
  
    def arduino_callback(self, data):
        arduino_data = data.data.split(",")
        #calculate odom with arduino data
	linear_vel = float(arduino_data[0])
	angular_vel = float(arduino_data[1])
        leftEnc = float(arduino_data[2])
        rightEnc = float(arduino_data[3])
        rateEnc = float(arduino_data[4])
        baseDistance = float(arduino_data[5])

        #imuData = data[4]
        
        
        leftEncInc = leftEnc - self.leftEncOld
        rightEncInc = rightEnc - self.rightEncOld
        length_error = (rightEncInc - leftEncInc) * rateEnc
        delta_th = length_error/baseDistance
        self.th += delta_th

        delta_d = (leftEncInc + rightEncInc)/2 * rateEnc
        delta_x = delta_d * math.cos(self.th)
        delta_y = delta_d * math.cos(self.th)
        self.x += delta_x
        self.y += delta_y

        self.leftEncOld = leftEnc
        self.rightEncOld = rightEnc

        self.current_time = rospy.Time.now()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"

        odom.twist.twist = Twist(Vector3(linear_vel, 0.0, 0.0), Vector3(0.0, 0.0, angular_vel))

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        print "printing"

if __name__ == "__main__":
    rospy.init_node("odometry_publisher")
    Odometry_Publisher()
    