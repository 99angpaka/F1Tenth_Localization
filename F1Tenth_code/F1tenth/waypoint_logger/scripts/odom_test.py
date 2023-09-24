#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math

def hook():
    rospy.loginfo("shutdown")

class Odometry_Test:
    def __init__(self):
        rospy.init_node('odometry_test', anonymous=True)
        rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback)
        self.origin_x = None
        self.origin_y = None
        
        self.ctrl_cmd_pub = rospy.Publisher('/vesc/test_cmd', AckermannDriveStamped, queue_size=0)
        self.ctrl_cmd_msg = AckermannDriveStamped()

    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def odom_callback(self, msg):

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.origin_x is not None:
            distance = self.calculate_distance(self.origin_x, self.origin_y, current_x, current_y)

            if distance >= 2 :
                self.ctrl_cmd_msg.header.stamp = rospy.Time.now()
                self.ctrl_cmd_msg.drive.speed = 0.0
                self.ctrl_cmd_msg.drive.steering_angle = 0.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                rospy.loginfo("x : %f, y : %f", current_x, current_y)
                rospy.loginfo("Distance : %f", distance)
                assert False
            
            else :
                self.ctrl_cmd_msg.header.stamp = rospy.Time.now()
                self.ctrl_cmd_msg.drive.speed = 0.5
                self.ctrl_cmd_msg.drive.steering_angle = 0.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

        else: 
            self.origin_x = current_x
            self.origin_y = current_y
            rospy.loginfo("x : %f, y : %f", self.origin_x, self.origin_y)


if __name__ == "__main__":
    try:
        odom_test = Odometry_Test()
        # odom_test.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
