#!/usr/bin/env python3

# ros package
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class WallFollower:
    def __init__(self):
        # Topics & Subs, Pubs
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher('/reactive/wallfollower/ackermann_cmd', AckermannDriveStamped, queue_size = 10 )

        self.wall_distance = 1.0
        self.forward_speed = 2.0
        self.angular_speed = 1.0

    def scan_callback(self, scan_msg):
        if len(scan_msg.ranges) == 0:
            return

        drive_msg = AckermannDriveStamped()
        left_candidate = scan_msg.ranges[714:725]
        left_distance = sum(left_candidate) / len(left_candidate)

        right_candidate = scan_msg.ranges[354:365]
        right_distance = sum(right_candidate) / len(right_candidate)

        target_angle = math.pi / 4 - math.atan(right_distance/left_distance)

        if abs(target_angle) < 0.25:
            drive_msg.drive.steering_angle = target_angle
            drive_msg.drive.speed = 5.0

        else :
            drive_msg.drive.steering_angle = 0.34 * (target_angle/abs(target_angle))
            drive_msg.drive.speed = 2.0

        print("left : %f  right : %f  angle : %f", left_distance, right_distance, drive_msg.drive.steering_angle)
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    print("wall_follower Initialized")
    wf_node = WallFollower()
    rospy.spin()
