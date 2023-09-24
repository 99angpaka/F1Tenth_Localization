#!/usr/bin/env python
import sys
import math
import numpy as np  
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry 

class ReactiveGapFollower:

    def __init__(self):
        lidar_topic = '/scan'
        drive_topic = '/reactive/gapfollower/ackermann_cmd'

        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=5)


    def lidar_callback(self, data):
    #  Preprocess the LiDAR scan array. Expert implementation includes:
    #  1.Setting each value to the mean over some window
    #  2.Rejecting high values (eg. > 3m)

        ranges = np.array(data.ranges)
        min_angle = -70 / 180 * math.pi
        max_angle = 70 / 180 * math.pi
        min_idx = abs(int((min_angle - data.angle_min) / data.angle_increment))
        max_idx = abs(int((max_angle - data.angle_min) / data.angle_increment))

        for i in range(min_idx, max_idx+1):
            if np.isnan(ranges[i]) or np.isinf(ranges[i]):
                ranges[i]= 0.0
            elif ranges[i] > data.range_max:
                ranges[i] = data.range_max


        closet_idx = min_idx
        closet_distance = data.range_max * 5

        for i in range(min_idx, max_idx+1):
            distance = ranges[i-2] + ranges[i-1] + ranges[i] + ranges[i+1] + ranges[i+2]
            if distance < closet_distance:
                closet_distance = distance
                closet_idx = i
        
        # rospy.loginfo("closet idx : %d ", closet_idx)

        radius = 100

        for i in range(closet_idx - radius, closet_idx + radius+1):
            ranges[i] = 0.0

        start = min_idx
        end = min_idx
        current_start = min_idx - 1
        duration = 0
        max_duration = 0

        for i in range(min_idx, max_idx+1):
            if current_start < min_idx :
                if ranges[i] > 0.0 :
                    current_start = i

            elif ranges[i] <= 0.0 :
                duration = i - current_start
                if duration > max_duration :
                    start = current_start
                    end = i-1
                    max_duration = duration
                
                current_start = min_idx - 1

        if current_start >= min_idx :
            duration = max_idx - current_start + 1
            
            if duration > max_duration :
                start = current_start
                end = max_idx
                max_duration = duration

        rospy.loginfo("start: %d, end: %d, duration: %d", start, end, duration)     
        rospy.loginfo("angle_min %f, angle_max : %f", data.angle_min, data.angle_max)
        rospy.loginfo(data.angle_increment)


        current_max = 0.0
        angle = 0.0
        
        for i in range(start, end+1):
            if ranges[i] > current_max :
                current_max = ranges[i]
                angle = data.angle_min + i * data.angle_increment
            elif ranges[i] == current_max :
                if(abs(data.angle_min + i * data.angle_increment) < abs(angle)):
                    angle = data.angle_min + i * data.angle_increment

        # max_angle = 0.34

        # if(abs(angle) > max_angle):
        #     angle = max_angle * angle / abs(angle)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle

        if abs(angle) > 0.3 :
            drive_msg.drive.speed = 0.3
        elif abs(angle) > 0.2 :
            drive_msg.drive.speed = 0.4
        else :
            drive_msg.drive.speed = 0.5

        self.drive_pub.publish(drive_msg)
        rospy.loginfo("angle: %f, speed: %f\n", angle, drive_msg.drive.speed)


def main(args):
    rospy.init_node("Gap_Follower")
    GF = ReactiveGapFollower()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)