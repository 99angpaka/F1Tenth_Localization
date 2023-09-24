#-*- coding:utf-8 -*-
#!/usr/bin/env python
import sys
import math
import numpy as np  

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, BrakeActiveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

#TO DO: Tune parameters
#PARAMS 
VELOCITY = 2.0 # meters per second
MIN_ACC = 1.0
TIME_THRESHOLD = 1.0

class EmergencyStop:
    def __init__(self):
        #Topics & Subs, Pubs
        self.brake = False
        self.stop_signal = 0
        self.velocity = None
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.velo_callback)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)#: Subscribe to LIDAR
        # self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)#: Publish to drive
        self.drive_pub = rospy.Publisher('/reactive/emergency/ackermann_cmd', AckermannDriveStamped, queue_size=10)
        self.active_pub = rospy.Publisher('/reactive/emergency/active', BrakeActiveStamped, queue_size=1)


    def velo_callback(self,data):
        #TO DO: Subscribe Current Velocity
        self.velocity = data.twist.twist.linear.x 

    def lidar_callback(self, lidar):

        if self.velocity is not None and self.brake == False:
            lidar_size = len(lidar.ranges) 
            lidar_step = int(np.deg2rad(5) / lidar.angle_increment)

            start = (lidar_size//2)-lidar_step
            end = (lidar_size//2)+lidar_step
            target_ranges = lidar.ranges[start:end]

            #np.median
            median_target = np.median(target_ranges)
            
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.speed = 1

            active_msg = BrakeActiveStamped()
            active_msg.header.stamp = rospy.Time.now()
            active_msg.header.frame_id = "laser"
            active_msg.active = False


            if (median_target < 0.5):  
                print(median_target)
                drive_msg.drive.speed = 0
                active_msg.active = True
                brake = True
                print("braking activate!")

            self.drive_pub.publish(drive_msg)
            self.active_pub.publish(active_msg)
        #TO DO: 1. Subscribe LiDAR data 
        #       2. Calculate minimum distance 
        #       2. Calculate Time to Collision(TTC) based on current velocity
        #       3. Publish drive.speed. (If TTC is smaller than minimum time, stop the car)
    
def main(args):
    rospy.init_node("Emergengy_Stop", anonymous=False)
    ES = EmergencyStop()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)