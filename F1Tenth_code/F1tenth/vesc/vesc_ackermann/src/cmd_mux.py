#!/usr/bin/env python
import sys
import rospy
#from ackermann_cmd_mux.msg import AckermannDriveStamped, AckermannDrive
from ackermann_msgs.msg import BrakeActiveStamped,  AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy

class ackermann_cmd_mux :
    def __init__(self):
        self.MODE = None # TELEMODE, AUTOMODE, EMERGENCYMODE

        # Subscribers and publishers
        self.default_sub = rospy.Subscriber('ackermann_cmd_default', AckermannDriveStamped, self.defaultinput_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.teleop_sub = rospy.Subscriber('ackermann_cmd_teleop', AckermannDriveStamped, self.teleop_callback)
        self.emergency_sub = rospy.Subscriber('/reactive/emergency/ackermann_cmd', AckermannDriveStamped, self.emergency_callback)
        self.emergemcy_active_sub = rospy.Subscriber('/reactive/emergency/active', BrakeActiveStamped, self.emergency_active_callback)
        # self.test_sub = rospy.Subscriber('/vesc/test_cmd', AckermannDriveStamped, self.gap_callback)

        # self.gap_sub = rospy.Subscriber('/reactive/gapfollower/ackermann_cmd', AckermannDriveStamped, self.gap_callback)
        self.wall_sub = rospy.Subscriber('/reactive/wallfollower/ackermann_cmd', AckermannDriveStamped, self.wall_callback)
        # self.waypoint_sub = rospy.Subscriber('/waypoint_follower/ackermann_cmd', AckermannDriveStamped, self.waypoint_callback)

        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)

    def set_mode(self, mode):
        if(self.MODE!= mode):
            self.MODE = mode
            rospy.loginfo("MODE : %s", self.MODE)

    def defaultinput_callback(self, data):
        if self.MODE == None :
            self.drive_pub.publish(data)

    def joy_callback(self, data):
        try :
            if(data.buttons[4] and not data.buttons[6]) :
                self.set_mode("TELEMODE")
            elif(data.buttons[6]) :
                self.set_mode("AUTOMODE")
        except JoyTeleopException as e:
            rospy.logerr(e)
            return

    def emergency_active_callback(self, data):
        if data.active == True and self.MODE != "TELEMODE":
            self.set_mode("EMERGENCYMODE")
            

    def teleop_callback(self, data):
        if self.MODE == "TELEMODE" :
            self.drive_pub.publish(data)

    def emergency_callback(self, data):
        if self.MODE == "EMERGENCYMODE" :
            self.drive_pub.publish(data)

    def wall_callback(self, data):
        if self.MODE == "AUTOMODE" :
            self.drive_pub.publish(data)

def main(args):
    rospy.init_node("ackermann_cmd_mux")
    acm = ackermann_cmd_mux()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)