#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point

# home = expanduser('~')


class WaypointLogger:

    def __init__(self):
        rospy.init_node('waypoints_logger')
        rospy.Subscriber('/vesc/odom', Odometry, self.save_waypoint)
        #Slam version
        #rospy.Subscriber('', Odometry, self.save_waypoint)
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=0)

        self.marker = Marker()
        self.file = open(strftime('%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        self.prev_point_x = None
        self.prev_point_y = None

    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                            data.pose.pose.orientation.y, 
                            data.pose.pose.orientation.z, 
                            data.pose.pose.orientation.w])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                data.twist.twist.linear.y, 
                                data.twist.twist.linear.z]),2)

        self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        speed))

        current_point_x = data.pose.pose.position.x
        current_point_y = data.pose.pose.position.y

        if self.prev_point_x is not None:
            distance = self.calculate_distance(self.prev_point_x, self.prev_point_y, current_point_x, current_point_y)

            if distance > 0.1:
                self.marker.header.frame_id = "laser"
                self.marker.header.stamp = rospy.Time.now()
                self.marker.type = Marker.POINTS
                self.marker.action = Marker.ADD
                self.marker.pose.orientation.w = 1.0
                point = Point()
                point.x = data.pose.pose.position.x
                point.y = data.pose.pose.position.y
                point.z = 0.0
                self.marker.points.append(point)
                self.marker.scale.x = 0.2
                self.marker.scale.y = 0.2
                self.marker.scale.z = 0.0
                self.marker.color.a = 1.0
                self.marker.color.r = 1.0
                self.marker.color.g = 1.0
                self.marker.color.b = 0.0
                print(point)

                self.marker_pub.publish(self.marker)

                self.prev_point_x = current_point_x
                self.prev_point_y = current_point_y

            else:
                pass

        else :
            self.prev_point_x = current_point_x
            self.prev_point_y = current_point_y           

    def shutdown():
        file.close()
        print('Goodbye')

if __name__ == '__main__':
    print('Saving waypoints...')
    try:
        waypointlogger_node = WaypointLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        waypointlogger_node.shutdown()