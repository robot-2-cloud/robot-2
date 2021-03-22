#!/usr/bin/env python
import rospy
from math import cos
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


# TODO: import ROS msg types and libraries
#Team 2, Nick Catanzaro, Walker Finlay, 3/12/2021
class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.brakebool_pub = rospy.Publisher('brake_bool', Bool, queue_size=1)
        self.brake_pub = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.speed = 0
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ads_drive_msg = AckermannDriveStamped()
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        processed = [x for x in scan_msg.ranges if (x < range_max and x > range_min)]

        #previous_second = scan_msg.secs
        idx = int(len(scan_msg.ranges)/2)
        #calculate phi
        phi = scan_msg.angle_min + idx*scan_msg.angle_increment

        #calculate r
        r = scan_msg.ranges[idx]

        #calculate v
        v = self.speed #since self.speed is set in odom_callback

        #calculate TTC
        try:
            TTC = r /(v * cos(phi))
            if abs(TTC) < .6:
                ads_drive_msg.drive.speed = 0
                self.brakebool_pub.publish(True)
            #print(TTC)
        except ZeroDivisionError:
            pass

        self.brake_pub.publish(ads_drive_msg)        







        # TODO: publish brake message and publish controller bool


def main():
    rospy.init_node('safety_node')
    
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()