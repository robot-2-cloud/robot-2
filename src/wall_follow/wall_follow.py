#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np


#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.0
kd = 0.5
ki = 0.5
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.0 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ 
    Receives lidar data and publishes Ackermann drive data for left wall follow
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between egrees, where 0 degrees is directly ahead
        # Outputs length in meters to object with angle in lidar scan field of view
        idx = int((angle - data.angle_min) / data.angle_increment)
        while data.ranges[idx] > data.angle_max or data.ranges[idx] < data.angle_min:
            idx = idx + 1
        return data.ranges[idx]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        angle = -kp*error
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        phi_a = np.pi/3
        theta = np.pi/6
        
        a = self.getRange(data, phi_a)
        b = self.getRange(data, phi_a + theta)

        alpha = np.arctan((b - a*np.cos(theta)) / (a*np.sin(theta)))
        L = VELOCITY * 0.1
        D_next = b*np.cos(alpha) - L*np.sin(alpha)
        e_t = DESIRED_DISTANCE_LEFT - D_next
        return e_t

    def lidar_callback(self, data):
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
