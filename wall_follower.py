#!/usr/bin/env python

from __future__ import print_function
import rospy
import tf
from std_msgs.msg import String, Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, LaserScan #import laserscan msg
import math
import numpy as np
import sys
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

kp = 0.6
kd = 0
ki = 0
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integ = 0.0

ANGLE_RANGE = 270
Dr = 0.35
VELOCITY = 2.00
CAR_LENGTH = 0.5

class WallFollow:
    def __init__(self):
        lidar_scan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        
        rospy.Subscriber('scan', LaserScan, self.lider_callback)

        self.drive_pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=10)
        

    def getRange(self, data, angle):
        index = (angle+45)*3+135
        return data.ranges[index]

        ranges = data.ranges
        angles = data.angles

    def followLeft(self, data, leftDist):
        phi_b = 90
        phi_a = 30
        theta = 60
        a, b = self.getRange(data, phi_a), self.getRange(data, phi_b)
        alpha = math.atan((b-a*math.cos(theta))/(a*math.sin(theta)))
        L = VELOCITY*rate
        Dt = b*math.cos(alpha) - L*math.sin(alpha)
        err = Dr - Dt
        return err

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        integral = integral + ki*error*rate
        
        ut = kp*error +integral + kd*(error - prev_error)
        print(ut)

        if ut < 10:
            VELOCITY = 1.75
        
        elif ut >= 10 and ut <20:
            VELOCITY = 1.25

        else:
            VELOCITY = 0.7

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = ut
        drive_msg.drive.speed = VELOCITY
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        error = self.followLeft(dataa, Dr)
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = wall_follower()
    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
    main(sys.argv)




















