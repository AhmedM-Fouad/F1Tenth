#!/usr/bin/env python

import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import rospy
from std_msgs.msg import String

class Safety(object):
    def __init__(self):
        self.speed= 0
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pubBrake = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.pubBrakeBool = rospy.Publisher('brake_bool', Bool, queue_size=10)
        rate = rospy.Rate(10)


    def odom_callback(self, Odom_msg):
        self.speed = Odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):

        speed = self.speed
        min_theta = scan_msg.angle_min
        theta_inc = scan_msg.angle_increment
        stop = False
        thresh = 3
        distances = scan_msg.ranges

        for i in range(len(distances)):
            if speed == 0:
                TTC = 10000
                #rospy.loginfo(TTC)
                #rospy.loginfo(stop)

            else:
                TTC = distances[i]/(speed*math.cos(min_theta + theta_inc*i))
                #rospy.loginfo(TTC)
                #rospy.loginfo(stop)


                if TTC < 3:
                    stop = True
                    stamp = AckermannDriveStamped()
                    stamp.drive.speed = 0
                    self.pubBrakeBool.publish(stop)
                    self.pubBrake.publish(stamp)
                    #rospy.loginfo(TTC)
                    #rospy.loginfo(stop)
                    

def main():
    rospy.init_node('Safety')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()