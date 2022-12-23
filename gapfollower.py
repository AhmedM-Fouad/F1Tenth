#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

steer_angle = 0
n = 5
disp_list = []
thresh = 6
VELOCITY = 2

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'


        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        global steer_angle
        global n
        global disp_list
        global thresh
        global VELOCITY

        
        ranges = data.ranges
        ranges = ranges[180:900]# decreasing the view to +90 to -90
        for i in ranges:#removing the bad values
            if ranges[i] > thresh:
                ranges[i]= thresh
            elif ranges[i] < 0:
                ranges[i] = 0
        
        proc_ranges = ranges
        return proc_ranges

    def find_gaps(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """ 
        threshold_difference = 2
        gaps_indeces = []
        window_size = 100
        gaps_selected = []
        j = 0
        k = 0

        for i in free_space_ranges:
            if abs(free_space_ranges[i]- free_space_ranges[i+1]) > threshold_difference:
                gaps_indeces[j] = i
                j = j+1

        for l in gaps_indeces:# l for large
            if abs(gaps_indeces[l] - gaps_indeces[l+1] >= window_size):
                gaps_selected[k] = l
                gaps_selected[k+1] = l + 1
                k = k+2


        return gaps_selected


    
    def find_best_angle(self, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        #gap_indeces = self.find_max_gap()
        #start_i = gap_indeces[0]
        #end_i = gap_indeces[1]
        m = 0
        angles = []
        midpoint = 0
        theta = 0
        min_angle = self.data.min_angle

        gaps_selected = self.find_gaps(ranges)

        for i in gaps_selected:
            midpoint = abs(gaps_selected[i] - gaps_selected[i+1])
            if midpoint < 360:
                theta = (360 - midpoint) * min_angle
                angles[m] = theta
                m = m+1
            else:
                theta = (n - 360) * min_angle
                angles[m] = theta
                m = m+1
        best_angle = min(angles)

        return best_angle
        

    def lidar_callback(self, data):

        proc_ranges = self.preprocess_lidar(self, data)
        gaps_selected = self.find_gaps(self, proc_ranges)
        

        if proc_ranges[360] < 2:
            steer_angle = self.find_best_angle(gaps_selected)
        else:
            steer_angle = 0
            


        


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = VELOCITY
        self.drive_pub.publish(drive_msg)
        
        
    	

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)