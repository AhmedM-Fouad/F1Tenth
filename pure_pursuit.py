#!/usr/bin/env python


#AHmed Fouad and Nathan McNece
#team 4 F1Tenth Competition



from __future__ import print_function
import sys
import math
import numpy as np
import rospy
import csv

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
#from geometry_msgs.msg import PoseStamped
#from sensor_msgs.msg import LaserScan - not really used here
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from os.path import expanduser
from numpy import linalg as linalg

class PurePursuit:
    def __init__(self):
        drive_topic = '/nav'
        #pose_topic = '/pose'
        odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry , self.pose_callback ,queue_size = 10)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.node = rospy.Publisher('/PurePursuit_node', Float32, queue_size=10)
        #since we only need to read the file once, we can just get the x-y coords from the file in the beginning 
        self.get_waypoints()

    def get_waypoints(self):
        home = expanduser('~')
        filepath=home+'/rcws/logs/wp.csv' 
        with open(filepath,'r') as csvfile:
            data = list(csv.reader(csvfile))
            waypoints_tmp=np.asarray(data)  #tansfer list to array
            waypoints_tmp2=waypoints_tmp.astype(np.float32) #transfer string to float 
            waypoints_xyw = waypoints_tmp2[:,0:3] #Only take x,y and w(angle) values
        #print("waypoints loaded")
        #print("ex pt (0,0):", waypoints_xyw[0,0])
        #print("5,2: ", waypoints_xyw[5,2])
        self.waypointsXYW = waypoints_xyw
        #return waypoints_xyw
        
    def get_pose(self,data):
        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2] 
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        #print("pose:", x,y,yaw)
        return np.asarray([x,y,yaw])

    def find_next_waypoint(self,waypoints,pose):
        distances = np.zeros(len(waypoints))
        lookoutDist = 0.5 # Lookout distance
        for i in range(len(distances)):
            distances[i] = np.sqrt((waypoints[i,0] - pose[0]) ** 2 + (waypoints[i,1] - pose[1]) ** 2)
        targetPoints = np.where( abs(distances - lookoutDist) < 0.3 )[0]
        for j in targetPoints:
            #vector from car to point
            vector1 = [(waypoints[j,0] - pose[0]) , (waypoints[j,1] - pose[1])]
            #vector of current direction of car
            #pose[2] is the yaw
            vector2 = [np.cos(pose[2]),np.sin(pose[2])]
            #find angle between two vectors
            angle = np.arctan2(linalg.norm(np.cross(vector1, vector2)),np.dot(vector1, vector2))
            if (abs(angle) < math.pi/2): #only use points in front of the car
                target = j
                break
        L_dist = distances[target]
        #print("goal index: ", target)
        #print("L distance: ", L_dist)
        return np.array([target,L_dist])

    def angle_control(self, L, waypoint_W, yaw):
        maxAngle = 0.4189
        curv = 2 * math.sin(waypoint_W - yaw) / L
        P = 0.4 # proportional control for angle
        angle = 2 * math.atan(P * curv)
        if (angle > maxAngle):
            return maxAngle
        elif (angle < -maxAngle):
            return -maxAngle
        else:
            return angle

    def velocity_control(self,angle):
        return 2.0

    def publishSpeedAndAngle(self, angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def pose_callback(self, data):
        #print("in pose callback")
        #print("data:",data.pose.pose.orientation.x)
        #print("waypoint [0,0]:",self.waypointsXYW[5,0])
        pose = self.get_pose(data)
        nextPointAndDist = self.find_next_waypoint(self.waypointsXYW, pose)
        targetIndex = int(nextPointAndDist[0])
        #print("Target Index:",targetIndex)
        L = nextPointAndDist[1]
        #goalPoint = self.transform_goal_point(nextPointAndDist,self.waypointsXYW, pose)
        angle = self.angle_control( L, self.waypointsXYW[targetIndex,2], pose[2])
        #print("angle:", angle)
        velocity = self.velocity_control(angle)
        self.publishSpeedAndAngle(angle,velocity)

def main(args):
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)