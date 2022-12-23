#!/usr/bin/env python



from hashlib import new
from pyexpat.errors import XML_ERROR_DUPLICATE_ATTRIBUTE
from sre_constants import BRANCH
from threading import local
from time import thread_time
import numpy as np
from numpy import linalg as LA
import tf
import math
from __future__ import print_function
import sys
import csv
from os.path import expanduser
import random
import cv2

import rospy
from numpy import linalg as LA
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from tf import transform_listener
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
run_speed = 1
np.set_printoptions(threshold=np.inf)
try_t = 1000
B_length = 0.7
R_is_goal = 1
d_thresh = 1
P_arc = 0.5
expend_size = 10
map_res = 0.05
map_origin = None # put the map origin from the yaml file of Oscherlesben
map_center = [-map_origin[0]/map_res, -map_origin[1]/map_res]
scan_res = 10
local_map_size_half = [256,256]
raw_map = cv2.imread('Oscherlesben.pgm')
kernel = np.ones((expend_size*2+1, expend_size*2+1), np.uint8)
gray_map = cv2.cvtcolor(raw_map, cv2.COLOR_BGR2_GRAY)
ret, binary_map = cv2.threshold(gray_map, 127, cv2.THRESH_BINARY_INV)
large_map = cv2.dilate(binary_map, kernel, iterations = 1)

MAX_ITER = 1200
MIN_ITER = 1000
X_SAMPLE_RANGE = 3
Y_SAMPLE_RANGE = 3
STD = 1.5
GOAL_THRESHOLD = 0.15
STEER_RANGE = 0.3
#SCAN_RANGE = 3.0
GOAL_AHEAD_DIST = 3.5
GOAL_AHEAD_DIST = 3.5
#LOOK_AHEAD_DIST = 0.4
# P_GAIN = 0.3
# SPEED = 2.7
NEAR_RANGE = 1.0
file_name = "/home/amf323/rcws/logs/.csv" # file name for waypoints


# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pf_topic = '/odom'
        scan_topic = '/scan'
        drive_topic = '/nav'
        odom_topic = '/odom'
        # TODO: create subscribers
        rospy.Subscriber(pf_topic,Odometry, self.pf_callback, queue_size = 5)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size = 5)
        show_topic = '/show'
        self.show_pub = rospy.Publisher(show_topic, Marker, queue_size = 10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size = 5)
        
        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
    
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry , self.pf_callback ,queue_size = 5)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=5)
        self.current_location_x = 0
        self.current_location_y = 0
        self.current_angle = 0
        self.local_map = [[0]*(local_map_size_half[0]*2+1)]* (local_map_size_half[1]*2+1)
        self.path = self.init_path()
        length = 1
        monte_carlo=np.random.uniform(-1.0, 1.0, length)
        
        # class attributes
        # TODO: maybe create your occupancy grid here
        #self.OG = OccupancyGrid()

    def Marker_method(self, Path):
        marker = Marker()
        for i in range(len(len(Path))):
            marker.ns = "Waypoints"
            marker.id = i
            marker.type = 2
            marker.header.frame_id = 'map'
            marker.action = Marker.ADD
            marker.pose.position.x = Path[i].x
            marker.pose.position.y = Path[i].y
            marker.pose.position.z = 0.5
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.show_pub.publish(marker)

            

    def goal_callback(self, goal_msg):
        goal_node = Node()
        goal_node.x = goal_msg.pose.position.x
        goal_node.y = goal_msg.pose.position.y
        self.path = [goal_node]
        return None


    def scan_callback(self, scan_msg):

        ranges = [x for x in scan_msg.ranges]

        current_location_map_x = len(large_map)-int(self.current_location_y/map_res+map_center[1])
        current_location_map_y = int(self.current_location_x/map_res+map_center[0])

        current_local_map = large_map[current_location_map_x-local_map_size_half[0]:current_location_map_x+local_map_size_half[0]+1, current_location_map_y-local_map_size_half[1]:current_location_map_y+local_map_size_half[1]+1].copy()
        for i in range(0, len(ranges), scan_res):
            dir_of_scan_local = i*scan_msg.angle_increment+scan_msg.angle_min
            dir_of_scan = dir_of_scan_local+self.current_angle
            location_scan_x_local = ranges(i)*math.cos(dir_of_scan_local)+0.275
            location_scan_y_local = ranges(i)*math.sin(dir_of_scan_local)
            location_scan_y = location_scan_x_local*math.sin(self.current_angle)+location_scan_y_local*math.cos(self.current_angle)
            location_scan_x = location_scan_y_local*math.sin(self.current_angle)+location_scan_x_local*math.cos(self.current_angle)
            location_scan_x_map = int(local_map_size_half[0]+1-location_scan_y/map_res)
            location_scan_y_map = int(local_map_size_half[1]+1-location_scan_x/map_res)

            if location_scan_x_map > expend_size and location_scan_x_map < len(current_local_map)-expend_size and location_scan_y_map > expend_size and location_scan_y_map < len(current_local_map[1])-expend_size:
                current_local_map[location_scan_x_map-expend_size:location_scan_x_map+expend_size+1,location_scan_y_map-expend_size:location_scan_y_map+expend_size+1] = (1)

            self.local_map  = current_local_map.copy()

        """
        LaserScan callback, you should update your occupancy grid here
        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
        """
        

    def init_path(self):
        goal_node = Node()
        goal_node.x = 1
        goal_node.y = 0
        path = [goal_node]
        return path



    def pf_callback(self, pose_msg):
        self.Marker_method(self.path)

        self.current_location_x = pose_msg.pose.pose.position.x
        self.current_location_y = pose_msg.pose.pose.position.y
        quaternion = np.arry([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])
        euler  = tf.transformations.euler_from_quaternion(quaternion)
        self.current_angle = euler[2]

        current_node = Node()
        current_node.x = self.current_location_x
        current_node.y = self.current_location_y
        current_node.is_root = True

        goal_node = self.path[0]

        if self.check_collision(goal_node, current_node):
            current_goal = goal_node
            i = 0
        else:
            tree = [current_node]
            goal_x = goal_node.x
            goal_y = goal_node.y
            while i < try_t:
                sampled_point = self.sample()
                nearest_node = self.nearest(tree, sampled_point)
                new_node = self.steer(nearest_node, sampled_point)

                if (not self.too_close(nearest_node,sampled_point)) and self.check_collision(nearest_node, new_node):
                    tree = tree+[new_node]
                    
                    if self.is_goal(new_node, goal_x, goal_y):
                        latest_added_node = new_node
                        break
                    i = i+1
            if i > 0.9*try_t:
                latest_added_node = self.nearest(tree, [goal_x, goal_y])
            new_path = self.find_path(tree, latest_added_node)
            self.path = new_path + self.path
            current_goal = self.path[0]
        current_goal_x  = current_goal.x
        current_goal_y  = current_goal.y

        x_local_trans = current_goal_x-self.current_location_x
        y_local_trans = current_goal_y-self.current_location_y
        dis = math.sqrt(x_local_trans*x_local_trans+y_local_trans*y_local_trans)

        speed = run_speed

        while dis<R_is_goal*0.9:
            if len(self.path)==1:
                speed = 0
                break
            self.path.pop(0)

            current_goal = self.path[0]
            current_goal_x = current_goal.x
            current_goal_y = current_goal.y
            x_local_trans = current_goal_x-self.current_location_x
            y_local_trans = current_goal_y-self.current_location_y

            x_local = x_local_trans*math.cos(self.current_angle)+y_local_trans*math.sin(self.current_angle)
            y_local = y_local_trans*math.cos(self.current_angle)-x_local_trans*math.sin(self.current_angle)
            gama = abs(y_local)/abs(x_local)

            if y_local > 0:
                angle_steering = min(P_arc*gama, 0.4189)

            else:
                angle_steering = max(-P_arc*gama, 0.41899)

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.drive.steering_angle = angle_steering
            drive_msg.drive.speed = speed
            self.drive_pub.publish(drive_msg)

    def to_local(self, point_x, point_y):
        diff_x = point_x-self.current_location_x
        diff_y = -(point_x-self.current_location_y)
        local_x = round(diff_y/map_res+local_map_size_half[0]+1)
        local_y = round(diff_x/map_res+local_map_size_half[1]+1)
        
        return (int(local_x), int(local_y))

    def to_larger(self, local_x, local_y):
        large_x = (local_y-local_map_size_half[1]-1)*map_res+self.current_location_x
        large_y = (local_x-local_map_size_half[0]-1)*map_res+self.current_location_y

        return(large_x, large_y)

        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens
        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:
        """

        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point
        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point
        """
        x = random.randint(0, 2*local_map_size_half[0]+1)
        y = random.randint(0, 2*local_map_size_half[1]+1)
        xx, yy = self.to_large(x, y)

        return [xx, yy]


    def too_close(self, nearest_node, sampled_point):
        diff_x = sampled_point[0]-nearest_node.x
        diff_y = sampled_point[1]-nearest_node.y
        dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
        if dis < B_length:
            return True
        return False

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point
        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """

        samplex_x = sampled_point[0]
        samplex_y = sampled_point[1]
        current_node = tree[0]
        current_x = current_node.x
        current_y = current_node.y
        diff_x = samplex_x-current_x
        diff_y = samplex_y-current_y
        min_dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
        index = 0

        for i in range(len(tree)):
            current_node = tree[i]
            current_x = current_node.x
            current_y= current_node.y

            diff_x = samplex_x-current_x
            diff_y - samplex_y-current_y
            dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
            if dis < min_dis:
                min_dis = dis
                index = i
            
        nearest_node =tree[index]
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.
        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = Node()
        diff_x = sampled_point[0]-nearest_node.x
        diff_y = sampled_point[1]-nearest_node.y
        dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
        new_node.x = nearest_node.x+diff_x*B_length/dis
        new_node.y = nearest_node.y+diff_y*B_length/dis
        new_node.parent = nearest_node

        return new_node

    def check_collision(self, nearest_node, new_node):

        nearest_node_local_x, nearest_node_local_y = self.to_local(nearest_node.x, nearest_node.y)
        new_node_local_x, new_node_local_y = self.to_local(new_node.x, new_node.y)

        diff_x = nearest_node_local_x-new_node_local_x
        diff_y = nearest_node_local_y-new_node_local_y
        if diff_x == 0:
            xx = new_node_local_x
            step = np.sign(nearest_node_local_y-new_node_local_y)
            
            for yy in range(new_node_local_y, nearest_node_local_y, step):
                if yy < 0 or yy > 2*local_map_size_half[1]:
                    return True
                if self.local_map[xx, yy] == 1:
                    return False

            return True
        if diff_y == 0:
            yy = new_node_local_y
            step = np.sign(nearest_node_local_x-new_node_local_x)
            for xx in range(new_node_local_x, nearest_node_local_x, step):
                if xx < 0 or xx > 2*local_map_size_half[0]:
                    return True
                if self.local_map[xx, yy] == 1:
                    return False
            return True

        if abs(diff_x) > abs(diff_y):
            ratio =  (diff_y+0.0)/diff_x
            step = np.sign(nearest_node_local_x-new_node_local_x)
            for xx in range(new_node_local_x, nearest_node_local_x, step):
                yy = int(round((xx-new_node_local_x)*ratio+ nearest_node_local_y))
                if xx < 0 or xx > 2*local_map_size_half[0] or yy < 0 or yy > 2*local_map_size_half[1]:
                    return True
                if self.local_map[xx, yy] == 1:
                    return False
            return True
        ratio = (diff_x+0.0)/diff_y
        collision_free = True
        step = np.sign(nearest_node_local_y-new_node_local_y)

        for yy in range(new_node_local_y, nearest_node_local_y, step):
            xx = int(round((yy-new_node_local_y)*ratio+ nearest_node_local_x))
            if xx < 0 or xx > 2*local_map_size_half[0] or yy < 0 or yy > 2*local_map_size_half[1]:
                return True

            if self.local_map[xx, yy] == 1:
                return False
        return True

        """
        This method should return whether the path between nearest and new_node is
        collision free.
        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.
        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        diff_x = goal_x-latest_added_node.x
        diff_y = goal_y-latest_added_node.y
        dis = math.sqrt(diff_x*diff_x+diff_y*diff_y)
        if dis < R_is_goal:
            return True
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal
        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        current_node = latest_added_node
        while current_node.is_root == False:
            path = [current_node]+path
            current_node = current_node.parent
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node
        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2
        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node
        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()