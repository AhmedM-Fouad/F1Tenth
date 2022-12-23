#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker(): # replace talker with proper function name for this node
    pub = rospy.Publisher('chatter', String, queue_size=10) #replace chatter with the new node that obtains data from lidar
    rospy.init_node('talker', anonymous=True) #replace talker with hte name u want for thew node
    rate = rospy.Rate(10) # 10hz
    # process 
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep() 


if __name__ == '__main__': 
    try:
        talker()
    except rospy.ROSInterruptException:
        pass