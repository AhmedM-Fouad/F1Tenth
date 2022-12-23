#!/usr/bin/env python

import rospy
from std_msgs.msg import String

global LidarVals
global MaxVal
global MinVal

# subscriber function
def callback(data):
    rospy.loginfo(rospy.get_caller_id(), LidarVals) 
    # use numpy to find both min and max values from array of values and save them into globals variables declared above
    


def lidar_listener(): 
    
    rospy.init_node('Obtain_Raw_lidar_Values', anonymous=True)

    rospy.Subscriber("chatter", String, callback) #replace chatter with the topic from lidar
    rospy.spin()

#if __name__ == '__main__':
    #LidarNode()

#publisher1 function
def MaxVals(): 
    pub = rospy.Publisher('Max Distance', String, queue_size=10) # topic is new lidar values
    rospy.init_node('Max Lidar Value', anonymous=True) # publishes processed values
    rate = rospy.Rate(10) # change to match the rate of data sent from lidar
    while not rospy.is_shutdown():
        # lidar_max = "Max Value: " + MaxVal
        pub.publish(lidar_max)
        rate.sleep()

def MinVals(): 
    pub = rospy.Publisher('Min Distance', String, queue_size=10) # topic is new lidar values
    rospy.init_node('Min Lidar Value', anonymous=True) # publishes processed values
    rate = rospy.Rate(10) # change to match the rate of data sent from lidar
    while not rospy.is_shutdown():
        # lidar_min = "Min Value: " + MinVal
        pub.publish(lidar_min)
        rate.sleep()


if __name__ == '__main__': 
    try:
        LidarNode()
    except rospy.ROSInterruptException:
        pass

