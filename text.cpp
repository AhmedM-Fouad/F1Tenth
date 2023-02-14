#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "std_msgs/Float32.h"

// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers
    const string pose_topic = "odom";
    const string scan_topic = "scan";
    const string drive_topic = "/nav";

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        ros::Subscriber sub = n.subscribe("odom", 10, odom_callback);
        ros::Publisher scan_callback = n.advertise<std_msgs::String>("scan", 1000);
        ros::Publisher pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        ros::Publisher pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        ros::Rate loop_rate(10);

        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = 0.0;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC

        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}