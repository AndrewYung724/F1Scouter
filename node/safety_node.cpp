#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <algorithm>
#include <cmath>
// #include <iostream> 
using namespace std;

// Solution based on submission of Zirui Zang

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double v_car;

    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Publisher brake_pub;
    ros::Publisher brake_bool_pub;
    laser_geometry::LaserProjection projector;
    tf::TransformListener tf_listener;

    std_msgs::Bool Bool_msg;
    // Bool_msg.data = false;
    ackermann_msgs::AckermannDriveStamped ackermann_msg;

public:

    Safety() {
        n = ros::NodeHandle();
        v_car = 0.0;
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

        laser_sub = n.subscribe("/scan", 1000, &Safety::scan_callback, this);
        odom_sub = n.subscribe("/odom", 1000, &Safety::odom_callback, this);
        brake_bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1000);
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);        
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        v_car = odom_msg->twist.twist.linear.x;
        // ROS_INFO_STREAM(v_car);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        double v_x = v_car - 0;
        vector<float> scan_ranges = scan_msg->ranges;
        vector<float> TTC = scan_ranges;
        float angle_increment = scan_msg->angle_increment;
        float angle_min = scan_msg->angle_min;
        float ttc_threshold = 0.32;
        // ROS_INFO_STREAM(scan_msg->header.frame_id);

        sensor_msgs::PointCloud cloud;
        tf_listener.waitForTransform(scan_msg->header.frame_id, "/center", scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment), ros::Duration(1.0));
        projector.transformLaserScanToPointCloud("center", *scan_msg, cloud, tf_listener, 0x02);
        vector<float> shifted_ind = cloud.channels[0].values;
        vector<float> shifted_scan_ranges = scan_ranges;
        
        // calculate the TTC
        for(int ind = 0; ind < shifted_ind.size(); ind++){
            // ROS_INFO_STREAM(cloud);
            double shifted_scan_range = sqrt(pow(cloud.points[ind].x, 2) + pow(cloud.points[ind].y, 2));
            shifted_scan_ranges[shifted_ind[ind]] = shifted_scan_range;
            if(!isnan(shifted_scan_range) && !isnan(shifted_scan_range)){
                // double ttc = scan_ranges[ind] / max(v_x * cos(angle_min + angle_increment * ind), 0.0);
                double ttc = shifted_scan_range / max(v_x * cos(atan2(cloud.points[ind].y, cloud.points[ind].x)), 0.0);
                if(ttc < 0){
                    TTC[ind] = numeric_limits<double>::infinity();
                }else{
                    TTC[ind] = ttc;
                }
            }else{
                TTC[ind] = numeric_limits<double>::infinity();
            }
            // ROS_INFO_STREAM(TTC[ind]);
        }
    
        float min_ttc = *min_element(TTC.begin(), TTC.end());
        if(min_ttc < ttc_threshold){
            Bool_msg.data = true;
        }else if(min_ttc > 1){
            Bool_msg.data = false;
        }
        if(Bool_msg.data == true){
            Bool_msg.data = true;
            ackermann_msg.drive.speed = 0.0;
            brake_bool_pub.publish(Bool_msg);
            brake_pub.publish(ackermann_msg);
        }
        
        if(min_ttc < 1){
            ROS_INFO_STREAM(min_ttc);
        }

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}