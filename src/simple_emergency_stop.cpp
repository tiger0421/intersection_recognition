#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cstdlib>
#include <cmath>
#include <bits/stdc++.h>
#include <vector>

class simpleEmergencyStop {
    public:
        simpleEmergencyStop();
        int SCAN_HZ;
        float stop_distance_thresh;
        void get_ros_param(void);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    private:
        ros::NodeHandle node_;
        ros::Publisher emergency_stop_flg_pub_;
        ros::Subscriber scan_sub_;
        bool emergency_stop_flg_ = false;
        bool emergency_stop_flg_prev_ = false;
};

simpleEmergencyStop::simpleEmergencyStop(){
    emergency_stop_flg_pub_ = node_.advertise<std_msgs::Bool>("emergency_stop_flg", 1, false);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &simpleEmergencyStop::scanCallback, this);
}

void simpleEmergencyStop::get_ros_param(void){
    SCAN_HZ = 10;
    stop_distance_thresh = 0.2;
    node_.getParam("extended_toe_finding/SCAN_HZ", SCAN_HZ);
    node_.getParam("simple_emergency_stop/stop_distance_thresh", stop_distance_thresh);
}

void simpleEmergencyStop::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
// if scan_min is between -45[rad] and +45[rad] and scan_range is less than threshold, publish /emergency_stop_flg
    std::vector<double> scan_copy(scan->ranges.size());
    std::copy(scan->ranges.begin(), scan->ranges.end(), scan_copy.begin());
    for(int i=0; i<scan->ranges.size(); i++){
        if(scan_copy[i] < scan->range_min){
            scan_copy[i] = 1000.0;
        }
    }
    int index_scan_min = std::min_element(scan_copy.begin(), scan_copy.end()) - scan_copy.begin();
    double distance_scan_min = scan->ranges[index_scan_min];
    if(distance_scan_min <= stop_distance_thresh){
        double rad_scan_min = index_scan_min * scan->angle_increment;
        if(-M_PI_4 <= rad_scan_min && rad_scan_min <= M_PI_4){
            ROS_WARN("emergency_stop_flg is switched to true");
            emergency_stop_flg_ = true;
            std_msgs::Bool emergency_stop_flg_for_pub;
            emergency_stop_flg_for_pub.data = emergency_stop_flg_;
            emergency_stop_flg_pub_.publish(emergency_stop_flg_for_pub);
            emergency_stop_flg_prev_ = emergency_stop_flg_;
        }
    }
    else{
        if(emergency_stop_flg_prev_ == true && stop_distance_thresh < distance_scan_min){
            ROS_WARN("emergency_stop_flg is switched to false");
            emergency_stop_flg_ = false;
            std_msgs::Bool emergency_stop_flg_for_pub;
            emergency_stop_flg_for_pub.data = emergency_stop_flg_;
            emergency_stop_flg_pub_.publish(emergency_stop_flg_for_pub);
            emergency_stop_flg_prev_ = emergency_stop_flg_;
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_emergency_stop");
    simpleEmergencyStop emergency_stop;
    emergency_stop.get_ros_param();
    ros::Rate loop_rate(emergency_stop.SCAN_HZ);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
