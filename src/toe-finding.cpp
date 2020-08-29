#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <bits/stdc++.h>
#include <vector>

class JunctionRecognition {
     public:
        JunctionRecognition();
        int hz;
        int off_set;
        void get_ros_param(void);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        const std::string TRAIN_DATA_FILES[7] = {"dead_end", "left", "right", "straight", "threeway_left", "threeway_center", "threeway_right"};

     private:
        ros::NodeHandle node_;
        ros::Publisher scan_pub_;
        ros::Subscriber scan_sub_;
};

JunctionRecognition::JunctionRecognition(){
//        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan2", 1, &JunctionRecognition::scanCallback, this);
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &JunctionRecognition::scanCallback, this);
        scan_pub_ = node_.advertise<std_msgs::String> ("hypothesis", 1, false);
}

void JunctionRecognition::get_ros_param(void){
    // error reason is maybe this
    this->hz = 1;
    sleep(1);
    node_.getParam("adjust_hz/scan_hz", this->hz);
    this->off_set = 10;
}

void JunctionRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    float delta_h, delta_i, delta_j;
    std::vector<float> v_scan;
    int num_cloud = scan->ranges.size();
    for(int i = 0; i < num_cloud; i += this->off_set){
        delta_h = scan->ranges[(i - this->off_set + num_cloud) % num_cloud];
        delta_i = scan->ranges[i];
        delta_j = scan->ranges[(i + this->off_set) % num_cloud];

        std::cout << (i-this->off_set+num_cloud)%num_cloud << std::endl;
        std::cout << i << std::endl;
        std::cout << (i+this->off_set) %num_cloud << std::endl;
        std::cout << std::endl;
    }


}

int main(int argc, char** argv){
    ros::init(argc, argv, "adjust_hz");
    JunctionRecognition recognition;
    recognition.get_ros_param();
    ros::Rate loop_rate(recognition.hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
