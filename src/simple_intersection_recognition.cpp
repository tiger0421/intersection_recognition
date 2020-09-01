#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <bits/stdc++.h>

class intersectionRecognition {
     public:
        intersectionRecognition();
        int hz;
        void get_ros_param(void);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        const std::string TRAIN_DATA_FILES[7] = {"dead_end", "left", "right", "straight", "threeway_left", "threeway_center", "threeway_right"};

     private:
        ros::NodeHandle node_;
        ros::Publisher scan_pub_;
        ros::Subscriber scan_sub_;
};

intersectionRecognition::intersectionRecognition(){
//        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan2", 1, &intersectionRecognition::scanCallback, this);
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &intersectionRecognition::scanCallback, this);
        scan_pub_ = node_.advertise<std_msgs::String> ("hypothesis", 1, false);
}

void intersectionRecognition::get_ros_param(void){
    // error reason is maybe this
    this->hz = 1;
    sleep(1);
    node_.getParam("adjust_hz/scan_hz", this->hz);
}

void intersectionRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    std::cout << "hz is " << this->hz << std::endl;
    unsigned int index_left_point = 0;
    unsigned int index_front_point = 0;
    unsigned int index_right_point = 0;

    if((scan->angle_max >= 3.14/2) && (scan->angle_min < -3.14/2)){
        index_right_point = static_cast<unsigned int>((-M_PI_2 - scan->angle_min) / scan->angle_increment);
        index_left_point = static_cast<unsigned int>((scan->angle_max - M_PI_2 + M_PI) / scan->angle_increment);
        index_front_point = (index_left_point + index_right_point) / 2;
    }
    else{
        std::cerr << "Lack of points" << std::endl;
        std::cerr << "shutdown this node" << std::endl;
        scan_sub_.shutdown();
        ros::shutdown();
    }

    std::cout<< "left data is " << index_left_point << std::endl;
    std::cout<< "front data is " << index_front_point << std::endl;
    std::cout<< "right data is " << index_right_point << std::endl;



    std::cout<< "left data is " << scan->ranges[index_left_point] << std::endl;
    std::cout<< "front data is " << scan->ranges[index_front_point] << std::endl;
    std::cout<< "right data is " << scan->ranges[index_right_point] << std::endl;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "adjust_hz");
    intersectionRecognition recognition;
    recognition.get_ros_param();
    ros::Rate loop_rate(recognition.hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
