#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include <unistd.h>

class My_Filter {
     public:
        My_Filter();
        bool turn_flg_ = false;
        int get_ros_param(void);
        void flgCallback(const std_msgs::Bool::ConstPtr& flg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle node_;
        ros::Publisher scan_pub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber flg_sub_;
};

My_Filter::My_Filter(){
        flg_sub_ = node_.subscribe<std_msgs::Bool> ("/turn_flg", 1, &My_Filter::flgCallback, this);
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &My_Filter::scanCallback, this);
        scan_pub_ = node_.advertise<sensor_msgs::LaserScan> ("/scan2", 1, false);
}

int My_Filter::get_ros_param(void){
    int hz = 1;
    sleep(1);
    node_.getParam("adjust_hz/scan_hz", hz);
    return hz;
}

void My_Filter::flgCallback(const std_msgs::Bool::ConstPtr& flg){
    turn_flg_ = flg->data;
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(!turn_flg_){
        scan_pub_.publish(*scan);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "adjust_hz");
    My_Filter filter;
    int hz = 1;
    hz = filter.get_ros_param();
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
