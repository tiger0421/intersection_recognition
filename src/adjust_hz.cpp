#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <unistd.h>

class My_Filter {
     public:
        My_Filter();
        int get_ros_param(void);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle node_;
        ros::Publisher scan_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &My_Filter::scanCallback, this);
        scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> ("/scan2", 1, false);
}

int My_Filter::get_ros_param(void){
    int hz = 1;
    sleep(1);
    node_.getParam("adjust_hz/scan_hz", hz);
    return hz;
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    scan_publisher_.publish(*scan);
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
