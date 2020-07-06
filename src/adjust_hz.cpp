#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>

class My_Filter {
     public:
        int hz = 1;
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        ros::Publisher scan_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &My_Filter::scanCallback, this);
        scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> ("/scan2", 1, false);
        node_.getParam("adjust_hz/scan_hz", hz);
        sleep(2)
        ros::Rate loop_rate(hz);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    scan_publisher_.publish(*scan);
    loop_rate.sleep();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "adjust_hz");
    My_Filter filter;
    ros::spin();

    return 0;
}
