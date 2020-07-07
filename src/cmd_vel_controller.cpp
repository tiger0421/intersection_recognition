#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <unistd.h>
#include <vector>

class Variables {
     public:
        Variables();
        std_msgs::Bool* turn_flg_;
        geometry_msgs::Twist* vel_;
        float rotate_rad_ = 0;
        void hypothesisCallback(const std_msgs::String::ConstPtr& hypothesis);
        void moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data);

     private:
        ros::NodeHandle node_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher turn_flg_pub_;
        ros::Subscriber hypothesis_sub_;
        ros::Subscriber imu_sub_;
        std::vector<std::string> target_condition_;
        std::vector<std::string> target_action_;
        int cnt_ = 0;
        int buff_ = 0;
};

Variables::Variables(){
    turn_flg_->data = false;
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
    turn_flg_pub_ = node_.advertise<std_msgs::Bool>("/turn_flg", 1, false);
    hypothesis_sub_ = node_.subscribe<std_msgs::String> ("/hypothesis", 1, &Variables::hypothesisCallback, this);
    imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("/imu_data", 1, &Variables::moveCallback, this);

    target_condition_.push_back("threeway_center");
    target_condition_.push_back("threeway_left");
}

void Variables::hypothesisCallback(const std_msgs::String::ConstPtr& hypothesis){
    if(!turn_flg_->data){
        if(! (hypothesis->data == target_condition_[cnt_])){
            ROS_INFO("different between h and target");
        }
        else{
            buff_++;
            if(buff_ >= 3){
                ROS_INFO("robot reaches target_node!!");
                buff_ = 0;
                cnt_++;
                if(hypothesis->data == target_condition_.back()){
// may need cmd_vel = 0.0
                    ROS_INFO("robot gets a goal");
                    hypothesis_sub_.shutdown();
                    ros::shutdown();
                }
                else{
                    ROS_INFO("start turning");
                    turn_flg_->data = true;
                    turn_flg_pub_.publish(*turn_flg_);
                }
            }
        }
    }
}

void Variables::moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    rotate_rad_ += imu_data->angular_velocity.z;
    if(rotate_rad_ >= 3.14/2){
        turn_flg_->data = false;
        turn_flg_pub_.publish(*turn_flg_);
        rotate_rad_ = 0;
    }

    if(! turn_flg_->data){
        vel_->linear.x = 1;
    }
    else{
// may inverse angular.z
        if(target_action_[cnt_] == "left") vel_->angular.z = 1;
        else vel_->angular.z = -1;
    }

    cmd_vel_pub_.publish(*vel_);
    vel_->linear.x = 0;
    vel_->angular.z = 0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "adjust_hz");
    Variables variables;
    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
