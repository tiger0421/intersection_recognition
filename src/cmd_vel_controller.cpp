#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "intersection_recognition/Scenario.h"
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        geometry_msgs::Twist vel_;
        int SCAN_HZ = 0;
        double IMU_HZ = 100.0;
        double CHANGE_DIRECTION_DISTANCE_THRESH = 0.0;
        double CHANGE_DIRECTION_RAD = 0.0;
        float reverse_turn = 0;
        float rotate_rad_ = 0;
        void getRosParam(void);
        void moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad);
        void emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle node_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher turn_finish_flg_pub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber rotate_rad_sub_;
        ros::Subscriber emergency_stop_flg_sub_;
        ros::Subscriber scan_sub_;

        bool turn_flg_ = false;
        bool emergency_stop_flg_ = true;
        bool big_modified_flg_left_ = false;
        bool big_modified_flg_right_ = false;
        const int SCENARIO_MAX = 10;
        std::string last_node_ = "start";
        double target_yaw_rad_ = 0;
        double current_yaw_rad_ = 0;
};

cmdVelController::cmdVelController(){
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
    turn_finish_flg_pub_ = node_.advertise<std_msgs::Bool>("turn_finish_flg", 1, false);

    imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("imu_data", 1, &cmdVelController::moveCallback, this);
    rotate_rad_sub_ = node_.subscribe<std_msgs::Float32> ("rotate_rad", 1, &cmdVelController::turnRadCallback, this);
    emergency_stop_flg_sub_ = node_.subscribe<std_msgs::Bool> ("emergency_stop_flg", 1, &cmdVelController::emergencyStopFlgCallback, this);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/hokuyo_scan", 1, &cmdVelController::scanCallback, this);

    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCAN_HZ = 10;
    IMU_HZ = 100.0;
    reverse_turn = 1.0;
    CHANGE_DIRECTION_DISTANCE_THRESH = 0.40;
    CHANGE_DIRECTION_RAD = 0.3;
    node_.getParam("extended_toe_finding/SCAN_HZ", SCAN_HZ);
    node_.getParam("cmd_vel_controller/IMU_HZ", IMU_HZ);
    node_.getParam("cmd_vel_controller/reverse_turn", reverse_turn);
    node_.getParam("cmd_vel_controller/CHANGE_DIRECTION_DISTANCE_THRESH", CHANGE_DIRECTION_DISTANCE_THRESH);
}

void cmdVelController::moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    current_yaw_rad_ += imu_data->angular_velocity.z / IMU_HZ;
    if(! emergency_stop_flg_){
        if(turn_flg_){
        // rotate_rad += imu_data->angular_velocity.z[rad/sec] * (1/IMU_HZ)[sec]
            rotate_rad_ += imu_data->angular_velocity.z / IMU_HZ;
        // 3.14/180 means 1[rad]
            std::cout << "rotate rad is " << rotate_rad_ << std::endl;
            std::cout << "target - current is " <<  -(target_yaw_rad_ - current_yaw_rad_) << std::endl;
            if(std::abs(rotate_rad_) < 3.14/180){
                turn_flg_ = false;
                std_msgs::Bool turn_finish_flg_for_pub;
                turn_finish_flg_for_pub.data = true;
                turn_finish_flg_pub_.publish(turn_finish_flg_for_pub);
                rotate_rad_ = 0;
            }

        // if rotate_rad = 0, do nothing because it is turn_flg is false.
            if(rotate_rad_ < 0){
                vel_.angular.z = -0.5;
            }
            else if(rotate_rad_ > 0){
                vel_.angular.z = 0.5;
            }

            vel_.linear.x = 0.0;
            cmd_vel_pub_.publish(vel_);
            vel_.angular.z = 0.0;
        }
    // if turn_flg is false
        else{
            vel_.linear.x = 0.55;
        // vel_.angular.z = (target_yaw_rad_ - current_yaw_rad_)[rad] * (1/IMU_HZ)[sec]
            vel_.angular.z = -(target_yaw_rad_ - current_yaw_rad_) * reverse_turn;
            cmd_vel_pub_.publish(vel_);
            vel_.linear.x = 0.0;
        }
    }
    else{
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
        cmd_vel_pub_.publish(vel_);
    }
}

void cmdVelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    std::vector<float> scan_copy(scan->ranges.size());
    std::copy(scan->ranges.begin(), scan->ranges.end(), scan_copy.begin());
    for(int i=0; i<scan->ranges.size(); i++){
        if(scan_copy[i] <= scan->range_min){
            scan_copy[i] = 1000.0;
        }
    }
    int index_scan_min_right = std::min_element(std::next(scan_copy.begin(), (-3*M_PI_4 - scan->angle_min)/scan->angle_increment), std::next(scan_copy.begin(), (-M_PI/12 - scan->angle_min)/scan->angle_increment)) - scan_copy.begin();
    int index_scan_min_left = std::min_element(std::next(scan_copy.begin(), (M_PI/12 - scan->angle_min)/scan->angle_increment), std::next(scan_copy.begin(), (3*M_PI_4 - scan->angle_min)/scan->angle_increment)) - scan_copy.begin();
    double distance_scan_min_right = scan->ranges[index_scan_min_right];
    double distance_scan_min_left = scan->ranges[index_scan_min_left];
    if(distance_scan_min_right <= CHANGE_DIRECTION_DISTANCE_THRESH && distance_scan_min_right < distance_scan_min_left){
        if(!big_modified_flg_right_){
          ROS_WARN("Modified robot-direction to left");
          big_modified_flg_right_ = true;
          target_yaw_rad_ -= CHANGE_DIRECTION_RAD;
        }
    }
    else if(distance_scan_min_left <= CHANGE_DIRECTION_DISTANCE_THRESH && distance_scan_min_left < distance_scan_min_right){
        if(!big_modified_flg_left_){
          ROS_WARN("Modified robot-direction to right");
          big_modified_flg_left_ = true;
          target_yaw_rad_ += CHANGE_DIRECTION_RAD;
        }
    }
    else{
      if(big_modified_flg_right_){
        big_modified_flg_right_ = false;
        target_yaw_rad_ += CHANGE_DIRECTION_RAD * 9 / 10;
      }
      else if(big_modified_flg_left_){
        big_modified_flg_left_ = false;
        target_yaw_rad_ -= CHANGE_DIRECTION_RAD * 9 / 10;
      }
    }
//    std::cout << "yaw rad is " << target_yaw_rad_ << "  diff is " << target_yaw_rad_ - current_yaw_rad_ << std::endl;
}

void cmdVelController::turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad){
    rotate_rad_ = turn_rad->data * reverse_turn;
    target_yaw_rad_ -= turn_rad->data * reverse_turn;
    if(rotate_rad_ == 0.0){
        turn_flg_ = false;
    }
    else{
        turn_flg_ = true;
    }
}

void cmdVelController::emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg){
    emergency_stop_flg_ = emergency_stop_flg->data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_controller");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
