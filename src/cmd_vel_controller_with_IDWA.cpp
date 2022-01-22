#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "intersection_recognition/Scenario.h"
#include <unistd.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        geometry_msgs::Twist vel_;
        std::string ROBOT_FRAME = "base_link";
        double IMU_HZ = 100.0;
        double robot_control_freq = 10.0;
        double path_update_freq = 2.0;
        double ROBOT_COLLISION_RADIUS = 0.5;
        double dt = 1 / path_update_freq;
        double predict_time = 3.0;
        double v_max_lim = 2.0;
        double v_min_lim = -2.0;
        double v_acc_max = 0.5;
        double delta_v = 0.05;
        double w_max_lim = 0.5;
        double w_min_lim = -0.5;
        double w_acc_max = 0.2;
        double delta_w = 0.05;
        double k_rho = 3.0;
        double k_alpha = 0.59;
        double k_v = 1.0;
        double lambda1 = 3/13;
        double lambda2 = 3/13;
        double lambda3 = 7/13;
        float reverse_turn = 1.0;
        float target_yaw_rad_ = 0;
        void getRosParam(void);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void targetYawRadCallback(const std_msgs::Float32::ConstPtr& target_yaw_rad);
        void emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        Eigen::MatrixXd calcDistance(const sensor_msgs::LaserScan::ConstPtr& scan, const Eigen::VectorXd &v_range, const Eigen::VectorXd &w_range);
        Eigen::MatrixXd minMaxNormalize(const Eigen::MatrixXd &mat);
        void visualize_trajectory(const double v, const double w, const ros::Publisher& pub);
        void timerForCmdVelCallback(const ros::TimerEvent& e_cmd_vel);
        void timerForDWACallback(const ros::TimerEvent& e_dwa);

     private:
        ros::NodeHandle node_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher turn_finish_flg_pub_;
        ros::Publisher selected_trajectory_pub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber target_yaw_rad_sub_;
        ros::Subscriber emergency_stop_flg_sub_;
        ros::Subscriber scan_sub_;
        ros::Timer timer_for_cmd_vel;
        ros::Timer timer_for_dwa;

        bool emergency_stop_flg_ = true;
        bool update_path_flg_ = true;
        double current_yaw_rad_ = 0.0;
        double rho = 5.0;
        double alpha = 0.0;
};

cmdVelController::cmdVelController(){
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
    turn_finish_flg_pub_ = node_.advertise<std_msgs::Bool>("turn_finish_flg", 1, false);
    selected_trajectory_pub_ = node_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("imu_data", 1, &cmdVelController::imuCallback, this);
    target_yaw_rad_sub_ = node_.subscribe<std_msgs::Float32> ("target_yaw_rad", 1, &cmdVelController::targetYawRadCallback, this);
    emergency_stop_flg_sub_ = node_.subscribe<std_msgs::Bool> ("emergency_stop_flg", 1, &cmdVelController::emergencyStopFlgCallback, this);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("scan", 1, &cmdVelController::scanCallback, this);

    getRosParam();
    timer_for_cmd_vel = node_.createTimer(ros::Duration(1.0/robot_control_freq), &cmdVelController::timerForCmdVelCallback, this);
    timer_for_dwa = node_.createTimer(ros::Duration(1.0/path_update_freq), &cmdVelController::timerForDWACallback, this);
}

void cmdVelController::getRosParam(void){
    node_.getParam("cmd_vel_controller/IMU_HZ", IMU_HZ);
    node_.getParam("cmd_vel_controller/robot_control_freq", robot_control_freq);
    node_.getParam("cmd_vel_controller/path_update_freq", path_update_freq);
    node_.getParam("cmd_vel_controller/ROBOT_COLLISION_RADIUS", ROBOT_COLLISION_RADIUS);
    node_.getParam("cmd_vel_controller/ROBOT_FRAME", ROBOT_FRAME);
    node_.getParam("cmd_vel_controller/predict_time", predict_time);
    node_.getParam("cmd_vel_controller/reverse_turn", reverse_turn);
    if(reverse_turn != 1.0) reverse_turn = -1.0;
    node_.getParam("cmd_vel_controller/v_max_lim", v_max_lim);
    node_.getParam("cmd_vel_controller/v_min_lim", v_min_lim);
    node_.getParam("cmd_vel_controller/v_acc_max", v_acc_max);
    node_.getParam("cmd_vel_controller/delta_v", delta_v);
    node_.getParam("cmd_vel_controller/w_max_lim", w_max_lim);
    node_.getParam("cmd_vel_controller/w_min_lim", w_min_lim);
    node_.getParam("cmd_vel_controller/w_acc_max", w_acc_max);
    node_.getParam("cmd_vel_controller/k_rho", k_rho);
    node_.getParam("cmd_vel_controller/k_alpha", k_alpha);
    node_.getParam("cmd_vel_controller/k_v", k_v);
    node_.getParam("cmd_vel_controller/lambda1", lambda1);
    node_.getParam("cmd_vel_controller/lambda2", lambda2);
    node_.getParam("cmd_vel_controller/lambda3", lambda3);
}

void cmdVelController::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    current_yaw_rad_ += imu_data->angular_velocity.z / IMU_HZ;
}

void cmdVelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(! emergency_stop_flg_){
        if(update_path_flg_){
            alpha = target_yaw_rad_ - current_yaw_rad_; // [rad]
            if(alpha > M_PI){
                alpha -= 2 * M_PI;
            }
            else if(alpha < -M_PI){
                alpha += 2 * M_PI;
            }
            // calculate ideal control
            double vi = k_v * v_max_lim * std::cos(alpha) * std::tanh(rho / k_rho);
            double wi = k_alpha * alpha + k_v * v_max_lim * std::tanh(rho / k_rho) / rho * std::sin(2.0 * alpha) / 2.0;

            // calculate dynamic window
            double next_v_max = std::min(v_max_lim, vel_.linear.x + v_acc_max / path_update_freq);
            double next_v_min = std::max(v_min_lim, vel_.linear.x - v_acc_max / path_update_freq);
            double next_w_max = std::min(w_max_lim, vel_.angular.z + w_acc_max / path_update_freq);
            double next_w_min = std::max(w_min_lim, vel_.angular.z - w_acc_max / path_update_freq);
            int v_range_size  = std::round((next_v_max - next_v_min) / delta_v) + 1;
            int w_range_size  = std::round((next_w_max - next_w_min) / delta_w) + 1;

            Eigen::VectorXd v_range = Eigen::VectorXd::LinSpaced(v_range_size, next_v_min, next_v_max);
            Eigen::VectorXd w_range = Eigen::VectorXd::LinSpaced(w_range_size, next_w_min, next_w_max);

            // calculate objective function
            // about velocity
            Eigen::MatrixXd mat_one = Eigen::MatrixXd::Ones(v_range.size(), w_range.size());
            Eigen::VectorXd vec_one = Eigen::VectorXd::Ones(v_range.size());
            Eigen::MatrixXd G_v = (vec_one - ((v_range - vi * vec_one).cwiseAbs() / (2 * v_max_lim))).asDiagonal() * mat_one;
            G_v = minMaxNormalize(G_v);

            // about angular velocity
            vec_one = Eigen::VectorXd::Ones(w_range.size());
            //mat_one = Eigen::MatrixXd::Ones(w_range.size(), v_range.size());
            Eigen::MatrixXd G_w = mat_one * (vec_one - ((w_range - wi * vec_one).cwiseAbs() / (2 * w_max_lim))).asDiagonal();
            G_w = minMaxNormalize(G_w);

            // about distance to obstacles
            Eigen::MatrixXd min_distance = calcDistance(scan, v_range, w_range);
            Eigen::MatrixXd G_dist = min_distance;
            G_dist = minMaxNormalize(G_dist);

            // merge variables
            Eigen::MatrixXd G = lambda1 * G_v + lambda2 * G_w + lambda3 * G_dist;
            G -= (min_distance.array() < ROBOT_COLLISION_RADIUS).matrix().cast<double>() * 1e6;

            // get a pair of velocity and angle velocity which takes highest score
            Eigen::MatrixXd::Index maxRow, maxCol;
            double max_distance = G.maxCoeff(&maxRow, &maxCol);
            vel_.linear.x = v_range(maxRow);
            vel_.angular.z = w_range(maxCol);
            visualize_trajectory(vel_.linear.x, vel_.angular.z, selected_trajectory_pub_);
            std::cout << "IDWA: selected v is " << vel_.linear.x << " selected w is " << vel_.angular.z << std::endl;
        }
    }
}

Eigen::MatrixXd cmdVelController::calcDistance(const sensor_msgs::LaserScan::ConstPtr& scan, const Eigen::VectorXd &v_range, const Eigen::VectorXd &w_range){
    // skip inf
    int num_scan = scan->ranges.size();
    int valid_beams = 0;
    for(int i = 0; i < num_scan; i ++) {
        if(!(std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i]) || scan->ranges[i] > scan->range_max || scan->ranges[i] < scan->range_min)){
            valid_beams++;
        }
    }
    Eigen::VectorXd scan_cp_eigen(valid_beams);
    Eigen::VectorXd scan_direction(valid_beams);
    for(int i = 0; i < num_scan; i ++) {
        if(!(std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i]) || scan->ranges[i] > scan->range_max || scan->ranges[i] < scan->range_min)){
            scan_cp_eigen(i) = scan->ranges[i];
            scan_direction(i) = scan->angle_min + scan->angle_increment * i;
        }
    }

    // calculate obstacle point
    Eigen::VectorXd obs_point_x = (scan_direction.array() + M_PI_2).cos().matrix().asDiagonal() * scan_cp_eigen;
    Eigen::VectorXd obs_point_y = (scan_direction.array() + M_PI_2).sin().matrix().asDiagonal() * scan_cp_eigen;

    // calculate robot state
    Eigen::MatrixXd state_x = Eigen::MatrixXd::Zero(v_range.size(), w_range.size());
    Eigen::MatrixXd state_y = Eigen::MatrixXd::Zero(v_range.size(), w_range.size());
    Eigen::MatrixXd min_distance = Eigen::MatrixXd::Ones(v_range.size(), w_range.size()) * 1e6;

    Eigen::MatrixXd vec_one = Eigen::MatrixXd::Ones(v_range.size(), w_range.size());
    for(double t = 0; t <= predict_time; t += dt){
        state_x += v_range.asDiagonal() * vec_one * (w_range.array() + M_PI_2).cos().matrix().asDiagonal() * dt;
        state_y += v_range.asDiagonal() * vec_one * (w_range.array() + M_PI_2).sin().matrix().asDiagonal() * dt;
        // calculate distance to obstacle
        for(int index_row = 0; index_row < state_x.rows(); index_row++){
            for(int index_col = 0; index_col < state_x.cols(); index_col++){
                auto distance_x = obs_point_x.array() - state_x(index_row, index_col);
                auto distance_y = obs_point_y.array() - state_y(index_row, index_col);
                Eigen::MatrixXd distance = (distance_x.array().pow(2) + distance_y.array().pow(2)).sqrt();
                min_distance(index_row, index_col) = distance.minCoeff();
            }
        }
    }
    return min_distance;
}

Eigen::MatrixXd cmdVelController::minMaxNormalize(const Eigen::MatrixXd &mat){
    double min = mat.minCoeff();
    double max = mat.maxCoeff();
    Eigen::MatrixXd result = (mat.array() - min).matrix() / (max - min);
    return result;
}

void cmdVelController::visualize_trajectory(const double v, const double w, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = 1;
    v_trajectory.color.g = 0;
    v_trajectory.color.b = 1;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    for(double t = 0.0; t <= predict_time; t+=dt){
        p.x += v * std::cos(w) * dt;
        p.y += v * std::sin(w) * dt;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

void cmdVelController::targetYawRadCallback(const std_msgs::Float32::ConstPtr& target_yaw_rad){
    current_yaw_rad_ = 0.0;
    target_yaw_rad_ = target_yaw_rad->data * reverse_turn;
}

void cmdVelController::emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg){
    emergency_stop_flg_ = emergency_stop_flg->data;
}

void cmdVelController::timerForCmdVelCallback(const ros::TimerEvent& e_cmd_vel){
    if(emergency_stop_flg_){
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
    }

    cmd_vel_pub_.publish(vel_);
}

void cmdVelController::timerForDWACallback(const ros::TimerEvent& e_dwa){
    update_path_flg_ = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_controller");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
