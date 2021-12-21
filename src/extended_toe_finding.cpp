#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include "intersection_recognition/Hypothesis.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cstdlib>
#include <cmath>
#include <bits/stdc++.h>
#include <vector>

class intersectionRecognition {
    public:
        intersectionRecognition();
        int SCAN_HZ;
        float distance_thresh;
        std::string robot_frame_;
        void get_ros_param(void);
        intersection_recognition::Hypothesis generate_publish_variable(bool center_flg, bool back_flg, bool left_flg, bool right_flg,
                                                                        int center_angle, int back_angle, int left_angle, int right_angle);
        float updateDistanceThresh(std::vector<float> *scan);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    private:
        ros::NodeHandle node_;
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher hypothesis_pub_;
};

intersectionRecognition::intersectionRecognition(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &intersectionRecognition::scanCallback, this);
    marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("corridor_visualization", 1);
    hypothesis_pub_ = node_.advertise<intersection_recognition::Hypothesis>("hypothesis", 1);
}

void intersectionRecognition::get_ros_param(void){
    SCAN_HZ = 10;
    robot_frame_ = "base_link";
    node_.getParam("extended_toe_finding/SCAN_HZ", SCAN_HZ);
    node_.getParam("extended_toe_finding/robot_frame", robot_frame_);
}

float intersectionRecognition::updateDistanceThresh(std::vector<float> *scan){
    return std::accumulate(scan->begin(), scan->end(), 0.0) / scan->size();
}

void intersectionRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int num_scan = scan->ranges.size();
    std::vector<double> x(num_scan), y(num_scan);
    std::vector<int> angle_bin(90);
    std::vector<int> toe_index_list;
    std::vector<float> scan_cp(num_scan);
    std::copy(scan->ranges.begin(), scan->ranges.end(), scan_cp.begin());

// skip inf
    int index_prev = 0;
    double last_scan = 0;
    while((std::isnan(scan_cp[index_prev]) || std::isinf(scan_cp[index_prev]) || scan_cp[index_prev] > scan->range_max) && index_prev < num_scan) index_prev++;
    last_scan = scan_cp[index_prev];

    double scan_range;
    for(int i = 0; i < num_scan; i ++) {
        double angle = scan->angle_min + scan->angle_increment * i;
        if(std::isnan(scan_cp[i]) || std::isinf(scan_cp[i]) || scan_cp[i] > scan->range_max){
            scan_range = last_scan;
            scan_cp[i] = last_scan;
        }
        else{
            scan_range = scan_cp[i];
            last_scan = scan_cp[i];
        }
        x[i] = scan_range * cos(angle);
        y[i] = scan_range * sin(angle);
    }

    for(int i = 1; i+10 < num_scan - 1; i ++) {
        double angle = atan2(y[i] - y[i+10-1], x[i] - x[i+10-1]);
        while(angle <       0.0) angle += M_PI_2;
        while(angle >= M_PI_2) angle -= M_PI_2;
        angle_bin[static_cast<int>(angle/M_PI*180)] += 1;
    }

    int max_angle = 0, max_bin = 0;
    max_angle = std::max_element(angle_bin.begin(), angle_bin.end()) - angle_bin.begin();
    max_bin = angle_bin[max_angle];

    double scan_angle = static_cast<double>((max_angle <= 45) ? max_angle : max_angle - 90)/180.0*M_PI;
    int scan_left   = static_cast<int>((scan_angle + M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_center = static_cast<int>((scan_angle          - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_right  = static_cast<int>((scan_angle - M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_back   = static_cast<int>((scan_angle - M_PI   - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;

    float distance_left = std::sqrt(x[scan_left]*x[scan_left] + y[scan_left]*y[scan_left]);
    float distance_center = std::sqrt(x[scan_center]*x[scan_center] + y[scan_center]*y[scan_center]);
    float distance_right = std::sqrt(x[scan_right]*x[scan_right] + y[scan_right]*y[scan_right]);
    float distance_back = std::sqrt(x[scan_back]*x[scan_back] + y[scan_back]*y[scan_back]);

// publish hypothesis of intersection recognition
    distance_thresh = updateDistanceThresh(&scan_cp);
    intersection_recognition::Hypothesis hypothesis;
    if(distance_left > distance_thresh){
        if (scan_left >= 0){
            toe_index_list.push_back(scan_left);
            hypothesis.left_flg = true;
            hypothesis.left_angle = scan_angle + M_PI_2 - scan->angle_min;
        }
    }
    else{
        hypothesis.left_flg = false;
        hypothesis.left_angle = 0;
    }

    if(distance_center > distance_thresh){
        toe_index_list.push_back(scan_center);
        hypothesis.center_flg = true;
        hypothesis.center_angle = scan_angle - scan->angle_min;
   }
    else{
        hypothesis.center_flg = false;
        hypothesis.center_angle = 0;
    }

    if(distance_right > distance_thresh){
        if (scan_right < num_scan){
            toe_index_list.push_back(scan_right);
            hypothesis.right_flg = true;
            hypothesis.right_angle = scan_angle - M_PI_2 - scan->angle_min;
        }
    }
    else{
        hypothesis.right_flg = false;
        hypothesis.right_angle = 0;
    }

    if(distance_back > distance_thresh){
        toe_index_list.push_back(scan_back);
        hypothesis.back_flg = true;
        hypothesis.back_angle = scan_angle - M_PI - scan->angle_min;
    }
    else{
        hypothesis.back_flg = false;
        hypothesis.back_angle = 0;
    }

// publish hypothesis of intersection recognition
    hypothesis_pub_.publish(hypothesis);

// publish line for rviz
    visualization_msgs::MarkerArray marker_line;
    marker_line.markers.resize(toe_index_list.size());
    geometry_msgs::Point linear_start;
    geometry_msgs::Point linear_end;
    double line_lifetime = 1 / static_cast<double>(SCAN_HZ);

    for(int i = 0; i < toe_index_list.size(); i++){
        marker_line.markers[i].header.frame_id = robot_frame_;
        marker_line.markers[i].header.stamp = ros::Time::now();
        marker_line.markers[i].ns = "toe";
        marker_line.markers[i].id = i;
        marker_line.markers[i].lifetime = ros::Duration(line_lifetime);

        marker_line.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        marker_line.markers[i].action = visualization_msgs::Marker::ADD;

        linear_start.x = 0;
        linear_start.y = 0;
        linear_start.z = 0;

        double range = scan_cp[toe_index_list[i]];
        linear_end.x = range;
        linear_end.y = 0;
        linear_end.z = 0;

        marker_line.markers[i].points.resize(2);
        marker_line.markers[i].points[0] = linear_start;
        marker_line.markers[i].points[1] = linear_end;

        marker_line.markers[i].scale.x = 0.1;

        float yaw_rad = scan->angle_min + toe_index_list[i] * scan->angle_increment;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw_rad);
        geometry_msgs::Quaternion geometry_quat;
        geometry_quat = tf2::toMsg(quat);
        marker_line.markers[i].pose.orientation = geometry_quat;

        marker_line.markers[i].color.r = 0.0f;
        marker_line.markers[i].color.g = 1.0f;
        marker_line.markers[i].color.b = 0.0f;
        marker_line.markers[i].color.a = 1.0f;
    }
    marker_pub_.publish(marker_line);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "extended_toe_finding");
    intersectionRecognition recognition;
    recognition.get_ros_param();
    ros::Rate loop_rate(recognition.SCAN_HZ);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
