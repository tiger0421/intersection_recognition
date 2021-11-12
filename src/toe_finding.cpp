#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <bits/stdc++.h>
#include <vector>
//#include <iostream>

class intersectionRecognition {
     public:
        intersectionRecognition();
        int hz;
        int off_set;
        float epsilon1;
        float epsilon2;
        float epsilon3;
        std::string robot_frame_;
        void get_ros_param(void);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle node_;
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
};

intersectionRecognition::intersectionRecognition(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &intersectionRecognition::scanCallback, this);
        marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_markerarray", 1);
}

void intersectionRecognition::get_ros_param(void){
    hz = 1;
    off_set = 10;
    epsilon1 = 0.25;
    epsilon3 = 0.8;
    robot_frame_ = "base_link";

    sleep(1);
    node_.getParam("toe_finding/scan_hz", hz);
    node_.getParam("toe_finding/scan_off_set", off_set);
    node_.getParam("toe_finding/epsilon1", epsilon1);
    node_.getParam("toe_finding/epsilon3", epsilon3);
    node_.getParam("toe_finding/robot_frame", robot_frame_);
}

void intersectionRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    float delta_h, delta_i, delta_j;
    std::vector<int> toe_index_list;
    int num_cloud = scan->ranges.size();
    float delta_max = 0.001;

// add all peaks to list
    for(int i = 0; i < num_cloud; i += off_set){
        delta_h = scan->ranges[(i - off_set + num_cloud) % num_cloud];
        delta_i = scan->ranges[i];
        delta_j = scan->ranges[(i + off_set) % num_cloud];

        if((delta_i > delta_h) && (delta_i > delta_j)){
            toe_index_list.push_back(i);
            delta_max = std::max(delta_max, scan->ranges[i]);
        }
    }

// remove peak
    float scan_avg = std::accumulate(scan->ranges.begin(), scan->ranges.end(), 0.0) / scan->ranges.size();
    int index = 0;
    std::vector<int>::iterator it = toe_index_list.begin();

    while(it != toe_index_list.end()){
        if((scan->ranges[*it] < scan_avg) || (scan->ranges[*it]/delta_max < this->epsilon1)){
            toe_index_list.erase(it);
        }
        else{
            ++it;
        }
    }

// merge peaks
    epsilon2 = scan->ranges.size() / 8;
    it = toe_index_list.begin();
    int index_prev = 0;
    for(int j = 0; j < toe_index_list.size();){
        if(toe_index_list.size() <= 1) break;
        index = *(it + j);
        index_prev = *(it + ((j - 1 + toe_index_list.size()) % toe_index_list.size()));
        if(abs(index - index_prev) < epsilon2){
            if(scan->ranges[index] > scan->ranges[index_prev]){
                toe_index_list.erase(it + (j-1+toe_index_list.size())%toe_index_list.size());
            }
            else{
                toe_index_list.erase(it + j);
            }
        }
        else{
            ++j;
        }
    }

// remove peak with epsilon3
    float delta_avg = 0;
    
// remove unimportant valley between toe_index_list[-1] and toe_index_list[0]
    if(toe_index_list.size() > 1){
        auto scan_iter_begin = std::next(scan->ranges.begin(), toe_index_list[0]);
        auto scan_iter_end = std::next(scan->ranges.begin(), toe_index_list.back());

        delta_avg = std::accumulate(scan_iter_end, scan->ranges.end(), 0);
        delta_avg += std::accumulate(scan->ranges.begin(), scan_iter_begin, 0);
        delta_avg /= (scan->ranges.size() - toe_index_list.back()) + toe_index_list[0] + 1;

        if((2*delta_avg/(scan->ranges[toe_index_list.back()] + scan->ranges[toe_index_list[0]])) > epsilon3){
            if(scan->ranges[toe_index_list[0]] < scan->ranges[toe_index_list.back()]){
                toe_index_list.erase(toe_index_list.begin());
            }
            else{
                toe_index_list.erase(toe_index_list.begin() + (toe_index_list.size() - 1));
            }
        }
// remove unimportant valley
        for(int i=0; i < toe_index_list.size()-1;){
            scan_iter_begin = std::next(scan->ranges.begin(), toe_index_list[i]);
            scan_iter_end = std::next(scan->ranges.begin(), toe_index_list[i+1]);
            if(2*delta_avg/(scan->ranges[toe_index_list[i]] + scan->ranges[toe_index_list[i+1]]) > epsilon3){
                if(scan->ranges[toe_index_list[i]] < scan->ranges[toe_index_list[i+1]]){
                    toe_index_list.erase(toe_index_list.begin() + i);
                }
                else{
                    toe_index_list.erase(toe_index_list.begin() + i + 1);
                }
            }
            else{
                i++;
            }
        }
    }

// publish line for rviz
    visualization_msgs::MarkerArray marker_line;
    marker_line.markers.resize(toe_index_list.size());
    geometry_msgs::Point linear_start;
    geometry_msgs::Point linear_end;

    linear_start.x = 0;
    linear_start.y = 0;
    linear_start.z = 0;

    double line_lifetime = 1 / static_cast<double>(hz);

    for(int i = 0; i < toe_index_list.size(); i++){
        marker_line.markers[i].header.frame_id = robot_frame_;
        marker_line.markers[i].header.stamp = ros::Time::now();
        marker_line.markers[i].ns = "toe";
        marker_line.markers[i].id = i;
        marker_line.markers[i].lifetime = ros::Duration(line_lifetime);

        marker_line.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        marker_line.markers[i].action = visualization_msgs::Marker::ADD;

        linear_end.x = scan->ranges[toe_index_list[i]];
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
    ros::init(argc, argv, "toe_finding");
    intersectionRecognition recognition;
    recognition.get_ros_param();
    ros::Rate loop_rate(recognition.hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
