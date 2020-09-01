#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <bits/stdc++.h>
#include <vector>

class intersectionRecognition {
     public:
        intersectionRecognition();
        int hz;
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
    hz = 10;
    node_.getParam("extended_toe_finding/scan_hz", hz);
}

void intersectionRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	int max_angle = 0, max_bin = 0;
	int num_scan = scan->ranges.size();
	std::vector<double> x(num_scan), y(num_scan);
	std::vector<int> angle_bin(90);
	std::vector<int> toe_index_list;
	for(int i = 0; i < num_scan; i ++) {
		double angle = scan->angle_min + scan->angle_increment * i;		
		x[i] = scan->ranges[i] * cos(angle);
		y[i] = scan->ranges[i] * sin(angle);
	}
	for(int i = 0; i < num_scan - 1; i ++) {
		double angle = atan2(y[i] - y[i-1], x[i] - x[i-1]);
		while(angle <       0.0) angle += M_PI/2.0;
		while(angle >= M_PI/2.0) angle -= M_PI/2.0;
		angle_bin[(int)(angle/M_PI*180)] += 1;
	}
	for(int i = 0; i < 90; i ++) {
		if(angle_bin[i]  > max_bin) {
			max_angle = i;
			max_bin = angle_bin[i];
		}
	}
	double scan_angle = (double)((max_angle <= 45) ? max_angle : max_angle - 90)/180.0*M_PI;
	int scan_left   = (scan_angle - M_PI/2.0 - scan->angle_min)/scan->angle_increment;
	int scan_center = (scan_angle            - scan->angle_min)/scan->angle_increment;
	int scan_right  = (scan_angle + M_PI/2.0 - scan->angle_min)/scan->angle_increment;
	if (scan_left >= 0) toe_index_list.push_back(scan_left);
	toe_index_list.push_back(scan_center);
	if (scan_right < num_scan) toe_index_list.push_back(scan_right);

	printf("%d\r\n", toe_index_list.size());

// publish line for rviz
    visualization_msgs::MarkerArray marker_line;
    marker_line.markers.resize(toe_index_list.size());
    geometry_msgs::Point linear_start;
    geometry_msgs::Point linear_end;

    for(int i = 0; i < toe_index_list.size(); i++){
        marker_line.markers[i].header.frame_id = "/base_link";
        marker_line.markers[i].header.stamp = ros::Time::now();
        marker_line.markers[i].ns = "toe";
        marker_line.markers[i].id = i;
        marker_line.markers[i].lifetime = ros::Duration(hz);

        marker_line.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        marker_line.markers[i].action = visualization_msgs::Marker::ADD;

        linear_start.x = 0;
        linear_start.y = 0;
        linear_start.z = 0;

		double range = scan->ranges[toe_index_list[i]];        
		linear_end.x = range > 0.01 ? range : 30.0;
        linear_end.y = 0;
        linear_end.z = 0;

        marker_line.markers[i].points.resize(2);
        marker_line.markers[i].points[0] = linear_start;
        marker_line.markers[i].points[1] = linear_end;

        marker_line.markers[i].scale.x = 0.1;

        float yaw_rad = scan->angle_min + toe_index_list[i] * scan->angle_increment;
        tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, yaw_rad);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
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
    ros::Rate loop_rate(recognition.hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

