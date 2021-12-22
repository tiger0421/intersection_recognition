#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "intersection_recognition/Scenario.h"
#include "intersection_recognition/Hypothesis.h"
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        int SCENARIO_MAX = 10;
        void getRosParam(void);
        bool compareScenarioAndHypothesis(const intersection_recognition::Hypothesis::ConstPtr& hypothesis);
        void loadNextScenario(void);
        void updateLastNode(bool center_flg, bool back_flg, bool left_flg, bool right_flg);
        bool compareLastNodeAndCurrentNode(const intersection_recognition::Hypothesis::ConstPtr& hypothesis);

        void turnFinishFlgCallback(const std_msgs::Bool::ConstPtr& turn_finish_flg);
        void hypothesisCallback(const intersection_recognition::Hypothesis::ConstPtr& hypothesis);
        void emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg);
        bool scenarioCallback(intersection_recognition::Scenario::Request& scenario,
                              intersection_recognition::Scenario::Response& res);
     private:
        ros::NodeHandle node_;
        ros::Publisher emergency_stop_flg_pub_;
        ros::Publisher rotate_rad_pub_;
        ros::Subscriber hypothesis_sub_;
        ros::Subscriber emergency_stop_flg_sub_;
        ros::Subscriber turn_finish_flg_sub_;
        ros::ServiceServer scenario_server_;

        std::list<std::string> target_type_;
        std::list<std::int16_t> target_order_;
        std::list<std::string> target_direction_;
        std::list<std::string> target_action_;

        std::list<std::string>::iterator target_type_itr_begin_;
        std::list<std::int16_t>::iterator target_order_itr_begin_;
        std::list<std::string>::iterator target_direction_itr_begin_;
        std::list<std::string>::iterator target_action_itr_begin_;

        int scenario_num_ = 0;
        int scenario_progress_cnt_ = 0;
        int scenario_order_cnt_ = 0;
        int reach_target_type_cnt_ = 0;
        int reach_different_type_cnt_ = 0;
        int reach_target_type_cnt_margin_ = 6;
        int reach_different_type_cnt_margin_ = 6;
        float rotate_rad_ = 0.0;
        bool turn_flg_ = false;
        bool change_node_flg_ = false;
        bool satisfy_conditions_flg_ = false;
        bool emergency_stop_flg_ = true;
        bool request_update_last_node_flg = true;
        std_msgs::Float32 rotate_rad_for_pub_;
        intersection_recognition::Hypothesis last_node_;
};

cmdVelController::cmdVelController(){
    emergency_stop_flg_pub_ = node_.advertise<std_msgs::Bool>("emergency_stop_flg", 1, false);
    rotate_rad_pub_ = node_.advertise<std_msgs::Float32>("rotate_rad", 1, false);

    turn_finish_flg_sub_ = node_.subscribe<std_msgs::Bool> ("turn_finish_flg", 1, &cmdVelController::turnFinishFlgCallback, this);
    hypothesis_sub_ = node_.subscribe<intersection_recognition::Hypothesis> ("hypothesis", 1, &cmdVelController::hypothesisCallback, this);
    emergency_stop_flg_sub_ = node_.subscribe<std_msgs::Bool> ("emergency_stop_flg", 1, &cmdVelController::emergencyStopFlgCallback, this);
    scenario_server_ = node_.advertiseService("scenario", &cmdVelController::scenarioCallback, this);

    updateLastNode(false, false, false, false);
    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCENARIO_MAX = 10;
    node_.getParam("scenario_executor/scenario_max", SCENARIO_MAX);
}

bool cmdVelController::compareScenarioAndHypothesis(const intersection_recognition::Hypothesis::ConstPtr& hypothesis){
    std::string target_type = *std::next(target_type_itr_begin_, scenario_progress_cnt_);
    std::string target_direction = *std::next(target_direction_itr_begin_, scenario_progress_cnt_);

// check "straight_road"
    if(target_type == "straight_road"){
        if(hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && !hypothesis->right_flg){
            return true;
        }
    }

// check 3_way_left and 3_way_right when 3_way is designated by scenario
    if(target_type == "3_way"){
    // 3_way_left
        if(hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && !hypothesis->right_flg){
            return true;
        }
    // 3_way_right
        if(hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && hypothesis->right_flg){
            return true;
        }
     // 3_way_center
        if(!hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && hypothesis->right_flg){
            return true;
        }
   }

// check "end"(= 突き当り)
    if(target_type == "end"){
    // dead_end
        if(!hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && !hypothesis->right_flg){
            return true;
        }
    // right
        if(!hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && hypothesis->right_flg){
            return true;
        }
    // left
        if(!hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && !hypothesis->right_flg){
            return true;
        }
    // 3_way_center
        if(!hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && hypothesis->right_flg){
            return true;
        }
    }

// check "corridor"(ex, 交差点， 通路)
    if(target_type == "corridor"){
        if(target_direction == "left"){
            if(hypothesis->left_flg){
                return true;
            }
        }
        else if(target_direction == "right"){
            if(hypothesis->right_flg){
                return true;
            }
        }
    // if target_direction is not designated, do below
        else{
            if(hypothesis->left_flg || hypothesis->right_flg){
                return true;
            }
        }
    }
    return false;
}

void cmdVelController::loadNextScenario(void){
    std::string action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);

// stop robot
    emergency_stop_flg_ = true;

    if(action == "stop"){
        ROS_INFO("Robot gets a goal");
        std_msgs::Bool emergency_stop_flg_for_pub;
        emergency_stop_flg_for_pub.data = emergency_stop_flg_;
        emergency_stop_flg_pub_.publish(emergency_stop_flg_for_pub);
    }
    else{
        ROS_INFO("Execute next action(%s)", action.c_str());
        emergency_stop_flg_ = false;
        change_node_flg_ = false;
        if(action.find("turn") != std::string::npos){
            turn_flg_ = true;

            //if(action.find("left")){
            if(action == "turn_left"){
                rotate_rad_for_pub_.data = M_PI_2;
                std::cout <<"L" << rotate_rad_for_pub_.data << std::endl;
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
              rotate_rad_for_pub_.data = -M_PI_2;
                std::cout <<"R" << rotate_rad_for_pub_.data << std::endl;
            }
            else{
                rotate_rad_for_pub_.data = M_PI;
            }
            rotate_rad_pub_.publish(rotate_rad_for_pub_);
        }
    }
}

void cmdVelController::updateLastNode(bool center_flg, bool back_flg, bool left_flg, bool right_flg){
    ROS_INFO("update last node");
    ROS_INFO("last node is front(%d) back(%d) left(%d) right(%d)", static_cast<int>(center_flg), static_cast<int>(back_flg),
                                                                      static_cast<int>(left_flg), static_cast<int>(right_flg));
    last_node_.center_flg = center_flg;
    last_node_.back_flg = back_flg;
    last_node_.left_flg = left_flg;
    last_node_.right_flg = right_flg;
}

bool cmdVelController::compareLastNodeAndCurrentNode(const intersection_recognition::Hypothesis::ConstPtr& hypothesis){
    if(last_node_.center_flg == hypothesis->center_flg && last_node_.back_flg == hypothesis->back_flg &&
        last_node_.left_flg == hypothesis->left_flg && last_node_.right_flg == hypothesis->right_flg){
            return true;
        }
    else{
        return false;
    }
}

void cmdVelController::hypothesisCallback(const intersection_recognition::Hypothesis::ConstPtr& hypothesis){
    if(! emergency_stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(hypothesis->center_flg, hypothesis->back_flg, hypothesis->left_flg, hypothesis->right_flg);
            request_update_last_node_flg = false;
        }
        if(! turn_flg_){
            if(change_node_flg_){
                satisfy_conditions_flg_ = compareScenarioAndHypothesis(hypothesis);
                if(satisfy_conditions_flg_){
                    ROS_INFO("find target node");
                    reach_target_type_cnt_++;
                    if(reach_target_type_cnt_margin_ <= reach_target_type_cnt_){
                        reach_target_type_cnt_ = 0;
                        scenario_order_cnt_++;
                        change_node_flg_ = false;
                        updateLastNode(hypothesis->center_flg, hypothesis->back_flg, hypothesis->left_flg, hypothesis->right_flg);
                        int order = *std::next(target_order_itr_begin_, scenario_progress_cnt_);
                        if(order <= scenario_order_cnt_){
                            ROS_INFO("Robot reaches target_node!!");
                            scenario_order_cnt_ = 0;
                            scenario_progress_cnt_++;
                            loadNextScenario();
                        }
                    }
                }
                else{
                    reach_target_type_cnt_ = 0;
                }
            }
            else{
                if(! compareLastNodeAndCurrentNode(hypothesis)){
                    reach_different_type_cnt_++;
                    if(reach_different_type_cnt_margin_ <= reach_different_type_cnt_){
                        reach_different_type_cnt_ = 0;
                        updateLastNode(hypothesis->center_flg, hypothesis->back_flg, hypothesis->left_flg, hypothesis->right_flg);
                        change_node_flg_ = true;
                    }
                }
                else{
                    reach_different_type_cnt_ = 0;
                }
            }
            rotate_rad_for_pub_.data = 0.00;
        }
    }
}

void cmdVelController::turnFinishFlgCallback(const std_msgs::Bool::ConstPtr& turn_finish_flg){
    ROS_INFO("finish turn");
    turn_flg_ = false;
    scenario_progress_cnt_++;
    loadNextScenario();
    request_update_last_node_flg = true;
    change_node_flg_ = false;
}

void cmdVelController::emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg){
    emergency_stop_flg_ = emergency_stop_flg->data;
}

bool cmdVelController::scenarioCallback(intersection_recognition::Scenario::Request& scenario,
                                        intersection_recognition::Scenario::Response& res){
    scenario_num_++;
    target_type_.push_back(scenario.type);
    target_order_.push_back(scenario.order);
    target_direction_.push_back(scenario.direction);
    target_action_.push_back(scenario.action);

    std::string last_action = *std::next(target_action_.begin(), scenario_num_ - 1);
// check whether scenario is loaded
    if(last_action == "stop"){
        ROS_INFO("Completed loading scenario");

        target_type_itr_begin_ = target_type_.begin();
        target_order_itr_begin_ = target_order_.begin();
        target_direction_itr_begin_ = target_direction_.begin();
        target_action_itr_begin_ = target_action_.begin();

        emergency_stop_flg_ = false;
        std_msgs::Bool emergency_stop_flg_for_pub;
        emergency_stop_flg_for_pub.data = emergency_stop_flg_;
        emergency_stop_flg_pub_.publish(emergency_stop_flg_for_pub);
        loadNextScenario();
    }
    else{
        emergency_stop_flg_ = true;
        std_msgs::Bool emergency_stop_flg_for_pub;
        emergency_stop_flg_for_pub.data = emergency_stop_flg_;
        emergency_stop_flg_pub_.publish(emergency_stop_flg_for_pub);
    }


//  debug
    std::cout << "####################################" << std::endl;
    std::cout << "type is " << scenario.type << std::endl;
    std::cout << "order is "  << std::hex << scenario.order << std::endl;
    std::cout << "direction is " << scenario.direction << std::endl;
    std::cout << "action is " << scenario.action << std::endl;
    std::cout << "####################################" << std::endl;

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "scenario_executor");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
