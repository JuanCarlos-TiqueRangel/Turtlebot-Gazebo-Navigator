#ifndef HIGH_LEVEL_COMMANDER_HPP_
#define HIGH_LEVEL_COMMANDER_HPP_

#include <string>

#include <iostream>
#include <fstream>

/*Library to get the yaml file name*/
#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_resource.hpp>

//#include <ament_index_cpp/get_search_directories.hpp>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class Nav2CommanderNode : public rclcpp::Node
{
    public:
        Nav2CommanderNode(const std::string& node_name);

        /*Set variables and interface to use NavigateToPose Path*/
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    private:
        void send_goal(const geometry_msgs::msg::PoseStamped &goal);
        void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);

        void test_interface_server();
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_audio_;
};


#endif