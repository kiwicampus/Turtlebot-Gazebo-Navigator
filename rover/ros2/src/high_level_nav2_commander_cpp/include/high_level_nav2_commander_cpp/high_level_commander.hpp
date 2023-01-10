#ifndef HIGH_LEVEL_COMMANDER_HPP_
#define HIGH_LEVEL_COMMANDER_HPP_

#include <string>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class Nav2CommanderNode : public rclcpp::Node
{
   public:
    Nav2CommanderNode(const std::string& node_name);
};


#endif