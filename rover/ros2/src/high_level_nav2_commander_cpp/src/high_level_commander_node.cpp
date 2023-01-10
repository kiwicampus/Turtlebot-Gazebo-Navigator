#include "high_level_nav2_commander_cpp/high_level_commander.hpp"

Nav2CommanderNode::Nav2CommanderNode(const std::string& node_name) : Node(node_name, rclcpp::NodeOptions())
{
  std::cout << "Hi! I'm the nav2 commander node!" << std::endl;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto nav2_commander = std::make_shared<Nav2CommanderNode>("nav2_commander_cpp");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nav2_commander);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}