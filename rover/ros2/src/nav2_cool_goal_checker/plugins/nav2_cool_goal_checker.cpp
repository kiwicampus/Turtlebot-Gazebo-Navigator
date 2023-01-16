#include "nav2_cool_goal_checker/plugins/nav2_cool_goal_checker.hpp"

namespace nav2_cool_goal_checker {

CoolGoalChecker::CoolGoalChecker()
{
}

void CoolGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
}

void CoolGoalChecker::reset()
{
}

bool CoolGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
}

bool CoolGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
}

rcl_interfaces::msg::SetParametersResult
CoolGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
}


}  // namespace nav2_cool_goal_checker

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_cool_goal_checker::CoolGoalChecker, nav2_core::GoalChecker)