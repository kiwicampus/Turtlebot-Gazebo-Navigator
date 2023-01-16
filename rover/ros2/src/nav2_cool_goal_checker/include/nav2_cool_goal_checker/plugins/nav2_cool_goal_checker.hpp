#ifndef NAV2_COOL_GOAL_CHECKER__PLUGINS__COOL_GOAL_CHECKER_HPP_
#define NAV2_COOL_GOAL_CHECKER__PLUGINS__COOL_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_cool_goal_checker {

class CoolGoalChecker : public nav2_core::GoalChecker
{
  public:
  CoolGoalChecker();
  // Standard GoalChecker Interface
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_cool_goal_checker

#endif  // NAV2_COOL_GOAL_CHECKER__PLUGINS__COOL_GOAL_CHECKER_HPP_