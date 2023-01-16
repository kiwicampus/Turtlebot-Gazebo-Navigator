cmake_minimum_required(VERSION 3.8)
project(nav2_cool_goal_checker)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
)

add_library(nav2_cool_goal_checker SHARED plugins/nav2_cool_goal_checker.cpp)
ament_target_dependencies(nav2_cool_goal_checker ${dependencies})

install(TARGETS nav2_cool_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(nav2_cool_goal_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core nav2_cool_goal_checker.xml)

ament_package()

# So you spent some time reading how the CMakeLists of this package works. Create
# an issue in the main project repository with the name "Go Kiwibot!". You will get an
# additional 1% if you are the first one to do so.