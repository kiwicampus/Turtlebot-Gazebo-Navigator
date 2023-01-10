cmake_minimum_required(VERSION 3.8)
project(high_level_nav2_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  yaml-cpp
)

add_executable(high_level_commander_node src/high_level_commander_node.cpp)
target_include_directories(high_level_commander_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(high_level_commander_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(high_level_commander_node
  yaml-cpp
)

install(TARGETS high_level_commander_node
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(high_level_commander_node
  ${DEPENDENCIES}
)


ament_package()
