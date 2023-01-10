# Section 2 - Navigation with high level interfaces

## General Objective

In this section your task if to make a node that can send the turtlebot to all the waypoints in [waypoints.yaml](../rover/ros2/src/tb_bringup/config/waypoints.yaml). Upon arriving to each waypoint the node must play the audio track specified in its `track` field and wait the time in its `wait_time` field. While the robot is navigating the node should print to the terminal the estimate time remaining to get to the goal.

## Guidelines

1. Your first task is to choose which language you will use. Remember that you can solve this section in either Python or C++ but Python will give you less points: in that case your functionality score will be multiplied by 0.7 but the answers to the tech questions will not be affected.

    If you choose to use python you should work on the [high_level_nav2_commander_py package](../rover/ros2/src/high_level_nav2_commander_py/), if you choose to use C++ you should work on the [high_level_nav2_commander_cpp package](../rover/ros2/src/high_level_nav2_commander_cpp/)

2. As was said in the main readme, in this section you have to create all the code from the ground up, though some files are already created. You can check the [ROS2 documentation on packages](https://docs.ros.org/en/dashing/Tutorials/Creating-Your-First-ROS2-Package.html) to understand better the file structure. 

    You should write your code in [high_level_commander_node.py](../rover/ros2/src/high_level_nav2_commander_py/high_level_nav2_commander_py/high_level_commander_node.py) for python and [high_level_commander_node.cpp](../rover/ros2/src/high_level_nav2_commander_cpp/src/high_level_commander_node.cpp) and [high_level_commander.hpp](../rover/ros2/src/high_level_nav2_commander_cpp/include/high_level_nav2_commander_cpp/high_level_commander.hpp) for C++. In both cases a very simple node is already there for you, so you can test that everything works fine by activating the nodes in the [nodes_launch.yaml](../configs/nodes_launch.yaml) file and checking that the nodes output `Hi! I'm the nav2 commander node!`

3. [15%] One of the first things you should do is reading [waypoints.yaml](../rover/ros2/src/tb_bringup/config/waypoints.yaml). You can achieve this using C++'s [yaml](https://github.com/jbeder/yaml-cpp/wiki/Tutorial) or python's [pyYAML](https://pyyaml.org/wiki/PyYAMLDocumentation) which are already included in the docker container. You can directly paste the route to the yaml file, but you will get **+5% extra** if you use [ament_index](https://docs.ros2.org/latest/api/ament_index_cpp/index.html)


4. [15%] After reading the yaml you should create an action client for the [/navigate_to_pose](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action server offered by nav2. Action servers are a core component of nav2, so we encourage you to read more about them [here](https://navigation.ros.org/concepts/index.html#action-server). Also you can check how to interact with them [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client)

5. [15%] Since you will have to play sounds you should also create a publisher for the `/device/speaker/command` topic. If you check out the [speaker module](../rover/ros2/src/interfaces/src/modules/speaker.cpp) you will see this topic expects a string message with the name of the track you wish to play.

6. [15%] As you may have realized, the [NavigateToPose](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action has a field in its feedback called `estimated_time_remaining`. Since you are expected to print that to the terminal you should create a callback function for the action server's feedback that accesses to that field and prints it. You can check out a feedback function callback [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client) for C++ and [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#getting-feedback) for python.

7. [40%] After achieving the above, its time to make everything work together. Create a function to successively call the `/navigate_to_pose` action server and play the specified sound when the robot arrives to each waypoint. This time the robot does not have to spin or wait each at the end of the trajectory. Make sure the time remaining to reach is goal is printed to the terminal.

8. Write a readme file for the package you did and add docstrings to each function. Remember documentation will be assessed as well