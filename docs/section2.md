# Section 2 - Navigation with high level interfaces

## General Objective

In this section your task if to make a node that can send the turtlebot to all the waypoints in [waypoints.yaml](../rover/ros2/src/tb_bringup/config/waypoints.yaml). The navigation task should occur when you publish through the topic `/navigation/start`. Upon arriving to each waypoint the node must play the audio track specified in its `track` field. While the robot is navigating the node should print to the terminal the estimate time remaining to get to the goal. Navigation should be cancelled when publishing through the topic `/navigation/stop`

## Guidelines - Problem Solution [70% + 30% EXTRA]

1. Your first task is to choose which language you will use. Remember that you can solve this section in either Python or C++ but Python will give you less points: in that case your functionality score will be multiplied by 0.7 but the answers to the tech questions will not be affected.

    If you choose to use python you should work on the [high_level_nav2_commander_py package](../rover/ros2/src/high_level_nav2_commander_py/), if you choose to use C++ you should work on the [high_level_nav2_commander_cpp package](../rover/ros2/src/high_level_nav2_commander_cpp/)

2. As was said in the main readme, in this section you have to create all the code from the ground up, though some files are already created. You can check the [ROS2 documentation on packages](https://docs.ros.org/en/dashing/Tutorials/Creating-Your-First-ROS2-Package.html) to understand better the file structure. 

    You should write your code in [high_level_commander_node.py](../rover/ros2/src/high_level_nav2_commander_py/high_level_nav2_commander_py/high_level_commander_node.py) for python and [high_level_commander_node.cpp](../rover/ros2/src/high_level_nav2_commander_cpp/src/high_level_commander_node.cpp) and [high_level_commander.hpp](../rover/ros2/src/high_level_nav2_commander_cpp/include/high_level_nav2_commander_cpp/high_level_commander.hpp) for C++. In both cases a very simple node is already there for you, so you can test that everything works fine by activating the nodes in the [nodes_launch.yaml](../configs/nodes_launch.yaml) file and checking that the nodes output `Hi! I'm the nav2 commander node!`

    For testing all your code in this section you should set in the [nodes_launch.yaml](../configs/nodes_launch.yaml) the nav2 stack to work in high level mode and you should turn ON the interfaces and your commander node, be it python or C++

3. [8% + 5%EXTRA] One of the first things you should do is reading [waypoints.yaml](../rover/ros2/src/tb_bringup/config/waypoints.yaml). You can achieve this using C++'s [yaml](https://github.com/jbeder/yaml-cpp/wiki/Tutorial) or python's [pyYAML](https://pyyaml.org/wiki/PyYAMLDocumentation) which are already included in the docker container and the build system of the nodes. You can directly paste the route to the yaml file, but you will get **+5% extra** if you use [ament_index](https://docs.ros2.org/latest/api/ament_index_cpp/index.html).


4. [5%] After reading the yaml you should create an action client for the [/navigate_to_pose](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action server offered by nav2. Action servers are a core component of nav2, so we encourage you to read more about them [here](https://navigation.ros.org/concepts/index.html#action-server). Also you can check how to interact with them [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client).

5. [3%] Since you will have to play sounds you should also create a publisher for the `/device/speaker/command` topic. If you check out the [speaker module](../rover/ros2/src/interfaces/src/modules/speaker.cpp) you will see this topic expects a string message with the name of the track you wish to play.

6. [3%] You will have to execute the navigation routine on a topic callback. For that reason you should create a subscriber to the `/navigation/start` topic which should be of type `std_msgs/msg/Bool`. 

7. [3%] You will have to cancel the navigation routine on a topic callback. For that reason you should create a subscriber to the `/navigation/stop` topic which should be of type `std_msgs/msg/Bool`. 

8. [8% + 10%EXTRA] As you may have realized, the [NavigateToPose](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) action has a field in its feedback called `estimated_time_remaining`. Since you are expected to print that to the terminal you should create a callback function for the action server's feedback that accesses to that field and prints it. You can check out a feedback function callback [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client) for C++ and [here](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#getting-feedback) for python. Though its not mandatory we heavily recommend you create callback for the goal status and the result as well, you will get **+5% extra** if you do

9. [35% + 10%EXTRA] After achieving the above, its time to make everything work together. Create a function to successively call the `/navigate_to_pose` action server and play the specified sound when the robot arrives to each waypoint. This function should run each time a message is received through `/navigation/start`. This time the robot does not have to spin or wait each at the end of the trajectory. Make sure the time remaining to reach is goal is printed to the terminal.

    As you may have guessed, this is the trickiest part of the section. While developing your solution you may find that some of your callbacks are not executing as you think they should (if you do you are probably on the right pathway!). This is an issue you may frequently encounter when writing ROS2 code and has to do with the way executors and callback groups are set by default, to understand why this happens and how to fix it you may use [this document](https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255). You will get **+10% extra** if you make a nice use of callback groups

9. [5% + 5%EXTRA] Just one final step. Create a function to cancel the `/navigate_to_pose` action if its running each time a message is received through `/navigation/stop`. You will get **+5% extra** if you cancel the entire navigation instead of a single waypoint.

10. Write a readme file for the package you did and add docstrings to each function. Remember documentation will be assessed as well

### Final Result

The following gif shows what you should see if you did everything well

![point2_solution](https://user-images.githubusercontent.com/71234974/211886931-238a87ef-2e91-4010-accc-a2eade6d0dd9.gif)


## Questions - [30% + 20% EXTRA]

Before answering the questions take some time to read [how behavior trees work in nav2](https://navigation.ros.org/behavior_trees/index.html) and how the [/navigate_to_pose BT](https://github.com/ros-planning/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml) is [configured](https://navigation.ros.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html#navigate-to-pose-with-replanning-and-recovery).

1. [2%]_In section 1 you called the `/follow_path` action from the controller_server manually. Which node in the BT is calling it now?_
1. [2%]_In section 1 you called the `/compute_path_to_pose` action from the planner_server manually manually. Which node in the BT is calling it now?_
1. [2%]_In section 1 you called the `/spin` action from the behavior_server manually manually. Which node in the BT is calling it now?_
1. [3%]_What's the different between an action BT node and an action server? Do all action BT nodes call action servers?_
1. [5% - EXTRA]_Do you think is advantageous to have a BT call the low level action servers instead of calling them manually? why?_

If you didn't read the [document on executors and callback groups](https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255) do it now to answer yes or no the following questions. Imagine you have a node that subscribes to two different topics, then

1. [1%] _Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor but they belong to the same MutuallyExclusiveCallbackGroup?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor but they belong to different MutuallyExclusiveCallbackGroups?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor but they belong to the same ReentrantCallbackGroup?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor but they belong to the same MutuallyExclusiveCallbackGroup?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor but they belong to different MutuallyExclusiveCallbackGroups?_
1. [1%] _Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor but they belong to the same ReentrantCallbackGroup?_

One final round on nav2's general concepts. You may read it's [documentation](https://navigation.ros.org/concepts/index.html) if you haven't by now. You may also find it useful to read [REP105](https://www.ros.org/reps/rep-0105.html).

1. [3%]_What's the purpose of the [nav2_params.yaml](../rover/ros2/src/tb_bringup/params/nav2_params.yaml) file?_
1. [5% - EXTRA]_What's the task of amcl? Why does't the BT interact with it while it does with most other servers?_
1. [5%]_What reference frames in REP105 are needed for nav2 to work? which of those is attached to the robot_
1. [5% - EXTRA]_Why do you think we have `map` and `odom` instead of a single frame?_
1. [5%]_Why do you think nav2 needs two costmaps instead of just one?_
1. [5% - EXTRA]_What's a costmap layer? what do we mean when we say nav2 costmaps are layered? Which kind of costmap layer would you use to populate the costmap with data coming from a 2D lidar?_