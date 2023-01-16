# Section 1: Low-level Interaction
In this section you will be asked to make the robot autonomously navigate using a low-level interaction with Nav2's servers. To do so, some interfaces will be defined to call the action servers in Nav2's controller, planner, and behavior servers. Then, those interfaces will be used to build a complex behavior consisting of making the robot autonomously navigate between three waypoints.

## Running the code
This section of the code is organized so you start building up your solution from the ground-up. On each part, you will be asked to run some test nodes that will ensure that each interface is working as expected. Then, when you reach the final part of this section, you will be running everything. As you develop the interfaces, we recommend you to set the `NODE_LOW_LEVEL_COMMANDER` to `0` in the [`nodes_launch.yaml`](../configs/nodes_launch.yaml). Then, when you reach the final part, you can set `NODE_LOW_LEVEL_COMMANDER` to `1` to run the node without having multiple consoles at the same time.

## [20% + 10% Extra] Use the controller server to move the robot (20% + 10% Extra)
Before you get hands-on with this part, we recommend you to take a look to the following references:
- [Nav2's Navigation Concepts](https://navigation.ros.org/concepts/index.html)
- [Controller Server Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
- [Navigation Plugins](https://navigation.ros.org/plugins/index.html#)

You will be working in the [`controller_interface.py`](../rover/ros2/src/low_level_pkg/low_level_pkg/util/controller_interface.py) file for this part. The goal of this part is to create the interface for the controller server. The implementation of this part is worth 15% plus an extra 5%. Questions are worth 5%, and there are some additional questions that can give you an extra 5%.

1. [2.0%] Import the appropriate action definition from the `nav2_msgs` package.
1. [3.0%] Fill the remaining fields in the action client initialization. You will need to pass the definition you imported in the previous point, and the name of the action.
1. [5.0%] Create the goal instance in the `call_action_client` function in the `ControllerInterface` class. The target poses are passed as a parameter to that function.
1. [5.0%] Fill the `read_path` function. You should read the `path.yaml` file that is located in the `tb_bringup` package.
1. [0.0%] Run the `controller_interface_test` with `ros2 run low_level_pkg controller_interface_test` after sourcing your workspace to see if the robot moves in the simulator.
1. [5.0%][Extra] Make the robot drive in a circular path. *Solution:* create the waypoints out of circular coordinates. Beyond that, I would like to take a look on the organization of the code. Are they overriding the `read_path` function or creating a new one?

![controller_interface](https://user-images.githubusercontent.com/14006555/212355450-85ebd92a-8b83-451d-99c4-9d7769b91fa4.gif)

### [5.0%] Conceptual Questions
Please solve the following questions:
1. [0.5%] *What is the difference between an action, a topic and a service in ROS?*
1. [0.5%] *What is the use of the controller server within the context of Nav2?*
1. [0.5%] *What is the name and type of the controller server action?*
1. [1.0%] *What are the fields of the controller server action and what is their purpose?*
1. [0.5%] *Which costmap is used by the controller server?*
1. [1.0%] *Describe the controller, progress checker, and goal checker in the context of Nav2's controller server*
1. [1.0%] *What is a future in the context of ROS asynchronous calls?*

### [5.0% Extra] Additional Questions
Solve the following questions if you want to get some extra points:
1. [1.0%] *Name two or three additional examples for controller servers:*
1. [0.5%] *Which is the default controller that is used in Nav2's controller server?*
1. [0.5%] *Name three of the available controller plugins:*
1. [1.0%] *What should I modify if my speed control topic has the name `/kiwi/output_cmd` ?*
1. [1.0%] *What is the purpose of using the `spin_until_future_complete` function in the `call_action_client`?*
1. [1.0%] *Why are two (2) `future` instances used in the `call_action_client` function?*


## [20% + 10% Extra] Use the planner server to move the robot to a waypoint
Before you get hands-on with this part, we recommend you to take a look to the following references:
- [Nav2's Navigation Concepts](https://navigation.ros.org/concepts/index.html)
- [Planner Server Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-planner-server.html)
- [Navigation Plugins](https://navigation.ros.org/plugins/index.html#)

You will be working in the [`planner_interface.py`](../rover/ros2/src/low_level_pkg/low_level_pkg/util/planner_interface.py) file for this part. The goal of this part is to create the interface for the planner server. The implementation of this part is worth 15% plus an extra 5%. Questions are worth 5%, and there are some additional questions that can give you an extra 5%.

1. [1.0%] Import the appropriate action definition from the `nav2_msgs` package.
1. [1.0%] Fill the remaining fields in the action client initialization. You will need to pass the definition you imported in the previous point, and the name of the action.
1. [10%] Create the goal instance in the `call_action_client` function in the `PlannerInterface` class. You will need to define the function parameters with the goal fields that are used in the planner server action. Use those parameters to initialize the goal.
1. [3.0%] Create a test function to read the `waypoints.yaml` waypoints from the `tb_bringup` package. Use your function to get the coordinates from two waypoints in the file. Take into account the message type that is used to pass the coordinates to the planner server.
1. [0.0%] Open Rviz and configure it to show the path that is published from the planner server.
1. [0.0%] Run the `planner_interface_test` node with `ros2 run low_level_pkg planner_interface_test` after sourcing your workspace so the path is shown in Rivz.
1. [5.0%][Extra] Use a different planner to calculate the path. Measure the path distance and how long does it take for the planner server to calculate the path. Select the path whose length is the shortest.

![planner_interface](https://user-images.githubusercontent.com/14006555/212355521-62c48315-5dc0-4f0a-b5d6-84b69f131a89.gif)

### [5%] Conceptual Questions
Please solve the following questions:
1. [1.0%] *Which field of the planner server action should be set to use a specific planner?*
1. [1.0%] *What is the information that is provided in the planner server action feedback?*
1. [1.0%] *Which costmap is used by the planner server?*
1. [1.0%] *Is sensor data being used by the planner server? If so, name a topic that is used*
1. [1.0%] *Name three (3) planners that could be used by Nav2:*

### [5% Extra] Additional Questions
Solve the following questions if you want to get some extra points:
1. [2.0%] *Which is the error code for an invalid planner on a planner server action call?*
1. [1.0%] *What would happen if you plan a path toward a goal that is located outside of the costmap?*
1. [1.0%] *What would happen if you plan a path toward a goal that is within the costmap but unreachable by the robot?*
1. [1.0%] *What would happen if you plan a path toward a goal that is the same as the start pose:*

## [20% + 5% Extra] Use behavior server to make the robot spin on its goal
Before you get hands-on with this part, we recommend you to take a look to the following references:
- [Nav2's Navigation Concepts](https://navigation.ros.org/concepts/index.html)
- [Behavior Server Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-behavior-server.html)
- [Navigation Plugins](https://navigation.ros.org/plugins/index.html#)

You will be working in the [`behavior_interface.py`](../rover/ros2/src/low_level_pkg/low_level_pkg/util/behavior_interface.py) file for this part. The goal of this part is to create the interface for the behavior server. The implementation of this part is worth 15% plus an extra 5%. Questions are worth 5%.

1. [1.0%] Import the appropriate action definitions for spinning and waiting from the `nav2_msgs` package.
1. [1.0%] Initialize the actions in the class constructor.
1. [13%] Define the `call_spin_action_client` and `call_wait_action_client` functions in the `BehaviorInterface` class.
1. [0.0%] Call the functions you created in the testing function. You should make the robot spin, then wait for a couple of seconds, and spin again.
1. [0.0%] Run the `behavior_interface_test` node with `ros2 run low_level_pkg planner_interface_test` after sourcing your workspace and check that the robot spins and waits.
1. [5.0%][Extra] Add the required attributes and functions so you can use the backup action.

![behavior_interface](https://user-images.githubusercontent.com/14006555/212353298-9c253284-2314-4677-9a85-c45fb76b90c0.gif)

### [5%] Conceptual Questions
Please solve the following questions:
1. [1.0%] *What is the main reason the behavior server exists in Nav2?*
1. [1.0%] *Is playing an alert sound valid as a behavior within the context of the behavior server?*
1. [1.0%] *What is the purpose of the `assisted teleop` behavior?*
1. [1.0%] *Which situation might trigger a `Clear Costmap` behavior?* ...
1. [1.0%] *Within the context of Nav2, what does it mean that each behavior has its own API?*

## Custom behavior tree (40% + 25% Extra)
In this section you will be using the interfaces from the last three (3) parts to make the robot autonomously navigate. The robot will be required to navigate to three points, which are defined in the [`waypoints.yaml`](../rover/ros2/src/tb_bringup/config/waypoints.yaml) file. The robot should navigate to the restaurant, then to the customer, and finally to the parking. On each waypoint, the robot will be required to rotate, and wait. Additionally, the robot will play a sound on each waypoint. All of these actions will be built in the [behavior_tree.py](../rover/ros2/src/low_level_pkg/low_level_pkg/behavior_tree.py) file. The implementation of this part is worth 40% plus an extra 25%.
To run this the behavior tree node you can use the commands you used in the previous points or activate it in the [nodes_launch.yaml](../configs/nodes_launch.yaml) file so it gets launched for you by the start script.
1. [2.0%] Create a function that allows you to read the [`waypoints.yaml`](../rover/ros2/src/tb_bringup/config/waypoints.yaml) file. The function should output the coordinates, spin angle, track name, and wait time for each waypoint. Reading the YAML file was done in the previous parts, however, we will be reading more data than the coordinates. You will be evaluated on how scalable you approach is, so take this into consideration.
1. [4.0%] Create a publisher to play the audio files on the interfaces node. You will need to seek the topic name and data type of the topic.
1. [4.0%] Create a service that allows you to start the navigation on its call. Think about the appropriate service definition for this.
1. [20%] Import the interfaces you created in the previous three parts. You should use the functions that were defined to call the action clients. Calling the action clients should enable you to make the robot navigate to the different waypoints in the file. Remember to publish the audio file name when the robot reaches its goal. We expect you to make this scalable too.
1. [10%] Organize your code so the robot navigates to the restaurant, then to the customer and finally to the parking. You will be evaluated on how you pass the waypoints to the code. Ask yourself how would you change the order of the waypoints. If you need to change your code, you might need to improve your approach.
1. [5%][Extra] Create a new waypoint for `maintenance`. Add the coordinates of the waypoint and all of the required information in the [`waypoints.yaml`](../rover/ros2/src/tb_bringup/config/waypoints.yaml). Make the robot navigate to this new waypoint before heading toward the `parking`.
1. [20%][Extra] Suppose there is not a predefined path but it needs to tell the robot where to go at runtime. Implement a way in which we can dynamically pass the next waypoint where the robot should navigate.

![behavior_tree](https://user-images.githubusercontent.com/14006555/212352809-efe8b816-8c91-46f0-b776-63ba6311b758.gif)