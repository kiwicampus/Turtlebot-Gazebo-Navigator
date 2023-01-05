<!-- https://www.makeareadme.com/ -->

<!-- ---------------------------------------------------------------------- -->
# **Gazebo Turtlebot Simulator** (Final Project for Integration Engineer)
<!--
Badges: On some READMEs, you may see small images that convey metadata, such
as whether or not all the tests are passing for the project. You can use Shields
to add some to your README. Many services also have instructions for adding a
badge.
Visuals: Depending on what you are making, it can be a good idea to include
screenshots or even a video (you'll frequently see GIFs rather than actual videos)
Tools like ttygif can help, but check out Asciinema for a more sophisticated method.
-->
<!-- https://shields.io/ -->
![GitHub stars](https://img.shields.io/github/stars/kiwicampus/2D-Test-Track-Planner?style=social)
![GitHub followers](https://img.shields.io/github/followers/kiwicampus?style=social)
![GitHub forks](https://img.shields.io/github/forks/kiwicampus/2D-Test-Track-Planner?label=Fork&style=social)
![Twitter Follow](https://img.shields.io/twitter/follow/kiwibot?style=social)
<img src="https://user-images.githubusercontent.com/43115782/108216251-41a1e600-7100-11eb-98ea-74d97d4ed00d.jpg" alt="kiwi_banner" width="1200">

![GitHub repo size](https://img.shields.io/github/repo-size/kiwicampus/2D-Test-Track-Planner?label=Repo%20Size)
![GitHub issues](https://img.shields.io/github/issues-raw/kiwicampus/2D-Test-Track-Planner?label=Open%20Issues)
![GitHub pull requests](https://img.shields.io/github/issues-pr-raw/kiwicampus/2D-Test-Track-Planner?label=Open%20Pull%20Request)
![GitHub language count](https://img.shields.io/github/languages/count/kiwicampus/2D-Test-Track-Planner?label=Languages)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![GitHub contributors](https://img.shields.io/github/contributors/kiwicampus/2D-Test-Track-Planner)
![GitHub last commit](https://img.shields.io/github/last-commit/kiwicampus/2D-Test-Track-Planner)
![Made from](https://img.shields.io/badge/From-Colombia-Yellow)

Hi Human! 

-

This project aims to assess your ability to develop new features in complex software systems using ROS2 and Nav2 as a case study.

The main idea of the project is to build a simple navigation system with Nav2 and a gazebo. You will start using Nav2’s low-level interfaces in order to understand how the system works, and then you will progressively scale up to use its high-level interfaces. In the end, your robot should be able to follow an arbitrary set of waypoints on a loop.

- 

You will probably use Nav2 in your job at Kiwibot so consider this project as the beginning of your training.

The project will comprise three sections that will be independent of each other, so if you feel stuck on one of them feel free to jump to the next one. Each section will have some extra points that will allow your project to stand out. Additionally to the project’s functionality, we will perform a general evaluation of
[xx%] The use you make of git
[xx%] The way you structure your solution (nice OOP will be rewarded)
[xx%] The code style (use a linter)
[xx%] How you write documentation

Dependencies
For running the project you will need a computer with Ubuntu 22 or 20. Though you can certainly use other ubuntu distributions or virtual machines on another OS we won't provide support if you are not using the above.
Explain how the final deliverable should look like and what they are expected to present

The simulator has been tested on a computer having a core i3 processor and no GPU, so you shouldn’t run into issues if you don’t have a super pro computer. However, if you have a small processor we do recommend setting ubuntu to work in performance mode and keeping your computer plugged into the AC adapter at all times.

A recommendation is to use VS Code as the main IDE for development. Make sure you also have installed in your host:

docker-ce
docker-compose
Remote development extensions for VSCode

Link to instructions to run the development container


License
This project is licensed under Apache 2, and thus all contributions will be licensed as such as per clause 5 of the Apache 2 License:

Submission of Contributions. Unless You explicitly state otherwise, any Contribution intentionally submitted for inclusion in the Work by You to the Licensor shall be under the terms and conditions of this License, without any additional terms or conditions. Notwithstanding the above, nothing herein shall supersede or modify the terms of any separate license agreement you may have executed with Licensor regarding such Contributions.


General context about nav2
Nav2 is a software project that aims to provide autonomous navigation capabilities to mobile robots. It comprises several modules (servers) with smaller tasks that can be combined together to achieve this goal or can be used independently to provide specific functionalities to the system (ex: localization, path planning, etc).

This modular architecture also makes it really easy to swap the algorithmic component of the different modules, allowing developers to choose the most suited approach for their needs. Since you will have to make heavy use of nav2 is highly recommended that you read its documentation before getting started

Section 1: Low-level interaction (python, provide node skeleton)

Point 1 - Use the controller server to move the robot
The controller server’s input is a path and its output is the linear and angular velocity the robot should have to follow that path.
At this point, you will have to have the robot follow a short straight path using the controller server
Details about the specific TODOs and some conceptual questions,. Gifs on how things should look like


Point 2 - Use the planner server to move the robot to a waypoint
The planner server’s input is a goal and its output is a path. At this point you will have to use a planner server to calculate a path to a goal and then pass this path to a controller server (replan every second)
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like

Point 3 - Use behavior server to make the robot spin on its goal
The behavior server holds different behaviors so it does not have a predefined input or output. It's an action server holder that runs certain routines when called so they depend on the specific  action. At this point, you will have to make the robot spin 180 degrees once it gets to its goal.
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like


Section 2: High-level interaction (python or C++, C++ gives more points, a node from the ground up)
If you completed section 1 successfully you may be wondering if using these action servers by hand is the best way to create a complete navigation system. Nav2 maintainers thought it wasn’t and they created the bt navigator: a module that interacts with all these action servers through a user-programmable behavior tree. 

Point  1 - Move the robot to a waypoint using NavigateToPose
Navigate to pose is an action server that takes in a pose and runs the action servers we used in part 1 following the logic on the navigate_to_pose_w_replanning_and_recovery behavior tree. At this point, you will have to use this action server to move the robot to a waypoint.
Details about the specific TODOs and some conceptual questions. Navigation-commander like node, do something with the feedback like plotting, etc,. Gifs on how things should look like

Point  2 - Change the behavior tree to make the robot spin on its goal
Now you will have to achieve the same you did in point 3 part 1 but using the behavior tree. Change the xml file so the robot spins 180 degrees when it gets to its goal.
Details about the specific TODOs and some conceptual questions. Optional: use Groot to show the BT. Gifs on how things should look like


-

Point  3 - Use a different planner and controller, including the velocity smoother
To demonstrate the advantages of nav2 modular architecture you will have to re-run the navigation using the RPP controller and the smac_2d planner. Additionally, you will have to use the velocity smoother.
Details about the specific TODOs and some conceptual questions. Gifs on how things should look like


Section 3: Custom plugin (C++, provide skeleton or detailed tutorial)
Point  4 - Write a custom plugin (to define which)
In this last point you will have to write a plugin for the XX server. Link to tutorial

