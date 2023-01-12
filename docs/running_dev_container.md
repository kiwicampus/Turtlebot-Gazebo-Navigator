<!-- ---------------------------------------------------------------------- -->
## **Running The Dev-Container**
 
If you have your [VSCode](https://code.visualstudio.com/) with the right extensions, and if you have Docker and Docker-compose installed in your system, when you open the project's main folder you'll see a window on the bottom right corner, click in "reopen in container" button, if you don't see anything press `Ctrl+Shift+P` and type `Remote-Containers: Rebuild and Reopen in container` or `Docker-images: Build Image` option. You can go for a walk because the building image process will start and it'll take a while due to the installation of all packages and dependencies of the dev-environment as [ROS2](https://index.ros.org/doc/ros2/), [OpenCV](https://opencv.org/), [Python](https://www.python.org/). While the process is completed here are some videos of [puppies (?)](https://www.youtube.com/watch?v=CSvFpBOe8eY). You can see at any time the logs of the building process clicking in `Starting with Dev Container` on the bottom right corner. When the process is done you'll see some messages of process succeed.
 
<img src="https://user-images.githubusercontent.com/43115782/87437367-d5806200-c5b3-11ea-9bf2-836e45f46ed8.gif" alt="building_dev-container" width="1200">
 
When the process is done you can open a terminal in the dev-container going to the menu bar `Terminal` and then `New Terminal`. Congratulations now you have everything that we use for our deployments.

To check out everything is working as it should navigate to the `/workspace/configs` folder and run `bash start.sh`, a utility script we provide for running everything you need. After a while a `gazebo` and a `rviz` window should spawn and you should be able to move the robot as the gif below shows. This may take too long the first time you run it, causing for the turtlebot never to spawn in the gazebo world; if this happens to you just kill everything by pressing `Ctrl+C` and launch it again.

If you are running into more issues now or while solving the project check out the [Troubleshooting](#troubleshooting) section. If your problem still persists please open an issue in the repo and we will do our best to help you. If there's something wrong from our side you will get a bonus of **+3%** for pointing it out.
 
![Ready](https://user-images.githubusercontent.com/71234974/211570809-b5c59b3a-2246-4dcc-99ca-2e6b20813b85.gif)


<br />

<!-- ---------------------------------------------------------------------- -->
## **Architecture**
 
Find the distribution of final project in the next list:
 
- **[rover](../rover/):** Main folder where most of the source code is located
  - **[configs](../configs/):** Path planner config files
     - [*start.sh:*](../configs/start.sh) bash script to run stack of the project
     - [*nodes_launch.yaml:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/nodes_launch.yaml) file describing which nodes launch or not and the configuration of nodes
  - **[ros2/src](../rover/ros2/src):** Development workspace & ROS 2 packages
     - [tb_bringup](../rover/ros2/src/tb_bringup/): Package with the launch and configuration files for running all the Nav2 related packages
     - [interfaces](../rover/ros2/src/interfaces/): Package for playing audio files in your computer
     - [low_level_nav2_commander](../rover/ros2/src/low_level_nav2_commander/): Package for using the low level nav2 action servers
     - [high_level_nav2_commander_cpp](../rover/ros2/src/high_level_nav2_commander_cpp/): Package for using the high level nav2 action servers in C++
     - [high_level_nav2_commander_py](../rover/ros2/src/high_level_nav2_commander_py/): Package for using the low level nav2 action servers in python
     - [speaker_goal_checker](../rover/ros2/src/speaker_goal_checker/): Package for playing sounds each time nav2 reaches a waypoint

Only some files are listed, and explained (most important).
 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Running The Project Stack**
 
Find a brief explanation on how to run our stack in your IDE and the explanation of the launch file, and config files as the key to managing which nodes are going to be launched and how they're going to work.
 
In order to launch locally (Inside your IDE), please locate into the `configs/` folder in the dev-container terminal and execute the following prompt command:
 
     $ bash start.sh

*Note:* if you've already launch and compile the whole stack and there's no a *hot* modification inside the stack, it's possible to avoid the entire compiling step running:

     $ bash start.sh -b 0

This bash performs the following steps to launch the *tb_navigator* stack:

1. Sources ROS Humble and clean the older development workspace (If enabled).
2. Builds the development workspace at [`rover/ros2/`](../rover/ros2)
3. Sources the resulting setup in the install folder `. install/setup.bash`
4. Executes [`ros2 launch /configs/tb_navigator.launch.py`](../configs/tb_navigator.launch.py) with the specified node in [*nodes_launch.yaml:*](../configs/nodes_launch.yaml). You can choose which nodes to launch by modifying the `launch` field on each node's entry on said `yaml`.
 
You can compile and launch everything by your own if you already have a background and experience with ROS/ROS2, but for those who want everything easy, and fast the bash script will set up and run everything for you. With the [`start.sh`](../configs/start.sh) bash script you can run the stack of the project, this file has all instruction to download third-party packages, other required dependencies if they're missing, and setup, source, and run the ros2 workspace, launching the nodes specified in the file `nodes_launch.yaml` (File created when you start or run the script for the first time).
 
## **Troubleshooting**

### GUI applicationsv (gazebo, rviz) not spawning

If you are having troubles or errors getting the user interface window, read about [Docker image with OpenCV with X11 forwarding for GUI](https://marcosnietoblog.wordpress.com/2017/04/30/docker-image-with-opencv-with-x11-forwarding-for-gui/) for explanations, run the prompt command (Do not this in the dev-container terminal):
 
     $ xhost +

the error usually looks something like this: 

     Gdk-ERROR **: 20:17:14.340: The program 'xxx' received an X Window System error.

If the error remains, run the bash file until the window is shown, it could take even 10 times.

### Gzserver dying

Sometimes gzserver may die silently. This is usuall happens because another instance of gzserver was already running. If that is the case type on a terminal in the container `pkill -f -9 gzserver` and check on the system monitor that gzserver is no longer running.

### Audio not playing

While developing the project you will need to play some sounds in your computer using the [interfaces package](../rover/ros2/src/interfaces/) we provide you. If you are having troubles getting audio from the virtual environment is very likely that your audio device is already in use by another application (ex: spotify, your browser running youtube, etc) if you get an error like the one shown below try closing all applications you suspect may be using your audio device and connecting and disconnecting headsets.

![Screenshot from 2023-01-06 12-58-32](https://user-images.githubusercontent.com/71234974/211584344-8f53331a-0c56-4467-afcc-59adc66e11ac.png)



<br />