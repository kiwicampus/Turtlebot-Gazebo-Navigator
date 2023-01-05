# -----------------------------------------------------------------------------
# Pepisg (pedro@kiwibot.com)
# -----------------------------------------------------------------------------
FROM osrf/ros:humble-desktop-full

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=ada
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
# -----------------------------------------------------------------------------
# Install apt packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    # auto completion
    bash-completion \
    # Parsing yaml in c++
    libyaml-cpp-dev \
    # ------------------------------
    # Audio tools
    pulseaudio \
    alsa-base \
    alsa-tools \
    alsa-utils \
    libasound2-dev \
    # Python
    python3 \
    python3-pip \
    # Clean up
    && apt-get autoremove && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Install ROS2 packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-ompl \
    ros-${ROS_DISTRO}-bond-core \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-test-msgs \
    # nav2 deps
    libgraphicsmagick++1-dev \
    libceres-dev \
    # ------------------------------
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    && apt-get install -y nodejs \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # ------------------------------
    # [Optional] Add sudo support for non-root user
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # ------------------------------
    # Lidar Permission
    && adduser ${USERNAME} dialout \
    # ------------------------------
    # Clean up
    && apt-get autoremove && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Install python packages
COPY rover/requirements.txt  ./
RUN \
    pip3 install -U pip \
    -r requirements.txt
# -----------------------------------------------------------------------------

RUN ROS_SOURCE="source /opt/ros/humble/setup.bash" \
    && echo $ROS_SOURCE >> "/home/${USERNAME}/.bashrc"

# Logging variables
# https://index.ros.org/doc/ros2/Tutorials/Logging-and-logger-configuration/
# # If you would like more or less verbose formatting
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}][{name}]: {message}"
# If you would like to force enabling or disabling it
# the output is colorized when it’s targeting a terminal. 
ENV RCUTILS_COLORIZED_OUTPUT=1                                   
# By default, info and debug log calls aren’t line buffered. 
# You can force it using 
# ENV RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   # deprecated
ENV RCUTILS_LOGGING_USE_STDOUT=1   
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1   

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# -----------------------------------------------------------------------------
# disable the creation of core dump files on a UNIX or Linux environment
# https://dev.intelrealsense.com/docs/troubleshooting
# https://www.ibm.com/support/pages/how-disable-creation-core-dump-files-unix-or-linux
ENV ulimit -c 0

# -----------------------------------------------------------------------------
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
CMD ["/bin/bash"]

# -----------------------------------------------------------------------------
