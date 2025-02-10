ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
  ros-dev-tools \
  wget \
  gedit

RUN rosdep init \
    && apt update && apt upgrade -y \
    && rosdep update \
    && apt install -y \
       ros-${ROS_DISTRO}-nav2-bringup \
       ros-${ROS_DISTRO}-navigation2 \
       ros-${ROS_DISTRO}-turtlebot3-gazebo \
       ros-${ROS_DISTRO}-turtlebot3 \
       ros-${ROS_DISTRO}-teleop-twist-keyboard \
       ros-${ROS_DISTRO}-rqt-common-plugins

RUN echo source /opt/ros/humble/setup.bash >> ~/.bashrc
RUN echo export TURTLEBOT3_MODEL=waffle >> ~/.bashrc
RUN echo export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models >> ~/.bashrc
RUN echo source /usr/share/gazebo/setup.bash >> ~/.bashrc
