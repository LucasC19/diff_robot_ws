ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]
RUN mkdir /diff_robot_ws
WORKDIR /diff_robot_ws

# Source Ros
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
&& sudo apt-get update -y

# Install pip
RUN sudo apt install python3-pip -y 

# Install Python libraries
RUN pip3 install opencv-python \
&& pip3 install pygame \
&& pip3 install jpl-rosa


# Install ros2 packages
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y 
RUN sudo apt install ros-${ROS_DISTRO}-rosidl-generator-dds-idl -y 
RUN sudo apt-get install ros-${ROS_DISTRO}-diagnostic-updater -y 
RUN sudo apt install ros-${ROS_DISTRO}-foxglove-bridge -y 
RUN sudo apt install ros-${ROS_DISTRO}-xacro -y 
RUN sudo apt install ros-${ROS_DISTRO}-cartographer -y 
RUN sudo apt install ros-${ROS_DISTRO}-cartographer-ros -y 
RUN sudo apt install ros-${ROS_DISTRO}-pointcloud-to-laserscan -y 
RUN sudo apt install ros-${ROS_DISTRO}-slam-toolbox -y 
RUN sudo apt install ros-${ROS_DISTRO}-navigation2 -y 
RUN sudo apt install ros-${ROS_DISTRO}-nav2-bringup -y
RUN sudo apt install ros-$ROS_DISTRO-nav2-util -y
RUN sudo apt install ros-${ROS_DISTRO}-rosbridge-server -y
RUN sudo apt install ros-${ROS_DISTRO}-rosbridge-suite -y
RUN sudo apt install ros-${ROS_DISTRO}-demo-nodes-cpp -y
RUN sudo apt install ros-$ROS_DISTRO-robot-localization -y 
RUN sudo apt install ros-$ROS_DISTRO-ros2-control -y 
RUN sudo apt install ros-$ROS_DISTRO-ros2-controllers -y

RUN sudo apt install joystick -y

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CMD ["bash"]