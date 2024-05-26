ARG ROS_DISTRO=humble
FROM arm64v8/ros:${ROS_DISTRO}-perception

SHELL ["/bin/bash", "-c"]

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install ros packages
RUN sudo apt install libudev-dev -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
&& sudo apt-get update -y \
&& sudo apt-get install ros-$ROS_DISTRO-diagnostic-updater \
&& sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y \
&& sudo apt install ros-$ROS_DISTRO-navigation2 -y \
&& sudo apt install ros-$ROS_DISTRO-nav2-util -y \
&& sudo apt install ros-$ROS_DISTRO-xacro -y \
&& sudo apt install ros-humble-rmw-cyclonedds-cpp -y 

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]
