ARG ROS_DISTRO=humble
FROM arm64v8/ros:${ROS_DISTRO}-perception

SHELL ["/bin/bash", "-c"]

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]
