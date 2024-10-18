# Use an official Ubuntu image as the base
FROM ubuntu:20.04

# Set environment variables for ROS distribution
ARG ROS_DISTRO=noetic

# Disable interactive installation prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    sudo \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y ros-noetic-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install Intel RealSense SDK and related packages
RUN curl -sSL https://librealsense.intel.com/Debian/apt-key.gpg | gpg --dearmor -o /usr/share/keyrings/librealsense-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/librealsense-archive-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y librealsense2-dkms librealsense2-dev libeigen3-dev

# Create a catkin workspace and clone the realsense-ros package
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy && \
    cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the catkin workspace
RUN echo "source /
