# Stage 1: Builder environment for ROS and rosserial
FROM ubuntu:20.04 AS builder

# Set environment for non-interactive installation and localization
RUN apt-get update && apt-get install -y locales && rm -rf /var/lib/apt/lists/* \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8
ENV LANG en_US.utf8
ENV LC_ALL en_US.utf8

# Argument to avoid interactive prompts during install
ARG DEBIAN_FRONTEND=noninteractive

# Install ROS and bootstrap tools
COPY --from=cristidragomir97/ros-core:latest /opt/ros/ /opt/ros/

# Install necessary build tools and dependencies for ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-dev \
    python3-pip \
    python3-empy \
    python3-gnupg \
    python3-pycryptodome \
    python3-rospkg \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-chrono-dev \
    libconsole-bridge-dev \
    liblog4cxx-dev \
    libtinyxml2-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up catkin workspace and install rosserial packages
RUN mkdir -p /root/catkin_ws/src/ && cd /root/catkin_ws/src/ \
    && git clone https://github.com/ros-drivers/rosserial.git -b noetic-devel \
    && cd /root/catkin_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.sh && catkin_make"

# Stage 2: Runtime environment for rosserial and python dependencies
FROM python:3.8-slim-bullseye AS runtime

# Install necessary Python packages and ROS dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-catkin-pkg \
    python3-rospkg \
    python3-distutils \
    python3-netifaces \
    python3-empy \
    python3-gnupg \
    python3-pycryptodome \
    python3-defusedxml \
    python3-serial \

    && rm -rf /var/lib/apt/lists/*

# Copy catkin workspace from builder stage
COPY --from=builder /root/catkin_ws/ /root/catkin_ws/

# Copy specific shared libraries required for runtime from the builder
COPY --from=builder /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10 /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10
COPY --from=builder /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
COPY --from=builder /usr/lib/aarch64-linux-gnu/libtinyxml2.so /usr/lib/aarch64-linux-gnu/libtinyxml2.so
COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0


# Set up environment variables for ROS
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Source ROS and catkin workspace setup files, then run the rosserial_python node
CMD ldconfig && \
    . /opt/ros/noetic/setup.sh && \
    . /root/catkin_ws/devel/setup.sh && \
    rosrun rosserial_python serial_node.py _port:=${SERIAL_PORT} __name:=serial_node_${ROS_HOSTNAME} _baud:=115200
