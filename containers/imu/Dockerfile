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



RUN mkdir -p /root/catkin_ws/src/ && cd /root/catkin_ws/src/ \
    && git clone https://github.com/dheera/ros-imu-bno055.git \
    && cd /root/catkin_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.sh && catkin_make"

# Entry point to source ROS and workspace, then run the imu_bno055 node
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    source /root/catkin_ws/devel/setup.bash && \
    rosrun imu_bno055 bno055_i2c_node"]


#### Stage 2: Runtime environment
###FROM python:3.8-slim-bullseye AS runtime
###
#### Install necessary Python packages and ROS dependencies
###RUN apt-get update && \
###    apt-get install -y --no-install-recommends \
###    python3-catkin-pkg \
###    python3-rospkg \
###    python3-distutils \
###    python3-netifaces \
###    python3-empy \
###    python3-gnupg \
###    python3-pycryptodome \
###    python3-defusedxml \
###    python3-serial \
###
###    && rm -rf /var/lib/apt/lists/*
###
#### Copy catkin workspace from builder stage
###COPY --from=builder /root/catkin_ws/ /root/catkin_ws/
###
#### Copy specific shared libraries required for runtime from the builder
###COPY --from=builder /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10 /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6 /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libapr-1.so.0 /usr/lib/aarch64-linux-gnu/libapr-1.so.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0 /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libicudata.so.66.1 /usr/lib/aarch64-linux-gnu/libicudata.so.66.1
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11 /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0 /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libdl-2.31.so /usr/lib/aarch64-linux-gnu/libdl-2.31.so
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0 /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1 /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1 /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1 /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5 /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
###COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
###
###
#### Set environment variables for non-interactive installation
###ARG DEBIAN_FRONTEND=noninteractive
###ENV LANG=C.UTF-8
###ENV LC_ALL=C.UTF-8
###
#### Install necessary runtime packages
###RUN apt-get update && apt-get install -y --no-install-recommends \
###    python3 \
###    python3-pip \
###    python3-yaml \
###    && rm -rf /var/lib/apt/lists/*
###
#### Copy the built workspace from the builder stage
###COPY --from=builder /root/catkin_ws /root/catkin_ws
###
#### Set up environment variables for ROS
###ENV ROS_DISTRO=noetic
###ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
###ENV PATH=$ROS_ROOT/bin:$PATH
###ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
###ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages:$PYTHONPATH
###ENV ROS_PACKAGE_PATH=$ROS_ROOT/share
###
#### Assume ROS binaries are mounted via volume at /opt/ros
#### Set the ROS master URI and hostname (adjust as needed)
###ENV ROS_MASTER_URI=http://localhost:11311
###ENV ROS_HOSTNAME=localhost
###
#### Entry point to source ROS and workspace, then run the imu_bno055 node
###CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
###    source /root/catkin_ws/devel/setup.bash && \
###    rosrun imu_bno055 bno055_i2c_node"]
###