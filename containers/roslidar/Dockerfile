FROM ubuntu:20.04 as builder

RUN apt-get update && apt-get install -y locales && rm -rf /var/lib/apt/lists/* \
    && localedef -i en_US -c -f UTF-8 -A /usr/share/locale/locale.alias en_US.UTF-8
ENV LANG en_US.utf8
ENV LC_ALL en_US.utf8

ARG DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
COPY --from=cristidragomir97/ros-core:latest /opt/ros/ /opt/ros/

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-dev \
    python3 \
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
    #libboost-python1.71-dev \
    python3-catkin-pkg \
    liblog4cxx-dev \
    libtinyxml2-dev \
    git \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install defusedxml netifaces \
    && mkdir -p ~/catkin_ws/src/ \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/Slamtec/rplidar_ros  

RUN cd ~/catkin_ws/src/ \
    && . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  

COPY rplidar_a1.launch  /root/catkin_ws/src/rplidar_ros/launch/rplidar_a1.launch

RUN  cd /root/catkin_ws  \
    && . /opt/ros/noetic/setup.sh \
    && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release 

CMD ldconfig && . /opt/ros/noetic/setup.sh \
   && . /root/catkin_ws/devel/setup.sh \
   && roslaunch rplidar_ros rplidar_a1.launch --wait

#FROM python:3.9-slim-bullseye as runtime 

### RUN apt-get update && \
###         apt-get install -y \
###         python3-catkin-pkg \
###         python3-rospkg \
###         python3-distutils \
###         python3-netifaces \
###         python3-empy \
###         python3-gnupg \
###         python3-pycryptodome \
###         python3-defusedxml
### 
### COPY --from=builder /root/catkin_ws/ /root/catkin_ws/
### 
### COPY --from=builder /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10 /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libtinyxml2.so /usr/lib/aarch64-linux-gnu/libtinyxml2.so
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libapr-1.so.0 /usr/lib/aarch64-linux-gnu/libapr-1.so.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0 /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libicudata.so.66.1 /usr/lib/aarch64-linux-gnu/libicudata.so.66.1
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11 /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0 /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libdl-2.31.so /usr/lib/aarch64-linux-gnu/libdl-2.31.so
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0 /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1 /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1 /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1 /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5 /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
### COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
### 
### CMD ldconfig && . /opt/ros/noetic/setup.sh \
###    && . /root/catkin_ws/devel/setup.sh \
###    && roslaunch rplidar_ros rplidar_a1.launch --wait
