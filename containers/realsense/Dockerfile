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
    wget \ 
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install defusedxml netifaces 

# install dependencies for librealsense
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    cmake 


RUN apt-get update && apt-get install --no-install-recommends -y unzip
# download librealsense v2.45.0 (latest supported by ROS)
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.45.0.zip  
RUN unzip v2.45.0.zip && cd librealsense-2.45.0/

# build librealsense
RUN mkdir -p /librealsense-2.45.0/build && cd /librealsense-2.45.0/build && cmake /librealsense-2.45.0 \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DBUILD_PYTHON_EXECUTABLE=/usr/bin/python3.8 
 
RUN cd /librealsense-2.45.0/build && make && make install


RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool python3-rosdep

RUN pip3 install netifaces defusedxml empy pyyaml

# initialise 
RUN . /opt/ros/noetic/setup.sh \
    && mkdir -p /root/catkin_ws/src \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  

# initialise ws_tool
RUN cd /root/catkin_ws  \
    && wstool init src 

RUN mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/IntelRealSense/realsense-ros.git

RUN cd ~/catkin_ws/src/realsense-ros \
    && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

## pull source for dependencies 
RUN cd /root/catkin_ws \
    && rosinstall_generator cv_bridge ddynamic_reconfigure image_transport diagnostic_updater tf --rosdistro noetic --deps --tar > rs.rosinstall \
    && wstool merge -t src rs.rosinstall \
    && wstool update -t src \
    && rosdep update && rosdep install --from-paths src --ignore-src --skip-keys "librealsense2" --skip-keys "xacro" --skip-keys "nodelet" -r -y


# build ros-realsense
RUN . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws  \
    && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
    && catkin_make install

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt install --no-install-recommends -y udev
RUN cd /librealsense-2.45.0/ && ./scripts/setup_udev_rules.sh
RUN rm -rf /librealsense-2.45.0/

CMD source /root/catkin_ws/devel/setup.bash && \
    roslaunch realsense2_camera rs_camera.launch depth_width:=424 depth_height:=240 depth_fps:=60 color_width:=320 color_height:=240 color_fps:=60