# add ros lists 
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# install python stuff
RUN apt update &&  apt-get install -y   \
    python3 \
    python3-dev \
    python3-pip \
    python3-empy \
    python3-rospkg \
    python3-catkin-pkg \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-vcstools 

# install ros stuff
RUN apt update &&  apt-get install -y   \
    ros-noetic-ros-base

# install python runtime libraries for ROS
RUN  pip3 install --upgrade pip && pip3 install defusedxml netifaces

# install extra libraries 
RUN apt update &&  apt-get install -y   \
    qtdeclarative5-dev \
    qtbase5-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    liblog4cxx-dev \
    libtinyxml2-dev \
    libtf-dev \
    liblaser-geometry-dev \
    libpoco-dev \
    libopencv-dev 
    

# initialize rosdep 
RUN rosdep init && \
  rosdep update --rosdistro noetic

# clone hector_slam
RUN mkdir -p ~/catkin_ws/src/ \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam

COPY mapping_default.launch /root/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch
COPY tutorial.launch /root/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch
    
    
    #CMD . /opt/ros/noetic/setup.sh \
    #    && . /root/catkin_ws/devel/setup.sh \
    #    && roslaunch hector_slam_launch tutorial.launch
#build hector_slam 
RUN apt-get update \
    && . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && rosdep install -q -y -r --from-paths src --ignore-src \
    && catkin_make -j2 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release  \
    && catkin_make install 
    COPY mapping_default.launch /root/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch
    COPY tutorial.launch /root/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch
    
    
    #CMD . /opt/ros/noetic/setup.sh \
    #    && . /root/catkin_ws/devel/setup.sh \
    #    && roslaunch hector_slam_launch tutorial.launch