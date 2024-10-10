FROM dragomirxyz/roscore AS builder

RUN apt-get update && apt-get install -y  build-essential python3 python3-dev python3-pip git libspnav-dev libspnav0 joystick libx11-dev

# clone rosboard
RUN mkdir -p ~/catkin_ws/src/ \    && cd ~/catkin_ws/src/ \    && git clone  https://github.com/ros-drivers/joystick_drivers

RUN mkdir -p /etc/ros/rosdep/sources.list.d/
RUN echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" > /etc/ros/rosdep/sources.list.d/30-ubiquity.list
# list of packages to be installed in rootfs we will use in next stage

RUN pip3 install netifaces defusedxml empy pyyaml

RUN rosdep init && rosdep update 

# initialise 
RUN . /opt/ros/noetic/setup.sh \
    && cd /root/catkin_ws/src/ \
    && catkin_init_workspace  

# initialise ws_tool
RUN cd /root/catkin_ws  \
    && wstool init src 

## pull source for dependencies 
RUN cd /root/catkin_ws \
    && rosinstall_generator diagnostics --rosdistro noetic --wet-only --tar > diagnostics.rosinstall \
    && rosinstall_generator roslint --rosdistro noetic --wet-only --tar > roslint.rosinstall 

## use wstool
RUN cd /root/catkin_ws \
    && wstool merge -t src diagnostics.rosinstall \
    && wstool merge -t src roslint.rosinstall \
    && wstool update -t src 

# build packages
RUN . /opt/ros/noetic/setup.sh \    
    && cd /root/catkin_ws \    
    && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y --os=ubuntu:focal  \
    && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \   
    && catkin_make install 




#FROM python:3.8-slim-buster as runtime 
#
#RUN apt-get update && \
#    apt-get install -y \
#        python3-catkin-pkg \
#        python3-distutils \
#        python3-netifaces \
#        python3-empy \
#        python3-defusedxml 
#
#RUN pip install pyyaml rospkg
#   
#COPY --from=builder /root/catkin_ws/ /root/catkin_ws/
#
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libpython3.8.so.1 /usr/lib/aarch64-linux-gnu/libpython3.8.so.1 
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libpython3.8.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.8.so.1.0
#
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libstdc++.so.6 /lib/aarch64-linux-gnu/libm.so.6
#COPY --from=builder /lib/aarch64-linux-gnu/libcrypt.so.1 /lib/aarch64-linux-gnu/libm.so.6
#COPY --from=builder /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10 /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libtinyxml2.so /usr/lib/aarch64-linux-gnu/libtinyxml2.so
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6 /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0 /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libapr-1.so.0 /usr/lib/aarch64-linux-gnu/libapr-1.so.0 
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0 /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libicui18n.so.66 /usr/lib/aarch64-linux-gnu/libicui18n.so.66
##COPY --from=builder /usr/lib/aarch64-linux-gnu/  /usr/lib/aarch64-linux-gnu/
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libicuuc.so.66 /usr/lib/aarch64-linux-gnu/libicuuc.so.66
#COPY --from=builder /usr/lib/aarch64-linux-gnu/libicudata.so.66 /usr/lib/aarch64-linux-gnu/libicudata.so.66



COPY joy_cmd_vel.py /usr/bin/joy_cmd_vel.py

CMD ldconfig && . /opt/ros/noetic/setup.sh && python3 /usr/bin/joy_cmd_vel.py