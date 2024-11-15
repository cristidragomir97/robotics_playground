FROM ros:noetic-ros-core-focal as builder
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    unzip \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    wget \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# add stuff to path 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

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

# download librealsense v2.50.0 (latest supported by ROS)
RUN cd / && wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.zip  
RUN unzip v2.50.0.zip && cd librealsense-2.50.0/ && mkdir -p /librealsense-2.50.0/build && cd /librealsense-2.50.0/build && cmake /librealsense-2.50.0 \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=true 
 
RUN cd /librealsense-2.50.0/build && make -j32 && make install


# build realsense-ros wrapper
RUN mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/src/ \
    && git clone https://github.com/IntelRealSense/realsense-ros.git  -b ros1-legacy 

RUN cd ~/catkin_ws/src/realsense-ros \
    && git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

# install ros-realsense 
RUN apt install --no-install-recommends -y libeigen3-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-catkin \
    ros-noetic-cv-bridge \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    udev 

# build ros-realsense
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws/src/ \
    && catkin_init_workspace  \
    && cd /root/catkin_ws  \
    && catkin_make -j32 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release \
    && catkin_make install

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt install --no-install-recommends -y udev
RUN cd /librealsense-2.50.0/ && ./scripts/setup_udev_rules.sh
RUN rm -rf /librealsense-2.50.0/

#CMD source /root/catkin_ws/devel/setup.bash && \
#    roslaunch realsense2_camera rs_camera.launch depth_width:=424 depth_height:=240 depth_fps:=60 color_width:=320 color_height:=240 color_fps:=60


FROM ubuntu:focal as runtime 

RUN apt-get update && \
        apt-get install -y \
        python3-catkin-pkg \
        python3-rospkg \
        python3-distutils \
        python3-netifaces \
        python3-empy \
        python3-gnupg \
        python3-pycryptodome \
        python3-defusedxml \
        rsync

# Copy catkin workspace from builder stage
COPY --from=builder /root/catkin_ws/ /root/catkin_ws/
# Docker COPY commands for container containers-camera-1
COPY --from=builder \
    /usr/lib/aarch64-linux-gnu/libresolv-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_dns-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so \
    /usr/lib/aarch64-linux-gnu/libgcc_s.so.1 \
    /usr/lib/aarch64-linux-gnu/libgfortran.so.5.0.0 \
    /usr/lib/aarch64-linux-gnu/libffi.so.7.1.0 \
    /usr/lib/aarch64-linux-gnu/libyaml-0.so.2.0.6 \
    /usr/lib/aarch64-linux-gnu/liblzma.so.5.2.4 \
    /usr/lib/aarch64-linux-gnu/libbz2.so.1.0.4 \
    /usr/lib/aarch64-linux-gnu/libcrypto.so.1.1 \
    /usr/lib/aarch64-linux-gnu/libssl.so.1.1 \
    /usr/lib/aarch64-linux-gnu/libmpdec.so.2.4.2 \
    /usr/lib/aarch64-linux-gnu/libz.so.1.2.11 \
    /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11 \
    /usr/lib/aarch64-linux-gnu/libm-2.31.so \
    /usr/lib/aarch64-linux-gnu/libutil-2.31.so \
    /usr/lib/aarch64-linux-gnu/libdl-2.31.so \
    /usr/lib/aarch64-linux-gnu/libpthread-2.31.so \
    /usr/lib/aarch64-linux-gnu/libc-2.31.so \
    /usr/lib/aarch64-linux-gnu/ld-2.31.so \
    /usr/lib/aarch64-linux-gnu/libhx509.so.5.0.0 \
    /usr/lib/aarch64-linux-gnu/libheimbase.so.1.0.0 \
    /usr/lib/aarch64-linux-gnu/libwind.so.0.0.0 \
    /usr/lib/aarch64-linux-gnu/libffi.so.7.1.0 \
    /usr/lib/aarch64-linux-gnu/libroken.so.18.1.0 \
    /usr/lib/aarch64-linux-gnu/libhcrypto.so.4.1.0 \
    /usr/lib/aarch64-linux-gnu/libasn1.so.8.0.0 \
    /usr/lib/aarch64-linux-gnu/libkrb5.so.26.0.0 \
    /usr/lib/aarch64-linux-gnu/libheimntlm.so.0.1.0 \
    /usr/lib/aarch64-linux-gnu/libkeyutils.so.1.8 \
    /usr/lib/aarch64-linux-gnu/libbrotlicommon.so.1.0.7 \
    /usr/lib/aarch64-linux-gnu/libtasn1.so.6.6.0 \
    /usr/lib/aarch64-linux-gnu/libp11-kit.so.0.3.0 \
    /usr/lib/aarch64-linux-gnu/libgmp.so.10.4.0 \
    /usr/lib/aarch64-linux-gnu/libhogweed.so.5.0 \
    /usr/lib/aarch64-linux-gnu/libunistring.so.2.1.0 \
    /usr/lib/aarch64-linux-gnu/libgssapi.so.3.0.0 \
    /usr/lib/aarch64-linux-gnu/libsasl2.so.2.0.25 \
    /usr/lib/aarch64-linux-gnu/libkrb5support.so.0.1 \
    /usr/lib/aarch64-linux-gnu/libcom_err.so.2.1 \
    /usr/lib/aarch64-linux-gnu/libk5crypto.so.3.1 \
    /usr/lib/aarch64-linux-gnu/libkrb5.so.3.3 \
    /usr/lib/aarch64-linux-gnu/libaec.so.0.0.10 \
    /usr/lib/aarch64-linux-gnu/libplds4.so \
    /usr/lib/aarch64-linux-gnu/libplc4.so \
    /usr/lib/aarch64-linux-gnu/libnssutil3.so \
    /usr/lib/aarch64-linux-gnu/libgfortran.so.5.0.0 \
    /usr/lib/aarch64-linux-gnu/libfygm.so.0.0.0 \
    /usr/lib/aarch64-linux-gnu/libfyut.so.0.0.0 \
    /usr/lib/aarch64-linux-gnu/libbrotlidec.so.1.0.7 \
    /usr/lib/aarch64-linux-gnu/liblber-2.4.so.2.10.12 \
    /usr/lib/aarch64-linux-gnu/libgnutls.so.30.27.0 \
    /usr/lib/aarch64-linux-gnu/libnettle.so.7.0 \
    /usr/lib/aarch64-linux-gnu/libpsl.so.5.3.2 \
    /usr/lib/aarch64-linux-gnu/libssh.so.4.8.4 \
    /usr/lib/aarch64-linux-gnu/librtmp.so.1 \
    /usr/lib/aarch64-linux-gnu/libidn2.so.0.3.6 \
    /usr/lib/aarch64-linux-gnu/libnghttp2.so.14.19.0 \
    /usr/lib/aarch64-linux-gnu/libsqlite3.so.0.8.6 \
    /usr/lib/aarch64-linux-gnu/libldap_r-2.4.so.2.10.12 \
    /usr/lib/aarch64-linux-gnu/libgssapi_krb5.so.2.2 \
    /usr/lib/aarch64-linux-gnu/libssl.so.1.1 \
    /usr/lib/aarch64-linux-gnu/libbz2.so.1.0.4 \
    /usr/lib/aarch64-linux-gnu/libsz.so.2.0.1 \
    /usr/lib/aarch64-linux-gnu/libhdf5_serial_hl.so.100.1.1 \
    /usr/lib/aarch64-linux-gnu/liburiparser.so.1.0.26 \
    /usr/lib/aarch64-linux-gnu/libminizip.so.1.0.0 \
    /usr/lib/aarch64-linux-gnu/libltdl.so.7.3.1 \
    /usr/lib/aarch64-linux-gnu/libgeos-3.8.0.so \
    /usr/lib/aarch64-linux-gnu/libnspr4.so \
    /usr/lib/aarch64-linux-gnu/libnss3.so \
    /usr/lib/aarch64-linux-gnu/liblcms2.so.2.0.8 \
    /usr/lib/aarch64-linux-gnu/libfontconfig.so.1.12.0 \
    /usr/lib/aarch64-linux-gnu/libfreetype.so.6.17.1 \
    /usr/lib/aarch64-linux-gnu/libsuperlu.so.5.2.1 \
    /usr/lib/aarch64-linux-gnu/libcrypto.so.1.1 \
    /usr/lib/aarch64-linux-gnu/libmysqlclient.so.21.2.39 \
    /usr/lib/aarch64-linux-gnu/libsmime3.so \
    /usr/lib/aarch64-linux-gnu/libarpack.so.2.1.0 \
    /usr/lib/aarch64-linux-gnu/libxml2.so.2.9.10 \
    /usr/lib/aarch64-linux-gnu/libfyba.so.0.0.0 \
    /usr/lib/aarch64-linux-gnu/libcurl-gnutls.so.4.6.0 \
    /usr/lib/aarch64-linux-gnu/libspatialite.so.7.1.0 \
    /usr/lib/aarch64-linux-gnu/libdap.so.25.2.2 \
    /usr/lib/aarch64-linux-gnu/libdapclient.so.6.1.11 \
    /usr/lib/aarch64-linux-gnu/libproj.so.15.3.1 \
    /usr/lib/aarch64-linux-gnu/libpq.so.5.12 \
    /usr/lib/aarch64-linux-gnu/libcfitsio.so.8.3.47 \
    /usr/lib/aarch64-linux-gnu/libgeotiff.so.5.0.1 \
    /usr/lib/aarch64-linux-gnu/libgif.so.7.1.0 \
    /usr/lib/aarch64-linux-gnu/libhdf5_serial.so.103.0.0 \
    /usr/lib/aarch64-linux-gnu/libnetcdf.so.15 \
    /usr/lib/aarch64-linux-gnu/libxerces-c-3.2.so \
    /usr/lib/aarch64-linux-gnu/libkmlengine.so.1.3.0 \
    /usr/lib/aarch64-linux-gnu/libkmldom.so.1.3.0 \
    /usr/lib/aarch64-linux-gnu/libkmlbase.so.1.3.0 \
    /usr/lib/aarch64-linux-gnu/libodbcinst.so.2.0.0 \
    /usr/lib/aarch64-linux-gnu/libodbc.so.2.0.0 \
    /usr/lib/aarch64-linux-gnu/libepsilon.so.1.0.0 \
    /usr/lib/aarch64-linux-gnu/libgeos_c.so.1.13.1 \
    /usr/lib/aarch64-linux-gnu/libqhull.so.7.2.0 \
    /usr/lib/aarch64-linux-gnu/libfreexl.so.1.1.0 \
    /usr/lib/aarch64-linux-gnu/libpoppler.so.97.0.0 \
    /usr/lib/aarch64-linux-gnu/libIlmThread-2_3.so.24.0.0 \
    /usr/lib/aarch64-linux-gnu/libIex-2_3.so.24.0.0 \
    /usr/lib/aarch64-linux-gnu/libHalf.so.24.0.0 \
    /usr/lib/aarch64-linux-gnu/libjbig.so.0 \
    /usr/lib/aarch64-linux-gnu/liblzma.so.5.2.4 \
    /usr/lib/aarch64-linux-gnu/libzstd.so.1.4.4 \
    /usr/lib/aarch64-linux-gnu/libgdcmCommon.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libgdcmIOD.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libjson-c.so.4.0.0 \
    /usr/lib/aarch64-linux-gnu/libCharLS.so.2.0 \
    /usr/lib/aarch64-linux-gnu/libopenjp2.so.2.3.1 \
    /usr/lib/aarch64-linux-gnu/libgdcmjpeg16.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libgdcmjpeg12.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libgdcmjpeg8.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libgdcmDICT.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libgdcmDSED.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libIlmImf-2_3.so.24.0.0 \
    /usr/lib/aarch64-linux-gnu/libtiff.so.5.5.0 \
    /usr/lib/aarch64-linux-gnu/libgdcmMSFF.so.3.0.5 \
    /usr/lib/aarch64-linux-gnu/libwebp.so.6.0.2 \
    /usr/lib/aarch64-linux-gnu/libpng16.so.16.37.0 \
    /usr/lib/aarch64-linux-gnu/libjpeg.so.8.2.2 \
    /usr/lib/aarch64-linux-gnu/libudev.so.1.6.17 \
    /usr/lib/aarch64-linux-gnu/libtbb.so.2 \
    /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0 \
    /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0 \
    /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0 \
    /usr/lib/aarch64-linux-gnu/libusb-1.0.so.0.2.0 \
    /usr/lib/aarch64-linux-gnu/libresolv-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_dns-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so \
    /usr/lib/aarch64-linux-gnu/libutil-2.31.so \
    /usr/lib/aarch64-linux-gnu/libicudata.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11 \
    /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0 \
    /usr/lib/aarch64-linux-gnu/libpython3.8.so.1.0 \
    /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libz.so.1.2.11 \
    /usr/lib/aarch64-linux-gnu/libpcre.so.3.13.3 \
    /usr/lib/aarch64-linux-gnu/librt-2.31.so \
    /usr/lib/aarch64-linux-gnu/libdl-2.31.so \
    /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1 \
    /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5 \
    /usr/lib/aarch64-linux-gnu/libPocoFoundation.so.62 \
    /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10.0.0 \
    /usr/lib/aarch64-linux-gnu/libm-2.31.so \
    /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 \
    /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libpthread-2.31.so \
    /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6.2.0 \
    /usr/lib/aarch64-linux-gnu/libc-2.31.so \
    /usr/lib/aarch64-linux-gnu/libgcc_s.so.1 \
    /usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.28 \
    /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0 \
    /usr/lib/aarch64-linux-gnu/ld-2.31.so \
    /usr/lib/aarch64-linux-gnu/libresolv-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_dns-2.31.so \
    /usr/lib/aarch64-linux-gnu/libnss_files-2.31.so \
    /usr/lib/aarch64-linux-gnu/libutil-2.31.so \
    /usr/lib/aarch64-linux-gnu/libicudata.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libexpat.so.1.6.11 \
    /usr/lib/aarch64-linux-gnu/libcrypt.so.1.1.0 \
    /usr/lib/aarch64-linux-gnu/libpython3.8.so.1.0 \
    /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libz.so.1.2.11 \
    /usr/lib/aarch64-linux-gnu/libpcre.so.3.13.3 \
    /usr/lib/aarch64-linux-gnu/librt-2.31.so \
    /usr/lib/aarch64-linux-gnu/libdl-2.31.so \
    /usr/lib/aarch64-linux-gnu/libicuuc.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libicui18n.so.66.1 \
    /usr/lib/aarch64-linux-gnu/libaprutil-1.so.0.6.1 \
    /usr/lib/aarch64-linux-gnu/libapr-1.so.0.6.5 \
    /usr/lib/aarch64-linux-gnu/libPocoFoundation.so.62 \
    /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/liblog4cxx.so.10.0.0 \
    /usr/lib/aarch64-linux-gnu/libm-2.31.so \
    /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 \
    /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0 \
    /usr/lib/aarch64-linux-gnu/libpthread-2.31.so \
    /usr/lib/aarch64-linux-gnu/libtinyxml2.so.6.2.0 \
    /usr/lib/aarch64-linux-gnu/libc-2.31.so \
    /usr/lib/aarch64-linux-gnu/libgcc_s.so.1 \
    /usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.28 \
    /usr/lib/aarch64-linux-gnu/libuuid.so.1.3.0 \
    /usr/lib/aarch64-linux-gnu/ld-2.31.so \
    /usr/lib/aarch64-linux-gnu/

COPY --from=builder \
    /usr/lib/python3/dist-packages/netifaces.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3/dist-packages/_yaml.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3/dist-packages/

COPY --from=builder \
    /usr/lib/python3.8/lib-dynload/_asyncio.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_contextvars.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_ctypes.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_queue.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_opcode.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_lzma.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_bz2.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_ssl.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_decimal.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/_hashlib.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3.8/lib-dynload/

COPY --from=builder /usr/lib/python3/dist-packages /usr/lib/python3/dist-packages


COPY --from=builder \
    /usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3.9.0 \
    /usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3.9.0 \
    /usr/lib/aarch64-linux-gnu/lapack/

COPY --from=builder \
    /usr/lib/python3/dist-packages/numpy/core/_multiarray_tests.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3/dist-packages/numpy/core/_multiarray_umath.cpython-38-aarch64-linux-gnu.so \
    /usr/lib/python3/dist-packages/numpy/core/

COPY --from=builder \
    /usr/lib/aarch64-linux-gnu/blas/libblas.so.3.9.0  /usr/lib/aarch64-linux-gnu/


COPY --from=builder \
    /usr/lib/libogdi.so.4.1 \
    /usr/lib/libdfalt.so.0.0.0 \
    /usr/lib/libmfhdfalt.so.0.0.0 \
    /usr/lib/libarmadillo.so.9.800.4 \
    /usr/lib/libgdal.so.26.0.4 \
    /usr/lib/

COPY --from=builder \
    /usr/local/lib/librealsense2.so.2.50.0 \
    /usr/local/lib/

COPY --from=builder  /etc/ld.so.conf /etc/ld.so.conf


RUN chmod 644 /usr/lib/python3/dist-packages/*.so 

COPY --from=builder /opt/ros/noetic /opt/ros_backup/noetic

# Set up entrypoint script to manage symlinks and copy missing files
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]