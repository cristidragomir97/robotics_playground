#!/bin/bash

# Define paths
ROS_VOLUME_PATH="/opt/ros/noetic"
ROS_BACKUP_PATH="/opt/ros_backup/noetic"
LIB_PATH="/usr/lib"

# Function to create symlinks for libraries in /usr/lib and its subdirectories
#!/bin/bash

LIB_PATH="/usr/lib"

# Function to create symlinks for libraries in /usr/lib and its subdirectories
create_symlinks() {
    find $LIB_PATH -type f -name "*.so.*" | while read lib; do
        # Extract the base name and the directory where the library is located
        base_lib=$(basename $lib)
        dir_lib=$(dirname $lib)

        echo "Processing library: $lib"

        # Create the base symlink (e.g., libpoco.so)
        base_symlink="${base_lib%%.so.*}.so"
        if [ ! -f "$dir_lib/$base_symlink" ] && [ "$lib" != "$dir_lib/$base_symlink" ]; then
            ln -sf "$lib" "$dir_lib/$base_symlink"
            echo "Created symlink: $dir_lib/$base_symlink"
        fi

        # Extract the major version (e.g., libpoco.so.6)
        if [[ "$base_lib" =~ \.so\.[0-9]+ ]]; then
            major_version=$(echo "$base_lib" | grep -oP "\.so\.[0-9]+")
            major_symlink="${base_lib%%.so.*}$major_version"
            if [ ! -f "$dir_lib/$major_symlink" ] && [ "$lib" != "$dir_lib/$major_symlink" ]; then
                ln -sf "$lib" "$dir_lib/$major_symlink"
                echo "Created symlink: $dir_lib/$major_symlink"
            fi
        fi

        # Create symlink for the minor version (e.g., libpoco.so.6.3)
        if [[ "$base_lib" =~ \.so\.[0-9]+\.[0-9]+ ]]; then
            minor_version="${base_lib%.*}"
            if [ ! -f "$dir_lib/$minor_version" ] && [ "$lib" != "$dir_lib/$minor_version" ]; then
                ln -sf "$lib" "$dir_lib/$minor_version"
                echo "Created symlink: $dir_lib/$minor_version"
            fi
        fi
    done
}

# Call the function to create symlink




# Check if the volume is mounted and contains files
if [ "$(ls -A $ROS_VOLUME_PATH)" ]; then
  echo "ROS volume detected at $ROS_VOLUME_PATH, checking for missing files..."
  # Symlink missing files from the backup path
  find $ROS_BACKUP_PATH -type f | while read file; do
    dest_file="${file/$ROS_BACKUP_PATH/$ROS_VOLUME_PATH}"
    if [ ! -f "$dest_file" ]; then
      ln -s "$file" "$dest_file"
    fi
  done
else
  echo "No ROS volume detected or empty, symlinking files from backup..."
  # Symlink everything from backup if volume is empty
  find $ROS_BACKUP_PATH -type f | while read file; do
    dest_file="${file/$ROS_BACKUP_PATH/$ROS_VOLUME_PATH}"
    ln -s "$file" "$dest_file"
  done
fi

echo "creating symlinks"
create_symlinks

# Run ldconfig to ensure the system cache is updated with new libraries
echo "Running ldconfig..."

echo "/usr/lib/aarch64-linux-gnu" >> /etc/ld.so.conf
echo "/usr/lib/aarch64-linux-gnu/lapack" >> /etc/ld.so.conf
echo "/usr/lib/aarch64-linux-gnu/blas" >> /etc/ld.so.conf
echo "/usr/lib" >> /etc/ld.so.conf
echo "/usr/local/lib" >> /etc/ld.so.conf
ldconfig 


echo $(ls -l /usr/lib/aarch64-linux-gnu/lapack/liblapack.so*)

ldd /root/catkin_ws/devel/lib/librealsense2_camera.so

source /opt/ros/noetic/setup.sh


# Source ROS environment
source $ROS_VOLUME_PATH/setup.bash
source /root/catkin_ws/devel/setup.bash

# Run the RealSense camera launch file with the desired settings
exec roslaunch realsense2_camera rs_camera.launch depth_width:=424 depth_height:=240 depth_fps:=60 color_width:=320 color_height:=240 color_fps:=60
