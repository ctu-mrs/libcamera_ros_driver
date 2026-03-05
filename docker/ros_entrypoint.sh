#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/root/ros2_ws/install/setup.bash" --

ros2 launch libcamera_ros_driver camera.launch.py $@
