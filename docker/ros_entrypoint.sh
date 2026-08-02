#!/bin/bash
set -e

# set ROS_DISTRO if it's missing
if [ -z "$ROS_DISTRO" ]; then
  for dir in /opt/ros/*; do
    if [ -f "$dir/setup.bash" ]; then
      export ROS_DISTRO=$(basename "$dir")
    fi
  done
fi

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/opt/ros2_ws/install/setup.bash" --

exec "$@"
