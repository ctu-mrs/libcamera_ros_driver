ARG PPA_VARIANT=stable

FROM ctumrs/mrs_uav_system:${PPA_VARIANT} AS base

WORKDIR /root/ros2_ws

RUN git clone --depth 1 -b ros2 https://github.com/ctu-mrs/libcamera_ros.git src/libcamera_ros && \
    git clone --depth 1 -b ros2 https://github.com/ctu-mrs/libcamera_ros_driver.git src/libcamera_ros_driver && \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    apt-get update && apt-get install -y meson python3-ply && colcon build --merge-install && \
    apt-get clean && rm -rf /var/lib/apt/lists/* build log src


FROM ctumrs/mrs_uav_system:${PPA_VARIANT}

COPY ros_entrypoint.sh /

COPY --from=base /root/ros2_ws/install /root/ros2_ws/install

ENTRYPOINT ["/ros_entrypoint.sh"]
