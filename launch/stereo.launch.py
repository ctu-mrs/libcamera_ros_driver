#!/usr/bin/env python3

# Launches TWO camera nodes in a SINGLE component container (one process).
# This is what gives a true stereo pair coherent timestamps: both sensors are
# driven from the same libcamera CameraManager in the same process. libcamera
# forbids more than one CameraManager per process, so the driver shares one
# (see getCameraManager() in src/libcamera_ros_driver.cpp).
#
# Each node selects a distinct sensor via its custom_config. Override the node
# name, custom_config and calibration per eye with the left_*/right_* args.

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'libcamera_ros_driver'
    this_pkg_path = get_package_share_directory(pkg_name)

    try:
        libcamera_path = get_package_prefix('libcamera_ros')
    except Exception:
        libcamera_path = '/opt/ros/jazzy'

    uav_name = LaunchConfiguration('uav_name')
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The uav name used for namespacing.',
    ))

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the nodes subscribe to sim time?',
    ))

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    default_calib = f'file://{this_pkg_path}/config/calib/libcamera.yaml'

    # per-camera args: node name suffix, custom_config (sensor selection), calib
    ld.add_action(DeclareLaunchArgument('left_name', default_value='front'))
    ld.add_action(DeclareLaunchArgument('left_custom_config', default_value=this_pkg_path + '/config/camera_left.yaml'))
    ld.add_action(DeclareLaunchArgument('left_calib_url', default_value=default_calib))

    ld.add_action(DeclareLaunchArgument('right_name', default_value='back'))
    ld.add_action(DeclareLaunchArgument('right_custom_config', default_value=this_pkg_path + '/config/camera_right.yaml'))
    ld.add_action(DeclareLaunchArgument('right_calib_url', default_value=default_calib))

    # libcamera runtime environment (same as camera.launch.py)
    ld.add_action(SetEnvironmentVariable(
        name='LIBPISP_BE_CONFIG_FILE',
        value=os.environ.get('LIBPISP_BE_CONFIG_FILE', f'{libcamera_path}/share/libpisp/backend_default_config.json')))
    ld.add_action(SetEnvironmentVariable(
        name='LIBCAMERA_IPA_MODULE_PATH',
        value=os.environ.get('LIBCAMERA_IPA_MODULE_PATH', f'{libcamera_path}/lib/libcamera/ipa/')))
    ld.add_action(SetEnvironmentVariable(
        name='LIBCAMERA_IPA_CONFIG_PATH',
        value=os.environ.get('LIBCAMERA_IPA_CONFIG_PATH', f'{libcamera_path}/share/libcamera/ipa')))

    def camera_node(side, custom_config, calib_url):
        return ComposableNode(
            package=pkg_name,
            plugin='libcamera_ros_driver::LibcameraRosDriver',
            namespace=uav_name,
            name=['rpi_camera_', side],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'frame_id': [uav_name, '/rpi_camera_', side]},
                {'calib_url': calib_url},
                {'config': this_pkg_path + '/config/default.yaml'},
                {'custom_config': custom_config},
            ],
            remappings=[
                ('~/image_raw', '~/image_raw'),
                ('~/camera_info', '~/camera_info'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    left = camera_node(LaunchConfiguration('left_name'),
                       LaunchConfiguration('left_custom_config'),
                       LaunchConfiguration('left_calib_url'))
    right = camera_node(LaunchConfiguration('right_name'),
                        LaunchConfiguration('right_custom_config'),
                        LaunchConfiguration('right_calib_url'))

    container = ComposableNodeContainer(
        namespace=uav_name,
        name='rpi_camera_stereo_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[left, right],
        parameters=[
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
    )

    ld.add_action(container)

    return ld
