#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'libcamera_ros_driver'
    this_pkg_path = get_package_share_directory(pkg_name)

    try:
        libcamera_path = get_package_prefix('libcamera_ros')
    except:
        libcamera_path = '/opt/ros/jazzy'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The uav name used for namespacing.',
    ))

    # #} end of uav_name

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ environment variables

    ld.add_action(SetEnvironmentVariable(
        name='LIBPISP_BE_CONFIG_FILE',
        value=os.environ.get(
            'LIBPISP_BE_CONFIG_FILE',
            f'{libcamera_path}/share/libpisp/backend_default_config.json'
        )
    ))

    ld.add_action(SetEnvironmentVariable(
        name='LIBCAMERA_IPA_MODULE_PATH',
        value=os.environ.get(
            'LIBCAMERA_IPA_MODULE_PATH',
            f'{libcamera_path}/lib/libcamera/ipa/'
        )
    ))

    ld.add_action(SetEnvironmentVariable(
        name='LIBCAMERA_IPA_CONFIG_PATH',
        value=os.environ.get(
            'LIBCAMERA_IPA_CONFIG_PATH',
            f'{libcamera_path}/share/libcamera/ipa'
        )
    ))

    # #} end of environment variables

    # #{ camera_name

    camera_name = LaunchConfiguration('camera_name')

    ld.add_action(DeclareLaunchArgument(
        'camera_name',
        default_value='front',
        description='Name of the camera (used to select which camera to use if multiple are present)'
    ))

    # #} end of camera_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    # #{ calib_url

    calib_url = LaunchConfiguration('calib_url')

    ld.add_action(DeclareLaunchArgument(
        'calib_url',
        default_value=os.getenv('CALIB_URL', f'file://{this_pkg_path}/config/calib/libcamera.yaml'),
        description='URL to the calibration file',
    ))

    # #} end of calib_url

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    ld.add_action(DeclareLaunchArgument(name='topic_namespace', default_value=''))

    # #} end of log_level

    # #{ default node

    default_node = ComposableNode(
        package=pkg_name,
        plugin='libcamera_ros_driver::LibcameraRosDriver',
        namespace=uav_name,
        name=['rpi_camera_', camera_name],

        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': [uav_name, "/rpi_camera_", camera_name]},
            {'calib_url': calib_url},
            {'config': this_pkg_path + '/config/default.yaml'},
            {'custom_config': custom_config},
        ],

        remappings=[
            # publishers
            ('~/image_raw', '~/image_raw'),
            ('~/camera_info', '~/camera_info'),
        ],

        extra_arguments=[
            {'use_intra_process_comms': True}
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[default_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of default node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=['rpi_camera_', camera_name, "_container"],
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[default_node],
        parameters=[
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of standalone container

    return ld
