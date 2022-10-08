'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 04:34:58
FilePath: /bubble_bringup/launch/bubble_launch.py
LastEditors: Ligcox
LastEditTime: 2022-07-01 11:23:34
E-mail: robomaster@birdiebot.top
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, Node


arg_list = [
    # Allow the user to choose how buffering on the stream works by setting
    # RCUTILS_LOGGING_BUFFERED_STREAM.
    # With an empty environment variable, use the default of the stream.
    # With a value of 0, force the stream to be unbuffered.
    # With a value of 1, force the stream to be line buffered.
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    # Declare the launch options
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    ),
    DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the bubble stack'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    ),
    DeclareLaunchArgument(
        "use_regular_send",
        default_value="False",
        description="Whether the node will send masseges to topic use fixed frequency",
        choices=['True', 'False']
    ),
    DeclareLaunchArgument(
        "use_synchronize",
        default_value="False",
        description="Whether sync massege in gimbal massege and targe massege",
        choices=['True', 'False']
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('bubble_bringup'), 'config', 'bubble_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'),
    DeclareLaunchArgument(
        'robot_type',
        default_value='standard',
        description='Robot name',
        choices=[
            "sentry_up", "sentry_down", "infantry", "engineer", "hero", "air", "radar", "gather", "standard"
        ]),
    DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/tty/none',
        description='Onboard serial port name'
    ),
]


def generate_launch_description():
    return LaunchDescription(arg_list + [
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
                namespace=LaunchConfiguration("namespace"),
            ),

            # launch bubble core
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('bubble_protocol'), 'bcp_api_core_launch.py')),
                launch_arguments={
                    'robot_type': LaunchConfiguration('robot_type'),
                    'serial_port': LaunchConfiguration('serial_port'),
                }.items()
            ),

            # launch bubble FSM package
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('bubble_decision'), 'launch', 'decision_launch.py')),
                launch_arguments={
                    'robot_type': LaunchConfiguration('robot_type'),
                }.items()
            ),

            # launch bubble aiming package
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('bubble_aiming'), "launch", 'aiming_launch.py')),
                launch_arguments={
                    'params_file': LaunchConfiguration("params_file"),
                    "use_regular_send": LaunchConfiguration("use_regular_send"),
                    "use_synchronize": LaunchConfiguration("use_synchronize")
                }.items()
            ),

            # launch bubble debug packages
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('bubble_debuger'), "launch", 'debuger_launch.py')),
                condition=IfCondition(LaunchConfiguration('debug_mode')),
                launch_arguments={
                    'params_file': LaunchConfiguration("params_file"),
                }.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(PythonExpression([LaunchConfiguration(
                        'use_robot_state'), " and ", LaunchConfiguration('debug_mode')])),
                arguments=['-d', os.path.join(
                    get_package_share_directory('bubble_bringup'), 'rviz', 'bubble_debug.rviz')
                ]
            ),
        ])
    ])
