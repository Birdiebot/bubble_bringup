'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 04:34:58
FilePath: /bubble/src/bubble_bringup/launch/infantry_launch.py
LastEditors: HarryWen
LastEditTime: 2022-06-04 22:33:52
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
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Allow the user to choose how buffering on the stream works by setting
    # RCUTILS_LOGGING_BUFFERED_STREAM.
    # With an empty environment variable, use the default of the stream.
    # With a value of 0, force the stream to be unbuffered.
    # With a value of 1, force the stream to be line buffered.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    bringup_dir = get_package_share_directory('bubble_bringup')
    decision_dir = get_package_share_directory('bubble_decision')

    def launch_dir(dir): return os.path.join(dir, 'launch')

    robot_type = LaunchConfiguration('robot_type')
    serial_port = LaunchConfiguration('serial_port')

    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='infantry',
        description='Robot name, optional [sentry_up| sentry_down| infantry| engineer| hero| air| radar| gather| standard.]')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'infantry_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_serial_port_file_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS0',
        description='Infantry robot Onboard serial port name')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir(bringup_dir), 'bubble_launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('params_file'),
                              'robot_type': robot_type,
                              'serial_port': serial_port
                              }.items()
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         launch_dir(decision_dir), 'decision_launch.py')),
        #     launch_arguments={
        #         'robot_type': robot_type,
        #     }.items()
        # ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_serial_port_file_cmd)

    # Add the actions to launch all of the bubble nodes
    ld.add_action(bringup_cmd_group)

    return ld
