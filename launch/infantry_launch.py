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
from launch.substitutions import LaunchConfiguration

ROBOT_TYPE = 'infantry'


def launch_dir(dir): return os.path.join(dir, 'launch')


bringup_dir = get_package_share_directory('bubble_bringup')
decision_dir = get_package_share_directory('bubble_decision')
state_publisher_dir = get_package_share_directory('bubble_state_publisher')

# Declare the launch options
args_list = [
    # Declare the launch options of core params
    DeclareLaunchArgument(
        'robot_type',
        default_value=ROBOT_TYPE,
        description='Robot name',
        choices=[
            "sentry_up", "sentry_down", "infantry", "engineer", "hero", "air", "radar", "gather", "standard"
        ]),
    DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS0',
        description='Infantry robot Onboard serial port name'),


    # Declare the launch options of stack params file
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'config', ROBOT_TYPE + '_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'),
    DeclareLaunchArgument(
        'visual_params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'visual_aiming_params.yaml'),
        description='Visual detector param file'),
    DeclareLaunchArgument(
        'camera_params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'camera_params.yaml'),
        description='Camera node param file'),


    # Declare the launch options of aming node
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


    # Declare the launch options of visual params
    DeclareLaunchArgument(
        "debug_mode",
        default_value="True",
        description="Enable debug mode",
        choices=['True', 'False']
    ),
    DeclareLaunchArgument(
        "model_path",
        default_value="/home/nvidia/Desktop/bubble/src/bubble_resources/model/autoaming_SJTU2021_base.onnx",
        description="yolox onnx model absolute path."
    ),
    DeclareLaunchArgument(
        "use_rune_infer",
        default_value="False",
        description="Enable rune detector",
        choices=['True', 'False']
    ),


    # Declare the launch options of robot state params
    DeclareLaunchArgument(
        "use_robot_state",
        default_value="False",
        description="Whether to apply robot state publisher.",
        choices=['True', 'False']
    ),
    DeclareLaunchArgument(
        "urdf_file_path",
        default_value=os.path.join(
            state_publisher_dir, ROBOT_TYPE + '_description_mesh.urdf'),
        description="Robot descript urdf model absolute path."
    ),
]


def generate_launch_description():
    # Create the launch description and populate
    return LaunchDescription(args_list + [
        # Allow the user to choose how buffering on the stream works by setting
        # RCUTILS_LOGGING_BUFFERED_STREAM.
        # With an empty environment variable, use the default of the stream.
        # With a value of 0, force the stream to be unbuffered.
        # With a value of 1, force the stream to be line buffered.

        # Set environment variables
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Specify the actions
        GroupAction([

            # launch bubble contrib packages
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    launch_dir(bringup_dir), 'bubble_launch.py')),
                launch_arguments={
                    'params_file': LaunchConfiguration('params_file'),
                    'robot_type': LaunchConfiguration('robot_type'),
                    'serial_port':  LaunchConfiguration('serial_port')

                }.items(),
            ),

            # launch bubble visual composable node container
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    launch_dir(bringup_dir), 'visual_launch.py')),
                launch_arguments={
                    "visual_params_file": LaunchConfiguration('visual_params_file'),
                    "camera_params_file": LaunchConfiguration('camera_params_file'),
                    "use_rune_infer": LaunchConfiguration('use_rune_infer'),
                    "model_path": LaunchConfiguration("model_path"),
                    "debug_mode": LaunchConfiguration("debug_mode"),
                }.items()
            ),



            # launch bubble robot state publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    state_publisher_dir, 'state_publisher_launch.py')),
                condition=IfCondition(LaunchConfiguration('use_robot_state')),
                launch_arguments={
                    "params_file": LaunchConfiguration('params_file'),
                    "urdf_file_path": LaunchConfiguration('urdf_file_path'),
                }.items()
            ),
        ])
    ])
