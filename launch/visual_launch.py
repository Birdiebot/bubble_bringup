'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 04:34:58
FilePath: /bubble/src/bubble_bringup/launch/visual_launch.py
LastEditors: Ligcox
LastEditTime: 2022-07-16 03:15:16
E-mail: robomaster@birdiebot.top
'''

import os

from click import option

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

arg_list = [
    # Allow the user to choose how buffering on the stream works by setting
    # RCUTILS_LOGGING_BUFFERED_STREAM.
    # With an empty environment variable, use the default of the stream.
    # With a value of 0, force the stream to be unbuffered.
    # With a value of 1, force the stream to be line buffered.
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    DeclareLaunchArgument(
        "debug_mode",
        default_value="False",
        description="Enable debug mode",
        choices=['True', "False"]
    ),

    DeclareLaunchArgument(
        "model_path",
        default_value="/home/nvidia/Desktop/bubble/src/bubble_resources/module/autoaming_SJTU2021_base.onnx",
        description="yolox onnx model absolute path."
    ),

    DeclareLaunchArgument(
        'visual_params_file',
        default_value=os.path.join(
            get_package_share_directory('bubble_bringup'), 'config', 'visual_aiming_params.yaml'),
        description='Visual detector param file'),
    DeclareLaunchArgument(
        'camera_params_file',
        default_value=os.path.join(
            get_package_share_directory('bubble_bringup'), 'config', 'camera_params.yaml'),
        description='Camera node param file'
    ),
    DeclareLaunchArgument(
        "use_rune_infer",
        default_value="False",
        description="Enable rune detector",
        choices=['True', 'False']
    ),
]


def generate_launch_description():
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(arg_list + [
            ComposableNodeContainer(
                name='visual_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='bubble_camera',
                        plugin='vInput',
                        name='camera',
                        parameters=[{
                            'params_file': LaunchConfiguration('params_file'),
                        }],
                    ),
                    ComposableNode(
                        package='bubble_visual_sjtu',
                        plugin='VisualNode',
                        name='Visual',
                    ),
                    ComposableNode(
                        package='bubble_rune',
                        node_plugin='VisualNode',
                        node_name='Visual',
                        parameters=[{
                            'params_file': LaunchConfiguration('params_file'),
                            "model_path": LaunchConfiguration("model_path"),
                            "debug_mode": LaunchConfiguration("debug_mode"),
                        }],
                        output='screen'
                    )
                ])
        ])
    else:

        return LaunchDescription(arg_list + [
            ComposableNodeContainer(
                name='visual_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='bubble_camera',
                        plugin='vInput',
                        name='camera',
                        parameters=[LaunchConfiguration('camera_params_file')],
                    ),
                    ComposableNode(
                        package='bubble_visual_sjtu',
                        plugin='VisualNode',
                        name='Visual',
                        parameters=[
                            LaunchConfiguration('visual_params_file'), {
                                "model_path": LaunchConfiguration("model_path"),
                                "debug_mode": LaunchConfiguration("debug_mode"),
                            }],
                    ),
                    ComposableNode(
                        package='bubble_rune',
                        name='VisualRune',
                        plugin='VisualRune',
                        parameters=[
                            LaunchConfiguration('visual_params_file'), {
                                "model_path": LaunchConfiguration("model_path"),
                                "debug_mode": LaunchConfiguration("debug_mode"),
                            }],
                    ),
                ])
        ])
