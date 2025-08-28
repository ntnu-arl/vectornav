#!/usr/bin/env python3

# Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vectornav_driver'),
            'config',
            'vn100_params_ros2.yaml'
        ]),
        description='Path to the vectornav driver configuration file'
    )

    # Get the configuration file path
    config_file = LaunchConfiguration('config_file')

    # Create the vectornav_driver node
    vectornav_driver_node = Node(
        package='vectornav_driver',
        executable='vectornav_driver_node',
        name='vectornav_driver_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Add any necessary topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        vectornav_driver_node
    ])