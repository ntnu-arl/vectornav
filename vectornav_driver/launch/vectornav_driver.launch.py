#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('vectornav_driver')
    
    # Path to the parameter file
    params_file = os.path.join(pkg_dir, 'config', 'vn100_params.yaml')
    
    return LaunchDescription([
        Node(
            package='vectornav_driver',
            executable='vectornav_driver_node',
            name='vectornav_driver_node',
            output='screen',
            parameters=[params_file],
            # arguments=["--ros-args", "--log-level", "debug"]
        )
    ])