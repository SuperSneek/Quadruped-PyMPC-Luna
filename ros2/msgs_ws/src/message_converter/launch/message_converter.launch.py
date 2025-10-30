#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('message_converter')

    # Configuration file path
    blind_state_config_path = os.path.join(
        pkg_dir, 'config', 'blind_state_converter_plugin.yaml'
    )
    base_state_config_path = os.path.join(
        pkg_dir, 'config', 'base_state_converter_plugin.yaml'
    )

    # Create the node with configuration
    node = Node(
        package='message_converter',
        executable='message_converter_node',
        name='message_converter',
        output='screen',
        parameters=[
            blind_state_config_path,
            base_state_config_path,
            {'use_sim_time': True}
        ]
    )
    
    return LaunchDescription([node])