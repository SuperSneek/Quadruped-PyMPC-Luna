#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('message_converter')


    # Conditionally select contact plugin configuration
    blind_state_config_path = os.path.join(
        pkg_dir, 'config', 'contact_plugin.yaml'
    )

    # Function to launch the node with conditional config
    def launch_node(context, *args, **kwargs):
        # Create the node with selected configuration
        node = Node(
            package='message_converter',
            executable='message_converter_node',
            name='message_converter',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                blind_state_config_path
            ]
        )
        return [node]
    
    return LaunchDescription([
        OpaqueFunction(function=launch_node)
    ])
