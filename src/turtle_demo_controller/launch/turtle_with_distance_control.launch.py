#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Launch the main PID controller (handles GoToPose actions)
        Node(
            package='turtle_demo_controller',
            executable='turt_controller',
            name='turtle_pid_controller',
            output='screen'
        ),
        
        # Note: Distance functionality is now provided by the distance_controller client
        # No separate service needed - the client calculates endpoints and uses position mode directly
    ])
