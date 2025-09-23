#!/usr/bin/env python3
"""
Turtlesim Demo with MCP Integration Launch File

This launch file starts the core components for the Turtlesim PID demo:
- Turtlesim simulation node
- PID controller for precise movement
- MCP bridge service for LLM integration

For complete functionality with ros-mcp-server, you need to separately launch:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Usage:
    # Terminal 1: Start this demo
    ros2 launch turtle_demo_controller turtle_demo_with_mcp.launch.py
    
    # Terminal 2: Start rosbridge (in a separate terminal)
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    
    ros-mcp-server is launched from LMStudio
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # Launch the unified PID controller (handles both GoToPose and RotateToAngle actions)
        Node(
            package='turtle_demo_controller',
            executable='turt_controller',
            name='turtle_pid_controller',
            output='screen'
        ),

        # Launch MCP bridge service for LLM integration
        Node(
            package='turtle_demo_controller',
            executable='mcp_bridge_service',
            name='mcp_bridge_service',
            output='screen'
        ),
    ])
