#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    turtle_ns = LaunchConfiguration('turtlebot_ns', 
        default='turtle_ns')

    logger_ns = LaunchConfiguration('logger_ns',
        default='logger_ns')

    turtle_node = Node(
        package='unmanned_systems_ros2_pkg',
        namespace=turtle_ns,
        executable='turtlebot_simple.py'
        )
    
    logger_node = Node(
        package='unmanned_systems_ros2_pkg',
        namespace=turtle_ns,
        executable='logger.py'
    )

    launch_description = LaunchDescription(
        [
        turtle_node,
        logger_node
        ]
    )

    return launch_description

