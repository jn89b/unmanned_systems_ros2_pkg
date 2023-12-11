#!/usr/bin/env python3
## example of launch file
# https://answers.ros.org/question/379101/spawning-multiple-robots-in-gazebo-and-assinging-namespaces/
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction

from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    foot_ninja_1_move = Node(
        package="unmanned_systems_ros2_pkg",
        executable="foot_ninja_1.py",
        name='foot_ninja_1_move',
    )

    foot_ninja_2_move = Node(
        package="unmanned_systems_ros2_pkg",
        executable="foot_ninja_2.py",
        name='foot_ninja_2_move',
    )

    ld = LaunchDescription()
    ld.add_action(foot_ninja_1_move)
    ld.add_action(foot_ninja_2_move)
    
    return ld