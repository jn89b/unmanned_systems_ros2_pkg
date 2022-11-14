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

from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    chatter_ns_launch_arg = DeclareLaunchArgument(
        "turtle_ns", default_value=TextSubstitution(text="turtle")
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    # include another launch file in the chatter_ns namespace
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    launch_include_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace('turtle'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir,'/navigation2.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ]
    )

    #add actions
    ld = LaunchDescription()
    ld.add_action(launch_include_with_namespace)

    return ld