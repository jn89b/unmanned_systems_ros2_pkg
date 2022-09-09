#!/usr/bin/env python3
## example of launch file
# https://answers.ros.org/question/379101/spawning-multiple-robots-in-gazebo-and-assinging-namespaces/
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

#https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/

def generate_launch_description():
    """launch"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'empty_worlds/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    print("path directory", get_package_share_directory('turtlebot3_description'))

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.sdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf])

    # Pose where we want to spawn the robot
    name = "turtle"
    spawn_x_val = 1.0
    spawn_y_val = 1.0
    spawn_z_val = 0.00
    spawn_yaw_val = 0.00
    
    spawn_second_tb = Node(
        package="unmanned_systems_ros2_pkg",
        executable="turtlebot_spawn.py",
        parameters=[
                    {'gazebo_name' : name}, 
                    {'x_pos': spawn_x_val }, 
                    {'y_pos': spawn_y_val}, 
                    {'z_pos': spawn_z_val}, 
                    {'yaw_pos': spawn_yaw_val}]
    )

    #start gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                    )
                )
    

    #add actions
    ld = LaunchDescription()
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_second_tb)
    # ld.add_action(spawn_entity_cmd)
    # ld.add_action(state_publisher_cmd)

    return ld

