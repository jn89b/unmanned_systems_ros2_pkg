import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    """look for urdf launch file"""
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    
    assert os.path.exists(urdf), "Turtlebot urdf exists "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name = 'robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
    )


    name = "some_robot"
    spawn_x_val = 2.0
    spawn_y_val = 2.0
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


    ld = LaunchDescription()
    ld.add_action(gazebo_launch_arg)
    ld.add_action(robot_state_pub_node)
    
    return ld
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='box_bot_description', executable='spawn_box_bot.py', arguments=[urdf], output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
    ])