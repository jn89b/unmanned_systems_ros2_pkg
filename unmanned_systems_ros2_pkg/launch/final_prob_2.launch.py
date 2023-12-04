
# Date: September 19, 2021
# Description: Load a world file into Gazebo.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

	# Set the path to the Gazebo ROS package
	pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

	# Set the path to this package.
	pkg_share = FindPackageShare(package='unmanned_systems_ros2_pkg').find('unmanned_systems_ros2_pkg')

	# Set the path to the world file
	world_file_name = 'final_exam/final_prob_2.world'
	world_path = os.path.join(pkg_share, 'worlds',  world_file_name)

	# Set the path to the SDF model files.
	gazebo_models_path = os.path.join(pkg_share, 'models')
	os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

	########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
	# Launch configuration variables specific to simulation
	headless = LaunchConfiguration('headless')
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_simulator = LaunchConfiguration('use_simulator')
	world = LaunchConfiguration('world')

	declare_simulator_cmd = DeclareLaunchArgument(
	name='headless',
	default_value='False',
	description='Whether to execute gzclient')
		
	declare_use_sim_time_cmd = DeclareLaunchArgument(
	name='use_sim_time',
	default_value='true',
	description='Use simulation (Gazebo) clock if true')

	declare_use_simulator_cmd = DeclareLaunchArgument(
	name='use_simulator',
	default_value='True',
	description='Whether to start the simulator')

	declare_world_cmd = DeclareLaunchArgument(
	name='world',
	default_value=world_path,
	description='Full path to the world model file to load')

	# Specify the actions

	# Start Gazebo server
	start_gazebo_server_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
	condition=IfCondition(use_simulator),
	launch_arguments={'world': world}.items())

	# Start Gazebo client    
	start_gazebo_client_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
	condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

	#load turtlebot
	urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
	urdf = os.path.join(
		get_package_share_directory('turtlebot3_description'),
		'urdf',
		urdf_file_name)
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	#Pursuer
	pursuer = "pursuer"
	foot_ninja_x = 0.0
	foot_ninja_y = 0.0
	foot_ninja_z = 0.00
	spawn_yaw_val = 0.00
	
	pursuer_tb = Node(
        package="unmanned_systems_ros2_pkg",
        executable="turtlebot_spawn.py",
		name='spawn_turtlebot',
		namespace=pursuer,
		remappings = [('/tf', 'tf'),
				('/tf_static', 'tf_static')],
		parameters=[
					{'gazebo_name' : pursuer}, 
					{'x_pos': foot_ninja_x }, 
					{'y_pos': foot_ninja_y}, 
					{'z_pos': foot_ninja_z}, 
					{'yaw_pos': spawn_yaw_val},
					{'model_name': pursuer}]
	)

	state_publisher_cmd = Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			namespace=pursuer,
			remappings = [('/tf', 'tf'),
				  ('/tf_static', 'tf_static')],
			parameters=[{'use_sim_time': use_sim_time}],
			arguments=[urdf])

	#Pursuer
	evader = "evader"
	foot_ninja_x = 2.0
	foot_ninja_y = 1.0
	foot_ninja_z = 0.00
	spawn_yaw_val = 0.00
	
	evader_tb = Node(
        package="unmanned_systems_ros2_pkg",
        executable="turtlebot_spawn.py",
		name='spawn_turtlebot',
		namespace=evader,
		remappings = [('/tf', 'tf'),
				('/tf_static', 'tf_static')],
		parameters=[
					{'gazebo_name' : evader}, 
					{'x_pos': foot_ninja_x }, 
					{'y_pos': foot_ninja_y}, 
					{'z_pos': foot_ninja_z}, 
					{'yaw_pos': spawn_yaw_val},
					{'model_name': evader}]
	)

	evader_publisher_cmd = Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			namespace=evader,
			remappings = [('/tf', 'tf'),
				  ('/tf_static', 'tf_static')],
			parameters=[{'use_sim_time': use_sim_time}],
			arguments=[urdf])

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_simulator_cmd)
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_use_simulator_cmd)
	ld.add_action(declare_world_cmd)

	# Add any actions
	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)

	ld.add_action(pursuer_tb)
	ld.add_action(state_publisher_cmd)

	ld.add_action(evader_tb)
	ld.add_action(evader_publisher_cmd)

	return ld