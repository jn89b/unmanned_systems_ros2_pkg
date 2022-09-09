import os

from launch import LaunchDescription
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_turtle_names(num_turtles:int) -> str:
    tmnt = ["Leonardo", "Donatello", "Michaelangelo"]
    returned_turtles = []
    
    for i in range(0,num_turtles+1):
        turtle_info = {"id:": tmnt[i],
                     "x_pose": i,
                     "y_pose": i,
                     "z_pose": 0.01}
        returned_turtles.append(turtle_info)
        
    return returned_turtles
    
def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    
    
    turtles = generate_turtle_names(2)
    
    #create list of turtles
    spawn_turtle_cmds = []
    for turtle in turtles:
        IncludeLaunchDescription()
