#!/usr/bin/env python3
"""
- Keep track of foot ninja's position 
- Keep track of turtlebot's position
- If ANY foot ninja heading and distance within bound of turtlebot:
    - Notify user they've been spotted
    - Else keep looking

"""
import rclpy
import math as m
import os

from functools import partial
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unmanned_systems_ros2_pkg import quaternion_tools, PIDTemplate
from ament_index_python.packages import get_package_share_directory


from PIL import Image

class RobotAgent():
    def __init__(self, agent_name:str, position_vector:list, heading:float):
        self.agent_name = agent_name
        self.position_vector = position_vector
        self.heading = heading


class DungeonMaster(Node):
    def __init__(self, node_name:str, agent_odom_topics:str):
        super().__init__(node_name)

        #what the agent topics names are, you can change them here
        self.agent_info = {}

        #this is how we keep track of the game states of our agents
        for agent in agent_odom_topics:
            self.agent_info[agent] = RobotAgent(agent,[None,None], None)
            
        #don't want to have duplicate code so mapped agent odom topics 
        for agent in agent_odom_topics:
            self.subscription_odom = self.create_subscription(
                        Odometry,
                        agent+'/odom',
                        partial(self.odom_callback, name=agent),
                        1
                    )
            self.subscription_odom  # prevent unused variable warning


    def odom_callback(self, msg:Odometry, name:str):
        """typical odom subscription"""
        agent = self.agent_info[name]

        x =  msg.pose.pose.position.x
        y =  msg.pose.pose.position.y
        z =  msg.pose.pose.position.z
        agent.position_vector = [x, y]
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)
        agent.heading = yaw


def main() -> None:
    rclpy.init(args=None)
    
    agent_names = ['foot_ninja_1', 'foot_ninja_2', '']
    dungeon_master = DungeonMaster('dungeon_master', agent_names)

    foot_ninja_detection_range = 0.4#meters

    #this is the bad guys
    foot_ninjas = [dungeon_master.agent_info[agent_names[0]], 
                dungeon_master.agent_info[agent_names[1]]]

    #this is the good guy
    leonardo = dungeon_master.agent_info[agent_names[2]]
    
    asset_directory = os.path.join(get_package_share_directory('unmanned_systems_ros2_pkg'), 'assets')

    while rclpy.ok():

        #check if we have info if not then callback
        if leonardo.position_vector == [None, None]:
            rclpy.spin_once(dungeon_master)
            continue

        #check if ninja detects leonardo
        for ninja in foot_ninjas:
            
            if ninja.position_vector == [None, None]:
                continue 

            distance = m.dist(leonardo.position_vector, 
                                ninja.position_vector)

            if distance <= foot_ninja_detection_range:
                #print("You've been caught!")
                im = Image.open(asset_directory+"/game_over.jpg")
                im.show()
                
                rclpy.shutdown()

        rclpy.spin_once(dungeon_master)


if __name__=='__main__':
    main()