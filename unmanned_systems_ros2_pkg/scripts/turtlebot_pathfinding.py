#!/usr/bin/env python3

#import modules here
import rclpy
import math 
from rclpy.node import Node

#import messages 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from unmanned_systems_ros2_pkg import PathFinding, Map

# define functions here 

# class node to run
class SomeNode(Node):
    def __init__(self):
        super().__init__('node_name')
        
        #publisher stuff
        buffer_size = 10
        pub_topic = "pub_topic"
        self.some_publisher = self.create_publisher(Twist, pub_topic, buffer_size)
        
        #subscriber stuff
        self.some_subscriber = self.create_subscription(Odometry, self.some_callback, buffer_size)

    def some_callback(self, msg:Odometry) -> None:
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
    def some_class_function() -> None:
        """you can include class functions here"""
        print("hello world")
        
def main() -> None:
    """run your main loop here"""
    
    #initiate map
    x_span = [0,10]
    y_span = [0,10]
    grid_space = 0.5
    
    enemy_list = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
    configSpace = Map.ConfigSpace(x_span, y_span, grid_space, enemy_list)
    configSpace.set_graph_coords()
    obstacle_radius = 0.5
    
    """define start and end"""
    start_position = (0,0)
    goal_position = (8, 8)
    
    step_size = 1
    move_list = [[step_size,0], #move left
                [-step_size,0], #move right 
                [0,step_size], #move up
                [0,-step_size], #move down
                [step_size, step_size],
                [step_size, -step_size],
                [-step_size, step_size],
                [-step_size, -step_size]
                ]
    
    
    #initiate node 
    rclpy.init(args=None)
    
    #create some node
    # some_node = SomeNode()

    #while your running node
    
    astar = PathFinding.Astar((0,0), (5,5), move_list,
                              configSpace, obstacle_radius)
        
    astar_path = astar.find_path()
    
    print("path is", astar_path)
    # while rclpy.ok():
    #     rclpy.spin(some_node)
    

if __name__ =='__main__':
    main()
