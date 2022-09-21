#!/usr/bin/env python3

#import modules here
import rclpy
import math 
from rclpy.node import Node

#import messages 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
    
    #initiate node 
    rclpy.init(args=None)
    
    #create some node
    some_node = SomeNode()

    
    while rcply.ok():
        pass
    

if __name__ =='__main__':
    pass
