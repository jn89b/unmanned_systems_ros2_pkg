#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TurtleBotNode(Node):

    def __init__(self):
        super().__init__('minimial_turtlebot')
        
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel" ,  10) 
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        
        self.current_position = [0,0]

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main(args=None):
    rclpy.init(args=args)
    print("starting")

    turtlebot_node = TurtleBotNode()
    des_x_position = 2.0
    cmd_vel = 0.2
    stop_vel = 0.0

    while rclpy.ok():
        
        if turtlebot_node.current_position[0] <= des_x_position:
            turtlebot_node.move_turtle(cmd_vel, 0.0)
            print("not there yet", turtlebot_node.current_position[0])    
        
        elif turtlebot_node.current_position[0] >= des_x_position:
            turtlebot_node.move_turtle(stop_vel, 0.0)
            
        rclpy.spin_once(turtlebot_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    """apply imported function"""
    main()