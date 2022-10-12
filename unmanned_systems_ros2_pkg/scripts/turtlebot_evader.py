#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode

def main():
    rclpy.init(args=None)
    turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'turtle')    
    turtlebot_evader.move_turtle(0.0,0.0)
    
    while rclpy.ok():
    
        print("detected range is", turtlebot_evader.detected_heading_angle_list)
        print("detected range is", turtlebot_evader.detected_range_list)

        rclpy.spin_once(turtlebot_evader)

if __name__=="__main__":
    main()
