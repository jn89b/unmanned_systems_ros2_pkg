#!/usr/bin/env python3

import rclpy
import math as m
import numpy as np
from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from unmanned_systems_ros2_pkg import ProNav


def get_mean_heading_target(heading_list:list)-> float:
     heading_list = np.array(heading_list)
     mean_heading_target = np.mean(heading_list)
     return mean_heading_target   

def compute_global_heading(heading_target_rad:float, 
                        curent_yaw_rad:float):
    
    global_heading_rad = heading_target_rad + curent_yaw_rad
    
    if global_heading_rad > 2*np.pi:
        global_heading_rad = global_heading_rad - 2*np.pi
    elif global_heading_rad < 0:
        global_heading_rad = global_heading_rad + 2*np.pi
    
    # print("global heading deg", np.rad2deg(global_heading_rad))
    
    return global_heading_rad

def main() -> None:
    rclpy.init(args=None)
    
    turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')    
    turtlebot_pursuer.move_turtle(0.0,0.0)
    
    # 5 works well for my side increase to make it more snappier on turns 
    pro_nav = ProNav.ProNav(10)    
    dt = 0.01 
    
    #odom frequency 
    odom_sampling_freq = 30.0
    
    dt = 1/3 #* 10
    
    old_evader_position = np.array([2,1])
    
    while rclpy.ok():
        
        rclpy.spin_once(turtlebot_pursuer)
        
        mean_target = get_mean_heading_target(
            turtlebot_pursuer.detected_heading_angle_list)
                 
        global_heading_ref = compute_global_heading(
            np.deg2rad(mean_target), turtlebot_pursuer.orientation_euler[2]
        )
        
        evader_position = np.array(turtlebot_pursuer.evader_position)
    
        evader_velocity = (evader_position - old_evader_position)/dt
                
        flight_path_rate, cmd_vel = pro_nav.true_pro_nav(
            np.array(turtlebot_pursuer.current_position), 
            evader_position,
            dt, 
            evader_velocity, 
            np.array(turtlebot_pursuer.current_velocity),
            True, global_heading_ref    
        )
        
        old_evader_position = evader_position
        turtlebot_pursuer.move_turtle(cmd_vel, flight_path_rate)
    
if __name__ == '__main__':
    main()
    

