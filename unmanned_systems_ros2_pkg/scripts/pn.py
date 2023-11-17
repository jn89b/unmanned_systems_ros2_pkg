#!/usr/bin/env python3

import rclpy
import math as m
import numpy as np
import threading 


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

    #lidar frequency 
    lidar_freq = 5.0 #hz

    turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')    
    turtlebot_pursuer.move_turtle(0.0,0.0)
    
    #since our lidar is super slow we're going to set this node to match our
    #lidar rate to about 3 times its sampling rate
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_pursuer, ), 
                              daemon=True)
    thread.start()
    rate = turtlebot_pursuer.create_rate(lidar_freq*3)
    
    # 5 works well for my side increase to make it more snappier on turns 
    
    #this value works well with simple pn
    #pro_nav = ProNav.ProNav(1.5)    
    
    #this value works well with true pn
    pro_nav = ProNav.ProNav(3.0)
    
    dt = 1/lidar_freq
    old_evader_position = np.array([2,1])
    
    while rclpy.ok():
        
        # rclpy.spin_once(turtlebot_pursuer)
        
        rate.sleep()
        
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
        
        # cmd_vel = 0.20
        # flight_path_rate = pro_nav.simple_pro_nav(
        #     global_heading_ref, dt
        # )
        
        # do this command for half a second        
        print("flight path rate", flight_path_rate)
        old_evader_position = evader_position
        turtlebot_pursuer.move_turtle(cmd_vel, flight_path_rate)
        
    
if __name__ == '__main__':
    main()
    

