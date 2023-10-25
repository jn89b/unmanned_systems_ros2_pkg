#!/usr/bin/env python3
import rclpy
import math as m

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools


def generate_random_waypoints(n_random_waypoints:int, max_val:int)->list:
    """generate random waypoints from 1 to 1"""
    
    random_wp_list = []
    for i in range(0,n_random_waypoints+1):
        rand_x = randint(0, max_val)
        rand_y = randint(0, max_val)
        random_wp_list.append((rand_x, rand_y))
        
    return random_wp_list

def compute_desired_heading(current_pos:list, des_pos:list) -> float:
    """compute desired heading based on positions"""
    return m.atan2(des_pos[1] - current_pos[1] , des_pos[0] - current_pos[0])

def compute_dist_error(current_pos:list, des_pos:list)->float:
    """compute distance error"""
    return m.dist(des_pos,current_pos)

def compute_heading_error(current_heading:float, des_heading:float) -> float:
    """compute heading error in radians"""
    return des_heading - current_heading

def gimme_da_loot(turtlebot:TurtleBotNode, waypoint:list) -> list:
    """helper function"""
    desired_heading = compute_desired_heading(
        turtlebot.current_position, waypoint)
    
    heading_error = compute_heading_error(
        turtlebot.orientation_euler[2], desired_heading)

    dist_error = compute_dist_error(
        turtlebot.current_position, waypoint)
    
    return [desired_heading, heading_error, dist_error]


def main() -> None:
    rclpy.init(args=None)
    
    turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'evader')    
    turtlebot_evader.move_turtle(0.0,0.0)

    set_random = False
    is_done = False
    n_random_waypoints = 1
    heading_tol = 0.1; #radians
    dist_tolerance = 0.25 #meters
    
    turn_speed = 0.1 #rad/speed
    line_speed = 0.1 #m/s
    stop_speed = 0.0 
    
    if set_random == False:
        waypoints = [[6,6]]
    else:
        waypoints = generate_random_waypoints(n_random_waypoints, 15);
    
    while rclpy.ok():

        if is_done == True:
            print("I'm done")
            turtlebot_evader.move_turtle(stop_speed, stop_speed)
            rclpy.shutdown()

        for waypoint in waypoints:
            print("current waypoint is", waypoint)
            
            desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

            while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):

                # print("current heading is", m.degrees(turtlebot_evader.orientation_euler[2]))
                # print("desired heading is", m.degrees(desired_heading), heading_error)
        
                if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                    turtlebot_evader.move_turtle(line_speed, stop_speed)
                elif abs(dist_error) < dist_tolerance and  abs(heading_error) >= heading_tol:
                    turtlebot_evader.move_turtle(stop_speed, turn_speed)
                else:
                    turtlebot_evader.move_turtle(line_speed, turn_speed)
                
                desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)
                
                rclpy.spin_once(turtlebot_evader)
                                
        #/we're done looping through our lists
        is_done = True
                        

if __name__=="__main__":
    main()
