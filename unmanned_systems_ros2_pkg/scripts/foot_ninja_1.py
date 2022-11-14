#!/usr/bin/env python3
import rclpy
import math as m

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools


def compute_desired_heading(current_pos:list, des_pos:list):
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
    
    foot_ninja_1 = TurtleBotNode.TurtleBotNode('foot_ninja_1_patrol', 'foot_ninja_1')    
    foot_ninja_1.move_turtle(0.0,0.0)


    heading_tol = 0.1; #radians
    dist_tolerance = 0.1 #meters
    turn_speed = 1.0 #rad/speed
    line_speed = 0.1 #m/s
    stop_speed = 0.0 #m/s

    patrol_distance = 0.25 #
    origin_position = [-1.25, -3.0]

    waypoints = [[origin_position[0],origin_position[1] + patrol_distance],
                [origin_position[0],origin_position[1]-patrol_distance]]


    while rclpy.ok():
        
        for waypoint in waypoints:
            print("going to waypoint", waypoint)
            desired_heading, heading_error, dist_error = gimme_da_loot(foot_ninja_1, waypoint)

            while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):
                # print("desired heading is", m.degrees(desired_heading), heading_error)
                print(foot_ninja_1.current_position)

                if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                    foot_ninja_1.move_turtle(line_speed, stop_speed)
                elif abs(dist_error) < dist_tolerance and  abs(heading_error) >= heading_tol:
                    foot_ninja_1.move_turtle(stop_speed, turn_speed)
                else:
                    foot_ninja_1.move_turtle(line_speed, turn_speed)
                
                desired_heading, heading_error, dist_error = gimme_da_loot(foot_ninja_1, waypoint)
                
                rclpy.spin_once(foot_ninja_1)

    


    foot_ninja_1.move_turtle(0.0,0.0)

if __name__=="__main__":
    main()