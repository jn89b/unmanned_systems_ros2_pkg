#!/usr/bin/env python3
import rclpy
from unmanned_systems_ros2_pkg import TurtleBotNode
import numpy as np 
import math as m
"""
Proportional Navigation test script 
wrt means with respect to

Lidar sensor:
gives Range and Angle of detection from where turtlebot is at 

https://gamedev.stackexchange.com/questions/17313/how-does-one-prevent-homing-missiles-from-orbiting-their-targets

"""


#from unmanned_systems import util_functions

class PN():
    def __init__(self, dt, N=None):
        self.dt = dt 
        
        self.old_target_heading = None 
        self.old_target_position = None
        
        self.LOS = [0,0] #0 is the new value, 1 is the old valu
        self.pos = [0,0] # same here
        self.LOS_dot_old = 0 #just checking if we're at the same heading
        self.V_old = 0
        
        if N != None:
            self.N = N
        else:
            self.N = 0.1

    """dont need the this other stuff... using lidar"""    
    def update_vals(
        self, target_heading:float, target_position:float, LOS_dot:float,V:float) -> None:
        """update all my old values"""                
        self.LOS[1] = self.LOS[0]
        self.LOS[0] = target_heading  
        self.LOS_dot_old = LOS_dot
        
        self.pos[1] = self.pos[0]
        self.pos[0] = target_position
        self.V_old = V
        
    def check_same_target_heading(self) -> bool:
        if self.LOS[0] == self.LOS[1]:
            return True
        else:
            return False

    def check_same_target_distance(self) -> bool:
        if self.pos[0] == self.pos[1]:
            return True

    def get_turn_rate(self, target_heading:float, target_position:float) -> tuple:
        """very basic PN"""
        #for lidar it already gives you your relative position from detected targets
        # if (self.old_target_heading!=None) and (self.old_target_position!=None):
        LOS_dot = (self.LOS[0] - self.LOS[1])/self.dt        
        V = (self.pos[0] - self.pos[1])/ self.dt
 
        if self.check_same_target_heading() and abs(target_heading) >= 3.0:
            LOS_dot = self.LOS_dot_old
            self.update_vals(target_heading, target_position, LOS_dot, V)
            return (self.N * target_heading)/self.dt, V*self.N
        
        elif self.check_same_target_heading() == False and abs(target_heading) <= 3.0:
            print("case 2")
            self.update_vals(target_heading, target_position, LOS_dot, V)
            return self.N* LOS_dot,  V*self.N
        
        else:
            LOS_dot=0.0
            self.update_vals(target_heading, target_position, LOS_dot, V)
            return LOS_dot*self.N, V*self.N
            
        # self.update_vals(target_heading, target_position, LOS_dot, V)

        # return LOS_dot*self.N, V*self.N        
    
def compute_mean(some_list) -> float:  
    """calculates the mean of list"""
    return sum(some_list)/len(some_list)


def main() -> None:
    
    #initiate node 
    rclpy.init(args=None)
    
    #create some node
    pursuer_node = TurtleBotNode.TurtleBotNode('pursuer')
    dt = 1/5
    lambda_const = 0.5
    
    los = PN(dt, lambda_const)
    
    const_speed = 0.2
    
    #while your running node
    while rclpy.ok():
        
        if not pursuer_node.detected_heading_angle_list:
            rclpy.spin_once(pursuer_node)
            continue
        
        target_heading_mean = compute_mean(pursuer_node.detected_heading_angle_list)
        target_distance_mean  = compute_mean(pursuer_node.detected_range_list)
        turn_deg, vel_cmd = los.get_turn_rate(target_heading_mean, target_distance_mean)        
        
        turn_direction = target_heading_mean - 180
        
        if turn_direction >= 0:
            turn_cmd_rad = m.radians(-turn_deg)
        else:
            turn_cmd_rad = m.radians(turn_deg)
        
        pursuer_node.move_turtle(const_speed, turn_cmd_rad)

        rclpy.spin_once(pursuer_node)
    

if __name__ =='__main__':
    main()