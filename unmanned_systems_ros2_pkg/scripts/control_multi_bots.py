#!/usr/bin/env python3

#import modules here
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import some_python_module
"""
To do:
    - Make first turtlebot turn for 0 to 5 seconds
    - From 5 to 10 seconds make turlebot turn


Template to set up script:
    -[x] Write a to do list 
    -[x] Initiliaze some "main function" 
    -[]Initiate ros node
    -[] Create ros node
    -[] Implement your stuff 

"""
class TurtlebotNode(Node):
    def __init__(self,node_name=''):
        if node_name != '':
            self.node_name = node_name
        else:
            node_name = "hello"
            self.node_name = ''
        
        super().__init__(node_name)
        
        self.turtlebot_pub = self.create_publisher(Twist, self.node_name+'/cmd_vel', 5)
        # self.turtlebot1_pub = self.create_publisher(Twist, 'turtlebot/cmd_vel', 5)
        # self.turtlebot1_pub = self.create_publisher(Twist, 'turtle/cmd_vel', 5)
        # self.turtlebot1_pub = self.create_publisher(Twist, 'turtle/cmd_vel', 5) 

    def move_turtlebot(self, linear_vel, ang_vel):
        """make turtlebot cmd vel message and then publish"""
        some_twist = Twist()
        some_twist.linear.x = linear_vel
        some_twist.angular.z = ang_vel #rad/s
        self.turtlebot_pub.publish(some_twist)
        # vel_pub.publish(some_twist)
         
def main():
    """this is where our main subroutine will run"""
    rclpy.init(args=None)
    
    turtlebot_node = TurtlebotNode()
    turtlebot_1_node = TurtlebotNode('turtle')
    
    first_tb_time = 3
    second_tb_time = 6
    
    ang_vel = 0.25
    stop_ang_vel = 0.0
    stop_vel = 0.0
    
    zero_time = some_python_module.get_time_in_secs(turtlebot_node)
    
    ## while loop runs 
    while rclpy.ok():
        time_now = some_python_module.get_time_in_secs(turtlebot_node)
        time_diff = time_now - zero_time
        
        #between first time inveraval make first tb move
        if (time_diff <= first_tb_time):
            turtlebot_node.move_turtlebot(stop_vel, ang_vel)
            
        
        #during second time interval first tb will stop and second will turn
        elif(time_diff >= first_tb_time and time_diff <= second_tb_time):
            turtlebot_node.move_turtlebot(stop_vel, stop_vel)
            turtlebot_1_node.move_turtlebot(stop_vel, ang_vel)

        #make sure all tb stop
        else:
            turtlebot_1_node.move_turtlebot(stop_vel, stop_vel)
            rclpy.shutdown()

        # rclpy.spin_once(turtlebot_node)

if __name__ == '__main__':
    main()