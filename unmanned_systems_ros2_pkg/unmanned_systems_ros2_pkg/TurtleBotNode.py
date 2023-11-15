#!/usr/bin/env python3
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unmanned_systems_ros2_pkg import quaternion_tools, PIDTemplate

class TurtleBotNode(Node):
    def __init__(self, node_name:str, ns='' , controller = None ):
        super().__init__(node_name)
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
            
        if controller != None:
            
            self.pid = PIDTemplate(kp=controller[0], 
                                   ki=controller[1],
                                   kd=controller[2],
                                   dt=controller[3]) 
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  1) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 1)
        
        self.evader_subscriber = self.create_subscription(
            Odometry, "/evader/odom", self.evader_callback, 1)
        
        self.lidar_subscriber = self.create_subscription(
             LaserScan, self.ns+"/scan", self.lidar_track_cb, 1)
        
        self.current_position = [0,0]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw
        
        self.current_velocity = [0.0, 0.0]
        
        self.evader_position = [0,0]
        self.evader_velocity = [0,0]
        
        self.detected_range_list = [] #depth detected
        self.detected_heading_angle_list = [] #heading detected


    def evader_callback(self, msg:Odometry) -> None:
        """subscribe to odometry"""
        self.evader_position[0] = msg.pose.pose.position.x
        self.evader_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w        
        roll,pitch,yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)

        # get x and y velocity
        evader_velocity_mag = msg.twist.twist.linear.x
        self.evader_velocity[0] = evader_velocity_mag*np.cos(yaw)
        self.evader_velocity[1] = evader_velocity_mag*np.sin(yaw)
        
        
    def odom_callback(self,msg:Odometry):
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
        current_vel_mag = msg.twist.twist.linear.x
        self.current_velocity[0] = current_vel_mag*np.cos(yaw)
        self.current_velocity[1] = current_vel_mag*np.sin(yaw)
        
        
    def move_turtle(self, linear_vel:float, angular_vel:float):
        """Moves turtlebot"""
        twist = Twist()
        
        if linear_vel >= 0.23:
            linear_vel = 0.23
        elif linear_vel <= -0.23:
            linear_vel = -0.23
        
        if angular_vel >= 2.84:
            angular_vel = 2.84
        elif angular_vel <= -2.84: 
            angular_vel = -2.84    
    
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
    def lidar_track_cb(self, msg:LaserScan):
        """lidar information remember the msg is an array of 0-> 359"""
        self.detected_range_list = []
        self.detected_heading_angle_list = []
        inf = float('inf')
        
        lidar_vals = msg.ranges
        
        #append detections if values not infinity or 0.0 
        for i, val in enumerate(lidar_vals):
            if val != inf:
                self.detected_heading_angle_list.append(i)
                self.detected_range_list.append(val)
                

