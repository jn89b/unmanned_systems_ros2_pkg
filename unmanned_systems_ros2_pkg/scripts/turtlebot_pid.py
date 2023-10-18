#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np

from rclpy.node import Node
from rclpy.duration  import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import PIDTemplate

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 
    

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw

        #Wrap yaw  from to 0 to 2pi
        if self.orientation_euler[2] < 0:
            self.orientation_euler[2] += 2*np.pi
        else:
            self.orientation_euler[2] = self.orientation_euler[2]
        
        print("yaw is", np.degrees(self.orientation_euler[2]))
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel 
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main()->None:
    rclpy.init(args=None)
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    
    cmd_vel = 1.0 #m/s
    ang_vel = 0.5 #rad/s
    stop_vel = 0.0
    time_duration = 5
    
    # time intilization ref 
    time_origin = get_time_in_secs(turtlebot_node)
    print("time now is", time_origin)

    kp_angular = 10
    ki_angular = 0.0
    kd_angular  = 0.0
    dt_angular = 1/20

    pid_angular = PIDTemplate.PID(
        kp = kp_angular,
        ki = ki_angular,
        kd = kd_angular,
        dt = dt_angular)

    # set a desired heading angle 
    des_yaw_angle_rad = np.deg2rad(225)

    MAX_ANG_SPEED_RAD = 2.84 #rad/s

    GOT_INITIAL_HEADING = False

    # try:
    try: 
        while rclpy.ok():

            if GOT_INITIAL_HEADING == False:
                initial_yaw_rad = turtlebot_node.orientation_euler[2]
                GOT_INITIAL_HEADING = True

            des_yaw_angle_rad = initial_yaw_rad + des_yaw_angle_rad      

            #COPY PASTE
            angular_gains = pid_angular.get_gains(
                des_yaw_angle_rad,
                turtlebot_node.orientation_euler[2]
            )

            print("my heading error is", 
                np.rad2deg(pid_angular.error[0]))


            if angular_gains >= MAX_ANG_SPEED_RAD:
                angular_gains = MAX_ANG_SPEED_RAD
            elif angular_gains <= -MAX_ANG_SPEED_RAD:
                angular_gains = -MAX_ANG_SPEED_RAD

            # COPY PASTE END HERE

            # add an error tolerance
            # if you're close to error:
                # angular_gains = 0



            print("my gains are", angular_gains)    

            turtlebot_node.move_turtle(0.0, angular_gains)


            rclpy.spin_once(turtlebot_node)

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)

    

if __name__ == '__main__':
    """apply imported function"""
    main()