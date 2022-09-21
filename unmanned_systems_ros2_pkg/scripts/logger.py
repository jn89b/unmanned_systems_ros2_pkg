#!/usr/bin/env python
# -*- coding: utf-8 -*- 


from ipaddress import ip_address
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node

import time
import csv
import os
import datetime

from nav_msgs.msg import Odometry

from unmanned_systems_ros2_pkg import quaternion_tools


def get_time_in_secs(some_node:Node) -> float:
	return some_node.get_clock().now().nanoseconds /1E9
	

class OdomLocNode(Node):
	def __init__(self,sub_topic):
		super().__init__('logger_node')
		self.current_position = [None,None]
		self.orientation_euler = [None,None,None]
		self.odom_subscriber = self.create_subscription(
			Odometry, sub_topic, self.odom_callback, 10)
		 

	def odom_callback(self,msg):
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
		
		print("yaw is", np.degrees(self.orientation_euler[2]))

def main():
	FILEPATH = "/home/justin/ros2/src/unmanned_systems_ros2_pkg/unmanned_systems_ros2_pkg/log/"
	FILENAME = "dumpster_log.csv"

	print(os.getcwd())
	rclpy.init(args=None)
	odom_node = OdomLocNode("odom")

	#---------Logfile Setup-------------#
	# populate the data header, these are just strings, you can name them anything
	myData = ["time", "x", "y", "yaw", "vel_x", "yaw_vel"]

	# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
	# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
	# be the correct (it will not be the "real" time
	# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
	# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
	# to include an updated absolute path here to the new directory where you want the files to appear
	fileNameBase = FILEPATH + \
	datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + fileNameSuffix
		num = num + 1

	# now we know we have a unique name, let's open the file, 'a' is append mode, in the unlikely event that we open
	# a file that already exists, this will simply add on to the end of it (rather than destroy or overwrite data)
	myFile = open(fileName, 'a')
	with myFile:
		writer = csv.writer(myFile)
		writer.writerow(myData)

	# get the CPU time at which we started the node, we will use this to subtract off so that our time vector
	# starts near 0
	time_now = get_time_in_secs(odom_node)

	# this is some ros magic to control the loop timing, you can change this to log data faster/slower as needed
	# note that the IMU publisher publishes data at a specified rate (500Hz) and while this number could be
	# changes, in general, you should keep the loop rate for the logger below the loop rate for the IMU publisher
	# rate = rospy.Rate(20) #100 Hz
	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	while rclpy.ok():
			# get the current time and subtract off the zero_time offset
		now = (get_time_in_secs(odom_node)- time_now)
		# create the data vector which we will write to the file, remember if you change
		# something here, but don't change the header string, your column headers won't
		# match the data
		myData = [now, odom_node.current_position[0], odom_node.current_position[1],
			odom_node.orientation_euler[2]]

		# stick everything in the file
		myFile = open(fileName, 'a')
		with myFile:
			writer = csv.writer(myFile)
			writer.writerow(myData)

		rclpy.spin_once(odom_node)


if __name__ == '__main__':
	main()


