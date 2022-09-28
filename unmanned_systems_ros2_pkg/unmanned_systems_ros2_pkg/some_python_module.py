#!/usr/bin/env python3
from rclpy.node import Node

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9

def print_hello():
        print("Import function HELLO WORLD")
