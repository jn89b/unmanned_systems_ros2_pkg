#!/usr/bin/env python3
"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
import xml.etree.ElementTree as ET


from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")
    
    node.declare_parameters(
    namespace='',
    parameters=[
        ('gazebo_name', 'Leonardo'),
        ('x_pos', 4.0),
        ('y_pos', 0.0),
        ('z_pos', 0.0),
        ('yaw_pos', 0.0)
    ]
    )
    
    gazebo_name_param = node.get_parameter('gazebo_name').value
    x_pos_param = node.get_parameter('x_pos').value
    y_pos_param = node.get_parameter('y_pos').value
    z_pos_param = node.get_parameter('z_pos').value
    yaw_pos_param = node.get_parameter('yaw_pos').value
    
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_"+TURTLEBOT3_MODEL, "model.sdf")

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the urdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    
    tree = ET.parse(sdf_file_path)
    print("opening tree")
    root = tree.getroot()
    imu_plugin = None
    diff_drive_plugin = None 
    for plugin in root.iter('plugin'):
        if 'turtlebot3_diff_drive' in plugin.attrib.values():
            diff_drive_plugin = plugin
        elif 'box_bot_imu_plugin' in plugin.attrib.values():
            imu_plugin = plugin

    print("diff drive", diff_drive_plugin)

    # We change the namespace to the robots corresponding one
    tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    tag_diff_drive_ns.text = '/' + gazebo_name_param

    ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + gazebo_name_param + '/tf'

    tag_diff_drive_ros_params = diff_drive_plugin.find('odometry_topic')
    print(tag_diff_drive_ros_params)
    ros_odom_remap = ET.SubElement(diff_drive_plugin, 'odometry_topic')
    ros_odom_remap.text = '/' + gazebo_name_param + '/odom'

    # tag_diff_drive_ros_params = diff_drive_plugin.find('odometry_frame')
    # ros_odom_remap_tf = ET.SubElement(diff_drive_plugin, 'odometry_frame')
    # ros_odom_remap_tf.text = '/' + gazebo_name_param + '/odom'

    # Set data for request
    request = SpawnEntity.Request()
    request.name = gazebo_name_param
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = gazebo_name_param
    request.initial_pose.position.x = float(x_pos_param)
    request.initial_pose.position.y = float(y_pos_param)
    request.initial_pose.position.z = float(z_pos_param)

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    