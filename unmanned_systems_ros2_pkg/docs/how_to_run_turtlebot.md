# How to run Turtleobt

For reference check out this website https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
**Make sure to choose the correct ROS version Foxy or Humble**


## Start the Gazebo simulation
First off in one terminal start the turtlebot3 simulation a simulation should then pull up of the turtlebot in an empty world
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```


## Run the turtlebot simple node
After that run the turtlebot_simple node by running the following command
```
ros2 run unmanned_systems_ros2_pkg turtlebot_simple.py 
```

## Running your homework problems 
After you have compiled your_ros2_node in the CMakeLists.txst
Just do the following command 
```
ros2 run unmanned_systems_ros2_pkg your_ros2_node.py
```

