# The Final Project World File

In the **unmanned_systems_ros2/worlds** copy the **entire turtlebot_final_project** folder and place it into  your **turtlebot3_simulations/turtlebot_gazebo/worlds** directory

Once you're done go the top level directory and do the following command:
```
colcon build --symlink-install
```

Check if the world and launch file works by entering the following command
```
ros2 launch unmanned_systems_ros2_pkg final_project_world.py 
```

# Test out NAV2 
In the **slam_map** directory copy paste the final_map.yaml file into your home directory and do the following command:
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/final_map.yaml
``` 
Test out it and see how it works!


# Make foot ninjas move
To make the foot ninjas move in a terminal do the following command 
```
ros2 launch unmanned_systems_ros2_pkg foot_ninja_move.launch.py
```

# Dungeon Master Node
The dungeon master node will detect if any of the foot ninjas have detected your turtlebot on your traversal, to run the node do the following command

```
ros2 run unmanned_systems_ros2_pkg dungeon_master.py 
```


