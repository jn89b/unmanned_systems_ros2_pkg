# Install Turtlebot

For reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

In your terminal cd to your ros2 workspace and go to the src folder 
```
cd ros2_ws/src

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git 

```

Then cd back to your top level directory of your ros2 workspace 
```
cd ../ 

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install 
```

