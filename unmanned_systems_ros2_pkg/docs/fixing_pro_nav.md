# Fixing Pro Nav
- Replace your ProNav.py with the new ProNav.py this is in unmanned_systems_ros_2_pkg  
- Replace the TurtleBotNode.py with the new TurtleBotNode.py
- Replace the pn.py with the new pn.py

# How was it fixed
- We have to control the frequency of the pn node so I have it running a specific rate  
- To check the rate of your lidar do the following command in termina
```
ros2 topic hz /pursuer/scan
```

This will give you the frequency of the command, use this value in the pn.py

