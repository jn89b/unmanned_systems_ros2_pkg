# Fixing Pro Nav
- Replace your ProNav.py with the new ProNav.py 
- Replace the TurtleBotNode.py with the new TurtleBotNode.py
- Replace the pn.py with the new pn.py

# How was it fixed
- The biggest thing is the current velocity of the pursuer was in body frame not in world frame 
- Also I found out a good dt to use is a tenth of the odometry, not sure why but it just works. 
- 