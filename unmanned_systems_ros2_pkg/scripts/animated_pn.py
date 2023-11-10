#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from unmanned_systems_ros2_pkg import ProNav

"""
Simple simulation with two agents: pursuer and evader
Coordinate frame is cartesian with origin at bottom left 
"""

class Agent():

    def __init__(self, init_position:np.ndarray, 
                 init_heading:float, max_ang_vel_deg:float=500,
                 max_lin_vel:float=10.0) -> None:
        
        self.position  = init_position
        self.heading = init_heading
        
        self.max_ang_vel_rad = np.rad2deg(max_ang_vel_deg)
        self.max_lin_vel = max_lin_vel
        
        self.lin_vel_cmd = 0.0
        self.ang_vel_cmd = 0.0
        
        self.position_history = []
        self.heading_history =  []
    
    def move(self, ang_vel:float, lin_vel:float, 
             dt:float) -> None:
        """
        Move agent based on angular and linear velocity
        """
        
        lin_vel = lin_vel + self.ang_vel_cmd
        # ang_vel = ang_vel + self.lin_vel_cmd
        
        if ang_vel > self.max_ang_vel_rad:
            ang_vel = self.max_ang_vel_rad
        elif ang_vel < -self.max_ang_vel_rad:
            ang_vel = -self.max_ang_vel_rad
        
        if lin_vel > self.max_lin_vel:
            lin_vel = self.max_lin_vel
        elif lin_vel < -self.max_lin_vel:
            lin_vel = -self.max_lin_vel
                    
        new_heading = (ang_vel*dt) + self.heading
    
        if new_heading > np.pi:
            new_heading = new_heading - np.pi
        elif new_heading < -np.pi:
            new_heading = new_heading + np.pi
        
        new_x = (lin_vel*np.cos(self.heading)*dt) + \
            self.position[0]
        new_y = (lin_vel*np.sin(self.heading)*dt) + \
            self.position[1]
            
        new_position = np.array([new_x, new_y])
    
        self.position_history.append(new_position)
        self.heading_history.append(new_heading)
        
        self.position = new_position
        self.heading = new_heading
        
            
def compute_heading(current_pos:np.ndarray, 
                    des_pos:np.ndarray) -> float:
    """compute desired heading based on positions"""
    return np.arctan2(des_pos[1] - current_pos[1] , 
                      des_pos[0] - current_pos[0])
        

def check_close(current_pos:np.ndarray, 
                des_pos:np.ndarray, 
                dist_tolerance:float=0.5) -> bool:
    """check if agent is close to waypoint"""
    des_pos = np.array(des_pos)
    current_pos = np.array(current_pos)
    dist_error = np.linalg.norm(des_pos-current_pos)
    if dist_error < dist_tolerance:
        return True
    else:
        return False
    
def compute_distance(current_pos:np.ndarray, 
                     des_pos:np.ndarray) -> float:
    """compute distance between two points"""
    des_pos = np.array(des_pos)
    current_pos = np.array(current_pos)
    return np.linalg.norm(current_pos-des_pos)

if __name__=="__main__":
    
    
    ### INTIALIZE AGENTS ###
    pursuer_current_position = [0,0]
    pursuer_heading = np.deg2rad(90)
    pursuer = Agent(pursuer_current_position, pursuer_heading)
    
    evader_current_position  = [2,2]
    evader_heading = np.deg2rad(45)
    evader = Agent(evader_current_position, evader_heading)

    #history         
    pursuer_history = []
    evader_history = []

    #### PRO NAVOPTIONS ####
    pro_nav_options = ["Simple ProNav", 
                       "True ProNav", 
                       "Augmented ProNav"]
    guidance_option = pro_nav_options[2]
    nav_constant = 5.0
    pro_nav = ProNav.ProNav(nav_constant)

    #### SIM PARAMETERS ####
    dt = 0.01
    N = 1000 #number of iterations
    
    #### SET VELOCITIES ####
    evader_vel = 0.8
    pursuer_vel = evader_vel*1.3
    pursuer.lin_vel_cmd = pursuer_vel

    #### SET SEED ####
    # this is used to make the simulation repeatable
    set_seed = True
    if set_seed == True:
        np.random.seed(1)
    
    RANDOM_VEL = True
    RANDOM_ANGLE = True
    
    for i in range(N):    
        #compute current los
        desired_heading_rad = compute_heading(
            pursuer.position, evader.position)
        
        current_distance = compute_distance(
            pursuer.position, evader.position)
        
        # use simple pro nav
        if guidance_option == pro_nav_options[0]:
            flight_path_rate = pro_nav.simple_pro_nav(
                desired_heading_rad, dt)

            pursuer.move(flight_path_rate, pursuer_vel, dt)

        # use true pro nav
        if guidance_option == pro_nav_options[1]:
            target_vel = np.array([
                evader.lin_vel_cmd*np.cos(desired_heading_rad),
                evader.lin_vel_cmd*np.sin(desired_heading_rad)])

            pursuer_vel = np.array([
                pursuer.lin_vel_cmd*np.cos(pursuer.heading),
                pursuer.lin_vel_cmd*np.sin(pursuer.heading)])

            flight_path_rate, vel_cmd = pro_nav.true_pro_nav(
                pursuer.position, evader.position, dt,
                target_vel, pursuer_vel)
            
            pursuer.move(flight_path_rate, vel_cmd, dt)
        
        if guidance_option == pro_nav_options[2]:
            target_vel = np.array([
                evader.lin_vel_cmd*np.cos(desired_heading_rad),
                evader.lin_vel_cmd*np.sin(desired_heading_rad)])

            pursuer_vel = np.array([
                pursuer.lin_vel_cmd*np.cos(pursuer.heading),
                pursuer.lin_vel_cmd*np.sin(pursuer.heading)])

            flight_path_rate, vel_cmd = pro_nav.augmented_pro_nav(
                pursuer.position, evader.position, dt,
                target_vel, pursuer_vel)
            
            pursuer.move(flight_path_rate, vel_cmd, dt)
                 
        ## move the evader
        if RANDOM_VEL == True and RANDOM_ANGLE == False:   
            random_vel = np.random.uniform(evader_vel/2, evader_vel)
            evader.move(0, random_vel, dt)
        
        elif RANDOM_VEL == False and RANDOM_ANGLE == True:
            random_ang_vel = np.random.uniform(-np.pi/2, np.pi/2)
            evader.move(random_ang_vel, evader_vel, dt)
        
        elif RANDOM_VEL == True and RANDOM_ANGLE == True:
            random_vel = np.random.uniform(evader_vel/2, evader_vel)
            random_ang_vel = np.random.uniform(-np.pi/2, np.pi/2)
            evader.move(random_ang_vel, random_vel, dt)
        
        else:
            evader.move(0, evader_vel, dt)
        
        # pursuer_history.append(pursuer.position)
        if check_close(pursuer.position, evader.position) == True:
            print("Pursuer caught evader")
            break
    
    pursuer_x = [x[0] for x in pursuer.position_history]
    pursuer_y = [x[1] for x in pursuer.position_history]
    pursuer_heading_dg = [np.rad2deg(x) for x in pursuer.heading_history]

    evader_x =  [x[0] for x in evader.position_history]
    evader_y =  [x[1] for x in evader.position_history]
    evader_heading_dg = [np.rad2deg(x) for x in evader.heading_history]

    #plot
    fig, ax = plt.subplots()
    ax.plot(evader_x, evader_y,  '-o', label='evader')
    ax.plot(pursuer_x, pursuer_y,'-x', label='pursuer')
    ax.set_title(guidance_option + " Took " + str(i) + " iterations")
    ax.legend()


    #plot heading
    fig1, ax1 = plt.subplots(2,1)
    ax1[0].plot(pursuer_heading_dg, label='pursuer heading')
    
    ax1[0].legend()
    plt.show()

    
    
    
    
    
    




