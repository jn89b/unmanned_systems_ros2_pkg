import numpy as np
import matplotlib.pyplot as plt

class ProNav():
    def __init__(self, nav_gain:float)->None:
        self.nav_gain = nav_gain
        
        # Previous Position
        self.old_distance = 0.0
        self.previous_heading_rad = 0.0
        self.current_to_target_old = np.array([0.0,0.0])
        
    def simple_pro_nav(self, 
                       new_heading_rad:float, 
                       dt:float) -> float:
        """
        Returns rate of flight path change in radians per second
        """
        print("new heading deg", np.rad2deg(new_heading_rad))
        print("old heading deg", np.rad2deg(self.previous_heading_rad))
        los_dot = (new_heading_rad - self.previous_heading_rad)/dt
            
        self.previous_heading_rad = new_heading_rad
        flight_path_rate = self.nav_gain*(los_dot)
        
        return flight_path_rate

    def true_pro_nav(self, current_position:np.ndarray, 
                     target_position:np.ndarray, dt:float, 
                     target_vel:np.ndarray, pursuer_vel:np.ndarray,
                     use_own_heading:bool=False,
                     los_value:float=0.0) -> [float,float]:
        """
        Current position and target position are 2D numpy arrays [x,y]
        Target velocity is a 2D numpy array [vx,vy]
        Pursuer velocity is a 2D numpy array [vx,vy]
        """
        current_position = np.array(current_position)
        target_position = np.array(target_position)
        
        #relative position
        r_to_target = target_position - current_position
        r_dist_to_target = np.linalg.norm(r_to_target)
        
        #line of sight
        if use_own_heading == True:
            los = los_value
        else:
            los = np.arctan2(r_to_target[1], r_to_target[0])
                
        los_dot = (los - self.previous_heading_rad)/dt

        #relative velocity
        v_rel = target_vel - pursuer_vel
        v_rel_mag = np.linalg.norm(v_rel)
        
        #closing velocity
        v_closing = (v_rel_mag)*r_to_target/r_dist_to_target        
        
        # v_closing = r_to_target/r_dist_to_target
        v_closing = np.linalg.norm(v_closing)
        
        #acceleration command 
        acmd = self.nav_gain*v_closing
        #we want to find the command velocity from this acceleration, 
        #since we're not using the acceleration directly
        v_x = -pursuer_vel[0]*np.cos(los) + acmd*np.cos(los)
        v_y =  pursuer_vel[1]*np.sin(los) + acmd*np.sin(los)

        v_cmd = np.array([v_x, v_y])
        v_cmd = np.linalg.norm(v_cmd)
        
        #get flight path rate from acceleration command            
        # not technically correct but its close enough
        flight_path_rate = self.nav_gain*(los_dot)
        self.previous_heading_rad = los
            
        return flight_path_rate, v_cmd
        

    def augmented_pro_nav(self, current_position:np.ndarray, 
                     target_position:np.ndarray, dt:float, 
                     target_vel:np.ndarray, pursuer_vel:np.ndarray,
                     use_own_heading:bool=False,
                     los_value:float=0.0) -> [float,float]:
        """ 
        Current position and target position are 2D numpy arrays [x,y]
        Target velocity is a 2D numpy array [vx,vy]
        Pursuer velocity is a 2D numpy array [vx,vy]
        """
        current_position = np.array(current_position)
        target_position = np.array(target_position)
        
        #relative position
        r_to_target = target_position - current_position
        r_dist_to_target = np.linalg.norm(r_to_target)
        
        #line of sight
        if use_own_heading == True:
            los = los_value
        else:
            los = np.arctan2(r_to_target[1], r_to_target[0])
        
        los_dot = (los - self.previous_heading_rad)/dt

        #relative velocity
        v_rel = target_vel - pursuer_vel
        v_rel_mag = np.linalg.norm(v_rel)
        
        #closing velocity
        v_closing = (v_rel_mag)*r_to_target/r_dist_to_target        
        v_closing = np.linalg.norm(v_closing)
        #acceleration command with augmented term
        #the augmented term is the worst case scenario 
        #the "smart" thing for the pursuer to do is move perpendicular 
        #to the line of sight so we account for that 
        norm_tangent = np.array([-np.sin(los), np.cos(los)])
        
        acmd = self.nav_gain*v_closing+ \
            ((self.nav_gain + norm_tangent) / 2)
        
        #we want to find the command velocity from this acceleration, 
        #since we're not using the acceleration directly
        v_x = -pursuer_vel*np.cos(los) + acmd*np.cos(los)
        v_y = pursuer_vel*np.sin(los) + acmd*np.sin(los)
        v_cmd = np.array([v_x, v_y])
        v_cmd = np.linalg.norm(v_cmd)
        
        # not technically correct but its close enough
        flight_path_rate = self.nav_gain*(los_dot) 
        self.previous_heading_rad = los
        
        return flight_path_rate, v_cmd

    