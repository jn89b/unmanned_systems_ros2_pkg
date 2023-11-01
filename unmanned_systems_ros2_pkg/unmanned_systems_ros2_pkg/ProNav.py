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
        los_dot = (new_heading_rad - self.previous_heading_rad)/dt
        
        # #wrap heading between 0 and 2pi
        # if los_dot > 2*np.pi:
        #     los_dot = los_dot - 2*np.pi
        # elif los_dot < 0:
        #     los_dot = los_dot + 2*np.pi
            
        # if new_heading_rad > 2*np.pi:
        #     new_heading_rad = new_heading_rad - 2*np.pi
        # elif new_heading_rad < 0:
        #     new_heading_rad = new_heading_rad + 2*np.pi
            
        # #wrap between -pi and pi
        # if los_dot > np.pi:
        #     los_dot = los_dot - np.pi
        # elif los_dot < -np.pi:
        #     los_dot = los_dot + np.pi
            
        # if new_heading_rad > np.pi:
        #     new_heading_rad = new_heading_rad - np.pi
        # elif new_heading_rad < -np.pi:
        #     new_heading_rad = new_heading_rad + np.pi
            
        self.previous_heading_rad = new_heading_rad
        flight_path_rate = self.nav_gain*(los_dot)
        
        return flight_path_rate

    def true_pro_nav(self, current_position:np.ndarray, 
                     target_position:np.ndarray, dt:float) -> [float,float]:
        """
        Returns 
        """
        current_position = np.array(current_position)
        target_position = np.array(target_position)
        
        current_to_target_new = target_position - current_position
        
        if np.linalg.norm(current_to_target_new) == 0:
            return 0.0, 0.0
        else:
            delta_los = np.linalg.norm(current_to_target_new) - \
                np.linalg.norm(self.current_to_target_old)
                   
            los_dot = np.linalg.norm(delta_los)
            
        self.current_to_target_old = current_to_target_new
        vel_cmd = -los_dot * 100
        
        a_n = vel_cmd * self.nav_gain * los_dot 
        
        print("an", a_n)

        return -a_n, vel_cmd

    def augmented_pro_nav(self, new_heading_rad:float, dt:float) -> [float,float]:
        """
        Returns augmented proportional navigation command 
        """
        pass


    