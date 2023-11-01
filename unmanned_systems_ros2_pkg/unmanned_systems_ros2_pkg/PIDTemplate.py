"""
class PID
attributes:
    kp, 
    ki, 
    kd 

methods?
compute_error(des, actual) -> return error 
get_gains() -> return command


"""
import numpy as np

class PID():
    def __init__(self, kp, ki, kd, dt) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = [0,0]

    def say_whaddup(self):
        print("whats up man")

    def compute_error(self, des:float, 
                      actual:float,
                      angle_wrap:bool=False) -> float:
        
        # if actual <= 0:
        #     self.error[0] = des+actual
        # else:
        self.error[0] = des-actual
        
        return self.error[0] 
        
    def get_gains(self, des, actual):
        #p gains
        self.compute_error(des, actual)
        p = self.kp * self.error[0] 
        d = self.kd * (self.error[0] - self.error[1])/self.dt
        #update error
        self.error[1] = self.error[0]
        
        return p+d
        