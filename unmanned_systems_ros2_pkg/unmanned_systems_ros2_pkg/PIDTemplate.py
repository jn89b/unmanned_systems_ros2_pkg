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

class PID():
    def __init__(self, kp, ki, kd, dt) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = [0,0]
        
    def compute_error(self, des, actual):
        self.error[0] = des-actual 
        
    def get_gains(self, des, actual):
        #p gains
        self.compute_error(des, actual)
        p = self.kp * self.error[0] 
        d = self.kd * (self.error[0] - self.error[1])/self.dt
        #update error
        self.error[1] = self.error[0]
        
        return p+d
        