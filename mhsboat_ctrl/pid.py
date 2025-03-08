from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mhsboat_ctrl.mhsboat_ctrl import VisionBoatController

import numpy as np
from typing import Tuple

class PIDController:
    def __init__(self, boat_controller: 'VisionBoatController', Kp: float, Ki: float, Kd: float, integral_bound: float):
        self.integral_bound = integral_bound
        self.boat_controller = boat_controller

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.integral = 0
        self.prev_t = self.get_time()
        
    def compute(self, error: float) -> float:
        self.t = self.get_time()
        self.dt = self.t - self.prev_t
        self.prev_t = self.t

        error = (error + np.pi) % (2 * np.pi) - np.pi  

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_bound, self.integral_bound)  
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        angle_out = (self.Kp * error) + (self.Kd * derivative) + (self.Ki * self.integral)
        return angle_out
    
    def get_time(self) -> float:
        return self.boat_controller.get_clock().now().nanoseconds / 1e9