import numpy as np
from time import time
from typing import Tuple

class PIDController:
    """
    PID controllers have three components:
    - proportional
    - integral
    - and derivative components
    
    Kp, Ki, and Kd gains respectively where e denotes error and t, time
    
    The following logic is used to compute the output value u(t), the motor outputs
    
    Proportional: Kp * e(t)
    Integral: Ki * ₀∫ᵗ e(𝜏)d𝜏
    Derivative: Kd * d(e(t))/dt
    
    All of these components are added together to determine the output
    
    we need to store the following
    - time
    - integral (adds over time)
    - time_prev
    - e_prev
    
    In our case, our system has tto handle these
    
    INPUTS
    - Current position (start at (0,0) when we want to start going forward then use odometry)
    - Desired path represented with a linear function (only need slope since started at origin)
    - Heading error (meaning that we have to calcuate heading by using our purse pursuit algorithm)
    - Look ahead distance
    
    OUTPUTS
    - Forward velocity
    - Angular velocity

    """

    def __init__(self, look_ahead: float, Kp: float, Ki: float, Kd: float, integral_bound: float):
        self.look_ahead = look_ahead
        self.integral_bound = integral_bound

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.integral = 0
        self.prev_t = time()

    def pure_pursuit(self, angle: float, position: float, orientation: float) -> Tuple[Tuple[float, float], float]:
        """ 
        Simplified pure pusuit algorithm using only a slope to represent path due to starting at the origin and linear nature
        Outputs the heading error by calculating the angle between the line from the robot to the desired goal and the robot's forward heading
        """
        slope = np.tan(angle)
        if slope == 0:
            y = position[1] 
            x = y / slope
        else:
            perp_slope = -1 / slope

            x = (-perp_slope * position[0] + position[1]) / (slope - perp_slope)
            y = x * slope

        dx =  self.look_ahead / np.sqrt(1 + slope ** 2)
        dy = self.look_ahead * slope / np.sqrt(1 + slope ** 2)

        goal_x = x + dx
        goal_y = y + dy

        angle_to_goal = np.arctan2(goal_y - position[1], goal_x - position[0])
        error = self.compute(angle_to_goal - orientation)
        return (goal_x, goal_y), error
        
    def compute(self, error: float) -> float:
        self.t = time()
        self.dt = self.t - self.prev_t
        self.prev_t = self.t

        error = (error + np.pi) % (2 * np.pi) - np.pi  

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_bound, self.integral_bound)  
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        angle_out = (self.Kp * error) + (self.Kd * derivative) + (self.Ki * self.integral)
        return angle_out