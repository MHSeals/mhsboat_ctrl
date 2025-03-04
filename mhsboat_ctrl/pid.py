from mhsboat_ctrl.mhsboat_ctrl import BoatController
import numpy as np

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

    def __init__(self, boat_controller: BoatController, slope: float, look_ahead: float, Kp: float, Ki: float, Kd: float):
        self.boat_controller = boat_controller
        self.slope = slope
        self.look_ahead = look_ahead

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.integral = 0
        self.prev_time = self.get_time()

    def pure_pursuit(self, position, orientation) -> float:
        """ 
        Simplified pure pusuit algorithm using only a slope to represent path due to starting at the origin and linear nature
        Outputs the heading error by calculating the angle between the line from the robot to the desired goal and the robot's forward heading
        """
        
        heading_slope = np.tan(orientation) 
        
        if np.isinf(heading_slope):
            y = position[1]
            x = y / self.slope
        else:
            heading_perpendicular_slope = -1 / heading_slope
            x = (-heading_perpendicular_slope * position[0] + position[1]) / (self.slope - heading_perpendicular_slope)
            y = self.slope * x

        dx =  self.look_ahead / np.sqrt(1 + self.slope ** 2)
        dy = self.look_ahead * self.slope / np.sqrt(1 + self.slope ** 2)

        goal_x = x + dx
        goal_y = y + dy
        
        goal_slope = (goal_y - position[1]) / (goal_x - position[0]) 
        return self.compute(np.atan2(goal_slope) - np.atan2(heading_slope))
        
    def compute(self, error: float) -> float:
        self.time = self.get_time()
        self.dt = self.time - self.prev_time
        self.prev_time = self.time

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        angle_out = (self.Kp * error) + (self.Kd * derivative) + (self.Ki * self.integral)
        return angle_out

    def get_time(self) -> float:
        self.boat_controller.get_clock().now().nanoseconds / 1e9