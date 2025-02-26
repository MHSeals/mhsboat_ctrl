from geometry_msgs.msg import TwistStamped

class SimulatedController():
    def __init__(self) -> None:
        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0

    def set_forward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.x = velocity

    def set_backward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.y = velocity

    def set_angular_velocity(self, velocity: float):
        self.cmd_vel.twist.angular.z = velocity