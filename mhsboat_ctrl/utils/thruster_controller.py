import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from mhsboat_ctrl.gui import GUI

class ThrusterController(Node):
    def __init__(self):
        super().__init__("boat_controller")
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.cmd_vel = TwistStamped()

    def set_forward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.x = velocity

    def set_backward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.y = velocity

    def set_angular_velocity(self, velocity: float):
        self.cmd_vel.twist.angular.z = velocity

class SimulatedController(Node):
    def __init__(self):
        super().__init__("boat_controller")
        self.cmd_vel = TwistStamped()
        self.gui = GUI(self)

    def set_forward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.x = velocity

    def set_backward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.y = velocity

    def set_angular_velocity(self, velocity: float):
        self.cmd_vel.twist.angular.z = velocity