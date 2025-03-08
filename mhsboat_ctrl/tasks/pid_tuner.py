import numpy as np
from typing import Tuple, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus

FORWARD_VELOCITY = 10.0  # m/s
ANGULAR_VELOCITY = 4.0  # rad/s
END_DISTANCE = 10 # m
ANGLE = 0

class PIDTuner(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid

    def search(self) -> Optional[Tuple[float, float]]:
        self.x = 0.0
        self.y = 0.0
        self.zr = 0.0

        return (0.0, 0.0)

    def run(self) -> TaskCompletionStatus:
        self.boat_controller.get_logger().info("Running PID Tuner")

        completion_status = TaskCompletionStatus.NOT_STARTED

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            and not completion_status == TaskCompletionStatus.FAILURE
        ):
            self.x += self.boat_controller.dx
            self.y += self.boat_controller.dy
            self.zr += self.boat_controller.dzr

            angular_velocity = self.boat_controller.pid.pure_pursuit(ANGLE, (self.x, self.y), self.zr)
            angular_velocity *= ANGULAR_VELOCITY * np.pi / 180
            
            self.boat_controller.set_angular_velocity(angular_velocity)
            self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)
            
            if (self.x ** 2 + self.y ** 2) ** (1 / 2) > END_DISTANCE:
                completion_status = TaskCompletionStatus.SUCCESS

        return completion_status

def main(controller: VisionBoatController):
    controller.add_task(PIDTuner(controller))