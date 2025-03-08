import numpy as np
from typing import Tuple, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskStatus

# TODO: If we ever plan to reuse any of this code we need to put these in a configuration file, so we don't have to copy and paste it
FORWARD_VELOCITY = 1.0  # m/s
ANGULAR_VELOCITY = 2 * np.pi / 5.3  # rad/s
TRAVEL_TIME = 5 # s
ANGLE = 0.0 # rad

class PIDTuner(Task):
    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid

    def search(self) -> Optional[Tuple[float, float]]:
        return (0.0, 0.0) # Start the task instantly

    def run(self) -> TaskStatus:
        self.boat_controller.get_logger().info("Running PID Tuner")

        completion_status = TaskStatus.ACTIVE
        
        start_time = self.boat_controller.get_clock().now().nanoseconds / 1e9

        while (
            not completion_status == TaskStatus.SUCCESS
            or not completion_status == TaskStatus.FAILURE
        ):
            error = ANGLE - self.boat_controller.orientation

            angular_velocity = self.boat_controller.pid.compute(error)
            angular_velocity *= ANGULAR_VELOCITY * np.pi / 180 # NOTE: Not really sure if this is correct, but it seems to work
            
            self.boat_controller.set_angular_velocity(angular_velocity)
            self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)
        
            time = self.boat_controller.get_clock().now().nanoseconds / 1e9
            if time - start_time > TRAVEL_TIME:
                completion_status = TaskStatus.SUCCESS

        return completion_status

def main(controller: VisionBoatController):
    controller.add_task(PIDTuner(controller))