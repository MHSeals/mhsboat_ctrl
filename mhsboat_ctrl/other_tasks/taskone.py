import numpy as np
from typing import Tuple, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.utils.math_utils import dist, mdpt, p2a

FORWARD_VELOCITY = 1  # m/s
ANGULAR_VELOCITY = 0.5  # rad/s
END_DEVIATION = 5

class TaskOne(Task):
    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid
        self.buoys = self.boat_controller.buoys
        self.red_pole_buoys = []
        self.green_pole_buoys = [] 

    def search(self) -> Optional[Tuple[float, float]]:
        # Don't put this in the task folder at the same time as pid_tuner
        return (0.0, 0.0)

    def run(self) -> TaskStatus:
        self.boat_controller.get_logger().info("Running Task One")

        self.boat_controller.orientation = 0

        completion_status = TaskCompletionStatus.ACTIVE

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            and not completion_status == TaskCompletionStatus.FAILURE
        ):
            self.green_pole_buoys = [buoy for buoy in self.buoys if(
                isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.GREEN)]

            self.red_pole_buoys = [buoy for buoy in self.buoys if(
                isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.RED)]

            closest_green_pole_buoy = min(self.green_pole_buoys, key=lambda buoy: buoy.size)
            closest_red_pole_buoy = min(self.red_pole_buoys, key=lambda buoy: buoy.size)
            
            mdpt = mdpt(closest_green_pole_buoy.x, closest_green_pole_buoy.y, closest_red_pole_buoy.x, closest_red_pole_buoy.y)
            
            # Will throw error if intrinsics aren't received
            error = p2a(self.boat_controller.intrinsics, (self.boat_controller.width / 2, self.boat_controller.height / 2), (mdpt))

            angular_velocity = self.boat_controller.pid.compute(error)
            angular_velocity *= ANGULAR_VELOCITY * np.pi / 180
            
            self.boat_controller.set_angular_velocity(angular_velocity)
            self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)

            # Handle case where all buoys aren't detected
            if(len(self.red_pole_buoys) < 1 or len(self.green_pole_buoys) < 1):
                if(self.last_seen == -1):
                    self.last_seen = self.boat_controller.get_clock().now().nanoseconds / 1e9
                    continue
                elif(last_seen_deviation > DRIVE_DEVIATION):
                    self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)
                    self.boat_controller.set_angular_velocity(0)
                elif(last_seen_deviation > END_DEVIATION):
                    self.boat_controller.through_gates = True
                    completion_status = TaskCompletionStatus.SUCCESS

        return completion_status

def main(controller: VisionBoatController):
    controller.add_task(TaskOne(controller))