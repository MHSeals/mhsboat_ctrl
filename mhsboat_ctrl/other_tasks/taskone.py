import numpy as np
from typing import Tuple, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

FORWARD_VELOCITY = 1  # m/s
ANGULAR_VELOCITY = 0.5  # rad/s
DRIVE_DEVIATION = 3
END_DEVIATION = 6

class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid
        self.buoy_map = self.boat_controller.buoy_map
        self._buoys = []
        self.red_pole_buoys = []
        self.green_pole_buoys = []

        self.x = 0.0
        self.y = 0.0
        self.zr = 0.0

    def search(self) -> Optional[Tuple[float, float]]:
        """
        For the sake of competition we are going to line up the boat
        with the task and start running task 1 immediately rather than
        trying to detect it.
        """

        return (0.0, 0.0) # Not really necessary for this task, so it isn't used

    def run(self) -> TaskCompletionStatus:
        """
        All previous navigation is being stripped down to traveling straight
        between the detected midpoints by using the PID algorithm.
        """

        self.boat_controller.get_logger().info("Running Task One")

        completion_status = TaskCompletionStatus.NOT_STARTED

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            and not completion_status == TaskCompletionStatus.FAILURE
        ):
            self.green_pole_buoys = [buoy for buoy in self.buoy_map if(
                isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.GREEN)]

            self.red_pole_buoys = [buoy for buoy in self.buoy_map if(
                isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.RED)]

            closest_green_pole_buoy = min(self.green_pole_buoys, key=lambda x: distance(0, 0, x.x, x.y))
            closest_red_pole_buoy = min(self.red_pole_buoys, key=lambda x: distance(0, 0, x.x, x.y))
            
            mdpt = midpoint(closest_green_pole_buoy.x, closest_green_pole_buoy.y, closest_red_pole_buoy.x, closest_red_pole_buoy.y)
            
            angle = np.arctan2(mdpt[1], mdpt[0])

            self.x += self.boat_controller.dx
            self.y += self.boat_controller.dy
            self.zr += self.boat_controller.dzr

            angular_velocity = self.boat_controller.pid.pure_pursuit(angle, (self.x, self.y), self.zr)
            angular_velocity *= ANGULAR_VELOCITY * np.pi / 180
            
            self.boat_controller.set_angular_velocity(angular_velocity)
            self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)

            # Handle case where all buoys aren't detected
            if(len(self.red_pole_buoys) < 1 or len(self.green_pole_buoys) < 1):
                if(self.last_seen == -1):
                    self.last_seen = self.boat_controller.get_clock().now().nanoseconds / 1e9
                    continue
                elif(last_seen_deviation > DRIVE_DEVIATION):
                    self.set_forward_velocity(FORWARD_VELOCITY)
                    self.set_angular_velocity(0)
                elif(last_seen_deviation > END_DEVIATION):
                    completion_status = TaskCompletionStatus.SUCCESS

        return completion_status


def main(controller: VisionBoatController):
    controller.add_task(TaskOne(controller))